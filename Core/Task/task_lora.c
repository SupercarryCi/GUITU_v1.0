#include "task_lora.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "main.h"
#include "task_debug.h"

#include "e22_lora_stm32_hal_port.h"
#include "e22_lora_tx.h"
#include "gpio.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>

/*
 * LoRa 任务链路：
 * PE10(E22_DIO1) 进 EXTI 中断，中断里只置 SYS_EVT_LORA_DIO1；
 * 具体 SX126x IRQ 读取、清标志和收包都放在任务上下文中完成，避免在 ISR 中跑 SPI。
 */

static LoraPacketMsg_t s_pending_tx_packet;
static uint8_t s_pending_tx_valid;

int32_t App_LoraHardwareInit(void)
{
    e22_lora_stm32_hal_config_t port_cfg;
    e22_lora_tx_config_t lora_cfg;

    memset(&port_cfg, 0, sizeof(port_cfg));
    port_cfg.hspi = &APP_LORA_SPI_HANDLE;
    port_cfg.nss.port = E22_NSS_GPIO_Port;
    port_cfg.nss.pin = E22_NSS_Pin;
    port_cfg.busy.port = E22_BUSY_GPIO_Port;
    port_cfg.busy.pin = E22_BUSY_Pin;
    port_cfg.txen.port = E22_TXEN_GPIO_Port;
    port_cfg.txen.pin = E22_TXEN_Pin;
    port_cfg.rxen.port = E22_RXEN_GPIO_Port;
    port_cfg.rxen.pin = E22_RXEN_Pin;
    port_cfg.dio1.port = E22_DIO1_GPIO_Port;
    port_cfg.dio1.pin = E22_DIO1_Pin;

    port_cfg.spi_timeout_ms = 1500U;
    port_cfg.busy_timeout_ms = 1500U;

    if (e22_lora_stm32_hal_bind(&port_cfg) == false)
    {
        return -1;
    }

    e22_lora_tx_get_default_config(&lora_cfg);
    lora_cfg.tx_power_dbm = 0;/* -9 —— +22 可调增益*/
    lora_cfg.rf_freq_hz = 915000000UL;

    if (e22_lora_tx_init(&lora_cfg) == false)
    {
        return -2;
    }

    if (e22_lora_rx_start() == false)
    {
        return -3;
    }

    return 0;
}

int32_t App_LoraTransmit(const LoraPacketMsg_t *packet)
{
    if ((packet == 0) || (packet->len == 0U) || (packet->len > APP_LORA_MAX_PAYLOAD_LEN))
    {
        return -1;
    }

    if ((e22_lora_tx_is_busy() == true) ||
        (e22_lora_rx_is_active() == true))
    {
        return -2;
    }

    /* 驱动返回 true 表示已经启动发送，这里转换成任务层的 0/负数返回约定。 */
    return (e22_lora_tx_send(packet->payload, (uint8_t)packet->len) == true) ? 0 : -3;
}

int32_t Lora_SendBytes(const uint8_t *data, uint16_t len)
{
    LoraPacketMsg_t packet;

    if ((data == 0) || (len == 0U) || (len > APP_LORA_MAX_PAYLOAD_LEN))
    {
        return -1;
    }

    if (g_loraTxQueue == 0)
    {
        return -2;
    }

    memset(&packet, 0, sizeof(packet));
    packet.len = len;
    memcpy(packet.payload, data, len);

    /* 发送只入队，真正的 SPI 访问由 LoraTask 统一完成。 */
    if (osMessageQueuePut(g_loraTxQueue, &packet, 0U, 0U) != osOK)
    {
        return -3;
    }

    return 0;
}

static int32_t task_lora_float_to_milli(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 1000.0f) + 0.5f);
    }

    return (int32_t)((value * 1000.0f) - 0.5f);
}

static int32_t task_lora_float_to_centi(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }

    return (int32_t)((value * 100.0f) - 0.5f);
}

static uint16_t task_lora_build_nav_packet(LoraPacketMsg_t *packet)
{
    NavState_t nav;
    int32_t x_mm;
    int32_t y_mm;
    int32_t z_mm;
    int32_t yaw_cdeg;
    int text_len;

    if (packet == 0)
    {
        return 0U;
    }

    App_StateGetNav(&nav);

    x_mm = task_lora_float_to_milli(nav.data.position_m[0]);
    y_mm = task_lora_float_to_milli(nav.data.position_m[1]);
    z_mm = task_lora_float_to_milli(nav.data.position_m[2]);
    yaw_cdeg = task_lora_float_to_centi(nav.data.YAW_deg);

    memset(packet, 0, sizeof(*packet));
    text_len = snprintf((char *)packet->payload,
                        APP_LORA_MAX_PAYLOAD_LEN,
                        "%ld,%ld,%ld,%ld",
                        (long)x_mm,
                        (long)y_mm,
                        (long)z_mm,
                        (long)yaw_cdeg);
    if ((text_len <= 0) || (text_len >= (int)APP_LORA_MAX_PAYLOAD_LEN))
    {
        return 0U;
    }

    packet->len = (uint16_t)text_len;
    return packet->len;
}

static int32_t task_lora_send_packet(LoraState_t *lora, const LoraPacketMsg_t *packet)
{
    int32_t result = -4;

    if ((lora == 0) || (packet == 0))
    {
        return -1;
    }

    if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
    {
        result = App_LoraTransmit(packet);
        if (result == 0)
        {
            lora->tx_count++;
        }
        else if (result != -2)
        {
            /* 射频忙或正在接收属于可重试状态，不计为硬错误。 */
            lora->error_count++;
        }

        osMutexRelease(g_spiLoraMutex);
    }
    else
    {
        lora->error_count++;
    }

    return result;
}

int32_t App_LoraPollRx(LoraPacketMsg_t *packet)
{
    e22_lora_rx_packet_t rx_packet;

    if (packet == 0)
    {
        return -1;
    }

    /*
     * DIO1 可能对应 RX_DONE、TX_DONE 或错误 IRQ。
     * 这里在任务上下文处理 SX126x IRQ，内部会通过 SPI 读取并清除芯片 IRQ。
     */
    e22_lora_tx_dio1_irq_handler();

    if (e22_lora_rx_read_packet(&rx_packet) == false)
    {
        return 0;
    }

    if (rx_packet.length > APP_LORA_MAX_PAYLOAD_LEN)
    {
        return -2;
    }

    memset(packet, 0, sizeof(*packet));
    packet->len = rx_packet.length;
    packet->rssi_valid = 1U;
    packet->rssi_dbm = rx_packet.rssi_dbm;
    memcpy(packet->payload, rx_packet.payload, rx_packet.length);

    return (int32_t)packet->len;
}

static void task_lora_poll_receive(LoraState_t *lora, LoraPacketMsg_t *packet)
{
    int32_t rx_result = -1;

    if ((lora == 0) || (packet == 0))
    {
        return;
    }

    memset(packet, 0, sizeof(*packet));
    if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
    {
        rx_result = App_LoraPollRx(packet);
        osMutexRelease(g_spiLoraMutex);
    }

    if (rx_result > 0)
    {
        lora->rx_count++;
        lora->last_rx = *packet;

        if (osMessageQueuePut(g_loraRxQueue, packet, 0U, 0U) == osOK)
        {
            (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_LORA_RX);
        }
        else
        {
            /* UI未及时消费时记录队列满，避免接收丢包无迹可查。 */
            lora->error_count++;
        }
    }
    else if (rx_result < 0)
    {
        lora->error_count++;
    }
}

static void task_lora_get_radio_state(LoraState_t *lora,
                                      uint8_t *tx_busy,
                                      uint8_t *rx_active)
{
    *tx_busy = 1U;
    *rx_active = 1U;

    if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
    {
        *tx_busy = (e22_lora_tx_is_busy() == true) ? 1U : 0U;
        *rx_active = (e22_lora_rx_is_active() == true) ? 1U : 0U;
        osMutexRelease(g_spiLoraMutex);
    }
    else if (lora != 0)
    {
        lora->error_count++;
    }
}

int32_t Task_LoraInitHardware(void)
{
    int32_t result;

    result = App_LoraHardwareInit();
#if (APP_LORA_ENABLE_DEFAULT == 0U)
    (void)result;
    return 0;
#else
    return result;
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  /*LORA芯片sx126x的dio1引脚中断回调*/
{
    if (GPIO_Pin == E22_DIO1_Pin)
    {
        if (g_sysEventFlags != 0)
        {
            (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_LORA_DIO1);
        }
    }
}

void Task_LoraEntry(void *argument)
{
    LoraState_t lora;
#if (APP_LORA_ENABLE_DEFAULT != 0U)
    uint32_t next_nav_tx_tick;
#endif

    (void)argument;
    memset(&lora, 0, sizeof(lora));
#if (APP_LORA_ENABLE_DEFAULT != 0U)
    next_nav_tx_tick = 0U;
    memset(&s_pending_tx_packet, 0, sizeof(s_pending_tx_packet));
    s_pending_tx_valid = 0U;
#endif

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
    #if (APP_LORA_ENABLE_DEFAULT != 0U)
        LoraPacketMsg_t packet;
        uint32_t now;
        uint32_t wait_timeout;
        int32_t tx_result;
        uint8_t tx_busy;
        uint8_t rx_active;
        uint8_t radio_started_tx = 0U;

        now = osKernelGetTickCount();
        wait_timeout = ((int32_t)(next_nav_tx_tick - now) > 0) ?
                       (next_nav_tx_tick - now) : 0U;

        /*
         * 先等待DIO1或导航周期，再统一轮询IRQ。即使EXTI漏触发，
         * 最长也会在下一个导航周期处理已完成的接收。
         */
        (void)osEventFlagsWait(g_sysEventFlags,
                               SYS_EVT_LORA_DIO1,
                               osFlagsWaitAny,
                               wait_timeout);

        /*接收*/
        task_lora_poll_receive(&lora, &packet);
        task_lora_get_radio_state(&lora, &tx_busy, &rx_active);

        if (s_pending_tx_valid == 0U)
        {
            if (osMessageQueueGet(g_loraTxQueue,
                                  &s_pending_tx_packet,
                                  0,
                                  0U) == osOK)
            {
                s_pending_tx_valid = 1U;
            }
        }

        /*发送*/
        if ((s_pending_tx_valid != 0U) &&
            (tx_busy == 0U) &&
            (rx_active == 0U))
        {
            tx_result = task_lora_send_packet(&lora, &s_pending_tx_packet);
            if (tx_result == 0)
            {
                s_pending_tx_valid = 0U;
                radio_started_tx = 1U;
                tx_busy = 1U;
            }
        }

        now = osKernelGetTickCount();
        if ((int32_t)(now - next_nav_tx_tick) >= 0)
        {
            /*
             * 接收、业务包或上一包发送占用射频时，直接跳过本周期导航包，
             * 不打断当前收包，也不让导航包挤占业务包。
             */
            if ((rx_active == 0U) &&
                (tx_busy == 0U) &&
                (radio_started_tx == 0U) &&
                (s_pending_tx_valid == 0U))
            {
                /* 只发送纯数据，坐标单位为mm，yaw单位为0.01度。 */
                if (task_lora_build_nav_packet(&packet) != 0U)
                {
                    (void)task_lora_send_packet(&lora, &packet);
                }
            }

            next_nav_tx_tick = now + APP_LORA_PERIOD_MS;
        }
    #else
        osDelay(APP_LORA_PERIOD_MS);
    #endif

        App_StateSetLora(&lora);
    }
}
