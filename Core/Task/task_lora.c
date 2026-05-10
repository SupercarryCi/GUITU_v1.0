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
#include <string.h>

/*
 * LoRa 任务链路：
 * PE10(E22_DIO1) 进 EXTI 中断，中断里只置 SYS_EVT_LORA_DIO1；
 * 具体 SX126x IRQ 读取、清标志和收包都放在任务上下文中完成，避免在 ISR 中跑 SPI。
 */

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
    lora_cfg.tx_power_dbm = 0;
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

    if (e22_lora_tx_is_busy() == true)
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

    (void)argument;
    memset(&lora, 0, sizeof(lora));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
    #if (APP_LORA_ENABLE_DEFAULT != 0U)
        LoraPacketMsg_t packet;
        uint32_t flags;
        
        /*发送*/
        if (osMessageQueueGet(g_loraTxQueue, &packet, 0, 0U) == osOK)
        {
            if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
            {
                if (App_LoraTransmit(&packet) == 0)
                {
                    lora.tx_count++;
                }
                else
                {
                    lora.error_count++;
                }
                osMutexRelease(g_spiLoraMutex);
            }
        }

        /*接收*/
        flags = osEventFlagsWait(g_sysEventFlags,
                                 SYS_EVT_LORA_DIO1,
                                 osFlagsWaitAny,
                                 APP_LORA_PERIOD_MS);
        if (((flags & osFlagsError) == 0U) && ((flags & SYS_EVT_LORA_DIO1) != 0U))
        {
            int32_t rx_result;

            memset(&packet, 0, sizeof(packet));
            if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
            {
                rx_result = App_LoraPollRx(&packet);
                if (rx_result > 0)
                {
                    lora.rx_count++;
                    lora.last_rx = packet;
                    (void)osMessageQueuePut(g_loraRxQueue, &packet, 0U, 0U);
                    (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_LORA_RX);
                }
                else if (rx_result < 0)
                {
                    lora.error_count++;
                }
                osMutexRelease(g_spiLoraMutex);
            }
        }
    #else
        osDelay(APP_LORA_PERIOD_MS);
    #endif

        App_StateSetLora(&lora);
    }
}
