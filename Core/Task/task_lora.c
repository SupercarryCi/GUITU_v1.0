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
#include "nrf24l01.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>

#define TASK_NRF24_PIPE_ENDPOINT1        1U
#define TASK_NRF24_PIPE_BEACON           2U
#define TASK_LORA_RX_POLL_PERIOD_MS      100U
#define TASK_RADIO_DUPLICATE_WINDOW_MS   500U
#define TASK_BEACON_DEBUG_PERIOD_MS      50U

/*
 * LoRa 任务链路：
 * PE10(E22_DIO1) 进 EXTI 中断，中断里只置 SYS_EVT_LORA_DIO1；
 * 具体 SX126x IRQ 读取、清标志和收包都放在任务上下文中完成，避免在 ISR 中跑 SPI。
 */

static LoraPacketMsg_t s_pending_tx_packet;
static uint8_t s_pending_tx_valid;
static uint8_t s_pending_nrf_attempted;
static uint8_t s_nrf_ready;
static uint8_t s_lora_ready;
static uint8_t s_last_tx_link;
static uint8_t s_last_nrf_rx_valid;
static uint32_t s_last_nrf_rx_hash;
static uint32_t s_last_nrf_rx_tick;
static uint32_t s_beacon_rx_count;

static uint32_t task_radio_payload_hash(const uint8_t *data, uint16_t len)
{
    uint32_t hash = 2166136261UL;
    uint16_t index;

    for (index = 0U; index < len; index++)
    {
        hash ^= data[index];
        hash *= 16777619UL;
    }
    hash ^= len;
    return hash;
}

static void task_radio_record_nrf_payload(const LoraPacketMsg_t *packet)
{
    s_last_nrf_rx_hash = task_radio_payload_hash(packet->payload, packet->len);
    s_last_nrf_rx_tick = osKernelGetTickCount();
    s_last_nrf_rx_valid = 1U;
}

static uint8_t task_radio_is_recent_nrf_duplicate(const LoraPacketMsg_t *packet)
{
    uint32_t now;

    if ((packet == 0) || (s_last_nrf_rx_valid == 0U))
    {
        return 0U;
    }

    now = osKernelGetTickCount();
    if ((uint32_t)(now - s_last_nrf_rx_tick) >
        TASK_RADIO_DUPLICATE_WINDOW_MS)
    {
        return 0U;
    }

    return (task_radio_payload_hash(packet->payload, packet->len) ==
            s_last_nrf_rx_hash) ? 1U : 0U;
}

static void task_radio_report_tx_link(uint8_t link)
{
    if (s_last_tx_link != link)
    {
        s_last_tx_link = link;
        App_DebugLog((link == 1U) ? "N,L,N" : "N,L,L");
    }
}

__weak void App_Nrf24BeaconFrameReceived(const uint8_t *data, uint8_t len)
{
    static uint32_t last_debug_tick;
    BeaconHeadingState_t heading;
    uint32_t now;
    uint16_t word0;
    uint16_t word1;
    uint16_t word2;
    uint16_t word3;
    uint8_t checksum = 0U;
    uint8_t index;

    if ((data == 0) || (len != 11U) || (data[0] != 0x55U))
    {
        return;
    }

    for (index = 0U; index < 10U; index++)
    {
        checksum = (uint8_t)(checksum + data[index]);
    }
    if (checksum != data[10])
    {
        return;
    }

    /* 三端只提供航向基准，其他WIT帧不参与二端导航。 */
    if (data[1] != 0x53U)
    {
        return;
    }

    word0 = (uint16_t)data[2] | ((uint16_t)data[3] << 8U);
    word1 = (uint16_t)data[4] | ((uint16_t)data[5] << 8U);
    word2 = (uint16_t)data[6] | ((uint16_t)data[7] << 8U);
    word3 = (uint16_t)data[8] | ((uint16_t)data[9] << 8U);

    now = osKernelGetTickCount();
    heading.yaw_deg = ((float)(int16_t)word2 * 180.0f) / 32768.0f;
    heading.last_update_tick = now;
    heading.valid = 1U;
    App_StateSetBeaconHeading(&heading);

    /* 限制文本输出速率，避免高频WIT帧占满调试串口和日志队列。 */
    if ((last_debug_tick != 0U) &&
        ((uint32_t)(now - last_debug_tick) < TASK_BEACON_DEBUG_PERIOD_MS))
    {
        return;
    }
    last_debug_tick = now;

    App_DebugLog("B,%02X,%04X,%04X,%04X,%04X",
                 (unsigned int)data[1],
                 (unsigned int)word0,
                 (unsigned int)word1,
                 (unsigned int)word2,
                 (unsigned int)word3);
}

static int32_t task_nrf24_init(void)
{
#if (APP_NRF24_ENABLE_DEFAULT != 0U)
    Nrf24l01Config_t config;
    int32_t result;

    Nrf24l01_GetDefaultConfig(&config);
    config.hspi = &APP_NRF24_SPI_HANDLE;
    config.ce_port = NRF24_CE_GPIO_Port;
    config.ce_pin = NRF24_CE_Pin;
    config.csn_port = NRF24_CSN_GPIO_Port;
    config.csn_pin = NRF24_CSN_Pin;
    config.rx_pipe_mask = NRF24L01_RX_PIPE_1 | NRF24L01_RX_PIPE_2;
    config.pipe2_lsb = 0xA3U;

    result = Nrf24l01_Init(&config);
    s_nrf_ready = (result == NRF24L01_OK) ? 1U : 0U;
    App_DebugLog("N,I,%ld", (long)result);
    return result;
#else
    s_nrf_ready = 0U;
    return 0;
#endif
}

int32_t App_LoraHardwareInit(void)
{
#if (APP_LORA_RF_ENABLE_DEFAULT != 0U)
    e22_lora_stm32_hal_config_t port_cfg;
    e22_lora_tx_config_t lora_cfg;
#endif

    /* NRF24初始化失败时保留LoRa后备链路，不阻塞系统启动。 */
    (void)task_nrf24_init();
    s_lora_ready = 0U;

#if (APP_LORA_RF_ENABLE_DEFAULT == 0U)
    /* 测试期间完全不初始化SX126x，避免共享SPI4影响NRF24收包。 */
    return (s_nrf_ready != 0U) ? 0 : -1;
#else
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
        return (s_nrf_ready != 0U) ? 0 : -1;
    }

    e22_lora_tx_get_default_config(&lora_cfg);
    lora_cfg.tx_power_dbm = 0;/* -9 —— +22 可调增益*/
    lora_cfg.rf_freq_hz = 915000000UL;

    if (e22_lora_tx_init(&lora_cfg) == false)
    {
        return (s_nrf_ready != 0U) ? 0 : -2;
    }

    if (e22_lora_rx_start() == false)
    {
        return (s_nrf_ready != 0U) ? 0 : -3;
    }

    s_lora_ready = 1U;
    return 0;
#endif
}

int32_t App_LoraTransmit(const LoraPacketMsg_t *packet)
{
    if ((s_lora_ready == 0U) ||
        (packet == 0) ||
        (packet->len == 0U) ||
        (packet->len > APP_LORA_MAX_PAYLOAD_LEN))
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

static int32_t task_nrf24_send_packet(const LoraPacketMsg_t *packet,
                                      uint8_t *retry_count)
{
#if (APP_NRF24_ENABLE_DEFAULT != 0U)
    int32_t result = NRF24L01_ERROR_ARGUMENT;

    if ((packet == 0) ||
        (packet->len == 0U) ||
        (packet->len > NRF24L01_MAX_PAYLOAD_LEN) ||
        (s_nrf_ready == 0U))
    {
        return NRF24L01_ERROR_ARGUMENT;
    }

    if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
    {
        result = Nrf24l01_Send(packet->payload,
                              (uint8_t)packet->len,
                              retry_count);
        osMutexRelease(g_spiLoraMutex);
    }
    else
    {
        result = NRF24L01_ERROR_SPI;
    }

    return result;
#else
    (void)packet;
    (void)retry_count;
    return NRF24L01_ERROR_ARGUMENT;
#endif
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

    if ((packet == 0) || (s_lora_ready == 0U))
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
    packet->port = 1U;
    packet->rssi_valid = 1U;
    packet->rssi_dbm = rx_packet.rssi_dbm;
    memcpy(packet->payload, rx_packet.payload, rx_packet.length);

    return (int32_t)packet->len;
}

static void task_lora_poll_receive(LoraState_t *lora, LoraPacketMsg_t *packet)
{
    int32_t rx_result = -1;

    if ((lora == 0) || (packet == 0) || (s_lora_ready == 0U))
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
        if (task_radio_is_recent_nrf_duplicate(packet) != 0U)
        {
            App_DebugLog("N,D,L");
            return;
        }

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

static void task_nrf24_poll_receive(LoraState_t *lora, LoraPacketMsg_t *packet)
{
#if (APP_NRF24_ENABLE_DEFAULT != 0U)
    uint8_t length;
    uint8_t pipe_number;
    int32_t result = 0;

    if ((lora == 0) || (packet == 0) || (s_nrf_ready == 0U))
    {
        return;
    }

    for (;;)
    {
        memset(packet, 0, sizeof(*packet));
        length = 0U;
        pipe_number = 0x07U;
        if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
        {
            result = Nrf24l01_PollReceive(packet->payload,
                                         NRF24L01_MAX_PAYLOAD_LEN,
                                         &length,
                                         &pipe_number);
            osMutexRelease(g_spiLoraMutex);
        }
        else
        {
            result = NRF24L01_ERROR_SPI;
        }

        if (result <= 0)
        {
            break;
        }

        packet->len = length;
        packet->port = (pipe_number == TASK_NRF24_PIPE_BEACON) ? 3U : 1U;
        packet->rssi_valid = 0U; /* NRF24没有可直接读取的RSSI数值。 */
        lora->rx_count++;
        lora->last_rx = *packet;

        if (pipe_number == TASK_NRF24_PIPE_ENDPOINT1)
        {
            task_radio_record_nrf_payload(packet);
            App_DebugLog("N,R,1,%u", (unsigned int)packet->len);
            if (osMessageQueuePut(g_loraRxQueue, packet, 0U, 0U) == osOK)
            {
                (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_LORA_RX);
            }
            else
            {
                lora->error_count++;
            }
        }
        else if (pipe_number == TASK_NRF24_PIPE_BEACON)
        {
            s_beacon_rx_count++;
            App_Nrf24BeaconFrameReceived(packet->payload, (uint8_t)packet->len);
            if ((s_beacon_rx_count & 0x1FU) == 0U)
            {
                App_DebugLog("N,R,3,%lu,%02X",
                             (unsigned long)s_beacon_rx_count,
                             (unsigned int)packet->payload[1]);
            }
        }
    }

    if (result < 0)
    {
        lora->error_count++;
        App_DebugLog("N,E,R,%ld", (long)result);
    }
#else
    (void)lora;
    (void)packet;
#endif
}

static void task_lora_get_radio_state(LoraState_t *lora,
                                      uint8_t *tx_busy,
                                      uint8_t *rx_active)
{
    *tx_busy = 1U;
    *rx_active = 1U;

    if (s_lora_ready == 0U)
    {
        *tx_busy = 0U;
        *rx_active = 0U;
        return;
    }

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
    uint32_t next_lora_poll_tick;
    uint32_t next_nrf_retry_tick;
#endif

    (void)argument;
    memset(&lora, 0, sizeof(lora));
#if (APP_LORA_ENABLE_DEFAULT != 0U)
    next_nav_tx_tick = 0U;
    next_lora_poll_tick = 0U;
    next_nrf_retry_tick = 0U;
    memset(&s_pending_tx_packet, 0, sizeof(s_pending_tx_packet));
    s_pending_tx_valid = 0U;
    s_pending_nrf_attempted = 0U;
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
        uint8_t business_packet_sent = 0U;
        uint8_t retry_count = 0U;
        uint8_t nav_due;
        uint32_t wait_flags;

        now = osKernelGetTickCount();
        wait_timeout = ((int32_t)(next_nav_tx_tick - now) > 0) ?
                       (next_nav_tx_tick - now) : 0U;
#if (APP_NRF24_ENABLE_DEFAULT != 0U)
        if ((s_nrf_ready != 0U) &&
            (wait_timeout > APP_NRF24_POLL_PERIOD_MS))
        {
            wait_timeout = APP_NRF24_POLL_PERIOD_MS;
        }
#endif

        /*
         * 先等待DIO1或导航周期，再统一轮询IRQ。即使EXTI漏触发，
         * 启用NRF24时最长会在下一个NRF24轮询周期处理接收。
         */
        wait_flags = osEventFlagsWait(g_sysEventFlags,
                                      SYS_EVT_LORA_DIO1,
                                      osFlagsWaitAny,
                                      wait_timeout);
        if ((wait_flags & osFlagsError) != 0U)
        {
            wait_flags = 0U;
        }

        /*接收*/
        now = osKernelGetTickCount();
        if (((wait_flags & SYS_EVT_LORA_DIO1) != 0U) ||
            ((int32_t)(now - next_lora_poll_tick) >= 0))
        {
            task_lora_poll_receive(&lora, &packet);
            next_lora_poll_tick = now + TASK_LORA_RX_POLL_PERIOD_MS;
        }
        task_nrf24_poll_receive(&lora, &packet);

        if (s_pending_tx_valid == 0U)
        {
            if (osMessageQueueGet(g_loraTxQueue,
                                  &s_pending_tx_packet,
                                  0,
                                  0U) == osOK)
            {
                s_pending_tx_valid = 1U;
                s_pending_nrf_attempted = 0U;
                next_nrf_retry_tick = 0U;
            }
        }

        now = osKernelGetTickCount();
        nav_due = ((int32_t)(now - next_nav_tx_tick) >= 0) ? 1U : 0U;
        if ((s_pending_tx_valid != 0U) || (nav_due != 0U))
        {
            task_lora_get_radio_state(&lora, &tx_busy, &rx_active);
        }
        else
        {
            tx_busy = 0U;
            rx_active = 0U;
        }

        /*
         * 每个业务包先通过NRF24发送；收到自动应答即认为链路可用，
         * 不再占用LoRa。NRF24未应答或包长超限时，再走LoRa后备链路。
         */
        if ((s_pending_tx_valid != 0U) &&
            (s_pending_nrf_attempted == 0U) &&
            ((int32_t)(now - next_nrf_retry_tick) >= 0) &&
            (tx_busy == 0U) &&
            (rx_active == 0U))
        {
            tx_result = task_nrf24_send_packet(&s_pending_tx_packet,
                                               &retry_count);
            if (tx_result == NRF24L01_OK)
            {
                s_pending_tx_valid = 0U;
                business_packet_sent = 1U;
                lora.tx_count++;
                task_radio_report_tx_link(1U);
            }
            else
            {
                s_pending_nrf_attempted = 1U;
                next_nrf_retry_tick = now + 250U;
                App_DebugLog("N,F,%ld,%u",
                             (long)tx_result,
                             (unsigned int)retry_count);
            }
        }

        /* NRF24发送失败后才尝试LoRa。 */
        if ((s_pending_tx_valid != 0U) &&
            (s_pending_nrf_attempted != 0U) &&
            (s_lora_ready != 0U) &&
            (tx_busy == 0U) &&
            (rx_active == 0U))
        {
            tx_result = task_lora_send_packet(&lora, &s_pending_tx_packet);
            if (tx_result == 0)
            {
                s_pending_tx_valid = 0U;
                s_pending_nrf_attempted = 0U;
                business_packet_sent = 1U;
                tx_busy = 1U;
                task_radio_report_tx_link(2U);
            }
        }

        now = osKernelGetTickCount();
        if ((s_pending_tx_valid != 0U) &&
            (s_pending_nrf_attempted != 0U) &&
            (s_lora_ready == 0U) &&
            ((int32_t)(now - next_nrf_retry_tick) >= 0))
        {
            /* LoRa不可用时低频重试NRF，避免队首业务包永久阻塞。 */
            s_pending_nrf_attempted = 0U;
        }

        now = osKernelGetTickCount();
        if (nav_due != 0U)
        {
            /*
             * 接收、业务包或上一包发送占用射频时，直接跳过本周期导航包，
             * 不打断当前收包，也不让导航包挤占业务包。
             */
            if ((business_packet_sent == 0U) &&
                (s_pending_tx_valid == 0U) &&
                (tx_busy == 0U) &&
                (rx_active == 0U))
            {
                /* 只发送纯数据，坐标单位为mm，yaw单位为0.01度。 */
                if (task_lora_build_nav_packet(&packet) != 0U)
                {
                    retry_count = 0U;
                    tx_result = task_nrf24_send_packet(&packet,
                                                       &retry_count);
                    if (tx_result == NRF24L01_OK)
                    {
                        lora.tx_count++;
                        task_radio_report_tx_link(1U);
                    }
                    if ((tx_result != NRF24L01_OK) &&
                        (s_lora_ready != 0U) &&
                        (rx_active == 0U) &&
                        (tx_busy == 0U))
                    {
                        App_DebugLog("N,F,%ld,%u",
                                     (long)tx_result,
                                     (unsigned int)retry_count);
                        if (task_lora_send_packet(&lora, &packet) == 0)
                        {
                            task_radio_report_tx_link(2U);
                        }
                    }
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
