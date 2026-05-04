#include "task_lora.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "main.h"
#include "task_debug.h"

#include <string.h>

/*
 * LoRa 当前是预留框架：
 * 默认 APP_LORA_ENABLE_DEFAULT=0，不做真实收发；
 * 启用后周期为 200ms，即最高 5Hz 处理收发包。
 */
/* 待你完善：初始化 LoRa 芯片寄存器、频点、功率、带宽、扩频因子等参数。 */
__weak int32_t App_LoraHardwareInit(void)
{
    HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);
    return 0;
}

/* 待你完善：把 LoraPacketMsg_t 通过 SPI 写入 LoRa FIFO 并触发发送。 */
__weak int32_t App_LoraTransmit(const LoraPacketMsg_t *packet)
{
    /* 业务接入点：实现 SPI LoRa 发送，成功返回 0。 */
    (void)packet;
    return 0;
}

/* 待你完善：轮询或读取 LoRa IRQ，收到包后填充 LoraPacketMsg_t。 */
__weak int32_t App_LoraPollRx(LoraPacketMsg_t *packet)
{
    /* 业务接入点：收到包时填充 packet 并返回 >0。 */
    (void)packet;
    return 0;
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

void Task_LoraEntry(void *argument)
{
    LoraState_t lora;
    uint32_t next_tick;

    (void)argument;
    memset(&lora, 0, sizeof(lora));
    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    next_tick = osKernelGetTickCount();

    for (;;)
    {
#if (APP_LORA_ENABLE_DEFAULT != 0U)
            /* 待你完善：启用 LoRa 后，可按具体芯片改为 DIO1 中断驱动而非轮询。 */
            LoraPacketMsg_t packet;

            if (osMessageQueueGet(g_loraTxQueue, &packet, NULL, 0U) == osOK)
            {
                if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
                {
                    if (App_LoraTransmit(&packet) == 0)
                    {
                        lora.tx_count++;
                        osEventFlagsSet(g_sysEventFlags, SYS_EVT_LORA_TX_DONE);
                    }
                    else
                    {
                        lora.error_count++;
                    }
                    osMutexRelease(g_spiLoraMutex);
                }
            }

            memset(&packet, 0, sizeof(packet));
            if (osMutexAcquire(g_spiLoraMutex, osWaitForever) == osOK)
            {
                if (App_LoraPollRx(&packet) > 0)
                {
                    lora.rx_count++;
                    lora.last_rx = packet;
                    (void)osMessageQueuePut(g_loraRxQueue, &packet, 0U, 0U);
                    osEventFlagsSet(g_sysEventFlags, SYS_EVT_LORA_RX);
                }
                osMutexRelease(g_spiLoraMutex);
            }
#endif

        App_StateSetLora(&lora);
        next_tick += APP_LORA_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
