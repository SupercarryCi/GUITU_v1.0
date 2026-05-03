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

__weak int32_t App_LoraHardwareInit(void)
{
    HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);
    return 0;
}

__weak int32_t App_LoraTransmit(const LoraPacketMsg_t *packet)
{
    (void)packet;
    return 0;
}

__weak int32_t App_LoraPollRx(LoraPacketMsg_t *packet)
{
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
    lora.enabled = APP_LORA_ENABLE_DEFAULT;

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);
    next_tick = osKernelGetTickCount();

    for (;;)
    {
        if (lora.enabled != 0U)
        {
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
                    packet.tick_ms = osKernelGetTickCount();
                    lora.rx_count++;
                    lora.last_tick_ms = packet.tick_ms;
                    lora.last_rx = packet;
                    (void)osMessageQueuePut(g_loraRxQueue, &packet, 0U, 0U);
                    osEventFlagsSet(g_sysEventFlags, SYS_EVT_LORA_RX);
                }
                osMutexRelease(g_spiLoraMutex);
            }
        }

        App_StateSetLora(&lora);
        next_tick += APP_LORA_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
