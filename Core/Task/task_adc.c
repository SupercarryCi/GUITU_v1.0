#include "task_adc.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"

#include <string.h>

static uint16_t s_adcDmaBuffer[APP_ADC_CHANNEL_COUNT];
static volatile uint32_t s_adcDmaErrorCount = 0U;

static uint32_t task_adc_tick(void)
{
    return osKernelGetTickCount();
}

int32_t Task_AdcInitHardware(void)
{
    if ((HAL_ADC_GetState(&APP_ADC_HANDLE) & HAL_ADC_STATE_ERROR_INTERNAL) != 0U)
    {
        return -1;
    }

    return 0;
}

void Task_AdcEntry(void *argument)
{
    AdcState_t adc;
    uint32_t next_tick;

    (void)argument;
    memset(&adc, 0, sizeof(adc));

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);
    next_tick = task_adc_tick();

    for (;;)
    {
        HAL_StatusTypeDef start_status;

        start_status = HAL_ADC_Start_DMA(&APP_ADC_HANDLE,
                                         (uint32_t *)s_adcDmaBuffer,
                                         APP_ADC_CHANNEL_COUNT);
        if (start_status == HAL_OK)
        {
            if (osSemaphoreAcquire(g_adcReadySem, APP_ADC_SAMPLE_TIMEOUT_MS) == osOK)
            {
                uint32_t i;

                for (i = 0U; i < APP_ADC_CHANNEL_COUNT; i++)
                {
                    adc.raw[i] = s_adcDmaBuffer[i];
                    adc.voltage_mv[i] = (uint16_t)(((uint32_t)s_adcDmaBuffer[i] * APP_ADC_VREF_MV) /
                                                   APP_ADC_FULL_SCALE);
                }

                adc.valid = 1U;
                adc.update_count++;
                adc.error_count = s_adcDmaErrorCount;
                adc.last_tick_ms = task_adc_tick();
                App_StateSetAdc(&adc);
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_ADC_UPDATED);
                HAL_ADC_Stop_DMA(&APP_ADC_HANDLE);
            }
            else
            {
                s_adcDmaErrorCount++;
                adc.error_count = s_adcDmaErrorCount;
                adc.last_tick_ms = task_adc_tick();
                App_StateSetAdc(&adc);
                HAL_ADC_Stop_DMA(&APP_ADC_HANDLE);
            }
        }
        else
        {
            s_adcDmaErrorCount++;
            adc.error_count = s_adcDmaErrorCount;
            adc.last_tick_ms = task_adc_tick();
            App_StateSetAdc(&adc);
        }

        next_tick += APP_ADC_SAMPLE_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc == &APP_ADC_HANDLE) && (g_adcReadySem != NULL))
    {
        (void)osSemaphoreRelease(g_adcReadySem);
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &APP_ADC_HANDLE)
    {
        s_adcDmaErrorCount++;
    }
}
