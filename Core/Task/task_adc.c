/**
 * @file    task_adc.c
 * @brief   ADC 采样任务实现
 *
 * 本文件实现 ADC 周期性采样任务 (AdcTask)。
 * 采用 "单次 DMA 传输 + 信号量同步" 模型：
 *   1. 任务周期启动 ADC1 DMA 采样（双通道顺序转换）。
 *   2. DMA 传输完成后触发 HAL_ADC_ConvCpltCallback，释放信号量通知任务。
 *   3. 任务获取信号量后，将原始值换算为电压 (mV)，并通过 App_StateSetAdc()
 *      更新到全局状态结构中，同时设置系统事件标志 SYS_EVT_ADC_UPDATED。
 *   4. 若采样超时或启动失败，则记录错误计数并同样更新状态。
 *
 * @author  cici
 * @date    2026-05-03
 */

#include "task_adc.h"
#include "task_debug.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"

#include <math.h>
#include <string.h>

static uint16_t s_adcDmaBuffer[APP_ADC_CHANNEL_COUNT] __attribute__((section(".dma_buffer"), aligned(32)));//哇还有陷阱

static volatile uint32_t s_adcDmaErrorCount = 0U;

float compute_gas_concentration(float A,float B,uint16_t V0,uint16_t VAO)
{
    float term1 = (3300.0f/VAO) -1.0f;
    float term2 = (3300.0f/V0) -1.0f;
    float ratio = term1 / term2;
    return A/powf(ratio, B);
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

    (void)argument;                         /*显式标记未使用参数，消除编译器警告 */
    memset(&adc, 0, sizeof(adc));    

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    next_tick = osKernelGetTickCount(); 

    for (;;)
    {
        HAL_StatusTypeDef start_status;
        start_status = HAL_ADC_Start_DMA(&APP_ADC_HANDLE,
                                         (uint32_t *)s_adcDmaBuffer,
                                         APP_ADC_CHANNEL_COUNT);
        if (start_status == HAL_OK)
        {
            /* 等待 DMA 传输完成信号量，超时时间 APP_ADC_SAMPLE_TIMEOUT_MS */
            if (osSemaphoreAcquire(g_adcReadySem, APP_ADC_SAMPLE_TIMEOUT_MS) == osOK)
            {
                uint32_t i;

                for (i = 0U; i < APP_ADC_CHANNEL_COUNT; i++)
                {
                    adc.raw[i] = s_adcDmaBuffer[i];
                    adc.voltage_mv[i] = (uint16_t)(((uint32_t)s_adcDmaBuffer[i] * APP_ADC_VREF_MV) /
                                                   APP_ADC_FULL_SCALE);
                    adc.gas_concentration[i] = compute_gas_concentration(ADC_A, ADC_B, ADC_V0, adc.voltage_mv[i]);//浓度换算公式
                    App_DebugLog("gas[%lu]=%.2f", (unsigned long)i, (double)adc.gas_concentration[i]);

                }

                adc.update_count++; 
                adc.error_count = s_adcDmaErrorCount;
                App_StateSetAdc(&adc);      //更新全局 ADC 状态
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_ADC_UPDATED);
                HAL_ADC_Stop_DMA(&APP_ADC_HANDLE);
            }
            else
            {
                s_adcDmaErrorCount++;
                adc.error_count = s_adcDmaErrorCount;
                App_StateSetAdc(&adc);
                HAL_ADC_Stop_DMA(&APP_ADC_HANDLE);
            }
        }
        else
        {
            s_adcDmaErrorCount++;
            adc.error_count = s_adcDmaErrorCount;
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


