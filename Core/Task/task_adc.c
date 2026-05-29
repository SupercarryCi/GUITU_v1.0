/**
 * @file    task_adc.c
 * @brief   ADC 采样任务实现
 *
 * 本文件实现 ADC 周期性采样任务 (AdcTask).
 * 采用 "单次 DMA 传输 + 信号量同步" 模型:
 *   1. 任务周期启动 ADC1 DMA 采样(双通道顺序转换).
 *   2. DMA 传输完成后触发 HAL_ADC_ConvCpltCallback, 释放信号量通知任务.
 *   3. 任务获取信号量后, 将原始值换算为电压(mV), 并通过 App_StateSetAdc()
 *      更新到全局状态结构中, 同时设置系统事件标志 SYS_EVT_ADC_UPDATED.
 *   4. 若采样超时或启动失败, 则记录错误计数并同样更新状态.
 *
 * @author  cici
 * @date    2026-05-03
 */

#include "task_adc.h"
#include "task_debug.h"
#include "task_lora.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

//static uint32_t s_adcTxCnt = 0U; /*测试变量*/

static uint16_t s_adcDmaBuffer[APP_ADC_CHANNEL_COUNT] __attribute__((section(".dma_buffer"), aligned(32))); /* DMA 缓冲区 */
static volatile uint32_t s_adcDmaErrorCount = 0U;

#define ADC_GAS_PPM_MAX 99999.0f

float compute_gas_concentration(float A, float B, uint16_t V0, uint16_t VAO)
{
    float term1;
    float term2;
    float ratio;
    float ppm;

    /*
     * MQ sensor curve: Rs/R0 = ((Vc/Vout)-1) / ((Vc/V0)-1),
     * ppm = A / (Rs/R0)^B. Result is an estimated ppm value.
     */
    if ((V0 == 0U) || (VAO == 0U) || (V0 >= APP_ADC_VREF_MV))
    {
        return 0.0f;
    }
    if (VAO >= APP_ADC_VREF_MV)
    {
        return ADC_GAS_PPM_MAX;
    }

    term1 = ((float)APP_ADC_VREF_MV / (float)VAO) - 1.0f;
    term2 = ((float)APP_ADC_VREF_MV / (float)V0) - 1.0f;
    if ((term1 <= 0.0f) || (term2 <= 0.0f))
    {
        return 0.0f;
    }

    ratio = term1 / term2;
    if (ratio <= 0.0f)
    {
        return 0.0f;
    }

    ppm = A / powf(ratio, B);
    if (ppm > ADC_GAS_PPM_MAX)
    {
        return ADC_GAS_PPM_MAX;
    }
    if (ppm < 0.0f)
    {
        return 0.0f;
    }

    return ppm;
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

    (void)argument;                         /* 显式标记未使用参数, 消除编译器警告 */
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
            /* 等待 DMA 传输完成信号量, 超时时间 APP_ADC_SAMPLE_TIMEOUT_MS */
            if (osSemaphoreAcquire(g_adcReadySem, APP_ADC_SAMPLE_TIMEOUT_MS) == osOK)
            {
                uint32_t i;

                for (i = 0U; i < APP_ADC_CHANNEL_COUNT; i++)
                {
                    adc.raw[i] = s_adcDmaBuffer[i];
                    adc.voltage_mv[i] = (uint16_t)(((uint32_t)s_adcDmaBuffer[i] * APP_ADC_VREF_MV) /
                                                   APP_ADC_FULL_SCALE);
                    adc.gas_concentration[i] = compute_gas_concentration(ADC_A,
                                                                          ADC_B,
                                                                          ADC_V0,
                                                                          adc.voltage_mv[i]); /* 浓度换算公式 */
                }

            /* 测试: 每 10 个 ADC 周期通过 LoRa 发送一次气体浓度⬇️ */
//                s_adcTxCnt++;
//                if ((s_adcTxCnt % 10U) == 0U)
//                {
//                    char text[64];

//                    snprintf(text,
//                             sizeof(text),
//                             "g0=%.2f,g1=%.2f",
//                             (double)adc.gas_concentration[0],
//                             (double)adc.gas_concentration[1]);
//                    (void)Lora_SendBytes((const uint8_t *)text, (uint16_t)strlen(text));
//                }
            /* 测试: 每 10 个 ADC 周期通过 LoRa 发送一次气体浓度⬆️ */

                adc.update_count++;
                adc.error_count = s_adcDmaErrorCount;
                App_StateSetAdc(&adc);      /* 更新全局 ADC 状态 */
                (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_ADC_UPDATED);
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
