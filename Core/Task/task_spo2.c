#include "task_spo2.h"

#include "app_config.h"
#include "app_event.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"

#include <string.h>

/*
 * 血氧链路：
 * Spo2Task 周期占用 I2C 总线 -> 读取传感器 -> 更新 Spo2State_t。
 */
/* 待你完善：初始化血氧传感器寄存器、采样率、LED 电流、FIFO 等参数。 */
__weak int32_t App_Spo2HardwareInit(void)
{
    return 0;
}

/* 待你完善：读取血氧传感器数据，并换算 spo2_percent/heart_rate_bpm 等字段。 */
__weak int32_t App_Spo2ReadSample(Spo2State_t *sample)
{
    /* 业务接入点：读到新血氧数据时填充 sample 并返回 >0。 */
    (void)sample;
    return 0;
}

int32_t Task_Spo2InitHardware(void)
{
    /* 初始化阶段也走 I2C 互斥锁，保持和运行态访问规则一致。 */
    if (osMutexAcquire(g_i2cBusMutex, osWaitForever) != osOK)
    {
        return -1;
    }

    if (App_Spo2HardwareInit() != 0)
    {
        osMutexRelease(g_i2cBusMutex);
        return -2;
    }

    osMutexRelease(g_i2cBusMutex);
    return 0;
}

void Task_Spo2Entry(void *argument)
{
    Spo2State_t spo2;
    uint32_t next_tick;

    (void)argument;
    memset(&spo2, 0, sizeof(spo2));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    next_tick = osKernelGetTickCount();

    for (;;)
    {
        Spo2State_t sample;
        int32_t result = 0;

        memset(&sample, 0, sizeof(sample));
        if (osMutexAcquire(g_i2cBusMutex, osWaitForever) == osOK)
        {
            result = App_Spo2ReadSample(&sample);
            osMutexRelease(g_i2cBusMutex);
        }
        else
        {
            result = -1;
        }

        if (result > 0)
        {
            sample.update_count = spo2.update_count + 1U;
            sample.error_count = spo2.error_count;
            spo2 = sample;
            App_StateSetSpo2(&spo2);
            osEventFlagsSet(g_sysEventFlags, SYS_EVT_SPO2_UPDATED);
        }
        else if (result < 0)
        {
            spo2.error_count++;
            App_StateSetSpo2(&spo2);
        }

        next_tick += APP_SPO2_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
