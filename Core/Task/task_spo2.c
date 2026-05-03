#include "task_spo2.h"

#include "app_config.h"
#include "app_event.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"

#include <string.h>

__weak int32_t App_Spo2HardwareInit(void)
{
    return 0;
}

__weak int32_t App_Spo2ReadSample(Spo2State_t *sample)
{
    (void)sample;
    return 0;
}

int32_t Task_Spo2InitHardware(void)
{
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

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);
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
            sample.valid = 1U;
            sample.update_count = spo2.update_count + 1U;
            sample.error_count = spo2.error_count;
            sample.last_tick_ms = osKernelGetTickCount();
            spo2 = sample;
            App_StateSetSpo2(&spo2);
            osEventFlagsSet(g_sysEventFlags, SYS_EVT_SPO2_UPDATED);
        }
        else if (result < 0)
        {
            spo2.error_count++;
            spo2.last_tick_ms = osKernelGetTickCount();
            App_StateSetSpo2(&spo2);
        }

        next_tick += APP_SPO2_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
