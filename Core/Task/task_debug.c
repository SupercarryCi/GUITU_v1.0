#include "task_debug.h"

#include "app_config.h"
#include "app_event.h"
#include "app_rtos.h"
#include "cmsis_os.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *task_debug_level_text(AppLogLevel_t level)
{
    switch (level)
    {
        case APP_LOG_DEBUG:
            return "D";
        case APP_LOG_INFO:
            return "I";
        case APP_LOG_WARN:
            return "W";
        case APP_LOG_ERROR:
            return "E";
        default:
            return "?";
    }
}

int32_t Task_DebugInitHardware(void)
{
    return 0;
}

void App_DebugLog(AppLogLevel_t level, const char *fmt, ...)
{
    DebugLogMsg_t msg;
    va_list args;

    if ((fmt == NULL) || (g_debugLogQueue == NULL))
    {
        return;
    }

    memset(&msg, 0, sizeof(msg));
    msg.level = level;
    if (osKernelGetState() != osKernelInactive)
    {
        msg.tick_ms = osKernelGetTickCount();
    }

    va_start(args, fmt);
    (void)vsnprintf(msg.text, sizeof(msg.text), fmt, args);
    va_end(args);

    (void)osMessageQueuePut(g_debugLogQueue, &msg, 0U, 0U);
}

void Task_DebugEntry(void *argument)
{
    DebugLogMsg_t msg;
    char line[APP_DEBUG_LOG_TEXT_LEN + 32U];

    (void)argument;

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);

    for (;;)
    {
        if (osMessageQueueGet(g_debugLogQueue, &msg, NULL, osWaitForever) == osOK)
        {
            int len;

            len = snprintf(line,
                           sizeof(line),
                           "[%lu][%s] %s\r\n",
                           (unsigned long)msg.tick_ms,
                           task_debug_level_text(msg.level),
                           msg.text);
            if (len <= 0)
            {
                continue;
            }
            if ((uint32_t)len > sizeof(line))
            {
                len = (int)sizeof(line);
            }

            if (osMutexAcquire(g_debugUartMutex, osWaitForever) == osOK)
            {
                (void)HAL_UART_Transmit(&APP_DEBUG_UART_HANDLE,
                                        (uint8_t *)line,
                                        (uint16_t)len,
                                        100U);
                osMutexRelease(g_debugUartMutex);
            }
        }
    }
}
