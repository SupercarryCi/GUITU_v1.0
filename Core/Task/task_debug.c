#include "task_debug.h"

#include "app_config.h"
#include "app_event.h"
#include "app_rtos.h"
#include "cmsis_os.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/*
 * Debug 日志链路：
 * 任意任务调用 App_DebugLog() -> g_debugLogQueue ->
 * DebugTask 串口输出，避免多个任务直接抢 UART。
 */
int32_t Task_DebugInitHardware(void)
{
    /* 待你完善：如需 debug 串口 DMA、日志等级过滤或环形缓冲，在这里补充。 */
    return 0;
}

void App_DebugLog(const char *fmt, ...)
{
    DebugLogMsg_t msg;//目前仅留32字节
    va_list args;//至臻

    if ((fmt == NULL) || (g_debugLogQueue == NULL))
    {
        return;
    }

    memset(&msg, 0, sizeof(msg));

    va_start(args, fmt);
    (void)vsnprintf(msg.text, sizeof(msg.text), fmt, args);
    va_end(args);

    /* 日志队列满时丢弃本条，避免低优先级 debug 反向阻塞实时任务。 */
    (void)osMessageQueuePut(g_debugLogQueue, &msg, 0U, 0U);
}

void Task_DebugEntry(void *argument)
{
    DebugLogMsg_t msg;
    char line[APP_DEBUG_LOG_TEXT_LEN + 32U];

    (void)argument;

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
        if (osMessageQueueGet(g_debugLogQueue, &msg, NULL, osWaitForever) == osOK)
        {
            int len;

            len = snprintf(line,
                           sizeof(line),
                           "%s\r\n",
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
