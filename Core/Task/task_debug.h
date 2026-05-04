#ifndef TASK_DEBUG_H
#define TASK_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_msg.h"

int32_t Task_DebugInitHardware(void);
void Task_DebugEntry(void *argument);
void App_DebugLog(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
