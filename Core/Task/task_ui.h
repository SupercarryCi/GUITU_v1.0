#ifndef TASK_UI_H
#define TASK_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t Task_UiInitHardware(void);
void Task_UiEntry(void *argument);

#ifdef __cplusplus
}
#endif

#endif
