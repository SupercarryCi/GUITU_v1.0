#ifndef TASK_SPO2_H
#define TASK_SPO2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t Task_Spo2InitHardware(void);
void Task_Spo2Entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif
