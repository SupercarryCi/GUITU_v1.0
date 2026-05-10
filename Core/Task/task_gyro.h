#ifndef TASK_GYRO_H
#define TASK_GYRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t Task_GyroInitHardware(void);
int32_t Task_UWBInitHardware(void);
int32_t Task_GyroStartRx(void);
void Task_GyroEntry(void *argument);

#ifdef __cplusplus
}
#endif

#endif
