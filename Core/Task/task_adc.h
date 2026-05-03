#ifndef TASK_ADC_H
#define TASK_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t Task_AdcInitHardware(void);
void Task_AdcEntry(void *argument);

#ifdef __cplusplus
}
#endif

#endif
