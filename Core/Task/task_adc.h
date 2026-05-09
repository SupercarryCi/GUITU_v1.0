#ifndef TASK_ADC_H
#define TASK_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ADC_V0 15       // 洁净空气中获得的ao输出(mv)
#define ADC_A  1000.0f  //气体浓度换算公式的系数
#define ADC_B  0.4f     //气体浓度换算公式的指数

int32_t Task_AdcInitHardware(void);
void Task_AdcEntry(void *argument);

#ifdef __cplusplus
}
#endif

#endif
