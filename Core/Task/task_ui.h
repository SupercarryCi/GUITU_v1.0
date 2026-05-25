#ifndef TASK_UI_H
#define TASK_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_state.h"

typedef struct
{
    uint8_t valid;          /* 0: 使用默认上方方向；1: heading_rad 有效 */
    float heading_rad;      /* 纸飞机方向，0 表示屏幕正上方，正值为顺时针 */
    uint32_t distance_m;    /* 距离当前返航目标或基地的距离，单位 m */
} UiReturnGuidance_t;

int32_t Task_UiInitHardware(void);
void Task_UiEntry(void *argument);
int32_t App_UiGetReturnGuidance(const AppSnapshot_t *snapshot, UiReturnGuidance_t *guidance);

#ifdef __cplusplus
}
#endif

#endif
