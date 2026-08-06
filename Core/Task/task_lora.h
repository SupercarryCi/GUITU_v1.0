#ifndef TASK_LORA_H
#define TASK_LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t Task_LoraInitHardware(void);
void Task_LoraEntry(void *argument);
int32_t Lora_SendBytes(const uint8_t *data, uint16_t len);

/*
 * 三端原始帧由 LoraTask 接收后调用此接口。
 * 后续业务可在其它文件提供同名强实现，默认实现只保留串口统计。
 */
void App_Nrf24BeaconFrameReceived(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
