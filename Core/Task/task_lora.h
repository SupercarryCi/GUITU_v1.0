#ifndef TASK_LORA_H
#define TASK_LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t Task_LoraInitHardware(void);
void Task_LoraEntry(void *argument);
int32_t Lora_SendBytes(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
