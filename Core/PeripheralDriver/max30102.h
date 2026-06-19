#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX30102_FIFO_DEPTH          32U
#define MAX30102_SAMPLE_RATE_HZ      100U
#define MAX30102_ALGO_BUFFER_SIZE    500U

int32_t MAX30102_Init(I2C_HandleTypeDef *hi2c);
int32_t MAX30102_ReadPartId(I2C_HandleTypeDef *hi2c, uint8_t *part_id);
int32_t MAX30102_ReadFifoSampleCount(I2C_HandleTypeDef *hi2c, uint8_t *sample_count);
int32_t MAX30102_ReadFifo(I2C_HandleTypeDef *hi2c, uint32_t *red, uint32_t *ir);

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                             int32_t n_ir_buffer_length,
                                             uint32_t *pun_red_buffer,
                                             int32_t *pn_spo2,
                                             int8_t *pch_spo2_valid,
                                             int32_t *pn_heart_rate,
                                             int8_t *pch_hr_valid);

#ifdef __cplusplus
}
#endif

#endif
