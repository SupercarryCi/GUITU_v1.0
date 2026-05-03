#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"

#define APP_GYRO_UART_HANDLE        huart1
#define APP_DEBUG_UART_HANDLE       huart2

#define APP_DISPLAY_SPI_HANDLE      hspi1
#define APP_TOUCH_SPI_HANDLE        hspi2
#define APP_LORA_SPI_HANDLE         hspi4

#define APP_SPO2_I2C_HANDLE         hi2c1
#define APP_ADC_HANDLE              hadc1

#define APP_GYRO_RX_MAX_LEN         96U
#define APP_GYRO_DMA_BUFFER_SIZE    APP_GYRO_RX_MAX_LEN
#define APP_GYRO_QUEUE_DEPTH        16U

#define APP_UI_CMD_QUEUE_DEPTH      8U
#define APP_LORA_QUEUE_DEPTH        4U
#define APP_RETURN_QUEUE_DEPTH      4U
#define APP_DEBUG_LOG_QUEUE_DEPTH   16U

#define APP_LORA_MAX_PAYLOAD_LEN    128U
#define APP_DEBUG_LOG_TEXT_LEN      96U

#define APP_ADC_CHANNEL_COUNT       2U
#define APP_ADC_SAMPLE_PERIOD_MS    100U
#define APP_ADC_SAMPLE_TIMEOUT_MS   20U
#define APP_ADC_VREF_MV             3300U
#define APP_ADC_FULL_SCALE          65535U

#define APP_UI_PERIOD_MS            50U
#define APP_SPO2_PERIOD_MS          500U
#define APP_LORA_PERIOD_MS          200U
#define APP_RETURN_PERIOD_MS        100U

/* LoRa is reserved by default. Set to 1 after the radio driver is ready. */
#define APP_LORA_ENABLE_DEFAULT     0U

#define APP_INIT_GYRO_DMA           (1UL << 0)
#define APP_INIT_UI                 (1UL << 1)
#define APP_INIT_LORA               (1UL << 2)
#define APP_INIT_SPO2               (1UL << 3)
#define APP_INIT_ADC                (1UL << 4)
#define APP_INIT_DEBUG              (1UL << 5)

#ifdef __cplusplus
}
#endif

#endif
