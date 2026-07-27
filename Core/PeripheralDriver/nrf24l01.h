#ifndef NRF24L01_H
#define NRF24L01_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#include <stdint.h>

/* 空口固定32字节：第0字节为业务长度，其余字节为业务数据。 */
#define NRF24L01_FRAME_SIZE       32U
#define NRF24L01_MAX_PAYLOAD_LEN  (NRF24L01_FRAME_SIZE - 1U)

typedef enum
{
    NRF24L01_OK = 0,
    NRF24L01_ERROR_ARGUMENT = -1,
    NRF24L01_ERROR_SPI = -2,
    NRF24L01_ERROR_NOT_FOUND = -3,
    NRF24L01_ERROR_MAX_RETRY = -4,
    NRF24L01_ERROR_TIMEOUT = -5,
    NRF24L01_ERROR_PAYLOAD = -6
} Nrf24l01Result_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *ce_port;
    uint16_t ce_pin;
    GPIO_TypeDef *csn_port;
    uint16_t csn_pin;
    uint32_t spi_timeout_ms;
    uint8_t channel;
    uint8_t address[5];
} Nrf24l01Config_t;

void Nrf24l01_GetDefaultConfig(Nrf24l01Config_t *config);
int32_t Nrf24l01_Init(const Nrf24l01Config_t *config);
int32_t Nrf24l01_Send(const uint8_t *data, uint8_t length, uint8_t *retry_count);
/* 返回1表示收到一包，0表示FIFO为空，负值表示驱动错误。 */
int32_t Nrf24l01_PollReceive(uint8_t *data, uint8_t capacity, uint8_t *length);
uint8_t Nrf24l01_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif
