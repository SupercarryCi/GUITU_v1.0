#ifndef E22_LORA_STM32_HAL_PORT_H
#define E22_LORA_STM32_HAL_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32G471xx) || defined(STM32G473xx) || defined(STM32G474xx) || defined(STM32G483xx) || defined(STM32G484xx) || defined(STM32G491xx) || defined(STM32G4A1xx)
#include "stm32g4xx_hal.h"
#else
#include "stm32h7xx_hal.h"
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} e22_lora_stm32_gpio_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    e22_lora_stm32_gpio_t nss;
    e22_lora_stm32_gpio_t reset;
    e22_lora_stm32_gpio_t busy;
    e22_lora_stm32_gpio_t txen;
    e22_lora_stm32_gpio_t rxen;
    e22_lora_stm32_gpio_t dio1;
    uint32_t spi_timeout_ms;
    uint32_t busy_timeout_ms;
} e22_lora_stm32_hal_config_t;

bool e22_lora_stm32_hal_bind(const e22_lora_stm32_hal_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* E22_LORA_STM32_HAL_PORT_H */
