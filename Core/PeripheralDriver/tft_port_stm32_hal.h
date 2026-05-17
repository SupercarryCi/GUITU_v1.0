#ifndef TFT_PORT_STM32_HAL_H
#define TFT_PORT_STM32_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ili9488.h"
#include "xpt2046.h"

extern ILI9488_Handle g_lcd;
extern XPT2046_Handle g_touch;

int TFT_Port_Init(void);
void TFT_Port_Backlight(uint8_t on);

#ifdef __cplusplus
}
#endif

#endif
