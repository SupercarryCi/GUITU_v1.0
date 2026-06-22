#ifndef APP_BUZZER_H
#define APP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t App_BuzzerInit(void);
int32_t App_BuzzerStart(void);
void App_BuzzerRequestLoraPopup(void);
void App_BuzzerSetGasAlarm(uint8_t active);
void App_BuzzerOff(void);

#ifdef __cplusplus
}
#endif

#endif
