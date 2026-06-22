#include "app_buzzer.h"

#include "app_config.h"
#include "cmsis_os2.h"
#include "main.h"

#define APP_BUZZER_LORA_PATTERN_TICKS    8U

static osTimerId_t s_buzzerTimer = NULL;
static volatile uint8_t s_gasAlarmActive = 0U;
static volatile uint8_t s_loraPatternActive = 0U;
static volatile uint8_t s_loraPatternIndex = 0U;

static void App_BuzzerWrite(uint8_t on)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,
                      BUZZER_Pin,
                      (on != 0U) ? APP_BUZZER_ACTIVE_STATE : APP_BUZZER_INACTIVE_STATE);
}

static void App_BuzzerTimerCallback(void *argument)
{
    static const uint8_t lora_pattern[APP_BUZZER_LORA_PATTERN_TICKS] =
    {
        1U, 1U, 0U, 0U, 1U, 1U, 0U, 0U
    };
    uint8_t output_on = 0U;

    (void)argument;

    if (s_gasAlarmActive != 0U)
    {
        output_on = 1U;
    }
    else if (s_loraPatternActive != 0U)
    {
        output_on = lora_pattern[s_loraPatternIndex];
        s_loraPatternIndex++;
        if (s_loraPatternIndex >= APP_BUZZER_LORA_PATTERN_TICKS)
        {
            s_loraPatternIndex = 0U;
            s_loraPatternActive = 0U;
        }
    }

    App_BuzzerWrite(output_on);
}

int32_t App_BuzzerInit(void)
{
    s_gasAlarmActive = 0U;
    s_loraPatternActive = 0U;
    s_loraPatternIndex = 0U;
    App_BuzzerWrite(0U);

    s_buzzerTimer = osTimerNew(App_BuzzerTimerCallback, osTimerPeriodic, NULL, NULL);
    if (s_buzzerTimer == NULL)
    {
        return -1;
    }

    return 0;
}

int32_t App_BuzzerStart(void)
{
    if (s_buzzerTimer == NULL)
    {
        return -1;
    }

    App_BuzzerWrite(0U);
    if (osTimerStart(s_buzzerTimer, APP_BUZZER_TIMER_PERIOD_MS) != osOK)
    {
        return -2;
    }

    return 0;
}

void App_BuzzerRequestLoraPopup(void)
{
    if (s_gasAlarmActive != 0U)
    {
        return;
    }

    s_loraPatternIndex = 0U;
    s_loraPatternActive = 1U;
}

void App_BuzzerSetGasAlarm(uint8_t active)
{
    active = (active != 0U) ? 1U : 0U;
    if (active != 0U)
    {
        s_loraPatternActive = 0U;
        App_BuzzerWrite(1U);
    }
    else if (s_gasAlarmActive != 0U)
    {
        App_BuzzerWrite(0U);
    }

    s_gasAlarmActive = active;
}

void App_BuzzerOff(void)
{
    s_gasAlarmActive = 0U;
    s_loraPatternActive = 0U;
    s_loraPatternIndex = 0U;
    App_BuzzerWrite(0U);
}
