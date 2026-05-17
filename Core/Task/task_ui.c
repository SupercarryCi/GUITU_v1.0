#include "task_ui.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "tft_port_stm32_hal.h"

#include <string.h>

/*
 * UI 链路：
 * 1. 绑定 ILI9488 + XPT2046 驱动。
 * 2. UiTask 周期读取系统快照，统一刷新 LCD。
 * 3. 触摸坐标通过 UI 命令队列交给 ControlTask，业务层再解释坐标含义。
 */

static uint8_t s_uiFirstRender = 1U;
static uint8_t s_touchPressed = 0U;

static uint16_t Ui_BarWidthU32(uint32_t value, uint32_t max_value, uint16_t max_width)
{
    if ((max_value == 0U) || (value >= max_value))
    {
        return max_width;
    }

    return (uint16_t)((value * max_width) / max_value);
}

static uint16_t Ui_BarWidthFloat(float value, float max_value, uint16_t max_width)
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if ((max_value <= 0.0f) || (value >= max_value))
    {
        return max_width;
    }

    return (uint16_t)((value * (float)max_width) / max_value);
}

int32_t App_UiHardwareInit(void)
{
    if (TFT_Port_Init() != 0)
    {
        return -1;
    }

    (void)ILI9488_Fill(&g_lcd, ILI9488_COLOR_BLACK);
    return 0;
}

void App_UiRender(const AppSnapshot_t *snapshot)
{
    uint16_t width;
    uint16_t bar_width;
    uint16_t y;

    if (snapshot == NULL)
    {
        return;
    }

    width = g_lcd.width;
    if (width < 40U)
    {
        return;
    }

    if (s_uiFirstRender != 0U)
    {
        (void)ILI9488_Fill(&g_lcd, ILI9488_COLOR_BLACK);
        s_uiFirstRender = 0U;
    }

    /* 顶部状态条：绿色表示初始化完成且无初始化错误，红色表示初始化失败。 */
    (void)ILI9488_FillRect(&g_lcd,
                           0U,
                           0U,
                           width,
                           18U,
                           (snapshot->system.init_result == 0) ? ILI9488_COLOR_GREEN : ILI9488_COLOR_RED);

    bar_width = (uint16_t)(width - 24U);
    y = 34U;

    /* ADC0 / ADC1 电压条。 */
    (void)ILI9488_FillRect(&g_lcd, 12U, y, bar_width, 14U, ILI9488_COLOR_BLUE);
    (void)ILI9488_FillRect(&g_lcd,
                           12U,
                           y,
                           Ui_BarWidthU32(snapshot->adc.voltage_mv[0], APP_ADC_VREF_MV, bar_width),
                           14U,
                           ILI9488_COLOR_CYAN);

    y = (uint16_t)(y + 24U);
    (void)ILI9488_FillRect(&g_lcd, 12U, y, bar_width, 14U, ILI9488_COLOR_BLUE);
    (void)ILI9488_FillRect(&g_lcd,
                           12U,
                           y,
                           Ui_BarWidthU32(snapshot->adc.voltage_mv[1], APP_ADC_VREF_MV, bar_width),
                           14U,
                           ILI9488_COLOR_YELLOW);

    y = (uint16_t)(y + 24U);
    (void)ILI9488_FillRect(&g_lcd, 12U, y, bar_width, 14U, ILI9488_COLOR_BLUE);
    (void)ILI9488_FillRect(&g_lcd,
                           12U,
                           y,
                           Ui_BarWidthFloat(snapshot->adc.gas_concentration[0], 1000.0f, bar_width),
                           14U,
                           ILI9488_COLOR_GREEN);

    y = (uint16_t)(y + 24U);
    (void)ILI9488_FillRect(&g_lcd, 12U, y, bar_width, 14U, ILI9488_COLOR_BLUE);
    (void)ILI9488_FillRect(&g_lcd,
                           12U,
                           y,
                           Ui_BarWidthFloat(snapshot->adc.gas_concentration[1], 1000.0f, bar_width),
                           14U,
                           ILI9488_COLOR_MAGENTA);

    y = (uint16_t)(y + 32U);
    (void)ILI9488_FillRect(&g_lcd, 12U, y, bar_width, 12U, ILI9488_COLOR_BLUE);
    (void)ILI9488_FillRect(&g_lcd,
                           12U,
                           y,
                           Ui_BarWidthU32(snapshot->spo2.spo2_percent, 100U, bar_width),
                           12U,
                           ILI9488_COLOR_RED);

    /* 底部色块：LoRa、返航、触摸状态。 */
    y = (uint16_t)(g_lcd.height - 28U);
    (void)ILI9488_FillRect(&g_lcd, 0U, y, width, 28U, ILI9488_COLOR_BLACK);
    (void)ILI9488_FillRect(&g_lcd,
                           6U,
                           (uint16_t)(y + 6U),
                           42U,
                           16U,
                           (snapshot->lora.error_count == 0U) ? ILI9488_COLOR_GREEN : ILI9488_COLOR_RED);
    (void)ILI9488_FillRect(&g_lcd,
                           58U,
                           (uint16_t)(y + 6U),
                           42U,
                           16U,
                           (snapshot->return_home.mode == RETURN_MODE_IDLE) ? ILI9488_COLOR_BLUE : ILI9488_COLOR_YELLOW);
    (void)ILI9488_FillRect(&g_lcd,
                           110U,
                           (uint16_t)(y + 6U),
                           42U,
                           16U,
                           (snapshot->ui.touch_count == 0U) ? ILI9488_COLOR_BLUE : ILI9488_COLOR_CYAN);
}

int32_t App_UiPollTouch(AppCommandMsg_t *command)
{
    XPT2046_Point point;

    if (command == NULL)
    {
        return 0;
    }

    if (XPT2046_ReadScreen(&g_touch, &point) == 0)
    {
        if (s_touchPressed != 0U)
        {
            return 0;
        }

        s_touchPressed = 1U;
        memset(command, 0, sizeof(*command));
        command->id = APP_CMD_USER_BASE;
        command->param0 = point.x;
        command->param1 = point.y;
        return 1;
    }

    s_touchPressed = 0U;
    return 0;
}

int32_t Task_UiInitHardware(void)
{
    return App_UiHardwareInit();
}

void Task_UiEntry(void *argument)
{
    UiState_t ui;
    AppSnapshot_t snapshot;
    uint32_t next_tick;

    (void)argument;
    memset(&ui, 0, sizeof(ui));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    next_tick = osKernelGetTickCount();

    for (;;)
    {
        AppCommandMsg_t command;

        memset(&snapshot, 0, sizeof(snapshot));
        App_StateGetSnapshot(&snapshot);

        if (osMutexAcquire(g_spiDisplayMutex, osWaitForever) == osOK)
        {
            App_UiRender(&snapshot);
            osMutexRelease(g_spiDisplayMutex);
            ui.render_count++;
        }

        memset(&command, 0, sizeof(command));
        if (osMutexAcquire(g_spiTouchMutex, osWaitForever) == osOK)
        {
            if (App_UiPollTouch(&command) > 0)
            {
                if (osMessageQueuePut(g_uiCmdQueue, &command, 0U, 0U) == osOK)
                {
                    ui.last_command = command;
                    ui.command_count++;
                    ui.touch_count++;
                    (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_UI_COMMAND);
                }
            }
            osMutexRelease(g_spiTouchMutex);
        }

        App_StateSetUi(&ui);
        next_tick += APP_UI_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
