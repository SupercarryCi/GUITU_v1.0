#include "task_ui.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

__weak int32_t App_UiHardwareInit(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RESET_t_GPIO_Port, RESET_t_Pin, GPIO_PIN_SET);
    return 0;
}

__weak void App_UiRender(const AppSnapshot_t *snapshot)
{
    (void)snapshot;
}

__weak int32_t App_UiPollTouch(AppCommandMsg_t *command)
{
    (void)command;
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

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);
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
                command.tick_ms = osKernelGetTickCount();
                if (osMessageQueuePut(g_uiCmdQueue, &command, 0U, 0U) == osOK)
                {
                    ui.last_command = command;
                    ui.command_count++;
                    ui.touch_count++;
                    ui.last_tick_ms = command.tick_ms;
                    osEventFlagsSet(g_sysEventFlags, SYS_EVT_UI_COMMAND);
                }
            }
            osMutexRelease(g_spiTouchMutex);
        }

        App_StateSetUi(&ui);
        next_tick += APP_UI_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
