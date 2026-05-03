#include "task_return.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "task_debug.h"

#include <string.h>

__weak int32_t App_ReturnOnStart(ReturnState_t *state)
{
    (void)state;
    return 0;
}

__weak int32_t App_ReturnStep(const AppSnapshot_t *snapshot, ReturnState_t *state)
{
    (void)snapshot;
    (void)state;
    return 0;
}

__weak void App_ReturnOnStop(ReturnState_t *state)
{
    (void)state;
}

static void task_return_set_mode(ReturnState_t *state, ReturnMode_t mode)
{
    state->mode = mode;
    state->last_tick_ms = osKernelGetTickCount();
    App_StateSetReturn(state);
}

void Task_ReturnEntry(void *argument)
{
    ReturnState_t state;
    ReturnCommandMsg_t command;

    (void)argument;
    memset(&state, 0, sizeof(state));
    state.mode = RETURN_MODE_IDLE;

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);
    App_StateSetReturn(&state);

    for (;;)
    {
        if (state.mode == RETURN_MODE_IDLE ||
            state.mode == RETURN_MODE_DONE ||
            state.mode == RETURN_MODE_FAULT)
        {
            if (osMessageQueueGet(g_returnCmdQueue, &command, NULL, osWaitForever) != osOK)
            {
                continue;
            }

            if (command.id == APP_CMD_RETURN_HOME_START)
            {
                memset(&state, 0, sizeof(state));
                state.mode = RETURN_MODE_RUNNING;
                state.last_tick_ms = osKernelGetTickCount();
                if (App_ReturnOnStart(&state) != 0)
                {
                    state.mode = RETURN_MODE_FAULT;
                    state.error_count++;
                    App_StateSetReturn(&state);
                    osEventFlagsSet(g_sysEventFlags, SYS_EVT_RETURN_FAULT);
                    continue;
                }

                App_StateSetReturn(&state);
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_RETURN_ACTIVE);
                App_DebugLog(APP_LOG_INFO, "return home start");
            }
        }
        else
        {
            AppSnapshot_t snapshot;
            int32_t step_result = 0;

            if (osMessageQueueGet(g_returnCmdQueue, &command, NULL, APP_RETURN_PERIOD_MS) == osOK)
            {
                if (command.id == APP_CMD_RETURN_HOME_STOP)
                {
                    App_ReturnOnStop(&state);
                    task_return_set_mode(&state, RETURN_MODE_IDLE);
                    App_DebugLog(APP_LOG_INFO, "return home stop");
                    continue;
                }
                if (command.id == APP_CMD_RETURN_HOME_PAUSE)
                {
                    task_return_set_mode(&state, RETURN_MODE_PAUSED);
                    continue;
                }
                if (command.id == APP_CMD_RETURN_HOME_RESUME)
                {
                    task_return_set_mode(&state, RETURN_MODE_RUNNING);
                    continue;
                }
            }

            if (state.mode != RETURN_MODE_RUNNING)
            {
                continue;
            }

            memset(&snapshot, 0, sizeof(snapshot));
            App_StateGetSnapshot(&snapshot);
            step_result = App_ReturnStep(&snapshot, &state);
            state.step_count++;
            state.last_tick_ms = osKernelGetTickCount();

            if (step_result > 0)
            {
                state.mode = RETURN_MODE_DONE;
                App_StateSetReturn(&state);
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_RETURN_DONE);
                App_DebugLog(APP_LOG_INFO, "return home done");
            }
            else if (step_result < 0)
            {
                state.mode = RETURN_MODE_FAULT;
                state.error_count++;
                App_StateSetReturn(&state);
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_RETURN_FAULT);
                App_DebugLog(APP_LOG_ERROR, "return home fault");
            }
            else
            {
                App_StateSetReturn(&state);
            }
        }
    }
}
