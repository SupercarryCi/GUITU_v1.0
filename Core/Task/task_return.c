#include "task_return.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "task_debug.h"

#include <string.h>

/*
 * 返航任务默认不运行。
 * UI 发开始命令后进入 RUNNING，周期读取整机快照并调用 App_ReturnStep()。
 */
/* 待你完善：返航开始时装载已记录路径、选择起始目标点、初始化控制参数。 */
__weak int32_t App_ReturnOnStart(ReturnState_t *state)
{
    (void)state;
    return 0;
}

/* 待你完善：根据导航状态和路径点计算返航控制输出。 */
__weak int32_t App_ReturnStep(const AppSnapshot_t *snapshot, ReturnState_t *state)
{
    /* 业务接入点：返回 0=继续，>0=完成，<0=故障。 */
    (void)snapshot;
    (void)state;
    return 0;
}

/* 待你完善：返航停止时关闭控制输出、清理临时路径/控制状态。 */
__weak void App_ReturnOnStop(ReturnState_t *state)
{
    (void)state;
}

static void task_return_set_mode(ReturnState_t *state, ReturnMode_t mode)
{
    state->mode = mode;
    App_StateSetReturn(state);
}

void Task_ReturnEntry(void *argument)
{
    ReturnState_t state;
    ReturnCommandMsg_t command;

    (void)argument;
    memset(&state, 0, sizeof(state));
    state.mode = RETURN_MODE_IDLE;

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
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
                App_DebugLog("return home start");
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
                    App_DebugLog("return home stop");
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

            if (step_result > 0)
            {
                state.mode = RETURN_MODE_DONE;
                App_StateSetReturn(&state);
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_RETURN_DONE);
                App_DebugLog("return home done");
            }
            else if (step_result < 0)
            {
                state.mode = RETURN_MODE_FAULT;
                state.error_count++;
                App_StateSetReturn(&state);
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_RETURN_FAULT);
                App_DebugLog("return home fault");
            }
            else
            {
                App_StateSetReturn(&state);
            }
        }
    }
}
