#include "task_return.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "task_debug.h"
#include "task_lora.h"

#include <string.h>

/*
 * 返航任务默认不运行。
 * UI 发开始命令后进入 RUNNING，再次点击 HOME 后回到 IDLE。
 */
/* 待你完善：返航开始时装载已记录路径、选择起始目标点、初始化控制参数。 */
__weak int32_t App_ReturnOnStart(ReturnState_t *state)
{
    (void)state;
    return 0;
}

/* 待你完善：后续如果需要独立返航控制输出，可以重新接入这个弱函数。 */
__weak int32_t App_ReturnStep(const AppSnapshot_t *snapshot, ReturnState_t *state)
{
    (void)snapshot;
    (void)state;
    return 0;
}

/* 待你完善：返航停止时关闭控制输出、清理临时路径/控制状态。 */
__weak void App_ReturnOnStop(ReturnState_t *state)
{
    (void)state;
}

static void task_return_send_lora_start_flag(void)
{
    const uint8_t flag = (uint8_t)'R';

    /* 只投递一次返航开始标志；队列满时允许丢弃，避免阻塞返航状态机。 */
    (void)Lora_SendBytes(&flag, 1U);
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
        if (osMessageQueueGet(g_returnCmdQueue, &command, NULL, osWaitForever) != osOK)
        {
            continue;
        }

        if (command.id == APP_CMD_RETURN_HOME_START)
        {
            if (state.mode == RETURN_MODE_RUNNING)
            {
                continue;
            }

            memset(&state, 0, sizeof(state));
            state.mode = RETURN_MODE_RUNNING;
            if (App_ReturnOnStart(&state) != 0)
            {
                memset(&state, 0, sizeof(state));
                state.mode = RETURN_MODE_IDLE;
                App_StateSetReturn(&state);
                (void)osEventFlagsClear(g_sysEventFlags, SYS_EVT_RETURN_ACTIVE);
                App_DebugLog("return home start failed");
                continue;
            }

            App_StateSetReturn(&state);
            (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_RETURN_ACTIVE);
            task_return_send_lora_start_flag();
            App_DebugLog("return home start");
        }
        else if (command.id == APP_CMD_RETURN_HOME_STOP)
        {
            if (state.mode == RETURN_MODE_RUNNING)
            {
                App_ReturnOnStop(&state);
                memset(&state, 0, sizeof(state));
                state.mode = RETURN_MODE_IDLE;
                App_StateSetReturn(&state);
                (void)osEventFlagsClear(g_sysEventFlags, SYS_EVT_RETURN_ACTIVE);
                App_DebugLog("return home stop");
            }
        }
    }
}
