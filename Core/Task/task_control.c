#include "task_control.h"

#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "task_debug.h"

/*
 * ControlTask 是全局控制中心。
 * UI 命令在这里分发，INS/PDR 位移增量也在这里累计成全局位置。
 */
static void task_control_dispatch_return(const AppCommandMsg_t *command)
{
    ReturnCommandMsg_t return_command;

    return_command.id = command->id;
    (void)osMessageQueuePut(g_returnCmdQueue, &return_command, 0U, 0U);
}

static void task_control_apply_nav_delta(const NavDeltaMsg_t *delta)
{
    NavState_t nav;

    if (delta == NULL)
    {
        return;
    }

    App_StateGetNav(&nav);

    /*
     * 后续返航路径优化、关键点校准、UWB/LoRa 修正，都应在这里修正 nav，
     * 不再让 INS/PDR 任务直接累计全局位置。
     */
    nav.data.position_m[0] += delta->delta_m[0];
    nav.data.position_m[1] += delta->delta_m[1];
    nav.data.position_m[2] += delta->delta_m[2];

    nav.data.velocity_mps[0] = delta->velocity_mps[0];
    nav.data.velocity_mps[1] = delta->velocity_mps[1];
    nav.data.velocity_mps[2] = delta->velocity_mps[2];

    nav.data.attitude_rad[0] = delta->attitude_rad[0];
    nav.data.attitude_rad[1] = delta->attitude_rad[1];
    nav.data.attitude_rad[2] = delta->attitude_rad[2];
    nav.data.YAW_deg = delta->yaw_deg;
    nav.update_count++;

    App_StateSetNav(&nav);
    (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_NAV_UPDATED);
}

static void task_control_dispatch_ui_command(const AppCommandMsg_t *command)
{
    if (command == NULL)
    {
        return;
    }

    switch (command->id)
    {
        case APP_CMD_RETURN_HOME_START:
        case APP_CMD_RETURN_HOME_STOP:
        case APP_CMD_RETURN_HOME_PAUSE:
        case APP_CMD_RETURN_HOME_RESUME:
            task_control_dispatch_return(command);
            break;

        case APP_CMD_LORA_SEND:
            /* 待你完善：组织 LoRa 发送包并投递到 g_loraTxQueue。 */
            App_DebugLog("ui requested lora send");
            break;

        case APP_CMD_MARK_PATH_POINT:
            /* 待你完善：把当前 NavState_t 追加到返航路径存储。 */
            App_DebugLog("ui marked path point");
            break;

        default:
            /* 待你完善：新增 UI 命令时在这里补充分发逻辑。 */
            break;
    }
}

void Task_ControlEntry(void *argument)
{
    AppCommandMsg_t command;
    NavDeltaMsg_t nav_delta;
    uint32_t flags;

    (void)argument;

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
        flags = osEventFlagsWait(g_sysEventFlags,
                                 SYS_EVT_UI_COMMAND | SYS_EVT_NAV_DELTA_READY,
                                 osFlagsWaitAny,
                                 osWaitForever);
        if ((flags & osFlagsError) != 0U)
        {
            continue;
        }

        if ((flags & SYS_EVT_UI_COMMAND) != 0U)
        {
            while (osMessageQueueGet(g_uiCmdQueue, &command, NULL, 0U) == osOK)
            {
                task_control_dispatch_ui_command(&command);
            }
        }

        if ((flags & SYS_EVT_NAV_DELTA_READY) != 0U)
        {
            while (osMessageQueueGet(g_navDeltaQueue, &nav_delta, NULL, 0U) == osOK)
            {
                task_control_apply_nav_delta(&nav_delta);
            }
        }
    }
}
