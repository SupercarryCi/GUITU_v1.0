#include "task_control.h"

#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "dijkstra.h"
#include "task_debug.h"
#include "task_lora.h"

#include <string.h>

#define TASK_CONTROL_NAV_PERIOD_MS 500U

static LoraNavContext g_loraNavCtx;
static uint8_t s_loraNavInited = 0U;
static uint8_t s_loraNavReturnActive = 0U;

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

static void task_control_send_quick_lora(uint32_t command_id)
{
    uint8_t payload;

    if ((command_id < 1U) || (command_id > 3U))
    {
        return;
    }

    /* 快捷指令只发送单字节，降低 LoRa 发送负担。 */
    payload = (uint8_t)('0' + command_id);
    (void)Lora_SendBytes(&payload, 1U);
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
            task_control_dispatch_return(command);
            break;

        case APP_CMD_LORA_SEND:
            task_control_send_quick_lora(command->param0);
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

static void task_control_lora_nav_init_once(void)
{
    if (s_loraNavInited == 0U)
    {
        LoraNav_Init(&g_loraNavCtx, NULL);
        s_loraNavInited = 1U;
    }
}

static void task_control_store_return_guide(uint8_t return_active, const LoraNavOutput *out)
{
    ReturnGuideState_t guide;

    memset(&guide, 0, sizeof(guide));
    if (out != NULL)
    {
        guide.valid = out->valid ? 1U : 0U;
        guide.return_mode = ((return_active != 0U) || out->return_mode) ? 1U : 0U;
        guide.route_valid = out->route_valid ? 1U : 0U;
        guide.arrived_home = out->arrived_home ? 1U : 0U;
        guide.distance_to_next_mm = out->distance_to_next_mm;
        guide.bearing_to_next_cdeg = out->bearing_to_next_cdeg;
        guide.relative_bearing_cdeg = out->relative_bearing_cdeg;
        guide.next_route_index = out->next_route_index;
        guide.turn_after_next = out->turn_after_next;
    }
    else
    {
        guide.return_mode = return_active;
    }

    App_StateSetReturnGuide(&guide);
}

static void task_control_process_lora_nav(void)
{
    NavState_t nav;
    ReturnState_t return_state;
    LoraNavPoint point;
    LoraNavOutput out;
    uint8_t return_active;

    task_control_lora_nav_init_once();

    App_StateGetNav(&nav);
    App_StateGetReturn(&return_state);
    return_active = (return_state.mode == RETURN_MODE_RUNNING) ? 1U : 0U;

    if ((return_active != 0U) && (s_loraNavReturnActive == 0U))
    {
        (void)LoraNav_EnterReturnMode(&g_loraNavCtx);
    }
    else if ((return_active == 0U) && (s_loraNavReturnActive != 0U))
    {
        LoraNav_ExitReturnMode(&g_loraNavCtx);
    }
    s_loraNavReturnActive = return_active;

    point.x_m = nav.data.position_m[0];
    point.y_m = nav.data.position_m[1];
    point.z_m = nav.data.position_m[2];
    point.yaw_deg = nav.data.YAW_deg;

    memset(&out, 0, sizeof(out));
    (void)LoraNav_ProcessPoint(&g_loraNavCtx, &point, &out);
    task_control_store_return_guide(return_active, &out);
}

void Task_ControlEntry(void *argument)
{
    AppCommandMsg_t command;
    NavDeltaMsg_t nav_delta;
    uint32_t flags;
    uint32_t now_tick;
    uint32_t next_nav_tick;
    uint32_t wait_tick;

    (void)argument;

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    task_control_lora_nav_init_once();
    next_nav_tick = osKernelGetTickCount() + TASK_CONTROL_NAV_PERIOD_MS;

    for (;;)
    {
        now_tick = osKernelGetTickCount();
        wait_tick = ((int32_t)(next_nav_tick - now_tick) > 0) ? (next_nav_tick - now_tick) : 0U;
        flags = osEventFlagsWait(g_sysEventFlags,
                                 SYS_EVT_UI_COMMAND | SYS_EVT_NAV_DELTA_READY,
                                 osFlagsWaitAny,
                                 wait_tick);
        if ((flags & osFlagsError) != 0U)
        {
            if ((flags == (uint32_t)osFlagsErrorTimeout) ||
                (flags == (uint32_t)osFlagsErrorResource))
            {
                flags = 0U;
            }
            else
            {
                continue;
            }
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

        now_tick = osKernelGetTickCount();
        if ((int32_t)(now_tick - next_nav_tick) >= 0)
        {
            task_control_process_lora_nav();
            next_nav_tick += TASK_CONTROL_NAV_PERIOD_MS;
            if ((int32_t)(now_tick - next_nav_tick) >= 0)
            {
                next_nav_tick = now_tick + TASK_CONTROL_NAV_PERIOD_MS;
            }
        }
    }
}
