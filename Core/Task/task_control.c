#include "task_control.h"

#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "task_debug.h"

/*
 * ControlTask 是 UI 命令分发中心。
 * 后续如果增加菜单、模式切换、LoRa 配置，都从这里转发到对应任务。
 */
static void task_control_dispatch_return(const AppCommandMsg_t *command)
{
    ReturnCommandMsg_t return_command;

    return_command.id = command->id;
    (void)osMessageQueuePut(g_returnCmdQueue, &return_command, 0U, 0U);
}

void Task_ControlEntry(void *argument)
{
    AppCommandMsg_t command;

    (void)argument;

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
        if (osMessageQueueGet(g_uiCmdQueue, &command, NULL, osWaitForever) == osOK)
        {
            switch (command.id)
            {
                case APP_CMD_RETURN_HOME_START:
                case APP_CMD_RETURN_HOME_STOP:
                case APP_CMD_RETURN_HOME_PAUSE:
                case APP_CMD_RETURN_HOME_RESUME:
                    task_control_dispatch_return(&command);
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
    }
}
