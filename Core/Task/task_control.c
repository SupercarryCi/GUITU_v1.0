#include "task_control.h"

#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "task_debug.h"

static void task_control_dispatch_return(const AppCommandMsg_t *command)
{
    ReturnCommandMsg_t return_command;

    return_command.id = command->id;
    return_command.tick_ms = command->tick_ms;
    (void)osMessageQueuePut(g_returnCmdQueue, &return_command, 0U, 0U);
}

void Task_ControlEntry(void *argument)
{
    AppCommandMsg_t command;

    (void)argument;

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);

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
                    App_DebugLog(APP_LOG_INFO, "ui requested lora send");
                    break;

                case APP_CMD_MARK_PATH_POINT:
                    App_DebugLog(APP_LOG_INFO, "ui marked path point");
                    break;

                default:
                    break;
            }
        }
    }
}
