#include "task_ui.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>

/*
 * UI 链路：
 * 周期读取 AppSnapshot_t 刷屏，同时轮询触摸。
 * 触摸产生 AppCommandMsg_t，由 ControlTask 统一分发。
 */
/* 待你完善：初始化屏幕控制器、背光、触摸控制器等真实 UI 硬件。 */
__weak int32_t App_UiHardwareInit(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RESET_t_GPIO_Port, RESET_t_Pin, GPIO_PIN_SET);
    return 0;
}

/* 待你完善：根据 snapshot 绘制导航、血氧、ADC、LoRa、返航状态等界面。 */
__weak void App_UiRender(const AppSnapshot_t *snapshot)
{
    /* 业务接入点：在这里使用 snapshot 更新屏幕显示内容。 */
    (void)snapshot;
}

/* 待你完善：扫描触摸屏，把用户操作转换为 AppCommandMsg_t。 */
__weak int32_t App_UiPollTouch(AppCommandMsg_t *command)
{
    /* 业务接入点：检测到有效触摸命令时填充 command 并返回 >0。 */
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

        /* 显示 SPI 和触摸 SPI 分开加锁，避免两个设备操作互相打断。 */
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
