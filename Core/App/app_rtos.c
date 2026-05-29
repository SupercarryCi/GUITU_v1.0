#include "app_rtos.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_state.h"
#include "main.h"
#include "task_adc.h"
#include "task_control.h"
#include "task_debug.h"
#include "task_gyro.h"
#include "task_lora.h"
#include "task_return.h"
#include "task_spo2.h"
#include "task_ui.h"
#include "task_ins_pdr.h"

/*
 * RTOS 对象在这里集中创建：
 * 任务文件只使用 extern 句柄，不负责创建队列/互斥锁/事件组。
 */
osEventFlagsId_t g_sysEventFlags = NULL;
osMessageQueueId_t g_gyroRxQueue = NULL;
osMessageQueueId_t g_uiCmdQueue = NULL;
osMessageQueueId_t g_navDeltaQueue = NULL;
osMessageQueueId_t g_loraTxQueue = NULL;
osMessageQueueId_t g_loraRxQueue = NULL;
osMessageQueueId_t g_returnCmdQueue = NULL;
osMessageQueueId_t g_debugLogQueue = NULL;
osSemaphoreId_t g_adcReadySem = NULL;
osMutexId_t g_spiDisplayMutex = NULL;
osMutexId_t g_spiTouchMutex = NULL;
osMutexId_t g_spiLoraMutex = NULL;
osMutexId_t g_i2cBusMutex = NULL;
osMutexId_t g_debugUartMutex = NULL;

static osThreadId_t g_initTaskHandle = NULL;
static osThreadId_t g_gyroTaskHandle = NULL;
static osThreadId_t g_uiTaskHandle = NULL;
static osThreadId_t g_controlTaskHandle = NULL;
static osThreadId_t g_loraTaskHandle = NULL;
static osThreadId_t g_spo2TaskHandle = NULL;
static osThreadId_t g_adcTaskHandle = NULL;
static osThreadId_t g_debugTaskHandle = NULL;
static osThreadId_t g_returnTaskHandle = NULL;
static osThreadId_t g_ins_pdr_TaskHandle = NULL;

static void InitTask(void *argument);
static int32_t App_InitPeripherals(uint32_t *done_mask);
static int32_t App_StartRuntimeTasks(void);

static void App_Error(uint32_t code)
{
    App_StateAddFault(code);

    if (g_sysEventFlags != NULL)
    {
        osEventFlagsSet(g_sysEventFlags, SYS_EVT_INIT_FAILED);
    }

    __disable_irq();
    while (1)
    {
    }
}

void App_RtosCreateObjects(void)
{
    const osThreadAttr_t initTaskAttr = {
        .name = "InitTask",
        .priority = osPriorityHigh,
        .stack_size = 1024U
    };

    g_sysEventFlags = osEventFlagsNew(NULL);
    g_gyroRxQueue = osMessageQueueNew(APP_GYRO_QUEUE_DEPTH, sizeof(GyroRxMsg_t), NULL);
    g_uiCmdQueue = osMessageQueueNew(APP_UI_CMD_QUEUE_DEPTH, sizeof(AppCommandMsg_t), NULL);
    g_navDeltaQueue = osMessageQueueNew(APP_NAV_DELTA_QUEUE_DEPTH, sizeof(NavDeltaMsg_t), NULL);
    g_loraTxQueue = osMessageQueueNew(APP_LORA_QUEUE_DEPTH, sizeof(LoraPacketMsg_t), NULL);
    g_loraRxQueue = osMessageQueueNew(APP_LORA_QUEUE_DEPTH, sizeof(LoraPacketMsg_t), NULL);
    g_returnCmdQueue = osMessageQueueNew(APP_RETURN_QUEUE_DEPTH, sizeof(ReturnCommandMsg_t), NULL);
    g_debugLogQueue = osMessageQueueNew(APP_DEBUG_LOG_QUEUE_DEPTH, sizeof(DebugLogMsg_t), NULL);
    g_adcReadySem = osSemaphoreNew(1U, 0U, NULL);
    g_spiDisplayMutex = osMutexNew(NULL);
    g_spiTouchMutex = osMutexNew(NULL);
    g_spiLoraMutex = osMutexNew(NULL);
    g_i2cBusMutex = osMutexNew(NULL);
    g_debugUartMutex = osMutexNew(NULL);

    if ((g_sysEventFlags == NULL) ||
        (g_gyroRxQueue == NULL) ||
        (g_uiCmdQueue == NULL) ||
        (g_navDeltaQueue == NULL) ||
        (g_loraTxQueue == NULL) ||
        (g_loraRxQueue == NULL) ||
        (g_returnCmdQueue == NULL) ||
        (g_debugLogQueue == NULL) ||
        (g_adcReadySem == NULL) ||
        (g_spiDisplayMutex == NULL) ||
        (g_spiTouchMutex == NULL) ||
        (g_spiLoraMutex == NULL) ||
        (g_i2cBusMutex == NULL) ||
        (g_debugUartMutex == NULL))
    {
        App_Error(0x1001U);
    }

    if (App_StateInit() != 0)
    {
        App_Error(0x1002U);
    }

    g_initTaskHandle = osThreadNew(InitTask, NULL, &initTaskAttr);
    if (g_initTaskHandle == NULL)
    {
        App_Error(0x1003U);
    }
}

static void InitTask(void *argument)
{
    uint32_t done_mask = 0U;
    int32_t init_result;

    (void)argument;

    init_result = App_InitPeripherals(&done_mask);   //外设初始化
    App_StateSetInitResult(done_mask, init_result);

    if (init_result != 0)
    {
        App_Error(0x1100U | ((uint32_t)(-init_result) & 0xFFU));
    }

    if (App_StartRuntimeTasks() != 0)                //任务建立
    {
        App_Error(0x1200U);
    }

    osEventFlagsSet(g_sysEventFlags, SYS_EVT_INIT_DONE);
    App_DebugLog("init mask=0x%08lX", (unsigned long)done_mask);

    /*
     * 不删除 InitTask。删除任务会交给 idle task 调用
     * prvCheckTasksWaitingTermination() 清理 TCB，调试时容易误判为卡死。
     * 初始化完成后挂起即可，业务任务已经全部启动。
     */
    (void)osThreadSuspend(osThreadGetId());
    for (;;)
    {
        osDelay(osWaitForever);
    }
}

static int32_t App_InitPeripherals(uint32_t *done_mask)
{
    uint32_t mask = 0U;

    if (done_mask == NULL)
    {
        return -1;
    }

    if (Task_GyroInitHardware() != 0)
    {
        *done_mask = mask;
        return -2;
    }
    mask |= APP_INIT_GYRO_DMA;

    if (Task_UiInitHardware() != 0)
    {
        *done_mask = mask;
        return -3;
    }
    mask |= APP_INIT_UI;

    if (Task_LoraInitHardware() != 0)
    {
        *done_mask = mask;
        return -4;
    }
    mask |= APP_INIT_LORA;

    if (Task_Spo2InitHardware() != 0)
    {
        *done_mask = mask;
        return -5;
    }
    mask |= APP_INIT_SPO2;

    if (Task_AdcInitHardware() != 0)
    {
        *done_mask = mask;
        return -6;
    }
    mask |= APP_INIT_ADC;

    if (Task_DebugInitHardware() != 0)
    {
        *done_mask = mask;
        return -7;
    }
    mask |= APP_INIT_DEBUG;

    if (Task_UWBInitHardware() != 0)
    {
        *done_mask = mask;
        return -8;
    }
    mask |= APP_INIT_UWB;

    *done_mask = mask;
    return 0;
}

static int32_t App_StartRuntimeTasks(void)
{
    const osThreadAttr_t gyroTaskAttr = {
        .name = "GyroTask",
        .priority = osPriorityHigh,
        .stack_size = 512U
    };
    const osThreadAttr_t uiTaskAttr = {
        .name = "UiTask",
        .priority = osPriorityNormal,
        .stack_size = 2048U
    };
    const osThreadAttr_t controlTaskAttr = {
        .name = "ControlTask",
        .priority = osPriorityAboveNormal,
        .stack_size = 512U
    };
    const osThreadAttr_t loraTaskAttr = {
        .name = "LoraTask",
        .priority = osPriorityNormal,
        .stack_size = 1024U
    };
    const osThreadAttr_t spo2TaskAttr = {
        .name = "Spo2Task",
        .priority = osPriorityLow,
        .stack_size = 512U
    };
    const osThreadAttr_t adcTaskAttr = {
        .name = "AdcTask",
        .priority = osPriorityLow,
        .stack_size = 512U
    };
    const osThreadAttr_t debugTaskAttr = {
        .name = "DebugTask",
        .priority = osPriorityLow,
        .stack_size = 1024U
    };
    const osThreadAttr_t returnTaskAttr = {
        .name = "ReturnTask",
        .priority = osPriorityAboveNormal,
        .stack_size = 2048U
    };
    const osThreadAttr_t ins_pdr_TaskAttr = {
        .name = "ins_pdr_Task",
        .priority = osPriorityAboveNormal,
        .stack_size = 1024U
    };

    g_gyroTaskHandle = osThreadNew(Task_GyroEntry, NULL, &gyroTaskAttr);
    g_uiTaskHandle = osThreadNew(Task_UiEntry, NULL, &uiTaskAttr);
    g_controlTaskHandle = osThreadNew(Task_ControlEntry, NULL, &controlTaskAttr);
    g_loraTaskHandle = osThreadNew(Task_LoraEntry, NULL, &loraTaskAttr);
    g_spo2TaskHandle = osThreadNew(Task_Spo2Entry, NULL, &spo2TaskAttr);
    g_adcTaskHandle = osThreadNew(Task_AdcEntry, NULL, &adcTaskAttr);
    g_debugTaskHandle = osThreadNew(Task_DebugEntry, NULL, &debugTaskAttr);
    g_returnTaskHandle = osThreadNew(Task_ReturnEntry, NULL, &returnTaskAttr);
    g_ins_pdr_TaskHandle = osThreadNew(Task_Ins_Pdr_Entry, NULL, &ins_pdr_TaskAttr);
    if ((g_gyroTaskHandle == NULL) ||
        (g_uiTaskHandle == NULL) ||
        (g_controlTaskHandle == NULL) ||
        (g_loraTaskHandle == NULL) ||
        (g_spo2TaskHandle == NULL) ||
        (g_adcTaskHandle == NULL) ||
        (g_debugTaskHandle == NULL) ||
        (g_returnTaskHandle == NULL)||
        (g_ins_pdr_TaskHandle == NULL))
    {
        return -1;
    }

    return 0;
}
