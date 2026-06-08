#include "app_state.h"
#include "cmsis_os2.h"

#include <string.h>

/*
 * 每类业务数据保留一份“最新状态”。
 * UI/返航等低频任务读取快照，避免直接依赖其他任务的内部变量。
 */
static AppSystemState_t g_systemState;
static GyroState_t g_gyroState;
static NavState_t g_navState;
static AdcState_t g_adcState;
static Spo2State_t g_spo2State;
static UiState_t g_uiState;
static LoraState_t g_loraState;
static ReturnState_t g_returnState;
static ReturnGuideState_t g_returnGuideState;
static osMutexId_t g_stateMutex = NULL;

int32_t App_StateInit(void)
{
    /* 状态必须在任务启动前清零，防止 UI 显示到未初始化数据。 */
    memset(&g_systemState, 0, sizeof(g_systemState));
    memset(&g_gyroState, 0, sizeof(g_gyroState));
    memset(&g_navState, 0, sizeof(g_navState));
    memset(&g_adcState, 0, sizeof(g_adcState));
    memset(&g_spo2State, 0, sizeof(g_spo2State));
    memset(&g_uiState, 0, sizeof(g_uiState));
    memset(&g_loraState, 0, sizeof(g_loraState));
    memset(&g_returnState, 0, sizeof(g_returnState));
    memset(&g_returnGuideState, 0, sizeof(g_returnGuideState));

    g_returnState.mode = RETURN_MODE_IDLE;

    g_stateMutex = osMutexNew(NULL);

    if (g_stateMutex == NULL)
    {
        return -1;
    }

    return 0;
}

void App_StateSetSystem(const AppSystemState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_systemState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetSystem(AppSystemState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_systemState;
    osMutexRelease(g_stateMutex);
}

void App_StateSetInitResult(uint32_t done_mask, int32_t result)
{
    if (g_stateMutex == NULL)
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_systemState.init_done_mask = done_mask;
    g_systemState.init_result = result;
    osMutexRelease(g_stateMutex);
}

void App_StateAddFault(uint32_t fault_code)
{
    if (g_stateMutex == NULL)
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_systemState.fault_count++;
    g_systemState.last_fault_code = fault_code;
    osMutexRelease(g_stateMutex);
}

void App_StateSetGyro(const GyroState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_gyroState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetGyro(GyroState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_gyroState;
    osMutexRelease(g_stateMutex);
}

void App_StateSetNav(const NavState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_navState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetNav(NavState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_navState;
    osMutexRelease(g_stateMutex);
}

void App_StateSetAdc(const AdcState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_adcState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetAdc(AdcState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_adcState;
    osMutexRelease(g_stateMutex);
}

void App_StateSetSpo2(const Spo2State_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_spo2State = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetSpo2(Spo2State_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_spo2State;
    osMutexRelease(g_stateMutex);
}

void App_StateSetUi(const UiState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_uiState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetUi(UiState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_uiState;
    osMutexRelease(g_stateMutex);
}

void App_StateSetLora(const LoraState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_loraState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetLora(LoraState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_loraState;
    osMutexRelease(g_stateMutex);
}

void App_StateSetReturn(const ReturnState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_returnState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetReturn(ReturnState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_returnState;
    osMutexRelease(g_stateMutex);
}

void App_StateSetReturnGuide(const ReturnGuideState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    g_returnGuideState = *state;
    osMutexRelease(g_stateMutex);
}

void App_StateGetReturnGuide(ReturnGuideState_t *state)
{
    if ((state == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    osMutexAcquire(g_stateMutex, osWaitForever);
    *state = g_returnGuideState;
    osMutexRelease(g_stateMutex);
}

void App_StateGetSnapshot(AppSnapshot_t *snapshot)
{
    if ((snapshot == NULL) || (g_stateMutex == NULL))
    {
        return;
    }

    /* 单次锁住复制整机快照，保证 UI/返航看到的数据来自同一时刻。 */
    osMutexAcquire(g_stateMutex, osWaitForever);
    snapshot->system = g_systemState;
    snapshot->gyro = g_gyroState;
    snapshot->nav = g_navState;
    snapshot->adc = g_adcState;
    snapshot->spo2 = g_spo2State;
    snapshot->ui = g_uiState;
    snapshot->lora = g_loraState;
    snapshot->return_home = g_returnState;
    snapshot->return_guide = g_returnGuideState;
    osMutexRelease(g_stateMutex);
}
