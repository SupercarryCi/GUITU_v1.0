#ifndef APP_RTOS_H
#define APP_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"

extern osEventFlagsId_t g_sysEventFlags;
extern osMessageQueueId_t g_gyroRxQueue;
extern osMessageQueueId_t g_uiCmdQueue;
extern osMessageQueueId_t g_loraTxQueue;
extern osMessageQueueId_t g_loraRxQueue;
extern osMessageQueueId_t g_returnCmdQueue;
extern osMessageQueueId_t g_debugLogQueue;
extern osSemaphoreId_t g_adcReadySem;
extern osMutexId_t g_spiDisplayMutex;
extern osMutexId_t g_spiTouchMutex;
extern osMutexId_t g_spiLoraMutex;
extern osMutexId_t g_i2cBusMutex;
extern osMutexId_t g_debugUartMutex;

void App_RtosCreateObjects(void);

#ifdef __cplusplus
}
#endif

#endif
