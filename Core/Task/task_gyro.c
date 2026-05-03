#include "task_gyro.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "ins_nav.h"

#include <string.h>

static uint8_t s_gyroRxDmaBuffer[APP_GYRO_DMA_BUFFER_SIZE];
static volatile uint32_t s_gyroDropFromIsr = 0U;
static volatile uint32_t s_gyroRestartError = 0U;

__weak int32_t App_GyroDecodeFrame(const uint8_t *data,
                                   uint16_t len,
                                   INS_SensorFrame *frame)
{
    (void)data;
    (void)len;
    (void)frame;
    return -1;
}

static uint32_t task_gyro_tick(void)
{
    return osKernelGetTickCount();
}

int32_t Task_GyroStartRx(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(&APP_GYRO_UART_HANDLE,
                                          s_gyroRxDmaBuffer,
                                          sizeof(s_gyroRxDmaBuffer));
    if (status != HAL_OK)
    {
        return -1;
    }

    if (APP_GYRO_UART_HANDLE.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(APP_GYRO_UART_HANDLE.hdmarx, DMA_IT_HT);
    }

    return 0;
}

void Task_GyroEntry(void *argument)
{
    GyroRxMsg_t msg;
    GyroState_t gyro;
    NavState_t nav;
    INS_Context ins_ctx;
    INS_Config ins_config;

    (void)argument;

    memset(&gyro, 0, sizeof(gyro));
    memset(&nav, 0, sizeof(nav));
    INS_DefaultConfig(&ins_config);
    INS_Init(&ins_ctx, &ins_config);

    osEventFlagsWait(g_sysEventFlags, SYS_EVT_INIT_DONE, osFlagsWaitAny, osWaitForever);

    for (;;)
    {
        if (osMessageQueueGet(g_gyroRxQueue, &msg, NULL, osWaitForever) == osOK)
        {
            INS_SensorFrame frame;
            INS_State ins_state;
            INS_Status ins_status = INS_STATUS_INVALID_ARGUMENT;
            int32_t decode_result;
            uint16_t copy_len = msg.len;

            if (copy_len > APP_GYRO_RX_MAX_LEN)
            {
                copy_len = APP_GYRO_RX_MAX_LEN;
            }

            gyro.rx_count++;
            gyro.drop_count = s_gyroDropFromIsr + s_gyroRestartError;
            gyro.raw_len = copy_len;
            gyro.last_tick_ms = msg.tick_ms;
            memcpy(gyro.raw, msg.data, copy_len);

            memset(&frame, 0, sizeof(frame));
            decode_result = App_GyroDecodeFrame(msg.data, msg.len, &frame);
            if (decode_result == 0)
            {
                gyro.frame = frame;
                gyro.frame_valid = 1U;
                ins_status = INS_UpdateSensorFrame(&ins_ctx, &frame, 0U, &ins_state);
                gyro.last_ins_status = ins_status;

                if (ins_status == INS_STATUS_OK)
                {
                    nav.valid = 1U;
                    nav.update_count++;
                    nav.last_tick_ms = task_gyro_tick();
                    nav.state = ins_state;
                    App_StateSetNav(&nav);
                    osEventFlagsSet(g_sysEventFlags, SYS_EVT_NAV_UPDATED);
                }
            }
            else
            {
                gyro.frame_valid = 0U;
                gyro.parse_error_count++;
                gyro.last_ins_status = INS_STATUS_INVALID_ARGUMENT;
            }

            App_StateSetGyro(&gyro);
            osEventFlagsSet(g_sysEventFlags, SYS_EVT_GYRO_UPDATED);
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &APP_GYRO_UART_HANDLE)
    {
        GyroRxMsg_t msg;
        uint16_t copy_len = Size;

        if (copy_len > APP_GYRO_RX_MAX_LEN)
        {
            copy_len = APP_GYRO_RX_MAX_LEN;
        }

        if ((copy_len > 0U) && (g_gyroRxQueue != NULL))
        {
            memset(&msg, 0, sizeof(msg));
            memcpy(msg.data, s_gyroRxDmaBuffer, copy_len);
            msg.len = Size;
            msg.tick_ms = osKernelGetTickCount();

            if (osMessageQueuePut(g_gyroRxQueue, &msg, 0U, 0U) != osOK)
            {
                s_gyroDropFromIsr++;
            }
        }

        if (Task_GyroStartRx() != 0)
        {
            s_gyroRestartError++;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &APP_GYRO_UART_HANDLE)
    {
        if (Task_GyroStartRx() != 0)
        {
            s_gyroRestartError++;
        }
    }
}
