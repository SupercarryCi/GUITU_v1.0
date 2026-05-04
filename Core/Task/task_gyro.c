#include "task_gyro.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"

#include <string.h>

/*
 * 陀螺仪链路：
 * USART1 ReceiveToIdle DMA -> HAL 回调复制原始帧 -> g_gyroRxQueue ->
 * GyroTask 解码 -> GyroState。
 */
static uint8_t s_gyroRxDmaBuffer[APP_GYRO_DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer"), aligned(32)));//我将稳稳地接住你
static volatile uint32_t s_gyroDropFromIsr = 0U;
static volatile uint32_t s_gyroRestartError = 0U;

/* 待你完善：按实际陀螺仪串口协议拆帧、校验，并填充 GyroFrame_t。 */
/* 待你完善：每次上电后在这里配置陀螺仪输出内容、量程、频率等寄存器。 */
__weak int32_t App_GyroHardwareInit(void)
{
    return 0;
}

__weak int32_t App_GyroDecodeFrame(const uint8_t *data,
                                   uint16_t len,
                                   GyroFrame_t *frame)
{
    /*
     * 业务接入点：
     * 在新文件中实现同名强符号函数，把串口协议帧转换为 GyroFrame_t。
     * 返回 0 表示解析成功；返回非 0 表示本帧丢弃。
     */
    (void)data;
    (void)len;
    (void)frame;
    return -1;
}

int32_t Task_GyroInitHardware(void)
{
    if (App_GyroHardwareInit() != 0)
    {
        return -1;
    }

    return Task_GyroStartRx();
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

    (void)argument;

    memset(&gyro, 0, sizeof(gyro));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
        if (osMessageQueueGet(g_gyroRxQueue, &msg, NULL, osWaitForever) == osOK)
        {
            GyroFrame_t frame;
            int32_t decode_result;
            uint16_t copy_len = msg.len;

            if (copy_len > APP_GYRO_RX_MAX_LEN)
            {
                copy_len = APP_GYRO_RX_MAX_LEN;
            }

            gyro.rx_count++;
            gyro.drop_count = s_gyroDropFromIsr + s_gyroRestartError;
            gyro.raw_len = copy_len;
            memcpy(gyro.raw, msg.data, copy_len);

            memset(&frame, 0, sizeof(frame));
            /* 待你完善：App_GyroDecodeFrame 输出的单位必须匹配 GyroFrame_t 定义。 */
            /* 协议解析在任务上下文执行，避免在中断里做重活。 */
            decode_result = App_GyroDecodeFrame(msg.data, msg.len, &frame);
            gyro.last_parse_result = decode_result;
            if (decode_result == 0)
            {
                gyro.frame = frame;
            }
            else
            {
                gyro.parse_error_count++;
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
            /* ISR 只复制定长小缓冲并投队列，不做协议解析。 */
            memset(&msg, 0, sizeof(msg));
            memcpy(msg.data, s_gyroRxDmaBuffer, copy_len);
            msg.len = Size;

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
