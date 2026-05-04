#include "task_gyro.h"
#include "wit_imu.h"

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

__weak int32_t App_GyroHardwareInit(void)
{
	WitImu_Init(&APP_GYRO_UART_HANDLE);  
	if (WitImu_ConfigureJY901P() != HAL_OK)    // 配置输出寄存器、速率、带宽并保存
    {
        return -1;
    }

    return 0;
}

int32_t App_GyroDecodeFrame(const uint8_t *data,uint16_t len,GyroFrame_t *frame)
{
    WitImuData imu_data;
    uint16_t parsed_count;

    /*
     * 业务接入点：
     * 在新文件中实现同名强符号函数，把串口协议帧转换为 GyroFrame_t。
     * 返回 0 表示得到一组完整运动样本；正数表示继续等完整样本；负数表示解析错误。
     */
    if ((data == NULL) || (frame == NULL) || (len == 0U))
    {
        return -1;
    }

    parsed_count = WitImu_ParseBuffer(data, len);
    if (parsed_count == 0U)
    {
        return 1;   /* 当前 DMA 片段还没有拼出完整 WIT 帧。 */
    }

    if (WitImu_TakeSampleReady() == 0U)
    {
        return 1;   /* 等待加速度、角速度、角度和四元数凑齐后再更新状态。 */
    }

    if (WitImu_CopyData(&imu_data) == 0U)
    {
        return -2;
    }

    frame->acc_raw = imu_data.acc;
    frame->gyro_raw = imu_data.gyro;
    frame->angle_raw = imu_data.angle;
    frame->quat_raw = imu_data.quat;
    frame->temp_raw = imu_data.temp;

    frame->accel_mps2[0] = WitImu_AccRawToMps2(imu_data.acc.x);
    frame->accel_mps2[1] = WitImu_AccRawToMps2(imu_data.acc.y);
    frame->accel_mps2[2] = WitImu_AccRawToMps2(imu_data.acc.z);

    frame->gyro_rad_s[0] = WitImu_GyroRawToRps(imu_data.gyro.x);
    frame->gyro_rad_s[1] = WitImu_GyroRawToRps(imu_data.gyro.y);
    frame->gyro_rad_s[2] = WitImu_GyroRawToRps(imu_data.gyro.z);

    frame->angle_deg[0] = WitImu_AngleRawToDeg(imu_data.angle.x);
    frame->angle_deg[1] = WitImu_AngleRawToDeg(imu_data.angle.y);
    frame->angle_deg[2] = WitImu_AngleRawToDeg(imu_data.angle.z);

    frame->quat[0] = WitImu_QuatRawToFloat(imu_data.quat.q0);
    frame->quat[1] = WitImu_QuatRawToFloat(imu_data.quat.q1);
    frame->quat[2] = WitImu_QuatRawToFloat(imu_data.quat.q2);
    frame->quat[3] = WitImu_QuatRawToFloat(imu_data.quat.q3);
    frame->temp_deg_c = ((float)imu_data.temp) / 100.0f;

    return 0;
}

int32_t Task_GyroInitHardware(void)
{
    if (App_GyroHardwareInit() != 0)
    {
        return -1;
    }

    __HAL_UART_CLEAR_FLAG(&APP_GYRO_UART_HANDLE, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(&APP_GYRO_UART_HANDLE, UART_RXDATA_FLUSH_REQUEST);//没必要但是保留吧

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
            decode_result = App_GyroDecodeFrame(msg.data, msg.len, &frame);
            gyro.last_parse_result = decode_result;
            if (decode_result == 0)
            {
                gyro.frame = frame;
            }
            else if (decode_result < 0)
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
            msg.len = copy_len;

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
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

        if (Task_GyroStartRx() != 0)
        {
            s_gyroRestartError++;
        }
    }
}
