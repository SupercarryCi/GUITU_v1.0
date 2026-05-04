#include "wit_imu.h"
#include <stddef.h>
#include <string.h>

/* JY901P写寄存器命令固定5字节：FF AA REG DATAL DATAH。
 * 注意：读取主动输出数据不需要解锁；写寄存器必须先写KEY解锁。
 */
#define WIT_REG_SAVE       WIT_IMU_REG_SAVE
#define WIT_REG_RSW        WIT_IMU_REG_RSW
#define WIT_REG_RRATE      WIT_IMU_REG_RRATE
#define WIT_REG_BANDWIDTH  WIT_IMU_REG_BANDWIDTH
#define WIT_REG_READADDR   WIT_IMU_REG_READADDR
#define WIT_REG_KEY        WIT_IMU_REG_KEY

#define WIT_KEY_UNLOCK     WIT_IMU_KEY_UNLOCK
#define WIT_SAVE_NOW       WIT_IMU_SAVE_NOW

/* 当前采集0x51加速度、0x52角速度、0x53角度、0x59四元数。
 * 四类帧都到齐后，输出最新一组运动数据。
 */
#define WIT_RSW_JY901P     WIT_IMU_RSW_MOTION_QUAT
#define WIT_RRATE_200HZ    WIT_IMU_RRATE_200HZ
#define WIT_BANDWIDTH_98HZ WIT_IMU_BANDWIDTH_98HZ
#define WIT_IMU_PI         3.1415926f
#define WIT_SAMPLE_READY_MASK (WIT_IMU_DATA_ACC | WIT_IMU_DATA_GYRO | WIT_IMU_DATA_ANGLE | WIT_IMU_DATA_QUAT)

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static uint8_t s_frame[WIT_IMU_FRAME_LEN];
static uint8_t s_index;
static uint8_t s_read_request_addr;
static uint8_t s_sample_mask;
static volatile uint8_t s_sample_ready;
static WitImuData s_data;

static HAL_StatusTypeDef write_reg(uint8_t reg, uint16_t value);

static int16_t make_int16(uint8_t low, uint8_t high)
{
  /* 协议低字节在前，高字节在后。
   * 合成uint16_t后再转int16_t，可以正确保留符号位。
   * 例如0xFF9C会被转换成-100，而不是65436。
   */
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static uint8_t checksum_ok(const uint8_t *frame)
{
  uint8_t sum = 0U;
  uint8_t i;

  for (i = 0U; i < (WIT_IMU_FRAME_LEN - 1U); i++)
  {
    sum = (uint8_t)(sum + frame[i]);
  }

  return (sum == frame[WIT_IMU_FRAME_LEN - 1U]);
}

static uint8_t parse_frame(const uint8_t *frame)
{
  uint8_t type = frame[1];

  /* SUM校验失败时直接丢弃整帧，等待下一帧重新同步。 */
  if (checksum_ok(frame) == 0U)
  {
    s_data.checksum_error++;
    s_sample_mask = 0U;
    return 0U;
  }

  s_data.frame_count++;
  s_data.last_type = type;

  switch (type)
  {
    case WIT_IMU_TYPE_TIME:
      s_data.time_count++;
      s_data.time.year = frame[2];
      s_data.time.month = frame[3];
      s_data.time.day = frame[4];
      s_data.time.hour = frame[5];
      s_data.time.minute = frame[6];
      s_data.time.second = frame[7];
      s_data.time.millisecond = (uint16_t)frame[8] | ((uint16_t)frame[9] << 8);
      s_data.valid |= WIT_IMU_DATA_TIME;
      return 1U;

    case WIT_IMU_TYPE_ACC:
      /* 加速度帧：Ax Ay Az Temp。 */
      s_data.acc_count++;
      s_data.acc.x = make_int16(frame[2], frame[3]);
      s_data.acc.y = make_int16(frame[4], frame[5]);
      s_data.acc.z = make_int16(frame[6], frame[7]);
      s_data.temp = make_int16(frame[8], frame[9]);
      s_data.valid |= WIT_IMU_DATA_ACC;
      return 1U;

    case WIT_IMU_TYPE_GYRO:
      /* 角速度帧：Wx Wy Wz Voltage；这里暂不使用Voltage。 */
      s_data.gyro_count++;
      s_data.gyro.x = make_int16(frame[2], frame[3]);
      s_data.gyro.y = make_int16(frame[4], frame[5]);
      s_data.gyro.z = make_int16(frame[6], frame[7]);
      s_data.valid |= WIT_IMU_DATA_GYRO;
      return 1U;

    case WIT_IMU_TYPE_ANGLE:
      /* 角度帧：Roll Pitch Yaw Version；这里暂不使用Version。 */
      s_data.angle_count++;
      s_data.angle.x = make_int16(frame[2], frame[3]);
      s_data.angle.y = make_int16(frame[4], frame[5]);
      s_data.angle.z = make_int16(frame[6], frame[7]);
      s_data.valid |= WIT_IMU_DATA_ANGLE;
      return 1U;

    case WIT_IMU_TYPE_QUAT:
      /* 四元数帧：q0 q1 q2 q3。 */
      s_data.quat_count++;
      s_data.quat.q0 = make_int16(frame[2], frame[3]);
      s_data.quat.q1 = make_int16(frame[4], frame[5]);
      s_data.quat.q2 = make_int16(frame[6], frame[7]);
      s_data.quat.q3 = make_int16(frame[8], frame[9]);
      s_data.valid |= WIT_IMU_DATA_QUAT;
      return 1U;

    case WIT_IMU_TYPE_READ:
      /* 读寄存器返回：从read_addr开始连续4个16位寄存器。 */
      s_data.read_count++;
      s_data.read_addr = s_read_request_addr;
      s_data.read_reg[0] = (uint16_t)make_int16(frame[2], frame[3]);
      s_data.read_reg[1] = (uint16_t)make_int16(frame[4], frame[5]);
      s_data.read_reg[2] = (uint16_t)make_int16(frame[6], frame[7]);
      s_data.read_reg[3] = (uint16_t)make_int16(frame[8], frame[9]);
      s_data.valid |= WIT_IMU_DATA_READ;
      return 1U;

    default:
      /* 未开启的类型会到这里，例如设备仍在输出磁场时直接忽略。 */
      s_data.unknown_count++;
      s_sample_mask = 0U;
      return 0U;
  }
}

void WitImu_Init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  s_rx_byte = 0U;
  s_index = 0U;
  s_read_request_addr = 0U;
  s_sample_mask = 0U;
  s_sample_ready = 0U;
  memset(&s_data, 0, sizeof(s_data));
}

HAL_StatusTypeDef WitImu_StartReceiveIT(void)
{
  if (s_uart == NULL)
  {
    return HAL_ERROR;
  }

  /* Clear stale UART state before arming the first 1-byte IT receive. */
  __HAL_UART_CLEAR_FLAG(s_uart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
  __HAL_UART_SEND_REQ(s_uart, UART_RXDATA_FLUSH_REQUEST);
  return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

static uint8_t is_imu_uart(UART_HandleTypeDef *huart)
{
  if ((s_uart == NULL) || (huart == NULL))
  {
    return 0U;
  }

  return (huart->Instance == s_uart->Instance) ? 1U : 0U;
}

uint8_t WitImu_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t parsed = 0U;

  if (is_imu_uart(huart) == 0U)
  {
    return 0U;
  }

  if (WitImu_ParseByte(s_rx_byte) != 0U)
  {
    parsed = 1U;
  }

  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
  return parsed;
}

void WitImu_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (is_imu_uart(huart) == 0U)
  {
    return;
  }

  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

uint8_t WitImu_TakeSampleReady(void)
{
  uint8_t ready;

  __disable_irq();
  ready = s_sample_ready;
  s_sample_ready = 0U;
  __enable_irq();

  return ready;
}

uint8_t WitImu_CopyData(WitImuData *data)
{
  if (data == NULL)
  {
    return 0U;
  }

  /* Parser updates s_data in UART ISR, so copy it as one snapshot. */
  __disable_irq();
  *data = s_data;
  __enable_irq();

  return 1U;
}

uint8_t WitImu_ParseByte(uint8_t byte)
{
  /* 简单串口字节流解析：
   * 1. 空闲时只等待0x55帧头；
   * 2. 收满11字节后做SUM校验；
   * 3. 校验通过则解析TYPE对应的数据。
   */
  if (s_index == 0U)
  {
    if (byte == WIT_IMU_FRAME_HEAD)
    {
      s_frame[s_index++] = byte;
    }
    return 0U;
  }

  s_frame[s_index++] = byte;

  if (s_index >= WIT_IMU_FRAME_LEN)
  {
    uint8_t parsed;
    uint8_t data_mask = 0U;

    s_index = 0U;
    parsed = parse_frame(s_frame);
    if (parsed != 0U)
    {
      switch (s_data.last_type)
      {
        case WIT_IMU_TYPE_ACC:
          data_mask = WIT_IMU_DATA_ACC;
          break;

        case WIT_IMU_TYPE_GYRO:
          data_mask = WIT_IMU_DATA_GYRO;
          break;

        case WIT_IMU_TYPE_ANGLE:
          data_mask = WIT_IMU_DATA_ANGLE;
          break;

        case WIT_IMU_TYPE_QUAT:
          data_mask = WIT_IMU_DATA_QUAT;
          break;

        default:
          data_mask = 0U;
          break;
      }

      s_sample_mask |= data_mask;
      if ((s_sample_mask & WIT_SAMPLE_READY_MASK) == WIT_SAMPLE_READY_MASK)
      {
        s_sample_ready = 1U;
        s_sample_mask = 0U;
      }
    }
    return parsed;
  }

  return 0U;
}

uint16_t WitImu_ParseBuffer(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint16_t frame_count = 0U;

  if (data == NULL)
  {
    return 0U;
  }

  /* DMA+IDLE工程在RxEvent回调里把本次收到的数据直接喂给这里。 */
  for (i = 0U; i < len; i++)
  {
    if (WitImu_ParseByte(data[i]) != 0U)
    {
      frame_count++;
    }
  }

  return frame_count;
}

const WitImuData *WitImu_GetData(void)
{
  return &s_data;
}

float WitImu_AccRawToMps2(int16_t raw)
{
  return ((float)raw) * (16.0f * WIT_IMU_GRAVITY_MPS2 / 32768.0f);
}

float WitImu_GyroRawToRps(int16_t raw)
{
  return ((float)raw) * (2000.0f * WIT_IMU_PI / 180.0f / 32768.0f);
}

float WitImu_AngleRawToDeg(int16_t raw)
{
  return ((float)raw) * (180.0f / 32768.0f);
}

float WitImu_QuatRawToFloat(int16_t raw)
{
  return ((float)raw) / 32768.0f;
}

static HAL_StatusTypeDef write_reg(uint8_t reg, uint16_t value)
{
  uint8_t cmd[5];

  if (s_uart == NULL)
  {
    return HAL_ERROR;
  }

  cmd[0] = 0xFFU;
  cmd[1] = 0xAAU;
  cmd[2] = reg;
  cmd[3] = (uint8_t)(value & 0xFFU);  /* 低字节先发 */
  cmd[4] = (uint8_t)(value >> 8);     /* 高字节后发 */

  return HAL_UART_Transmit(s_uart, cmd, sizeof(cmd), 100U);
}

HAL_StatusTypeDef WitImu_WriteRegister(uint8_t reg, uint16_t value)
{
  return write_reg(reg, value);
}

HAL_StatusTypeDef WitImu_Unlock(void)
{
  return write_reg(WIT_REG_KEY, WIT_KEY_UNLOCK);
}

HAL_StatusTypeDef WitImu_SaveConfig(void)
{
  return write_reg(WIT_REG_SAVE, WIT_SAVE_NOW);
}

HAL_StatusTypeDef WitImu_SetOutput(uint16_t rsw)
{
  return write_reg(WIT_REG_RSW, rsw);
}

HAL_StatusTypeDef WitImu_SetReportRate(uint16_t rrate)
{
  return write_reg(WIT_REG_RRATE, rrate);
}

HAL_StatusTypeDef WitImu_SetGyroBandwidth(uint16_t bandwidth)
{
  return write_reg(WIT_REG_BANDWIDTH, bandwidth);
}

HAL_StatusTypeDef WitImu_SetGyroBandwidth98Hz(void)
{
  /* BANDWIDTH寄存器0x1F：0x02对应98Hz内部滤波带宽。 */
  return WitImu_SetGyroBandwidth(WIT_BANDWIDTH_98HZ);
}

HAL_StatusTypeDef WitImu_ConfigureJY901P(void)
{
  /* 写配置必须先解锁，10秒内完成配置并保存。
   * 这里不再写BAUD：当前已经能用115200收到数据，说明传感器波特率正确。
   * 只写输出内容、输出速率和内部滤波带宽，避免波特率寄存器带来额外切换时序。
   */
  if (WitImu_Unlock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(200U);

  /* 打开加速度、角速度、角度和四元数输出，当前用于采集完整运动数据。 */
  if (WitImu_SetOutput(WIT_RSW_JY901P) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(100U);

  /* JY901P最高200Hz。 */
  if (WitImu_SetReportRate(WIT_RRATE_200HZ) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(100U);

  /* 陀螺仪内部滤波带宽配置为98Hz，比42Hz提高一档。 */
  if (WitImu_SetGyroBandwidth98Hz() != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(100U);

  if (WitImu_SaveConfig() != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(100U);

  /* 读回RSW/RRATE/BAUD，串口1调试输出会用这个值判断配置是否真的生效。 */
  return WitImu_ReadRegisters(WIT_REG_RSW);
}

HAL_StatusTypeDef WitImu_ReadRegisters(uint8_t start_addr)
{
  s_read_request_addr = start_addr;
  return write_reg(WIT_REG_READADDR, start_addr);
}
