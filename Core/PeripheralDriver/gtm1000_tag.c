/**
  ******************************************************************************
  * @file    gtm1000_tag.c
  * @brief   GTM1000 UWB标签卡独立初始化配置文件
  ******************************************************************************
  */

#include "gtm1000_tag.h"

/* 协议固定字段 */
#define GTM1000_TAG_HEAD1                 0x59u
#define GTM1000_TAG_HEAD2                 0x4Du

/* 配置命令 */
#define GTM1000_TAG_CMD_IMU_SLEEP_SET     0x0Fu
#define GTM1000_TAG_CMD_PERIOD_SET        0x11u

/* 帧长度参数 */
#define GTM1000_TAG_FIXED_LEN             7u
#define GTM1000_TAG_CHECKSUM_LEN          1u
#define GTM1000_TAG_MAX_PAYLOAD_LEN       100u
#define GTM1000_TAG_TX_TIMEOUT_MS         100u

static uint16_t g_gtm1000_tag_seq;

/**
  * @brief  计算协议校验和。
  * @param  data 待计算数据，不包含校验和字节
  * @param  len  待计算长度
  * @retval 所有字节无符号累加后的低 8 位
  */
static uint8_t GTM1000_TagCalcChecksum(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint8_t sum;

  sum = 0u;
  for (i = 0u; i < len; i++)
  {
    sum = (uint8_t)(sum + data[i]);
  }

  return sum;
}

/**
  * @brief  组协议帧并通过 UART 发送。
  * @param  huart       连接 GTM1000 的串口句柄
  * @param  cmd         命令类型
  * @param  payload     净荷数据，长度为 0 时可传 NULL
  * @param  payload_len 净荷长度，最大 100 字节
  * @retval GTM1000_TAG_OK 或错误码
  */
static GTM1000_TagStatusTypeDef GTM1000_TagSendFrame(UART_HandleTypeDef *huart,
                                                     uint8_t cmd,
                                                     const uint8_t *payload,
                                                     uint16_t payload_len)
{
  uint8_t frame[GTM1000_TAG_FIXED_LEN + GTM1000_TAG_MAX_PAYLOAD_LEN + GTM1000_TAG_CHECKSUM_LEN];
  uint16_t frame_len;
  uint16_t i;
  HAL_StatusTypeDef hal_status;

  if ((huart == 0) || ((payload == 0) && (payload_len != 0u)))
  {
    return GTM1000_TAG_PARAM_ERROR;
  }

  if (payload_len > GTM1000_TAG_MAX_PAYLOAD_LEN)
  {
    return GTM1000_TAG_PARAM_ERROR;
  }

  frame[0] = GTM1000_TAG_HEAD1;
  frame[1] = GTM1000_TAG_HEAD2;
  frame[2] = cmd;
  frame[3] = (uint8_t)(g_gtm1000_tag_seq & 0xFFu);
  frame[4] = (uint8_t)((g_gtm1000_tag_seq >> 8) & 0xFFu);
  frame[5] = (uint8_t)(payload_len & 0xFFu);
  frame[6] = (uint8_t)((payload_len >> 8) & 0xFFu);

  for (i = 0u; i < payload_len; i++)
  {
    frame[GTM1000_TAG_FIXED_LEN + i] = payload[i];
  }

  frame_len = (uint16_t)(GTM1000_TAG_FIXED_LEN + payload_len);
  frame[frame_len] = GTM1000_TagCalcChecksum(frame, frame_len);
  frame_len++;

  hal_status = HAL_UART_Transmit(huart, frame, frame_len, GTM1000_TAG_TX_TIMEOUT_MS);
  if (hal_status != HAL_OK)
  {
    return GTM1000_TAG_TX_ERROR;
  }

  g_gtm1000_tag_seq++;
  return GTM1000_TAG_OK;
}

/**
  * @brief  设置 GTM1000 标签卡 IMU 休眠。
  * @param  huart  连接 GTM1000 的串口句柄
  * @param  enable 0：关闭休眠；非 0：开启休眠
  * @retval GTM1000_TAG_OK 或错误码
  */
GTM1000_TagStatusTypeDef GTM1000_TagSetImuSleep(UART_HandleTypeDef *huart, uint8_t enable)
{
  uint8_t payload;

  payload = (enable != 0u) ? 1u : 0u;
  return GTM1000_TagSendFrame(huart, GTM1000_TAG_CMD_IMU_SLEEP_SET, &payload, 1u);
}

/**
  * @brief  设置 GTM1000 标签卡定位周期。
  * @param  huart  连接 GTM1000 的串口句柄
  * @param  period 周期参数；10 表示 0.03s
  * @retval GTM1000_TAG_OK 或错误码
  */
GTM1000_TagStatusTypeDef GTM1000_TagSetLocatePeriod(UART_HandleTypeDef *huart, uint8_t period)
{
  return GTM1000_TagSendFrame(huart, GTM1000_TAG_CMD_PERIOD_SET, &period, 1u);
}

/**
  * @brief  初始化并配置 GTM1000 标签卡。
  * @param  huart 连接 GTM1000 的串口句柄，例如 &huart7
  * @retval GTM1000_TAG_OK 或错误码
  * @note   默认不使用 SOS 脚，不设置 ID，只开启 IMU 休眠并设置定位周期为 0.03s。
  */
GTM1000_TagStatusTypeDef GTM1000_TagInit(UART_HandleTypeDef *huart)
{
  GTM1000_TagStatusTypeDef status;

  if (huart == 0)
  {
    return GTM1000_TAG_PARAM_ERROR;
  }

  status = GTM1000_TagSetImuSleep(huart, GTM1000_TAG_IMU_SLEEP_ENABLE);
  if (status != GTM1000_TAG_OK)
  {
    return status;
  }

  HAL_Delay(GTM1000_TAG_CONFIG_DELAY_MS);

  status = GTM1000_TagSetLocatePeriod(huart, GTM1000_TAG_LOCATE_PERIOD_30MS);
  if (status != GTM1000_TAG_OK)
  {
    return status;
  }

  return GTM1000_TAG_OK;
}
