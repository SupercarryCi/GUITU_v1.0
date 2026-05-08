/**
  ******************************************************************************
  * @file    gtm1000_tag.h
  * @brief   GTM1000 UWB标签卡独立初始化配置文件
  ******************************************************************************
  * @note
  *  本模块是一个可直接移植的完整文件组，只依赖 STM32 HAL 的 UART 发送函数。
  *  移植到其它 STM32 工程时，只需要拷贝 gtm1000_tag.h 和 gtm1000_tag.c，
  *  然后在串口初始化完成后调用 GTM1000_TagInit(&huartx) 即可。
  *
  *  默认配置：
  *    1. 不使用 SOS 唤醒脚
  *    2. 不修改模块 ID
  *    3. 开启 IMU 休眠
  *    4. 定位周期设置为 0.03s
  ******************************************************************************
  */

#ifndef GTM1000_TAG_H
#define GTM1000_TAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* GTM1000 标签卡默认配置 */
#define GTM1000_TAG_IMU_SLEEP_ENABLE       1u
#define GTM1000_TAG_LOCATE_PERIOD_30MS     10u
#define GTM1000_TAG_CONFIG_DELAY_MS        10u

typedef enum
{
  GTM1000_TAG_OK = 0,
  GTM1000_TAG_ERROR = -1,
  GTM1000_TAG_PARAM_ERROR = -2,
  GTM1000_TAG_TX_ERROR = -3
} GTM1000_TagStatusTypeDef;

GTM1000_TagStatusTypeDef GTM1000_TagInit(UART_HandleTypeDef *huart);
GTM1000_TagStatusTypeDef GTM1000_TagSetImuSleep(UART_HandleTypeDef *huart, uint8_t enable);
GTM1000_TagStatusTypeDef GTM1000_TagSetLocatePeriod(UART_HandleTypeDef *huart, uint8_t period);

#ifdef __cplusplus
}
#endif

#endif /* GTM1000_TAG_H */
