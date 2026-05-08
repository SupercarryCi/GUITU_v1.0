#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"

#define APP_GYRO_UART_HANDLE        huart2      /* 陀螺仪数据接收串口 (USART2) */
#define APP_DEBUG_UART_HANDLE       huart3      /* 调试日志输出串口 (USART3) */
#define APP_UWB_UART_HANDLE         huart1      /* UWB标签卡配置串口 (USART1) */

#define APP_DISPLAY_SPI_HANDLE      hspi1       /* 显示屏 SPI 接口 */
#define APP_TOUCH_SPI_HANDLE        hspi2       /* 触摸面板 SPI 接口 */
#define APP_LORA_SPI_HANDLE         hspi4       /* LoRa 射频模块 SPI 接口 */

#define APP_SPO2_I2C_HANDLE         hi2c1       /* 血氧传感器 I2C 接口 */

#define APP_ADC_HANDLE              hadc1       /* 通用 ADC (电池/电压采集) */

#define APP_GYRO_RX_MAX_LEN         96U         /* 单次 DMA 投队列的最大字节数 */
#define APP_GYRO_DMA_BUFFER_SIZE    APP_GYRO_RX_MAX_LEN   /* DMA 接收缓冲区大小 */
#define APP_GYRO_QUEUE_DEPTH        16U         /* 陀螺仪接收队列深度  */

#define APP_UI_CMD_QUEUE_DEPTH      8U          /* 用户操作命令队列深度 */
#define APP_LORA_QUEUE_DEPTH        4U          /* LoRa 收发队列深度 */
#define APP_RETURN_QUEUE_DEPTH      4U          /* 返航控制命令队列深度 */
#define APP_DEBUG_LOG_QUEUE_DEPTH   16U         /* 调试日志队列深度 */

#define APP_LORA_MAX_PAYLOAD_LEN    128U        /* LoRa 单包最大负载字节数 */
#define APP_DEBUG_LOG_TEXT_LEN      32U         /* 单条调试日志最大文本长度 */

#define APP_ADC_CHANNEL_COUNT       2U          /* ADC 采集通道数量 */
#define APP_ADC_SAMPLE_PERIOD_MS    100U        /* ADC 采样周期 (ms) */
#define APP_ADC_SAMPLE_TIMEOUT_MS   20U         /* 单次 ADC 采样超时时间 (ms) */
#define APP_ADC_VREF_MV             3300U       /* ADC 参考电压 (mV)，用于换算实际电压 */
#define APP_ADC_FULL_SCALE          65535U      /* ADC 满量程值 (16bit) */

#define APP_UI_PERIOD_MS            50U         /* UI 刷新周期 (20Hz) ?*/
#define APP_SPO2_PERIOD_MS          500U        /* 血氧采集周期 (2Hz) */
#define APP_LORA_PERIOD_MS          200U        /* LoRa 收发周期 (5Hz) */
#define APP_RETURN_PERIOD_MS        100U        /* 返航控制周期 (10Hz) */

/* LoRa is reserved by default. Set to 1 after the radio driver is ready. */
/* LoRa 当前只预留框架，默认不启用真实收发任务。 */
#define APP_LORA_ENABLE_DEFAULT     0U

#define APP_INIT_GYRO_DMA           (1UL << 0)  /* 陀螺仪 DMA 接收启动完成 */
#define APP_INIT_UI                 (1UL << 1)  /* UI 硬件初始化完成 */
#define APP_INIT_LORA               (1UL << 2)  /* LoRa 硬件初始化完成 */
#define APP_INIT_SPO2               (1UL << 3)  /* 血氧传感器初始化完成 */
#define APP_INIT_ADC                (1UL << 4)  /* ADC 采样初始化完成 */
#define APP_INIT_DEBUG              (1UL << 5)  /* 调试串口初始化完成 */
#define APP_INIT_UWB                (1UL << 6)  /* UWB 初始化完成 */

#ifdef __cplusplus
}
#endif

#endif
