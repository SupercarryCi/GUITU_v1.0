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
#define APP_NRF24_SPI_HANDLE        hspi4       /* NRF24L01 与 LoRa 共用 SPI4 */

#define APP_SPO2_I2C_HANDLE         hi2c1       /* 血氧传感器 I2C 接口 */

#define APP_ADC_HANDLE              hadc1       /* 通用 ADC (电池/电压采集) */

#define APP_GYRO_RX_MAX_LEN         96U         /* 单次 DMA 投队列的最大字节数 */
#define APP_GYRO_DMA_BUFFER_SIZE    APP_GYRO_RX_MAX_LEN   /* DMA 接收缓冲区大小 */
#define APP_GYRO_QUEUE_DEPTH        16U         /* 陀螺仪接收队列深度  */

#define APP_UI_CMD_QUEUE_DEPTH      8U          /* 用户操作命令队列深度 */
#define APP_NAV_DELTA_QUEUE_DEPTH   16U         /* INS/PDR 位移增量队列深度 */
#define APP_LORA_QUEUE_DEPTH        4U          /* LoRa 收发队列深度 */
#define APP_RETURN_QUEUE_DEPTH      4U          /* 返航控制命令队列深度 */
#define APP_DEBUG_LOG_QUEUE_DEPTH   16U         /* 调试日志队列深度 */

#define APP_LORA_MAX_PAYLOAD_LEN    128U        /* LoRa 单包最大负载字节数 */
#define APP_DEBUG_LOG_TEXT_LEN      32U         /* 单条调试日志最大文本长度 */

#define APP_ADC_CHANNEL_COUNT       2U          /* 全局状态保留两路 ADC 数据位置 */
#define APP_ADC_ACTIVE_CHANNEL_COUNT 1U          /* ADC.2/PC4 改作 NRF24 CE，仅采集 ADC.1 */
#define APP_ADC_SAMPLE_PERIOD_MS    100U        /* ADC 采样周期 (ms) */
#define APP_ADC_STARTUP_DELAY_MS    10000U      /* 系统初始化完成后等待传感器稳定 */
#define APP_ADC_SAMPLE_TIMEOUT_MS   20U         /* 单次 ADC 采样超时时间 (ms) */
#define APP_ADC_VREF_MV             3300U       /* ADC 参考电压 (mV)，用于换算实际电压 */
#define APP_ADC_FULL_SCALE          65535U      /* ADC 满量程值 (16bit) */

#define APP_UI_PERIOD_MS            500U        /* UI 显示刷新周期 (2Hz) */
#define APP_UI_TOUCH_PERIOD_MS      50U         /* 触摸轮询周期 (20Hz) */
#define APP_SPO2_PERIOD_MS          100U        /* 血氧 FIFO 读取周期 (10Hz)，避免 100Hz 采样溢出 */

/* 佩戴反射光强阈值：0 表示尚未标定，此时只输出串口测试数据，不确认佩戴。 */
#define APP_SPO2_WEAR_IR_DC_MIN     50000U
#define APP_SPO2_WEAR_RED_DC_MIN    30000U

#define APP_LORA_PERIOD_MS          1000U       /* LoRa 导航位置发送/收发轮询周期 (1Hz) */
#define APP_NRF24_POLL_PERIOD_MS    10U         /* 未连接 IRQ，需及时排空三端高频接收 FIFO */
#define APP_RETURN_PERIOD_MS        100U        /* 返航控制周期 (10Hz) */

/* PDR航向源测试开关：切换航向，不改变二端本机IMU的判步输入。 */
#define APP_HEADING_SOURCE_LOCAL_IMU     0U
#define APP_HEADING_SOURCE_BEACON_IMU    1U
#define APP_HEADING_SOURCE               APP_HEADING_SOURCE_BEACON_IMU
#define APP_BEACON_HEADING_TIMEOUT_MS    200U
#define APP_BEACON_HEADING_OFFSET_DEG    0.0f

/* 气体报警阈值，传感器标定后只需要调整这里。 */
#define APP_GAS_NGAS_ALARM_THRESHOLD_PPM 50000.0f
#define APP_GAS_LPG_ALARM_THRESHOLD_PPM  50000.0f

/* 蜂鸣器为 PE8 GPIO 输出，默认高电平响。若硬件为低电平响，在这里对调。 */
#define APP_BUZZER_ACTIVE_STATE          GPIO_PIN_SET
#define APP_BUZZER_INACTIVE_STATE        GPIO_PIN_RESET
#define APP_BUZZER_TIMER_PERIOD_MS       50U

/* 联调阶段保留无线任务运行NRF24，暂不初始化SX126x及LoRa收发。 */
#define APP_LORA_ENABLE_DEFAULT     1U
#define APP_LORA_RF_ENABLE_DEFAULT  0U
#define APP_NRF24_ENABLE_DEFAULT    1U

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
