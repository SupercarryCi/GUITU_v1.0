#ifndef APP_EVENT_H
#define APP_EVENT_H

/* 系统事件标志位，供 g_sysEventFlags 使用。 */
#define SYS_EVT_INIT_DONE          (1UL << 0)   /* 系统初始化通过 */
#define SYS_EVT_INIT_FAILED        (1UL << 1)   /* 系统初始化失败 */
#define SYS_EVT_GYRO_UPDATED       (1UL << 2)   /* 陀螺仪数据更新 */
#define SYS_EVT_NAV_UPDATED        (1UL << 3)   /* 导航数据更新 */
#define SYS_EVT_ADC_UPDATED        (1UL << 4)   /* ADC 数据更新 */
#define SYS_EVT_SPO2_UPDATED       (1UL << 5)   /* 血氧数据更新 */
#define SYS_EVT_UI_COMMAND         (1UL << 6)   /* UI 下发命令 */
#define SYS_EVT_LORA_RX            (1UL << 7)   /* LoRa 收到完整数据包 */
#define SYS_EVT_LORA_TX_DONE       (1UL << 8)   /* LoRa 发送完成 */
#define SYS_EVT_RETURN_ACTIVE      (1UL << 9)   /* 返航模式激活 */
#define SYS_EVT_RETURN_DONE        (1UL << 10)  /* 返航完成 */
#define SYS_EVT_RETURN_FAULT       (1UL << 11)  /* 返航故障 */
#define SYS_EVT_LORA_DIO1          (1UL << 12)  /* LoRa DIO1 外部中断到达 */

#endif
