#ifndef APP_MSG_H
#define APP_MSG_H

#include <stdint.h>

#include "app_config.h"

typedef enum
{
    APP_CMD_NONE = 0,
    APP_CMD_RETURN_HOME_START,
    APP_CMD_RETURN_HOME_STOP,
    APP_CMD_RETURN_HOME_PAUSE,
    APP_CMD_RETURN_HOME_RESUME,
    APP_CMD_MARK_PATH_POINT,
    APP_CMD_LORA_SEND,
    APP_CMD_USER_BASE = 100
} AppCommandId_t;//需要大改，先放着

typedef struct
{
    /* 一帧原始陀螺仪数据 */
    uint8_t data[APP_GYRO_RX_MAX_LEN];
    uint16_t len;
} GyroRxMsg_t;

typedef struct
{
    /* UI 触摸输出的统一命令，ControlTask 负责分发给具体任务 */
    AppCommandId_t id;
    uint32_t param0;
    uint32_t param1;
} AppCommandMsg_t;

typedef struct
{
    /* LoRa 预留数据包结构，后续启用射频驱动后直接复用 */
    uint8_t port;
    uint8_t rssi_valid;
    int16_t rssi_dbm;
    uint16_t len;
    uint8_t payload[APP_LORA_MAX_PAYLOAD_LEN];
} LoraPacketMsg_t;

typedef struct
{
    /*返航命令，归途特色*/
    AppCommandId_t id;
} ReturnCommandMsg_t;

typedef struct
{
    char text[APP_DEBUG_LOG_TEXT_LEN];//经典串口帝巴戈
} DebugLogMsg_t;

#endif
