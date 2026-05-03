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
} AppCommandId_t;

typedef enum
{
    APP_LOG_DEBUG = 0,
    APP_LOG_INFO,
    APP_LOG_WARN,
    APP_LOG_ERROR
} AppLogLevel_t;

typedef struct
{
    uint8_t data[APP_GYRO_RX_MAX_LEN];
    uint16_t len;
    uint32_t tick_ms;
} GyroRxMsg_t;

typedef struct
{
    AppCommandId_t id;
    uint32_t param0;
    uint32_t param1;
    uint32_t tick_ms;
} AppCommandMsg_t;

typedef struct
{
    uint8_t port;
    uint8_t rssi_valid;
    int16_t rssi_dbm;
    uint16_t len;
    uint32_t tick_ms;
    uint8_t payload[APP_LORA_MAX_PAYLOAD_LEN];
} LoraPacketMsg_t;

typedef struct
{
    AppCommandId_t id;
    uint32_t tick_ms;
} ReturnCommandMsg_t;

typedef struct
{
    AppLogLevel_t level;
    uint32_t tick_ms;
    char text[APP_DEBUG_LOG_TEXT_LEN];
} DebugLogMsg_t;

#endif
