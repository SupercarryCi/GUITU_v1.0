#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

#include "app_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 全局状态层：                 暂时保留这样，后面都要重改
 * 任务之间不直接共享裸全局变量，而是通过 App_StateSet/Get 访问。
 * app_state.c 内部用互斥锁保护一致性。
 */

typedef struct
{
    uint32_t init_done_mask;     /* 初始化完成掩码，记录哪些外设已就绪 */
    int32_t  init_result;        /* 初始化结果：0 成功，负数为失败步序号 */
    uint32_t fault_count;        /* 累计故障次数 */
    uint32_t last_fault_code;    /* 最近一次故障码 */        //真的有用吗？
} AppSystemState_t;

typedef struct
{
    float accel_mps2[3];
    float gyro_rad_s[3];
    float euler_rad[3];
} GyroFrame_t;

typedef struct
{
    uint16_t raw_len;              /* 当前帧原始数据长度 */
    uint32_t rx_count;             /* 累计接收帧数 */
    uint32_t drop_count;           /* 队列满导致丢弃的帧数 */
    uint32_t parse_error_count;    /* 解析失败次数 */
    int32_t last_parse_result;     /* 最近一次协议解析结果 */
    GyroFrame_t frame;             /* 解析后的传感器数据结构体 */
    uint8_t raw[APP_GYRO_RX_MAX_LEN];  /* 原始接收缓冲区 */
} GyroState_t;

typedef struct
{
    float position_m[3];
    float velocity_mps[3];
    float attitude_rad[3];
} NavData_t;

/* ---------- INS 导航解算状态 ---------- */
typedef struct
{
    uint32_t update_count;         /* 累计解算更新次数 */
    NavData_t data;                /* 导航状态占位，具体算法由业务层完善 */
} NavState_t;

/* ---------- ADC 采样状态 ---------- */
typedef struct
{
    uint16_t raw[APP_ADC_CHANNEL_COUNT];        /* 各通道原始采样值 */
    uint16_t voltage_mv[APP_ADC_CHANNEL_COUNT]; /* 换算后的电压值 (mV) */
    uint32_t update_count;                      /* 累计采样次数 */
    uint32_t error_count;                       /* 采样错误次数 */
} AdcState_t;

/* ---------- 血氧传感器状态 ---------- */
typedef struct
{
    uint8_t  spo2_percent;         /* 血氧饱和度 (%) */
    uint16_t heart_rate_bpm;       /* 心率 (次/分钟) */
    uint16_t perfusion_permille;   /* 灌注指数 (‰) */
    uint32_t update_count;         /* 累计更新次数 */
    uint32_t error_count;          /* 读取错误次数 */
} Spo2State_t;

/* ---------- UI 交互状态 ---------- */
typedef struct
{
    AppCommandMsg_t last_command;  /* 最近一次收到的 UI 命令 */
    uint32_t command_count;        /* 累计收到命令次数 */
    uint32_t render_count;         /* 累计屏幕刷新次数 */
    uint32_t touch_count;          /* 累计触摸事件次数 */
} UiState_t;

/* ---------- LoRa 通信状态 ---------- */
typedef struct
{
    uint32_t tx_count;             /* 累计发送包数 */
    uint32_t rx_count;             /* 累计接收包数 */
    uint32_t error_count;          /* 通信错误次数 */
    LoraPacketMsg_t last_rx;       /* 最近一次接收到的数据包 */
} LoraState_t;

/* ---------- 返航模式 ---------- */
typedef enum
{
    RETURN_MODE_IDLE = 0,          /* 空闲：未启动返航 */
    RETURN_MODE_RUNNING,           /* 运行中：正在执行返航路径 */
    RETURN_MODE_PAUSED,            /* 暂停：返航被用户暂停 */
    RETURN_MODE_DONE,              /* 完成：已成功抵达目标 */
    RETURN_MODE_FAULT              /* 故障：返航过程中出错 */
} ReturnMode_t;

/* ---------- 返航运行状态 ---------- */
typedef struct
{
    ReturnMode_t mode;             /* 当前返航模式 */
    uint32_t path_points;          /* 已记录的路径点数量 */
    uint32_t target_index;         /* 当前目标点索引 */
    uint32_t step_count;           /* 累计执行步数 */
    uint32_t error_count;          /* 返航过程错误次数 */
} ReturnState_t;

/* ---------- 全局状态快照 ---------- */
typedef struct
{
    AppSystemState_t system;       /* 系统状态 */
    GyroState_t      gyro;         /* 陀螺仪状态 */
    NavState_t       nav;          /* 导航状态 */
    AdcState_t       adc;          /* ADC 状态 */
    Spo2State_t      spo2;         /* 血氧状态 */
    UiState_t        ui;           /* UI 状态 */
    LoraState_t      lora;         /* LoRa 状态 */
    ReturnState_t    return_home;  /* 返航状态 */
} AppSnapshot_t;

/* ---------- 状态管理接口 ---------- */

/* 初始化全局状态层（创建互斥锁），成功返回 0 */
int32_t App_StateInit(void);

/* 系统状态读写 */
void App_StateSetSystem(const AppSystemState_t *state);
void App_StateGetSystem(AppSystemState_t *state);

/* 记录初始化结果（done_mask + result），供故障排查 */
void App_StateSetInitResult(uint32_t done_mask, int32_t result);

/* 记录一次故障，fault_code 按工程规范编码 */
void App_StateAddFault(uint32_t fault_code);

/* 陀螺仪状态读写 */
void App_StateSetGyro(const GyroState_t *state);
void App_StateGetGyro(GyroState_t *state);

/* 导航状态读写 */
void App_StateSetNav(const NavState_t *state);
void App_StateGetNav(NavState_t *state);

/* ADC 状态读写 */
void App_StateSetAdc(const AdcState_t *state);
void App_StateGetAdc(AdcState_t *state);

/* 血氧状态读写 */
void App_StateSetSpo2(const Spo2State_t *state);
void App_StateGetSpo2(Spo2State_t *state);

/* UI 状态读写 */
void App_StateSetUi(const UiState_t *state);
void App_StateGetUi(UiState_t *state);

/* LoRa 状态读写 */
void App_StateSetLora(const LoraState_t *state);
void App_StateGetLora(LoraState_t *state);

/* 返航状态读写 */
void App_StateSetReturn(const ReturnState_t *state);
void App_StateGetReturn(ReturnState_t *state);

/* 一次性读取所有模块状态到快照结构体中 */
void App_StateGetSnapshot(AppSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
