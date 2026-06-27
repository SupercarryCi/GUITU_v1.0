#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

#include "app_msg.h"
#include "wit_imu.h"

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
    /* --- 原始量（WIT陀螺仪的modbus协议 int16_t）--- */
    WitImuVector3 acc_raw;       /* WIT 0x51 原始加速度：raw / 32768 * 16g    [x,y,z] */
    WitImuVector3 gyro_raw;      /* WIT 0x52 原始角速度：raw / 32768 * 2000°/s [x,y,z] */
    WitImuVector3 angle_raw;     /* WIT 0x53 原始角度：raw / 32768 * 180°      [x,y,z] */
    WitImuQuat    quat_raw;      /* WIT 0x59 原始四元数：raw / 32768           [q0,q1,q2,q3] */
    int16_t       temp_raw;      /* WIT 原始温度：raw / 100 °C */

    /* --- 转换后的物理量（float）--- */
    float accel_mps2[3];         /* [0]=X轴, [1]=Y轴, [2]=Z轴  加速度 单位 m/s² */
    float gyro_rad_s[3];         /* [0]=Roll率, [1]=Pitch率, [2]=Yaw率  角速度 单位 rad/s */
    float angle_deg[3];          /* [0]=Roll横滚, [1]=Pitch俯仰, [2]=Yaw航向  单位 ° */
    float quat[4];               /* [0]=q0/w(实部), [1]=q1/x, [2]=q2/y, [3]=q3/z */
    float temp_deg_c;            /* 温度 单位 °C */
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
    float YAW_deg;
} NavData_t;

typedef struct
{
    uint32_t update_count;         /* 累计解算更新次数 */
    NavData_t data;                /* 导航状态占位，具体算法由业务层完善 */
} NavState_t;

typedef struct
{
    uint16_t raw[APP_ADC_CHANNEL_COUNT];                    /* 各通道原始采样值 */
    uint16_t voltage_mv[APP_ADC_CHANNEL_COUNT];             /* 换算后的电压值 (mV) */
    float gas_concentration[APP_ADC_CHANNEL_COUNT];         /* 根据电压换算的气体浓度*/
    uint32_t update_count;                                  /* 累计采样次数 */
    uint32_t error_count;                                   /* 采样错误次数 */
} AdcState_t;

typedef struct
{
    uint8_t  spo2_percent;         /* 血氧饱和度 (%) */
    uint8_t  spo2_valid;           /* 已获得可显示的有效血氧值 */
    uint16_t heart_rate_bpm;       /* 心率 (次/分钟) */
    uint8_t  heart_rate_valid;     /* 已获得可显示的有效心率值 */
    uint16_t perfusion_permille;   /* 灌注指数 (‰) */
    uint32_t update_count;         /* 累计更新次数 */
    uint32_t error_count;          /* 读取错误次数 */
} Spo2State_t;

typedef struct
{
    AppCommandMsg_t last_command;  /* 最近一次收到的 UI 命令 */
    uint32_t command_count;        /* 累计收到命令次数 */
    uint32_t render_count;         /* 累计屏幕刷新次数 */
    uint32_t touch_count;          /* 累计触摸事件次数 */
} UiState_t;

typedef struct
{
    uint32_t tx_count;             /* 累计发送包数 */
    uint32_t rx_count;             /* 累计接收包数 */
    uint32_t error_count;          /* 通信错误次数 */
    LoraPacketMsg_t last_rx;       /* 最近一次接收到的数据包 */
} LoraState_t;

typedef enum
{
    RETURN_MODE_IDLE = 0,          /* 空闲：未启动返航 */
    RETURN_MODE_RUNNING            /* 运行中：正在执行返航路径 */
} ReturnMode_t;

typedef struct
{
    ReturnMode_t mode;             /* 当前返航模式 */
    uint32_t path_points;          /* 已记录的路径点数量 */
    uint32_t target_index;         /* 当前目标点索引 */
    uint32_t step_count;           /* 累计执行步数 */
} ReturnState_t;

typedef struct
{
    uint8_t valid;                 /* 引导输出是否有效 */
    uint8_t return_mode;           /* Dijkstra 算法是否处于返航模式 */
    uint8_t route_valid;           /* 是否已生成可用返航路径 */
    uint8_t arrived_home;          /* 是否到达 home 节点 */
    int32_t distance_to_next_mm;   /* 到下一个关键点的距离 */
    int16_t bearing_to_next_cdeg;  /* 到下一个关键点的绝对方向 */
    int16_t relative_bearing_cdeg; /* 相对当前航向的方向 */
    uint16_t next_route_index;     /* 当前目标路线点索引 */
    int8_t turn_after_next;        /* 到达目标点后的转向：-1左，0无，1右 */
} ReturnGuideState_t;

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
    ReturnGuideState_t return_guide; /* 返航引导缓存 */
} AppSnapshot_t;//系统快照

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

/* 返航引导缓存读写 */
void App_StateSetReturnGuide(const ReturnGuideState_t *state);
void App_StateGetReturnGuide(ReturnGuideState_t *state);

/* 一次性读取所有模块状态到快照结构体中 */
void App_StateGetSnapshot(AppSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
