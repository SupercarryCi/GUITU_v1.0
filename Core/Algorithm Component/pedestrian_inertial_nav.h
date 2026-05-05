#ifndef PEDESTRIAN_INERTIAL_NAV_H
#define PEDESTRIAN_INERTIAL_NAV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * pedestrian_inertial_nav
 *
 * 作用：
 *   对“已经转换到行人坐标系、并且已经去重力”的线加速度做积分，
 *   得到行人坐标系下的局部速度和局部位移。
 *
 * 推荐链路：
 *   机体系加速度 + 姿态
 *     -> pedestrian_frame_transform
 *     -> linear_accel_ped_mps2
 *     -> pedestrian_inertial_nav
 *     -> 行人坐标系局部位移 / 速度
 *
 * 坐标约定：
 *   1. 输入加速度必须已经在行人坐标系下。
 *   2. 输入加速度必须已经去重力，推荐直接使用
 *      pft_output.linear_accel_ped_mps2。
 *   3. 输出位置和速度仍然位于同一个行人坐标系：
 *      - X/Y：水平面
 *      - Z  ：竖直方向
 *
 * 限制说明：
 *   1. 本模块不估计姿态、航向、零偏。
 *   2. 本模块不做滤波、步态检测、完整组合导航。
 *   3. 纯加速度积分漂移很快，因此这里额外加入了
 *      “零速检测 + ZUPT”来抑制静止阶段的速度漂移。
 *   4. 输出是局部笛卡尔位移，不是经纬高。
 */

typedef struct
{
    float x;
    float y;
    float z;
} pin_vec3f_t;

typedef struct
{
    /* 已累计运行时间 */
    float elapsed_time_s;

    /* 行人坐标系下的位置，单位：m */
    pin_vec3f_t position_ped_m;

    /* 行人坐标系下的速度，单位：m/s */
    pin_vec3f_t velocity_ped_mps;

    /* 最近一次检测结果，1 表示检测为静止 */
    uint8_t zero_velocity_detected;

    /* 最近一次是否实际执行了 ZUPT */
    uint8_t zupt_applied;

    /* 便于上层调参观察的模长信息 */
    float last_accel_norm_mps2;
    float last_gyro_norm_rps;
} pin_state_t;

typedef struct
{
    /* 初始位置 */
    pin_vec3f_t initial_position_ped_m;

    /* 初始速度 */
    pin_vec3f_t initial_velocity_ped_mps;

    /* 是否启用加速度零偏补偿 */
    uint8_t enable_acc_bias_correction;

    /* 行人坐标系下的加速度零偏 */
    pin_vec3f_t acc_bias_ped_mps2;

    /* 是否启用零速检测 */
    uint8_t enable_zero_velocity_detector;

    /* 是否在检测到静止时自动执行 ZUPT */
    uint8_t enable_zupt;

    /* 线加速度静止阈值，单位：m/s^2 */
    float accel_stationary_threshold_mps2;

    /* 角速度静止阈值，单位：rad/s */
    float gyro_stationary_threshold_rps;

    /* 连续满足静止条件多少帧后，才判定为静止 */
    uint16_t stationary_confirm_samples;
} pin_config_t;

typedef struct
{
    /* 采样周期 */
    float dt_s;

    /* 行人坐标系下、去重力后的线加速度 */
    pin_vec3f_t linear_accel_ped_mps2;

    /* 当前帧角速度，建议直接传传感器机体系角速度。
     * 对零速检测来说，只使用模长，因此机体系/行人系都可以。 */
    pin_vec3f_t gyro_body_rps;

    /* 外部强制零速。
     * 当该标志为 1 时，不再依赖内部检测，直接执行 ZUPT。 */
    uint8_t zero_velocity;
} pin_input_t;

typedef struct
{
    uint8_t initialized;
    uint8_t has_prev_accel;

    /* 已连续满足静止条件的帧数 */
    uint16_t stationary_sample_count;

    pin_vec3f_t prev_linear_accel_ped_mps2;
    pin_vec3f_t acc_bias_ped_mps2;

    uint8_t enable_acc_bias_correction;
    uint8_t enable_zero_velocity_detector;
    uint8_t enable_zupt;

    float accel_stationary_threshold_mps2;
    float gyro_stationary_threshold_rps;
    uint16_t stationary_confirm_samples;

    pin_state_t state;
} pin_context_t;

typedef enum
{
    PIN_OK = 0,
    PIN_ERR_NULL = -1,
    PIN_ERR_NOT_INITIALIZED = -2,
    PIN_ERR_DT = -3
} pin_status_t;

/* 填充一套可直接使用的默认参数。 */
void PIN_DefaultConfig(pin_config_t *config);

/* 初始化 / 复位模块。 */
void PIN_Init(pin_context_t *ctx, const pin_config_t *config);
void PIN_Reset(pin_context_t *ctx, const pin_config_t *config);

/* 动态设置加速度零偏。 */
void PIN_SetAccBias(pin_context_t *ctx, const pin_vec3f_t *acc_bias_ped_mps2, uint8_t enable);

/*
 * 更新局部速度和位置。
 *
 * 输入：
 *   input->linear_accel_ped_mps2 : 行人坐标系、去重力后的线加速度
 *   input->gyro_body_rps         : 用于零速检测的角速度
 *   input->dt_s                  : 当前采样周期
 *   input->zero_velocity         : 外部强制零速标志
 *
 * 输出：
 *   out_state->position_ped_m    : 行人坐标系局部位移
 *   out_state->velocity_ped_mps  : 行人坐标系局部速度
 *   out_state->zero_velocity_detected / zupt_applied :
 *                                 便于上层调试和观察检测状态
 */
pin_status_t PIN_Update(pin_context_t *ctx, const pin_input_t *input, pin_state_t *out_state);

/* 查询模块状态。 */
uint8_t PIN_IsReady(const pin_context_t *ctx);
const pin_state_t *PIN_GetState(const pin_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif
