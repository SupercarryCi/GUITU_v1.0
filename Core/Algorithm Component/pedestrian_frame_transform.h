#ifndef PEDESTRIAN_FRAME_TRANSFORM_H
#define PEDESTRIAN_FRAME_TRANSFORM_H

#include <stdint.h>

/*
 * pedestrian_frame_transform
 *
 * 作用：
 *   将“机体坐标系加速度 -> 行人坐标系加速度”的变换封装成一个小而独立、
 *   便于移植的模块。
 *
 * 本模块采用的坐标约定：
 *   1. 输入加速度位于 IMU/机体坐标系。
 *   2. 输出行人坐标系由输入姿态构造得到，是一个与地面方向对齐的坐标系：
 *      - X/Y：水平面
 *      - Z：竖直方向
 *   3. 默认认为 yaw 是绝对航向角。如果 yaw 已经融合了磁力计，那么输出
 *      的水平坐标轴也是绝对方向下的水平坐标轴。
 *
 * 需要注意：
 *   1. 本模块只做坐标变换。
 *      不负责滤波、零偏估计、死区、积分、零速检测，也不负责完整 INS 状态推进。
 *   2. 这里保留了当前工程里 body->navigation 的 DCM 排列方式，便于与你
 *      现有代码直接对接。但这也意味着：四元数定义、轴方向定义、输入加速度定义
 *      必须彼此一致，否则结果会错。
 *   3. "linear_accel_ped_mps2" 只是“变换到行人坐标系后，再在 Z 轴减去重力”
 *      的结果。若姿态本身不准，去重力后的线加速度也一定不准。
 *   4. 结构体里的 gyro_body_rps 只是为了接口完整性预留，本函数本身不使用它。
 *   5. 即使上层已经给出单位化四元数，这里也会再次单位化一次，作为保护。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define PFT_DEFAULT_GRAVITY_MPS2 9.80665f

typedef struct
{
    /* 机体坐标系加速度，单位：m/s^2。 */
    float x;
    float y;
    float z;
} pft_vec3f_t;

typedef struct
{
    /* 四元数采用标量在前格式：q = [w, x, y, z]。 */
    float w;
    float x;
    float y;
    float z;
} pft_quatf_t;

typedef struct
{
    /* 欧拉角，单位：度。 */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} pft_euler_deg_t;

typedef struct
{
    /* 必需输入：机体坐标系加速度。 */
    pft_vec3f_t accel_body_mps2;

    /* 为上层调用者保留的输入。本变换函数本身不使用。 */
    pft_vec3f_t gyro_body_rps;

    /* 欧拉角输入，仅在 use_quaternion == 0 时使用。 */
    pft_euler_deg_t angle_deg;

    /* 四元数输入。当 use_quaternion != 0 时优先使用。 */
    pft_quatf_t quat;

    /* 1：以 quat 作为主姿态输入。0：使用 roll/pitch/yaw。 */
    uint8_t use_quaternion;

    /* 1：在行人坐标系 Z 轴去重力，并输出 linear_accel_ped_mps2。 */
    uint8_t remove_gravity;

    /* 重力大小。若 <= 0，则回退到默认值 9.80665。 */
    float gravity_mps2;
} pft_input_t;

typedef struct
{
    /* 行人坐标系下的加速度，仍然包含重力。 */
    pft_vec3f_t accel_ped_mps2;

    /* 行人坐标系下、在 Z 轴去重力之后的线加速度。 */
    pft_vec3f_t linear_accel_ped_mps2;

    /* 本次计算实际使用的 DCM。行表示行人坐标轴，列表示机体坐标轴。 */
    float dcm_body_to_ped[3][3];
} pft_output_t;

typedef enum
{
    PFT_OK = 0,
    PFT_ERR_NULL = -1,
    PFT_ERR_QUAT = -2
} pft_status_t;

/*
 * 将机体坐标系加速度转换为行人坐标系加速度。
 *
 * 输入：
 *   input->accel_body_mps2 : 机体坐标系加速度
 *   input->quat            : 首选姿态来源
 *   input->angle_deg       : 备用姿态来源
 *
 * 输出：
 *   output->accel_ped_mps2        : 已完成坐标变换、但仍保留重力的加速度
 *   output->linear_accel_ped_mps2 : 已完成坐标变换，并在行人坐标系 Z 轴去重力
 *   output->dcm_body_to_ped       : 本次变换实际使用的 DCM
 *
 * 返回值：
 *   PFT_OK       : 成功
 *   PFT_ERR_NULL : 输入或输出指针为空
 *   PFT_ERR_QUAT : 四元数模长非法
 *
 * 使用/移植建议：
 *   1. 如果上层 IMU 解算已经直接输出四元数，优先用四元数。
 *   2. roll/pitch/yaw 与 quaternion 必须来自同一套姿态解算，不能混用。
 *   3. 如果 yaw 已经由磁力计修正为绝对航向，则输出的水平坐标系也是绝对方向下的。
 *   4. 如果你只关心水平/竖直分离后的线加速度，把 remove_gravity 设为 1，
 *      直接使用 linear_accel_ped_mps2。
 *   5. 如果你的 IMU 原始轴定义与当前工程不同，请先在调用前完成轴重映射。
 */
pft_status_t PFT_BodyAccelToPedestrian(const pft_input_t *input, pft_output_t *output);

/*
 * 将 PDR 单步前向距离投影成行人/导航水平坐标下的位移增量。
 * 当前工程约定：X 指东，Y 指北，yaw=0 时前向沿 +Y。
 *
 * 输入：
 *   step_distance_m : 本步前向距离，单位 m
 *   yaw_deg         : 当前航向角，单位 deg
 *
 * 输出：
 *   delta_ped_m.x/y : 水平位移增量，单位 m
 *   delta_ped_m.z   : 固定为 0，竖直位移由 INS/气压计等模块处理
 */
pft_status_t PFT_StepDistanceToPedDelta(float step_distance_m,
                                        float yaw_deg,
                                        pft_vec3f_t *delta_ped_m);

#ifdef __cplusplus
}
#endif

#endif
