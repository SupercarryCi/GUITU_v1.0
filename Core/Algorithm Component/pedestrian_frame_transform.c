#include "pedestrian_frame_transform.h"

#include <math.h>

#define PFT_PI_F 3.14159265358979323846f

/*
 * 实现说明
 *
 * 这个文件刻意保持为“小而独立”的数学模块，方便直接拷贝到别的 MCU 工程。
 *
 * 本模块负责：
 *   - 接收机体坐标系加速度与姿态
 *   - 构建 body->pedestrian 的方向余弦矩阵 DCM
 *   - 将加速度旋转到行人坐标系
 *   - 按需在行人坐标系 Z 轴去重力
 *
 * 本模块不负责：
 *   - 传感器标定
 *   - 低通/高通滤波
 *   - 零偏估计
 *   - 运动检测 / 步态检测
 *   - 惯导速度、位置积分
 *
 * 关键前提：
 *   1. 输入四元数/欧拉角描述的姿态，必须与输入加速度属于同一个机体坐标系。
 *   2. 四元数格式固定为标量在前：[w, x, y, z]。
 *   3. 这里有意保留了当前工程已有的 body->navigation DCM 排列方式，
 *      目的是让该模块能直接复用你原来的姿态到导航坐标系变换逻辑。
 *   4. 去重力只在行人坐标系 Z 轴上进行，所以姿态精度会直接决定线加速度精度。
 */

static float pft_deg_to_rad(float deg)
{
    return deg * (PFT_PI_F / 180.0f);
}

static void pft_quat_from_euler(const pft_euler_deg_t *euler_deg, pft_quatf_t *quat)
{
    /* 将 ZYX 欧拉角转换为四元数。
     * 这里只是备用路径。
     * 如果上层滤波器已经直接输出四元数，优先直接使用四元数，
     * 因为这样可以避免再次做姿态表示转换。
     */
    const float half_roll = 0.5f * pft_deg_to_rad(euler_deg->roll_deg);
    const float half_pitch = 0.5f * pft_deg_to_rad(euler_deg->pitch_deg);
    const float half_yaw = 0.5f * pft_deg_to_rad(euler_deg->yaw_deg);

    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);

    quat->w = cr * cp * cy + sr * sp * sy;
    quat->x = sr * cp * cy - cr * sp * sy;
    quat->y = cr * sp * cy + sr * cp * sy;
    quat->z = cr * cp * sy - sr * sp * cy;
}

static pft_status_t pft_normalize_quat(pft_quatf_t *quat)
{
    /* 四元数单位化有两个目的：
     * 1. 防止错误模长破坏后续 DCM 计算
     * 2. 即使调用者忘了单位化，这里也能做一次保护
     */
    const float norm_sq = quat->w * quat->w +
                          quat->x * quat->x +
                          quat->y * quat->y +
                          quat->z * quat->z;

    if (norm_sq < 1e-12f)
    {
        return PFT_ERR_QUAT;
    }

    {
        const float inv_norm = 1.0f / sqrtf(norm_sq);
        quat->w *= inv_norm;
        quat->x *= inv_norm;
        quat->y *= inv_norm;
        quat->z *= inv_norm;
    }

    return PFT_OK;
}

static void pft_build_dcm_body_to_ped(const pft_quatf_t *quat, float dcm[3][3])
{
    /* 按当前工程已有的 INS 坐标变换方式构建 DCM。
     *
     * 这里最需要注意：
     * 由四元数推导出来的 DCM，行列排布与旋转方向，
     * 会受到“四元数定义方式”和“你想做 body->nav 还是 nav->body 变换”的影响。
     *
     * 这里故意保留当前工程原来使用的排列方式：
     *
     *   ped_acc = DCM_body_to_ped * body_acc
     *
     * 如果你把这个文件移植到另一个工程，而那个工程的四元数定义不同，
     * 这里就是第一处必须重点复查的地方。
     */
    const float q0 = quat->w;
    const float q1 = quat->x;
    const float q2 = quat->y;
    const float q3 = quat->z;

    const float c11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    const float c12 = 2.0f * (q1 * q2 + q0 * q3);
    const float c13 = 2.0f * (q1 * q3 - q0 * q2);
    const float c21 = 2.0f * (q1 * q2 - q0 * q3);
    const float c22 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    const float c23 = 2.0f * (q2 * q3 + q0 * q1);
    const float c31 = 2.0f * (q1 * q3 + q0 * q2);
    const float c32 = 2.0f * (q2 * q3 - q0 * q1);
    const float c33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    /* 行表示行人坐标轴，列表示机体坐标轴。 */
    dcm[0][0] = c11;
    dcm[0][1] = c21;
    dcm[0][2] = c31;

    dcm[1][0] = c12;
    dcm[1][1] = c22;
    dcm[1][2] = c32;

    dcm[2][0] = c13;
    dcm[2][1] = c23;
    dcm[2][2] = c33;
}

pft_status_t PFT_BodyAccelToPedestrian(const pft_input_t *input, pft_output_t *output)
{
    pft_quatf_t quat;
    float gravity;

    /* 最基本的参数检查。 */
    if ((input == 0) || (output == 0))
    {
        return PFT_ERR_NULL;
    }

    /* 优先使用四元数输入。
     * 原因是：
     * 1. 可以避免欧拉角奇异点问题
     * 2. 更符合现代姿态滤波器的直接输出形式
     */
    if (input->use_quaternion != 0U)
    {
        quat = input->quat;
    }
    else
    {
        pft_quat_from_euler(&input->angle_deg, &quat);
    }

    /* 即使调用者认为四元数已经有效，这里也再次单位化。
     * 因为非单位四元数会直接导致旋转矩阵畸变。
     */
    if (pft_normalize_quat(&quat) != PFT_OK)
    {
        return PFT_ERR_QUAT;
    }

    /* 构建本次计算实际使用的 DCM。
     * 同时把 DCM 输出给调用者，方便上层在排查轴向/符号问题时直接查看，
     * 不需要再重复计算一次矩阵。
     */
    pft_build_dcm_body_to_ped(&quat, output->dcm_body_to_ped);

    /* 将机体坐标系加速度旋转到行人坐标系。
     *
     * 注意：
     * output->accel_ped_mps2 里仍然包含重力。
     * 因为加速度计实际测到的是“比力 + 当前姿态下看到的重力投影”的组合，
     * 不是天然就已经去重力的线加速度。
     */
    output->accel_ped_mps2.x =
        output->dcm_body_to_ped[0][0] * input->accel_body_mps2.x +
        output->dcm_body_to_ped[0][1] * input->accel_body_mps2.y +
        output->dcm_body_to_ped[0][2] * input->accel_body_mps2.z;

    output->accel_ped_mps2.y =
        output->dcm_body_to_ped[1][0] * input->accel_body_mps2.x +
        output->dcm_body_to_ped[1][1] * input->accel_body_mps2.y +
        output->dcm_body_to_ped[1][2] * input->accel_body_mps2.z;

    output->accel_ped_mps2.z =
        output->dcm_body_to_ped[2][0] * input->accel_body_mps2.x +
        output->dcm_body_to_ped[2][1] * input->accel_body_mps2.y +
        output->dcm_body_to_ped[2][2] * input->accel_body_mps2.z;

    /* 先把“已完成坐标变换、但仍含重力”的结果拷贝出来，
     * 后面如有需要，再继续去重力。
     */
    output->linear_accel_ped_mps2 = output->accel_ped_mps2;

    /* 允许调用者传入本地重力值。
     * 如果未传或传入非法值，则使用默认标准值，
     * 这对大多数嵌入式场景已经够用。
     */
    gravity = input->gravity_mps2;
    if (gravity <= 0.0f)
    {
        gravity = PFT_DEFAULT_GRAVITY_MPS2;
    }

    if (input->remove_gravity != 0U)
    {
        /* 在本模块定义下，变换到行人坐标系后，重力默认沿 +Z 方向表现，
         * 因此只在 Z 轴减去重力。
         *
         * 如果你的工程把“向上/向下”定义得与本文件相反，
         * 这里就是最应该首先检查的符号位置。
         */
        output->linear_accel_ped_mps2.z -= gravity;
    }

    /* 到这里为止：
     * - accel_ped_mps2        ：已变换到行人坐标系，但仍保留重力
     * - linear_accel_ped_mps2 ：已变换到行人坐标系，并完成去重力
    */
    return PFT_OK;
}

pft_status_t PFT_StepDistanceToPedDelta(float step_distance_m,
                                        float yaw_deg,
                                        pft_vec3f_t *delta_ped_m)
{
    float yaw_rad;

    if (delta_ped_m == 0)
    {
        return PFT_ERR_NULL;
    }

    yaw_rad = pft_deg_to_rad(yaw_deg);

    /*
     * 当前工程约定：X 指东，Y 指北，yaw=0 时前向沿 +Y。
     * 这里按航向角从北开始、向东为正进行水平投影。
     */
    delta_ped_m->x = step_distance_m * sinf(yaw_rad);
    delta_ped_m->y = step_distance_m * cosf(yaw_rad);
    delta_ped_m->z = 0.0f;

    return PFT_OK;
}
