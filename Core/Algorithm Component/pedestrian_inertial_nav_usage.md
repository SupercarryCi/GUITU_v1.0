# pedestrian_inertial_nav

## 作用

`pedestrian_inertial_nav` 是一个独立的小模块，用来接收“已经变换到行人坐标系、并且已经去重力”的线加速度，然后积分得到：

- 行人坐标系下的速度 `velocity_ped_mps`
- 行人坐标系下的局部位移 `position_ped_m`

模块内部已经加入：

- 零速检测
- ZUPT（零速更新）
- 可选的加速度零偏补偿

## 推荐数据链路

1. 传感器输出机体系加速度、角速度和姿态
2. 调用 `PFT_BodyAccelToPedestrian()`
3. 取 `pft_output.linear_accel_ped_mps2`
4. 同时保留原始角速度 `gyro_body_rps`
5. 调用 `PIN_Update()`
6. 读取 `pin_state.position_ped_m`

## 最小调用示例

```c
#include "pedestrian_frame_transform.h"
#include "pedestrian_inertial_nav.h"

static pin_context_t g_pin_ctx;

void app_nav_init(void)
{
    pin_config_t cfg;
    PIN_DefaultConfig(&cfg);

    /* 如有需要，可在这里调整阈值 */
    cfg.accel_stationary_threshold_mps2 = 0.20f;
    cfg.gyro_stationary_threshold_rps = 0.15f;
    cfg.stationary_confirm_samples = 5U;

    PIN_Init(&g_pin_ctx, &cfg);
}

void app_nav_step(float dt_s,
                  float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float qw, float qx, float qy, float qz)
{
    pft_input_t pft_in;
    pft_output_t pft_out;
    pin_input_t pin_in;
    pin_state_t pin_state;

    pft_in.accel_body_mps2.x = ax;
    pft_in.accel_body_mps2.y = ay;
    pft_in.accel_body_mps2.z = az;
    pft_in.gyro_body_rps.x = gx;
    pft_in.gyro_body_rps.y = gy;
    pft_in.gyro_body_rps.z = gz;
    pft_in.quat.w = qw;
    pft_in.quat.x = qx;
    pft_in.quat.y = qy;
    pft_in.quat.z = qz;
    pft_in.use_quaternion = 1U;
    pft_in.remove_gravity = 1U;
    pft_in.gravity_mps2 = 9.80665f;

    if (PFT_BodyAccelToPedestrian(&pft_in, &pft_out) != PFT_OK)
    {
        return;
    }

    pin_in.dt_s = dt_s;
    pin_in.linear_accel_ped_mps2.x = pft_out.linear_accel_ped_mps2.x;
    pin_in.linear_accel_ped_mps2.y = pft_out.linear_accel_ped_mps2.y;
    pin_in.linear_accel_ped_mps2.z = pft_out.linear_accel_ped_mps2.z;
    pin_in.gyro_body_rps.x = gx;
    pin_in.gyro_body_rps.y = gy;
    pin_in.gyro_body_rps.z = gz;
    pin_in.zero_velocity = 0U;

    if (PIN_Update(&g_pin_ctx, &pin_in, &pin_state) != PIN_OK)
    {
        return;
    }

    /* 当前位置 */
    /* pin_state.position_ped_m */

    /* 当前是否检测到静止 */
    /* pin_state.zero_velocity_detected */

    /* 当前是否实际执行了 ZUPT */
    /* pin_state.zupt_applied */
}
```

## 零速检测逻辑

当前模块内部使用的是一个简单、嵌入式友好的阈值法：

- 线加速度模长小于 `accel_stationary_threshold_mps2`
- 角速度模长小于 `gyro_stationary_threshold_rps`
- 连续满足 `stationary_confirm_samples` 帧

满足后，认为当前静止。

## ZUPT 逻辑

当满足任一条件时，模块会执行 ZUPT：

- `pin_in.zero_velocity = 1`
- 内部零速检测判定静止，且配置里启用了 `enable_zupt`

当前实现方式是：

- 把前一时刻速度置零
- 把当前时刻速度置零
- 静止阶段停止位置积分漂移

## 注意事项

- 传给本模块的加速度必须是“行人坐标系 + 去重力后的线加速度”。
- 零速检测阈值需要根据你的 IMU 噪声、安装位置、采样率重新调。
- 这仍然不是完整惯导，只是“行人系线加速度积分 + 零速约束”。
- 如果需要长期稳定坐标，后面仍建议加步态约束、航向约束或 EKF 融合。
- 以当前陀螺仪数据（jy901p），陀螺仪y轴指向北，x轴指向东时，yaw为0。


