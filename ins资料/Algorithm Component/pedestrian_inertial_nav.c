#include "pedestrian_inertial_nav.h"

#include <math.h>
#include <string.h>

/* 三维向量加法 */
static pin_vec3f_t pin_vec3_add(pin_vec3f_t a, pin_vec3f_t b)
{
    pin_vec3f_t out = { a.x + b.x, a.y + b.y, a.z + b.z };
    return out;
}

/* 三维向量减法 */
static pin_vec3f_t pin_vec3_sub(pin_vec3f_t a, pin_vec3f_t b)
{
    pin_vec3f_t out = { a.x - b.x, a.y - b.y, a.z - b.z };
    return out;
}

/* 三维向量数乘 */
static pin_vec3f_t pin_vec3_scale(pin_vec3f_t a, float s)
{
    pin_vec3f_t out = { a.x * s, a.y * s, a.z * s };
    return out;
}

/* 三维向量模长 */
static float pin_vec3_norm(pin_vec3f_t a)
{
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

/* 内部零速检测器：
 * - 线加速度模长足够小
 * - 角速度模长足够小
 * - 连续满足若干帧后，才输出静止 */
static uint8_t pin_zero_velocity_detect(pin_context_t *ctx,
                                        pin_vec3f_t linear_accel_ped_mps2,
                                        pin_vec3f_t gyro_body_rps,
                                        float *accel_norm_mps2,
                                        float *gyro_norm_rps)
{
    float accel_norm;
    float gyro_norm;
    uint16_t confirm_samples;

    if ((ctx == 0) || (ctx->enable_zero_velocity_detector == 0U))
    {
        if (accel_norm_mps2 != 0)
        {
            *accel_norm_mps2 = 0.0f;
        }
        if (gyro_norm_rps != 0)
        {
            *gyro_norm_rps = 0.0f;
        }
        return 0U;
    }

    accel_norm = pin_vec3_norm(linear_accel_ped_mps2);
    gyro_norm = pin_vec3_norm(gyro_body_rps);

    if (accel_norm_mps2 != 0)
    {
        *accel_norm_mps2 = accel_norm;
    }
    if (gyro_norm_rps != 0)
    {
        *gyro_norm_rps = gyro_norm;
    }

    if ((accel_norm <= ctx->accel_stationary_threshold_mps2) &&
        (gyro_norm <= ctx->gyro_stationary_threshold_rps))
    {
        if (ctx->stationary_sample_count < 65535U)
        {
            ctx->stationary_sample_count++;
        }
    }
    else
    {
        ctx->stationary_sample_count = 0U;
    }

    confirm_samples = ctx->stationary_confirm_samples;
    if (confirm_samples == 0U)
    {
        confirm_samples = 1U;
    }

    return (ctx->stationary_sample_count >= confirm_samples) ? 1U : 0U;
}

void PIN_DefaultConfig(pin_config_t *config)
{
    if (config == 0)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));

    /* 给一套偏保守、能直接起步调试的默认参数。 */
    config->enable_zero_velocity_detector = 1U;
    config->enable_zupt = 1U;
    config->accel_stationary_threshold_mps2 = 0.20f;
    config->gyro_stationary_threshold_rps = 0.15f;
    config->stationary_confirm_samples = 5U;
}

void PIN_Init(pin_context_t *ctx, const pin_config_t *config)
{
    PIN_Reset(ctx, config);
}

void PIN_Reset(pin_context_t *ctx, const pin_config_t *config)
{
    if ((ctx == 0) || (config == 0))
    {
        return;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->initialized = 1U;

    ctx->state.position_ped_m = config->initial_position_ped_m;
    ctx->state.velocity_ped_mps = config->initial_velocity_ped_mps;

    ctx->enable_acc_bias_correction = config->enable_acc_bias_correction;
    ctx->acc_bias_ped_mps2 = config->acc_bias_ped_mps2;

    ctx->enable_zero_velocity_detector = config->enable_zero_velocity_detector;
    ctx->enable_zupt = config->enable_zupt;
    ctx->accel_stationary_threshold_mps2 = config->accel_stationary_threshold_mps2;
    ctx->gyro_stationary_threshold_rps = config->gyro_stationary_threshold_rps;
    ctx->stationary_confirm_samples = config->stationary_confirm_samples;
}

void PIN_SetAccBias(pin_context_t *ctx, const pin_vec3f_t *acc_bias_ped_mps2, uint8_t enable)
{
    if ((ctx == 0) || (acc_bias_ped_mps2 == 0))
    {
        return;
    }

    ctx->acc_bias_ped_mps2 = *acc_bias_ped_mps2;
    ctx->enable_acc_bias_correction = enable;
}

pin_status_t PIN_Update(pin_context_t *ctx, const pin_input_t *input, pin_state_t *out_state)
{
    pin_vec3f_t accel_used;
    pin_vec3f_t accel_avg;
    pin_vec3f_t vel_prev;
    pin_vec3f_t vel_cur;
    pin_vec3f_t pos_prev;
    pin_vec3f_t pos_cur;
    uint8_t zero_velocity_detected;
    uint8_t zupt_applied;
    float accel_norm_mps2;
    float gyro_norm_rps;
    float dt;

    if ((ctx == 0) || (input == 0))
    {
        return PIN_ERR_NULL;
    }
    if (ctx->initialized == 0U)
    {
        return PIN_ERR_NOT_INITIALIZED;
    }

    dt = input->dt_s;
    if (dt <= 0.0f)
    {
        return PIN_ERR_DT;
    }

    /* 先做可选的加速度零偏补偿。 */
    accel_used = input->linear_accel_ped_mps2;
    if (ctx->enable_acc_bias_correction != 0U)
    {
        accel_used = pin_vec3_sub(accel_used, ctx->acc_bias_ped_mps2);
    }

    /* 用当前线加速度和角速度做零速检测。 */
    zero_velocity_detected = pin_zero_velocity_detect(ctx,
                                                      accel_used,
                                                      input->gyro_body_rps,
                                                      &accel_norm_mps2,
                                                      &gyro_norm_rps);

    /* ZUPT 的触发来源有两种：
     * 1. 外部直接告诉模块“当前静止”
     * 2. 内部零速检测器判定当前静止，且允许自动 ZUPT */
    zupt_applied = input->zero_velocity;
    if ((zupt_applied == 0U) &&
        (ctx->enable_zupt != 0U) &&
        (zero_velocity_detected != 0U))
    {
        zupt_applied = 1U;
    }

    vel_prev = ctx->state.velocity_ped_mps;
    pos_prev = ctx->state.position_ped_m;

    /* 速度积分采用梯形积分。
     * 第一帧没有上一帧加速度时，退化成欧拉积分。 */
    if (ctx->has_prev_accel != 0U)
    {
        accel_avg = pin_vec3_scale(pin_vec3_add(ctx->prev_linear_accel_ped_mps2, accel_used), 0.5f);
    }
    else
    {
        accel_avg = accel_used;
    }

    vel_cur = pin_vec3_add(vel_prev, pin_vec3_scale(accel_avg, dt));

    /* ZUPT 的简单实现：
     * 一旦确认静止，直接把前一时刻和当前时刻速度都压成 0，
     * 这样静止阶段的位置也不会继续被漂移速度带着走。 */
    if (zupt_applied != 0U)
    {
        vel_prev.x = 0.0f;
        vel_prev.y = 0.0f;
        vel_prev.z = 0.0f;

        vel_cur.x = 0.0f;
        vel_cur.y = 0.0f;
        vel_cur.z = 0.0f;
    }

    /* 位置同样使用梯形积分。 */
    pos_cur = pin_vec3_add(pos_prev, pin_vec3_scale(pin_vec3_add(vel_prev, vel_cur), 0.5f * dt));

    ctx->state.elapsed_time_s += dt;
    ctx->state.velocity_ped_mps = vel_cur;
    ctx->state.position_ped_m = pos_cur;
    ctx->state.zero_velocity_detected = zero_velocity_detected;
    ctx->state.zupt_applied = zupt_applied;
    ctx->state.last_accel_norm_mps2 = accel_norm_mps2;
    ctx->state.last_gyro_norm_rps = gyro_norm_rps;

    ctx->prev_linear_accel_ped_mps2 = accel_used;
    ctx->has_prev_accel = 1U;

    if (out_state != 0)
    {
        *out_state = ctx->state;
    }

    return PIN_OK;
}

uint8_t PIN_IsReady(const pin_context_t *ctx)
{
    if (ctx == 0)
    {
        return 0U;
    }

    return ctx->initialized;
}

const pin_state_t *PIN_GetState(const pin_context_t *ctx)
{
    if (ctx == 0)
    {
        return 0;
    }

    return &ctx->state;
}
