#include "ins_nav.h"

#include <math.h>
#include <string.h>

#ifdef INS_USE_DOUBLE
#define INS_SIN  sin
#define INS_COS  cos
#define INS_TAN  tan
#define INS_SQRT sqrt
#define INS_ATAN atan
#define INS_ATAN2 atan2
#define INS_FABS fabs
#else
#define INS_SIN  sinf
#define INS_COS  cosf
#define INS_TAN  tanf
#define INS_SQRT sqrtf
#define INS_ATAN atanf
#define INS_ATAN2 atan2f
#define INS_FABS fabsf
#endif

#define INS_PI            ((ins_real_t)3.14159265358979323846)
#define INS_WGS84_A       ((ins_real_t)6378137.0)
#define INS_WGS84_F       ((ins_real_t)(1.0 / 298.257223563))
#define INS_EARTH_RATE    ((ins_real_t)7.292115e-5)
#define INS_E2            (INS_WGS84_F * ((ins_real_t)2.0 - INS_WGS84_F))
#define INS_EPS           ((ins_real_t)1e-12)

static INS_Vec3 ins_vec3_add(INS_Vec3 a, INS_Vec3 b)
{
    INS_Vec3 out = { a.x + b.x, a.y + b.y, a.z + b.z };
    return out;
}

static INS_Vec3 ins_vec3_sub(INS_Vec3 a, INS_Vec3 b)
{
    INS_Vec3 out = { a.x - b.x, a.y - b.y, a.z - b.z };
    return out;
}

static INS_Vec3 ins_vec3_scale(INS_Vec3 a, ins_real_t s)
{
    INS_Vec3 out = { a.x * s, a.y * s, a.z * s };
    return out;
}

static INS_Vec3 ins_vec3_cross(INS_Vec3 a, INS_Vec3 b)
{
    INS_Vec3 out;
    out.x = a.y * b.z - a.z * b.y;
    out.y = a.z * b.x - a.x * b.z;
    out.z = a.x * b.y - a.y * b.x;
    return out;
}

static void ins_calc_radii(ins_real_t latitude, ins_real_t* rm, ins_real_t* rn)
{
    ins_real_t sin_lat = INS_SIN(latitude);
    ins_real_t denom = (ins_real_t)1.0 - INS_E2 * sin_lat * sin_lat;
    ins_real_t sqrt_denom = INS_SQRT(denom);

    if (rn != 0)
    {
        *rn = INS_WGS84_A / sqrt_denom;
    }
    if (rm != 0)
    {
        *rm = INS_WGS84_A * ((ins_real_t)1.0 - INS_E2) / (sqrt_denom * denom);
    }
}

static INS_Quaternion ins_quaternion_identity(void)
{
    INS_Quaternion q = { (ins_real_t)1.0, (ins_real_t)0.0, (ins_real_t)0.0, (ins_real_t)0.0 };
    return q;
}

static INS_Quaternion ins_quaternion_multiply(INS_Quaternion p, INS_Quaternion q)
{
    INS_Quaternion out;
    out.q0 = p.q0 * q.q0 - p.q1 * q.q1 - p.q2 * q.q2 - p.q3 * q.q3;
    out.q1 = p.q0 * q.q1 + p.q1 * q.q0 + p.q2 * q.q3 - p.q3 * q.q2;
    out.q2 = p.q0 * q.q2 - p.q1 * q.q3 + p.q2 * q.q0 + p.q3 * q.q1;
    out.q3 = p.q0 * q.q3 + p.q1 * q.q2 - p.q2 * q.q1 + p.q3 * q.q0;
    return out;
}

static INS_Quaternion ins_quaternion_normalize(INS_Quaternion q)
{
    ins_real_t norm = INS_SQRT(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    if (norm <= INS_EPS)
    {
        return ins_quaternion_identity();
    }

    q.q0 /= norm;
    q.q1 /= norm;
    q.q2 /= norm;
    q.q3 /= norm;
    return q;
}

static INS_Quaternion ins_quaternion_from_attitude(INS_Attitude att)
{
    ins_real_t half_roll = att.roll * (ins_real_t)0.5;
    ins_real_t half_pitch = att.pitch * (ins_real_t)0.5;
    ins_real_t half_yaw = att.yaw * (ins_real_t)0.5;

    INS_Quaternion q;
    q.q0 = INS_COS(half_roll) * INS_COS(half_pitch) * INS_COS(half_yaw)
         + INS_SIN(half_roll) * INS_SIN(half_pitch) * INS_SIN(half_yaw);
    q.q1 = INS_SIN(half_roll) * INS_COS(half_pitch) * INS_COS(half_yaw)
         - INS_COS(half_roll) * INS_SIN(half_pitch) * INS_SIN(half_yaw);
    q.q2 = INS_COS(half_roll) * INS_SIN(half_pitch) * INS_COS(half_yaw)
         + INS_SIN(half_roll) * INS_COS(half_pitch) * INS_SIN(half_yaw);
    q.q3 = INS_COS(half_roll) * INS_COS(half_pitch) * INS_SIN(half_yaw)
         - INS_SIN(half_roll) * INS_SIN(half_pitch) * INS_COS(half_yaw);
    return ins_quaternion_normalize(q);
}

static INS_Attitude ins_attitude_from_quaternion(INS_Quaternion q)
{
    INS_Attitude out;
    ins_real_t c32 = (ins_real_t)2.0 * (q.q2 * q.q3 + q.q0 * q.q1);
    ins_real_t c33 = q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3;
    ins_real_t c31 = (ins_real_t)2.0 * (q.q1 * q.q3 - q.q0 * q.q2);
    ins_real_t c11 = q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3;
    ins_real_t c21 = (ins_real_t)2.0 * (q.q1 * q.q2 + q.q0 * q.q3);

    out.roll = INS_ATAN2(c32, c33);
    out.pitch = INS_ATAN(-c31 / INS_SQRT(c32 * c32 + c33 * c33));
    out.yaw = INS_ATAN2(c21, c11);
    return out;
}

static INS_Quaternion ins_delta_quaternion(INS_Vec3 delta, ins_real_t sign)
{
    INS_Quaternion q;
    ins_real_t half_x = delta.x * (ins_real_t)0.5;
    ins_real_t half_y = delta.y * (ins_real_t)0.5;
    ins_real_t half_z = delta.z * (ins_real_t)0.5;
    ins_real_t mag = INS_SQRT(half_x * half_x + half_y * half_y + half_z * half_z);
    ins_real_t coef = (ins_real_t)1.0;

    if (mag > INS_EPS)
    {
        coef = INS_SIN(mag) / mag;
    }

    q.q0 = INS_COS(mag);
    q.q1 = sign * half_x * coef;
    q.q2 = sign * half_y * coef;
    q.q3 = sign * half_z * coef;
    return ins_quaternion_normalize(q);
}

static INS_Vec3 ins_rotate_b_to_n(INS_Attitude att, INS_Vec3 vec_b)
{
    ins_real_t roll = att.roll;
    ins_real_t pitch = att.pitch;
    ins_real_t yaw = att.yaw;
    ins_real_t sr = INS_SIN(roll);
    ins_real_t cr = INS_COS(roll);
    ins_real_t sp = INS_SIN(pitch);
    ins_real_t cp = INS_COS(pitch);
    ins_real_t sy = INS_SIN(yaw);
    ins_real_t cy = INS_COS(yaw);

    INS_Vec3 vec_n;
    vec_n.x = (cp * cy) * vec_b.x
            + (-cr * sy + sr * sp * cy) * vec_b.y
            + (sr * sy + cr * sp * cy) * vec_b.z;
    vec_n.y = (cp * sy) * vec_b.x
            + (cr * cy + sr * sp * sy) * vec_b.y
            + (-sr * cy + cr * sp * sy) * vec_b.z;
    vec_n.z = (-sp) * vec_b.x
            + (sr * cp) * vec_b.y
            + (cr * cp) * vec_b.z;
    return vec_n;
}

static INS_Vec3 ins_apply_small_rotation(INS_Vec3 zeta, INS_Vec3 vec)
{
    return ins_vec3_sub(vec, ins_vec3_scale(ins_vec3_cross(zeta, vec), (ins_real_t)0.5));
}

static INS_Vec3 ins_normal_gravity(INS_Blh blh)
{
    ins_real_t sin_lat = INS_SIN(blh.latitude);
    ins_real_t sin_lat2 = sin_lat * sin_lat;
    ins_real_t g0 = (ins_real_t)9.7803267715
                  * ((ins_real_t)1.0
                  + (ins_real_t)0.0052790414 * sin_lat2
                  + (ins_real_t)0.0000232718 * sin_lat2 * sin_lat2);
    ins_real_t g = g0
                 - ((ins_real_t)3.087691089e-6 - (ins_real_t)4.397731e-9 * sin_lat2) * blh.height
                 + (ins_real_t)0.721e-12 * blh.height * blh.height;

    INS_Vec3 out = { (ins_real_t)0.0, (ins_real_t)0.0, g };
    return out;
}

static void ins_extrapolation(const INS_ImuSample* cur,
                              const INS_ImuSample* prev,
                              const INS_ImuSample* prev2,
                              const INS_State* state_prev,
                              const INS_State* state_prev2,
                              INS_Vec3* w_ie_n,
                              INS_Vec3* w_en_n,
                              INS_Vec3* v_mid)
{
    ins_real_t rm_prev, rn_prev;
    ins_real_t rm_prev2, rn_prev2;
    ins_real_t dt2 = prev->timestamp - prev2->timestamp;
    ins_real_t dt1 = (cur->timestamp - prev->timestamp) * (ins_real_t)0.5;

    if (dt2 <= INS_EPS)
    {
        dt2 = cur->timestamp - prev->timestamp;
    }
    if (dt2 <= INS_EPS)
    {
        dt2 = (ins_real_t)1.0;
    }

    ins_calc_radii(state_prev->blh.latitude, &rm_prev, &rn_prev);
    ins_calc_radii(state_prev2->blh.latitude, &rm_prev2, &rn_prev2);

    INS_Vec3 wie_prev = {
        INS_EARTH_RATE * INS_COS(state_prev->blh.latitude),
        (ins_real_t)0.0,
        -INS_EARTH_RATE * INS_SIN(state_prev->blh.latitude)
    };
    INS_Vec3 wie_prev2 = {
        INS_EARTH_RATE * INS_COS(state_prev2->blh.latitude),
        (ins_real_t)0.0,
        -INS_EARTH_RATE * INS_SIN(state_prev2->blh.latitude)
    };

    INS_Vec3 wen_prev = {
        state_prev->velocity.ve / (rn_prev + state_prev->blh.height),
        -state_prev->velocity.vn / (rm_prev + state_prev->blh.height),
        -state_prev->velocity.ve * INS_TAN(state_prev->blh.latitude) / (rn_prev + state_prev->blh.height)
    };
    INS_Vec3 wen_prev2 = {
        state_prev2->velocity.ve / (rn_prev2 + state_prev2->blh.height),
        -state_prev2->velocity.vn / (rm_prev2 + state_prev2->blh.height),
        -state_prev2->velocity.ve * INS_TAN(state_prev2->blh.latitude) / (rn_prev2 + state_prev2->blh.height)
    };

    if (w_ie_n != 0)
    {
        w_ie_n->x = wie_prev.x + dt1 * (wie_prev.x - wie_prev2.x) / dt2;
        w_ie_n->y = wie_prev.y + dt1 * (wie_prev.y - wie_prev2.y) / dt2;
        w_ie_n->z = wie_prev.z + dt1 * (wie_prev.z - wie_prev2.z) / dt2;
    }

    if (w_en_n != 0)
    {
        w_en_n->x = wen_prev.x + dt1 * (wen_prev.x - wen_prev2.x) / dt2;
        w_en_n->y = wen_prev.y + dt1 * (wen_prev.y - wen_prev2.y) / dt2;
        w_en_n->z = wen_prev.z + dt1 * (wen_prev.z - wen_prev2.z) / dt2;
    }

    if (v_mid != 0)
    {
        v_mid->x = state_prev->velocity.vn + dt1 * (state_prev->velocity.vn - state_prev2->velocity.vn) / dt2;
        v_mid->y = state_prev->velocity.ve + dt1 * (state_prev->velocity.ve - state_prev2->velocity.ve) / dt2;
        v_mid->z = state_prev->velocity.vd + dt1 * (state_prev->velocity.vd - state_prev2->velocity.vd) / dt2;
    }
}

static void ins_velocity_update(const INS_ImuSample* cur,
                                const INS_ImuSample* prev,
                                const INS_ImuSample* prev2,
                                const INS_State* state_prev,
                                const INS_State* state_prev2,
                                INS_State* out)
{
    INS_Vec3 vk = cur->acc_delta;
    INS_Vec3 vk_1 = prev->acc_delta;
    INS_Vec3 theta_k = cur->gyro_delta;
    INS_Vec3 theta_k_1 = prev->gyro_delta;
    INS_Vec3 w_ie_n = { 0 };
    INS_Vec3 w_en_n = { 0 };
    INS_Vec3 v_mid = { 0 };
    INS_Vec3 g_prev;
    INS_Vec3 g_prev2;
    INS_Vec3 g_mid;
    INS_Vec3 v_inc_b;
    INS_Vec3 zeta;
    INS_Vec3 v_inc_n;
    INS_Vec3 coriolis;
    INS_Vec3 gravity_coriolis;
    INS_Vec3 vel_prev;
    ins_real_t dt = cur->timestamp - prev->timestamp;

    v_inc_b = ins_vec3_add(vk, ins_vec3_scale(ins_vec3_cross(theta_k, vk), (ins_real_t)0.5));
    v_inc_b = ins_vec3_add(v_inc_b,
                           ins_vec3_scale(
                               ins_vec3_add(ins_vec3_cross(theta_k_1, vk), ins_vec3_cross(vk_1, theta_k)),
                               (ins_real_t)(1.0 / 12.0)));

    ins_extrapolation(cur, prev, prev2, state_prev, state_prev2, &w_ie_n, &w_en_n, &v_mid);
    zeta = ins_vec3_scale(ins_vec3_add(w_ie_n, w_en_n), dt);
    v_inc_n = ins_rotate_b_to_n(state_prev->attitude, v_inc_b);
    v_inc_n = ins_apply_small_rotation(zeta, v_inc_n);

    g_prev = ins_normal_gravity(state_prev->blh);
    g_prev2 = ins_normal_gravity(state_prev2->blh);
    g_mid = ins_vec3_add(g_prev,
                         ins_vec3_scale(ins_vec3_sub(g_prev, g_prev2),
                                        dt / ((ins_real_t)2.0 * (prev->timestamp - prev2->timestamp + INS_EPS))));

    coriolis = ins_vec3_cross(ins_vec3_add(ins_vec3_scale(w_ie_n, (ins_real_t)2.0), w_en_n), v_mid);
    gravity_coriolis = ins_vec3_scale(ins_vec3_sub(g_mid, coriolis), dt);

    vel_prev.x = state_prev->velocity.vn;
    vel_prev.y = state_prev->velocity.ve;
    vel_prev.z = state_prev->velocity.vd;

    vel_prev = ins_vec3_add(vel_prev, v_inc_n);
    vel_prev = ins_vec3_add(vel_prev, gravity_coriolis);

    out->velocity.vn = vel_prev.x;
    out->velocity.ve = vel_prev.y;
    out->velocity.vd = vel_prev.z;
}

static void ins_position_update(const INS_ImuSample* cur,
                                const INS_ImuSample* prev,
                                const INS_ImuSample* prev2,
                                const INS_State* state_prev,
                                const INS_State* state_prev2,
                                INS_State* out)
{
    ins_real_t dt = cur->timestamp - prev->timestamp;
    ins_real_t rm_prev;
    ins_real_t rn_prev;
    ins_real_t rn_prev2;
    ins_real_t rn_mid;
    ins_real_t h_mid;
    ins_real_t lat_mid;

    (void)prev2;

    out->blh.height = state_prev->blh.height
                    - (out->velocity.vd + state_prev->velocity.vd) * (ins_real_t)0.5 * dt;

    ins_calc_radii(state_prev->blh.latitude, &rm_prev, &rn_prev);
    ins_calc_radii(state_prev2->blh.latitude, 0, &rn_prev2);

    h_mid = (out->blh.height + state_prev->blh.height) * (ins_real_t)0.5;
    out->blh.latitude = state_prev->blh.latitude
                      + (out->velocity.vn + state_prev->velocity.vn) * dt
                      / ((ins_real_t)2.0 * (rm_prev + h_mid));

    rn_mid = rn_prev
           + dt * (rn_prev - rn_prev2)
           / ((ins_real_t)2.0 * (prev->timestamp - prev2->timestamp + INS_EPS));
    lat_mid = (out->blh.latitude + state_prev->blh.latitude) * (ins_real_t)0.5;

    out->blh.longitude = state_prev->blh.longitude
                       + (out->velocity.ve + state_prev->velocity.ve) * dt
                       / ((ins_real_t)2.0 * (rn_mid + h_mid) * INS_COS(lat_mid));
}

static void ins_posture_update(const INS_ImuSample* cur,
                               const INS_ImuSample* prev,
                               const INS_State* state_prev,
                               INS_Quaternion q_prev,
                               INS_State* out)
{
    INS_Vec3 erv_b;
    INS_Vec3 w_ie_n;
    INS_Vec3 w_en_n;
    INS_Vec3 erv_n;
    INS_Quaternion q_bkk_1;
    INS_Quaternion q_nk_1k;
    INS_Quaternion q_mid;
    ins_real_t dt = cur->timestamp - prev->timestamp;
    ins_real_t rm;
    ins_real_t rn;

    erv_b = ins_vec3_add(cur->gyro_delta,
                         ins_vec3_scale(ins_vec3_cross(prev->gyro_delta, cur->gyro_delta),
                                        (ins_real_t)(1.0 / 12.0)));
    q_bkk_1 = ins_delta_quaternion(erv_b, (ins_real_t)1.0);

    ins_calc_radii(state_prev->blh.latitude, &rm, &rn);
    w_ie_n.x = INS_EARTH_RATE * INS_COS(state_prev->blh.latitude);
    w_ie_n.y = (ins_real_t)0.0;
    w_ie_n.z = -INS_EARTH_RATE * INS_SIN(state_prev->blh.latitude);

    w_en_n.x = state_prev->velocity.ve / (rn + state_prev->blh.height);
    w_en_n.y = -state_prev->velocity.vn / (rm + state_prev->blh.height);
    w_en_n.z = -state_prev->velocity.ve * INS_TAN(state_prev->blh.latitude) / (rn + state_prev->blh.height);

    erv_n = ins_vec3_scale(ins_vec3_add(w_ie_n, w_en_n), dt);
    q_nk_1k = ins_delta_quaternion(erv_n, (ins_real_t)-1.0);

    q_mid = ins_quaternion_multiply(q_nk_1k, q_prev);
    out->quaternion = ins_quaternion_normalize(ins_quaternion_multiply(q_mid, q_bkk_1));
    out->attitude = ins_attitude_from_quaternion(out->quaternion);
}

void INS_DefaultConfig(INS_Config* config)
{
    if (config == 0)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
}

void INS_Init(INS_Context* ctx, const INS_Config* config)
{
    INS_Reset(ctx, config);
}

void INS_Reset(INS_Context* ctx, const INS_Config* config)
{
    if ((ctx == 0) || (config == 0))
    {
        return;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->initialized = 1U;
    ctx->bias = config->bias;

    ctx->state_prev.timestamp = config->timestamp;
    ctx->state_prev.blh = config->initial_blh;
    ctx->state_prev.velocity = config->initial_velocity;
    ctx->state_prev.attitude = config->initial_attitude;
    ctx->state_prev.quaternion = ins_quaternion_from_attitude(config->initial_attitude);

    ctx->state_prev2 = ctx->state_prev;
    ctx->q_prev = ctx->state_prev.quaternion;
}

void INS_SetBiasConfig(INS_Context* ctx, const INS_BiasConfig* bias)
{
    if ((ctx == 0) || (bias == 0))
    {
        return;
    }

    ctx->bias = *bias;
}

INS_Status INS_Update(INS_Context* ctx,
                      const INS_ImuSample* sample,
                      uint8_t zero_velocity,
                      INS_State* out_state)
{
    INS_ImuSample cur;
    INS_State state_cur;

    if ((ctx == 0) || (sample == 0))
    {
        return INS_STATUS_INVALID_ARGUMENT;
    }
    if (ctx->initialized == 0U)
    {
        return INS_STATUS_NOT_INITIALIZED;
    }

    cur = *sample;
    if (ctx->bias.enable_acc_bias_correction != 0U)
    {
        cur.acc_delta = ins_vec3_sub(cur.acc_delta, ctx->bias.acc_bias_delta);
    }
    if (ctx->bias.enable_gyro_bias_correction != 0U)
    {
        cur.gyro_delta = ins_vec3_sub(cur.gyro_delta, ctx->bias.gyro_bias_delta);
    }

    if (ctx->has_prev_imu == 0U)
    {
        ctx->imu_prev = cur;
        ctx->has_prev_imu = 1U;
        return INS_STATUS_NEED_MORE_DATA;
    }

    if (cur.timestamp <= ctx->imu_prev.timestamp)
    {
        return INS_STATUS_INVALID_TIMESTAMP;
    }

    if (ctx->has_prev2_imu == 0U)
    {
        ctx->imu_prev2 = ctx->imu_prev;
        ctx->imu_prev = cur;
        ctx->has_prev2_imu = 1U;
        return INS_STATUS_NEED_MORE_DATA;
    }

    state_cur = ctx->state_prev;
    state_cur.timestamp = cur.timestamp;

    ins_velocity_update(&cur, &ctx->imu_prev, &ctx->imu_prev2, &ctx->state_prev, &ctx->state_prev2, &state_cur);
    if (zero_velocity != 0U)
    {
        state_cur.velocity.vn = (ins_real_t)0.0;
        state_cur.velocity.ve = (ins_real_t)0.0;
        state_cur.velocity.vd = (ins_real_t)0.0;
    }
    ins_position_update(&cur, &ctx->imu_prev, &ctx->imu_prev2, &ctx->state_prev, &ctx->state_prev2, &state_cur);
    ins_posture_update(&cur, &ctx->imu_prev, &ctx->state_prev, ctx->q_prev, &state_cur);

    ctx->imu_prev2 = ctx->imu_prev;
    ctx->imu_prev = cur;
    ctx->state_prev2 = ctx->state_prev;
    ctx->state_prev = state_cur;
    ctx->q_prev = state_cur.quaternion;
    ctx->has_solution = 1U;

    if (out_state != 0)
    {
        *out_state = state_cur;
    }

    return INS_STATUS_OK;
}

INS_Status INS_UpdateSensorFrame(INS_Context* ctx,
                                 const INS_SensorFrame* frame,
                                 uint8_t zero_velocity,
                                 INS_State* out_state)
{
    INS_ImuSample sample;

    if ((ctx == 0) || (frame == 0))
    {
        return INS_STATUS_INVALID_ARGUMENT;
    }
    if (frame->dt <= (ins_real_t)0.0)
    {
        return INS_STATUS_INVALID_ARGUMENT;
    }

    sample.timestamp = frame->timestamp;
    sample.gyro_delta = ins_vec3_scale(frame->gyro_rad_s, frame->dt);
    sample.acc_delta = ins_vec3_scale(frame->accel_mps2, frame->dt);

    return INS_Update(ctx, &sample, zero_velocity, out_state);
}

uint8_t INS_IsReady(const INS_Context* ctx)
{
    if (ctx == 0)
    {
        return 0U;
    }
    return ctx->has_solution;
}

const INS_State* INS_GetState(const INS_Context* ctx)
{
    if (ctx == 0)
    {
        return 0;
    }
    return &ctx->state_prev;
}
