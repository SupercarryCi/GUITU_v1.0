#include "pdr_step_detector.h"
#include <math.h>
#include <string.h>

/*
 * ============================================================
 * 参数区：200 Hz 数据版本
 * ============================================================
 */

/*
 * IMU 实际采样率：200 Hz
 */
#define PDR_FS_HZ                       200.0f

/*
 * 是否删除相邻重复样本。
 * 如果你的 IMU 是 data-ready 中断读取，建议设为 0。
 */
#define PDR_DROP_DUPLICATES             0

/*
 * 一步的最小/最大间隔，单位 ms。
 *
 * 200 Hz 下：
 *   180 ms -> 36 samples
 *   900 ms -> 180 samples
 */
#define PDR_STEP_MIN_MS                 180.0f
#define PDR_STEP_MAX_MS                 900.0f

/*
 * 停顿后重置。
 *
 * 200 Hz 下：
 *   1200 ms -> 240 samples
 */
#define PDR_IDLE_RESET_MS               1200.0f

/*
 * 走/跑分类阈值，单位 ms。
 *
 * 200 Hz 下：
 *   480 ms -> 96 samples
 *   500 ms -> 100 samples
 */
#define PDR_RUN_ENTER_MS                480.0f
#define PDR_WALK_ENTER_MS               500.0f

/*
 * 加速度动态能量参数。
 */
#define PDR_ACC_ENERGY_ALPHA            0.02f
#define PDR_ACC_ENERGY_MIN              120.0f
#define PDR_ACC_RUN_ENTER               1200.0f
#define PDR_ACC_WALK_ENTER              600.0f

/*
 * 陀螺仪正负判断死区。
 */
#define PDR_GYRO_DEADBAND_MIN           600.0f
#define PDR_GYRO_DEADBAND_RATIO         0.30f

/*
 * 陀螺仪半周期峰值阈值。
 */
#define PDR_GYRO_LOBE_MIN               1200.0f
#define PDR_GYRO_LOBE_RATIO             0.35f

/*
 * 检测到多少个有效候选步后才开始判断 WALK/RUN。
 */
#define PDR_MIN_CANDIDATES_TO_CLASSIFY  2U

/*
 * ============================================================
 * 200 Hz 滤波器系数
 * ============================================================
 *
 * 当前参数：
 *   FS = 200 Hz
 *   gyro HPF = 0.5 Hz
 *   gyro LPF = 5.0 Hz
 *   acc gravity LPF = 0.3 Hz
 *
 * 一阶低通：
 *   alpha = dt / (RC + dt)
 *
 * 一阶高通：
 *   beta = RC / (RC + dt)
 *
 * RC = 1 / (2 * pi * fc)
 * dt = 1 / fs
 */
#define PDR_GYRO_HPF_BETA               0.98453496f   // fc=0.5Hz, fs=200Hz
#define PDR_GYRO_LPF_ALPHA              0.13575525f   // fc=5.0Hz, fs=200Hz
#define PDR_ACC_GRAVITY_LPF_ALPHA       0.00933678f   // fc=0.3Hz, fs=200Hz


/*
* ============================================================
* 小工具函数
* ============================================================
*/

static inline float pdr_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static inline float pdr_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static inline uint32_t pdr_ms_to_samples(float ms)
{
    return (uint32_t)((ms * PDR_FS_HZ / 1000.0f) + 0.5f);
}

/*
 * 加速度模长。
 *
 * 如果 MCU 没有 FPU，sqrtf 可能稍贵。
 * 但 200 Hz 下通常可以接受。吗
 *
 * 如果资源很紧，可以把这里改成近似：
 *   abs(ax) + abs(ay) + abs(az)
 * 但阈值需要重新标定。
 */
static inline float pdr_acc_mag(int32_t ax, int32_t ay, int32_t az)
{
    float x = (float)ax;
    float y = (float)ay;
    float z = (float)az;

    return sqrtf(x * x + y * y + z * z);
}

/*
 * 带死区的符号判断。
 * 在死区内保持上一个符号，避免 0 附近噪声导致频繁正负跳变。
 */
static inline int8_t pdr_sign_with_deadband(float x, float deadband, int8_t last_sign)
{
    if (x > deadband) {
        return 1;
    }

    if (x < -deadband) {
        return -1;
    }

    return last_sign;
}


/*
 * ============================================================
 * 一阶滤波器
 * ============================================================
 */

static void pdr_lpf_init(PdrLPF* f, float alpha)
{
    f->alpha = alpha;
    f->y = 0.0f;
    f->initialized = 0;
}

static float pdr_lpf_update(PdrLPF* f, float x)
{
    if (!f->initialized) {
        f->y = x;
        f->initialized = 1;
        return f->y;
    }

    f->y = f->y + f->alpha * (x - f->y);
    return f->y;
}

static void pdr_hpf_init(PdrHPF* f, float beta)
{
    f->beta = beta;
    f->x_prev = 0.0f;
    f->y_prev = 0.0f;
    f->initialized = 0;
}

static float pdr_hpf_update(PdrHPF* f, float x)
{
    float y;

    if (!f->initialized) {
        f->x_prev = x;
        f->y_prev = 0.0f;
        f->initialized = 1;
        return 0.0f;
    }

    y = f->beta * (f->y_prev + x - f->x_prev);

    f->x_prev = x;
    f->y_prev = y;

    return y;
}


/*
 * ============================================================
 * 模式更新
 * ============================================================
 */

static void pdr_update_mode(PdrStepDetector* det)
{
    if (det->candidate_count < PDR_MIN_CANDIDATES_TO_CLASSIFY) {
        det->mode = PDR_MODE_IDLE;
        return;
    }

    /*
     * 跑步条件：
     *   1. 平滑步间隔较短
     *   2. 或者加速度动态能量很高
     */
    if ((det->interval_ema_ms < PDR_RUN_ENTER_MS) ||
        (det->acc_energy_ema > PDR_ACC_RUN_ENTER)) {
        det->mode = PDR_MODE_RUN;
    }
    /*
     * 行走条件：
     *   1. 平滑步间隔较长
     *   2. 且加速度动态能量没有跑步那么高
     */
    else if ((det->interval_ema_ms > PDR_WALK_ENTER_MS) &&
        (det->acc_energy_ema < PDR_ACC_WALK_ENTER)) {
        det->mode = PDR_MODE_WALK;
    }
    /*
     * 中间区域保持上一状态，避免 WALK/RUN 抖动。
     */
}


/*
 * ============================================================
 * 对外接口
 * ============================================================
 */

void pdr_step_init(PdrStepDetector* det)
{
    if (det == 0) {
        return;
    }

    memset(det, 0, sizeof(PdrStepDetector));

    pdr_hpf_init(&det->gyro_hpf, PDR_GYRO_HPF_BETA);
    pdr_lpf_init(&det->gyro_lpf, PDR_GYRO_LPF_ALPHA);
    pdr_lpf_init(&det->acc_gravity_lpf, PDR_ACC_GRAVITY_LPF_ALPHA);

    det->mode = PDR_MODE_IDLE;
    det->last_sign = 0;
    det->has_last_step = 0;
    det->has_last_raw = 0;
}


/*
 * 每次输入一帧 IMU 数据。
 *
 * 当前版本固定使用 gz 作为主轴：
 *   gyro_raw = gz
 *
 * 如果后续要支持 gx/gy/gz 自动主轴选择，可以在外部先选好主轴，
 * 或者把 gx/gy/gz 方差窗口加进来。
 */
PdrStepOutput pdr_step_update(
    PdrStepDetector* det,
    int32_t ax,
    int32_t ay,
    int32_t az,
    int32_t gx,
    int32_t gy,
    int32_t gz
) {
    PdrStepOutput out;

    /*
     * 输出默认值
     */
    memset(&out, 0, sizeof(out));

    if (det == 0) {
        return out;
    }

#if PDR_DROP_DUPLICATES
    /*
     * 去掉相邻完全重复样本。
     * 实际部署更推荐使用 IMU data-ready 中断。
     */
    if (det->has_last_raw) {
        if ((ax == det->last_ax) &&
            (ay == det->last_ay) &&
            (az == det->last_az) &&
            (gx == det->last_gx) &&
            (gy == det->last_gy) &&
            (gz == det->last_gz)) {

            out.step_detected = 0;
            out.step_count = det->step_count;
            out.mode = det->mode;
            return out;
        }
    }

    det->last_ax = ax;
    det->last_ay = ay;
    det->last_az = az;
    det->last_gx = gx;
    det->last_gy = gy;
    det->last_gz = gz;
    det->has_last_raw = 1;
#else
    (void)gx;
    (void)gy;
#endif

    det->sample_index++;

    /*
     * ------------------------------------------------------------
     * 1. 陀螺仪主轴滤波
     * ------------------------------------------------------------
     * Python 脚本中 USE_AXIS = "gz"，这里直接使用 gz。
     */
    float gyro_raw = (float)gz;

    float gyro_hp = pdr_hpf_update(&det->gyro_hpf, gyro_raw);
    float gyro_f = pdr_lpf_update(&det->gyro_lpf, gyro_hp);

    /*
     * ------------------------------------------------------------
     * 2. 加速度动态能量
     * ------------------------------------------------------------
     * acc_mag = sqrt(ax^2 + ay^2 + az^2)
     * gravity_est = lowpass(acc_mag)
     * acc_dyn = acc_mag - gravity_est
     * acc_energy_ema = EMA(abs(acc_dyn))
     */
    float acc_mag = pdr_acc_mag(ax, ay, az);
    float gravity_est = pdr_lpf_update(&det->acc_gravity_lpf, acc_mag);
    float acc_dyn = acc_mag - gravity_est;

    det->acc_energy_ema =
        (1.0f - PDR_ACC_ENERGY_ALPHA) * det->acc_energy_ema +
        PDR_ACC_ENERGY_ALPHA * pdr_absf(acc_dyn);

    /*
     * ------------------------------------------------------------
     * 3. 静止/停顿状态重置
     * ------------------------------------------------------------
     * 如果长时间没有有效步，并且加速度动态能量很低，
     * 认为已经停止，重置状态。
     */
    if (det->has_last_step) {
        uint32_t no_step_samples = det->sample_index - det->last_step_index;
        uint32_t idle_reset_samples = pdr_ms_to_samples(PDR_IDLE_RESET_MS);

        if ((no_step_samples > idle_reset_samples) &&
            (det->acc_energy_ema < PDR_ACC_ENERGY_MIN)) {

            det->mode = PDR_MODE_IDLE;
            det->candidate_count = 0;
            det->interval_ema_ms = 0.0f;
            det->has_last_step = 0;
            det->lobe_peak = 0.0f;
        }
    }

    /*
     * ------------------------------------------------------------
     * 4. 带死区的正负半周期判断
     * ------------------------------------------------------------
     */
    float deadband = pdr_maxf(
        PDR_GYRO_DEADBAND_MIN,
        PDR_GYRO_DEADBAND_RATIO * det->gyro_peak_ema
    );

    int8_t sign_now = pdr_sign_with_deadband(
        gyro_f,
        deadband,
        det->last_sign
    );

    /*
     * ------------------------------------------------------------
     * 5. 记录当前半周期最大幅值
     * ------------------------------------------------------------
     */
    if (sign_now != 0) {
        float abs_gyro = pdr_absf(gyro_f);

        if (abs_gyro > det->lobe_peak) {
            det->lobe_peak = abs_gyro;
        }
    }

    /*
     * ------------------------------------------------------------
     * 6. 检测正负切换
     * ------------------------------------------------------------
     */
    uint8_t sign_changed = 0;

    if ((det->last_sign != 0) && (sign_now != det->last_sign)) {
        sign_changed = 1;
    }

    if (sign_changed) {
        /*
         * 如果之前没有参考步点，则当前切换只作为参考，不计步。
         * 这对应 Python 版中的：
         *   if last_step_index is None: establish reference
         */
        if (!det->has_last_step) {
            det->last_step_index = det->sample_index;
            det->has_last_step = 1;
            det->lobe_peak = pdr_absf(gyro_f);
            det->last_sign = sign_now;

            out.step_detected = 0;
            out.step_count = det->step_count;
            out.mode = det->mode;
            out.gyro_filtered = gyro_f;
            out.acc_energy = det->acc_energy_ema;
            out.lobe_peak = det->lobe_peak;
            out.gyro_threshold = pdr_maxf(
                PDR_GYRO_LOBE_MIN,
                PDR_GYRO_LOBE_RATIO * det->gyro_peak_ema
            );
            out.interval_ms = 0.0f;

            return out;
        }

        uint32_t dt_samples = det->sample_index - det->last_step_index;
        float interval_ms = 1000.0f * ((float)dt_samples) / PDR_FS_HZ;

        uint32_t step_min_samples = pdr_ms_to_samples(PDR_STEP_MIN_MS);
        uint32_t step_max_samples = pdr_ms_to_samples(PDR_STEP_MAX_MS);

        float gyro_threshold = pdr_maxf(
            PDR_GYRO_LOBE_MIN,
            PDR_GYRO_LOBE_RATIO * det->gyro_peak_ema
        );

        /*
         * ------------------------------------------------------------
         * 7. 停顿后重新建立参考点
         * ------------------------------------------------------------
         * 如果两次正负切换间隔超过 STEP_MAX_MS，
         * 不能一直拿停顿前的 last_step_index 计算 dt。
         * 当前切换作为新段落参考点，本次不计步。
         */
        if (dt_samples > step_max_samples) {
            det->mode = PDR_MODE_IDLE;
            det->candidate_count = 0;
            det->interval_ema_ms = 0.0f;

            det->last_step_index = det->sample_index;
            det->has_last_step = 1;
            det->lobe_peak = pdr_absf(gyro_f);
            det->last_sign = sign_now;

            out.step_detected = 0;
            out.step_count = det->step_count;
            out.mode = det->mode;
            out.gyro_filtered = gyro_f;
            out.acc_energy = det->acc_energy_ema;
            out.lobe_peak = det->lobe_peak;
            out.gyro_threshold = gyro_threshold;
            out.interval_ms = interval_ms;

            return out;
        }

        /*
         * ------------------------------------------------------------
         * 8. 三个判步条件
         * ------------------------------------------------------------
         */
        uint8_t interval_ok =
            (dt_samples >= step_min_samples) &&
            (dt_samples <= step_max_samples);

        uint8_t gyro_ok =
            det->lobe_peak > gyro_threshold;

        uint8_t acc_ok =
            det->acc_energy_ema > PDR_ACC_ENERGY_MIN;

        if (interval_ok && gyro_ok && acc_ok) {
            /*
             * 确认为一步
             */
            det->step_count++;
            det->candidate_count++;

            /*
             * 更新步间隔 EMA
             */
            if (det->interval_ema_ms <= 1.0e-6f) {
                det->interval_ema_ms = interval_ms;
            }
            else {
                det->interval_ema_ms =
                    0.8f * det->interval_ema_ms +
                    0.2f * interval_ms;
            }

            /*
             * 更新半周期峰值 EMA
             */
            if (det->gyro_peak_ema <= 1.0e-6f) {
                det->gyro_peak_ema = det->lobe_peak;
            }
            else {
                det->gyro_peak_ema =
                    0.9f * det->gyro_peak_ema +
                    0.1f * det->lobe_peak;
            }

            /*
             * 更新 WALK/RUN 状态
             */
            pdr_update_mode(det);

            det->last_step_index = det->sample_index;
            det->has_last_step = 1;

            out.step_detected = 1;
            out.interval_ms = interval_ms;
        }

        /*
         * 一个半周期结束，重置半周期峰值。
         */
        det->lobe_peak = pdr_absf(gyro_f);
    }

    det->last_sign = sign_now;

    /*
     * 输出调试量
     */
    out.step_count = det->step_count;
    out.mode = det->mode;
    out.gyro_filtered = gyro_f;
    out.acc_energy = det->acc_energy_ema;
    out.lobe_peak = det->lobe_peak;
    out.gyro_threshold = pdr_maxf(
        PDR_GYRO_LOBE_MIN,
        PDR_GYRO_LOBE_RATIO * det->gyro_peak_ema
    );

    return out;
}