#ifndef PDR_STEP_DETECTOR_H
#define PDR_STEP_DETECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 运动状态
 */
typedef enum {
    PDR_MODE_IDLE = 0,
    PDR_MODE_WALK = 1,
    PDR_MODE_RUN  = 2
} PdrMotionMode;

/*
 * 单次更新后的输出结果
 */
typedef struct {
    uint8_t step_detected;      // 本次采样是否检测到一步
    uint32_t step_count;        // 累计步数
    PdrMotionMode mode;         // 当前状态：IDLE / WALK / RUN

    /*
     * 调试量，可通过串口打印。
     * 部署时如果不用，可以删掉这些字段以节省 RAM。
     */
    float gyro_filtered;        // 滤波后的 gz
    float acc_energy;           // 加速度动态能量
    float lobe_peak;            // 当前/刚结束半周期最大陀螺仪幅值
    float gyro_threshold;       // 当前陀螺仪判步门限
    float interval_ms;          // 本次步间隔，只有 step_detected=1 时有意义
} PdrStepOutput;

/*
 * 一阶低通滤波器
 */
typedef struct {
    float alpha;
    float y;
    uint8_t initialized;
} PdrLPF;

/*
 * 一阶高通滤波器
 */
typedef struct {
    float beta;
    float x_prev;
    float y_prev;
    uint8_t initialized;
} PdrHPF;

/*
 * 检测器状态
 */
typedef struct {
    /*
     * 滤波器
     */
    PdrHPF gyro_hpf;
    PdrLPF gyro_lpf;
    PdrLPF acc_gravity_lpf;

    /*
     * 主状态
     */
    uint32_t sample_index;
    uint32_t last_step_index;
    uint8_t has_last_step;

    int8_t last_sign;

    float lobe_peak;
    float gyro_peak_ema;
    float acc_energy_ema;
    float interval_ema_ms;

    uint32_t step_count;
    uint16_t candidate_count;
    PdrMotionMode mode;

    /*
     * 去重用。嵌入式里更推荐用 IMU data-ready 中断；
     * 如果你能保证每次都是新样本，可以关闭去重。
     */
    int32_t last_ax;
    int32_t last_ay;
    int32_t last_az;
    int32_t last_gx;
    int32_t last_gy;
    int32_t last_gz;
    uint8_t has_last_raw;
} PdrStepDetector;


/*
 * 初始化检测器。
 */
void pdr_step_init(PdrStepDetector *det);


/*
 * 每来一帧 IMU 数据调用一次。
 *
 * 输入：
 *   ax, ay, az: 加速度原始值
 *   gx, gy, gz: 陀螺仪原始值
 *
 * 当前版本固定使用 gz 作为主轴。
 *
 * 返回：
 *   PdrStepOutput，其中 step_detected=1 表示本次确认了一步。
 */
PdrStepOutput pdr_step_update(
    PdrStepDetector *det,
    int32_t ax,
    int32_t ay,
    int32_t az,
    int32_t gx,
    int32_t gy,
    int32_t gz
);

#ifdef __cplusplus
}
#endif

#endif