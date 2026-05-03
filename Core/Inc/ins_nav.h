#ifndef __INS_NAV_H
#define __INS_NAV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef INS_USE_DOUBLE
typedef float ins_real_t;
#else
typedef double ins_real_t;
#endif

typedef enum
{
    INS_STATUS_OK = 0,
    INS_STATUS_NEED_MORE_DATA = 1,
    INS_STATUS_NOT_INITIALIZED = 2,
    INS_STATUS_INVALID_ARGUMENT = 3,
    INS_STATUS_INVALID_TIMESTAMP = 4
} INS_Status;

typedef struct
{
    ins_real_t x;
    ins_real_t y;
    ins_real_t z;
} INS_Vec3;

typedef struct
{
    ins_real_t latitude;   /* rad */
    ins_real_t longitude;  /* rad */
    ins_real_t height;     /* m */
} INS_Blh;

typedef struct
{
    ins_real_t vn;         /* m/s */
    ins_real_t ve;         /* m/s */
    ins_real_t vd;         /* m/s, positive down */
} INS_Velocity;

typedef struct
{
    ins_real_t roll;       /* rad */
    ins_real_t pitch;      /* rad */
    ins_real_t yaw;        /* rad */
} INS_Attitude;

typedef struct
{
    ins_real_t q0;
    ins_real_t q1;
    ins_real_t q2;
    ins_real_t q3;
} INS_Quaternion;

typedef struct
{
    ins_real_t timestamp;  /* s */
    INS_Vec3 gyro_delta;   /* rad */
    INS_Vec3 acc_delta;    /* m/s */
} INS_ImuSample;

typedef struct
{
    ins_real_t timestamp;      /* s */
    ins_real_t dt;             /* s */
    INS_Vec3 accel_mps2;       /* body-frame accelerometer output / specific force */
    INS_Vec3 gyro_rad_s;       /* body-frame angular rate */
    INS_Vec3 mag;              /* magnetic field, currently not used by the pure INS core */
    INS_Attitude angle;        /* sensor-reported euler angles, currently not used by the pure INS core */
    INS_Quaternion quaternion; /* sensor-reported quaternion, currently not used by the pure INS core */
} INS_SensorFrame;

typedef struct
{
    ins_real_t timestamp;
    INS_Blh blh;
    INS_Velocity velocity;
    INS_Attitude attitude;
    INS_Quaternion quaternion;
} INS_State;

typedef struct
{
    uint8_t enable_acc_bias_correction;
    uint8_t enable_gyro_bias_correction;
    INS_Vec3 acc_bias_delta;   /* same unit as acc_delta, per sample */
    INS_Vec3 gyro_bias_delta;  /* same unit as gyro_delta, per sample */
} INS_BiasConfig;

typedef struct
{
    ins_real_t timestamp;
    INS_Blh initial_blh;
    INS_Velocity initial_velocity;
    INS_Attitude initial_attitude;
    INS_BiasConfig bias;
} INS_Config;

typedef struct
{
    uint8_t initialized;
    uint8_t has_prev_imu;
    uint8_t has_prev2_imu;
    uint8_t has_solution;

    INS_BiasConfig bias;

    INS_ImuSample imu_prev;
    INS_ImuSample imu_prev2;

    INS_State state_prev;
    INS_State state_prev2;

    INS_Quaternion q_prev;
} INS_Context;

/* Default config is zeroed state with all corrections disabled. */
void INS_DefaultConfig(INS_Config* config);

/* Initialize / reset the mechanization context. */
void INS_Init(INS_Context* ctx, const INS_Config* config);
void INS_Reset(INS_Context* ctx, const INS_Config* config);

/* Optional bias configuration, applied before each update. */
void INS_SetBiasConfig(INS_Context* ctx, const INS_BiasConfig* bias);

/* Core update.
 * Input must match the original algorithm semantics:
 * - gyro_delta: angular increment in rad
 * - acc_delta : velocity increment in m/s
 * zero_velocity != 0 forces current output velocity to zero before position update.
 */
INS_Status INS_Update(INS_Context* ctx,
                      const INS_ImuSample* sample,
                      uint8_t zero_velocity,
                      INS_State* out_state);

/* Preferred high-level interface for embedded use.
 * Input is the sensor's direct output frame. Internally the module converts:
 * - gyro_rad_s * dt -> gyro_delta
 * - accel_mps2 * dt -> acc_delta
 * The pure INS core currently uses accel_mps2 and gyro_rad_s only.
 */
INS_Status INS_UpdateSensorFrame(INS_Context* ctx,
                                 const INS_SensorFrame* frame,
                                 uint8_t zero_velocity,
                                 INS_State* out_state);

uint8_t INS_IsReady(const INS_Context* ctx);
const INS_State* INS_GetState(const INS_Context* ctx);

#ifdef __cplusplus
}
#endif

#endif
