#pragma once

#include "IMU_Structs.h"

enum class INSUpdateStatus
{
    Ok,
    NeedMoreData,
    NotInitialized,
    InvalidTimestamp
};

struct INSMechanizationConfig
{
    // Internal units follow the existing mechanization code:
    // - latitude / longitude: rad
    // - height: m
    // - velocity: m/s
    // - roll / pitch / yaw: rad
    double TimeStamp;
    POSITION blh;
    VELOCITY vel;
    POSTURE pos;
    bool enable_acc_calibration;
    bool enable_gyr_calibration;

    INSMechanizationConfig()
        : TimeStamp(0.0),
          enable_acc_calibration(false),
          enable_gyr_calibration(false)
    {
    }
};

INSMechanizationConfig CreateExampleINSConfig();
INSMechanizationConfig CreateSelfDataINSConfig();

// Minimal usage:
//   INSMechanizationConfig cfg;
//   cfg.blh.latitude = yours_initial_lat_rad;
//   cfg.blh.longitude = yours_initial_lon_rad;
//   cfg.blh.H = yours_initial_height_m;
//   cfg.vel = VELOCITY();
//   cfg.pos.roll = yours_initial_roll_rad;
//   cfg.pos.pitch = yours_initial_pitch_rad;
//   cfg.pos.yaw = yours_initial_yaw_rad;
//   INSMechanization ins(cfg);
//
//   INSDataEpoch nav;
//   INSUpdateStatus st = ins.Update(imu_epoch, &nav, zero_velocity_flag);
//   if (st == INSUpdateStatus::Ok) { /* consume nav */ }

class INSMechanization
{
public:
    INSMechanization();
    explicit INSMechanization(const INSMechanizationConfig& config);

    void Reset(const INSMechanizationConfig& config);

    // Feed one raw IMU epoch and optionally request zero-velocity correction
    // for the current step. Returns Ok only when a valid navigation solution
    // has been produced.
    INSUpdateStatus Update(const IMUDataEpoch& imu, INSDataEpoch* result, bool zero_velocity = false);
    INSUpdateStatus Update(const IMUDataEpoch& imu, INSDataEpoch* result, TimeIntervalsArray* zero_speed_intervals);

    // Collect stationary IMU epochs before calling Update when you want to
    // reuse the project's original mean-based bias compensation.
    void ClearCalibrationSamples();
    void AccumulateCalibrationSample(const IMUDataEpoch& imu);

    bool IsInitialized() const;
    bool HasSolution() const;
    bool HasCalibrationSamples() const;

    const INSDataEpoch& GetState() const;
    const Quater& GetQuaternion() const;

private:
    void ApplyCalibration(IMUDataEpoch* imu) const;
    bool ShouldApplyZeroVelocity(double timestamp, TimeIntervalsArray* zero_speed_intervals) const;

    bool is_initialized_;
    bool has_solution_;
    bool has_prv_imu_;
    bool has_pprv_imu_;

    bool enable_acc_calibration_;
    bool enable_gyr_calibration_;

    double calibration_epoch_count_;
    double acc_mean_[3];
    double gyr_mean_[3];

    IMUDataEpoch imu_prv_;
    IMUDataEpoch imu_pprv_;

    INSDataEpoch state_prv_;
    INSDataEpoch state_pprv_;

    Quater q_prv_;
};
