#include "INSMechanization.h"

INSMechanizationConfig CreateExampleINSConfig()
{
    INSMechanizationConfig config;
    config.TimeStamp = initial_time;
    config.blh.latitude = initial_latitude;
    config.blh.longitude = initial_longitude;
    config.blh.H = initial_elevation;
    config.vel.Vn = initial_velocity_north;
    config.vel.Ve = initial_velocity_east;
    config.vel.Vd = initial_velocity_upward;
    config.pos.roll = initial_roll;
    config.pos.pitch = initial_pitch;
    config.pos.yaw = initial_heading;
    return config;
}

INSMechanizationConfig CreateSelfDataINSConfig()
{
    INSMechanizationConfig config;
    config.TimeStamp = ours_initial_second;
    config.blh.latitude = ours_initial_latitude;
    config.blh.longitude = ours_initial_longitude;
    config.blh.H = ours_initial_height;
    config.vel.Vn = ours_initial_Vn;
    config.vel.Ve = ours_initial_Ve;
    config.vel.Vd = ours_initial_Vd;
    config.pos.roll = ours_initial_roll;
    config.pos.pitch = ours_initial_pitch;
    config.pos.yaw = ours_initial_yaw;
    return config;
}

INSMechanization::INSMechanization()
    : is_initialized_(false),
      has_solution_(false),
      has_prv_imu_(false),
      has_pprv_imu_(false),
      enable_acc_calibration_(false),
      enable_gyr_calibration_(false),
      calibration_epoch_count_(0.0)
{
    ClearCalibrationSamples();
}

INSMechanization::INSMechanization(const INSMechanizationConfig& config)
    : INSMechanization()
{
    Reset(config);
}

void INSMechanization::Reset(const INSMechanizationConfig& config)
{
    is_initialized_ = true;
    has_solution_ = false;
    has_prv_imu_ = false;
    has_pprv_imu_ = false;
    ClearCalibrationSamples();

    enable_acc_calibration_ = config.enable_acc_calibration;
    enable_gyr_calibration_ = config.enable_gyr_calibration;

    state_prv_.TimeStamp = config.TimeStamp;
    state_prv_.blh = config.blh;
    state_prv_.vel = config.vel;
    state_prv_.pos = config.pos;

    state_pprv_ = state_prv_;
    q_prv_.SetQbn(config.pos);
}

void INSMechanization::ClearCalibrationSamples()
{
    calibration_epoch_count_ = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        acc_mean_[i] = 0.0;
        gyr_mean_[i] = 0.0;
    }
}

void INSMechanization::AccumulateCalibrationSample(const IMUDataEpoch& imu)
{
    IMUDataEpoch sample = imu;
    CalAvgAcc_Gyr(sample, &calibration_epoch_count_, acc_mean_, gyr_mean_);
}

void INSMechanization::ApplyCalibration(IMUDataEpoch* imu) const
{
    if (imu == nullptr || calibration_epoch_count_ <= 0.0)
    {
        return;
    }

    if (enable_acc_calibration_)
    {
        double acc_mean[3] = { acc_mean_[0], acc_mean_[1], acc_mean_[2] };
        AccCalibration(acc_mean, imu);
    }

    if (enable_gyr_calibration_)
    {
        double gyr_mean[3] = { gyr_mean_[0], gyr_mean_[1], gyr_mean_[2] };
        GyrCalibration(gyr_mean, imu);
    }
}

bool INSMechanization::ShouldApplyZeroVelocity(double timestamp, TimeIntervalsArray* zero_speed_intervals) const
{
    if (zero_speed_intervals == nullptr)
    {
        return false;
    }

    for (int i = 0; i < zero_time_intervals_num; ++i)
    {
        TimeInterval& interval = zero_speed_intervals->getInterval(i);

        if (timestamp > interval.end)
        {
            interval.used = true;
            continue;
        }

        if (timestamp >= interval.start && timestamp <= interval.end)
        {
            return true;
        }
    }

    return false;
}

INSUpdateStatus INSMechanization::Update(const IMUDataEpoch& imu, INSDataEpoch* result, bool zero_velocity)
{
    if (!is_initialized_)
    {
        return INSUpdateStatus::NotInitialized;
    }

    IMUDataEpoch imu_cur = imu;
    ApplyCalibration(&imu_cur);

    if (!has_prv_imu_)
    {
        imu_prv_ = imu_cur;
        has_prv_imu_ = true;
        return INSUpdateStatus::NeedMoreData;
    }

    if (imu_cur.TimeStamp <= imu_prv_.TimeStamp)
    {
        return INSUpdateStatus::InvalidTimestamp;
    }

    if (!has_pprv_imu_)
    {
        imu_pprv_ = imu_prv_;
        imu_prv_ = imu_cur;
        has_pprv_imu_ = true;
        return INSUpdateStatus::NeedMoreData;
    }

    INSDataEpoch state_cur;
    Quater q_cur;

    state_cur.TimeStamp = imu_cur.TimeStamp;

    VelocityUpdate(imu_cur, imu_prv_, imu_pprv_, state_prv_, state_pprv_, &state_cur);

    if (zero_velocity)
    {
        state_cur.vel.Vn = 0.0;
        state_cur.vel.Ve = 0.0;
        state_cur.vel.Vd = 0.0;
    }

    PositionUpdate(imu_cur, imu_prv_, imu_pprv_, state_prv_, state_pprv_, &state_cur);
    PostureUpdate(imu_cur, imu_prv_, state_prv_, q_prv_, &q_cur, &state_cur);

    imu_pprv_ = imu_prv_;
    imu_prv_ = imu_cur;

    state_pprv_ = state_prv_;
    state_prv_ = state_cur;
    q_prv_ = q_cur;
    has_solution_ = true;

    if (result != nullptr)
    {
        *result = state_cur;
    }

    return INSUpdateStatus::Ok;
}

INSUpdateStatus INSMechanization::Update(const IMUDataEpoch& imu, INSDataEpoch* result, TimeIntervalsArray* zero_speed_intervals)
{
    return Update(imu, result, ShouldApplyZeroVelocity(imu.TimeStamp, zero_speed_intervals));
}

bool INSMechanization::IsInitialized() const
{
    return is_initialized_;
}

bool INSMechanization::HasSolution() const
{
    return has_solution_;
}

bool INSMechanization::HasCalibrationSamples() const
{
    return calibration_epoch_count_ > 0.0;
}

const INSDataEpoch& INSMechanization::GetState() const
{
    return state_prv_;
}

const Quater& INSMechanization::GetQuaternion() const
{
    return q_prv_;
}
