#pragma once

#include "MORB_SLAM/Odometry.hpp"
#include "MORB_SLAM/ImuTypes.h"

#include <Eigen/Core>

#include <mutex>

namespace MORB_SLAM {


class CameraSettings;
class Atlas;

class InertialOdometry : public Odometry {

public:
    bool GrabOdom(double curr_timestamp, double prev_timestamp) override;
    void PreintegrateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf) override;
    bool PredictStateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf, bool map_updated) override;
    bool ReadyForStereoInitialization(Frame &curr_frame, Frame &last_frame) override;
    void NewKeyFrame(std::shared_ptr<KeyFrame> ref_kf) override;
    void NewMap() override;
    // void LocalOdomBA(std::shared_ptr<KeyFrame> curr_kf, bool &b_abortBA, float &Tinit) override;

public:
    InertialOdometry(std::shared_ptr<CameraSettings> settings);

    void AddAccel(const Eigen::Vector3f &accel_meas, const double timestamp_s);
    void AddAccel(const std::vector<Eigen::Vector3f> &v_accel_meas, const std::vector<double> v_timestamp_s); // adding a batch of accel measurements
    void AddGyro(const Eigen::Vector3f &gyro_meas, const double timestamp_s);
    void AddGyro(const std::vector<Eigen::Vector3f> &v_gyro_meas, const std::vector<double> v_timestamp_s); // adding a batch of gyro measurements
    
private:
    std::mutex mMutexAccel;
    std::mutex mMutexGyro;
    std::vector<Eigen::Vector3f> mvAccelQueue;
    std::vector<double> mvAccelTimestampQueue;
    std::vector<Eigen::Vector3f> mvGyroQueue;    
    std::vector<double> mvGyroTimestampQueue;
    std::vector<IMU::Point> mvImuBatch;

    void newParameterLoader(CameraSettings &settings);
    std::shared_ptr<IMU::Calib> mpImuCalib;
    bool mbStationaryInitEnabled = false;

    std::vector<IMU::Point> interpolateImu(const std::vector<Eigen::Vector3f> &v_imu, const std::vector<double> &v_imu_timestamp_s, const double &curr_frame_timestamp_s, const double &prev_frame_timestamp_s, const bool &is_accel) const;
    void combineImu(std::vector<IMU::Point> &v_accel, std::vector<IMU::Point>& v_gyro, std::vector<IMU::Point> &v_imu_combined);

    std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFromLastKF;

    bool mbStationaryImuInit = false; // TO DO: include in OdometrySettings class 
    // bool mbMonocular;
};


} // namespace MORB_SLAM