#pragma once

#include "MORB_SLAM/Odometry.hpp"
#include "MORB_SLAM/ImuTypes.h"
#include "MORB_SLAM/Settings/Settings.h"
#include "MORB_SLAM/Frame.h"

#include <Eigen/Core>

#include <mutex>

namespace MORB_SLAM {


class Atlas;

struct InertialFrameData : public ExternalFrameData {
    // default constructor
    InertialFrameData()
        : mpImuPreintegrated(nullptr),
          mpImuPreintegratedFrame(nullptr),
          mbImuPreintegrated(false)
    {
        mpMutexImu = std::make_shared<std::mutex>();
    }

    InertialFrameData(IMU::Calib imuCalib)
        : mImuCalib(imuCalib),
          mpImuPreintegrated(nullptr),
          mpImuPreintegratedFrame(nullptr),
          mbImuPreintegrated(false)
    {
        mpMutexImu = std::make_shared<std::mutex>();
    } 
    IMU::Bias mImuBias;
    IMU::Calib mImuCalib;
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFrame;
    bool mbImuPreintegrated;
    std::shared_ptr<std::mutex> mpMutexImu;
};

class InertialOdometrySettings : public Settings {
public:
    InertialOdometrySettings(const std::string& configFile);

    float noiseGyro() const { return noiseGyro_; }
    float noiseAcc() const { return noiseAcc_; }
    float gyroWalk() const { return gyroWalk_; }
    float accWalk() const { return accWalk_; }
    float accFrequency() const { return accFrequency_; }
    float gyroFrequency() const { return gyroFrequency_; }
    const Sophus::SE3f &Tbc() const { return Tbc_; }

    bool fastIMUInit() const { return fastIMUInit_; }
    bool stationaryIMUInit() const { return stationaryIMUInit_; }

    friend std::ostream& operator<<(std::ostream& output, const InertialOdometrySettings& s);

private:
    void readIMU(cv::FileStorage& fSettings);

    float noiseGyro_, noiseAcc_;
    float gyroWalk_, accWalk_;
    float accFrequency_;
    float gyroFrequency_;
    Sophus::SE3f Tbc_;

    bool fastIMUInit_;
    bool stationaryIMUInit_;
};

class InertialOdometry : public Odometry {

public:
    bool GrabOdom(double curr_timestamp, double prev_timestamp) override;
    bool InitFrameData(Frame& frame) override;
    void PreintegrateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf) override;
    bool PredictStateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf, bool map_updated) override;
    bool ReadyForStereoInitialization(Frame &curr_frame, Frame &last_frame) override;
    bool ReadyForMonocularInitialization(Frame &curr_frame, Frame &last_frame) override;
    void InitialMapMonocular(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> initial_kf) override;
    void NewKeyFrame(std::shared_ptr<KeyFrame> ref_kf) override;
    void NewMap() override;
    void LocalOdomBA(std::shared_ptr<KeyFrame> curr_kf, bool &b_abortBA) override;
    void InitializeOdom() override;
    void PostInitializeOdom() override;
    void MergeLocalInitializeMap(const std::shared_ptr<Map> &curr_map) override;
    void MergeOdomBA(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> merge_kf, std::shared_ptr<Map> curr_map, KeyFrameAndPose& corr_poses) override;
    void LoopClosingOptimizeEssentialGraph(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF, const KeyFrameAndPose& NonCorrectedSim3, const KeyFrameAndPose& CorrectedSim3, const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>& LoopConnections) override;
    
public:
    InertialOdometry(std::shared_ptr<InertialOdometrySettings> settings, const CameraType &cam);

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

    void newParameterLoader(InertialOdometrySettings &settings);
    std::shared_ptr<IMU::Calib> mpImuCalib;
    bool mbStationaryInitEnabled = false;

    std::vector<IMU::Point> interpolateImu(const std::vector<Eigen::Vector3f> &v_imu, const std::vector<double> &v_imu_timestamp_s, const double &curr_frame_timestamp_s, const double &prev_frame_timestamp_s, const bool &is_accel) const;
    void combineImu(std::vector<IMU::Point> &v_accel, std::vector<IMU::Point>& v_gyro, std::vector<IMU::Point> &v_imu_combined);

    std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFromLastKF;

    bool mbStationaryImuInit = false; // TO DO: include in OdometrySettings class 
    bool mbMonocular;

    void initializeIMU(ImuInitializater::ImuInitType priorG, ImuInitializater::ImuInitType priorA, bool bFIBA);

    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;

    bool mbFastInit;

    void scaleRefinement();
};


} // namespace MORB_SLAM