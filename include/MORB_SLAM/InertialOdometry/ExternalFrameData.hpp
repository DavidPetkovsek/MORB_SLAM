#pragma once

#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/InertialOdometry/ImuTypes.h"

namespace MORB_SLAM {


struct InertialFrameData : public ExternalFrameData {
    // default constructor
    InertialFrameData()
        : mpImuPreintegrated(nullptr),
          mpImuPreintegratedFrame(nullptr),
          mbImuPreintegrated(false),
          mpcpi(nullptr)
    {
        mpMutexImu = std::make_shared<std::mutex>();
    }

    InertialFrameData(IMU::Calib imuCalib)
        : mImuCalib(imuCalib),
          mpImuPreintegrated(nullptr),
          mpImuPreintegratedFrame(nullptr),
          mbImuPreintegrated(false),
          mpcpi(nullptr)
    {
        mpMutexImu = std::make_shared<std::mutex>();
    } 
    IMU::Bias mImuBias;
    IMU::Calib mImuCalib;
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFrame;
    bool mbImuPreintegrated;
    std::shared_ptr<std::mutex> mpMutexImu;
    std::shared_ptr<ConstraintPoseImu> mpcpi;

    bool imuIsPreintegrated() {
        std::unique_lock<std::mutex> lock(*mpMutexImu);
        return mbImuPreintegrated;
    }
    
    void setIntegrated() {
        while (!mpMutexImu)
            mpMutexImu = std::make_shared<std::mutex>();

        std::unique_lock<std::mutex> lock(*mpMutexImu);
        mbImuPreintegrated = true;
    }

    void SetNewBias(const IMU::Bias &b) {
        mImuBias = b;
        if (mpImuPreintegrated) mpImuPreintegrated->SetNewBias(b);
    }

};


struct InertialKeyFrameData : public ExternalKeyFrameData {
    InertialKeyFrameData(IMU::Calib imuCalib)
        : mpImuPreintegrated(nullptr),
          mImuCalib(imuCalib) { }

    InertialKeyFrameData(InertialFrameData frame_data)
        : mpImuPreintegrated(frame_data.mpImuPreintegrated),
          mImuCalib(frame_data.mImuCalib),
          mImuBias(frame_data.mImuBias)
    { }

    void SetNewBias(const IMU::Bias& b);
    Eigen::Vector3f GetGyroBias() {
        std::unique_lock<std::mutex> lock(mMutexImu);
        return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
    }

    Eigen::Vector3f GetAccBias() {
        std::unique_lock<std::mutex> lock(mMutexImu);
        return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
    }

    IMU::Bias GetImuBias() {
        std::unique_lock<std::mutex> lock(mMutexImu);
        return mImuBias;
    }
    bool bImu;
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
    IMU::Calib mImuCalib;
    IMU::Bias mImuBias;
    std::mutex mMutexImu;

    void MergePrevious(std::shared_ptr<ExternalKeyFrameData> &eKFd_prev) override {
        std::shared_ptr<InertialKeyFrameData> inertial_eKFd_prev = std::static_pointer_cast<InertialKeyFrameData>(eKFd_prev);
        if(mpImuPreintegrated && inertial_eKFd_prev->mpImuPreintegrated) {
            mpImuPreintegrated->MergePrevious(inertial_eKFd_prev->mpImuPreintegrated);
        }
    }
};


} // namespace MORB_SLAM
