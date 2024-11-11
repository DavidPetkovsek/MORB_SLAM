#pragma once

#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/InertialOdometry/ImuTypes.h"

namespace MORB_SLAM {


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
};


} // namespace MORB_SLAM
