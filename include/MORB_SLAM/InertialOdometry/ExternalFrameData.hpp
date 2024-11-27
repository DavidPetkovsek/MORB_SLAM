#pragma once

#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/InertialOdometry/ImuTypes.h"

namespace MORB_SLAM {

class ConstraintPoseImu;

struct InertialFrameData : public ExternalFrameData {
    InertialFrameData()
        : mpImuPreintegrated(nullptr),
          mpImuPreintegratedFrame(nullptr),
          mbImuPreintegrated(false),
          mpcpi(nullptr),
          mpMutexImu(std::make_shared<std::mutex>()) { }

    InertialFrameData(IMU::Calib imuCalib)
        : mImuCalib(imuCalib),
          mpImuPreintegrated(nullptr),
          mpImuPreintegratedFrame(nullptr),
          mbImuPreintegrated(false),
          mpcpi(nullptr),
          mpMutexImu(std::make_shared<std::mutex>()) {}

    IMU::Bias mImuBias;
    IMU::Calib mImuCalib;
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
    std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFrame;
    bool mbImuPreintegrated;
    std::shared_ptr<ConstraintPoseImu> mpcpi;
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

    void SetNewBias(const IMU::Bias &b) {
        mImuBias = b;
        if (mpImuPreintegrated) mpImuPreintegrated->SetNewBias(b);
    }
};


struct InertialKeyFrameData : public ExternalKeyFrameData {
    // template<class Archive>
    // void serialize(Archive & ar, unsigned int version) { 
    //     ar & boost::serialization::base_object<ExternalKeyFrameData>(*this);
    //     ar& mImuBias;
    //     ar& mBackupImuPreintegrated;
    //     ar& mImuCalib;
    // }

    // void PreSave() override {
    //     if (mpImuPreintegrated) mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
    // }

    // void PostLoad() override {
    //     mpImuPreintegrated = std::make_shared<IMU::Preintegrated>(std::move(&mBackupImuPreintegrated));
    // }

    InertialKeyFrameData(IMU::Calib imuCalib)
        : mpImuPreintegrated(nullptr),
          mImuCalib(imuCalib) { }

    InertialKeyFrameData(InertialFrameData frame_data)
        : mpImuPreintegrated(frame_data.mpImuPreintegrated),
          mImuCalib(frame_data.mImuCalib),
          mImuBias(frame_data.mImuBias) { }

    std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
    // IMU::Preintegrated mBackupImuPreintegrated;
    IMU::Calib mImuCalib;
    IMU::Bias mImuBias;
    IMU::Bias mBiasGBA;

    void SetNewBias(const IMU::Bias& b) {
        if(std::shared_ptr<std::mutex> pMutexPose = mpMutexPose.lock()) {
            std::unique_lock<std::mutex> lock(*pMutexPose);
            mImuBias = b;
            if (mpImuPreintegrated) mpImuPreintegrated->SetNewBias(b);
        }
    }

    Eigen::Vector3f GetGyroBias() {
        if(std::shared_ptr<std::mutex> pMutexPose = mpMutexPose.lock()) {
            std::unique_lock<std::mutex> lock(*pMutexPose);
            return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
        } else {
            std::cout<<"ERROR: Getting gyro bias from a KeyFrame that doesn't exist." <<std::endl;
            return Eigen::Vector3f();
        }
    }

    Eigen::Vector3f GetAccBias() {
        if(std::shared_ptr<std::mutex> pMutexPose = mpMutexPose.lock()) {
            std::unique_lock<std::mutex> lock(*pMutexPose);
            return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
        } else {
            std::cout<<"ERROR: Getting accel bias from a KeyFrame that doesn't exist." <<std::endl;
            return Eigen::Vector3f();
        }
    }

    IMU::Bias GetImuBias() {
        if(std::shared_ptr<std::mutex> pMutexPose = mpMutexPose.lock()) {
            std::unique_lock<std::mutex> lock(*pMutexPose);
            return mImuBias;
        } else {
            std::cout<<"ERROR: Getting imu bias from a KeyFrame that doesn't exist." <<std::endl;
            return IMU::Bias();
        }
    }

    void MergePrevious(std::shared_ptr<ExternalKeyFrameData> &eKFd_prev) override {
        std::shared_ptr<InertialKeyFrameData> inertial_eKFd_prev = std::static_pointer_cast<InertialKeyFrameData>(eKFd_prev);
        if(mpImuPreintegrated && inertial_eKFd_prev->mpImuPreintegrated) {
            mpImuPreintegrated->MergePrevious(inertial_eKFd_prev->mpImuPreintegrated);
        }
    }

    void UpdateChildSpanningTree() override {
        mBiasGBA = GetImuBias();
    }

    void UpdateParentSpanningTree() override {
        SetNewBias(mBiasGBA);
    }
};


} // namespace MORB_SLAM
