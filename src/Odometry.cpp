#include "MORB_SLAM/Odometry.hpp"

#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {


    void Odometry::SetLocalMapper(const std::shared_ptr<LocalMapping> &pLocalMapper) { mwpLocalMapper = pLocalMapper; };
    void Odometry::SetTracker(const std::shared_ptr<Tracking> &pTracker) { mwpTracker = pTracker; };
    void Odometry::SetAtlas(const std::shared_ptr<Atlas> &pAtlas) { mpAtlas = pAtlas; };

    // Local Mapping
    bool Odometry::LocalMappingResetRequested() {
        if (std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            return pLocalMapper->mbResetRequested;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingSetInitializing(bool is_initializing) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            pLocalMapper->bInitializing = is_initializing;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingProcessKeyFramesInQueue(std::vector<std::shared_ptr<KeyFrame>> &vpKF) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock()) {
            while (pLocalMapper->CheckNewKeyFrames()) {
                pLocalMapper->ProcessNewKeyFrame();
                vpKF.push_back(pLocalMapper->mpCurrentKeyFrame);
            }
        } else {
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
        }
    }

    void Odometry::LocalMappingSetTimeInit(float t_init) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            pLocalMapper->mTinit = t_init;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    float Odometry::LocalMappingGetTimeInit() {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            return pLocalMapper->mTinit;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingIncrementTimeInit(float t_increment) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            pLocalMapper->mTinit += t_increment;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingSetPoseReverseAxisFlip(Sophus::SE3f pose) {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            pLocalMapper->mPoseReverseAxisFlip = pose;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }

    void Odometry::LocalMappingSetNewKeyFramesBad() {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock()) {
            for (std::shared_ptr<KeyFrame> newKeyFrame : pLocalMapper->mlNewKeyFrames) {
                newKeyFrame->SetBadFlag();
            }
            pLocalMapper->mlNewKeyFrames.clear();
        } else {
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
        }
    }

    std::shared_ptr<KeyFrame> Odometry::LocalMappingGetCurrentKeyFrame() {
        if(std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock())
            return pLocalMapper->mpCurrentKeyFrame;
        else
            throw std::runtime_error("ERROR: Cannot access 'LocalMapping' object because it has been destroyed.");
    }


} //namespace MORB_SLAM