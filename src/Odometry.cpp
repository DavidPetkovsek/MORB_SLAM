#include "MORB_SLAM/Odometry.hpp"

#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {


    void Odometry::SetLocalMapper(const std::shared_ptr<LocalMapping> &pLocalMapper) { mwpLocalMapper = pLocalMapper; };
    void Odometry::SetTracker(const std::shared_ptr<Tracking> &pTracker) { mwpTracker = pTracker; };
    void Odometry::SetAtlas(const std::shared_ptr<Atlas> &pAtlas) { mpAtlas = pAtlas; };

    // Local Mapping
    bool Odometry::LocalMappingResetRequested() { std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock(); return pLocalMapper->mbResetRequested; }
    void Odometry::LocalMappingSetInitializing(bool is_initializing) { std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock(); pLocalMapper->bInitializing = is_initializing; }
    void Odometry::LocalMappingProcessKeyFramesInQueue(std::vector<std::shared_ptr<KeyFrame>> &vpKF) {
        std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock();
        while (pLocalMapper->CheckNewKeyFrames()) {
            pLocalMapper->ProcessNewKeyFrame();
            vpKF.push_back(pLocalMapper->mpCurrentKeyFrame);
        }
    }
    void Odometry::LocalMappingSetTimeInit(float t_init) { std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock(); pLocalMapper->mTinit = t_init; }
    float Odometry::LocalMappingGetTimeInit() { std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock(); return pLocalMapper->mTinit; }
    void Odometry::LocalMappingIncrementTimeInit(float t_increment) { std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock(); pLocalMapper->mTinit += t_increment; }
    void Odometry::LocalMappingSetPoseReverseAxisFlip(Sophus::SE3f pose) { std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock(); pLocalMapper->mPoseReverseAxisFlip = pose; }
    void Odometry::LocalMappingSetNewKeyFramesBad() {
        std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock();
        for (std::shared_ptr<KeyFrame> newKeyFrame : pLocalMapper->mlNewKeyFrames) {
            newKeyFrame->SetBadFlag();
        }
        pLocalMapper->mlNewKeyFrames.clear();
    }
    std::shared_ptr<KeyFrame> Odometry::LocalMappingGetCurrentKeyFrame() { std::shared_ptr<LocalMapping> pLocalMapper = mwpLocalMapper.lock(); return pLocalMapper->mpCurrentKeyFrame; }


} //namespace MORB_SLAM