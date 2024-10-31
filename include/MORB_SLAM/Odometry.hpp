#pragma once

#include <memory>
#include <vector>
#include <list>
#include <sophus/se3.hpp>

#include "MORB_SLAM/ImuTypes.h"
#include "MORB_SLAM/ImprovedTypes.hpp"

namespace MORB_SLAM {

class Frame;
class KeyFrame;
class LocalMapping;
class Tracking;
class Atlas;

class Odometry {
public:
    // virtual ~Odometry();

    void SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper);
    void SetTracker(std::shared_ptr<Tracking> pTracker);
    void SetAtlas(const std::shared_ptr<Atlas> &pAtlas);

    virtual bool GrabOdom(double curr_timestamp, double prev_timestamp) = 0;
    virtual void PreintegrateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf) = 0;
    virtual bool PredictStateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf, bool map_updated) = 0;
    virtual bool ReadyForStereoInitialization(Frame &curr_frame, Frame &last_frame) = 0;
    virtual bool ReadyForMonocularInitialization(Frame &curr_frame, Frame &last_frame) = 0;
    virtual void InitialMapMonocular(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> initial_kf) = 0;
    virtual void NewKeyFrame(std::shared_ptr<KeyFrame> ref_kf) = 0;
    virtual void NewMap() = 0;
    virtual void LocalOdomBA(std::shared_ptr<KeyFrame> curr_kf, bool &b_abortBA) = 0;
    virtual void InitializeOdom() = 0;
    virtual void PostInitializeOdom() = 0;

protected:
    // Atlas
    std::shared_ptr<Atlas> mpAtlas;

    // Tracking
    int TrackingGetMatchesInliers();
    void TrackingLockPreTeleportTranslation(bool is_locked);
    void TrackingSetTeleported(bool is_teleported);
    void TrackingUpdateFrameOdom(const float s, const IMU::Bias& b, std::shared_ptr<KeyFrame> curr_kf); // TODO: Rework this
    void TrackingSetState(TrackingState state);
    TrackingState TrackingGetState();

    // Local Mapping
    bool LocalMappingResetRequested();
    void LocalMappingSetInitializing(bool is_initializing);
    void LocalMappingProcessKeyFramesInQueue(std::vector<std::shared_ptr<KeyFrame>> &vpKF);
    void LocalMappingSetTimeInit(float t_init);
    float LocalMappingGetTimeInit();
    void LocalMappingIncrementTimeInit(float t_increment);
    void LocalMappingSetPoseReverseAxisFlip(Sophus::SE3f pose);
    void LocalMappingSetNewKeyFramesBad();
    std::shared_ptr<KeyFrame> LocalMappingGetCurrentKeyFrame();

private:
    std::weak_ptr<LocalMapping> mwpLocalMapper;
    std::weak_ptr<Tracking> mwpTracker;

};


} //namespace MORB_SLAM