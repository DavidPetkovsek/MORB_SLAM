#pragma once

#include <memory>
#include <vector>
#include <list>
#include <sophus/se3.hpp>
#include <map>
#include <set>
#include <g2o/types/sim3.h>

#include "MORB_SLAM/ImprovedTypes.hpp"

namespace MORB_SLAM {

class Frame;
class KeyFrame;
class LocalMapping;
class Tracking;
class Atlas;
class Map;
class ExternalFrameData;
class ExternalKeyFrameData;
typedef std::map<std::shared_ptr<KeyFrame>,g2o::Sim3,std::less<std::shared_ptr<KeyFrame>>, Eigen::aligned_allocator<std::pair<std::shared_ptr<KeyFrame> const, g2o::Sim3>>> KeyFrameAndPose;

class Odometry {
public:
    // virtual ~Odometry();

    void SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper);
    void SetTracker(std::shared_ptr<Tracking> pTracker);
    void SetAtlas(const std::shared_ptr<Atlas> &pAtlas);

    virtual std::shared_ptr<ExternalFrameData> DefaultExternalFrameData() = 0;
    virtual std::shared_ptr<ExternalFrameData> DefaultExternalFrameData(std::shared_ptr<KeyFrame> p_curr_kf) = 0;
    virtual std::shared_ptr<ExternalKeyFrameData> DefaultExternalKeyFrameData(Frame &curr_frame) = 0;
    virtual bool GrabOdom(double curr_timestamp, double prev_timestamp) = 0;
    virtual bool TrackingInitKeyFrameData(Frame &curr_frame, std::shared_ptr<KeyFrame> new_kf) = 0;
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
    virtual void MergeLocalInitializeMap(const std::shared_ptr<Map> &curr_map) = 0;
    virtual void MergeOdomBA(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> merge_kf, std::shared_ptr<Map> curr_map, KeyFrameAndPose& corr_poses) = 0;
    virtual void LoopClosingOptimizeEssentialGraph(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF, const KeyFrameAndPose& NonCorrectedSim3, const KeyFrameAndPose& CorrectedSim3, const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>& LoopConnections) = 0;
    virtual void MergeLocalUpdateTrackingFrame(std::shared_ptr<KeyFrame> pCurrentKF) = 0;
    virtual void TrackLocalMapPoseOptimization(Frame &curr_frame, bool &b_map_updated, bool reloc_recently) = 0; // TO DO: b_map_updated and reloc_recently are TEMPORARY parameters, to be reworked

protected:
    // Atlas
    std::shared_ptr<Atlas> mpAtlas;

    // Tracking
    std::weak_ptr<Tracking> mwpTracker;
    // TO-DO: Remove. Tracker was initially a private member of Odometry, so any child classes can't access it. This has been changed, so the below methods aren't needed anymore.
    int TrackingGetMatchesInliers();
    void TrackingLockPreTeleportTranslation(bool is_locked);
    void TrackingSetTeleported(bool is_teleported);
    // void TrackingUpdateFrameOdom(const float s, const IMU::Bias& b, std::shared_ptr<KeyFrame> curr_kf); // TODO: Rework this
    void TrackingSetState(TrackingState state);
    TrackingState TrackingGetState();
    std::shared_ptr<KeyFrame> TrackingGetLastKeyFrame();

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

};


} //namespace MORB_SLAM