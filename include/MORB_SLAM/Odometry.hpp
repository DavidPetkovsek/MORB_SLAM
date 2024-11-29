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
    virtual ~Odometry() {}

    void SetLocalMapper(const std::shared_ptr<LocalMapping> &pLocalMapper);
    void SetTracker(const std::shared_ptr<Tracking> &pTracker);
    void SetAtlas(const std::shared_ptr<Atlas> &pAtlas);

    /*
        If custom members are required for the Frame and KeyFrame class, define structs that derive from ExternalFrameData and ExternalKeyFrameData respectively
    */

    // Default external data assigned to a Frame when it is created at the start of the Track loop
    virtual std::shared_ptr<ExternalFrameData> DefaultExternalFrameData() { return std::shared_ptr<ExternalFrameData>(nullptr); }

    // Default external data assigned to a Frame when it is created at the start of the Track loop, based on the latest KeyFrame tracked
    virtual std::shared_ptr<ExternalFrameData> DefaultExternalFrameData(std::shared_ptr<KeyFrame> p_latest_kf) { return std::shared_ptr<ExternalFrameData>(nullptr); }

    // Default external data assigned to a KeyFrame that has just been promoted from a Frame at the end of the Track loop
    virtual std::shared_ptr<ExternalKeyFrameData> DefaultExternalKeyFrameData(Frame &curr_frame) { return std::shared_ptr<ExternalKeyFrameData>(nullptr); }

    /*
        Core functionality used in the Track loop
    */
    // Called when a new Frame comes into the Tracking loop. Retrieve relevant odometry data from the previous Frame timestamp up until the current Frame timestamp for processing. Perform any necessary pre processing.
    virtual bool GrabOdom(double curr_timestamp, double prev_timestamp) = 0;

    // Called at the start of the Track loop, shortly after Odometry::GrabOdom() is called. Preintegrate the odometry data. Returns true if preintegration is successful.
    virtual bool PreintegrateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf) = 0;

    // Called within Tracking::StereoInitialization() (after Odometry::PreintegrateOdom()) if the Tracking state is NOT_INITIALIZED. Returns true if Tracking can proceed in intializing the map.
    virtual bool ReadyForStereoInitialization(Frame &curr_frame, Frame &last_frame) = 0;

    // Called if Odometry::ReadyForStereoInitialization() returns true. Any logic required when the map is created at startup, when a new map is created, or the current map is reset should go here
    virtual void StereoInitialization(Frame &curr_frame) = 0;

    // ***Monocular SLAM is currently not maintained
    virtual bool ReadyForMonocularInitialization(Frame &curr_frame, Frame &last_frame) = 0;
    virtual void InitialMapMonocular(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> initial_kf) = 0;

    // Called during the tracking loop to get an initial extimate of the frame (before a local pose optimization is performed in Tracking::TrackLocalMap())
    virtual bool PredictStateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf, bool map_updated) = 0;

    // A pose optimization that occurs during Tracking::TrackLocalMap() to determine the pose of the current frame 
    virtual void TrackLocalMapPoseOptimization(Frame &curr_frame, bool &b_map_updated, bool reloc_recently) = 0; // TO DO: b_map_updated and reloc_recently are TEMPORARY parameters, to be reworked

    // Called when a Frame become a KeyFrame in the Tracking thread.
    virtual void NewKeyFrameEvent(std::shared_ptr<KeyFrame> ref_kf) = 0;

    /*
        Core functionality used in the LocalMapping thread
    */
    // Called in every LocalMapping loop once the odom source is initialized. Performs a bundle adjustment on a local window of keyframes close to the current keyframe.
    virtual void LocalBundleAdjustment(std::shared_ptr<KeyFrame> curr_kf, bool &b_abortBA) = 0;

    // Called once in the LocalMapping loop if the odometry source has not been initialized yet i.e Map::mbOdomInitialized == false. Any odometry parameters that need to be initialized can be done here.
    virtual void InitializeOdom() = 0;

    // Called in every LocalMapping loop once the odometry source has been initialized. Any refinement in the odometry parameters can be done here.
    virtual void PostInitializeOdom() = 0;

    // KeyFrame culling will not occur unless this method returns true. Any custom conditions for KeyFrame culling are defined here.
    virtual bool ReadyForKeyFrameCulling(const std::shared_ptr<KeyFrame> &curr_kf) = 0;

    /*
        Core functionality in the LoopClosing thread
    */
    // Called in LoopClosing thread when a map merge is performed, but the current map is not fully mature yet.
    virtual void InitializeMergeMap(const std::shared_ptr<Map> &curr_map) = 0;

    // Called during a map merge. Performs a bundle adjustment on the welding window.
    virtual void MergeLocalBundleAdjustment(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> merge_kf, std::shared_ptr<Map> curr_map, KeyFrameAndPose& corr_poses) = 0;

    // Called during a map merge. Uses the current KeyFrame to update the Tracking Frames.
    virtual void MergeLocalUpdateTrackingFrame(std::shared_ptr<KeyFrame> pCurrentKF) = 0;

    // Called during a loop correction. Updates the essential graph
    virtual void LoopClosingOptimizeEssentialGraph(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF, const KeyFrameAndPose& NonCorrectedSim3, const KeyFrameAndPose& CorrectedSim3, const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>& LoopConnections) = 0;
    
    // Called in LoopClosing::RunGlobalBundleAdjustment(), which is callled during loop correction.
    virtual void GlobalBundleAdjustment(std::shared_ptr<Map> pMap, const long unsigned int nLoopId, bool &mbStopGBA) = 0;


protected:
    std::shared_ptr<Atlas> mpAtlas;
    std::weak_ptr<Tracking> mwpTracker;

    /*
        Methods to interface with the LocalMapping thread. Should only be used in the LocalMapping related functions.
    */
    // Returns true if the LocalMapper was sent a request to reset (from elsewhere in the system)
    bool LocalMappingResetRequested();

    // Sets the LocalMapper to an initializing state. Notifies other threads that the odometry is being initialized. During initializing, KeyFrames are not added to the map.
    void LocalMappingSetInitializing(bool is_initializing);

    // The Tracking thread may have inserted KeyFrame into the LocalMapping queue while any of the LocalMapping functions above were called
    // This method processes each KeyFrame in the queue and adds it to the map
    void LocalMappingProcessKeyFramesInQueue(std::vector<std::shared_ptr<KeyFrame>> &vpKF);
    
    void LocalMappingSetPoseReverseAxisFlip(Sophus::SE3f pose);
    
    // Sets all KeyFrames in the LocalMapping queue as bad and erases them
    void LocalMappingSetNewKeyFramesBad();

    // Get the most current KeyFrame in the LocalMapping thread
    std::shared_ptr<KeyFrame> LocalMappingGetCurrentKeyFrame();

   // After a global bundle adjustment is performed, and new keyframes added to the map while the BA occured needs to be updated
   void LocalMappingCorrectMapAfterGBA(const std::shared_ptr<Map> &pMap, unsigned long GBAid);

private:
    std::weak_ptr<LocalMapping> mwpLocalMapper;

};


} //namespace MORB_SLAM