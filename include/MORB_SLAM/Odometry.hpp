#pragma once

#include <memory>

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
    virtual void NewKeyFrame(std::shared_ptr<KeyFrame> ref_kf) = 0;
    virtual void NewMap() = 0;
    virtual void LocalOdomBA(std::shared_ptr<KeyFrame> curr_kf, bool &b_abortBA, float &Tinit) = 0;

protected:
    // Atlas
    int AtlasNumMaps();

    // Tracking
    int TrackingGetMatchesInliers();

private:
    std::weak_ptr<LocalMapping> mwpLocalMapper;
    std::weak_ptr<Tracking> mwpTracker;
    std::shared_ptr<Atlas> mpAtlas;

};


} //namespace MORB_SLAM