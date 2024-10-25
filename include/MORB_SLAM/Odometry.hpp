#pragma once

#include <memory>

namespace MORB_SLAM {

class Frame;
class KeyFrame;
class Atlas;

class Odometry {
public:
    // virtual ~Odometry();

    virtual bool GrabOdom(double curr_timestamp, double prev_timestamp) = 0;
    virtual void PreintegrateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf) = 0;
    virtual bool PredictStateOdom(Frame &curr_frame, Frame &prev_frame, std::shared_ptr<KeyFrame> last_kf, bool map_updated) = 0;
    virtual bool ReadyForStereoInitialization(Frame &curr_frame, Frame &last_frame, std::shared_ptr<Atlas> p_atlas) = 0;
    virtual void NewKeyFrame(std::shared_ptr<KeyFrame> ref_kf) = 0;
    virtual void NewMap() = 0;
};


} //namespace MORB_SLAM