#pragma once

namespace MORB_SLAM {

class Frame;
class KeyFrame;

class Odometry {
public:
    // virtual ~Odometry();

    virtual bool GrabOdom(double curr_timestamp, double prev_timestamp) = 0;
    virtual bool PreintegrateOdom(Frame &curr_frame, Frame &prev_frame) = 0;
};


} //namespace MORB_SLAM