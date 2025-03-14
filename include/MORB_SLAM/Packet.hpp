#pragma once
#include <opencv2/opencv.hpp>
#include <optional>
#ifdef FactoryEngine
#include <apps/morb_sophus/se3.hpp>
#else
#include <sophus/se3.hpp>
#endif

#include "MORB_SLAM/ImprovedTypes.hpp"

namespace MORB_SLAM{

struct Packet {
    TrackingState trackingState;
    bool mapUpdated; // did a LocalBA, GBA, MapMerge, or LoopClose occur right before this frame?
    std::optional<Sophus::SE3f> mapPose; // Tcw, Transformation from world to camera frame
    std::optional<Sophus::SE3f> deltaPose; // Tc1c2, Transformation from last frame to current frame, where c1 is current frame and c2 is last frame

    Packet();
    Packet(const TrackingState &trackingState, const bool &mapUpdated, std::optional<Sophus::SE3f> mapPose = std::nullopt, std::optional<Sophus::SE3f> deltaPose = std::nullopt);
    virtual ~Packet();
};
struct InertialPacket{
    std::optional<Eigen::Vector3f> velocity;
};

struct StereoPacket : public Packet, public InertialPacket {
    cv::Mat imgLeft;
    cv::Mat imgRight;
    StereoPacket(const cv::Mat &imgLeft, const cv::Mat &imgRight);
    StereoPacket(const TrackingState &trackingState, const bool &mapUpdated, const cv::Mat &imgLeft, const cv::Mat &imgRight);
};

struct MonoPacket : public Packet, public InertialPacket {
    cv::Mat img;
    MonoPacket(const cv::Mat &img);
    MonoPacket(const TrackingState &trackingState, const bool &mapUpdated, const cv::Mat &img);
};

struct RGBDPacket : public Packet, public InertialPacket {
    cv::Mat img;
    cv::Mat depthImg;
    RGBDPacket(const cv::Mat &img, const cv::Mat &depthImg);
    RGBDPacket(const TrackingState &trackingState, const bool &mapUpdated, const cv::Mat &img, const cv::Mat &depthImg);
};
}