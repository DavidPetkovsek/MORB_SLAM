#include "MORB_SLAM/Packet.hpp"



namespace MORB_SLAM{

Packet::Packet()
    : trackingState{TrackingState::SYSTEM_NOT_READY},
      mapUpdated{false},
      mapPose{std::nullopt},
      deltaPose{std::nullopt} {}

Packet::Packet(const TrackingState &trackingState, const bool &mapUpdated, std::optional<Sophus::SE3f> mapPose, std::optional<Sophus::SE3f> deltaPose)
    : trackingState{trackingState},
      mapUpdated{mapUpdated},
      mapPose{mapPose},
      deltaPose{deltaPose} {}

Packet::~Packet(){}

StereoPacket::StereoPacket(const cv::Mat &imgLeft, const cv::Mat &imgRight): Packet(), imgLeft{imgLeft}, imgRight{imgRight} {}
StereoPacket::StereoPacket(const TrackingState &trackingState, const bool &mapUpdated, const cv::Mat &imgLeft, const cv::Mat &imgRight): Packet(trackingState, mapUpdated), imgLeft{imgLeft}, imgRight{imgRight} {}

MonoPacket::MonoPacket(const cv::Mat &img): Packet(), img{img} {}
MonoPacket::MonoPacket(const TrackingState &trackingState, const bool &mapUpdated, const cv::Mat &img): Packet(trackingState, mapUpdated), img{img} {}

RGBDPacket::RGBDPacket(const cv::Mat &img, const cv::Mat &depthImg): Packet(), img{img}, depthImg{depthImg} {}
RGBDPacket::RGBDPacket(const TrackingState &trackingState, const bool &mapUpdated, const cv::Mat &img, const cv::Mat &depthImg): Packet(trackingState, mapUpdated), img{img}, depthImg{depthImg} {}

}
