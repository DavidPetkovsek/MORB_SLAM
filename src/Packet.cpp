#include "MORB_SLAM/Packet.hpp"



namespace MORB_SLAM{

Packet::Packet(std::optional<Sophus::SE3f> mapPose, std::optional<Sophus::SE3f> deltaPose) : mapPose{mapPose}, deltaPose{deltaPose} {}
Packet::~Packet(){}

StereoPacket::StereoPacket(const cv::Mat &imgLeft, const cv::Mat &imgRight): Packet(), imgLeft{imgLeft}, imgRight{imgRight} {}
StereoPacket::StereoPacket(const Sophus::SE3f &mapPose, const cv::Mat &imgLeft, const cv::Mat &imgRight): Packet(mapPose), imgLeft{imgLeft}, imgRight{imgRight} {}
StereoPacket::StereoPacket(const Sophus::SE3f &mapPose, const Sophus::SE3f &deltaPose, const cv::Mat &imgLeft, const cv::Mat &imgRight): Packet(mapPose, deltaPose), imgLeft{imgLeft}, imgRight{imgRight} {}

MonoPacket::MonoPacket(const cv::Mat &img): Packet(), img{img} {}
MonoPacket::MonoPacket(const Sophus::SE3f &pose, const cv::Mat &img): Packet(pose), img{img} {}

RGBDPacket::RGBDPacket(const cv::Mat &img, const cv::Mat &depthImg): Packet(), img{img}, depthImg{depthImg} {}
RGBDPacket::RGBDPacket(const Sophus::SE3f &pose, const cv::Mat &img, const cv::Mat &depthImg): Packet(pose), img{img}, depthImg{depthImg} {}

}
