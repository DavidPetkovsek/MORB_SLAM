#pragma once
#include <opencv2/opencv.hpp>
#include <optional>
#ifdef FactoryEngine
#include <apps/morb_sophus/se3.hpp>
#else
#include <sophus/se3.hpp>
#endif

namespace MORB_SLAM{

struct Packet {
    std::optional<Sophus::SE3f> mapPose; // Transformation from world to camera frame
    std::optional<Sophus::SE3f> deltaPose; // Transformation from last frame to current frame, where c1 is current frame and c2 is last frame

    Packet(std::optional<Sophus::SE3f> mapPose = std::nullopt, std::optional<Sophus::SE3f> deltaPose = std::nullopt);
    virtual ~Packet();
};
struct InertialPacket{};

struct StereoPacket : public Packet, public InertialPacket {
    cv::Mat imgLeft;
    cv::Mat imgRight;
    StereoPacket(const cv::Mat &imgLeft, const cv::Mat &imgRight);
    StereoPacket(const Sophus::SE3f &mapPose, const cv::Mat &imgLeft, const cv::Mat &imgRight);
    StereoPacket(const Sophus::SE3f &mapPose, const Sophus::SE3f &deltaPose, const cv::Mat &imgLeft, const cv::Mat &imgRight);
};

struct MonoPacket : public Packet, public InertialPacket {
    cv::Mat img;
    MonoPacket(const cv::Mat &img);
    MonoPacket(const Sophus::SE3f &pose, const cv::Mat &img);
};

struct RGBDPacket : public Packet, public InertialPacket {
    cv::Mat img;
    cv::Mat depthImg;
    RGBDPacket(const cv::Mat &img, const cv::Mat &depthImg);
    RGBDPacket(const Sophus::SE3f &pose, const cv::Mat &img, const cv::Mat &depthImg);
};
}