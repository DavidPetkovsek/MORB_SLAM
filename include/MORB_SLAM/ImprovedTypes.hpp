#pragma once

#include <memory>
#include <utility>
#include <iostream>
#include <array>

namespace MORB_SLAM{

class System;
class Atlas;
class Tracking;
typedef std::shared_ptr<System> System_ptr;
typedef std::weak_ptr<System> System_wptr;
typedef std::shared_ptr<Atlas> Atlas_ptr;
typedef std::weak_ptr<Atlas> Atlas_wptr;
typedef std::shared_ptr<Tracking> Tracking_ptr;
typedef std::weak_ptr<Tracking> Tracking_wptr;

typedef std::pair<int, int> IntPair;


class TrackingState{
  static size_t size;
  size_t id;
  TrackingState(const char *toString):id{++size}, toString{toString} {}
public:
  const char *toString;

  static const TrackingState SYSTEM_NOT_READY;
  static const TrackingState NO_IMAGES_YET;
  static const TrackingState NOT_INITIALIZED;
  static const TrackingState OK;
  static const TrackingState RECENTLY_LOST;
  static const TrackingState LOST;
  static const TrackingState OK_KLT;
  static const std::array<TrackingState, 7> STATES;

  bool operator==(const TrackingState &other) const { return id == other.id; }
  std::ostream &operator<<(std::ostream &os){
    os << toString;
    return os;
  }
};
size_t TrackingState::size(0);
const TrackingState TrackingState::SYSTEM_NOT_READY("SYSTEM_NOT_READY");
const TrackingState TrackingState::NO_IMAGES_YET("NO_IMAGES_YET");
const TrackingState TrackingState::NOT_INITIALIZED("NOT_INITIALIZED");
const TrackingState TrackingState::OK("OK");
const TrackingState TrackingState::RECENTLY_LOST("RECENTLY_LOST");
const TrackingState TrackingState::LOST("LOST");
const TrackingState TrackingState::OK_KLT("OK_KLT");
const std::array<TrackingState, 7> TrackingState::STATES{SYSTEM_NOT_READY, NO_IMAGES_YET, NOT_INITIALIZED, OK, RECENTLY_LOST, LOST, OK_KLT};



class CameraType{
  static size_t size;
  size_t id;
  CameraType(bool isInertial, bool hasMulticam, const char *toString):id{++size}, _isInertial{isInertial}, _hasMulticam{hasMulticam}, toString{toString} {}
  bool _isInertial;
  bool _hasMulticam;
public:
  const char *toString;

  static const CameraType MONOCULAR;
  static const CameraType STEREO;
  static const CameraType RGBD;
  static const CameraType IMU_MONOCULAR;
  static const CameraType IMU_STEREO;
  static const CameraType IMU_RGBD;
  static const std::array<CameraType, 6> TYPES;

  bool isInertial() const { return _isInertial; }
  bool hasMulticam() const { return _hasMulticam; }
  bool operator==(const CameraType &other) const { return id == other.id; }
};
std::ostream &operator<<(std::ostream &os, const CameraType &t){
    os << t.toString;
    return os;
}
size_t CameraType::size(0);
const CameraType CameraType::MONOCULAR(false, false, "MONOCULAR");
const CameraType CameraType::STEREO(false, true, "STEREO");
const CameraType CameraType::RGBD(false, true, "RGBD");
const CameraType CameraType::IMU_MONOCULAR(true, false, "IMU_MONOCULAR");
const CameraType CameraType::IMU_STEREO(true, true, "IMU_STEREO");
const CameraType CameraType::IMU_RGBD(true, true, "IMU_RGBD");
const std::array<CameraType, 6> CameraType::TYPES{MONOCULAR, STEREO, RGBD, IMU_MONOCULAR, IMU_STEREO, IMU_RGBD};


namespace ImuInitializater{
  enum ImuInitType{ //enum values are used for math stuff -- DO NOT CHANGE
    MONOCULAR_INIT_A=10000000000,
    STEREO_INIT_A=100000,
    VIBA1_A=100000, // VIBA = visual-inertial bundle adjustment
    VIBA2_A=0,
    DEFAULT_A=100,
    MONOCULAR_INIT_G=100,
    STEREO_INIT_G=100,
    VIBA1_G=1,
    VIBA2_G=0,
    DEFAULT_G=1000000,
  };
}
}