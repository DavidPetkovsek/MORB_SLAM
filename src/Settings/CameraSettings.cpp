#include "MORB_SLAM/Settings/CameraSettings.hpp"

#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>

namespace MORB_SLAM {


CameraSettings::CameraSettings(const std::string& configFile, const CameraType& sensor)
    : sensor_(sensor),
      bNeedToUndistort_(false),
      bNeedToRectify_(false),
      bNeedToResize1_(false),
      bNeedToResize2_(false) {

  cv::FileStorage fSettings = loadFile(configFile);

  // Read first camera
  readCamera1(fSettings);
  Verbose::Log(Verbose::SUCCESS, "Loaded camera 1");

  // Read second camera if stereo (not rectified)
  if (sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
    readCamera2(fSettings);
    Verbose::Log(Verbose::SUCCESS, "Loaded camera 2");
  }

  // Read image info
  readImageInfo(fSettings);
  Verbose::Log(Verbose::SUCCESS, "Loaded image info");

  if (sensor_ == MORB_SLAM::CameraType::RGBD || sensor_ == MORB_SLAM::CameraType::IMU_RGBD) {
    readRGBD(fSettings);
    Verbose::Log(Verbose::SUCCESS, "Loaded RGB-D calibration");
  }

  if (bNeedToRectify_) {
    precomputeRectificationMaps();
    Verbose::Log(Verbose::SUCCESS, "Computed rectification maps");
  }

  Verbose::Log(Verbose::SUCCESS, "Loaded camera settings\n---");
}

void CameraSettings::readCamera1(cv::FileStorage& fSettings) {
  bool found;

  // Read camera model
  std::string cameraModel = readParameter<std::string>(fSettings, "Camera.type", found);

  std::vector<float> vCalibration;
  if (cameraModel == "PinHole") {
    cameraModelType_ = PinHole;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration1_ = std::make_shared<Pinhole>(vCalibration);

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera1.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera1.k3", found, false);
      if (found) {
        vPinHoleDistorsion1_.resize(5);
        vPinHoleDistorsion1_[4] = readParameter<float>(fSettings, "Camera1.k3", found);
      } else {
        vPinHoleDistorsion1_.resize(4);
      }
      vPinHoleDistorsion1_[0] = readParameter<float>(fSettings, "Camera1.k1", found);
      vPinHoleDistorsion1_[1] = readParameter<float>(fSettings, "Camera1.k2", found);
      vPinHoleDistorsion1_[2] = readParameter<float>(fSettings, "Camera1.p1", found);
      vPinHoleDistorsion1_[3] = readParameter<float>(fSettings, "Camera1.p2", found);
    }

    // Check if we need to correct distortion from the images
    if ((sensor_ == MORB_SLAM::CameraType::MONOCULAR || sensor_ == MORB_SLAM::CameraType::IMU_MONOCULAR) &&
        vPinHoleDistorsion1_.size() != 0) {
      bNeedToUndistort_ = true;
    }
  } else if (cameraModel == "Rectified") {
    cameraModelType_ = Rectified;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration1_ = std::make_shared<Pinhole>(vCalibration);

    // Rectified images are assumed to be ideal PinHole images (no distortion)
  } else if (cameraModel == "KannalaBrandt8") {
    cameraModelType_ = KannalaBrandt;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    if (sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
      int colBegin = readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
      int colEnd = readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
      std::vector<int> vOverlapping = {colBegin, colEnd};

      calibration1_ = std::make_shared<KannalaBrandt8>(vCalibration, vOverlapping);
    }else{
      calibration1_ = std::make_shared<KannalaBrandt8>(vCalibration);
    }
  } else {
    Verbose::Log(Verbose::FATAL, "Error: ", cameraModel, " not known");
    throw std::invalid_argument("Error: " + cameraModel + " not known");
  }
}

void CameraSettings::readCamera2(cv::FileStorage& fSettings) {
  bool found;
  std::vector<float> vCalibration;
  if (cameraModelType_ == PinHole) {
    bNeedToRectify_ = true;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration2_ = std::make_shared<Pinhole>(vCalibration);

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera2.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera2.k3", found, false);
      if (found) {
        vPinHoleDistorsion2_.resize(5);
        vPinHoleDistorsion2_[4] = readParameter<float>(fSettings, "Camera2.k3", found);
      } else {
        vPinHoleDistorsion2_.resize(4);
      }
      vPinHoleDistorsion2_[0] = readParameter<float>(fSettings, "Camera2.k1", found);
      vPinHoleDistorsion2_[1] = readParameter<float>(fSettings, "Camera2.k2", found);
      vPinHoleDistorsion2_[2] = readParameter<float>(fSettings, "Camera2.p1", found);
      vPinHoleDistorsion2_[3] = readParameter<float>(fSettings, "Camera2.p2", found);
    }
  } else if (cameraModelType_ == KannalaBrandt) {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    int colBegin = readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
    int colEnd = readParameter<int>(fSettings, "Camera2.overlappingEnd", found);
    std::vector<int> vOverlapping = {colBegin, colEnd};

    calibration2_ = std::make_shared<KannalaBrandt8>(vCalibration, vOverlapping);
  }

  // Load stereo extrinsic calibration
  if (cameraModelType_ == Rectified) {
    b_ = readParameter<float>(fSettings, "Stereo.b", found);
    bf_ = b_ * calibration1_->getParameter(0);
  } else {
    cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
    Tlr_ = Converter::toSophus(cvTlr);

    // TODO: also search for Trl and invert if necessary
    b_ = Tlr_.translation().norm();
    bf_ = b_ * calibration1_->getParameter(0);
  }

  thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
}

void CameraSettings::readImageInfo(cv::FileStorage& fSettings) {
  bool found;
  // Read original and desired image dimensions
  int originalRows = readParameter<int>(fSettings, "Camera.height", found);
  int originalCols = readParameter<int>(fSettings, "Camera.width", found);
  originalImSize_.width = originalCols;
  originalImSize_.height = originalRows;

  newImSize_ = originalImSize_;
  int newHeigh = readParameter<int>(fSettings, "Camera.newHeight", found, false);
  if (found) {
    bNeedToResize1_ = true;
    newImSize_.height = newHeigh;

    if (!bNeedToRectify_) {
      // Update calibration
      float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
      calibration1_->setParameter(calibration1_->getParameter(1) * scaleRowFactor, 1);
      calibration1_->setParameter(calibration1_->getParameter(3) * scaleRowFactor, 3);

      if ((sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) && cameraModelType_ != Rectified) {
        calibration2_->setParameter(calibration2_->getParameter(1) * scaleRowFactor, 1);
        calibration2_->setParameter(calibration2_->getParameter(3) * scaleRowFactor, 3);
      }
    }
  }

  int newWidth = readParameter<int>(fSettings, "Camera.newWidth", found, false);
  if (found) {
    bNeedToResize1_ = true;
    newImSize_.width = newWidth;

    if (!bNeedToRectify_) {
      // Update calibration
      float scaleColFactor = (float)newImSize_.width / (float)originalImSize_.width;
      calibration1_->setParameter(calibration1_->getParameter(0) * scaleColFactor, 0);
      calibration1_->setParameter(calibration1_->getParameter(2) * scaleColFactor, 2);

      if ((sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) && cameraModelType_ != Rectified) {
        calibration2_->setParameter(calibration2_->getParameter(0) * scaleColFactor, 0);
        calibration2_->setParameter(calibration2_->getParameter(2) * scaleColFactor, 2);

        if (cameraModelType_ == KannalaBrandt) {
          std::static_pointer_cast<KannalaBrandt8>(calibration1_)->scaleLappingArea(scaleColFactor);
          std::static_pointer_cast<KannalaBrandt8>(calibration2_)->scaleLappingArea(scaleColFactor);
        }
      }
    }
  }
  fps_ = readParameter<int>(fSettings, "Camera.fps", found);
}

void CameraSettings::readRGBD(cv::FileStorage& fSettings) {
  bool found;

  depthMapFactor_ = readParameter<float>(fSettings, "RGBD.DepthMapFactor", found);
  thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
  b_ = readParameter<float>(fSettings, "Stereo.b", found);
  bf_ = b_ * calibration1_->getParameter(0);
}


void CameraSettings::precomputeRectificationMaps() {
  // Precompute rectification maps, new calibrations, ...
  cv::Mat K1 = calibration1_->toK();
  K1.convertTo(K1, CV_64F);
  cv::Mat K2 = calibration2_->toK();
  K2.convertTo(K2, CV_64F);

  cv::Mat cvTlr;
  cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
  cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
  R12.convertTo(R12, CV_64F);
  cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
  t12.convertTo(t12, CV_64F);

  cv::Mat R_r1_u1, R_r2_u2;
  cv::Mat P1, P2, Q;

  cv::stereoRectify(K1, camera1DistortionCoef(), K2, camera2DistortionCoef(), newImSize_, R12, t12, R_r1_u1, R_r2_u2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, newImSize_);
  cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1, P1.rowRange(0, 3).colRange(0, 3), newImSize_, CV_32F, M1l_, M2l_);
  cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2, P2.rowRange(0, 3).colRange(0, 3), newImSize_, CV_32F, M1r_, M2r_);

  // Update calibration
  calibration1_->setParameter(P1.at<double>(0, 0), 0);
  calibration1_->setParameter(P1.at<double>(1, 1), 1);
  calibration1_->setParameter(P1.at<double>(0, 2), 2);
  calibration1_->setParameter(P1.at<double>(1, 2), 3);

  // Update bf
  bf_ = b_ * P1.at<double>(0, 0);

  // Compute T_r1_u1 and T_r2_u2
  Eigen::Matrix3f eigenR_r1_u1;
  cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
  T_r1_u1_ = Sophus::SE3f(eigenR_r1_u1, Eigen::Vector3f::Zero());

  if (sensor_ == MORB_SLAM::CameraType::IMU_STEREO || sensor_ == MORB_SLAM::CameraType::STEREO) {
    Eigen::Matrix3f eigenR_r2_u2;
    cv::cv2eigen(R_r2_u2, eigenR_r2_u2);
    T_r2_u2_ = Sophus::SE3f(eigenR_r2_u2, Eigen::Vector3f::Zero());
  }
}

std::ostream& operator<<(std::ostream& output, const CameraSettings& settings) {
  output << "SLAM settings: " << std::endl;

  output << "\t-Camera 1 parameters (";
  if (settings.cameraModelType_ == CameraSettings::PinHole || settings.cameraModelType_ == CameraSettings::Rectified) {
    output << "Pinhole";
  } else {
    output << "Kannala-Brandt";
  }
  output << ")" << ": [";
  for (size_t i = 0; i < settings.calibration1_->size(); i++) {
    output << " " << settings.calibration1_->getParameter(i);
  }
  output << " ]" << std::endl;

  if (!settings.vPinHoleDistorsion1_.empty()) {
    output << "\t-Camera 1 distortion parameters: [ ";
    for (float d : settings.vPinHoleDistorsion1_) {
      output << " " << d;
    }
    output << " ]" << std::endl;
  }

  if (settings.sensor_ == MORB_SLAM::CameraType::STEREO || settings.sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
    output << "\t-Camera 2 parameters (";
    if (settings.cameraModelType_ == CameraSettings::PinHole || settings.cameraModelType_ == CameraSettings::Rectified) {
      output << "Pinhole";
    } else {
      output << "Kannala-Brandt";
    }
    output << "" << ": [";
    for (size_t i = 0; i < settings.calibration2_->size(); i++) {
      output << " " << settings.calibration2_->getParameter(i);
    }
    output << " ]" << std::endl;

    if (!settings.vPinHoleDistorsion2_.empty()) {
      output << "\t-Camera 1 distortion parameters: [ ";
      for (float d : settings.vPinHoleDistorsion2_) {
        output << " " << d;
      }
      output << " ]" << std::endl;
    }
  }

  output << "\t-Original image size: [ " << settings.originalImSize_.width << " , " << settings.originalImSize_.height << " ]" << std::endl;
  output << "\t-Current image size: [ " << settings.newImSize_.width << " , " << settings.newImSize_.height << " ]" << std::endl;

  if (settings.bNeedToRectify_) {
    output << "\t-Camera 1 parameters after rectification: [ ";
    for (size_t i = 0; i < settings.calibration1_->size(); i++) {
      output << " " << settings.calibration1_->getParameter(i);
    }
    output << " ]" << std::endl;
  } else if (settings.bNeedToResize1_) {
    output << "\t-Camera 1 parameters after resize: [ ";
    for (size_t i = 0; i < settings.calibration1_->size(); i++) {
      output << " " << settings.calibration1_->getParameter(i);
    }
    output << " ]" << std::endl;

    if ((settings.sensor_ == MORB_SLAM::CameraType::STEREO || settings.sensor_ == MORB_SLAM::CameraType::IMU_STEREO) && settings.cameraModelType_ == CameraSettings::KannalaBrandt) {
      output << "\t-Camera 2 parameters after resize: [ ";
      for (size_t i = 0; i < settings.calibration2_->size(); i++) {
        output << " " << settings.calibration2_->getParameter(i);
      }
      output << " ]" << std::endl;
    }
  }
  output << "\t-Sequence FPS: " << settings.fps_ << std::endl;

  // Stereo stuff
  if (settings.sensor_ == MORB_SLAM::CameraType::STEREO || settings.sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
    output << "\t-Stereo baseline: " << settings.b_ << std::endl;
    output << "\t-Stereo depth threshold : " << settings.thDepth_ << std::endl;

    if (settings.cameraModelType_ == CameraSettings::KannalaBrandt) {
      auto vOverlapping1 = std::static_pointer_cast<KannalaBrandt8>(settings.calibration1_)->getLappingArea();
      auto vOverlapping2 = std::static_pointer_cast<KannalaBrandt8>(settings.calibration2_)->getLappingArea();
      output << "\t-Camera 1 overlapping area: [ " << vOverlapping1[0] << " , " << vOverlapping1[1] << " ]" << std::endl;
      output << "\t-Camera 2 overlapping area: [ " << vOverlapping2[0] << " , " << vOverlapping2[1] << " ]" << std::endl;
    }
  }

  if (settings.sensor_ == MORB_SLAM::CameraType::RGBD || settings.sensor_ == MORB_SLAM::CameraType::IMU_RGBD) {
    output << "\t-RGB-D depth map factor: " << settings.depthMapFactor_ << std::endl;
  }


  return output;
}

};  // namespace MORB_SLAM
