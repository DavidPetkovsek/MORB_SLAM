/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "MORB_SLAM/Tracking.h"

#include "MORB_SLAM/ImprovedTypes.hpp"
#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/G2oTypes.h"
#include "MORB_SLAM/GeometricTools.h"
#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/MLPnPsolver.h"
#include "MORB_SLAM/ORBmatcher.h"
#include "MORB_SLAM/Optimizer.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"



namespace MORB_SLAM {

Tracking::Tracking(std::shared_ptr<ORBVocabulary> pVoc, const Atlas_ptr &pAtlas,
                   std::shared_ptr<KeyFrameDatabase> pKFDB, const CameraType sensor, std::shared_ptr<Settings> settings)
    : mState(TrackingState::NO_IMAGES_YET),
      mLastProcessedState(TrackingState::NO_IMAGES_YET),
      mSensor(sensor),
      mbOnlyTracking(false),
      mbMapUpdated(false),
      notEnoughMatchPoints_trackOnlyMode(false),
      mpORBVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      mbReadyToInitialize(false),
      mpAtlas(pAtlas),
      mpLastKeyFrame(nullptr),
      mnLastRelocFrameId(0),
      time_recently_lost(1),
      mbCreatedMap(false),
      mpCamera2(nullptr),
      mForcedLost(false),
      mTeleported(false),
      mLockPreTeleportTranslation(false),
      mStereoInitDefaultPose(Sophus::SE3f()),
      mbReset(false),
      mbResetActiveMap(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false),
      mGlobalOriginPose(Sophus::SE3f()),
      mInitialFramePose(Sophus::SE3f()) {
  // Load camera parameters from settings file
  newParameterLoader(*settings);

  initID = 0;
  lastID = 0;

  std::vector<std::shared_ptr<const GeometricCamera>> vpCams = mpAtlas->GetAllCameras();
  std::cout << "There are " << vpCams.size() << " camera(s) in the atlas" << std::endl;
  for (std::shared_ptr<const GeometricCamera> pCam : vpCams) {
    std::cout << "Camera " << pCam->GetId();
    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      std::cout << " is pinhole" << std::endl;
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      std::cout << " is fisheye" << std::endl;
    } else {
      std::cout << " is unknown" << std::endl;
    }
  }

  mBaseTranslation.setZero();
  mPreTeleportTranslation.setZero();
}

Tracking::~Tracking() {}

void Tracking::newParameterLoader(Settings& settings) {
  mpCamera = settings.camera1();
  mpCamera = mpAtlas->AddCamera(mpCamera);

  if (settings.needToUndistort()) {
    mDistCoef = settings.camera1DistortionCoef();
  } else {
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  }

  mK = cv::Mat::eye(3, 3, CV_32F);
  mK.at<float>(0, 0) = mpCamera->getParameter(0);
  mK.at<float>(1, 1) = mpCamera->getParameter(1);
  mK.at<float>(0, 2) = mpCamera->getParameter(2);
  mK.at<float>(1, 2) = mpCamera->getParameter(3);

  if (mSensor.hasMulticam() && settings.cameraModelType() == Settings::KannalaBrandt) {
    mpCamera2 = settings.camera2();
    mpCamera2 = mpAtlas->AddCamera(mpCamera2);

    mTlr = settings.Tlr();
  }

  if (mSensor.hasMulticam()) {
    mbf = settings.bf();
    mThDepth = settings.b() * settings.thDepth();
  }

  if (mSensor == CameraType::RGBD || mSensor == CameraType::IMU_RGBD) {
    mDepthMapFactor = settings.depthMapFactor();
    if (fabs(mDepthMapFactor) < 1e-5)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }

  mFPS = settings.fps();
  // might be pointless

  // ORB parameters
  int nFeatures = settings.nFeatures();
  int nLevels = settings.nLevels();
  int fIniThFAST = settings.initThFAST();
  int fMinThFAST = settings.minThFAST();
  float fScaleFactor = settings.scaleFactor();

  mpORBextractorLeft = std::make_shared<ORBextractor>(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

  if (mSensor == CameraType::STEREO || mSensor == CameraType::IMU_STEREO)
    mpORBextractorRight = std::make_shared<ORBextractor>(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

  if (mSensor == CameraType::MONOCULAR || mSensor == CameraType::IMU_MONOCULAR)
    mpIniORBextractor = std::make_shared<ORBextractor>(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

  mFastInit = settings.fastIMUInit();
  mStationaryInit = settings.stationaryIMUInit();

  // IMU parameters
  Sophus::SE3f Tbc = settings.Tbc();
  float Ng = settings.noiseGyro();
  float Na = settings.noiseAcc();
  float Ngw = settings.gyroWalk();
  float Naw = settings.accWalk();

  const float sf = sqrt(settings.imuFrequency());
  mpImuCalib = std::make_shared<IMU::Calib>(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

  mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
}

void Tracking::SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper) { mpLocalMapper = pLocalMapper; }

void Tracking::SetLoopClosing(std::shared_ptr<LoopClosing> pLoopClosing) { mpLoopClosing = pLoopClosing; }

StereoPacket Tracking::GrabImageStereo(const cv::Mat& imRectLeft, const cv::Mat& imRectRight, const double& timestamp, const Camera_ptr &cam) {
  cv::Mat imGrayLeft = imRectLeft;
  cv::Mat imGrayRight = imRectRight;

  if (imGrayLeft.channels() == 3) {
    cvtColor(imGrayLeft, imGrayLeft, cv::COLOR_BGR2GRAY);
    cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
  } else if (imGrayLeft.channels() == 4) {
    cvtColor(imGrayLeft, imGrayLeft, cv::COLOR_BGRA2GRAY);
    cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
  }

  if (mSensor == CameraType::STEREO && !mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
  else if (mSensor == CameraType::STEREO && mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr);
  else if (mSensor == CameraType::IMU_STEREO && !mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
  else if (mSensor == CameraType::IMU_STEREO && mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

  Track();

  if(mState != TrackingState::OK && mState != TrackingState::NOT_INITIALIZED)
    std::cout << "Current state on Frame " << mCurrentFrame.mnId << ": " << mState << std::endl;
  
  //if state isnt lost, its still possible that it is lost if it trails to infinity - note if its in lost state no keyframes will be produced, but if its in OK state, keyframe will show
  //if mLastFrame.GetPose() from stereo is not close enough to IMU pose, then set to lost
  if (mState != TrackingState::LOST && mState != TrackingState::RECENTLY_LOST && !mReturnPose.translation().isZero(0) && !mForcedLost)
    return StereoPacket(mReturnPose, imGrayLeft, imGrayRight);

  return StereoPacket(imGrayLeft, imGrayRight); // we do not have a new pose to report
}

RGBDPacket Tracking::GrabImageRGBD(const cv::Mat& imRGB, const cv::Mat& imD, const double& timestamp, const Camera_ptr &cam) {
  cv::Mat mImGray = imRGB;
  cv::Mat imDepth = imD;

  if (mImGray.channels() == 3) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  if (mSensor == CameraType::RGBD)
    mCurrentFrame = Frame(cam, mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
  else if (mSensor == CameraType::IMU_RGBD)
    mCurrentFrame = Frame(cam, mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);

  Track();

  //if state isnt lost, its still possible that it is lost if it trails to infinity - note if its in lost state no keyframes will be produced, but if its in OK state, keyframe will show
  //if mLastFrame.GetPose() from stereo is not close enough to IMU pose, then set to lost
  if (mState != TrackingState::LOST && mState != TrackingState::RECENTLY_LOST)
    return RGBDPacket(mCurrentFrame.GetPose(), mImGray, imDepth);

  return RGBDPacket(mImGray, imDepth);
}

MonoPacket Tracking::GrabImageMonocular(const cv::Mat& im, const double& timestamp, const Camera_ptr &cam) {
  cv::Mat mImGray = im;

  if (mImGray.channels() == 3) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  if (mSensor == CameraType::MONOCULAR) {
    if (mState == TrackingState::NOT_INITIALIZED || mState == TrackingState::NO_IMAGES_YET || (lastID - initID) < mFPS)
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
    else
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
  } else if (mSensor == CameraType::IMU_MONOCULAR) {
    if (mState == TrackingState::NOT_INITIALIZED || mState == TrackingState::NO_IMAGES_YET)
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
    else
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
  }

  lastID = mCurrentFrame.mnId;
  Track();

  //if state isnt lost, its still possible that it is lost if it trails to infinity - note if its in lost state no keyframes will be produced, but if its in OK state, keyframe will show
  //if mLastFrame.GetPose() from stereo is not close enough to IMU pose, then set to lost
  if (mState != TrackingState::LOST && mState != TrackingState::RECENTLY_LOST)
    return MonoPacket(mCurrentFrame.GetPose(), mImGray);

  return MonoPacket(mImGray);
}

void Tracking::GrabImuData(const std::vector<IMU::Point>& imuMeasurements) {
  std::scoped_lock<std::mutex> lock(mMutexImuQueue);
  for(auto &point : imuMeasurements)
    mlQueueImuData.emplace_back(point); // copy ctor
}

void Tracking::PreintegrateIMU() {
  if (!mCurrentFrame.mpPrevFrame || mCurrentFrame.mpPrevFrame->isPartiallyConstructed) {
    mCurrentFrame.setIntegrated();
    return;
  }

  if (mlQueueImuData.size() == 0) {
    Verbose::PrintMess("No IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
    mCurrentFrame.setIntegrated();
    return;
  }

  std::vector<IMU::Point> frameIMUDataList;
  frameIMUDataList.reserve(mlQueueImuData.size());
  // Fill frameIMUDataList with data for this frame from the global imu data queue
  {
    std::scoped_lock<std::mutex> lock(mMutexImuQueue);
    auto itr = mlQueueImuData.begin();
    // iterate until the end of the queue or until we hit a timestamp that is newer than current frame
    for(; itr != mlQueueImuData.end() && itr->t < mCurrentFrame.mTimeStamp; ++itr)
      if(itr->t >= mCurrentFrame.mpPrevFrame->mTimeStamp) // ignore measurements before the previous frame
        frameIMUDataList.emplace_back(*itr); // add to local vector (via copy)
    // If there are points to remove (we found imu measurements to use)
    if(!frameIMUDataList.empty() && itr != mlQueueImuData.begin())
      mlQueueImuData.erase(mlQueueImuData.begin(), --itr); // erase them from the global queue since they are now copied locally
  }

  // Fails if there's 0 or 1 measurement
  const int n = static_cast<int>(frameIMUDataList.size()) - 1;
  if (n <= 0) {
    std::cout << "Empty IMU measurements vector!!!" << std::endl;
    mCurrentFrame.setIntegrated();
    return;
  }

  std::shared_ptr<IMU::Preintegrated> pImuPreintegratedFromLastFrame = std::make_shared<IMU::Preintegrated>(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

  // SUS math is being used here
  // points are not equally distributed and are being interpolated. There may not be any benefit to anything this if statement is doing in the below for loop
  // NOTE: this does actually help smooth out the IMU measurements, testing without this interpolation gave worse results.

  // we get here only if there are at least 2 measurements
  for (int i = 0; i < n; i++) { // iterates until the second to last element
    float tstep;
    float dt = frameIMUDataList[i + 1].t - frameIMUDataList[i].t;
    Eigen::Vector3f acc, angVel;
    if ((i == 0) && i < (n - 1)) { // first iteration but not the last iteration
      float timeFromLastFrameToFirstIMUFrame = frameIMUDataList[0].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
      acc = (frameIMUDataList[0].a + frameIMUDataList[1].a - (frameIMUDataList[1].a - frameIMUDataList[0].a) * (timeFromLastFrameToFirstIMUFrame / dt)) * 0.5f;
      angVel = (frameIMUDataList[0].w + frameIMUDataList[1].w - (frameIMUDataList[1].w - frameIMUDataList[0].w) * (timeFromLastFrameToFirstIMUFrame / dt)) * 0.5f;
      tstep = frameIMUDataList[1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
    } else if (i < (n - 1)) { // not the first nor the last iteration
      acc = (frameIMUDataList[i].a + frameIMUDataList[i + 1].a) * 0.5f;
      angVel = (frameIMUDataList[i].w + frameIMUDataList[i + 1].w) * 0.5f;
      tstep = dt;
    } else if ((i > 0) && (i == (n - 1))) { // not the first but is the last iteration
      float timeFromeLastIMUFrameToCurrentFrame = frameIMUDataList[i + 1].t - mCurrentFrame.mTimeStamp;
      acc = (frameIMUDataList[i].a + frameIMUDataList[i + 1].a - (frameIMUDataList[i + 1].a - frameIMUDataList[i].a) * (timeFromeLastIMUFrameToCurrentFrame / dt)) * 0.5f;
      angVel = (frameIMUDataList[i].w + frameIMUDataList[i + 1].w - (frameIMUDataList[i + 1].w - frameIMUDataList[i].w) * (timeFromeLastIMUFrameToCurrentFrame / dt)) * 0.5f;
      tstep = mCurrentFrame.mTimeStamp - frameIMUDataList[i].t;
    } else if ((i == 0) && (i == (n - 1))) { // both the first and the last iteration
      // there are two measurements but we ignore the second for fun TODO: dont?
      acc = frameIMUDataList[0].a;
      angVel = frameIMUDataList[0].w;
      tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
    }

    if (dt == 0 || tstep == 0) {
      std::cout << "WARNING: a PreintegrateIMU datapoint has dt=0, skipping iteration" << std::endl;
      continue;
    }

    //Prediction steps of the EKF
    mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
    pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
  }

  mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
  mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
  mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;
  mCurrentFrame.setIntegrated();
}

bool Tracking::PredictStateIMU() {
  //Is it even possible to get here with no previous frame? Maybe through LocalMappingDisabled shenanigans?
  if (!mCurrentFrame.mpPrevFrame || mCurrentFrame.mpPrevFrame->isPartiallyConstructed) {
    Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  //If the map was merged or loop was closed on the last Frame use mpLastKeyFrame, otherwise use mCurrentFrame
  if (mbMapUpdated && mpLastKeyFrame) {
    const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mpImuPreintegratedFromLastKF->dT;
    IMU::Bias b = mpLastKeyFrame->GetImuBias();

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(b));
    Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(b);
    Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(b);
    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    mCurrentFrame.mImuBias = b;
    return true;
  } else if (!mbMapUpdated && mCurrentFrame.mpImuPreintegratedFrame) {
    const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();

    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;
    IMU::Bias b = mLastFrame.mImuBias;

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(b));
    Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(b);
    Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(b);

    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    mCurrentFrame.mImuBias = b;
    return true;
  }

  // only happens gets here if there was no IMU data when PreintegrateIMU() was called this frame
  std::cout << "not IMU prediction!!" << std::endl;
  return false;
}

void Tracking::Track() {
  if (mpLocalMapper->mbBadImu) {
    mForcedLost = false;
    std::cout << "TRACK: Reset map because local mapper set the bad imu flag " << std::endl;
    RequestResetActiveMap();
    return;
  }

  std::shared_ptr<Map> pCurrentMap = mpAtlas->GetCurrentMap(false);
  if (!pCurrentMap) {
    std::cout << "ERROR: There is not an active map in the atlas" << std::endl;
    mForcedLost = false;
    return;
  }

  if (mState != TrackingState::NO_IMAGES_YET) {
    if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
      mForcedLost = false;
      std::cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << std::endl;
      std::scoped_lock<std::mutex> lock(mMutexImuQueue);
      mlQueueImuData.clear();
      CreateMapInAtlas();
      return;
    }
  }
  
  if (mSensor.isInertial() && mpLastKeyFrame) {
    mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());
  }

  if (mState == TrackingState::NO_IMAGES_YET) {
    mState = TrackingState::NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  if (mSensor.isInertial() && !mbCreatedMap) {
    PreintegrateIMU();
  }
  mbCreatedMap = false;

  // Get Map Mutex -> Map cannot be changed
  std::unique_lock<std::mutex> lock(pCurrentMap->mMutexMapUpdate);

  mbMapUpdated = false;

  int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
  if (nCurMapChangeIndex > pCurrentMap->GetLastMapChange()) {
    pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
    mbMapUpdated = true;
  }

  if (mState == TrackingState::NOT_INITIALIZED) {
    if (mSensor.hasMulticam()) {
      StereoInitialization();
    } else {
      MonocularInitialization();
    }

    // If initialization was successful, set mState to OK
    if (mState != TrackingState::OK) {
      mLastFrame = Frame(mCurrentFrame);
      mForcedLost = false;
      return;
    }
  } else {
    // System is initialized. Track Frame.
    bool bOK = false;
    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if (!mbOnlyTracking) {
      if (mState == TrackingState::OK) {
        // Local Mapping might have changed some MapPoints tracked in last frame, so update them here
        CheckReplacedInLastFrame();

        // If the state is not LOST and a bundle adjustment didn't occur on the previous frame
        if((mbHasPrevDeltaFramePose || pCurrentMap->isImuInitialized()) && mCurrentFrame.mnId > mnLastRelocFrameId + 1){
          bOK = TrackWithMotionModel();
        }
        // If the state was lost/reset on the previous Frame, or if TrackWithMotionModel() failed
        if (!bOK) {
          bOK = TrackReferenceKeyFrame();
        }

        // If both Track helper functions failed, we are lost
        if (!bOK) {
          std::cout << "TrackReferenceKeyFrame failed, is LOST" << std::endl;
          
          // if there's enough KeyFrames in the map we're recently lost, if not we're lost
          if (pCurrentMap->KeyFramesInMap() > 10 && (mCurrentFrame.mnId > (mnLastRelocFrameId + mFPS) || !mSensor.isInertial())) {
            mState = TrackingState::RECENTLY_LOST;
            mTimeStampLost = mCurrentFrame.mTimeStamp;
          } else {
            mState = TrackingState::LOST;
          }

        }
      } else if (mState == TrackingState::RECENTLY_LOST) {
        bOK = true;
        if (mSensor.isInertial()) {
          bOK = (pCurrentMap->isImuInitialized()) ? PredictStateIMU() : false;
          if (pCurrentMap->isImuInitialized())
            bOK = PredictStateIMU();
          else
            bOK = false;
          if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost || mForcedLost) {
            if(mForcedLost) {
              std::cout << "BONK! TrackingState forcefully set to LOST" << std::endl;
              mForcedLost = false;
            } else {
              Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
            }
            mState = TrackingState::LOST;
            bOK = false;
          }
        } else {
          // Relocalization
          bOK = Relocalization();
          if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost && !bOK) {
            mState = TrackingState::LOST;
            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
            bOK = false;
          }
        }
      // I don't think a Track() loop can actually start with the state set to LOST, unless thru non-stereoinertial runs? This probably never runs
      } else if (mState == TrackingState::LOST) {
        if(mForcedLost) {
          std::cout << "BONK! TrackingState forcefully set to LOST" << std::endl;
          mForcedLost = false;
        }
        Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);
        // Dont store the current Map in the Atlas if there's less than 10 KFs
        if (pCurrentMap->KeyFramesInMap() < 10) {
          RequestResetActiveMap();
          Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
        } else {
          setStereoInitDefaultPose(mpLastKeyFrame->GetPose());
          CreateMapInAtlas();
        }

        if (mpLastKeyFrame) mpLastKeyFrame = nullptr;
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
        return;
      }

    } else {
      // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
      if (mState == TrackingState::LOST) {
        if (mSensor.isInertial())
          Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
        bOK = Relocalization();
      } else {
        if (!notEnoughMatchPoints_trackOnlyMode) {
          // In last frame we tracked enough MapPoints in the map
          if (mbHasPrevDeltaFramePose) {
            bOK = TrackWithMotionModel();
          } else {
            bOK = TrackReferenceKeyFrame();
          }
        } else {
          // In last frame we tracked mainly "visual odometry" points.

          // We compute two camera poses, one from motion model and one doing
          // relocalization. If relocalization is sucessfull we choose that
          // solution, otherwise we retain the "visual odometry" solution.

          bool bOKMM = false;
          bool bOKReloc = false;
          std::vector<std::shared_ptr<MapPoint>> vpMPsMM;
          std::vector<bool> vbOutMM;
          Sophus::SE3f TcwMM;
          if (mbHasPrevDeltaFramePose) {
            bOKMM = TrackWithMotionModel();
            vpMPsMM = mCurrentFrame.mvpMapPoints;
            vbOutMM = mCurrentFrame.mvbOutlier;
            TcwMM = mCurrentFrame.GetPose();
          }
          bOKReloc = Relocalization();

          if (bOKMM && !bOKReloc) {
            mCurrentFrame.SetPose(TcwMM);
            mCurrentFrame.mvpMapPoints = vpMPsMM;
            mCurrentFrame.mvbOutlier = vbOutMM;

            if (notEnoughMatchPoints_trackOnlyMode) {
              for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                  mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                }
              }
            }
          } else if (bOKReloc) {
            notEnoughMatchPoints_trackOnlyMode = false;
          }

          bOK = bOKReloc || bOKMM;
        }
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (!mbOnlyTracking) {
      if (bOK) {
        bOK = TrackLocalMap();
      } else {
        std::cout << "Fail to track local map!" << std::endl;
      }
    } else {
      // notEnoughMatchPoints_trackOnlyMode true means that there are few matches to MapPoints in the map.
      // We cannot retrieve a local map and therefore we do not perform TrackLocalMap().
      // Once the system relocalizes the camera we will use the local map again.
      if (bOK && !notEnoughMatchPoints_trackOnlyMode) bOK = TrackLocalMap();
    }

    if (bOK) {
      mState = TrackingState::OK;
    // Occurs if this the Frame we're becoming lost
    } else if (mState == TrackingState::OK) {
      if (mSensor.isInertial() && (!pCurrentMap->isImuInitialized() || !pCurrentMap->GetInertialBA2())) {
          std::cout << "IMU is not or recently initialized. Reseting active map..." << std::endl;
          mForcedLost = false;
          RequestResetActiveMap();
      }
      mState = TrackingState::RECENTLY_LOST;
      mTimeStampLost = mCurrentFrame.mTimeStamp;
    }

    if (bOK || mState == TrackingState::RECENTLY_LOST) {
      // Update motion model
      if (mLastFrame.HasPose() && mCurrentFrame.HasPose()) {
        Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
        mPrevDeltaFramePose = mCurrentFrame.GetPose() * LastTwc;
        mbHasPrevDeltaFramePose = true;
      } else {
        mbHasPrevDeltaFramePose = false;
      }

      // Clean VO matches
      for (int i = 0; i < mCurrentFrame.N; i++) {
        std::shared_ptr<MapPoint> pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations() < 1) {
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i] = nullptr;
          }
      }

      // Check if we need to insert a new keyframe
      if (mSensor.isInertial() && NeedNewKeyFrame()) {
        CreateNewKeyFrame();
      }

      //TK16
      // We allow points with high innovation (considererd outliers by the Huber Function) pass to the new keyframe, so that bundle adjustment will finally decide if they are outliers or not.
      // We don't want next frame to estimate its position with those points so we discard them in the frame. Only has effect if lastframe is tracked
      for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i] = nullptr;
      }
    }

    // Reset if the camera get lost soon after initialization
    if(mState == TrackingState::LOST) {
      mForcedLost = false;
      if (pCurrentMap->KeyFramesInMap() <= 10) {
        RequestResetActiveMap();
        return;
      }

      if (mSensor.isInertial()) {
        if (!pCurrentMap->isImuInitialized()) {
          Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
          RequestResetActiveMap();
          return;
        }
      }

      setStereoInitDefaultPose(mpLastKeyFrame->GetPose());
      CreateMapInAtlas();

      return;
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    if(!mTeleported && !mLockPreTeleportTranslation) {
      mPreTeleportTranslation = mpReferenceKF->GetRotation().transpose()*mpReferenceKF->GetTranslation();
    } else if(mTeleported) {
      mTeleported = false;
      mLockPreTeleportTranslation = false;
      mBaseTranslation -= (mpReferenceKF->GetRotation().transpose()*mpReferenceKF->GetTranslation()) - mPreTeleportTranslation;
      mPreTeleportTranslation = mpReferenceKF->GetRotation().transpose()*mpReferenceKF->GetTranslation();
      if(pCurrentMap->GetInertialBA2())
        mpLocalMapper->setIsDoneVIBA(true);
    }

    if(mpLocalMapper->getIsDoneVIBA()) {
      Eigen::Vector3f translation_print = mCurrentFrame.GetPose().rotationMatrix().inverse()*mCurrentFrame.GetPose().translation();
      mReturnPose = Sophus::SE3f(mCurrentFrame.GetPose().rotationMatrix(), mCurrentFrame.GetPose().rotationMatrix()*(translation_print+mBaseTranslation));
    } else {
      Eigen::Vector3f zero;
      zero.setZero();
      mReturnPose = Sophus::SE3f(Eigen::Matrix3f::Identity(), zero);
    }

    mLastFrame = Frame(mCurrentFrame);
  }

  if ((mState == TrackingState::OK || mState == TrackingState::RECENTLY_LOST) && mCurrentFrame.HasPose())
    mRelativeFramePose = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();

}

void Tracking::StereoInitialization() {
  // If there aren't enough keypoints, can't initialize and return
  if (mCurrentFrame.N <= 500) {
    std::cout << "There aren't enough KeyPoints in the Frame to initialize the Map" << std::endl;
    return;
  }

  if (mSensor.isInertial()) {
    if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
      return;
    }

    if (!stationaryIMUInitEnabled() && (mpAtlas->CountMaps() <= 1) && (mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA).norm() < 0.5) {
      std::cout << "More acceleration is required to initialize the Map" << std::endl;
      return;
    }

    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
  }

  //TODO Add setting for relocalization attempt
  if(mpAtlas->CountMaps() > 1 && true && Relocalization()) {
      std::shared_ptr<Map> pCurrentMap = mpAtlas->GetCurrentMap();
      
      {
        std::unique_lock<std::mutex> mergeLock(mpRelocalizationTargetMap->mMutexMapUpdate);

        mpAtlas->ChangeMap(mpRelocalizationTargetMap);
        mpAtlas->SetMapBad(pCurrentMap);
        mpAtlas->RemoveBadMaps();

        if(!mHasGlobalOriginPose) {
          Eigen::Matrix3f outputRotation = mCurrentFrame.GetPose().rotationMatrix().inverse() * mGlobalOriginPose.rotationMatrix();
          Eigen::Vector3f outputTranslation = mGlobalOriginPose.rotationMatrix().inverse()*mGlobalOriginPose.translation() - mCurrentFrame.GetPose().rotationMatrix().inverse()*mCurrentFrame.GetPose().translation();
          mInitialFramePose = Sophus::SE3f(outputRotation, outputTranslation);
          mHasGlobalOriginPose = true;
        }
      }
  } else {
    // Set Frame pose to the default pose
    mCurrentFrame.SetPose(getStereoInitDefaultPose());
  }

  if (mSensor.isInertial()) {
    Eigen::Vector3f Vwb0;
    Vwb0.setZero();
    mCurrentFrame.SetVelocity(Vwb0);
  }

  // Create KeyFrame
  std::shared_ptr<KeyFrame> pKFini = std::make_shared<KeyFrame>(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  // Insert KeyFrame in the map
  // TODO: Why is this an Atlas function?
  mpAtlas->AddKeyFrame(pKFini);

  // Create MapPoints and asscoiate to KeyFrame
  if (!mpCamera2) {
    for (int i = 0; i < mCurrentFrame.N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        Eigen::Vector3f x3D;
        mCurrentFrame.UnprojectStereo(i, x3D);
        std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint>(x3D, pKFini, mpAtlas->GetCurrentMap());
        pNewMP->AddObservation(pKFini, i);
        pKFini->AddMapPoint(pNewMP, i);
        pNewMP->ComputeDistinctiveDescriptors();
        pNewMP->UpdateNormalAndDepth();
        mpAtlas->AddMapPoint(pNewMP);

        mCurrentFrame.mvpMapPoints[i] = pNewMP;
      }
    }
  } else {
    for (int i = 0; i < mCurrentFrame.Nleft; i++) {
      int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
      if (rightIndex != -1) {
        Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

        std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint>(x3D, pKFini, mpAtlas->GetCurrentMap());

        pNewMP->AddObservation(pKFini, i);
        pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

        pKFini->AddMapPoint(pNewMP, i);
        pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

        pNewMP->ComputeDistinctiveDescriptors();
        pNewMP->UpdateNormalAndDepth();
        mpAtlas->AddMapPoint(pNewMP);

        mCurrentFrame.mvpMapPoints[i] = pNewMP;
        mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
      }
    }
  }
  Verbose::PrintMess("New Map created with " + std::to_string(mpAtlas->MapPointsInMap()) + " MapPoints", Verbose::VERBOSITY_QUIET);

  mpLocalMapper->InsertKeyFrame(pKFini);

  mLastFrame = Frame(mCurrentFrame);
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFini;

  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
  mpReferenceKF = pKFini;
  mCurrentFrame.mpReferenceKF = pKFini;

  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

  mState = TrackingState::OK;
}

void Tracking::MonocularInitialization() {
  if (!mbReadyToInitialize) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() > 100) {
      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      std::fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

      if (mSensor == CameraType::IMU_MONOCULAR) {
        mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
      }

      mbReadyToInitialize = true;
      return;
    }
  } else {
    if (((int)mCurrentFrame.mvKeys.size() <= 100) || ((mSensor == CameraType::IMU_MONOCULAR) && (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0))) {
      mbReadyToInitialize = false;
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches < 100) {
      mbReadyToInitialize = false;
      return;
    }

    Sophus::SE3f Tcw;
    std::vector<bool> vbTriangulated;  // Triangulated Correspondences (mvIniMatches)

    if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn, mCurrentFrame.mvKeysUn, mvIniMatches, Tcw, mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i] = -1;
          nmatches--;
        }
      } 

      // Set Frame Poses
      mInitialFrame.SetPose(Sophus::SE3f());
      mCurrentFrame.SetPose(Tcw);

      CreateInitialMapMonocular();
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  std::shared_ptr<KeyFrame> pKFini = std::make_shared<KeyFrame>(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
  std::shared_ptr<KeyFrame> pKFcur = std::make_shared<KeyFrame>(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mSensor == CameraType::IMU_MONOCULAR)
    pKFini->mpImuPreintegrated = (std::shared_ptr<IMU::Preintegrated>)(nullptr);

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  // Insert KFs in the map
  mpAtlas->AddKeyFrame(pKFini);
  mpAtlas->AddKeyFrame(pKFcur);

  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0) continue;

    // Create MapPoint.
    Eigen::Vector3f worldPos;
    worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
    std::shared_ptr<MapPoint> pMP = std::make_shared<MapPoint>(worldPos, pKFcur, mpAtlas->GetCurrentMap());

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    // Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    // Add to Map
    mpAtlas->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  std::set<std::shared_ptr<MapPoint>> sMPs;
  sMPs = pKFini->GetMapPoints();

  // Bundle Adjustment
  Verbose::PrintMess("New Map created with " + std::to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
  Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth;
  if (mSensor == CameraType::IMU_MONOCULAR)
    invMedianDepth = 4.0f / medianDepth;
  else
    invMedianDepth = 1.0f / medianDepth;

  // TODO Check, originally 100 tracks
  if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50) {
    Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
    RequestResetActiveMap();
    return;
  }

  // Scale initial baseline
  Sophus::SE3f Tc2w = pKFcur->GetPose();
  Tc2w.translation() *= invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  std::vector<std::shared_ptr<MapPoint>> vpAllMapPoints = pKFini->GetMapPointMatches();
  for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
      std::shared_ptr<MapPoint> pMP = vpAllMapPoints[iMP];
      pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      pMP->UpdateNormalAndDepth();
    }
  }

  if (mSensor == CameraType::IMU_MONOCULAR) {
    pKFcur->mPrevKF = pKFini;
    pKFini->mNextKF = pKFcur;
    pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);
  mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  // Compute here initial velocity
  std::vector<std::shared_ptr<KeyFrame>> vKFs = mpAtlas->GetAllKeyFrames();

  Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
  mbHasPrevDeltaFramePose = false;
  Eigen::Vector3f phi = deltaT.so3().log();

  double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) / (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
  phi *= aux;

  mLastFrame = Frame(mCurrentFrame);

  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
  mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

  mState = TrackingState::OK;
  initID = pKFcur->mnId;
}

void Tracking::CreateMapInAtlas() {

  mpAtlas->CreateNewMap();  

  mState = TrackingState::NO_IMAGES_YET;

  // Reset the variables with information about the last KF
  mbHasPrevDeltaFramePose = false;
  notEnoughMatchPoints_trackOnlyMode = false;

  if (mSensor.isInertial() && mpImuPreintegratedFromLastKF) {
    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
  }

  if (mpLastKeyFrame) mpLastKeyFrame = nullptr;
  if (mpReferenceKF) mpReferenceKF = nullptr;

  // Reset the current and last Frames
  mLastFrame = Frame();
  mCurrentFrame = Frame();

  // prevents PreintegrateIMU() from being called in the next frame
  mbCreatedMap = true;

  Verbose::PrintMess("First frame id in map: " + std::to_string(mCurrentFrame.mnId + 1), Verbose::VERBOSITY_NORMAL);

  // only used by monocular tracking
  mbReadyToInitialize = false;
  
  // only used by the viewer or monocular tracking
  mvIniMatches.clear();
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i = 0; i < mLastFrame.N; i++) {
    std::shared_ptr<MapPoint> pMP = mLastFrame.mvpMapPoints[i];

    if (pMP) {
      std::shared_ptr<MapPoint> pRep = pMP->GetReplaced();
      if (pRep) {
        mLastFrame.mvpMapPoints[i] = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  // Compute Bag of Words vector
  mCurrentFrame.ComputeBoW();

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.7, true);
  std::vector<std::shared_ptr<MapPoint>> vpMapPointMatches;

  int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

  if (nmatches < 15) {
    std::cout << "TRACK_REF_KF: Less than 15 matches!!" << std::endl;
    return false;
  }

  mCurrentFrame.mvpMapPoints = vpMapPointMatches;
  mCurrentFrame.SetPose(mLastFrame.GetPose());

  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        std::shared_ptr<MapPoint> pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = nullptr;
        mCurrentFrame.mvbOutlier[i] = false;
        if (i < mCurrentFrame.Nleft) {
          pMP->mbTrackInView = false;
        } else {
          pMP->mbTrackInViewR = false;
        }
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  return (mSensor.isInertial() || (nmatchesMap >= 10));
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  // For StereoInertial Tracking, this line does nothing but add rounding error (I think)
  mLastFrame.SetPose(mRelativeFramePose * mLastFrame.mpReferenceKF->GetPose());

  if (mnLastKeyFrameId == mLastFrame.mnId || !mSensor.hasMulticam() || !mbOnlyTracking)
    return;

  // Create "visual odometry" MapPoints
  // We sort points according to their measured depth by the stereo/RGB-D sensor
  std::vector<std::pair<float, int>> vDepthIdx;
  const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
  vDepthIdx.reserve(Nfeat);
  for (int i = 0; i < Nfeat; i++) {
    float z = mLastFrame.mvDepth[i];
    if (z > 0) {
      vDepthIdx.emplace_back(z, i);
    }
  }

  if (vDepthIdx.empty()) return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // We insert all close points (depth<mThDepth)
  // If more than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;

    // if the observed point isn't in the MapPoints already, add it
    if (!mLastFrame.mvpMapPoints[i] || mLastFrame.mvpMapPoints[i]->Observations() < 1) {
      Eigen::Vector3f x3D;

      if (mLastFrame.Nleft == -1) {
        mLastFrame.UnprojectStereo(i, x3D);
      } else {
        // rotates/translates the MapPoint coords to be from the reference from of the origin
        x3D = mLastFrame.UnprojectStereoFishEye(i);
      }

      std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint>(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
      mLastFrame.mvpMapPoints[i] = pNewMP;
    }
    nPoints++;

    if (vDepthIdx[j].first > mThDepth && nPoints > 100) break;
  }
}

bool Tracking::TrackWithMotionModel() {
  ORBmatcher matcher(0.9, true);

  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points if in Localization Mode
  UpdateLastFrame();

  if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId > mnLastRelocFrameId + mFPS)) {
    // Predict state with IMU if it is initialized and it doesnt need reset
    return PredictStateIMU();
  }

  //No IMU, so assume the pose changed by the same amount it changed by last Frame
  mCurrentFrame.SetPose(mPrevDeltaFramePose * mLastFrame.GetPose());

  std::fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);

  // Project points seen in previous frame
  int th = (mSensor == CameraType::STEREO) ? 7 : 15;
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, !mSensor.hasMulticam());

  // If few matches, uses a higher-tolerance search
  if (nmatches < 20) {
    Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
    std::fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);

    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, !mSensor.hasMulticam());
    Verbose::PrintMess("Matches with wider search: " + std::to_string(nmatches), Verbose::VERBOSITY_NORMAL);
  }

  if (nmatches < 20) {
    Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
    return mSensor.isInertial();
  }

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        std::shared_ptr<MapPoint> pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = nullptr;
        mCurrentFrame.mvbOutlier[i] = false;
        if (i < mCurrentFrame.Nleft) {
          pMP->mbTrackInView = false;
        } else {
          pMP->mbTrackInViewR = false;
        }
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0) {
        nmatchesMap++;
      }
    }
  }

  // TODO: Should these be the same number?
  if (mbOnlyTracking) {
    notEnoughMatchPoints_trackOnlyMode = nmatchesMap < 10;
    return nmatches > 20;
  }
  return (mSensor.isInertial() || (nmatchesMap >= 10));
}

bool Tracking::TrackLocalMap() {
  // We have an estimation of the camera pose and some map points tracked in the frame.
  //We retrieve the local map and try to find matches to points in the local map.

  UpdateLocalMap();
  SearchLocalPoints();

  if (!mpAtlas->isImuInitialized() || mCurrentFrame.mpImuPreintegratedFrame == nullptr || mCurrentFrame.mnId <= mnLastRelocFrameId + mFPS) {
    Optimizer::PoseOptimization(&mCurrentFrame);
  } else if(mbMapUpdated || mCurrentFrame.mpPrevFrame->mpcpi == nullptr) {
    Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame);
  } else {
    Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame);
  }

  mnMatchesInliers = 0;

  // Update MapPoints Statistics
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        if (mbOnlyTracking || mCurrentFrame.mvpMapPoints[i]->Observations() > 0) 
          mnMatchesInliers++;
      // SUS why only stereo? No stereo_inertial
      } else if (mSensor == CameraType::STEREO) {
        mCurrentFrame.mvpMapPoints[i] = nullptr;
      }
    }
  }

  if(mForcedLost) {
    std::cout << "BONK queued for the next frame" << std::endl;
    return false;
  }

  // Decide if the tracking was succesful. More restrictive if there was a relocalization recently

  if (mCurrentFrame.mnId < mnLastRelocFrameId + mFPS) return mnMatchesInliers >= 50;

  if (mState == TrackingState::RECENTLY_LOST) return mnMatchesInliers >= 150;

  if(mSensor.isInertial()){
    if (!mSensor.hasMulticam()) {
      return !((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) || (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized()));
    } else {
      return mnMatchesInliers >= 15;
    } 
  } else {
    return mnMatchesInliers >= 30;
  }
}

bool Tracking::NeedNewKeyFrame() {
  if (mSensor.isInertial() && !mpAtlas->GetCurrentMap()->isImuInitialized()) 
    return (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25;

  if (mbOnlyTracking) return false;

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

  const int nKFs = mpAtlas->KeyFramesInMap();

  // Do not insert keyframes if not enough frames have passed from last relocalization
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mFPS && nKFs > mFPS)
    return false;

  // Tracked MapPoints in the reference keyframe
  int nMinObs = (nKFs <= 2) ? 2 : 3;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  // Local Mapping accept keyframes?
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Check how many "close" points are being tracked and how many could be potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;

  if (mSensor.hasMulticam()) {
    int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
    for (int i = 0; i < N; i++) {
      if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
          nTrackedClose++;
        else
          nNonTrackedClose++;
      }
    }
  }

  bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

  // Thresholds
  float thRefRatio = 0.75f;
  if (nKFs < 2) thRefRatio = 0.4f;
  if (mSensor == CameraType::MONOCULAR) thRefRatio = 0.9f;
  if (mpCamera2) thRefRatio = 0.75f;
  if (mSensor == CameraType::IMU_MONOCULAR) {
    if (mnMatchesInliers > 350)  // Points tracked from the local map
      thRefRatio = 0.75f;
    else
      thRefRatio = 0.90f;
  }

  // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mFPS;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle (MinFrames was always hardcoded to 0, removed)
  const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId) && bLocalMappingIdle);
  // Condition 1c: tracking is weak
  const bool c1c = mSensor.hasMulticam() && !mSensor.isInertial() && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
  // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
  const bool c2 = (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) && mnMatchesInliers > 15);

  // Temporal condition for Inertial cases
  bool c3 = mpLastKeyFrame && mSensor.isInertial() && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5;

  bool c4 = (((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) || mState == TrackingState::RECENTLY_LOST) && (mSensor == CameraType::IMU_MONOCULAR);

  if (((c1a || c1b || c1c) && c2) || c3 || c4) {
    // If the mapping accepts keyframes, insert keyframe. Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle || mpLocalMapper->IsInitializing()) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      if (mSensor.hasMulticam() && (mpLocalMapper->KeyframesInQueue() < 3)) {
        return true;
      }
    }
  }
  
  return false;
}

void Tracking::CreateNewKeyFrame() {
  if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized()) return;

  if (!mpLocalMapper->SetNotStop(true)) return;

  mpReferenceKF = std::make_shared<KeyFrame>(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mpAtlas->isImuInitialized())
    mpReferenceKF->bImu = true;

  mpReferenceKF->SetNewBias(mCurrentFrame.mImuBias);
  mCurrentFrame.mpReferenceKF = mpReferenceKF;

  if (mpLastKeyFrame) {
    mpReferenceKF->mPrevKF = mpLastKeyFrame;
    mpLastKeyFrame->mNextKF = mpReferenceKF;
  } else
    Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

  // Reset preintegration from last KF (Create new object)
  if (mSensor.isInertial())
    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(mpReferenceKF->GetImuBias(), mpReferenceKF->mImuCalib);

  if (mSensor.hasMulticam()){  // TODO check if incluide imu_stereo
    mCurrentFrame.UpdatePoseMatrices();
    // We sort points by the measured depth by the stereo/RGBD sensor. We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    int maxPoint = 100;

    std::vector<std::pair<float, int>> vDepthIdx;
    int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
    vDepthIdx.reserve(mCurrentFrame.N);
    for (int i = 0; i < N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        vDepthIdx.emplace_back(z, i);
      }
    }

    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());

      int nPoints = 0;
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        std::shared_ptr<MapPoint> pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP) {
          bCreateNew = true;
        } else if (pMP->Observations() < 1) {
          bCreateNew = true;
          mCurrentFrame.mvpMapPoints[i] = nullptr;
        }

        if (bCreateNew) {
          Eigen::Vector3f x3D;

          if (mCurrentFrame.Nleft == -1) {
            mCurrentFrame.UnprojectStereo(i, x3D);
          } else {
            x3D = mCurrentFrame.UnprojectStereoFishEye(i);
          }

          std::shared_ptr<MapPoint> pNewMP = std::make_shared<MapPoint>(x3D, mpReferenceKF, mpAtlas->GetCurrentMap());
          pNewMP->AddObservation(mpReferenceKF, i);

          // Check if it is a stereo observation in order to not duplicate MapPoints
          if (mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
            mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
            pNewMP->AddObservation(mpReferenceKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
            mpReferenceKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
          }

          mpReferenceKF->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
        }
        nPoints++;

        if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
          break;
        }
      }
    }
  }

  mpLocalMapper->InsertKeyFrame(mpReferenceKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = mpReferenceKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  for (std::shared_ptr<MapPoint>& pMP : mCurrentFrame.mvpMapPoints) {
    if (pMP) {
      if (pMP->isBad()) {
        pMP = nullptr;
      } else {
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
      }
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (std::shared_ptr<MapPoint> pMP : mvpLocalMapPoints) {
    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId || pMP->isBad()) continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }

  if (nToMatch > 0) {
    ORBmatcher matcher(0.8);
    int th = 1; // this ultimately is a multiplier on the size of a square search area in a frame
    if (mSensor == CameraType::RGBD || mSensor == CameraType::IMU_RGBD)
      th = 3;
    if (mpAtlas->isImuInitialized()) {
      if (mpAtlas->GetCurrentMap()->GetInertialBA2())
        th = 2;
      else
        th = 6;
    } else if (!mpAtlas->isImuInitialized() && mSensor.isInertial()) {
      th = 10;
    }

    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
      th = 5;

    if (mState == TrackingState::LOST || mState == TrackingState::RECENTLY_LOST)  // Lost for less than 1 second
      th = 15;

    matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();

  for (std::shared_ptr<KeyFrame> pKF : mvpLocalKeyFrames) {
    for (std::shared_ptr<MapPoint> pMP : pKF->GetMapPointMatches()) {
      if (!pMP || pMP->isBad() || pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId) continue;
      mvpLocalMapPoints.push_back(pMP);
      pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  std::map<std::shared_ptr<KeyFrame>, int> keyframeCounter;
  if (!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId < mnLastRelocFrameId + 2)) {
    for (int i = 0; i < mCurrentFrame.N; i++) {
      std::shared_ptr<MapPoint> pMP = mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!pMP->isBad()) {
          const std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = pMP->GetObservations();
          for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++) {
            if(std::shared_ptr<KeyFrame> pKF = (it->first).lock()) {
              keyframeCounter[pKF]++;
            }
          }
        } else {
          mCurrentFrame.mvpMapPoints[i] = nullptr;
        }
      }
    }
  } else {
    for (int i = 0; i < mLastFrame.N; i++) {
      // Using lastframe since current frame has not matches yet
      if (mLastFrame.mvpMapPoints[i]) {
        std::shared_ptr<MapPoint> pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP) continue;
        if (!pMP->isBad()) {
          const std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = pMP->GetObservations();
          for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++) {
            if(std::shared_ptr<KeyFrame> pKF = (it->first).lock()) {
              keyframeCounter[pKF]++;
            }
          }
        } else {
          // MODIFICATION
          mLastFrame.mvpMapPoints[i] = nullptr;
        }
      }
    }
  }

  int max = 0;
  std::shared_ptr<KeyFrame> pKFmax = nullptr;

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
  for (auto &pair : keyframeCounter) {
    std::shared_ptr<KeyFrame> pKF = pair.first;

    if (pKF->isBad()) continue;

    if (pair.second > max) {
      max = pair.second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(pKF);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }

  // Include also some not-already-included keyframes that are neighbors to already-included keyframes
  for (std::shared_ptr<KeyFrame> pKF : mvpLocalKeyFrames) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80) break;

    for (std::shared_ptr<KeyFrame> pNeighKF : pKF->GetBestCovisibilityKeyFrames(10)) {
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    for (std::shared_ptr<KeyFrame> pChildKF : pKF->GetChilds()) {
      if (pChildKF && !pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    std::shared_ptr<KeyFrame> pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }
  }

  // Add 20 last temporal KFs (mainly for IMU)
  if (mSensor.isInertial() && mvpLocalKeyFrames.size() < 80 && !mCurrentFrame.isPartiallyConstructed) {
    std::shared_ptr<KeyFrame> tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

    const int Nd = 20;
    for (int i = 0; i < Nd; i++) {
      if (!tempKeyFrame || tempKeyFrame->isPartiallyConstructed) break;
      if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(tempKeyFrame);
        tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        tempKeyFrame = tempKeyFrame->mPrevKF;
      }
    }
  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
  // Compute Bag of Words Vector
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for relocalization
  std::vector<std::shared_ptr<KeyFrame>> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

  if (vpCandidateKFs.empty()) {
    Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  const int nKFs = vpCandidateKFs.size();

  // We perform first an ORB matching with each candidate
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.75, true); 

  std::vector<MLPnPsolver*> vpMLPnPsolvers;
  vpMLPnPsolvers.resize(nKFs);

  std::vector<std::vector<std::shared_ptr<MapPoint>>> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  std::vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    std::shared_ptr<KeyFrame> pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
      if (nmatches < 15) {
        vbDiscarded[i] = true;
        continue;
      } else {
        MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991);  // This solver needs at least 6 points
        vpMLPnPsolvers[i] = pSolver;
        nCandidates++;
      }
    }
  }

  std::shared_ptr<Map> relocMap;
  // Alternatively perform some iterations of P4P RANSAC until we found a camera pose supported by enough inliers
  bool bMatch = false;
  ORBmatcher matcher2(0.9, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i]) continue;

      // Perform 5 Ransac Iterations
      std::vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      MLPnPsolver* pSolver = vpMLPnPsolvers[i];
      Eigen::Matrix4f eigTcw;
      bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      if (!bTcw) std::cout << "camera pose not calculated" << std::endl;

      // If a Camera Pose is computed, optimize
      if (bTcw) {
        Sophus::SE3f Tcw(eigTcw);
        mCurrentFrame.SetPose(Tcw);

        std::set<std::shared_ptr<MapPoint>> sFound;

        for (size_t j = 0; j < vbInliers.size(); j++) {
          if (vbInliers[j]) {
            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
            sFound.insert(vvpMapPointMatches[i][j]);
          } else
            mCurrentFrame.mvpMapPoints[j] = nullptr;
        }

        int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        if (nGood < 10) continue;

        for (int io = 0; io < mCurrentFrame.N; io++)
          if (mCurrentFrame.mvbOutlier[io])
            mCurrentFrame.mvpMapPoints[io] = nullptr;

        // If few inliers, search by projection in a coarse window and optimize again
        if (nGood < 50) {
          int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

          if (nadditional + nGood >= 50) {
            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

            // If many inliers but still not enough, search by projection again in a narrower window the camera has been already optimized with many points
            if (nGood > 30 && nGood < 50) {
              sFound.clear();
              for (int ip = 0; ip < mCurrentFrame.N; ip++)
                if (mCurrentFrame.mvpMapPoints[ip])
                  sFound.insert(mCurrentFrame.mvpMapPoints[ip]);

              nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

              // Final optimization
              if (nGood + nadditional >= 50) {
                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                for (int io = 0; io < mCurrentFrame.N; io++)
                  if (mCurrentFrame.mvbOutlier[io])
                    mCurrentFrame.mvpMapPoints[io] = nullptr;
              }
            }
          }
        }

        // If the pose is supported by enough inliers stop ransacs and continue
        if (nGood >= 50) {
          relocMap = vpCandidateKFs[i]->GetMap();
          bMatch = true;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    return false;
  } else {
    mnLastRelocFrameId = mCurrentFrame.mnId;
    mpRelocalizationTargetMap = relocMap;
    std::cout << "Relocalized!!" << std::endl;
    return true;
  }
}

void Tracking::Reset(bool bLocMap) {
  std::unique_lock<std::mutex> lock(mMutexReset);

  Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

  // Reset Local Mapping
  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
    mpLocalMapper->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  mpLoopClosing->RequestReset();
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
  mpKeyFrameDB->clear();
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearAtlas();
  mpAtlas->CreateNewMap();

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = TrackingState::NO_IMAGES_YET;

  mbReadyToInitialize = false;

  mRelativeFramePose = Sophus::SE3f();

  mCurrentFrame = Frame();
  mnLastRelocFrameId = 0;
  mLastFrame = Frame();
  mpReferenceKF = nullptr;
  mpLastKeyFrame = nullptr;
  mvIniMatches.clear();

  mbHasPrevDeltaFramePose = false;
  mbReset = false;
  mbResetActiveMap = false;

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap) {
  std::unique_lock<std::mutex> lock(mMutexReset);
  
  Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);

  std::shared_ptr<Map> pMap = mpAtlas->GetCurrentMap();

  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
    mpLocalMapper->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  mpLoopClosing->RequestResetActiveMap(pMap);
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
  mpKeyFrameDB->clearMap(pMap);  // Only clear the active map references
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearMap();

  mState = TrackingState::NO_IMAGES_YET;

  mbReadyToInitialize = false;

  mnLastRelocFrameId = mCurrentFrame.mnId;

  mCurrentFrame = Frame();
  mLastFrame = Frame();
  mpReferenceKF = nullptr;
  mpLastKeyFrame = nullptr;
  mvIniMatches.clear();

  mbHasPrevDeltaFramePose = false;
  mbResetActiveMap = false;

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::InformOnlyTracking(const bool& flag) { mbOnlyTracking = flag; }

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias& b, std::shared_ptr<KeyFrame> pCurrentKeyFrame) {
  std::shared_ptr<Map> pMap = pCurrentKeyFrame->GetMap();
  if(s != 1.0f) {
    mRelativeFramePose.translation() *= s;
  }

  mpLastKeyFrame = pCurrentKeyFrame;

  mLastFrame.SetNewBias(b);
  mCurrentFrame.SetNewBias(b);

  while (!mCurrentFrame.imuIsPreintegrated()) {
    usleep(500);
  }

  if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
    mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(), mLastFrame.mpLastKeyFrame->GetImuPosition(), mLastFrame.mpLastKeyFrame->GetVelocity());
  } else {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mLastFrame.mpImuPreintegrated->dT;

    mLastFrame.SetImuPoseVelocity(
      IMU::NormalizeRotation(Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
      twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
      Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }

  if (mCurrentFrame.mpImuPreintegrated) {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mCurrentFrame.mpImuPreintegrated->dT;

    mCurrentFrame.SetImuPoseVelocity(
        IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
        Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }
}

int Tracking::GetMatchesInliers() { return mnMatchesInliers; }

void Tracking::setForcedLost(bool forceLost) { mForcedLost = forceLost; }

void Tracking::setStereoInitDefaultPose(const Sophus::SE3f default_pose) { mStereoInitDefaultPose = default_pose; }

Sophus::SE3f Tracking::GetPoseRelativeToBase(Sophus::SE3f initialPose) {
  Eigen::Vector3f translation0 = initialPose.rotationMatrix().transpose()*initialPose.translation();
  Eigen::Vector3f translation1 = mCurrentFrame.GetPose().rotationMatrix().transpose()*mCurrentFrame.GetPose().translation()+mBaseTranslation;
  
  if(!mpLocalMapper->getIsDoneVIBA())
    translation1.setZero();
  return Sophus::SE3f(initialPose.rotationMatrix(), translation1);
}

void Tracking::RequestSystemReset() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbReset = true;
}

void Tracking::RequestResetActiveMap() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbResetActiveMap = true;
}

void Tracking::ActivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void Tracking::DeactivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

void Tracking::CheckTrackingModeChanged() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  if (mbActivateLocalizationMode) {
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped()) {
      usleep(1000);
    }

    InformOnlyTracking(true);
    mbActivateLocalizationMode = false;
  }
  if (mbDeactivateLocalizationMode) {
    InformOnlyTracking(false);
    mpLocalMapper->Release();
    mbDeactivateLocalizationMode = false;
  }
}

void Tracking::CheckTrackingReset() {
  if(mbReset) {
    Reset();
  } else if(mbResetActiveMap) {
    ResetActiveMap();
  }
}

}  // namespace MORB_SLAM
