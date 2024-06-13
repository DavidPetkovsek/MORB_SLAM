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

#pragma once

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <unordered_set>
#include <fstream>

#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/ImuTypes.h"
#include "MORB_SLAM/KeyFrameDatabase.h"
#include "MORB_SLAM/LocalMapping.h"
#include "MORB_SLAM/LoopClosing.h"
#include "MORB_SLAM/ORBVocabulary.h"
#include "MORB_SLAM/ORBextractor.h"
#include "MORB_SLAM/Settings.h"
#include "MORB_SLAM/System.h"
#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/Camera.hpp"
#include "MORB_SLAM/Packet.hpp"

namespace MORB_SLAM {

class Tracking {
 public:
  
  Tracking(std::shared_ptr<ORBVocabulary> pVoc, const Atlas_ptr &pAtlas, std::shared_ptr<KeyFrameDatabase> pKFDB, const CameraType sensor, std::shared_ptr<Settings> settings);

  ~Tracking();

  // Preprocess the input and call Track(). Extract features and performs stereo
  // matching.
  StereoPacket GrabImageStereo(const cv::Mat& imRectLeft, const cv::Mat& imRectRight, const double& timestamp, const Camera_ptr &cam);
  RGBDPacket GrabImageRGBD(const cv::Mat& imRGB, const cv::Mat& imD, const double& timestamp, const Camera_ptr &cam);
  MonoPacket GrabImageMonocular(const cv::Mat& im, const double& timestamp, const Camera_ptr &cam);

  void GrabImuData(const std::vector<IMU::Point>& imuMeasurements);

  void SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper);
  void SetLoopClosing(std::shared_ptr<LoopClosing> pLoopClosing);

  // Load new settings
  // The focal lenght should be similar or scale prediction will fail when projecting points

  // Use this function if you have deactivated local mapping and you only want to localize the camera.
  void InformOnlyTracking(const bool& flag);

  void UpdateFrameIMU(const float s, const IMU::Bias& b, std::shared_ptr<KeyFrame> pCurrentKeyFrame);
  std::shared_ptr<KeyFrame> GetLastKeyFrame() { return mpLastKeyFrame; }

  void CreateMapInAtlas();

  //--
  int GetMatchesInliers();

  float GetImageScale();

  Sophus::SE3f getStereoInitDefaultPose() const { return mStereoInitDefaultPose; }
  void setStereoInitDefaultPose(const Sophus::SE3f default_pose);

#ifdef REGISTER_LOOP
  void RequestStop();
  bool isStopped();
  void Release();
  bool stopRequested();
#endif

 public:

  TrackingState mState;
  TrackingState mLastProcessedState;

  // Input sensor
  CameraType mSensor;

  // Current Frame
  Frame mCurrentFrame;
  Frame mLastFrame;

  // Initialization Variables (Monocular)
  std::vector<int> mvIniLastMatches;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point2f> mvbPrevMatched;
  std::vector<cv::Point3f> mvIniP3D;
  Frame mInitialFrame;

  // The sum of all the changes in translation (teleportations) caused by InitializeIMU(), Loop Closing, and Map Merging
  Eigen::Vector3f mBaseTranslation;
  // Stores the current KeyFrame's translation before each teleportation 
  Eigen::Vector3f mPreTeleportTranslation;
  // Set to true after teleportation occurs
  bool mTeleported;
  // Set to true right before a teleportation occurs, prevents mPreTeleportTranslation from being changed
  bool mLockPreTeleportTranslation;

  Sophus::SE3f mReturnPose;

  // Lists used to recover the full camera trajectory at the end of the execution. Basically we store the reference keyframe for each frame and its relative transformation
protected:

  Sophus::SE3f mRelativeFramePose;

  bool mFastInit;
  bool mStationaryInit;

  Sophus::SE3f mStereoInitDefaultPose;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

public:

  // True if local mapping is deactivated and we are performing only localization
  bool mbOnlyTracking;

  void CheckTrackingModeChanged();
  // This stops local mapping thread (map building) and performs only camera tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Reset the system (clear Atlas or the active map)
  void CheckTrackingReset();
  void RequestReset();
  void RequestResetActiveMap();

  bool fastIMUInitEnabled() const { return mFastInit; }
  bool stationaryIMUInitEnabled() const { return mStationaryInit; }

  void setForcedLost(bool forceLost);

 protected:
  // Main tracking function. It is independent of the input sensor.
  void Track();

  // Map initialization for stereo and RGB-D
  void StereoInitialization();

  // Map initialization for monocular
  void MonocularInitialization();
  void CreateInitialMapMonocular();

  void CheckReplacedInLastFrame();
  bool TrackReferenceKeyFrame();
  void UpdateLastFrame();
  bool TrackWithMotionModel();
  bool PredictStateIMU();

  bool Relocalization();

  void UpdateLocalMap();
  void UpdateLocalPoints();
  void UpdateLocalKeyFrames();

  bool TrackLocalMap();
  void SearchLocalPoints();

  bool NeedNewKeyFrame();
  void CreateNewKeyFrame();

  // Perform preintegration from last frame
  void PreintegrateIMU();

  // Reset IMU biases and compute frame velocity
  void ResetFrameIMU();

  Sophus::SE3f GetPoseRelativeToBase(Sophus::SE3f initialPose);

  void Reset(bool bLocMap = false);
  void ResetActiveMap(bool bLocMap = false);

  bool mbMapUpdated;

  // Imu preintegration from last frame
  std::shared_ptr<IMU::Preintegrated> mpImuPreintegratedFromLastKF;

  // Queue of IMU measurements between frames
  std::list<IMU::Point> mlQueueImuData;

  // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
  std::mutex mMutexImuQueue;

  // Imu calibration parameters
  std::shared_ptr<IMU::Calib> mpImuCalib;

  // Last Bias Estimation (at keyframe creation)

  // In case of performing only localization, this flag is true when there are no matches to points in the map. Still tracking will continue if there are
  // enough matches with temporal points. In that case we are doing visual odometry. The system will try to do relocalization to recover "zero-drift" localization to the map.
  bool notEnoughMatchPoints_trackOnlyMode;

  // Other Thread Pointers
  std::shared_ptr<LocalMapping> mpLocalMapper;
  std::shared_ptr<LoopClosing> mpLoopClosing;

  // ORB
  std::shared_ptr<ORBextractor> mpORBextractorLeft;
  std::shared_ptr<ORBextractor> mpORBextractorRight;
  std::shared_ptr<ORBextractor> mpIniORBextractor;

  // BoW
  std::shared_ptr<ORBVocabulary> mpORBVocabulary;
  std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;

  // Initalization (only for monocular)
  bool mbReadyToInitialize;

  // Local Map
  std::shared_ptr<KeyFrame> mpReferenceKF;
  std::vector<std::shared_ptr<KeyFrame>> mvpLocalKeyFrames;
  std::vector<std::shared_ptr<MapPoint>> mvpLocalMapPoints;

  // Atlas
  Atlas_ptr mpAtlas;

  // Calibration matrix
  cv::Mat mK;
  Eigen::Matrix3f mK_;
  cv::Mat mDistCoef;
  float mbf;
  float mImageScale;

  float mImuFreq;
  bool mInsertKFsLost;

  // New KeyFrame rules (according to fps)
  int mMinFrames;
  int mMaxFrames;

  int mnFramesToResetIMU;

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable and inserted from just one frame. Far points requiere a match in two keyframes.
  float mThDepth;

  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
  float mDepthMapFactor;

  // Current matches in frame
  int mnMatchesInliers;

  // Last Frame, KeyFrame and Relocalisation Info
  std::shared_ptr<KeyFrame> mpLastKeyFrame;
  unsigned int mnLastKeyFrameId;
  unsigned int mnLastRelocFrameId;
  double mTimeStampLost;
  double time_recently_lost;

  unsigned int mnFirstFrameId;
  unsigned int mnInitialFrameId;

  bool mbCreatedMap;

  // Motion Model
  bool mbHasPrevDeltaFramePose{false};
  Sophus::SE3f mPrevDeltaFramePose;

  std::list<std::shared_ptr<MapPoint>> mlpTemporalPoints;

  std::shared_ptr<const GeometricCamera> mpCamera;
  std::shared_ptr<const GeometricCamera> mpCamera2;

  int initID, lastID;

  Sophus::SE3f mTlr;

  void newParameterLoader(Settings& settings);

  bool mForcedLost;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;
  bool mbResetActiveMap;

#ifdef REGISTER_LOOP
  bool Stop();

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;
#endif
};
typedef std::shared_ptr<Tracking> Tracking_ptr;
typedef std::weak_ptr<Tracking> Tracking_wptr;

}  // namespace MORB_SLAM

