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

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <mutex>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <memory>

#ifdef FactoryEngine
#include <apps/morb_dbow2/DBoW2/BowVector.h>
#include <apps/morb_dbow2/DBoW2/FeatureVector.h>
#else
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#endif
#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/ImuTypes.h"
#include "MORB_SLAM/KeyFrameDatabase.h"
#include "MORB_SLAM/MapPoint.h"
#include "MORB_SLAM/ORBVocabulary.h"
#include "MORB_SLAM/ORBextractor.h"
#include "MORB_SLAM/SerializationUtils.h"

namespace MORB_SLAM {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

class KeyFrame : public std::enable_shared_from_this<KeyFrame> {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& mnId;
    ar& const_cast<long unsigned int&>(mnFrameId);
    ar& const_cast<double&>(mTimeStamp);
    // Grid
    ar& const_cast<int&>(mnGridCols);
    ar& const_cast<int&>(mnGridRows);
    ar& const_cast<float&>(mfGridElementWidthInv);
    ar& const_cast<float&>(mfGridElementHeightInv);
    // Calibration parameters
    ar& const_cast<float&>(fx);
    ar& const_cast<float&>(fy);
    ar& const_cast<float&>(invfx);
    ar& const_cast<float&>(invfy);
    ar& const_cast<float&>(cx);
    ar& const_cast<float&>(cy);
    ar& const_cast<float&>(mbf);
    ar& const_cast<float&>(mb);
    ar& const_cast<float&>(mThDepth);
    serializeMatrix(ar, mDistCoef, version);
    // Number of Keypoints
    ar& const_cast<int&>(N);
    // KeyPoints
    serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
    serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
    ar& const_cast<std::vector<float>&>(mvuRight);
    ar& const_cast<std::vector<float>&>(mvDepth);
    serializeMatrix<Archive>(ar, mDescriptors, version);
    // BOW
    ar& mBowVec;
    ar& mFeatVec;
    // Scale
    ar& const_cast<int&>(mnScaleLevels);
    ar& const_cast<float&>(mfScaleFactor);
    ar& const_cast<float&>(mfLogScaleFactor);
    ar& const_cast<std::vector<float>&>(mvScaleFactors);
    ar& const_cast<std::vector<float>&>(mvLevelSigma2);
    ar& const_cast<std::vector<float>&>(mvInvLevelSigma2);
    // Image bounds and calibration
    ar& const_cast<int&>(mnMinX);
    ar& const_cast<int&>(mnMinY);
    ar& const_cast<int&>(mnMaxX);
    ar& const_cast<int&>(mnMaxY);
    // Pose
    serializeSophusSE3<Archive>(ar, mTcw, version);
    // MapPointsId associated to keypoints
    ar& mvBackupMapPointsId;
    // Grid
    ar& mGrid;
    // Connected KeyFrameWeight
    ar& mBackupConnectedKeyFrameIdWeights;
    // Spanning Tree and Loop Edges
    ar& mbFirstConnection;
    ar& mBackupParentId;
    ar& mvBackupChildrensId;
    ar& mvBackupLoopEdgesId;
    // Bad flags
    ar& mbNotErase;
    ar& mbToBeErased;
    ar& mbBad;

    ar& mnOriginMapId;

    // Camera variables
    ar& mnBackupIdCamera;
    ar& mnBackupIdCamera2;

    // Fisheye variables
    ar& const_cast<int&>(NLeft);
    ar& const_cast<int&>(NRight);
    serializeSophusSE3<Archive>(ar, mTlr, version);
    serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
    ar& mGridRight;

    // Inertial variables
    ar& mImuBias;
    ar& mBackupImuPreintegrated;
    ar& mImuCalib;
    ar& mBackupPrevKFId;
    ar& mBackupNextKFId;
    ar& bImu;
    ar& boost::serialization::make_array(mVw.data(), mVw.size());
    ar& boost::serialization::make_array(mOwb.data(), mOwb.size());
    ar& mbHasVelocity;
  }

 public:
  
  KeyFrame();
  KeyFrame(Frame& F, std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrameDatabase> pKFDB);
  ~KeyFrame();

  // Pose functions
  void SetPose(const Sophus::SE3f& Tcw);
  void SetVelocity(const Eigen::Vector3f& Vw_);

  Sophus::SE3f GetPose();

  Sophus::SE3f GetPoseInverse();
  Eigen::Vector3f GetCameraCenter();

  Eigen::Vector3f GetImuPosition();
  Eigen::Matrix3f GetImuRotation();
  Eigen::Matrix3f GetRotation();
  Eigen::Vector3f GetTranslation();
  Eigen::Vector3f GetVelocity();
  bool isVelocitySet();

  // Bag of Words Representation
  void ComputeBoW();

  // Covisibility graph functions
  void AddConnection(std::shared_ptr<KeyFrame> pKF, const int& weight);
  void EraseConnection(std::shared_ptr<KeyFrame> pKF);

  void UpdateConnections(bool upParent = true);
  void UpdateBestCovisibles();
  std::set<std::shared_ptr<KeyFrame>> GetConnectedKeyFrames();
  std::vector<std::shared_ptr<KeyFrame>> GetVectorCovisibleKeyFrames();
  std::vector<std::shared_ptr<KeyFrame>> GetBestCovisibilityKeyFrames(const int& N);
  std::vector<std::shared_ptr<KeyFrame>> GetCovisiblesByWeight(const int& w);
  int GetWeight(std::shared_ptr<KeyFrame> pKF);

  // Spanning tree functions
  void AddChild(std::shared_ptr<KeyFrame> pKF);
  void EraseChild(std::shared_ptr<KeyFrame> pKF);
  void ChangeParent(std::shared_ptr<KeyFrame> pKF);
  std::set<std::shared_ptr<KeyFrame>> GetChilds();
  std::shared_ptr<KeyFrame> GetParent();
  bool hasChild(std::shared_ptr<KeyFrame> pKF);
  void SetFirstConnection(bool bFirst);

  // Loop Edges
  void AddLoopEdge(std::shared_ptr<KeyFrame> pKF);
  std::set<std::shared_ptr<KeyFrame>> GetLoopEdges();

  // MapPoint observation functions
  void AddMapPoint(std::shared_ptr<MapPoint> pMP, const size_t& idx);
  void EraseMapPointMatch(const int& idx);
  void EraseMapPointMatch(std::shared_ptr<MapPoint> pMP);
  void ReplaceMapPointMatch(const int& idx, std::shared_ptr<MapPoint> pMP);
  std::set<std::shared_ptr<MapPoint>> GetMapPoints();
  std::vector<std::shared_ptr<MapPoint>> GetMapPointMatches();
  int TrackedMapPoints(const int& minObs);
  std::shared_ptr<MapPoint> GetMapPoint(const size_t& idx);

  // KeyPoint functions
  std::vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r, const bool bRight = false) const;
  bool UnprojectStereo(int i, Eigen::Vector3f& x3D);

  // Image
  bool IsInImage(const float& x, const float& y) const;

  // Enable/Disable bad flag changes
  void SetNotErase();
  void SetErase();

  // Set/check bad flag
  bool SetBadFlag();
  bool isBad();

  // Compute Scene Depth (q=2 median). Used in monocular.
  float ComputeSceneMedianDepth(const int q);

  static bool weightComp(int a, int b) { return a > b; }

  static bool lId(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame> pKF2) { return pKF1->mnId < pKF2->mnId; }

  std::shared_ptr<Map> GetMap();
  void UpdateMap(std::shared_ptr<Map> pMap);

  void SetNewBias(const IMU::Bias& b);

  Eigen::Vector3f GetGyroBias();
  Eigen::Vector3f GetAccBias();
  IMU::Bias GetImuBias();

  bool ProjectPointUnDistort(std::shared_ptr<MapPoint> pMP, cv::Point2f& kp, float& u, float& v);

  void PreSave(std::set<std::shared_ptr<KeyFrame>>& spKF, std::set<std::shared_ptr<MapPoint>>& spMP, std::set<std::shared_ptr<const GeometricCamera>>& spCam);
  void PostLoad(std::map<long unsigned int, std::shared_ptr<KeyFrame>>& mpKFid, std::map<long unsigned int, std::shared_ptr<MapPoint>>& mpMPid, std::map<unsigned int, std::shared_ptr<const GeometricCamera>>& mpCamId);

  void SetORBVocabulary(std::shared_ptr<ORBVocabulary> pORBVoc);
  void SetKeyFrameDatabase(std::shared_ptr<KeyFrameDatabase> pKFDB);

  bool bImu;

  static long unsigned int nKFsInMemory;

  // The following variables are accesed from only 1 thread or never change (no mutex needed).
 public:

  bool isPartiallyConstructed{false};
  
  // Only used in the new LocalInertialBA
  bool mbVerifyLocalInertialBA{false};
  
  static long unsigned int nNextId;
  long unsigned int mnId;
  const long unsigned int mnFrameId;

  const double mTimeStamp;

  // Grid (to speed up feature matching)
  const int mnGridCols;
  const int mnGridRows;
  const float mfGridElementWidthInv;
  const float mfGridElementHeightInv;

  // Variables used by the tracking
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnFuseTargetForKF;

  // Variables used by the local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnBAFixedForKF;

  // Variables used by the keyframe database
  long unsigned int mnLoopQuery;
  int mnLoopWords;
  float mLoopScore;
  long unsigned int mnRelocQuery;
  int mnRelocWords;
  float mRelocScore;
  long unsigned int mnMergeQuery;
  int mnMergeWords;
  float mMergeScore;
  long unsigned int mnPlaceRecognitionQuery;
  int mnPlaceRecognitionWords;
  float mPlaceRecognitionScore;

  // Variables used by loop closing
  Sophus::SE3f mTcwGBA;
  Sophus::SE3f mTcwBefGBA;
  Eigen::Vector3f mVwbGBA;
  Eigen::Vector3f mVwbBefGBA;
  IMU::Bias mBiasGBA;
  long unsigned int mnBAGlobalForKF;

  // Variables used by merging
  Sophus::SE3f mTcwMerge;
  Sophus::SE3f mTcwBefMerge;
  Sophus::SE3f mTwcBefMerge;
  Eigen::Vector3f mVwbMerge;
  long unsigned int mnBALocalForMerge;

  // Calibration parameters
  const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
  cv::Mat mDistCoef;

  // Number of KeyPoints
  const int N;

  // KeyPoints, stereo coordinate and descriptors (all associated by an index)
  const std::vector<cv::KeyPoint> mvKeys;
  const std::vector<cv::KeyPoint> mvKeysUn;
  const std::vector<float> mvuRight;  // negative value for monocular points
  const std::vector<float> mvDepth;   // negative value for monocular points
  const cv::Mat mDescriptors;

  // BoW
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // Scale
  const int mnScaleLevels;
  const float mfScaleFactor;
  const float mfLogScaleFactor;
  const std::vector<float> mvScaleFactors;
  const std::vector<float> mvLevelSigma2;
  const std::vector<float> mvInvLevelSigma2;

  // Image bounds and calibration
  const int mnMinX;
  const int mnMinY;
  const int mnMaxX;
  const int mnMaxY;

  // Preintegrated IMU measurements from previous keyframe
  std::shared_ptr<KeyFrame> mPrevKF;
  std::shared_ptr<KeyFrame> mNextKF;

  std::shared_ptr<IMU::Preintegrated> mpImuPreintegrated;
  IMU::Calib mImuCalib;

  unsigned int mnOriginMapId;

  // The following variables need to be accessed trough a mutex to be thread safe.
 protected:
  // sophus poses
  Sophus::SE3<float> mTcw;
  Eigen::Matrix3f mRcw;
  Sophus::SE3<float> mTwc;
  Eigen::Matrix3f mRwc;

  // IMU position
  Eigen::Vector3f mOwb;
  // Velocity (Only used for inertial SLAM)
  Eigen::Vector3f mVw;
  bool mbHasVelocity;

  // Transformation matrix between cameras in stereo fisheye
  Sophus::SE3<float> mTlr;
  Sophus::SE3<float> mTrl;

  // Imu bias
  IMU::Bias mImuBias;

  // MapPoints associated to keypoints
  std::vector<std::shared_ptr<MapPoint>> mvpMapPoints;
  // For save relation without pointer, this is necessary for save/load function
  std::vector<long long int> mvBackupMapPointsId;

  // BoW
  std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
  std::shared_ptr<ORBVocabulary> mpORBvocabulary;

  // Grid over the image to speed up feature matching
  std::vector<std::vector<std::vector<size_t>>> mGrid;
  std::vector<std::vector<std::vector<size_t>>> mGridRight;

  std::map<std::shared_ptr<KeyFrame>, int> mConnectedKeyFrameWeights;
  std::vector<std::shared_ptr<KeyFrame>> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;
  // For save relation without pointer, this is necessary for save/load function
  std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

  // Spanning Tree and Loop Edges
  bool mbFirstConnection;
  std::shared_ptr<KeyFrame> mpParent;
  std::set<std::shared_ptr<KeyFrame>> mspChildrens;
  std::set<std::shared_ptr<KeyFrame>> mspLoopEdges;
  // For save relation without pointer, this is necessary for save/load function
  long long int mBackupParentId;
  std::vector<long unsigned int> mvBackupChildrensId;
  std::vector<long unsigned int> mvBackupLoopEdgesId;

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  std::shared_ptr<Map> mpMap;

  // Backup variables for inertial
  long long int mBackupPrevKFId;
  long long int mBackupNextKFId;
  IMU::Preintegrated mBackupImuPreintegrated;

  // Backup for Cameras
  unsigned int mnBackupIdCamera, mnBackupIdCamera2;

  // Mutex
  std::mutex mMutexPose;  // for pose, velocity and biases
  std::mutex mMutexConnections;
  std::mutex mMutexFeatures;
  std::mutex mMutexMap;

 public:
  std::shared_ptr<const GeometricCamera> mpCamera, mpCamera2;

  Sophus::SE3f GetRelativePoseTrl();
  Sophus::SE3f GetRelativePoseTlr();

  // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
  const std::vector<cv::KeyPoint> mvKeysRight;

  const int NLeft, NRight;

  Sophus::SE3<float> GetRightPose();
  Sophus::SE3<float> GetRightPoseInverse();

  Eigen::Vector3f GetRightCameraCenter();

};

}  // namespace MORB_SLAM

