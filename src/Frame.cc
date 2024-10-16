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

#include "MORB_SLAM/Frame.h"

#include <MORB_SLAM/CameraModels/KannalaBrandt8.h>
#include <MORB_SLAM/CameraModels/Pinhole.h>

#include <thread>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/G2oTypes.h"
#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/MapPoint.h"
#include "MORB_SLAM/ORBextractor.h"
#include "MORB_SLAM/ORBmatcher.h"

namespace MORB_SLAM {

long unsigned int Frame::nNextId = 0;
bool Frame::mbInitialComputations = true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

// For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

Frame::Frame()
    : mpcpi(nullptr),
      mbHasPose(false),
      mbHasVelocity(false),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(nullptr),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(nullptr),
      mbImuPreintegrated(false),
      mpLastKeyFrame(nullptr),
      isPartiallyConstructed(true) {
  mpMutexImu = std::make_shared<std::mutex>();
  mnId = nNextId++;
}

Frame::~Frame(){}

// Copy Constructor.
Frame::Frame(const Frame &frame)
    : mpcpi(frame.mpcpi),
      mTcw(frame.mTcw),
      mbHasPose(false),
      mTlr(frame.mTlr),
      mTrl(frame.mTrl),
      mRlr(frame.mRlr),
      mtlr(frame.mtlr),
      mbHasVelocity(false),
      mpORBvocabulary(frame.mpORBvocabulary),
      mpORBextractorLeft(frame.mpORBextractorLeft),
      mpORBextractorRight(frame.mpORBextractorRight),
      mTimeStamp(frame.mTimeStamp),
      mK(frame.mK.clone()),
      mDistCoef(frame.mDistCoef.clone()),
      mbf(frame.mbf),
      mb(frame.mb),
      mThDepth(frame.mThDepth),
      N(frame.N),
      mvKeys(frame.mvKeys),
      mvKeysRight(frame.mvKeysRight),
      mvKeysUn(frame.mvKeysUn),
      mvpMapPoints(frame.mvpMapPoints),
      mvuRight(frame.mvuRight),
      mvDepth(frame.mvDepth),
      mBowVec(frame.mBowVec),
      mFeatVec(frame.mFeatVec),
      mDescriptors(frame.mDescriptors.clone()),
      mDescriptorsRight(frame.mDescriptorsRight.clone()),
      mvbOutlier(frame.mvbOutlier),
      mImuBias(frame.mImuBias),
      mImuCalib(frame.mImuCalib),
      mpImuPreintegrated(frame.mpImuPreintegrated),
      mpLastKeyFrame(frame.mpLastKeyFrame),
      mpPrevFrame(frame.mpPrevFrame),
      mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame),
      mnId(frame.mnId),
      mpReferenceKF(frame.mpReferenceKF),
      mnScaleLevels(frame.mnScaleLevels),
      mfScaleFactor(frame.mfScaleFactor),
      mfLogScaleFactor(frame.mfLogScaleFactor),
      mvScaleFactors(frame.mvScaleFactors),
      mvInvScaleFactors(frame.mvInvScaleFactors),
      mvLevelSigma2(frame.mvLevelSigma2),
      mvInvLevelSigma2(frame.mvInvLevelSigma2),
      mbImuPreintegrated(frame.mbImuPreintegrated),
      mpMutexImu(frame.mpMutexImu),
      camera{frame.camera},
      mpCamera(frame.mpCamera),
      mpCamera2(frame.mpCamera2),
      Nleft(frame.Nleft),
      Nright(frame.Nright),
      monoLeft(frame.monoLeft),
      monoRight(frame.monoRight),
      mvLeftToRightMatch(frame.mvLeftToRightMatch),
      mvRightToLeftMatch(frame.mvRightToLeftMatch),
      mvStereo3Dpoints(frame.mvStereo3Dpoints) {
  for (int i = 0; i < FRAME_GRID_COLS; i++)
    for (int j = 0; j < FRAME_GRID_ROWS; j++) {
      mGrid[i][j] = frame.mGrid[i][j];
      if (frame.Nleft > 0) {
        mGridRight[i][j] = frame.mGridRight[i][j];
      }
    }

  if (frame.mbHasPose) SetPose(frame.GetPose());

  if (frame.HasVelocity()) {
    SetVelocity(frame.GetVelocity());
  }
}

// Copy for ExternalMapViewer.
Frame::Frame(const Frame &frame, const bool copyExternalMapViewer)
    : mTcw(frame.mTcw),
      mnId(frame.mnId),
      mpReferenceKF(frame.mpReferenceKF),
      isPartiallyConstructed(true){}

// Constructor for rectified stereo cameras.
Frame::Frame(const Camera_ptr &cam, const cv::Mat &imLeft, const cv::Mat &imRight,
             const double &timeStamp, const std::shared_ptr<ORBextractor> &extractorLeft,
             const std::shared_ptr<ORBextractor> &extractorRight, std::shared_ptr<ORBVocabulary> voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth,
             const std::shared_ptr<const GeometricCamera> &pCamera,
             Frame *pPrevF, const IMU::Calib &ImuCalib)
    : mpcpi(nullptr),
      mbHasPose(false),
      mbHasVelocity(false),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractorLeft),
      mpORBextractorRight(extractorRight),
      mTimeStamp(timeStamp),
      mK(K.clone()),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(nullptr),
      mbImuPreintegrated(false),
      camera(cam),
      mpCamera(pCamera),
      mpCamera2(nullptr),
      mpLastKeyFrame(nullptr) {
  // Frame ID
  mnId = nNextId++;

  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  auto leftFut = camera->queueLeft(std::bind(&Frame::ExtractORB, this, true, imLeft, 0, 0));
  auto rightFut = camera->queueRight(std::bind(&Frame::ExtractORB, this, false, imRight, 0, 0));
  if(!leftFut.get() || !rightFut.get()) return;

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imLeft);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }
  mb = mbf / fx;
  N = mvKeys.size();
  if (mvKeys.empty()) return;

  UndistortKeyPoints();
  ComputeStereoMatches();

  mvpMapPoints = std::vector<std::shared_ptr<MapPoint>>(N, nullptr);
  mvbOutlier = std::vector<bool>(N, false);

  if (pPrevF && pPrevF->HasVelocity()) {
    SetVelocity(pPrevF->GetVelocity());
  } else {
    mVw.setZero();
  }

  mpMutexImu = std::make_shared<std::mutex>();

  // Set no stereo fisheye information
  Nleft = -1;
  Nright = -1;
  monoLeft = -1;
  monoRight = -1;

  AssignFeaturesToGrid();
}

// Constructor for RGB-D cameras.
Frame::Frame(const Camera_ptr &cam, const cv::Mat &imGray, const cv::Mat &imDepth,
             const double &timeStamp, const std::shared_ptr<ORBextractor> &extractor,
             std::shared_ptr<ORBVocabulary> voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
             const float &thDepth, const std::shared_ptr<const GeometricCamera> &pCamera,
             Frame *pPrevF, const IMU::Calib &ImuCalib)
    : mpcpi(nullptr),
      mbHasPose(false),
      mbHasVelocity(false),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractor),
      mpORBextractorRight(nullptr),
      mTimeStamp(timeStamp),
      mK(K.clone()),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(nullptr),
      mbImuPreintegrated(false),
      camera(cam), 
      mpCamera(pCamera),
      mpCamera2(nullptr),
      mpLastKeyFrame(nullptr) {
  // Frame ID
  mnId = nNextId++;

  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  ExtractORB(true, imGray, 0, 0);

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }
  mb = mbf / fx;
  N = mvKeys.size();
  if (mvKeys.empty()) return;

  UndistortKeyPoints();

  ComputeStereoFromRGBD(imDepth);

  mvpMapPoints = std::vector<std::shared_ptr<MapPoint>>(N, nullptr);

  mvbOutlier = std::vector<bool>(N, false);

  if (pPrevF) {
    if (pPrevF->HasVelocity()) SetVelocity(pPrevF->GetVelocity());
  } else {
    mVw.setZero();
  }

  mpMutexImu = std::make_shared<std::mutex>();

  // Set no stereo fisheye information
  Nleft = -1;
  Nright = -1;
  mvLeftToRightMatch = std::vector<int>(0);
  mvRightToLeftMatch = std::vector<int>(0);
  mvStereo3Dpoints = std::vector<Eigen::Vector3f>(0);
  monoLeft = -1;
  monoRight = -1;

  AssignFeaturesToGrid();
}

// Constructor for monocular cameras.
Frame::Frame(const Camera_ptr &cam, const cv::Mat &imGray, const double &timeStamp,
             const std::shared_ptr<ORBextractor> &extractor, std::shared_ptr<ORBVocabulary> voc,
             const std::shared_ptr<const GeometricCamera> &pCamera, cv::Mat &distCoef, const float &bf,
             const float &thDepth, Frame *pPrevF, const IMU::Calib &ImuCalib)
    : mpcpi(nullptr),
      mbHasPose(false),
      mbHasVelocity(false),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractor),
      mpORBextractorRight(nullptr),
      mTimeStamp(timeStamp),
      mK(pCamera->toK()),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(nullptr),
      mbImuPreintegrated(false),
      camera(cam),
      mpCamera(pCamera),
      mpCamera2(nullptr),
      mpLastKeyFrame(nullptr) {
  // Frame ID
  mnId = nNextId++;

  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  ExtractORB(true, imGray, 0, 1000);

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

    fx = mpCamera->toK().at<float>(0, 0);
    fy = mpCamera->toK().at<float>(1, 1);
    cx = mpCamera->toK().at<float>(0, 2);
    cy = mpCamera->toK().at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }
  mb = mbf / fx;
  N = mvKeys.size();
  if (mvKeys.empty()) return;

  UndistortKeyPoints();

  // Set no stereo information
  mvuRight = std::vector<float>(N, -1);
  mvDepth = std::vector<float>(N, -1);

  mvpMapPoints = std::vector<std::shared_ptr<MapPoint>>(N, nullptr);

  mvbOutlier = std::vector<bool>(N, false);

  // Set no stereo fisheye information
  Nleft = -1;
  Nright = -1;
  mvLeftToRightMatch = std::vector<int>(0);
  mvRightToLeftMatch = std::vector<int>(0);
  mvStereo3Dpoints = std::vector<Eigen::Vector3f>(0);
  monoLeft = -1;
  monoRight = -1;

  AssignFeaturesToGrid();

  if (pPrevF) {
    if (pPrevF->HasVelocity()) {
      SetVelocity(pPrevF->GetVelocity());
    }
  } else {
    mVw.setZero();
  }

  mpMutexImu = std::make_shared<std::mutex>();
}

// Constructor for non-rectified stereo cameras.
Frame::Frame(const Camera_ptr &cam, const cv::Mat &imLeft, const cv::Mat &imRight,
             const double &timeStamp, const std::shared_ptr<ORBextractor> &extractorLeft,
             const std::shared_ptr<ORBextractor> &extractorRight, std::shared_ptr<ORBVocabulary> voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth,
             const std::shared_ptr<const GeometricCamera> &pCamera, const std::shared_ptr<const GeometricCamera> &pCamera2,
             Sophus::SE3f &Tlr, Frame *pPrevF, const IMU::Calib &ImuCalib)
    : mpcpi(nullptr),
      mbHasPose(false),
      mbHasVelocity(false),
      mpORBvocabulary(voc),
      mpORBextractorLeft(extractorLeft),
      mpORBextractorRight(extractorRight),
      mTimeStamp(timeStamp),
      mK(K.clone()),
      mDistCoef(distCoef.clone()),
      mbf(bf),
      mThDepth(thDepth),
      mImuCalib(ImuCalib),
      mpImuPreintegrated(nullptr),
      mpPrevFrame(pPrevF),
      mpImuPreintegratedFrame(nullptr),
      mpReferenceKF(nullptr),
      mbImuPreintegrated(false),
      camera(cam),
      mpCamera(pCamera),
      mpCamera2(pCamera2),
      mpLastKeyFrame(nullptr) {
  imgLeft = imLeft.clone();
  imgRight = imRight.clone();

  // Frame ID
  mnId = nNextId++;

  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  auto leftFut = camera->queueLeft(std::bind(&Frame::ExtractORB, this, true, imLeft,
                    std::static_pointer_cast<const KannalaBrandt8>(mpCamera)->getLappingArea()[0],
                    std::static_pointer_cast<const KannalaBrandt8>(mpCamera)->getLappingArea()[1]));
  auto rightFut = camera->queueRight(std::bind(&Frame::ExtractORB, this, false, imRight,
                    std::static_pointer_cast<const KannalaBrandt8>(mpCamera2)->getLappingArea()[0],
                    std::static_pointer_cast<const KannalaBrandt8>(mpCamera2)->getLappingArea()[1]));
  if(!leftFut.get() || !rightFut.get()) return;

  // This is done only for the first Frame (or after a change in the
  // calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imLeft);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }
  mb = mbf / fx;
  Nleft = mvKeys.size();
  Nright = mvKeysRight.size();
  N = Nleft + Nright;

  if (N == 0) return;

  // Sophus/Eigen
  mTlr = Tlr;
  mTrl = mTlr.inverse();
  mRlr = mTlr.rotationMatrix();
  mtlr = mTlr.translation();

  ComputeStereoFishEyeMatches();

  // Put all descriptors in the same matrix
  cv::vconcat(mDescriptors, mDescriptorsRight, mDescriptors);

  mvpMapPoints = std::vector<std::shared_ptr<MapPoint>>(N, static_cast<std::shared_ptr<MapPoint>>(nullptr));
  mvbOutlier = std::vector<bool>(N, false);

  AssignFeaturesToGrid();

  mpMutexImu = std::make_shared<std::mutex>();

  UndistortKeyPoints();
}


void Frame::AssignFeaturesToGrid() {
  // Fill matrix with points
  const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

  int nReserve = 0.5f * N / (nCells);

  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++) {
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
      mGrid[i][j].reserve(nReserve);
      if (Nleft != -1) {
        mGridRight[i][j].reserve(nReserve);
      }
    }
  }

  for (int i = 0; i < N; i++) {
    const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i] : ((i < Nleft) ? mvKeys[i] : mvKeysRight[i - Nleft]);

    int nGridPosX, nGridPosY;
    if (PosInGrid(kp, nGridPosX, nGridPosY)) {
      if (Nleft == -1 || i < Nleft)
        mGrid[nGridPosX][nGridPosY].push_back(i);
      else
        mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
    }
  }
}

void Frame::ExtractORB(bool isLeft, const cv::Mat &im, const int x0, const int x1) {
  std::vector<int> vLapping = {x0, x1};
  if (isLeft)
    monoLeft = (*mpORBextractorLeft)(im, mvKeys, mDescriptors, vLapping);
  else
    monoRight = (*mpORBextractorRight)(im, mvKeysRight, mDescriptorsRight, vLapping);
}

void Frame::SetPose(const Sophus::SE3<float> &Tcw) {
  mTcw = Tcw;

  UpdatePoseMatrices();
  mbHasPose = true;
}

void Frame::SetNewBias(const IMU::Bias &b) {
  mImuBias = b;
  if (mpImuPreintegrated) mpImuPreintegrated->SetNewBias(b);
}

void Frame::SetVelocity(Eigen::Vector3f Vwb) {
  mVw = Vwb;
  mbHasVelocity = true;
}

Eigen::Vector3f Frame::GetVelocity() const { return mVw; }

void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb) {
  mVw = Vwb;
  mbHasVelocity = true;

  Sophus::SE3f Twb(Rwb, twb);
  Sophus::SE3f Tbw = Twb.inverse();

  mTcw = mImuCalib.mTcb * Tbw;

  UpdatePoseMatrices();
  mbHasPose = true;
}

void Frame::UpdatePoseMatrices() {
  Sophus::SE3<float> Twc = mTcw.inverse();
  mRwc = Twc.rotationMatrix();
  mOw = Twc.translation();
  mRcw = mTcw.rotationMatrix();
  mtcw = mTcw.translation();
}

Eigen::Matrix<float, 3, 1> Frame::GetImuPosition() const {
  return mRwc * mImuCalib.mTcb.translation() + mOw;
}

Eigen::Matrix<float, 3, 3> Frame::GetImuRotation() {
  return mRwc * mImuCalib.mTcb.rotationMatrix();
}

Sophus::SE3f Frame::GetRelativePoseTrl() { return mTrl; }

Sophus::SE3f Frame::GetRelativePoseTlr() { return mTlr; }

bool Frame::isInFrustum(std::shared_ptr<MapPoint>pMP, float viewingCosLimit) {
  if (Nleft == -1) {
    pMP->mbTrackInView = false;
    pMP->mTrackProjX = -1;
    pMP->mTrackProjY = -1;

    // 3D in absolute coordinates
    Eigen::Matrix<float, 3, 1> P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const Eigen::Matrix<float, 3, 1> Pc = mRcw * P + mtcw;
    const float Pc_dist = Pc.norm();

    // Check positive depth
    const float &PcZ = Pc(2);
    const float invz = 1.0f / PcZ;
    if (PcZ < 0.0f) return false;

    const Eigen::Vector2f uv = mpCamera->project(Pc);

    if (uv(0) < mnMinX || uv(0) > mnMaxX) return false;
    if (uv(1) < mnMinY || uv(1) > mnMaxY) return false;

    pMP->mTrackProjX = uv(0);
    pMP->mTrackProjY = uv(1);

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const Eigen::Vector3f PO = P - mOw;
    const float dist = PO.norm();

    if (dist < minDistance || dist > maxDistance) return false;

    // Check viewing angle
    Eigen::Vector3f Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn) / dist;

    if (viewCos < viewingCosLimit) return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist, this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = uv(0);
    pMP->mTrackProjXR = uv(0) - mbf * invz;

    pMP->mTrackDepth = Pc_dist;

    pMP->mTrackProjY = uv(1);
    pMP->mnTrackScaleLevel = nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
  } else {
    pMP->mbTrackInView = false;
    pMP->mbTrackInViewR = false;
    pMP->mnTrackScaleLevel = -1;
    pMP->mnTrackScaleLevelR = -1;

    pMP->mbTrackInView = isInFrustumChecks(pMP, viewingCosLimit);
    pMP->mbTrackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

    return pMP->mbTrackInView || pMP->mbTrackInViewR;
  }
}

std::vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel, const bool bRight) const {
  std::vector<size_t> vIndices;
  vIndices.reserve(N);

  float factorX = r;
  float factorY = r;

  const int nMinCellX = std::max(0, (int)floor((x - mnMinX - factorX) * mfGridElementWidthInv));
  if (nMinCellX >= FRAME_GRID_COLS) {
    return vIndices;
  }

  const int nMaxCellX = std::min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
  if (nMaxCellX < 0) {
    return vIndices;
  }

  const int nMinCellY = std::max(0, (int)floor((y - mnMinY - factorY) * mfGridElementHeightInv));
  if (nMinCellY >= FRAME_GRID_ROWS) {
    return vIndices;
  }

  const int nMaxCellY = std::min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + factorY) * mfGridElementHeightInv));
  if (nMaxCellY < 0) {
    return vIndices;
  }

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const std::vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
      if (vCell.empty()) continue;

      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]] :
                                    (!bRight) ? mvKeys[vCell[j]] : mvKeysRight[vCell[j]];
        if (bCheckLevels) {
          if (kpUn.octave < minLevel || maxLevel >= 0 && kpUn.octave > maxLevel)
            continue;
        }

        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        if (fabs(distx) < factorX && fabs(disty) < factorY)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
  posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
  posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
    return false;

  return true;
}

void Frame::ComputeBoW() {
  if (mBowVec.empty()) {
    std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Updates mBowVec and mFeatVec with the WordValue and NodeID respectively, for each descriptor
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
  }
}

void Frame::UndistortKeyPoints() {
  if (mDistCoef.at<float>(0) == 0.0) {
    mvKeysUn = mvKeys;
    return;
  }

  // Fill matrix with points
  cv::Mat mat(N, 2, CV_32F);

  for (int i = 0; i < N; i++) {
    mat.at<float>(i, 0) = mvKeys[i].pt.x;
    mat.at<float>(i, 1) = mvKeys[i].pt.y;
  }

  // Undistort points
  mat = mat.reshape(2);
  cv::undistortPoints(mat, mat, mpCamera->toK(), mDistCoef, cv::Mat(), mK);
  mat = mat.reshape(1);

  // Fill undistorted keypoint vector
  mvKeysUn.resize(N);
  for (int i = 0; i < N; i++) {
    cv::KeyPoint kp = mvKeys[i];
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    mvKeysUn[i] = kp;
  }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
  if (mDistCoef.at<float>(0) != 0.0) {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;
    mat.at<float>(1, 0) = imLeft.cols;
    mat.at<float>(1, 1) = 0.0;
    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imLeft.rows;
    mat.at<float>(3, 0) = imLeft.cols;
    mat.at<float>(3, 1) = imLeft.rows;

    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, mpCamera->toK(), mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    // Undistort corners
    mnMinX = std::min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    mnMaxX = std::max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    mnMinY = std::min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    mnMaxY = std::max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  } else {
    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
  }
}

void Frame::ComputeStereoMatches() {
  mvuRight = std::vector<float>(N, -1.0f);
  mvDepth = std::vector<float>(N, -1.0f);

  const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

  const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

  // Assign keypoints to row table
  std::vector<std::vector<size_t>> vRowIndices(nRows);

  for (int i = 0; i < nRows; i++) vRowIndices[i].reserve(200);

  const int Nr = mvKeysRight.size();

  for (int iR = 0; iR < Nr; iR++) {
    const cv::KeyPoint &kp = mvKeysRight[iR];
    const float &kpY = kp.pt.y;
    const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
    const int maxr = ceil(kpY + r);
    const int minr = floor(kpY - r);

    for (int yi = minr; yi <= maxr; yi++) vRowIndices[yi].push_back(iR);
  }

  // Set limits for search
  const float minZ = mb;
  const float minD = 0;
  const float maxD = mbf / minZ;

  // For each left keypoint search a match in the right image
  std::vector<std::pair<int, int>> vDistIdx;
  vDistIdx.reserve(N);

  for (int iL = 0; iL < N; iL++) {
    const cv::KeyPoint &kpL = mvKeys[iL];
    const int &levelL = kpL.octave;
    const float &vL = kpL.pt.y;
    const float &uL = kpL.pt.x;

    const std::vector<size_t> &vCandidates = vRowIndices[vL];

    if (vCandidates.empty()) continue;

    const float minU = uL - maxD;
    const float maxU = uL - minD;

    if (maxU < 0) continue;

    int bestDist = ORBmatcher::TH_HIGH;
    size_t bestIdxR = 0;

    const cv::Mat &dL = mDescriptors.row(iL);

    // Compare descriptor to right keypoints
    for (size_t iC = 0; iC < vCandidates.size(); iC++) {
      const size_t iR = vCandidates[iC];
      const cv::KeyPoint &kpR = mvKeysRight[iR];

      if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
        continue;

      const float &uR = kpR.pt.x;

      if (uR >= minU && uR <= maxU) {
        const cv::Mat &dR = mDescriptorsRight.row(iR);
        const int dist = ORBmatcher::DescriptorDistance(dL, dR);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdxR = iR;
        }
      }
    }

    // Subpixel match by correlation
    if (bestDist < thOrbDist) {
      // coordinates in image pyramid at keypoint scale
      const float uR0 = mvKeysRight[bestIdxR].pt.x;
      const float scaleFactor = mvInvScaleFactors[kpL.octave];
      const float scaleduL = round(kpL.pt.x * scaleFactor);
      const float scaledvL = round(kpL.pt.y * scaleFactor);
      const float scaleduR0 = round(uR0 * scaleFactor);

      // sliding window search
      const int w = 5;
      cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave]
                       .rowRange(scaledvL - w, scaledvL + w + 1)
                       .colRange(scaleduL - w, scaleduL + w + 1);

      int bestDist = INT_MAX;
      int bestincR = 0;
      const int L = 5;
      std::vector<float> vDists;
      vDists.resize(2 * L + 1);

      const float iniu = scaleduR0 + L - w;
      const float endu = scaleduR0 + L + w + 1;
      if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
        continue;

      for (int incR = -L; incR <= +L; incR++) {
        cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave]
                        .rowRange(scaledvL - w, scaledvL + w + 1)
                        .colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);

        float dist = cv::norm(IL, IR, cv::NORM_L1);
        if (dist < bestDist) {
          bestDist = dist;
          bestincR = incR;
        }

        vDists[L + incR] = dist;
      }

      if (bestincR == -L || bestincR == L) continue;

      // Sub-pixel match (Parabola fitting)
      const float dist1 = vDists[L + bestincR - 1];
      const float dist2 = vDists[L + bestincR];
      const float dist3 = vDists[L + bestincR + 1];

      const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

      if (deltaR < -1 || deltaR > 1) continue;

      // Re-scaled coordinate
      float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

      float disparity = (uL - bestuR);

      if (disparity >= minD && disparity < maxD) {
        if (disparity <= 0) {
          disparity = 0.01;
          bestuR = uL - 0.01;
        }
        mvDepth[iL] = mbf / disparity;
        mvuRight[iL] = bestuR;
        vDistIdx.emplace_back(bestDist, iL);
      }
    }
  }

  std::sort(vDistIdx.begin(), vDistIdx.end());
  const float median = vDistIdx[vDistIdx.size() / 2].first;
  const float thDist = 1.5f * 1.4f * median;

  for (int i = vDistIdx.size() - 1; i >= 0; i--) {
    if (vDistIdx[i].first < thDist)
      break;
    else {
      mvuRight[vDistIdx[i].second] = -1;
      mvDepth[vDistIdx[i].second] = -1;
    }
  }
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth) {
  mvuRight = std::vector<float>(N, -1);
  mvDepth = std::vector<float>(N, -1);

  for (int i = 0; i < N; i++) {
    const cv::KeyPoint &kp = mvKeys[i];
    const cv::KeyPoint &kpU = mvKeysUn[i];

    const float &v = kp.pt.y;
    const float &u = kp.pt.x;

    const float d = imDepth.at<float>(v, u);

    if (d > 0) {
      mvDepth[i] = d;
      mvuRight[i] = kpU.pt.x - mbf / d;
    }
  }
}

bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D) {
  const float z = mvDepth[i];
  if (z > 0) {
    const float u = mvKeysUn[i].pt.x;
    const float v = mvKeysUn[i].pt.y;
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    Eigen::Vector3f x3Dc(x, y, z);
    x3D = mRwc * x3Dc + mOw;
    return true;
  }
  return false;
}

bool Frame::imuIsPreintegrated() {
  std::unique_lock<std::mutex> lock(*mpMutexImu);
  return mbImuPreintegrated;
}

void Frame::setIntegrated() {
  while (!mpMutexImu)
    mpMutexImu = std::make_shared<std::mutex>();

  std::unique_lock<std::mutex> lock(*mpMutexImu);
  mbImuPreintegrated = true;
}


void Frame::ComputeStereoFishEyeMatches() {
  // Speed it up by matching keypoints in the lapping area
  std::vector<cv::KeyPoint> stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
  std::vector<cv::KeyPoint> stereoRight(mvKeysRight.begin() + monoRight, mvKeysRight.end());

  cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
  cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

  mvLeftToRightMatch = std::vector<int>(Nleft, -1);
  mvRightToLeftMatch = std::vector<int>(Nright, -1);
  mvDepth = std::vector<float>(Nleft, -1.0f);
  mvuRight = std::vector<float>(Nleft, -1);
  mvStereo3Dpoints = std::vector<Eigen::Vector3f>(Nleft);

  // Perform a brute force between Keypoint in the left and right image
  std::vector<std::vector<cv::DMatch>> matches;

  BFmatcher.knnMatch(stereoDescLeft, stereoDescRight, matches, 2);

  int nMatches = 0;
  int descMatches = 0;

  // Check matches using Lowe's ratio
  for (std::vector<std::vector<cv::DMatch>>::iterator it = matches.begin();
       it != matches.end(); ++it) {
    if ((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7) {
      // For every good match, check parallax and reprojection error to discard spurious matches
      Eigen::Vector3f p3D;
      descMatches++;
      float sigma1 = mvLevelSigma2[mvKeys[(*it)[0].queryIdx + monoLeft].octave];
      float sigma2 = mvLevelSigma2[mvKeysRight[(*it)[0].trainIdx + monoRight].octave];
      float depth = std::static_pointer_cast<const KannalaBrandt8>(mpCamera)->TriangulateMatches(
          mpCamera2, mvKeys[(*it)[0].queryIdx + monoLeft], mvKeysRight[(*it)[0].trainIdx + monoRight], mRlr, mtlr, sigma1, sigma2, p3D);
      if (depth > 0.0001f) {
        mvLeftToRightMatch[(*it)[0].queryIdx + monoLeft] = (*it)[0].trainIdx + monoRight;
        mvRightToLeftMatch[(*it)[0].trainIdx + monoRight] = (*it)[0].queryIdx + monoLeft;
        mvStereo3Dpoints[(*it)[0].queryIdx + monoLeft] = p3D;
        mvDepth[(*it)[0].queryIdx + monoLeft] = depth;
        nMatches++;
      }
    }
  }
}

bool Frame::isInFrustumChecks(std::shared_ptr<MapPoint>pMP, float viewingCosLimit, bool bRight) {
  // 3D in absolute coordinates
  Eigen::Vector3f P = pMP->GetWorldPos();

  Eigen::Matrix3f mR;
  Eigen::Vector3f mt, twc;
  if (bRight) {
    Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
    Eigen::Vector3f trl = mTrl.translation();
    mR = Rrl * mRcw;
    mt = Rrl * mtcw + trl;
    twc = mRwc * mTlr.translation() + mOw;
  } else {
    mR = mRcw;
    mt = mtcw;
    twc = mOw;
  }

  // 3D in camera coordinates
  Eigen::Vector3f Pc = mR * P + mt;
  const float Pc_dist = Pc.norm();
  const float &PcZ = Pc(2);

  // Check positive depth
  if (PcZ < 0.0f) return false;

  // Project in image and check it is not outside
  Eigen::Vector2f uv;
  if (bRight)
    uv = mpCamera2->project(Pc);
  else
    uv = mpCamera->project(Pc);

  if (uv(0) < mnMinX || uv(0) > mnMaxX) return false;
  if (uv(1) < mnMinY || uv(1) > mnMaxY) return false;

  // Check distance is in the scale invariance region of the MapPoint
  const float maxDistance = pMP->GetMaxDistanceInvariance();
  const float minDistance = pMP->GetMinDistanceInvariance();
  const Eigen::Vector3f PO = P - twc;
  const float dist = PO.norm();

  if (dist < minDistance || dist > maxDistance) return false;

  // Check viewing angle
  Eigen::Vector3f Pn = pMP->GetNormal();

  const float viewCos = PO.dot(Pn) / dist;

  if (viewCos < viewingCosLimit) return false;

  // Predict scale in the image
  const int nPredictedLevel = pMP->PredictScale(dist, this);

  if (bRight) {
    pMP->mTrackProjXR = uv(0);
    pMP->mTrackProjYR = uv(1);
    pMP->mnTrackScaleLevelR = nPredictedLevel;
    pMP->mTrackViewCosR = viewCos;
  } else {
    pMP->mTrackProjX = uv(0);
    pMP->mTrackProjY = uv(1);
    pMP->mnTrackScaleLevel = nPredictedLevel;
    pMP->mTrackViewCos = viewCos;
    pMP->mTrackDepth = Pc_dist;
  }

  return true;
}

Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i) {
  return mRwc * mvStereo3Dpoints[i] + mOw;
}

}  // namespace MORB_SLAM
