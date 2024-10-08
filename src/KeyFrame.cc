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

#include "MORB_SLAM/KeyFrame.h"

#include <mutex>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/ImuTypes.h"

namespace MORB_SLAM {

long unsigned int KeyFrame::nKFsInMemory = 0;
long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame()
    : mnFrameId(0),
      mTimeStamp(0),
      mnGridCols(FRAME_GRID_COLS),
      mnGridRows(FRAME_GRID_ROWS),
      mfGridElementWidthInv(0),
      mfGridElementHeightInv(0),
      mnTrackReferenceForFrame(0),
      mnFuseTargetForKF(0),
      mnBALocalForKF(0),
      mnBAFixedForKF(0),
      mnLoopQuery(0),
      mnLoopWords(0),
      mnRelocQuery(0),
      mnRelocWords(0),
      mnMergeQuery(0),
      mnMergeWords(0),
      mnPlaceRecognitionQuery(0),
      mnPlaceRecognitionWords(0),
      mPlaceRecognitionScore(0),
      mnBAGlobalForKF(0),
      mnBALocalForMerge(0),
      fx(0),
      fy(0),
      cx(0),
      cy(0),
      invfx(0),
      invfy(0),
      mbf(0),
      mb(0),
      mThDepth(0),
      N(0),
      mnScaleLevels(0),
      mfScaleFactor(0),
      mfLogScaleFactor(0),
      mvScaleFactors(0),
      mvLevelSigma2(0),
      mvInvLevelSigma2(0),
      mnMinX(0),
      mnMinY(0),
      mnMaxX(0),
      mnMaxY(0),
      mPrevKF(nullptr),
      mNextKF(nullptr),
      mbHasVelocity(false),
      mbFirstConnection(true),
      mpParent(nullptr),
      mbNotErase(false),
      mbToBeErased(false),
      mbBad(false),
      NLeft(0),
      NRight(0),
      isPartiallyConstructed(true) {
        nKFsInMemory++;
      }

KeyFrame::KeyFrame(Frame &F, std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrameDatabase> pKFDB)
    : bImu(pMap->isImuInitialized()),
      mnFrameId(F.mnId),
      mTimeStamp(F.mTimeStamp),
      mnGridCols(FRAME_GRID_COLS),
      mnGridRows(FRAME_GRID_ROWS),
      mfGridElementWidthInv(F.mfGridElementWidthInv),
      mfGridElementHeightInv(F.mfGridElementHeightInv),
      mnTrackReferenceForFrame(0),
      mnFuseTargetForKF(0),
      mnBALocalForKF(0),
      mnBAFixedForKF(0),
      mnLoopQuery(0),
      mnLoopWords(0),
      mnRelocQuery(0),
      mnRelocWords(0),
      mnPlaceRecognitionQuery(0),
      mnPlaceRecognitionWords(0),
      mPlaceRecognitionScore(0),
      mnBAGlobalForKF(0),
      mnBALocalForMerge(0),
      fx(F.fx),
      fy(F.fy),
      cx(F.cx),
      cy(F.cy),
      invfx(F.invfx),
      invfy(F.invfy),
      mbf(F.mbf),
      mb(F.mb),
      mThDepth(F.mThDepth),
      mDistCoef(F.mDistCoef),
      N(F.N),
      mvKeys(F.mvKeys),
      mvKeysUn(F.mvKeysUn),
      mvuRight(F.mvuRight),
      mvDepth(F.mvDepth),
      mDescriptors(F.mDescriptors.clone()),
      mBowVec(F.mBowVec),
      mFeatVec(F.mFeatVec),
      mnScaleLevels(F.mnScaleLevels),
      mfScaleFactor(F.mfScaleFactor),
      mfLogScaleFactor(F.mfLogScaleFactor),
      mvScaleFactors(F.mvScaleFactors),
      mvLevelSigma2(F.mvLevelSigma2),
      mvInvLevelSigma2(F.mvInvLevelSigma2),
      mnMinX(F.mnMinX),
      mnMinY(F.mnMinY),
      mnMaxX(F.mnMaxX),
      mnMaxY(F.mnMaxY),
      mPrevKF(nullptr),
      mNextKF(nullptr),
      mpImuPreintegrated(F.mpImuPreintegrated),
      mImuCalib(F.mImuCalib),
      mbHasVelocity(false),
      mTlr(F.GetRelativePoseTlr()),
      mTrl(F.GetRelativePoseTrl()),
      mvpMapPoints(F.mvpMapPoints),
      mpKeyFrameDB(pKFDB),
      mpORBvocabulary(F.mpORBvocabulary),
      mbFirstConnection(true),
      mpParent(nullptr),
      mbNotErase(false),
      mbToBeErased(false),
      mbBad(false),
      mpMap(pMap),
      mpCamera(F.mpCamera),
      mpCamera2(F.mpCamera2),
      mvKeysRight(F.mvKeysRight),
      NLeft(F.Nleft),
      NRight(F.Nright) {
  mnId = nNextId++;
  nKFsInMemory++;

  mGrid.resize(mnGridCols);
  if (F.Nleft != -1) mGridRight.resize(mnGridCols);
  for (int i = 0; i < mnGridCols; i++) {
    mGrid[i].resize(mnGridRows);
    if (F.Nleft != -1) mGridRight[i].resize(mnGridRows);
    for (int j = 0; j < mnGridRows; j++) {
      mGrid[i][j] = F.mGrid[i][j];
      if (F.Nleft != -1) {
        mGridRight[i][j] = F.mGridRight[i][j];
      }
    }
  }

  if (!F.HasVelocity()) {
    mVw.setZero();
    mbHasVelocity = false;
  } else {
    mVw = F.GetVelocity();
    mbHasVelocity = true;
  }

  mImuBias = F.mImuBias;
  SetPose(F.GetPose());

  mnOriginMapId = pMap->GetId();
}
KeyFrame::~KeyFrame() {
  nKFsInMemory--;
}

void KeyFrame::ComputeBoW() {
  if (mBowVec.empty() || mFeatVec.empty()) {
    std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from leaves up) We assume the vocabulary tree has 6 levels, change the 4 otherwise
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
  }
}

void KeyFrame::SetPose(const Sophus::SE3f &Tcw) {
  std::unique_lock<std::mutex> lock(mMutexPose);

  mTcw = Tcw;
  mRcw = mTcw.rotationMatrix();
  mTwc = mTcw.inverse();
  mRwc = mTwc.rotationMatrix();

  if (mImuCalib.isSet()) {
    mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
  }
}

void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw) {
  std::unique_lock<std::mutex> lock(mMutexPose);
  mVw = Vw;
  mbHasVelocity = true;
}

Sophus::SE3f KeyFrame::GetPose() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTcw;
}

Sophus::SE3f KeyFrame::GetPoseInverse() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTwc;
}

Eigen::Vector3f KeyFrame::GetCameraCenter() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTwc.translation();
}

Eigen::Vector3f KeyFrame::GetImuPosition() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mOwb;
}

Eigen::Matrix3f KeyFrame::GetImuRotation() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Eigen::Matrix3f KeyFrame::GetRotation() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mRcw;
}

Eigen::Vector3f KeyFrame::GetTranslation() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTcw.translation();
}

Eigen::Vector3f KeyFrame::GetVelocity() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mVw;
}

bool KeyFrame::isVelocitySet() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mbHasVelocity;
}

void KeyFrame::AddConnection(std::shared_ptr<KeyFrame> pKF, const int &weight) {
  {
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF] = weight;
    else if (mConnectedKeyFrameWeights[pKF] != weight)
      mConnectedKeyFrameWeights[pKF] = weight;
    else
      return;
  }

  UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
  std::unique_lock<std::mutex> lock(mMutexConnections);
  std::vector<std::pair<int, std::shared_ptr<KeyFrame>>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (std::map<std::shared_ptr<KeyFrame>, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
    vPairs.push_back(std::make_pair(mit->second, mit->first));

  sort(vPairs.begin(), vPairs.end());
  std::list<std::shared_ptr<KeyFrame>> lKFs;
  std::list<int> lWs;
  for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
    if (!vPairs[i].second->isBad()) {
      lKFs.push_front(vPairs[i].second);
      lWs.push_front(vPairs[i].first);
    }
  }

  mvpOrderedConnectedKeyFrames = std::vector<std::shared_ptr<KeyFrame>>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetConnectedKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexConnections);
  std::set<std::shared_ptr<KeyFrame>> s;
  for (std::map<std::shared_ptr<KeyFrame>, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); mit++)
    s.insert(mit->first);
  return s;
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetVectorCovisibleKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexConnections);
  return mvpOrderedConnectedKeyFrames;
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
  std::unique_lock<std::mutex> lock(mMutexConnections);
  if ((int)mvpOrderedConnectedKeyFrames.size() < N)
    return mvpOrderedConnectedKeyFrames;
  else
    return std::vector<std::shared_ptr<KeyFrame>>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N); //SUS
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetCovisiblesByWeight(const int &w) {
  std::unique_lock<std::mutex> lock(mMutexConnections);

  if (mvpOrderedConnectedKeyFrames.empty()) {
    return std::vector<std::shared_ptr<KeyFrame>>();
  }

  std::vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);

  if (it == mvOrderedWeights.end() && mvOrderedWeights.back() < w) {
    return std::vector<std::shared_ptr<KeyFrame>>();
  } else {
    int n = it - mvOrderedWeights.begin();
    return std::vector<std::shared_ptr<KeyFrame>>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
  }
}

int KeyFrame::GetWeight(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lock(mMutexConnections);
  if (mConnectedKeyFrameWeights.count(pKF))
    return mConnectedKeyFrameWeights[pKF];
  else
    return 0;
}

void KeyFrame::AddMapPoint(std::shared_ptr<MapPoint>pMP, const size_t &idx) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  mvpMapPoints[idx] = pMP;
}

void KeyFrame::EraseMapPointMatch(const int &idx) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  mvpMapPoints[idx] = nullptr;
}

void KeyFrame::EraseMapPointMatch(std::shared_ptr<MapPoint>pMP) {
  std::tuple<int, int> indexes = pMP->GetIndexInKeyFrame(shared_from_this());
  int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
  if (leftIndex != -1) mvpMapPoints[leftIndex] = nullptr;
  if (rightIndex != -1) mvpMapPoints[rightIndex] = nullptr;
}

void KeyFrame::ReplaceMapPointMatch(const int &idx, std::shared_ptr<MapPoint>pMP) {
  mvpMapPoints[idx] = pMP;
}

std::set<std::shared_ptr<MapPoint>> KeyFrame::GetMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  std::set<std::shared_ptr<MapPoint>> s;
  for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
    if (!mvpMapPoints[i]) continue;
    std::shared_ptr<MapPoint>pMP = mvpMapPoints[i];
    if (!pMP->isBad()) s.insert(pMP);
  }
  return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);

  int nPoints = 0;
  const bool bCheckObs = minObs > 0;
  for (int i = 0; i < N; i++) {
    std::shared_ptr<MapPoint>pMP = mvpMapPoints[i];
    if (pMP && !pMP->isBad()) {
      if (bCheckObs) {
        if (mvpMapPoints[i]->Observations() >= minObs) nPoints++;
      } else
        nPoints++; //SUS
    }
  }

  return nPoints;
}

std::vector<std::shared_ptr<MapPoint>> KeyFrame::GetMapPointMatches() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mvpMapPoints;
}

std::shared_ptr<MapPoint>KeyFrame::GetMapPoint(const size_t &idx) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent) {
  std::map<std::shared_ptr<KeyFrame>, int> KFcounter;

  std::vector<std::shared_ptr<MapPoint>> vpMP;

  {
    std::unique_lock<std::mutex> lockMPs(mMutexFeatures);
    vpMP = mvpMapPoints;
  }

  // For all map points in this KF, check in which other KFs they are seen, and increase the counter for those KFs
  for (std::vector<std::shared_ptr<MapPoint>>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
    std::shared_ptr<MapPoint>pMP = *vit;

    if (!pMP || pMP->isBad()) continue;

    std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = pMP->GetObservations();

    for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      if(std::shared_ptr<KeyFrame> pKF = (mit->first).lock()){
        if (pKF->mnId == mnId || pKF->isBad() || pKF->GetMap() != mpMap)
          continue;
        KFcounter[pKF]++;
      }
    }
  }

  // This should not happen
  if (KFcounter.empty()) return;

  // If the counter is greater than threshold add connection
  // In case no keyframe counter is over threshold add the one with maximum counter
  int nmax = 0;
  std::shared_ptr<KeyFrame> pKFmax = nullptr;
  const int th = 15;

  std::vector<std::pair<int, std::shared_ptr<KeyFrame>>> vPairs;
  vPairs.reserve(KFcounter.size());
  if (!upParent) std::cout << "UPDATE_CONN: current KF " << mnId << std::endl;

  std::shared_ptr<KeyFrame> self = shared_from_this();

  for (std::map<std::shared_ptr<KeyFrame>, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
    if (!upParent)
      std::cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << std::endl;
    if (mit->second > nmax) {
      nmax = mit->second;
      pKFmax = mit->first;
    }
    if (mit->second >= th) {
      vPairs.push_back(std::make_pair(mit->second, mit->first));
      (mit->first)->AddConnection(self, mit->second);
    }
  }

  if (vPairs.empty()) {
    vPairs.push_back(std::make_pair(nmax, pKFmax));
    pKFmax->AddConnection(self, nmax);
  }

  sort(vPairs.begin(), vPairs.end());
  std::list<std::shared_ptr<KeyFrame>> lKFs;
  std::list<int> lWs;
  for (size_t i = 0; i < vPairs.size(); i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  {
    std::unique_lock<std::mutex> lockCon(mMutexConnections);

    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames = std::vector<std::shared_ptr<KeyFrame>>(lKFs.begin(), lKFs.end());
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

    if (mbFirstConnection && mnId != mpMap->GetInitKFid()) {
      mpParent = mvpOrderedConnectedKeyFrames.front();
      mpParent->AddChild(self);
      mbFirstConnection = false;
    }
  }
}

void KeyFrame::AddChild(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(std::shared_ptr<KeyFrame> pKF) {
  std::shared_ptr<KeyFrame> self = shared_from_this();
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  if (pKF == self) {
    std::cout << "ERROR: Change parent KF, the parent and child are the same KF" << std::endl;
    throw std::invalid_argument("The parent and child can not be the same");
  }

  mpParent = pKF;
  if(pKF)
    pKF->AddChild(self);
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetChilds() {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mspChildrens;
}

std::shared_ptr<KeyFrame>KeyFrame::GetParent() {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mpParent;
}

bool KeyFrame::hasChild(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mspChildrens.count(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst) {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mbFirstConnection = bFirst;
}

void KeyFrame::AddLoopEdge(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  mbNotErase = true;
  mspLoopEdges.insert(pKF);
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetLoopEdges() {
  std::unique_lock<std::mutex> lockCon(mMutexConnections);
  return mspLoopEdges;
}

void KeyFrame::SetNotErase() {
  std::unique_lock<std::mutex> lock(mMutexConnections);
  mbNotErase = true;
}

void KeyFrame::SetErase() {
  {
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if (mspLoopEdges.empty()) {
      mbNotErase = false;
    }
  }

  if (mbToBeErased) {
    SetBadFlag();
  }
}

bool KeyFrame::SetBadFlag() {
  {
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if (mnId == mpMap->GetInitKFid()) {
      return false;
    } else if (mbNotErase) {
      mbToBeErased = true;
      return false;
    }
  }
  
  std::shared_ptr<KeyFrame> self = shared_from_this();

  for (std::map<std::shared_ptr<KeyFrame>, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
    mit->first->EraseConnection(self);

  for (size_t i = 0; i < mvpMapPoints.size(); i++)
    if (mvpMapPoints[i])
      mvpMapPoints[i]->EraseObservation(self);

  {
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::unique_lock<std::mutex> lock1(mMutexFeatures);

    if(mPrevKF) { 
      mPrevKF->mNextKF = mNextKF;
    }
    if(mNextKF) {
      if(mpImuPreintegrated && mNextKF->mpImuPreintegrated) {
        mNextKF->mpImuPreintegrated->MergePrevious(mpImuPreintegrated);
      }
      mNextKF->mPrevKF = mPrevKF;
    }
    mPrevKF = nullptr;
    mNextKF = nullptr;

    mConnectedKeyFrameWeights.clear();
    mvpOrderedConnectedKeyFrames.clear();
    
    mspLoopEdges.clear();

    // Update Spanning Tree
    std::set<std::shared_ptr<KeyFrame>> sParentCandidates;
    if (mpParent) sParentCandidates.insert(mpParent);

    // Assign at each iteration one children with a parent (the pair with highest covisibility weight) Include that children as new parent candidate for the rest
    while (!mspChildrens.empty()) {
      bool bContinue = false;

      int max = -1;
      std::shared_ptr<KeyFrame> pC;
      std::shared_ptr<KeyFrame> pP;

      for (std::shared_ptr<KeyFrame> pKF : mspChildrens) {
        if (!pKF || pKF->isBad()) continue;

        // Check if a parent candidate is connected to the keyframe
        std::vector<std::shared_ptr<KeyFrame>> vpConnected = pKF->GetVectorCovisibleKeyFrames();
        for (std::shared_ptr<KeyFrame>connectedKF : vpConnected) {
          for (std::shared_ptr<KeyFrame>candidate : sParentCandidates) {
            if (connectedKF->mnId == candidate->mnId) {
              int w = pKF->GetWeight(connectedKF);
              if (w > max) {
                pC = pKF;
                pP = connectedKF;
                max = w;
                bContinue = true;
              }
            }
          }
        }
      }

      if (bContinue) {
        pC->ChangeParent(pP);
        sParentCandidates.insert(pC);
        mspChildrens.erase(pC);
      } else
        break;
    }

    // If a children has no covisibility links with any parent candidate, assign
    // to the original parent of this KF
    if (!mspChildrens.empty()) {
      for (std::shared_ptr<KeyFrame> pKF : mspChildrens) {
        if (!pKF || pKF->isBad()) continue;
        pKF->ChangeParent(mpParent);
      }
    }

    if (mpParent) {
      mpParent->EraseChild(self);
      mpParent = nullptr;
    }
    mbBad = true;
  }

  mpMap->EraseKeyFrame(self);
  mpKeyFrameDB->erase(self);
  return true;
}

bool KeyFrame::isBad() {
  std::unique_lock<std::mutex> lock(mMutexConnections);
  return mbBad;
}

void KeyFrame::EraseConnection(std::shared_ptr<KeyFrame> pKF) {
  bool bUpdate = false;
  {
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.count(pKF)) {
      mConnectedKeyFrameWeights.erase(pKF);
      bUpdate = true;
    }
  }

  if (bUpdate) UpdateBestCovisibles();
}

std::vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const {
  std::vector<size_t> vIndices;
  vIndices.reserve(N);

  float factorX = r;
  float factorY = r;

  const int nMinCellX = std::max(0, (int)std::floor((x - mnMinX - factorX) * mfGridElementWidthInv));
  if (nMinCellX >= mnGridCols) return vIndices;

  const int nMaxCellX = std::min((int)mnGridCols - 1, (int)std::ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
  if (nMaxCellX < 0) return vIndices;

  const int nMinCellY = std::max(0, (int)std::floor((y - mnMinY - factorY) * mfGridElementHeightInv));
  if (nMinCellY >= mnGridRows) return vIndices;

  const int nMaxCellY = std::min((int)mnGridRows - 1, (int)std::ceil((y - mnMinY + factorY) * mfGridElementHeightInv));
  if (nMaxCellY < 0) return vIndices;

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const std::vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]] : (!bRight) ? mvKeys[vCell[j]] : mvKeysRight[vCell[j]];
        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        if (std::fabs(distx) < r && std::fabs(disty) < r) vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const {
  return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D) {
  const float z = mvDepth[i];
  if (z > 0) {
    const float u = mvKeys[i].pt.x;
    const float v = mvKeys[i].pt.y;
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    Eigen::Vector3f x3Dc(x, y, z);

    std::unique_lock<std::mutex> lock(mMutexPose);
    x3D = mRwc * x3Dc + mTwc.translation();
    return true;
  } else
    return false;
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
  if (N == 0) return -1.0;

  std::vector<std::shared_ptr<MapPoint>> vpMapPoints;
  Eigen::Matrix3f Rcw;
  Eigen::Vector3f tcw;
  {
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPose);
    vpMapPoints = mvpMapPoints;
    tcw = mTcw.translation();
    Rcw = mRcw;
  }

  std::vector<float> vDepths;
  vDepths.reserve(N);
  Eigen::Matrix<float, 1, 3> Rcw2 = Rcw.row(2);
  float zcw = tcw(2);
  for (int i = 0; i < N; i++) {
    if (mvpMapPoints[i]) {
      std::shared_ptr<MapPoint>pMP = mvpMapPoints[i];
      Eigen::Vector3f x3Dw = pMP->GetWorldPos();
      float z = Rcw2.dot(x3Dw) + zcw;
      vDepths.push_back(z);
    }
  }

  sort(vDepths.begin(), vDepths.end());

  return vDepths[(vDepths.size() - 1) / q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b) {
  std::unique_lock<std::mutex> lock(mMutexPose);
  mImuBias = b;
  if (mpImuPreintegrated) mpImuPreintegrated->SetNewBias(b);
}

Eigen::Vector3f KeyFrame::GetGyroBias() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

Eigen::Vector3f KeyFrame::GetAccBias() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mImuBias;
}

std::shared_ptr<Map> KeyFrame::GetMap() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mpMap;
}

void KeyFrame::UpdateMap(std::shared_ptr<Map> pMap) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mpMap = pMap;
}

void KeyFrame::PreSave(std::set<std::shared_ptr<KeyFrame>> &spKF, std::set<std::shared_ptr<MapPoint>> &spMP, std::set<std::shared_ptr<const GeometricCamera>> &spCam) {
  // Save the id of each MapPoint in this KF, there can be null pointer in the std::vector
  mvBackupMapPointsId.clear();
  mvBackupMapPointsId.reserve(N);
  for (int i = 0; i < N; ++i) {
    if (mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end())  // Checks if the element is not null
      mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
    else  // If the element is null his value is -1 because all the id are positives
      mvBackupMapPointsId.push_back(-1);
  }
  // Save the id of each connected KF with it weight
  mBackupConnectedKeyFrameIdWeights.clear();
  for (std::map<std::shared_ptr<KeyFrame>, int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it) {
    if (spKF.find(it->first) != spKF.end())
      mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
  }

  // Save the parent id
  mBackupParentId = -1;
  if (mpParent && spKF.find(mpParent) != spKF.end())
    mBackupParentId = mpParent->mnId;

  // Save the id of the childrens KF
  mvBackupChildrensId.clear();
  mvBackupChildrensId.reserve(mspChildrens.size());
  for (std::shared_ptr<KeyFrame> pKFi : mspChildrens) {
    if (spKF.find(pKFi) != spKF.end())
      mvBackupChildrensId.push_back(pKFi->mnId);
  }

  // Save the id of the loop edge KF
  mvBackupLoopEdgesId.clear();
  mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
  for (std::shared_ptr<KeyFrame> pKFi : mspLoopEdges) {
    if (spKF.find(pKFi) != spKF.end())
      mvBackupLoopEdgesId.push_back(pKFi->mnId);
  }

  // Camera data
  mnBackupIdCamera = -1;
  if (mpCamera && spCam.find(mpCamera) != spCam.end())
    mnBackupIdCamera = mpCamera->GetId();

  mnBackupIdCamera2 = -1;
  if (mpCamera2 && spCam.find(mpCamera2) != spCam.end())
    mnBackupIdCamera2 = mpCamera2->GetId();

  // Inertial data
  mBackupPrevKFId = -1;
  if (mPrevKF && spKF.find(mPrevKF) != spKF.end())
    mBackupPrevKFId = mPrevKF->mnId;

  mBackupNextKFId = -1;
  if (mNextKF && spKF.find(mNextKF) != spKF.end())
    mBackupNextKFId = mNextKF->mnId;

  if (mpImuPreintegrated) mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
}

void KeyFrame::PostLoad(std::map<long unsigned int, std::shared_ptr<KeyFrame>> &mpKFid,
                        std::map<long unsigned int, std::shared_ptr<MapPoint>> &mpMPid,
                        std::map<unsigned int, std::shared_ptr<const GeometricCamera>> &mpCamId) {
  // Rebuild the empty variables

  // Pose
  SetPose(mTcw);

  mTrl = mTlr.inverse();

  // Reference reconstruction
  // Each MapPoint sight from this KeyFrame
  mvpMapPoints.clear();
  mvpMapPoints.resize(N);
  for (int i = 0; i < N; ++i) {
    if (mvBackupMapPointsId[i] != -1)
      mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
    else
      mvpMapPoints[i] = nullptr;
  }

  // Conected KeyFrames with him weight
  mConnectedKeyFrameWeights.clear();
  for (std::map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end(); it != end; ++it) {
    std::shared_ptr<KeyFrame> pKFi = mpKFid[it->first];
    if (pKFi == nullptr) {
      continue;  // pKFi is not set, therefore discard this point.
    }
    mConnectedKeyFrameWeights[pKFi] = it->second;
  }

  // Restore parent KeyFrame
  if (mBackupParentId >= 0) mpParent = mpKFid[mBackupParentId];

  // KeyFrame childrens
  mspChildrens.clear();
  for (std::vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it != end; ++it) {
    mspChildrens.insert(mpKFid[*it]);
  }

  // Loop edge KeyFrame
  mspLoopEdges.clear();
  for (std::vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it) {
    mspLoopEdges.insert(mpKFid[*it]);
  }

  // Camera data
  if (mnBackupIdCamera >= 0) {
    mpCamera = mpCamId[mnBackupIdCamera];
  } else {
    std::cout << "ERROR: There is not a main camera in KF " << mnId << std::endl;
  }
  if (mnBackupIdCamera2 >= 0) {
    mpCamera2 = mpCamId[mnBackupIdCamera2];
  }

  // Inertial data
  if (mBackupPrevKFId != -1) {
    mPrevKF = mpKFid[mBackupPrevKFId];
  }
  if (mBackupNextKFId != -1) {
    mNextKF = mpKFid[mBackupNextKFId];
  }
  mpImuPreintegrated = std::make_shared<IMU::Preintegrated>(std::move(&mBackupImuPreintegrated));

  // Remove all backup container
  mvBackupMapPointsId.clear();
  mBackupConnectedKeyFrameIdWeights.clear();
  mvBackupChildrensId.clear();
  mvBackupLoopEdgesId.clear();

  UpdateBestCovisibles();
}

bool KeyFrame::ProjectPointUnDistort(std::shared_ptr<MapPoint>pMP, cv::Point2f &kp, float &u, float &v) {
  // 3D in absolute coordinates
  Eigen::Vector3f P = pMP->GetWorldPos();

  // 3D in camera coordinates
  Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
  float &PcX = Pc(0);
  float &PcY = Pc(1);
  float &PcZ = Pc(2);

  // Check positive depth
  if (PcZ < 0.0f) {
    std::cout << "Negative depth: " << PcZ << std::endl;
    return false;
  }

  // Project in image and check it is not outside
  const float invz = 1.0f / PcZ;
  u = fx * PcX * invz + cx;
  v = fy * PcY * invz + cy;

  if (u < mnMinX || u > mnMaxX) return false;
  if (v < mnMinY || v > mnMaxY) return false;

  kp = cv::Point2f(u, v);

  return true;
}

Sophus::SE3f KeyFrame::GetRelativePoseTrl() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTrl;
}

Sophus::SE3f KeyFrame::GetRelativePoseTlr() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTlr;
}

Sophus::SE3<float> KeyFrame::GetRightPose() {
  std::unique_lock<std::mutex> lock(mMutexPose);

  return mTrl * mTcw;
}

Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
  std::unique_lock<std::mutex> lock(mMutexPose);

  return mTwc * mTlr;
}

Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
  std::unique_lock<std::mutex> lock(mMutexPose);

  return (mTwc * mTlr).translation();
}

void KeyFrame::SetORBVocabulary(std::shared_ptr<ORBVocabulary> pORBVoc) { mpORBvocabulary = pORBVoc; }

void KeyFrame::SetKeyFrameDatabase(std::shared_ptr<KeyFrameDatabase> pKFDB) { mpKeyFrameDB = pKFDB; }

}  // namespace MORB_SLAM
