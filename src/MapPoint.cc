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

#include "MORB_SLAM/MapPoint.h"

#include <mutex>

#include "MORB_SLAM/ORBmatcher.h"

namespace MORB_SLAM {

long unsigned int MapPoint::nNextId = 0;
long unsigned int MapPoint::nMPsInMemory = 0;
std::mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint()
    : mnFirstKFid(0),
      mnFirstFrame(0),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr) {
  nMPsInMemory++;
}

MapPoint::MapPoint(const Eigen::Vector3f& Pos, std::shared_ptr<KeyFrame> pRefKF, std::shared_ptr<Map> pMap)
    : mnFirstKFid(pRefKF->mnId),
      mnFirstFrame(pRefKF->mnFrameId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mnOriginMapId(pMap->GetId()),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mfMinDistance(0),
      mfMaxDistance(0),
      mpMap(pMap) {
  SetWorldPos(Pos);

  mNormalVector.setZero();

  mbTrackInViewR = false;
  mbTrackInView = false;

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
  nMPsInMemory++;
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, std::shared_ptr<KeyFrame> pRefKF,
                   std::shared_ptr<KeyFrame> pHostKF, std::shared_ptr<Map> pMap)
    : mnFirstKFid(pRefKF->mnId),
      mnFirstFrame(pRefKF->mnFrameId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mnOriginMapId(pMap->GetId()),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mfMinDistance(0),
      mfMaxDistance(0),
      mpMap(pMap) {
  mInvDepth = invDepth;
  mInitU = (double)uv_init.x;
  mInitV = (double)uv_init.y;

  mNormalVector.setZero();

  // Worldpos is not set
  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
  nMPsInMemory++;
}

MapPoint::MapPoint(const Eigen::Vector3f& Pos, std::shared_ptr<Map> pMap, Frame* pFrame, const int& idxF)
    : mnFirstKFid(-1),
      mnFirstFrame(pFrame->mnId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mnOriginMapId(pMap->GetId()),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mpMap(pMap) {
  SetWorldPos(Pos);

  Eigen::Vector3f Ow;
  if (pFrame->Nleft == -1 || idxF < pFrame->Nleft) {
    Ow = pFrame->GetCameraCenter();
  } else {
    Eigen::Matrix3f Rwl = pFrame->GetRwc();
    Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
    Eigen::Vector3f twl = pFrame->GetOw();

    Ow = Rwl * tlr + twl;
  }
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector / mNormalVector.norm();

  Eigen::Vector3f PC = mWorldPos - Ow;
  const float dist = PC.norm();
  const int level = (pFrame->Nleft == -1)
                        ? pFrame->mvKeysUn[idxF].octave
                        : (idxF < pFrame->Nleft)
                              ? pFrame->mvKeys[idxF].octave
                              : pFrame->mvKeysRight[idxF].octave;
  const float levelScaleFactor = pFrame->mvScaleFactors[level];
  const int nLevels = pFrame->mnScaleLevels;

  mfMaxDistance = dist * levelScaleFactor;
  mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

  pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
  nMPsInMemory++;
}

MapPoint::~MapPoint() {
  // std::cout << "MapPoint Destroyed" << std::endl;
  nMPsInMemory--;
}

void MapPoint::SetWorldPos(const Eigen::Vector3f& Pos) {
  std::unique_lock<std::mutex> lock2(mGlobalMutex);
  std::unique_lock<std::mutex> lock(mMutexPos);
  mWorldPos = Pos;
}

Eigen::Vector3f MapPoint::GetWorldPos() {
  std::unique_lock<std::mutex> lock(mMutexPos);
  return mWorldPos;
}

Eigen::Vector3f MapPoint::GetNormal() {
  std::unique_lock<std::mutex> lock(mMutexPos);
  return mNormalVector;
}

std::weak_ptr<KeyFrame> MapPoint::GetReferenceKeyFrame() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mpRefKF;
}

void MapPoint::AddObservation(std::shared_ptr<KeyFrame> pKF, int idx) {
  std::weak_ptr<KeyFrame> wpKF = pKF;
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  std::tuple<int, int> indexes;

  if (mObservations.count(wpKF)) {
    indexes = mObservations[wpKF];
  } else {
    indexes = std::tuple<int, int>(-1, -1);
  }

  if (pKF->NLeft != -1 && idx >= pKF->NLeft) {
    std::get<1>(indexes) = idx;
  } else {
    std::get<0>(indexes) = idx;
  }

  mObservations[wpKF] = indexes;
  // pKF->mvAllMPs.push_back(shared_from_this());

  if (!pKF->mpCamera2 && pKF->mvuRight[idx] >= 0)
    nObs += 2;
  else
    nObs++;
}

void MapPoint::EraseObservation(std::shared_ptr<KeyFrame> pKF) {
  std::weak_ptr<KeyFrame> wpKF = pKF;
  bool bBad = false;
  {
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    if (mObservations.count(wpKF)) {
      std::tuple<int, int> indexes = mObservations[wpKF];
      int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

      if (leftIndex != -1) {
        if (!pKF->mpCamera2 && pKF->mvuRight[leftIndex] >= 0)
          nObs -= 2;
        else
          nObs--;
      }
      if (rightIndex != -1) {
        nObs--;
      }

      mObservations.erase(wpKF);
      if(std::shared_ptr<KeyFrame> shared_mpRefKF = mpRefKF.lock()) {
        if (shared_mpRefKF == pKF) {
          while(!mObservations.empty() && mObservations.begin()->first.expired())
            mObservations.erase(mObservations.begin());
          if(!mObservations.empty())
            mpRefKF = mObservations.begin()->first;
        }
      }

      // If only 2 observations or less, discard point
      if (nObs <= 2) bBad = true;
    }
  }

  if (bBad) SetBadFlag();
}

std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> MapPoint::GetObservations() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mObservations;
}

int MapPoint::Observations() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return nObs;
}

void MapPoint::SetBadFlag() {
  std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> obs;
  {
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    mbBad = true;
    obs = mObservations;
    mObservations.clear();
    // mpRefKF = nullptr;
    mpReplaced = nullptr;
  }
  for (auto &pair : obs) {
    if(std::shared_ptr<KeyFrame> pKF = pair.first.lock()) {
      int leftIndex = std::get<0>(pair.second), rightIndex = std::get<1>(pair.second);
      if (leftIndex != -1) {
        pKF->EraseMapPointMatch(leftIndex);
      }
      if (rightIndex != -1) {
        pKF->EraseMapPointMatch(rightIndex);
      }
    }
  }

  mpMap->EraseMapPoint(shared_from_this());
}

std::shared_ptr<MapPoint> MapPoint::GetReplaced() {
  std::unique_lock<std::mutex> lock1(mMutexFeatures);
  std::unique_lock<std::mutex> lock2(mMutexPos);
  return mpReplaced;
}

void MapPoint::Replace(std::shared_ptr<MapPoint> pMP) {
  std::shared_ptr<MapPoint> self = shared_from_this();
  if (pMP->mnId == self->mnId) return;

  int nvisible, nfound;
  std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> obs;
  {
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    obs = mObservations;
    mObservations.clear();
    mbBad = true;
    nvisible = mnVisible;
    nfound = mnFound;
    mpReplaced = pMP;
  }

  for (auto &pair : obs) {
    // Replace measurement in keyframe
    if(std::shared_ptr<KeyFrame> pKF = pair.first.lock()) {
      std::tuple<int, int> indexes = pair.second;
      int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

      if (!pMP->IsInKeyFrame(pKF)) {
        if (leftIndex != -1) {
          pKF->ReplaceMapPointMatch(leftIndex, pMP);
          pMP->AddObservation(pKF, leftIndex);
        }
        if (rightIndex != -1) {
          pKF->ReplaceMapPointMatch(rightIndex, pMP);
          pMP->AddObservation(pKF, rightIndex);
        }
      } else {
        if (leftIndex != -1) {
          pKF->EraseMapPointMatch(leftIndex);
        }
        if (rightIndex != -1) {
          pKF->EraseMapPointMatch(rightIndex);
        }
      }
    }
  }
  pMP->IncreaseFound(nfound);
  pMP->IncreaseVisible(nvisible);
  pMP->ComputeDistinctiveDescriptors();

  mpMap->EraseMapPoint(self);
}

bool MapPoint::isBad() {
  std::unique_lock<std::mutex> lock1(mMutexFeatures, std::defer_lock);
  std::unique_lock<std::mutex> lock2(mMutexPos, std::defer_lock);
  lock(lock1, lock2);

  return mbBad;
}

void MapPoint::IncreaseVisible(int n) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  mnVisible += n;
}

void MapPoint::IncreaseFound(int n) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  mnFound += n;
}

float MapPoint::GetFoundRatio() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors() {
  // Retrieve all observed descriptors
  std::vector<cv::Mat> vDescriptors;

  std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations;

  {
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    if (mbBad) return;
    observations = mObservations;
  }

  if (observations.empty()) return;

  vDescriptors.reserve(observations.size());

  for (auto &pair : observations) {
    if(std::shared_ptr<KeyFrame> pKF = pair.first.lock()) {
      if (!pKF->isBad()) {
        if (std::get<0>(pair.second) != -1)
          vDescriptors.push_back(pKF->mDescriptors.row(std::get<0>(pair.second)));
        if (std::get<1>(pair.second) != -1)
          vDescriptors.push_back(pKF->mDescriptors.row(std::get<1>(pair.second)));
      }
    }
  }

  if (vDescriptors.empty()) return;

  // Compute distances between them
  const size_t N = vDescriptors.size();

  int Distances[N][N];
  for (size_t i = 0; i < N; i++) {
    Distances[i][i] = 0;
    for (size_t j = i + 1; j < N; j++) {
      int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
      Distances[i][j] = distij;
      Distances[j][i] = distij;
    }
  }

  // Take the descriptor with least median distance to the rest
  int BestMedian = INT_MAX;
  int BestIdx = 0;
  for (size_t i = 0; i < N; i++) {
    std::vector<int> vDists(Distances[i], Distances[i] + N);
    sort(vDists.begin(), vDists.end());
    int median = vDists[0.5 * (N - 1)];

    if (median < BestMedian) {
      BestMedian = median;
      BestIdx = i;
    }
  }

  {
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mDescriptor = vDescriptors[BestIdx].clone();
  }
}

cv::Mat MapPoint::GetDescriptor() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mDescriptor.clone();
}

std::tuple<int, int> MapPoint::GetIndexInKeyFrame(std::shared_ptr<KeyFrame> pKF) {
  std::weak_ptr<KeyFrame> wpKF = pKF;
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  
  if (mObservations.count(wpKF))
    return mObservations[wpKF];
  else
    return std::tuple<int, int>(-1, -1);
}

bool MapPoint::IsInKeyFrame(std::shared_ptr<KeyFrame> pKF) {
  std::weak_ptr<KeyFrame> wpKF = pKF;
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return (mObservations.count(wpKF));
}

void MapPoint::UpdateNormalAndDepth() {
  std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations;
  std::shared_ptr<KeyFrame> pRefKF;
  Eigen::Vector3f Pos;
  {
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    if (mbBad) return;
    observations = mObservations;
    if(!(pRefKF = mpRefKF.lock())) {
      mpRefKF = observations.begin()->first;
      return;
    }
    Pos = mWorldPos;
  }

  if (observations.empty()) return;

  Eigen::Vector3f normal;
  normal.setZero();
  for (auto &pair : observations) {
    if(std::shared_ptr<KeyFrame> pKF = pair.first.lock()) {
      std::tuple<int, int> indexes = pair.second;
      int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

      if (leftIndex != -1) {
        Eigen::Vector3f Owi = pKF->GetCameraCenter();
        Eigen::Vector3f normali = Pos - Owi;
        normal += normali / normali.norm();
      }
      if (rightIndex != -1) {
        Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
        Eigen::Vector3f normali = Pos - Owi;
        normal += normali / normali.norm();
      }
    }
  }

  Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
  const float dist = PC.norm();

  std::tuple<int, int> indexes = observations[mpRefKF];
  int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
  int level;
  if (pRefKF->NLeft == -1) {
    level = pRefKF->mvKeysUn[leftIndex].octave;
  } else if (leftIndex != -1) {
    level = pRefKF->mvKeys[leftIndex].octave;
  } else {
    level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
  }

  // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
  const float levelScaleFactor = pRefKF->mvScaleFactors[level];
  const int nLevels = pRefKF->mnScaleLevels;

  {
    std::unique_lock<std::mutex> lock3(mMutexPos);
    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
    mNormalVector = normal / normal.norm();
  }
}

void MapPoint::SetNormalVector(const Eigen::Vector3f& normal) {
  std::unique_lock<std::mutex> lock3(mMutexPos);
  mNormalVector = normal;
}

float MapPoint::GetMinDistanceInvariance() {
  std::unique_lock<std::mutex> lock(mMutexPos);
  return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance() {
  std::unique_lock<std::mutex> lock(mMutexPos);
  return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float& currentDist, std::shared_ptr<KeyFrame> pKF) {
  float ratio;
  {
    std::unique_lock<std::mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pKF->mnScaleLevels)
    nScale = pKF->mnScaleLevels - 1;

  return nScale;
}

int MapPoint::PredictScale(const float& currentDist, Frame* pF) {
  float ratio;
  {
    std::unique_lock<std::mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pF->mnScaleLevels)
    nScale = pF->mnScaleLevels - 1;

  return nScale;
}

void MapPoint::PrintObservations() {
  std::cout << "MP_OBS: MP " << mnId << std::endl;
  for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++) {
    if(std::shared_ptr<KeyFrame> pKFi = (mit->first).lock()) {
      // std::tuple<int,int> indexes = mit->second; // UNUSED
      // int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes); // UNUSED
      std::cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << std::endl;
    }
  }
}

std::shared_ptr<Map> MapPoint::GetMap() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mpMap;
}

void MapPoint::UpdateMap(std::shared_ptr<Map> pMap) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mpMap = pMap;
}

void MapPoint::PreSave(std::set<std::shared_ptr<KeyFrame>>& spKF, std::set<std::shared_ptr<MapPoint>>& spMP) {
  mBackupReplacedId = -1;

  if (mpReplaced && spMP.find(mpReplaced) != spMP.end())
    mBackupReplacedId = mpReplaced->mnId;

  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();

  std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> tmp_mObservations;
  tmp_mObservations.insert(mObservations.begin(), mObservations.end());

  // Save the id and position in each KF who view it
  for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::const_iterator it = tmp_mObservations.begin(), end = tmp_mObservations.end(); it != end; ++it) {
    if(std::shared_ptr<KeyFrame> pKFi = (it->first).lock()) {
      if (spKF.find(pKFi) != spKF.end()) {
        mBackupObservationsId1[pKFi->mnId] = std::get<0>(it->second);
        mBackupObservationsId2[pKFi->mnId] = std::get<1>(it->second);
      } else {
        EraseObservation(pKFi);  // iterate -- afterwards to pull back once
        //   it--;
      }
    }
  }

  if(std::shared_ptr<KeyFrame> shared_mpRefKF = mpRefKF.lock()) {
    // Save the id of the reference KF
    if (spKF.find(shared_mpRefKF) != spKF.end()) {
      mBackupRefKFId = shared_mpRefKF->mnId;
    }   
  }

}

void MapPoint::PostLoad(std::map<long unsigned int, std::shared_ptr<KeyFrame>>& mpKFid,
                        std::map<long unsigned int, std::shared_ptr<MapPoint>>& mpMPid) {
  mpRefKF = mpKFid[mBackupRefKFId];
   
  if (!mpRefKF.lock()) {
    std::cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << std::endl;
  }
  mpReplaced = nullptr;
  if (mBackupReplacedId >= 0) {
    std::map<long unsigned int, std::shared_ptr<MapPoint>>::iterator it =
        mpMPid.find(mBackupReplacedId);
    if (it != mpMPid.end()) mpReplaced = it->second;
  }

  mObservations.clear();

  for (std::map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it) {
    std::weak_ptr<KeyFrame> pKFi = mpKFid[it->first];
    std::map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
    std::tuple<int, int> indexes = std::tuple<int, int>(it->second, it2->second);
    if (pKFi.lock()) {
      mObservations[pKFi] = indexes;
    }
  }

  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();
}

}  // namespace MORB_SLAM
