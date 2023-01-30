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

#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/KeyFrameDatabase.h"
#include "MORB_SLAM/LoopClosing.h"
#include "MORB_SLAM/Settings.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping {
 public:
  
  LocalMapping(System* pSys, const Atlas_ptr &pAtlas, const float bMonocular,
               bool bInertial, const string& _strSeqName = std::string());

  void SetLoopCloser(LoopClosing* pLoopCloser);

  void SetTracker(Tracking* pTracker);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFrame* pKF);
  void EmptyQueue();

  // Thread Synch
  void RequestStop();
  void RequestReset();
  void RequestResetActiveMap(std::shared_ptr<Map> pMap);
  bool Stop();
  void Release();
  bool isStopped();
  bool stopRequested();
  bool AcceptKeyFrames();
  void SetAcceptKeyFrames(bool flag);
  bool SetNotStop(bool flag);

  void InterruptBA();

  void RequestFinish();
  bool isFinished();

  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }

  inline bool getIsDoneVIBA() { return isDoneVIBA; }
  inline void setIsDoneVIBA(bool viba) { isDoneVIBA = viba; }

  bool IsInitializing();
  double GetCurrKFTime();
  KeyFrame* GetCurrKF();

  std::mutex mMutexImuInit;

  Eigen::MatrixXd mcovInertial;
  Eigen::Matrix3d mRwg;
  Eigen::Vector3d mbg;
  Eigen::Vector3d mba;
  double mScale;
  double mInitTime;
  double mCostTime;

  unsigned int mInitSect;
  unsigned int mIdxInit;
  unsigned int mnKFs;
  double mFirstTs;
  int mnMatchesInliers;

  // For debugging (erase in normal mode)
  int mIdxIteration;
  string strSequence;

  bool mbNotBA1;
  bool mbNotBA2;
  bool mbBadImu;

  bool mbWriteStats;

  // not consider far points (clouds)
  bool mbFarPoints;
  float mThFarPoints;

#ifdef REGISTER_TIMES
  vector<double> vdKFInsert_ms;
  vector<double> vdMPCulling_ms;
  vector<double> vdMPCreation_ms;
  vector<double> vdLBA_ms;
  vector<double> vdKFCulling_ms;
  vector<double> vdLMTotal_ms;

  vector<double> vdLBASync_ms;
  vector<double> vdKFCullingSync_ms;
  vector<int> vnLBA_edges;
  vector<int> vnLBA_KFopt;
  vector<int> vnLBA_KFfixed;
  vector<int> vnLBA_MPs;
  int nLBA_exec;
  int nLBA_abort;
#endif
 protected:
  bool CheckNewKeyFrames();
  void ProcessNewKeyFrame();
  void CreateNewMapPoints();

  void MapPointCulling();
  void SearchInNeighbors();
  void KeyFrameCulling();

  System* mpSystem;

  bool mbMonocular;
  bool mbInertial;

  void ResetIfRequested();
  bool mbResetRequested;
  bool mbResetRequestedActiveMap;
  std::shared_ptr<Map> mpMapToReset;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Atlas_ptr mpAtlas;

  LoopClosing* mpLoopCloser;
  Tracking* mpTracker;

  std::list<KeyFrame*> mlNewKeyFrames;

  KeyFrame* mpCurrentKeyFrame;

  std::list<MapPoint*> mlpRecentAddedMapPoints;

  std::mutex mMutexNewKFs;

  bool mbAbortBA;

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;

  bool mbAcceptKeyFrames;
  std::mutex mMutexAccept;

  void InitializeIMU(ImuInitializater::ImuInitType priorG = ImuInitializater::ImuInitType::DEFAULT_G, 
                    ImuInitializater::ImuInitType priorA = ImuInitializater::ImuInitType::DEFAULT_A,
                    bool bFirst = false);
  void ScaleRefinement();

  bool bInitializing;

  Eigen::MatrixXd infoInertial;
  int mNumLM;
  int mNumKFCulling;

  float mTinit;

  int countRefinement;

  bool isDoneVIBA;

  // DEBUG
  ofstream f_lm;
};

}  // namespace MORB_SLAM

