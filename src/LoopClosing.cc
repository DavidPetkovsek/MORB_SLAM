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

#include "MORB_SLAM/LoopClosing.h"

#include <mutex>
#include <thread>

#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/G2oTypes.h"
#include "MORB_SLAM/ORBmatcher.h"
#include "MORB_SLAM/Optimizer.h"
#include "MORB_SLAM/Sim3Solver.h"

namespace MORB_SLAM {

LoopClosing::LoopClosing(const Atlas_ptr &pAtlas, std::shared_ptr<KeyFrameDatabase> pDB, std::shared_ptr<ORBVocabulary> pVoc, const bool bFixScale, const bool bActiveLC, bool bInertial)
    : hasMergedLocalMap(false),
      mbResetRequested(false),
      mbResetActiveMapRequested(false),
      mbFinishRequested(false),
      mpAtlas(pAtlas),
      mpKeyFrameDB(pDB),
      mpLastCurrentKF(nullptr),
      mbLoopDetected(false),
      mnLoopNumCoincidences(0),
      mnLoopNumNotFound(0),
      mbMergeDetected(false),
      mnMergeNumCoincidences(0),
      mnMergeNumNotFound(0), 
      mbRunningGBA(false),
      mbStopGBA(false),
      mbFixScale(bFixScale),
      mnFullBAIdx(0),
      mbActiveLC(bActiveLC),
      mbInertial(bInertial) {}

void LoopClosing::SetTracker(Tracking_ptr pTracker) { mpTracker = pTracker; }

void LoopClosing::SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper) { mpLocalMapper = pLocalMapper; }

void LoopClosing::Run() {
  while (1) {

    // NEW LOOP AND MERGE DETECTION ALGORITHM
    //----------------------------

    if (CheckNewKeyFrames()) {
      bool bFindedRegion = NewDetectCommonRegions();
      if (bFindedRegion) {
        if (mbMergeDetected) {
          if (mbInertial && (!mpCurrentKF->GetMap()->isImuInitialized())) {
            std::cout << "IMU is not initilized, merge is aborted" << std::endl;
          } else {
            Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
            g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
            Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
            g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
            g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();

            mSold_new = (gSw2c * gScw1);

            if (mbInertial) {
              std::cout << "Merge check transformation with IMU" << std::endl;
              if (mSold_new.scale() < 0.90 || mSold_new.scale() > 1.1) {
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
                mbMergeDetected = false;
                Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                continue;
              }
              // If inertial, force only yaw
              if ((mpTracker->mSensor == CameraType::IMU_MONOCULAR || mpTracker->mSensor == CameraType::IMU_STEREO || mpTracker->mSensor == CameraType::IMU_RGBD) && mpCurrentKF->GetMap()->GetInertialBA1()) {
                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                phi(0) = 0;
                phi(1) = 0;
                mSold_new = g2o::Sim3(ExpSO3(phi), mSold_new.translation(), 1.0);
              }
            }

            mg2oMergeScw = mg2oMergeSlw;

            Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

            mpLocalMapper->setIsDoneVIBA(false);
            mpTracker->mLockPreTeleportTranslation = true;
            // TODO UNCOMMENT
            if (mpTracker->mSensor == CameraType::IMU_MONOCULAR || mpTracker->mSensor == CameraType::IMU_STEREO || mpTracker->mSensor == CameraType::IMU_RGBD)
              MergeLocal2();
            else
              MergeLocal();

            mpTracker->mTeleported = true;
            Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
          }

          // Reset all variables
          mpMergeLastCurrentKF->SetErase();
          mpMergeMatchedKF->SetErase();
          mnMergeNumCoincidences = 0;
          mvpMergeMatchedMPs.clear();
          mvpMergeMPs.clear();
          mnMergeNumNotFound = 0;
          mbMergeDetected = false;

          if (mbLoopDetected) {
            // Reset Loop variables
            mpLoopLastCurrentKF->SetErase();
            mpLoopMatchedKF->SetErase();
            mnLoopNumCoincidences = 0;
            mvpLoopMatchedMPs.clear();
            mvpLoopMPs.clear();
            mnLoopNumNotFound = 0;
            mbLoopDetected = false;
          }
        }

        if (mbLoopDetected) {
          bool bGoodLoop = true;
          Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);

          mg2oLoopScw = mg2oLoopSlw;
          if (mbInertial) {
            Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
            g2o::Sim3 g2oTwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
            g2o::Sim3 g2oSww_new = g2oTwc * mg2oLoopScw;

            Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
            std::cout << "phi = " << phi.transpose() << std::endl;
              // OG SLAM conditions
            // if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f && fabs(phi(2)) < 0.349f) {
              // Less Strict conditions
            if (fabs(phi(0)) < 0.032f && fabs(phi(1)) < 0.032f) {
              // If inertial, force only yaw (pitch+roll are aligned by gravity, forcing them to change would cause SLAM to fly off)
              if (mpCurrentKF->GetMap()->GetInertialBA2()) {
                phi(0) = 0;
                phi(1) = 0;
                g2oSww_new = g2o::Sim3(ExpSO3(phi), g2oSww_new.translation(), 1.0);
                mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
              }

            } else {
              std::cout << "BAD LOOP!!!" << std::endl;
              bGoodLoop = false;
            }
          }

          if (bGoodLoop) {
            mvpLoopMapPoints = mvpLoopMPs;
            mpLocalMapper->setIsDoneVIBA(false);
            mpTracker->mLockPreTeleportTranslation = true;
            CorrectLoop();
            mpTracker->mTeleported = true;
            std::cout << "Loop Closed Successfully" << std::endl;
          }

          // Reset all variables
          mpLoopLastCurrentKF->SetErase();
          mpLoopMatchedKF->SetErase();
          mnLoopNumCoincidences = 0;
          mvpLoopMatchedMPs.clear();
          mvpLoopMPs.clear();
          mnLoopNumNotFound = 0;
          mbLoopDetected = false;
        }
      }
      mpLastCurrentKF = mpCurrentKF;
    }

    ResetIfRequested();

    if (CheckFinish()) {
      break;
    }

    usleep(5000);
  }
}

void LoopClosing::InsertKeyFrame(std::shared_ptr<KeyFrame> pKF) {
  std::unique_lock<std::mutex> lock(mMutexLoopQueue);
  if (pKF->mnId != 0) mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexLoopQueue);
  return (!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::NewDetectCommonRegions() {
  // To deactivate placerecognition. No loopclosing nor merging will be performed
  if (!mbActiveLC) return false;

  {
    std::unique_lock<std::mutex> lock(mMutexLoopQueue);
    mpCurrentKF = mlpLoopKeyFrameQueue.front();
    mlpLoopKeyFrameQueue.pop_front();
    // Avoid that a keyframe can be erased while it is being process by this thread
    mpCurrentKF->SetNotErase();

    mpLastMap = mpCurrentKF->GetMap();
  }

  if (mbInertial && !mpLastMap->GetInertialBA2()) {
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  if (mpTracker->mSensor == CameraType::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) { // 12
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  if (mpLastMap->GetAllKeyFrames().size() < 12) {
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  // Check the last candidates with geometric validation
  // Loop candidates
  bool bLoopDetectedInKF = false;

  if (mnLoopNumCoincidences > 0) {
    // Find from the last KF candidates
    Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
    g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
    g2o::Sim3 gScw = gScl * mg2oLoopSlw;
    int numProjMatches = 0;
    std::vector<std::shared_ptr<MapPoint>> vpMatchedMPs;
    bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs);
    if (bCommonRegion) {
      bLoopDetectedInKF = true;

      mnLoopNumCoincidences++;
      mpLoopLastCurrentKF->SetErase();
      mpLoopLastCurrentKF = mpCurrentKF;
      mg2oLoopSlw = gScw;
      mvpLoopMatchedMPs = vpMatchedMPs;

      mbLoopDetected = mnLoopNumCoincidences >= 3;
      mnLoopNumNotFound = 0;

      if (!mbLoopDetected) {
        std::cout << "PR: Loop detected with Reffine Sim3" << std::endl;
      }
    } else {
      bLoopDetectedInKF = false;

      mnLoopNumNotFound++;
      if (mnLoopNumNotFound >= 2) {
        mpLoopLastCurrentKF->SetErase();
        mpLoopMatchedKF->SetErase();
        mnLoopNumCoincidences = 0;
        mvpLoopMatchedMPs.clear();
        mvpLoopMPs.clear();
        mnLoopNumNotFound = 0;
      }
    }
  }

  // Merge candidates
  bool bMergeDetectedInKF = false;
  if (mnMergeNumCoincidences > 0) {
    // Find from the last KF candidates
    Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();

    g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
    g2o::Sim3 gScw = gScl * mg2oMergeSlw;
    int numProjMatches = 0;
    std::vector<std::shared_ptr<MapPoint>> vpMatchedMPs;
    bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);
    if (bCommonRegion) {
      bMergeDetectedInKF = true;

      mnMergeNumCoincidences++;
      mpMergeLastCurrentKF->SetErase();
      mpMergeLastCurrentKF = mpCurrentKF;
      mg2oMergeSlw = gScw;
      mvpMergeMatchedMPs = vpMatchedMPs;

      mbMergeDetected = mnMergeNumCoincidences >= 3;
    } else {
      mbMergeDetected = false;
      bMergeDetectedInKF = false;

      mnMergeNumNotFound++;
      if (mnMergeNumNotFound >= 2) {
        mpMergeLastCurrentKF->SetErase();
        mpMergeMatchedKF->SetErase();
        mnMergeNumCoincidences = 0;
        mvpMergeMatchedMPs.clear();
        mvpMergeMPs.clear();
        mnMergeNumNotFound = 0;
      }
    }
  }

  if (mbMergeDetected || mbLoopDetected) {
    mpKeyFrameDB->add(mpCurrentKF);
    return true;
  }

  // TODO: This is only necessary if we use a minimun score for pick the best candidates
  // UNUSED
  // const std::vector<std::shared_ptr<KeyFrame>> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

  // Extract candidates from the bag of words
  std::vector<std::shared_ptr<KeyFrame>> vpMergeBowCand, vpLoopBowCand;
  if (!bMergeDetectedInKF || !bLoopDetectedInKF) {
    // Search in BoW
    mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 3);
  }

  // Check the BoW candidates if the geometric candidate list is empty
  // Loop candidates
  if (!bLoopDetectedInKF && !vpLoopBowCand.empty()) {
    mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
  }
  // Merge candidates
  if (!bMergeDetectedInKF && !vpMergeBowCand.empty()) {
    mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
  }

  mpKeyFrameDB->add(mpCurrentKF);

  if (mbMergeDetected || mbLoopDetected) {
    return true;
  }

  mpCurrentKF->SetErase();

  return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKF, g2o::Sim3& gScw, int& nNumProjMatches,
                                                std::vector<std::shared_ptr<MapPoint>>& vpMPs, std::vector<std::shared_ptr<MapPoint>>& vpMatchedMPs) {
  std::set<std::shared_ptr<MapPoint>> spAlreadyMatchedMPs;
  nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

  int nProjMatches = 30;
  int nProjOptMatches = 50;
  int nProjMatchesRep = 100;

  if (nNumProjMatches >= nProjMatches) {
    Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
    g2o::Sim3 gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
    g2o::Sim3 gScm = gScw * gSwm;
    Eigen::Matrix<double, 7, 7> mHessian7x7;

    bool bFixedScale = mbFixScale;  // TODO CHECK; Solo para el monocular inertial
    if (mpTracker->mSensor == CameraType::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetInertialBA2())
      bFixedScale = false;
    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);

    if (numOptMatches > nProjOptMatches) {
      g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

      std::vector<std::shared_ptr<MapPoint>> vpMatchedMP;
      vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), nullptr);

      nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
      if (nNumProjMatches >= nProjMatchesRep) {
        gScw = gScw_estimation;
        return true;
      }
    }
  }
  return false;
}

bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<std::shared_ptr<KeyFrame>>& vpBowCand, std::shared_ptr<KeyFrame>& pMatchedKF2, std::shared_ptr<KeyFrame>& pLastCurrentKF, g2o::Sim3& g2oScw,
                                            int& nNumCoincidences, std::vector<std::shared_ptr<MapPoint>>& vpMPs, std::vector<std::shared_ptr<MapPoint>>& vpMatchedMPs) {
  int nBoWMatches = 20; // lower this and try again
  int nBoWInliers = 15;
  int nSim3Inliers = 20;
  int nProjMatches = 50;
  int nProjOptMatches = 100; //80;

  std::set<std::shared_ptr<KeyFrame>> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

  int nNumCovisibles = 10;

  ORBmatcher matcherBoW(0.8, true);
  ORBmatcher matcher(0.75, true);

  // Varibles to select the best numbe
  std::shared_ptr<KeyFrame> pBestMatchedKF;
  int nBestMatchesReproj = 0;
  int nBestNumCoindicendes = 0;
  g2o::Sim3 g2oBestScw;
  std::vector<std::shared_ptr<MapPoint>> vpBestMapPoints;
  std::vector<std::shared_ptr<MapPoint>> vpBestMatchedMapPoints;

  int numCandidates = vpBowCand.size();
  std::vector<int> vnStage(numCandidates, 0);
  std::vector<int> vnMatchesStage(numCandidates, 0);

  int index = 0;
  for (std::shared_ptr<KeyFrame> pKFi : vpBowCand) {
    if (!pKFi || pKFi->isBad()) continue;

    // Current KF against KF with covisibles version
    std::vector<std::shared_ptr<KeyFrame>> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
    if (vpCovKFi.empty()) {
      vpCovKFi.push_back(pKFi);
    } else {
      vpCovKFi.push_back(vpCovKFi[0]);
      vpCovKFi[0] = pKFi;
    }

    bool bAbortByNearKF = false;
    for (size_t j = 0; j < vpCovKFi.size(); ++j) {
      if (spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end()) {
        bAbortByNearKF = true;
        break;
      }
    }
    if (bAbortByNearKF) {
      continue;
    }

    std::vector<std::vector<std::shared_ptr<MapPoint>>> vvpMatchedMPs;
    vvpMatchedMPs.resize(vpCovKFi.size());
    std::set<std::shared_ptr<MapPoint>> spMatchedMPi;
    int numBoWMatches = 0;

    std::shared_ptr<KeyFrame> pMostBoWMatchesKF = pKFi;

    std::vector<std::shared_ptr<MapPoint>> vpMatchedPoints = std::vector<std::shared_ptr<MapPoint>>(mpCurrentKF->GetMapPointMatches().size(), nullptr);
    std::vector<std::shared_ptr<KeyFrame>> vpKeyFrameMatchedMP = std::vector<std::shared_ptr<KeyFrame>>(mpCurrentKF->GetMapPointMatches().size(), nullptr);

    for (size_t j = 0; j < vpCovKFi.size(); ++j) {
      if (!vpCovKFi[j] || vpCovKFi[j]->isBad()) continue;

      matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]); 
    }

    for (size_t j = 0; j < vpCovKFi.size(); ++j) {
      for (size_t k = 0; k < vvpMatchedMPs[j].size(); ++k) {
        std::shared_ptr<MapPoint> pMPi_j = vvpMatchedMPs[j][k];
        if (!pMPi_j || pMPi_j->isBad()) continue;

        if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end()) {
          spMatchedMPi.insert(pMPi_j);
          numBoWMatches++;

          vpMatchedPoints[k] = pMPi_j;
          vpKeyFrameMatchedMP[k] = vpCovKFi[j];
        }
      }
    }

    if (numBoWMatches >= nBoWMatches) { // TODO pick a good threshold
      // Geometric validation

      // Scale is not fixed if the cam is IMU_MONO and the IMU's unititialized
      bool bFixedScale = mbFixScale && !(mpTracker->mSensor == CameraType::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetInertialBA2());

      Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
      solver.SetRansacParameters(0.99, nBoWInliers, 300);  // at least 15 inliers

      bool bNoMore = false;
      std::vector<bool> vbInliers;
      int nInliers;
      bool bConverge = false;
      Eigen::Matrix4f mTcm;
      while (!bConverge && !bNoMore) {
        mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
      }

      if (bConverge) {
        // Match by reprojection
        vpCovKFi.clear();
        vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
        vpCovKFi.push_back(pMostBoWMatchesKF);
        std::set<std::shared_ptr<KeyFrame>> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

        std::set<std::shared_ptr<MapPoint>> spMapPoints;
        std::vector<std::shared_ptr<MapPoint>> vpMapPoints;
        std::vector<std::shared_ptr<KeyFrame>> vpKeyFrames;
        for (std::shared_ptr<KeyFrame> pCovKFi : vpCovKFi) {
          for (std::shared_ptr<MapPoint> pCovMPij : pCovKFi->GetMapPointMatches()) {
            if (!pCovMPij || pCovMPij->isBad()) continue;

            if (spMapPoints.find(pCovMPij) == spMapPoints.end()) {
              spMapPoints.insert(pCovMPij);
              vpMapPoints.push_back(pCovMPij);
              vpKeyFrames.push_back(pCovKFi);
            }
          }
        }

        g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(), solver.GetEstimatedTranslation().cast<double>(), (double)solver.GetEstimatedScale());
        g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(), pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
        g2o::Sim3 gScw = gScm * gSmw;  // Similarity matrix of current from the world position
        Sophus::Sim3f mScw = Converter::toSophus(gScw);

        std::vector<std::shared_ptr<MapPoint>> vpMatchedMP;
        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), nullptr);
        std::vector<std::shared_ptr<KeyFrame>> vpMatchedKF;
        vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), nullptr);
        int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);

        if (numProjMatches >= nProjMatches) {
          // Optimize Sim3 transformation with every matches
          Eigen::Matrix<double, 7, 7> mHessian7x7;

          int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

          if (numOptMatches >= nSim3Inliers) {
            g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(), pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
            g2o::Sim3 gScw = gScm * gSmw;  // Similarity matrix of current from the world position
            Sophus::Sim3f mScw = Converter::toSophus(gScw);

            std::vector<std::shared_ptr<MapPoint>> vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), nullptr);
            int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

            if (numProjOptMatches >= nProjOptMatches) {
              int max_x = -1, min_x = 1000000;
              int max_y = -1, min_y = 1000000;
              for (std::shared_ptr<MapPoint> pMPi : vpMatchedMP) {
                if (!pMPi || pMPi->isBad()) {
                  continue;
                }

                std::tuple<size_t, size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                int index = std::get<0>(indexes);
                if (index >= 0) {
                  int coord_x = pKFi->mvKeysUn[index].pt.x;
                  if (coord_x < min_x) {
                    min_x = coord_x;
                  }
                  if (coord_x > max_x) {
                    max_x = coord_x;
                  }
                  int coord_y = pKFi->mvKeysUn[index].pt.y;
                  if (coord_y < min_y) {
                    min_y = coord_y;
                  }
                  if (coord_y > max_y) {
                    max_y = coord_y;
                  }
                }
              }

              int nNumKFs = 0;
              // Check the Sim3 transformation with the current KeyFrame covisibles
              std::vector<std::shared_ptr<KeyFrame>> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

              for (size_t j = 0; nNumKFs < 3 && j < vpCurrentCovKFs.size(); ++j) {
                std::shared_ptr<KeyFrame> pKFj = vpCurrentCovKFs[j];
                Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                g2o::Sim3 gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                g2o::Sim3 gSjw = gSjc * gScw;
                int numProjMatches_j = 0;
                std::vector<std::shared_ptr<MapPoint>> vpMatchedMPs_j;
                bool bValid = DetectCommonRegionsFromLastKF(pKFj, pMostBoWMatchesKF, gSjw, numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                if (bValid) {
                  nNumKFs++;
                }
              }

              if (nNumKFs < 3) {
                vnStage[index] = 8;
                vnMatchesStage[index] = nNumKFs;
              }

              if (nBestMatchesReproj < numProjOptMatches) {
                nBestMatchesReproj = numProjOptMatches;
                nBestNumCoindicendes = nNumKFs;
                pBestMatchedKF = pMostBoWMatchesKF;
                g2oBestScw = gScw;
                vpBestMapPoints = vpMapPoints;
                vpBestMatchedMapPoints = vpMatchedMP;
              }
            }
          }
        }
      }
    }
    index++;
  }

  if (nBestMatchesReproj > 0) {
    pLastCurrentKF = mpCurrentKF;
    nNumCoincidences = nBestNumCoindicendes;
    pMatchedKF2 = pBestMatchedKF;
    pMatchedKF2->SetNotErase();
    g2oScw = g2oBestScw;
    vpMPs = vpBestMapPoints;
    vpMatchedMPs = vpBestMatchedMapPoints;

    return nNumCoincidences >= 3;
  }
  return false;
}

bool LoopClosing::DetectCommonRegionsFromLastKF(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKF, g2o::Sim3& gScw, int& nNumProjMatches, std::vector<std::shared_ptr<MapPoint>>& vpMPs, std::vector<std::shared_ptr<MapPoint>>& vpMatchedMPs) {
  std::set<std::shared_ptr<MapPoint>> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
  nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

  return (nNumProjMatches >= 30);
}

int LoopClosing::FindMatchesByProjection(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKFw, g2o::Sim3& g2oScw,
    std::set<std::shared_ptr<MapPoint>>& spMatchedMPinOrigin, std::vector<std::shared_ptr<MapPoint>>& vpMapPoints, std::vector<std::shared_ptr<MapPoint>>& vpMatchedMapPoints) {
  int nNumCovisibles = 10;
  std::vector<std::shared_ptr<KeyFrame>> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
  int nInitialCov = vpCovKFm.size();
  vpCovKFm.push_back(pMatchedKFw);
  std::set<std::shared_ptr<KeyFrame>> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
  std::set<std::shared_ptr<KeyFrame>> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();

  if (nInitialCov < nNumCovisibles) {
    for (int i = 0; i < nInitialCov; ++i) {
      std::vector<std::shared_ptr<KeyFrame>> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
      int nInserted = 0;
      for (size_t j = 0; j < vpKFs.size() && nInserted < nNumCovisibles; ++j) {
        if (spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end()) {
          spCheckKFs.insert(vpKFs[j]);
          ++nInserted;
        }
      }
      vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
    }
  }
  std::set<std::shared_ptr<MapPoint>> spMapPoints;
  vpMapPoints.clear();
  vpMatchedMapPoints.clear();

  for (std::shared_ptr<KeyFrame> pKFi : vpCovKFm) {
    for (std::shared_ptr<MapPoint> pMPij : pKFi->GetMapPointMatches()) {
      if (!pMPij || pMPij->isBad()) continue;

      if (spMapPoints.find(pMPij) == spMapPoints.end()) {
        spMapPoints.insert(pMPij);
        vpMapPoints.push_back(pMPij);
      }
    }
  }

  Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
  ORBmatcher matcher(0.9, true);

  vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), nullptr);
  int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

  return num_matches;
}

void LoopClosing::CorrectLoop() {
  // Send a stop signal to Local Mapping
  // Avoid new keyframes are inserted while correcting the loop
  mpLocalMapper->RequestStop();
  mpLocalMapper->EmptyQueue();  // Proccess keyframes in the queue

  // If a Global Bundle Adjustment is running, abort it
  if (isRunningGBA()) {
    std::cout << "Stoping Global Bundle Adjustment...";
    std::unique_lock<std::mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;
    std::cout << "  Done!!" << std::endl;
  }

  // Wait until Local Mapping has effectively stopped
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  // Ensure current keyframe is updated
  // assert(mpCurrentKF->GetMap()->CheckEssentialGraph());
  mpCurrentKF->UpdateConnections();
  // assert(mpCurrentKF->GetMap()->CheckEssentialGraph());

  // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
  std::vector<std::shared_ptr<KeyFrame>> vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
  vpCurrentConnectedKFs.push_back(mpCurrentKF);

  KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
  CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
  Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
  Sophus::SE3f Tcw = mpCurrentKF->GetPose();
  g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>(), 1.0);
  NonCorrectedSim3[mpCurrentKF] = g2oScw;

  // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
  Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(), mg2oLoopScw.translation() / mg2oLoopScw.scale());
  mpCurrentKF->SetPose(correctedTcw.cast<float>());

  std::shared_ptr<Map> pLoopMap = mpCurrentKF->GetMap();

  {
    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pLoopMap->mMutexMapUpdate);

    const bool bImuInit = pLoopMap->isImuInitialized();

    for (std::vector<std::shared_ptr<KeyFrame>>::iterator vit = vpCurrentConnectedKFs.begin(), vend = vpCurrentConnectedKFs.end(); vit != vend; vit++) {
      std::shared_ptr<KeyFrame> pKFi = *vit;

      if (pKFi != mpCurrentKF) {
        Sophus::SE3f Tiw = pKFi->GetPose();
        Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
        g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
        g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
        // Pose corrected with the Sim3 of the loop closure
        CorrectedSim3[pKFi] = g2oCorrectedSiw;

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
        pKFi->SetPose(correctedTiw.cast<float>());

        // Pose without correction
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(), Tiw.translation().cast<double>(), 1.0);
        NonCorrectedSim3[pKFi] = g2oSiw;
      }
    }

    // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
    for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++) {
      std::shared_ptr<KeyFrame> pKFi = mit->first;
      g2o::Sim3 g2oCorrectedSiw = mit->second;
      g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

      g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

      std::vector<std::shared_ptr<MapPoint>> vpMPsi = pKFi->GetMapPointMatches();
      for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++) {
        std::shared_ptr<MapPoint> pMPi = vpMPsi[iMP];
        if (!pMPi) continue;
        if (pMPi->isBad()) continue;
        if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId) continue;

        // Project with non-corrected pose and project back with corrected pose
        Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

        pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
        pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
        pMPi->mnCorrectedReference = pKFi->mnId;
        pMPi->UpdateNormalAndDepth();
      }

      // Correct velocity according to orientation correction
      if (bImuInit) {
        Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation()).cast<float>();
        pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
      }

      // Make sure connections are updated
      pKFi->UpdateConnections();
    }
    // TODO Check this index increasement
    mpAtlas->GetCurrentMap()->IncreaseChangeIndex();

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    for (size_t i = 0; i < mvpLoopMatchedMPs.size(); i++) {
      if (mvpLoopMatchedMPs[i]) {
        std::shared_ptr<MapPoint> pLoopMP = mvpLoopMatchedMPs[i];
        std::shared_ptr<MapPoint> pCurMP = mpCurrentKF->GetMapPoint(i);
        if (pCurMP)
          pCurMP->Replace(pLoopMP);
        else {
          mpCurrentKF->AddMapPoint(pLoopMP, i);
          pLoopMP->AddObservation(mpCurrentKF, i);
          pLoopMP->ComputeDistinctiveDescriptors();
        }
      }
    }
  }

  // Project MapPoints observed in the neighborhood of the loop keyframe into the current keyframe and neighbors using corrected poses.
  // Fuse duplications.
  SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

  // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
  std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>> LoopConnections;

  for (std::vector<std::shared_ptr<KeyFrame>>::iterator vit = vpCurrentConnectedKFs.begin(), vend = vpCurrentConnectedKFs.end(); vit != vend; vit++) {
    std::shared_ptr<KeyFrame> pKFi = *vit;
    std::vector<std::shared_ptr<KeyFrame>> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

    // Update connections. Detect new links.
    pKFi->UpdateConnections();
    LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
    for (std::vector<std::shared_ptr<KeyFrame>>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++) {
      LoopConnections[pKFi].erase(*vit_prev);
    }
    for (std::vector<std::shared_ptr<KeyFrame>>::iterator vit2 = vpCurrentConnectedKFs.begin(), vend2 = vpCurrentConnectedKFs.end(); vit2 != vend2; vit2++) {
      LoopConnections[pKFi].erase(*vit2);
    }
  }

  // Optimize graph
  bool bFixedScale = mbFixScale && !(mpTracker->mSensor == CameraType::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetInertialBA2());
  // TODO CHECK; Solo para el monocular inertial

  if (mbInertial && pLoopMap->isImuInitialized()) {
    Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
  } else {
    Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
  }

  mpAtlas->InformNewBigChange();

  // Add loop edge
  mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
  mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

  // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
  if (!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1)) {
    mbRunningGBA = true;
    mbStopGBA = false;
    std::cout << "Creating CorrectLoop thread" << std::endl;
    mpThreadGBA = std::jthread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
  }

  // Loop closed. Release Local Mapping.
  mpLocalMapper->Release();

}

void LoopClosing::MergeLocal() {
  std::cout << "MERGE LOCAL MAP" << std::endl;
  int numTemporalKFs = 25;  // Temporal KFs in the local window if the map is inertial.

  // Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
  std::shared_ptr<KeyFrame> pNewChild;
  std::shared_ptr<KeyFrame> pNewParent;

  std::vector<std::shared_ptr<KeyFrame>> vpLocalCurrentWindowKFs;
  std::vector<std::shared_ptr<KeyFrame>> vpMergeConnectedKFs;

  // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
  bool bRelaunchBA = false;

  // If a Global Bundle Adjustment is running, abort it
  if (isRunningGBA()) {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;
    bRelaunchBA = true;
  }

  mpLocalMapper->RequestStop();
  // Wait until Local Mapping has effectively stopped
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  mpLocalMapper->EmptyQueue();

  // Merge map will become in the new active map with the local window of KFs and MPs from the current map. Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
  // Translation (I think?): The Merge Map becomes the new active map, and all of the MapPoints, KeyFrames, etc.  from the Current Map will be transormed to fit the Merge Map
  std::shared_ptr<Map> pCurrentMap = mpCurrentKF->GetMap();
  std::shared_ptr<Map> pMergeMap = mpMergeMatchedKF->GetMap();

  // Ensure current keyframe is updated
  mpCurrentKF->UpdateConnections();

  // Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
  std::set<std::shared_ptr<KeyFrame>> spLocalWindowKFs;
  // Get MPs in the welding area from the current map
  std::set<std::shared_ptr<MapPoint>> spLocalWindowMPs;
  if (mbInertial){  // TODO Check the correct initialization
    std::shared_ptr<KeyFrame> pKFi = mpCurrentKF;
    int nInserted = 0;
    while (pKFi && nInserted < numTemporalKFs) {
      spLocalWindowKFs.insert(pKFi);
      pKFi = mpCurrentKF->mPrevKF;
      nInserted++;

      std::set<std::shared_ptr<MapPoint>> spMPi = pKFi->GetMapPoints();
      spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
    }

    pKFi = mpCurrentKF->mNextKF;
    while (pKFi) {
      spLocalWindowKFs.insert(pKFi);

      std::set<std::shared_ptr<MapPoint>> spMPi = pKFi->GetMapPoints();
      spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

      pKFi = mpCurrentKF->mNextKF;
    }
  } else {
    spLocalWindowKFs.insert(mpCurrentKF);
  }

  std::vector<std::shared_ptr<KeyFrame>> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
  spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
  spLocalWindowKFs.insert(mpCurrentKF);
  const int nMaxTries = 5;
  for (size_t nNumTries = 0; static_cast<int>(spLocalWindowKFs.size()) < numTemporalKFs && nNumTries < nMaxTries; ++nNumTries) {
    std::vector<std::shared_ptr<KeyFrame>> vpNewCovKFs;
    for (std::shared_ptr<KeyFrame> pKFi : spLocalWindowKFs) {
      std::vector<std::shared_ptr<KeyFrame>> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
      for (std::shared_ptr<KeyFrame> pKFcov : vpKFiCov) {
        if (pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end()) {
          vpNewCovKFs.push_back(pKFcov);
        }
      }
    }

    spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
  }

  for (std::shared_ptr<KeyFrame> pKFi : spLocalWindowKFs) {
    if (!pKFi || pKFi->isBad()) continue;

    std::set<std::shared_ptr<MapPoint>> spMPs = pKFi->GetMapPoints();
    spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
  }

  std::set<std::shared_ptr<KeyFrame>> spMergeConnectedKFs;
  if (mbInertial) {  // TODO Check the correct initialization
    std::shared_ptr<KeyFrame> pKFi = mpMergeMatchedKF;
    int nInserted = 0;
    while (pKFi && nInserted < numTemporalKFs / 2) {
      spMergeConnectedKFs.insert(pKFi);
      pKFi = mpCurrentKF->mPrevKF;
      nInserted++;
    }

    pKFi = mpMergeMatchedKF->mNextKF;
    while (pKFi && nInserted < numTemporalKFs) {
      spMergeConnectedKFs.insert(pKFi);
      pKFi = mpCurrentKF->mNextKF;
    }
  } else {
    spMergeConnectedKFs.insert(mpMergeMatchedKF);
  }
  vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
  spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
  spMergeConnectedKFs.insert(mpMergeMatchedKF);
  for (size_t nNumTries = 0; static_cast<int>(spMergeConnectedKFs.size()) < numTemporalKFs && nNumTries < nMaxTries; ++nNumTries) {
    std::vector<std::shared_ptr<KeyFrame>> vpNewCovKFs;
    for (std::shared_ptr<KeyFrame> pKFi : spMergeConnectedKFs) {
      std::vector<std::shared_ptr<KeyFrame>> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
      for (std::shared_ptr<KeyFrame> pKFcov : vpKFiCov) {
        if (pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end()) {
          vpNewCovKFs.push_back(pKFcov);
        }
      }
    }

    spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
  }

  std::set<std::shared_ptr<MapPoint>> spMapPointMerge;
  for (std::shared_ptr<KeyFrame> pKFi : spMergeConnectedKFs) {
    std::set<std::shared_ptr<MapPoint>> vpMPs = pKFi->GetMapPoints();
    spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
  }

  std::vector<std::shared_ptr<MapPoint>> vpCheckFuseMapPoint;
  vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
  std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

  Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
  g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
  g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
  g2o::Sim3 g2oCorrectedScw = mg2oMergeScw;  // TODO Check the transformation

  KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
  vCorrectedSim3[mpCurrentKF] = g2oCorrectedScw;
  vNonCorrectedSim3[mpCurrentKF] = g2oNonCorrectedScw;

  for (std::shared_ptr<KeyFrame> pKFi : spLocalWindowKFs) {
    if (!pKFi || pKFi->isBad()) {
      Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
      continue;
    }

    if (pKFi->GetMap() != pCurrentMap)
      Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

    g2o::Sim3 g2oCorrectedSiw;

    if (pKFi != mpCurrentKF) {
      Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
      g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
      // Pose without correction
      vNonCorrectedSim3[pKFi] = g2oSiw;

      Sophus::SE3d Tic = Tiw * Twc;
      g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
      g2oCorrectedSiw = g2oSic * mg2oMergeScw;
      vCorrectedSim3[pKFi] = g2oCorrectedSiw;
    } else {
      g2oCorrectedSiw = g2oCorrectedScw;
    }
    pKFi->mTcwMerge = pKFi->GetPose();

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
    double s = g2oCorrectedSiw.scale();
    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

    pKFi->mTcwMerge = correctedTiw.cast<float>();

    if (pCurrentMap->isImuInitialized()) {
      Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
      pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
    }

    // TODO DEBUG to know which are the KFs that had been moved to the other map
  }

  int numPointsWithCorrection = 0;

  std::set<std::shared_ptr<MapPoint>>::iterator itMP = spLocalWindowMPs.begin();
  while (itMP != spLocalWindowMPs.end()) {
    std::shared_ptr<MapPoint> pMPi = *itMP;
    if (!pMPi || pMPi->isBad()) {
      itMP = spLocalWindowMPs.erase(itMP);
      continue;
    }

    if(std::shared_ptr<KeyFrame> pKFref = (pMPi->GetReferenceKeyFrame()).lock()) {
      if (vCorrectedSim3.find(pKFref) == vCorrectedSim3.end()) {
        itMP = spLocalWindowMPs.erase(itMP);
        numPointsWithCorrection++;
        continue;
      }
      g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
      g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

      // Project with non-corrected pose and project back with corrected pose
      Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
      Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
      Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

      pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
      pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

      itMP++;
    } else {
      itMP = spLocalWindowMPs.erase(itMP);
      continue;
    }


  }

  {
    std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate);  // We update the current map with the merge information
    std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate);  // We remove the Kfs and MPs in the merged area from the old map

    for (std::shared_ptr<KeyFrame> pKFi : spLocalWindowKFs) {
      if (!pKFi || pKFi->isBad()) continue;

      pKFi->mTcwBefMerge = pKFi->GetPose();
      pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
      pKFi->SetPose(pKFi->mTcwMerge);

      // Make sure connections are updated
      pKFi->UpdateMap(pMergeMap);
      pMergeMap->AddKeyFrame(pKFi);
      pCurrentMap->EraseKeyFrame(pKFi);

      if (pCurrentMap->isImuInitialized()) {
        pKFi->SetVelocity(pKFi->mVwbMerge);
      }
    }

    for (std::shared_ptr<MapPoint> pMPi : spLocalWindowMPs) {
      if (!pMPi || pMPi->isBad()) continue;

      pMPi->SetWorldPos(pMPi->mPosMerge);
      pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
      pMPi->UpdateMap(pMergeMap);
      pMergeMap->AddMapPoint(pMPi);
      pCurrentMap->EraseMapPoint(pMPi);
    }

    mpAtlas->ChangeMap(pMergeMap);
    mpAtlas->SetMapBad(pCurrentMap);
    pMergeMap->IncreaseChangeIndex();
    // TODO for debug
    pMergeMap->ChangeId(pCurrentMap->GetId());
  }

  // Rebuild the essential graph in the local window
  pCurrentMap->GetOriginKF()->SetFirstConnection(false);
  // Old parent, it will be the new child of this KF
  pNewChild = mpCurrentKF->GetParent();     
  // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
  pNewParent = mpCurrentKF;
  mpCurrentKF->ChangeParent(mpMergeMatchedKF);
  while (pNewChild) {
    // We remove the relation between the old parent and the new for avoid loop
    pNewChild->EraseChild(pNewParent);  
    std::shared_ptr<KeyFrame> pOldParent = pNewChild->GetParent();

    pNewChild->ChangeParent(pNewParent);

    pNewParent = pNewChild;
    pNewChild = pOldParent;
  }

  // Update the connections between the local window
  mpMergeMatchedKF->UpdateConnections();

  vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
  vpMergeConnectedKFs.push_back(mpMergeMatchedKF);

  // Project MapPoints observed in the neighborhood of the merge keyframe into the current keyframe and neighbors using corrected poses.
  // Fuse duplications.
  SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

  // Update connectivity
  for (std::shared_ptr<KeyFrame> pKFi : spLocalWindowKFs) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->UpdateConnections();
  }
  for (std::shared_ptr<KeyFrame> pKFi : spMergeConnectedKFs) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->UpdateConnections();
  }

  bool bStop = false;
  vpLocalCurrentWindowKFs.clear();
  vpMergeConnectedKFs.clear();
  std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
  std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
  if (mpTracker->mSensor == CameraType::IMU_MONOCULAR || mpTracker->mSensor == CameraType::IMU_STEREO || mpTracker->mSensor == CameraType::IMU_RGBD) {
    Optimizer::MergeInertialBA(mpCurrentKF, mpMergeMatchedKF, &bStop, pCurrentMap, vCorrectedSim3);
  } else {
    Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs, &bStop);
  }

  // Loop closed. Release Local Mapping.
  mpLocalMapper->Release();

  // Update the non critical area from the current map to the merged map
  std::vector<std::shared_ptr<KeyFrame>> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
  std::vector<std::shared_ptr<MapPoint>> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

  if (vpCurrentMapKFs.size() != 0) {
    if (mpTracker->mSensor == CameraType::MONOCULAR) {
      // We update the current map with the merge information
      std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate);

      for (std::shared_ptr<KeyFrame> pKFi : vpCurrentMapKFs) {
        if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
          continue;
        }

        g2o::Sim3 g2oCorrectedSiw;

        Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        // Pose without correction
        vNonCorrectedSim3[pKFi] = g2oSiw;

        Sophus::SE3d Tic = Tiw * Twc;
        g2o::Sim3 g2oSim(Tic.unit_quaternion(), Tic.translation(), 1.0);
        g2oCorrectedSiw = g2oSim * mg2oMergeScw;
        vCorrectedSim3[pKFi] = g2oCorrectedSiw;

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        double s = g2oCorrectedSiw.scale();

        Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

        pKFi->mTcwBefMerge = pKFi->GetPose();
        pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

        pKFi->SetPose(correctedTiw.cast<float>());

        if (pCurrentMap->isImuInitialized()) {
          Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
          pKFi->SetVelocity(Rcor * pKFi->GetVelocity());  // TODO: should add here scale s
        }
      }
      for (std::shared_ptr<MapPoint> pMPi : vpCurrentMapMPs) {
        if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pCurrentMap) continue;

        if(std::shared_ptr<KeyFrame> pKFref = (pMPi->GetReferenceKeyFrame()).lock()) {
          g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
          g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

          // Project with non-corrected pose and project back with corrected pose
          Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
          Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
          pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());

          pMPi->UpdateNormalAndDepth();
        }
      }
    }

    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped()) {
      usleep(1000);
    }

    // Optimize graph (and update the loop position for each element from the beginning to the end)
    if (mpTracker->mSensor != CameraType::MONOCULAR) {
      Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs);
    }

    {
      // Get Merge Map std::mutex
      std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate);  // We update the current map with the Merge information
      std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate);  // We remove the Kfs and MPs in the merged area from the old map

      for (std::shared_ptr<KeyFrame> pKFi : vpCurrentMapKFs) {
        if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap) continue;

        // Make sure connections are updated
        pKFi->UpdateMap(pMergeMap);
        pMergeMap->AddKeyFrame(pKFi);
        pCurrentMap->EraseKeyFrame(pKFi);
      }

      for (std::shared_ptr<MapPoint> pMPi : vpCurrentMapMPs) {
        if (!pMPi || pMPi->isBad()) continue;

        pMPi->UpdateMap(pMergeMap);
        pMergeMap->AddMapPoint(pMPi);
        pCurrentMap->EraseMapPoint(pMPi);
      }
    }
  }

  mpLocalMapper->Release();

  if (bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1))) {
    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbStopGBA = false;
    std::cout << "Creating MergeLocal thread" << std::endl;
    mpThreadGBA = std::jthread(&LoopClosing::RunGlobalBundleAdjustment, this, pMergeMap, mpCurrentKF->mnId);
  }

  mpMergeMatchedKF->SetNotErase();
  mpCurrentKF->SetNotErase();

  pCurrentMap->IncreaseChangeIndex();
  pMergeMap->IncreaseChangeIndex();

  mpAtlas->RemoveBadMaps();
}

void LoopClosing::MergeLocal2() {
  std::cout << "MERGE LOCAL MAP 2" << std::endl;
  loopClosed = true;
  Verbose::PrintMess("Merge detected!!!", Verbose::VERBOSITY_NORMAL);

  // int numTemporalKFs = 11; // TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

  // Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
  std::shared_ptr<KeyFrame> pNewChild;
  std::shared_ptr<KeyFrame> pNewParent;

  KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
 
  // If a Global Bundle Adjustment is running, abort it
  if (isRunningGBA()) {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    mbStopGBA = true;
    mnFullBAIdx++;
  }

  mpLocalMapper->RequestStop();
  // Wait until Local Mapping has effectively stopped
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  std::shared_ptr<Map> pCurrentMap = mpCurrentKF->GetMap();
  std::shared_ptr<Map> pMergeMap = mpMergeMatchedKF->GetMap();

  {
    float s_on = mSold_new.scale();
    Sophus::SE3f T_on(mSold_new.rotation().cast<float>(), mSold_new.translation().cast<float>());

    std::unique_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    mpLocalMapper->EmptyQueue();

    bool bScaleVel = false;
    if (s_on != 1) bScaleVel = true;
    mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on, s_on, bScaleVel);
    mpTracker->UpdateFrameIMU(s_on, mpCurrentKF->GetImuBias(), mpTracker->GetLastKeyFrame());
  }

  const int numKFnew = pCurrentMap->KeyFramesInMap();

  if (mpTracker->mSensor.isInertial() && !pCurrentMap->GetInertialBA2()) {
    // Map is not completly initialized
    Eigen::Vector3d bg, ba;
    bg << 0., 0., 0.;
    ba << 0., 0., 0.;
    Optimizer::InertialOptimization(pCurrentMap, bg, ba);
    IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
    std::unique_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    mpTracker->UpdateFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

    // Set map initialized
    pCurrentMap->SetInertialBA2();
    pCurrentMap->SetInertialBA1();
    pCurrentMap->SetImuInitialized();
  }

  // Load KFs and MPs from merge map
  {
    // Get Merge Map Mutex (This section stops tracking!!)
    std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate);  // We update the current std::map with the merge information
    std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate);  // We remove the Kfs and MPs in the merged area from the old std::map

    std::vector<std::shared_ptr<KeyFrame>> vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
    std::vector<std::shared_ptr<MapPoint>> vpMergeMapMPs = pMergeMap->GetAllMapPoints();

    for (std::shared_ptr<KeyFrame> pKFi : vpMergeMapKFs) {
      if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap) continue;
      
      // Make sure connections are updated
      pKFi->UpdateMap(pCurrentMap);
      pCurrentMap->AddKeyFrame(pKFi);
      pMergeMap->EraseKeyFrame(pKFi);
    }

    for (std::shared_ptr<MapPoint> pMPi : vpMergeMapMPs) {
      if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap) continue;

      pMPi->UpdateMap(pCurrentMap);
      pCurrentMap->AddMapPoint(pMPi);
      pMergeMap->EraseMapPoint(pMPi);
    }

    // Save non corrected poses (already merged maps)
    std::vector<std::shared_ptr<KeyFrame>> vpKFs = pCurrentMap->GetAllKeyFrames();
    for (std::shared_ptr<KeyFrame> pKFi : vpKFs) {
      Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
      g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
      NonCorrectedSim3[pKFi] = g2oSiw;
    }
  }

  
  // Critical zone
  pMergeMap->GetOriginKF()->SetFirstConnection(false);
  pNewChild = mpMergeMatchedKF->GetParent();  // Old parent, it will be the new child of this KF
  pNewParent = mpMergeMatchedKF;  // Old child, now it will be the parent of its own parent(we need eliminate this KF from children std::list in its old parent)
  mpMergeMatchedKF->ChangeParent(mpCurrentKF);
  while (pNewChild) {
    pNewChild->EraseChild(pNewParent);  // We remove the relation between the old parent and the new for avoid loop
    std::shared_ptr<KeyFrame> pOldParent = pNewChild->GetParent();
    pNewChild->ChangeParent(pNewParent);
    pNewParent = pNewChild;
    pNewChild = pOldParent;
  }

  std::vector<std::shared_ptr<MapPoint>> vpCheckFuseMapPoint;  // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
  std::vector<std::shared_ptr<KeyFrame>> vpCurrentConnectedKFs;
  std::vector<std::shared_ptr<KeyFrame>> vpMergeConnectedKFs;

  vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
  std::vector<std::shared_ptr<KeyFrame>> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
  vpMergeConnectedKFs.insert(vpMergeConnectedKFs.end(), aux.begin(), aux.end());
  if(vpMergeConnectedKFs.size() > 6)
    vpMergeConnectedKFs.erase(vpMergeConnectedKFs.begin() + 6, vpMergeConnectedKFs.end());

  mpCurrentKF->UpdateConnections();
  vpCurrentConnectedKFs.push_back(mpCurrentKF);

  aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
  vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
  if (vpCurrentConnectedKFs.size() > 6)
    vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6, vpCurrentConnectedKFs.end());

  std::set<std::shared_ptr<MapPoint>> spMapPointMerge;
  for (std::shared_ptr<KeyFrame> pKFi : vpMergeConnectedKFs) {
    std::set<std::shared_ptr<MapPoint>> vpMPs = pKFi->GetMapPoints();
    spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
    if (spMapPointMerge.size() > 1000) break;
  }

  vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
  std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

  SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

  for (std::shared_ptr<KeyFrame> pKFi : vpCurrentConnectedKFs) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->UpdateConnections();
  }
  for (std::shared_ptr<KeyFrame> pKFi : vpMergeConnectedKFs) {
    if (!pKFi || pKFi->isBad()) continue;

    pKFi->UpdateConnections();
  }

  // TODO Check: If new map is too small, we suppose that not enough information can be propagated from new to old map
  if (numKFnew < 10) {
    mpLocalMapper->Release();
    return;
  }

  // Perform BA
  bool bStopFlag = false;
  std::shared_ptr<KeyFrame> pCurrKF = mpTracker->GetLastKeyFrame();
  if (pCurrKF == nullptr) {
    std::cerr << "\033[22;34mcurrent KF is nullptr" << std::endl;
    mpLocalMapper->Release();
    return;
  }
  Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap, CorrectedSim3);

  // Release Local Mapping.
  mpLocalMapper->Release();
  hasMergedLocalMap = true;

  return;
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose& CorrectedPosesMap, std::vector<std::shared_ptr<MapPoint>>& vpMapPoints) {
  ORBmatcher matcher(0.8);

  int total_replaces = 0;

  for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; mit++) {
    int num_replaces = 0;
    std::shared_ptr<KeyFrame> pKFi = mit->first;
    std::shared_ptr<Map> pMap = pKFi->GetMap();

    g2o::Sim3 g2oScw = mit->second;
    Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

    std::vector<std::shared_ptr<MapPoint>> vpReplacePoints(vpMapPoints.size(), nullptr);
    matcher.Fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    const int nLP = vpMapPoints.size();
    for (int i = 0; i < nLP; i++) {
      std::shared_ptr<MapPoint> pRep = vpReplacePoints[i];
      if (pRep) {
        num_replaces += 1;
        pRep->Replace(vpMapPoints[i]);
      }
    }

    total_replaces += num_replaces;
  }
}

void LoopClosing::SearchAndFuse(const std::vector<std::shared_ptr<KeyFrame>>& vConectedKFs, std::vector<std::shared_ptr<MapPoint>>& vpMapPoints) {
  ORBmatcher matcher(0.8);

  for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end(); mit != mend; mit++) {
    int num_replaces = 0;
    std::shared_ptr<KeyFrame> pKF = (*mit);
    std::shared_ptr<Map> pMap = pKF->GetMap();
    Sophus::SE3f Tcw = pKF->GetPose();
    Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
    Scw.setScale(1.f);
    std::vector<std::shared_ptr<MapPoint>> vpReplacePoints(vpMapPoints.size(), nullptr);
    matcher.Fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    const int nLP = vpMapPoints.size();
    for (int i = 0; i < nLP; i++) {
      std::shared_ptr<MapPoint> pRep = vpReplacePoints[i];
      if (pRep) {
        num_replaces += 1;
        pRep->Replace(vpMapPoints[i]);
      }
    }
  }
}

void LoopClosing::RequestReset() {
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetRequested = true;
  }

  while (1) {
    {
      std::unique_lock<std::mutex> lock2(mMutexReset);
      if (!mbResetRequested) break;
    }
    usleep(5000);
  }
}

void LoopClosing::RequestResetActiveMap(std::shared_ptr<Map> pMap) {
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetActiveMapRequested = true;
    mpMapToReset = pMap;
  }

  while (1) {
    {
      std::unique_lock<std::mutex> lock2(mMutexReset);
      if (!mbResetActiveMapRequested) break;
    }
    usleep(3000);
  }
}

void LoopClosing::ResetIfRequested() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  if (mbResetRequested) {
    std::cout << "Loop closer reset requested..." << std::endl;
    mlpLoopKeyFrameQueue.clear();
    mbResetRequested = false;
    mbResetActiveMapRequested = false;
  } else if (mbResetActiveMapRequested) {
    for (std::list<std::shared_ptr<KeyFrame>>::const_iterator it = mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();) {
      std::shared_ptr<KeyFrame> pKFi = *it;
      if (pKFi->GetMap() == mpMapToReset) {
        it = mlpLoopKeyFrameQueue.erase(it);
      } else
        ++it;
    }

    mbResetActiveMapRequested = false;
  }
}

void LoopClosing::RunGlobalBundleAdjustment(std::shared_ptr<Map> pActiveMap, unsigned long nLoopKF) {
  Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

  const bool bImuInit = pActiveMap->isImuInitialized();

  if (!bImuInit)
    Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF, false);
  else
    Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA);

  int idx = mnFullBAIdx;

  // Update all MapPoints and KeyFrames
  // Local Mapping was active during BA, that means that there might be new keyframes not included in the Global BA and they are not consistent with the updated map.
  // We need to propagate the correction through the spanning tree
  {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    if (idx != mnFullBAIdx) return;

    if (!bImuInit && pActiveMap->isImuInitialized()) return;

    if (!mbStopGBA) {
      Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
      Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

      mpLocalMapper->RequestStop();
      // Wait until Local Mapping has effectively stopped

      while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {
        usleep(1000);
      }

      // Get Map Mutex
      std::unique_lock<std::mutex> lock(pActiveMap->mMutexMapUpdate);

      // Correct keyframes starting at map first keyframe
      std::list<std::shared_ptr<KeyFrame>> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(), pActiveMap->mvpKeyFrameOrigins.end());

      while (!lpKFtoCheck.empty()) {
        std::shared_ptr<KeyFrame> pKF = lpKFtoCheck.front();
        const std::set<std::shared_ptr<KeyFrame>> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();

        for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
          std::shared_ptr<KeyFrame> pChild = *sit;
          if (!pChild || pChild->isBad()) continue;

          if (pChild->mnBAGlobalForKF != nLoopKF) {
            Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

            Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
            if (pChild->isVelocitySet()) {
              pChild->mVwbGBA = Rcor * pChild->GetVelocity();
            } else
              Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);

            pChild->mBiasGBA = pChild->GetImuBias();

            pChild->mnBAGlobalForKF = nLoopKF;
          }
          lpKFtoCheck.push_back(pChild);
        }

        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);

        if (pKF->bImu) {
          pKF->mVwbBefGBA = pKF->GetVelocity();
          // assert(!pKF->mVwbGBA.empty());
          pKF->SetVelocity(pKF->mVwbGBA);
          pKF->SetNewBias(pKF->mBiasGBA);
        }

        lpKFtoCheck.pop_front();
      }

      // Correct MapPoints
      const std::vector<std::shared_ptr<MapPoint>> vpMPs = pActiveMap->GetAllMapPoints();

      for (size_t i = 0; i < vpMPs.size(); i++) {
        std::shared_ptr<MapPoint> pMP = vpMPs[i];

        if (pMP->isBad()) continue;

        if (pMP->mnBAGlobalForKF == nLoopKF) {
          // If optimized by Global BA, just update
          pMP->SetWorldPos(pMP->mPosGBA);
        // Update according to the correction of its reference keyframe
        } else if(std::shared_ptr<KeyFrame> pRefKF = (pMP->GetReferenceKeyFrame()).lock()) {

          if (pRefKF->mnBAGlobalForKF != nLoopKF) continue;

          // Map to non-corrected camera
          Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

          // Backproject using corrected camera
          pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
      }

      pActiveMap->InformNewBigChange();
      pActiveMap->IncreaseChangeIndex();

      mpLocalMapper->Release();
      Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
    }

    mbRunningGBA = false;
  }
}

void LoopClosing::RequestFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

bool LoopClosing::CheckFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

}  // namespace MORB_SLAM
