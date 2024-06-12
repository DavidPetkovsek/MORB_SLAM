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

#include "MORB_SLAM/Optimizer.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <complex>
#include <mutex>
#include <unsupported/Eigen/MatrixFunctions>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/G2oTypes.h"
#include "MORB_SLAM/OptimizableTypes.h"

namespace MORB_SLAM {

void Optimizer::LocalBundleAdjustment(std::shared_ptr<KeyFrame> pKF, bool* pbStopFlag, std::shared_ptr<Map> pMap, bool bInertial) {
  // Local KeyFrames: First Breath Search from Current Keyframe
  std::list<std::shared_ptr<KeyFrame>> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  std::shared_ptr<Map> pCurrentMap = pKF->GetMap();

  const std::vector<std::shared_ptr<KeyFrame>> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
  for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
    std::shared_ptr<KeyFrame> pKFi = vNeighKFs[i];
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
      lLocalKeyFrames.push_back(pKFi);
  }

  int num_fixedKF = 0;
  // Local MapPoints seen in Local KeyFrames
  std::list<std::shared_ptr<MapPoint>> lLocalMapPoints;
  std::set<std::shared_ptr<MapPoint>> sNumObsMP;
  for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;
    if (pKFi->mnId == pMap->GetInitKFid()) {
      num_fixedKF = 1;
    }
    std::vector<std::shared_ptr<MapPoint>> vpMPs = pKFi->GetMapPointMatches();
    for (std::vector<std::shared_ptr<MapPoint>>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      std::shared_ptr<MapPoint> pMP = *vit;
      if (pMP)
        if (!pMP->isBad() && pMP->GetMap() == pCurrentMap) {
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
        }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
  // Keyframes
  std::list<std::shared_ptr<KeyFrame>> lFixedCameras;
  for (std::list<std::shared_ptr<MapPoint>>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = (*lit)->GetObservations();
    for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      if(std::shared_ptr<KeyFrame> pKFi = (mit->first).lock()) {
        if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
          pKFi->mnBAFixedForKF = pKF->mnId;
          if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lFixedCameras.push_back(pKFi);
        }        
      }
    }
  }

  num_fixedKF += lFixedCameras.size();

  if (num_fixedKF == 0) {
    Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
    return;
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  if (bInertial) solver->setUserLambdaInit(100.0);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // DEBUG LBA
  pCurrentMap->msOptKFs.clear();
  pCurrentMap->msFixedKFs.clear();

  // Set Local KeyFrame vertices
  for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msOptKFs.insert(pKFi->mnId);
  }

  // Set Fixed KeyFrame vertices
  for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lFixedCameras.begin(),
                                 lend = lFixedCameras.end();
       lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    // DEBUG LBA
    pCurrentMap->msFixedKFs.insert(pKFi->mnId);
  }

  // Set MapPoint vertices
  const int nExpectedSize =
      (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  std::vector<MORB_SLAM::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<MORB_SLAM::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  std::vector<std::shared_ptr<MapPoint>> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  std::vector<std::shared_ptr<MapPoint>> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<std::shared_ptr<MapPoint>> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  int nPoints = 0;

  int nEdges = 0;

  for (std::list<std::shared_ptr<MapPoint>>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    std::shared_ptr<MapPoint> pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    nPoints++;

    const std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = pMP->GetObservations();

    // Set edges
    for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      if(std::shared_ptr<KeyFrame> pKFi = (mit->first).lock()) {
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
          const int leftIndex = std::get<0>(mit->second);

          // Monocular observation
          if (leftIndex != -1 && pKFi->mvuRight[std::get<0>(mit->second)] < 0) {
            const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            MORB_SLAM::EdgeSE3ProjectXYZ* e = new MORB_SLAM::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            e->pCamera = pKFi->mpCamera;

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);

            nEdges++;
          } else if (leftIndex != -1 && pKFi->mvuRight[std::get<0>(mit->second)] >=
                                            0)  // Stereo observation
          {
            const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
            Eigen::Matrix<double, 3, 1> obs;
            const float kp_ur = pKFi->mvuRight[std::get<0>(mit->second)];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberStereo);

            e->fx = pKFi->fx;
            e->fy = pKFi->fy;
            e->cx = pKFi->cx;
            e->cy = pKFi->cy;
            e->bf = pKFi->mbf;

            optimizer.addEdge(e);
            vpEdgesStereo.push_back(e);
            vpEdgeKFStereo.push_back(pKFi);
            vpMapPointEdgeStereo.push_back(pMP);

            nEdges++;
          }

          if (pKFi->mpCamera2) {
            int rightIndex = std::get<1>(mit->second);

            if (rightIndex != -1) {
              rightIndex -= pKFi->NLeft;

              Eigen::Matrix<double, 2, 1> obs;
              cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
              obs << kp.pt.x, kp.pt.y;

              MORB_SLAM::EdgeSE3ProjectXYZToBody* e =
                  new MORB_SLAM::EdgeSE3ProjectXYZToBody();

              e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                  optimizer.vertex(id)));
              e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                  optimizer.vertex(pKFi->mnId)));
              e->setMeasurement(obs);
              const float& invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
              e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

              g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuberMono);

              Sophus::SE3f Trl = pKFi->GetRelativePoseTrl();
              e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                    Trl.translation().cast<double>());

              e->pCamera = pKFi->mpCamera2;

              optimizer.addEdge(e);
              vpEdgesBody.push_back(e);
              vpEdgeKFBody.push_back(pKFi);
              vpMapPointEdgeBody.push_back(pMP);

              nEdges++;
            }
          }
        }
      }
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  std::vector<std::pair<std::shared_ptr<KeyFrame>, std::shared_ptr<MapPoint>>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                   vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    MORB_SLAM::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    std::shared_ptr<MapPoint> pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      std::shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
    MORB_SLAM::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
    std::shared_ptr<MapPoint> pMP = vpMapPointEdgeBody[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      std::shared_ptr<KeyFrame> pKFi = vpEdgeKFBody[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    std::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      std::shared_ptr<KeyFrame> pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      std::shared_ptr<KeyFrame> pKFi = vToErase[i].first;
      std::shared_ptr<MapPoint> pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data
  // Keyframes
  for (std::list<std::shared_ptr<KeyFrame>>::iterator lit = lLocalKeyFrames.begin(),
                                 lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    std::shared_ptr<KeyFrame> pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());
    pKFi->SetPose(Tiw);
  }

  // Points
  for (std::list<std::shared_ptr<MapPoint>>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    std::shared_ptr<MapPoint> pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

void Optimizer::LocalBundleAdjustment(std::shared_ptr<KeyFrame> pMainKF,
                                      std::vector<std::shared_ptr<KeyFrame>> vpAdjustKF,
                                      std::vector<std::shared_ptr<KeyFrame>> vpFixedKF,
                                      bool* pbStopFlag) {
  // bool bShowImages = false; // UNUSED

  std::vector<std::shared_ptr<MapPoint>> vpMPs;

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;
  std::set<std::shared_ptr<KeyFrame>> spKeyFrameBA;

  std::shared_ptr<Map> pCurrentMap = pMainKF->GetMap();

  // Set fixed KeyFrame vertices
  int numInsertedPoints = 0;
  for (std::shared_ptr<KeyFrame> pKFi : vpFixedKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
      Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }

    pKFi->mnBALocalForMerge = pMainKF->mnId;

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

    std::set<std::shared_ptr<MapPoint>> spViewMPs = pKFi->GetMapPoints();
    for (std::shared_ptr<MapPoint> pMPi : spViewMPs) {
      if (pMPi)
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
    }

    spKeyFrameBA.insert(pKFi);
  }

  // Set non fixed Keyframe vertices
  std::set<std::shared_ptr<KeyFrame>> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
  numInsertedPoints = 0;
  for (std::shared_ptr<KeyFrame> pKFi : vpAdjustKF) {
    if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) continue;

    pKFi->mnBALocalForMerge = pMainKF->mnId;

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKFi->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKFi->mnId);
    optimizer.addVertex(vSE3);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;

    std::set<std::shared_ptr<MapPoint>> spViewMPs = pKFi->GetMapPoints();
    for (std::shared_ptr<MapPoint> pMPi : spViewMPs) {
      if (pMPi) {
        if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
          if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(pMPi);
            pMPi->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
        }
      }
    }

    spKeyFrameBA.insert(pKFi);
  }

  const int nExpectedSize =
      (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

  std::vector<MORB_SLAM::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<std::shared_ptr<MapPoint>> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<std::shared_ptr<MapPoint>> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuber2D = sqrt(5.99);
  const float thHuber3D = sqrt(7.815);

  // Set MapPoint vertices
  std::map<std::shared_ptr<KeyFrame>, int> mpObsKFs;
  std::map<std::shared_ptr<KeyFrame>, int> mpObsFinalKFs;
  std::map<std::shared_ptr<MapPoint>, int> mpObsMPs;
  for (unsigned int i = 0; i < vpMPs.size(); ++i) {
    std::shared_ptr<MapPoint> pMPi = vpMPs[i];
    if (pMPi->isBad()) continue;

    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
    const int id = pMPi->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = pMPi->GetObservations();
    int nEdges = 0;
    // SET EDGES
    for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {
      if(std::shared_ptr<KeyFrame> pKF = (mit->first).lock()) {
        if (pKF->isBad() || pKF->mnId > maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId || !pKF->GetMapPoint(std::get<0>(mit->second)))
          continue;

        nEdges++;

        const cv::KeyPoint& kpUn = pKF->mvKeysUn[std::get<0>(mit->second)];

        if (pKF->mvuRight[std::get<0>(mit->second)] < 0)  // Monocular
        {
          mpObsMPs[pMPi]++;
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          MORB_SLAM::EdgeSE3ProjectXYZ* e = new MORB_SLAM::EdgeSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);

          e->pCamera = pKF->mpCamera;

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKF);
          vpMapPointEdgeMono.push_back(pMPi);

          mpObsKFs[pKF]++;
        } else  // RGBD or Stereo
        {
          mpObsMPs[pMPi] += 2;
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKF->mvuRight[std::get<0>(mit->second)];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber3D);

          e->fx = pKF->fx;
          e->fy = pKF->fy;
          e->cx = pKF->cx;
          e->cy = pKF->cy;
          e->bf = pKF->mbf;

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKF);
          vpMapPointEdgeStereo.push_back(pMPi);

          mpObsKFs[pKF]++;
        }
      }
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  bool bDoMore = true;

  if (pbStopFlag)
    if (*pbStopFlag) bDoMore = false;

  std::map<unsigned long int, int> mWrongObsKF;
  if (bDoMore) {
    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      MORB_SLAM::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      std::shared_ptr<MapPoint> pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        e->setLevel(1);
        badMonoMP++;
      }
      e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
      std::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        e->setLevel(1);
        badStereoMP++;
      }

      e->setRobustKernel(0);
    }
    Verbose::PrintMess("[BA]: First optimization(Huber), there are " +
                           std::to_string(badMonoMP) + " monocular and " +
                           std::to_string(badStereoMP) + " stereo bad edges",
                       Verbose::VERBOSITY_DEBUG);

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
  }

  std::vector<std::pair<std::shared_ptr<KeyFrame>, std::shared_ptr<MapPoint>>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
  std::set<std::shared_ptr<MapPoint>> spErasedMPs;
  std::set<std::shared_ptr<KeyFrame>> spErasedKFs;

  // Check inlier observations
  int badMonoMP = 0, badStereoMP = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    MORB_SLAM::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    std::shared_ptr<MapPoint> pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      std::shared_ptr<KeyFrame> pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badMonoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    std::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      std::shared_ptr<KeyFrame> pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badStereoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  Verbose::PrintMess("[BA]: Second optimization, there are " +
                         std::to_string(badMonoMP) + " monocular and " +
                         std::to_string(badStereoMP) + " sterero bad edges",
                     Verbose::VERBOSITY_DEBUG);

  // Get Map Mutex
  std::unique_lock<std::mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      std::shared_ptr<KeyFrame> pKFi = vToErase[i].first;
      std::shared_ptr<MapPoint> pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }
  for (unsigned int i = 0; i < vpMPs.size(); ++i) {
    std::shared_ptr<MapPoint> pMPi = vpMPs[i];
    if (pMPi->isBad()) continue;

    const std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = pMPi->GetObservations();
    for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {
      if(std::shared_ptr<KeyFrame> pKF = (mit->first).lock()) {
        if (pKF->isBad() || pKF->mnId > maxKFid || pKF->mnBALocalForKF != pMainKF->mnId || !pKF->GetMapPoint(std::get<0>(mit->second)))
          continue;
        // Monocular
        if (pKF->mvuRight[std::get<0>(mit->second)] < 0) {
          mpObsFinalKFs[pKF]++;
        // RGBD or Stereo
        } else {
          mpObsFinalKFs[pKF]++;
        }
      }
    }
  }

  // Recover optimized data
  // Keyframes
  for (std::shared_ptr<KeyFrame> pKFi : vpAdjustKF) {
    if (pKFi->isBad()) continue;

    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                     SE3quat.translation().cast<float>());

    int numMonoBadPoints = 0, numMonoOptPoints = 0;
    int numStereoBadPoints = 0, numStereoOptPoints = 0;
    std::vector<std::shared_ptr<MapPoint>> vpMonoMPsOpt, vpStereoMPsOpt;
    std::vector<std::shared_ptr<MapPoint>> vpMonoMPsBad, vpStereoMPsBad;

    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      MORB_SLAM::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      std::shared_ptr<MapPoint> pMP = vpMapPointEdgeMono[i];
      std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) {
        continue;
      }

      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        numMonoBadPoints++;
        vpMonoMPsBad.push_back(pMP);

      } else {
        numMonoOptPoints++;
        vpMonoMPsOpt.push_back(pMP);
      }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
      std::shared_ptr<MapPoint> pMP = vpMapPointEdgeStereo[i];
      std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i];

      if (pKFi != pKFedge) {
        continue;
      }

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        numStereoBadPoints++;
        vpStereoMPsBad.push_back(pMP);
      } else {
        numStereoOptPoints++;
        vpStereoMPsOpt.push_back(pMP);
      }
    }

    pKFi->SetPose(Tiw);
  }

  // Points
  for (std::shared_ptr<MapPoint> pMPi : vpMPs) {
    if (pMPi->isBad()) continue;

    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMPi->mnId + maxKFid + 1));
    pMPi->SetWorldPos(vPoint->estimate().cast<float>());
    pMPi->UpdateNormalAndDepth();
  }
}



}  // namespace MORB_SLAM
