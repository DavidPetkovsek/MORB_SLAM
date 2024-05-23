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
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"

namespace MORB_SLAM {

bool sortByVal(const std::pair<MapPoint*, int>& a, const std::pair<MapPoint*, int>& b) {
  return (a.second < b.second);
}

void Optimizer::MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF,
                                bool* pbStopFlag, std::shared_ptr<Map> pMap,
                                LoopClosing::KeyFrameAndPose& corrPoses) {
  const int Nd = 6;

  const unsigned long maxKFid = pCurrKF->mnId;

  std::vector<KeyFrame*> vpOptimizableKFs;
  vpOptimizableKFs.reserve(2 * Nd);

  // For cov KFS, inertial parameters are not optimized
  const int maxCovKF = 30;
  std::vector<KeyFrame*> vpOptimizableCovKFs;
  vpOptimizableCovKFs.reserve(maxCovKF);

  // Add sliding window for current KF
  vpOptimizableKFs.push_back(pCurrKF);
  pCurrKF->mnBALocalForKF = pCurrKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else
      break;
  }

  std::list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
  } else {
    // vpOptimizableKFs.back()->mnBALocalForKF = 0;
    // vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
    vpOptimizableKFs.pop_back();
  }

  // Add temporal neighbours to merge KF (previous and next KFs)
  vpOptimizableKFs.push_back(pMergeKF);
  pMergeKF->mnBALocalForKF = pCurrKF->mnId;

  // Previous KFs
  for (int i = 1; i < (Nd / 2); i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else
      break;
  }

  // We fix just once the old map
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Next KFs
  if (pMergeKF->mNextKF) {
    vpOptimizableKFs.push_back(pMergeKF->mNextKF);
    vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;

    while (vpOptimizableKFs.size() < (2 * Nd)) {
      if (vpOptimizableKFs.back()->mNextKF) {
        vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
        vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
      } else
        break;
    }
  }
  int N = vpOptimizableKFs.size();

  // Optimizable points seen by optimizable keyframes
  std::list<MapPoint*> lLocalMapPoints;
  std::map<MapPoint*, int> mLocalObs;
  for (int i = 0; i < N; i++) {
    std::vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end();
         vit != vend; vit++) {
      // Using mnBALocalForKF we avoid redundance here, one MP can not be added
      // several times to lLocalMapPoints
      MapPoint* pMP = *vit;
      if (pMP) {
        if (!pMP->isBad()) {
          if (pMP->mnBALocalForKF != pCurrKF->mnId) {
            mLocalObs[pMP] = 1;
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pCurrKF->mnId;
          } else {
            mLocalObs[pMP]++;
          }
        }
      }
    }
  }

  std::vector<std::pair<MapPoint*, int>> pairs;
  pairs.reserve(mLocalObs.size());
  for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
    pairs.push_back(*itr);
  sort(pairs.begin(), pairs.end(), sortByVal);

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
  // Keyframes
  int i = 0;
  for (std::vector<std::pair<MapPoint*, int>>::iterator lit = pairs.begin(),
                                              lend = pairs.end();
       lit != lend; lit++, i++) {
    std::map<KeyFrame*, std::tuple<int, int>> observations =
        lit->first->GetObservations();
    if (i >= maxCovKF) break;
    for (std::map<KeyFrame*, std::tuple<int, int>>::iterator mit = observations.begin(),
                                                   mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pCurrKF->mnId &&
          pKFi->mnBAFixedForKF !=
              pCurrKF->mnId)  // If optimizable or already included...
      {
        pKFi->mnBALocalForKF = pCurrKF->mnId;
        if (!pKFi->isBad()) {
          vpOptimizableCovKFs.push_back(pKFi);
          break;
        }
      }
    }
  }

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);
  // Set Local KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }
  // Set Local cov keyframes vertices
  int Ncov = vpOptimizableCovKFs.size();
  for (int i = 0; i < Ncov; i++) {
    KeyFrame* pKFi = vpOptimizableCovKFs[i];
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }
  // Set Fixed KeyFrame vertices
  for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                 lend = lFixedKeyFrames.end();
       lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // Create intertial constraints
  std::vector<EdgeInertial*> vei(N, (EdgeInertial*)nullptr);
  std::vector<EdgeGyroRW*> vegr(N, (EdgeGyroRW*)nullptr);
  std::vector<EdgeAccRW*> vear(N, (EdgeAccRW*)nullptr);
  for (int i = 0; i < N; i++) {
    // std::cout << "inserting inertial edge " << i << std::endl;
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 =
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        std::cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
             << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
             << std::endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      // TODO Uncomment
      g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
      vei[i]->setRobustKernel(rki);
      rki->setDelta(sqrt(16.92));
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                  .cast<double>()
                                  .inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                  .cast<double>()
                                  .inverse();
      vear[i]->setInformation(InfoA);
      optimizer.addEdge(vear[i]);
    } else
      Verbose::PrintMess("ERROR building inertial edge",
                         Verbose::VERBOSITY_NORMAL);
  }

  Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);

  // Set MapPoint vertices
  const int nExpectedSize =
      (N + Ncov + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // Mono
  std::vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // Stereo
  std::vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);
  const float chi2Mono2 = 5.991;
  const float thHuberStereo = sqrt(7.815);
  const float chi2Stereo2 = 7.815;

  const unsigned long iniMPid = maxKFid * 5;

  for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    if (!pMP) continue;

    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();

    // Create visual constraints
    for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (!pKFi) continue;

      if ((pKFi->mnBALocalForKF != pCurrKF->mnId) &&
          (pKFi->mnBAFixedForKF != pCurrKF->mnId))
        continue;

      if (pKFi->mnId > maxKFid) {
        continue;
      }

      if (optimizer.vertex(id) == nullptr || optimizer.vertex(pKFi->mnId) == nullptr)
        continue;

      if (!pKFi->isBad()) {
        const cv::KeyPoint& kpUn = pKFi->mvKeysUn[std::get<0>(mit->second)];

        if (pKFi->mvuRight[std::get<0>(mit->second)] < 0)  // Monocular observation
        {
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono();
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
          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        } else  // stereo observation
        {
          const float kp_ur = pKFi->mvuRight[std::get<0>(mit->second)];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }
      }
    }
  }

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(8);

  std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Mono2) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Stereo
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Stereo2) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex and erase outliers
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data
  // Keyframes
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  for (int i = 0; i < Ncov; i++) {
    KeyFrame* pKFi = vpOptimizableCovKFs[i];

    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // Points
  for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                 lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}


}  // namespace MORB_SLAM
