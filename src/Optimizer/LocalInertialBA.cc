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

#include "MORB_SLAM/Exceptions.hpp"

namespace MORB_SLAM {
void Optimizer::LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, std::shared_ptr<Map> pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit) {
  std::shared_ptr<Map> pCurrentMap = pKF->GetMap();

  const int maxOpt = bLarge ? 25 : 10;
  const int opt_it = bLarge ? 4 : 10;

  const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
  const unsigned long maxKFid = pKF->mnId;

  std::vector<KeyFrame*> vpOptimizableKFs;
  // const std::vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
  std::list<KeyFrame*> lpOptVisKFs;

  vpOptimizableKFs.reserve(Nd);
  vpOptimizableKFs.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
    } else
      break;
  }

  int N = vpOptimizableKFs.size();
  // std::cout << "LocalInertialBA: N = " << N << std::endl;

  // Optimizable points seen by temporal optimizable keyframes
  std::list<MapPoint*> lLocalMapPoints;
  for (int i = 0; i < N; i++) {
    std::vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP && !pMP->isBad() && pMP->mnBALocalForKF != pKF->mnId) {
        lLocalMapPoints.push_back(pMP);
        pMP->mnBALocalForKF = pKF->mnId;
      }
    }
  }

  // Fixed Keyframe: First frame previous KF to optimization window)
  std::list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // // Optimizable visual KFs
  // const int maxCovKF = 0;
  // for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++) {
  //   if (lpOptVisKFs.size() >= maxCovKF) break;

  //   KeyFrame* pKFi = vpNeighsKFs[i];
  //   if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId) continue;

  //   pKFi->mnBALocalForKF = pKF->mnId;
  //   if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
  //     lpOptVisKFs.push_back(pKFi);

  //     std::vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
  //     for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
  //       MapPoint* pMP = *vit;
  //       if (pMP && !pMP->isBad() && pMP->mnBALocalForKF != pKF->mnId) {
  //         lLocalMapPoints.push_back(pMP);
  //         pMP->mnBALocalForKF = pKF->mnId;
  //       }
  //     }
  //   }
  // }

  // Fixed KFs which are not covisible optimizable
  const int maxFixKF = 200;

  for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    std::map<KeyFrame*, std::tuple<int, int>> observations = (*lit)->GetObservations();
    for (std::map<KeyFrame*, std::tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad()) {
          lFixedKeyFrames.push_back(pKFi);
          break;
        }
      }
    }
    if (lFixedKeyFrames.size() >= maxFixKF) break;
  }

  // bool bNonFixed = (lFixedKeyFrames.size() == 0); // UNUSED

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  if (bLarge) {
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-2);  // to avoid iterating for finding optimal lambda
    optimizer.setAlgorithm(solver);
  } else {
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e0);
    optimizer.setAlgorithm(solver);
  }

  // Set Local temporal KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3*(pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3*(pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3*(pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }
  // Set Local visual KeyFrame vertices
  for (std::list<KeyFrame*>::iterator it = lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it != itEnd; it++) {
    KeyFrame* pKFi = *it;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);
  }

  // Set Fixed KeyFrame vertices
  for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    // This should be done only for the KF just before the temporal window
    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3*(pKFi->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3*(pKFi->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3*(pKFi->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // Create intertial constraints
  std::vector<EdgeInertial*> vei(N, (EdgeInertial*)nullptr);
  std::vector<EdgeGyroRW*> vegr(N, (EdgeGyroRW*)nullptr);
  std::vector<EdgeAccRW*> vear(N, (EdgeAccRW*)nullptr);

  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      // std::cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << std::endl;
      continue;
    }

    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid + 3*(pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid + 3*(pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid + 3*(pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid + 3*(pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid + 3*(pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid + 3*(pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        std::cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2 << std::endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      if (i == N - 1 || bRecInit) {
        // All inertial residuals are included without robust cost function, but
        // not that one linking the last optimizable keyframe inside of the
        // local window and the first fixed keyframe out. The information matrix
        // for this measurement is also downweighted. This is done to avoid
        // accumulating error due to fixing variables.
        g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
        vei[i]->setRobustKernel(rki);
        if (i == N - 1) vei[i]->setInformation(vei[i]->information() * 1e-2);
        rki->setDelta(sqrt(16.92));
      }
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
      vear[i]->setInformation(InfoA);

      optimizer.addEdge(vear[i]);
    } else
      std::cout << "ERROR building inertial edge" << std::endl;
  }

  // Set MapPoint vertices
  const int nExpectedSize = (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

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

  std::map<int, std::pair<int, KeyFrame*>> mVisEdges;
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];
    mVisEdges[pKFi->mnId] = std::pair<int, KeyFrame*>(0, pKFi);
  }
  for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++) {
    mVisEdges[(*lit)->mnId] = std::pair<int, KeyFrame*>(0, *lit);
  }

  for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();

    // Create visual constraints
    for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) continue;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = std::get<0>(mit->second);

        cv::KeyPoint kpUn;

        // Monocular left observation
        if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
          mVisEdges[pKFi->mnId].first++;

          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs);

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        }
        // Stereo observation
        else if (leftIndex != -1) {
          kpUn = pKFi->mvKeysUn[leftIndex];
          mVisEdges[pKFi->mnId].first++;

          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }

        // Monocular right observation
        if (pKFi->mpCamera2) {
          int rightIndex = std::get<1>(mit->second);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;
            mVisEdges[pKFi->mnId].first++;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            EdgeMono* e = new EdgeMono(1);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);

            // Add here uncerteinty
            const float unc2 = pKFi->mpCamera->uncertainty2(obs);

            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
          }
        }
      }
    }
  }

  // std::cout << "Total map points: " << lLocalMapPoints.size() << std::endl;
  bool isBad = false;
  for (auto mit = mVisEdges.begin(), mend = mVisEdges.end(); mit != mend; mit++) {
    // assert(mit->second.first >= 3);
    if(mit->second.first < 3){
      KeyFrame *pKFi = mit->second.second;
      if(pKFi){
        std::cout << "KF " << pKFi->mnId << " is bad, deleting" << std::endl;
        if(pKFi->mnId != pKF->mnId) {
          if(pKFi->SetBadFlag()){
            if(pKFi->mNextKF) {
              if(pKFi->mpImuPreintegrated && pKFi->mNextKF->mpImuPreintegrated)
                pKFi->mNextKF->mpImuPreintegrated->MergePrevious(pKFi->mpImuPreintegrated);
              pKFi->mNextKF->mPrevKF = pKFi->mPrevKF;
            }
            if(pKFi->mPrevKF) { 
              pKFi->mPrevKF->mNextKF = pKFi->mNextKF;
            }
            pKFi->mNextKF = nullptr;
            pKFi->mPrevKF = nullptr;
          } else if (pKFi->mnId == pKFi->GetMap()->GetInitKFid()) {
            throw ResetActiveMapSignal();
          }
        }
        isBad = true;
      }
    }
  }
  if(isBad){
    std::cout << "Found bad keyframes: skipping LocalInertialBA..." << std::endl;
    return;
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();
  optimizer.optimize(opt_it);  // Originally to 2
  float err_end = optimizer.activeRobustChi2();
  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];
    bool bClose = pMP->mTrackDepth < 10.f;

    if (pMP->isBad()) continue;

    if ((e->chi2() > chi2Mono2 && !bClose) || (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
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

  // TODO: Some convergence problems have been detected here
  // that is not a todo
  if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) {
    std::cout << "FAIL LOCAL-INERTIAL BA!!!!" << std::endl;
    return;
  }

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++)
    (*lit)->mnBAFixedForKF = 0;

  // Recover optimized data
  // Local temporal Keyframes
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;

    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + 3*(pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid + 3*(pKFi->mnId) + 2));
      VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid + 3*(pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // Local visual KeyFrame
  for (std::list<KeyFrame*>::iterator it = lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it != itEnd; it++) {
    KeyFrame* pKFi = *it;
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;
  }

  // Points
  for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + iniMPid + 1));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

}  // namespace MORB_SLAM
