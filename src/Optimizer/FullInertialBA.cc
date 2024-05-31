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

void Optimizer::FullInertialBA(std::shared_ptr<Map> pMap, int its, const bool bFixLocal,
                               const long unsigned int nLoopId,
                               bool* pbStopFlag, bool bInit, ImuInitializater::ImuInitType priorG,
                               ImuInitializater::ImuInitType priorA, Eigen::VectorXd* vSingVal,
                               bool* bHess) {
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  const std::vector<std::shared_ptr<MapPoint>> vpMPs = pMap->GetAllMapPoints();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1e-5);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  int nNonFixed = 0;

  // Set KeyFrame vertices
  std::shared_ptr<KeyFrame> pIncKF = nullptr;
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    pIncKF = pKFi;
    bool bFixed = false;
    if (bFixLocal) {
      bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) ||
               (pKFi->mnBAFixedForKF >= (maxKFid - 1));
      if (!bFixed) nNonFixed++;
      VP->setFixed(bFixed);
    }
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(bFixed);
      optimizer.addVertex(VV);
      if (!bInit && pIncKF != nullptr) {
        VertexGyroBias* VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
        VG->setFixed(bFixed);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pKFi);
        VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
        VA->setFixed(bFixed);
        optimizer.addVertex(VA);
      }
    }
  }

  if (bInit) {
    VertexGyroBias* VG = new VertexGyroBias(pIncKF);
    VG->setId(4 * maxKFid + 2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pIncKF);
    VA->setId(4 * maxKFid + 3);
    VA->setFixed(false);
    optimizer.addVertex(VA);
  }

  if (bFixLocal) {
    if (nNonFixed < 3) return;
  }

  // IMU links
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];

    if (!pKFi->mPrevKF) {
      if(pMap->GetOriginKF() != pKFi)
        Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!", Verbose::VERBOSITY_NORMAL);
      continue;
    }

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
      if (pKFi->bImu && pKFi->mPrevKF->bImu) {
        pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
        g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
        g2o::HyperGraph::Vertex* VV1 =
            optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);

        g2o::HyperGraph::Vertex* VG1;
        g2o::HyperGraph::Vertex* VA1;
        g2o::HyperGraph::Vertex* VG2 = nullptr;
        g2o::HyperGraph::Vertex* VA2 = nullptr;
        if (!bInit) {
          VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
          VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
          VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
          VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
        } else {
          VG1 = optimizer.vertex(4 * maxKFid + 2);
          VA1 = optimizer.vertex(4 * maxKFid + 3);
        }

        g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
        g2o::HyperGraph::Vertex* VV2 =
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

        if (!bInit) {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
            std::cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
                 << std::endl;
            continue;
          }
        } else {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
            std::cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << std::endl;
            continue;
          }
        }

        EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
        ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
        ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
        ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
        ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
        ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
        ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

        g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
        ei->setRobustKernel(rki);
        rki->setDelta(sqrt(16.92));

        optimizer.addEdge(ei);

        if (!bInit) {
          EdgeGyroRW* egr = new EdgeGyroRW();
          egr->setVertex(0, VG1);
          egr->setVertex(1, VG2);
          Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                      .cast<double>()
                                      .inverse();
          egr->setInformation(InfoG);
          egr->computeError();
          optimizer.addEdge(egr);

          EdgeAccRW* ear = new EdgeAccRW();
          ear->setVertex(0, VA1);
          ear->setVertex(1, VA2);
          Eigen::Matrix3d InfoA =
              pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                  .cast<double>()
                  .inverse();
          ear->setInformation(InfoA);
          ear->computeError();
          optimizer.addEdge(ear);
        }
      } else
        std::cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << std::endl;
    }
  }
  if (bInit) {
    g2o::HyperGraph::Vertex* VG = optimizer.vertex(4 * maxKFid + 2);
    g2o::HyperGraph::Vertex* VA = optimizer.vertex(4 * maxKFid + 3);

    // Add prior to common biases
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;  //
    epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);

    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;  //
    epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);
  }

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  const unsigned long iniMPid = maxKFid * 5;

  std::vector<bool> vbNotIncludedMP(vpMPs.size(), false);

  for (size_t i = 0; i < vpMPs.size(); i++) {
    std::shared_ptr<MapPoint> pMP = vpMPs[i];
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>> observations = pMP->GetObservations();

    bool bAllFixed = true;

    // Set edges
    for (std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>>::const_iterator
             mit = observations.begin(),
             mend = observations.end();
         mit != mend; mit++) {
      std::shared_ptr<KeyFrame> pKFi = mit->first;

      if (pKFi->mnId > maxKFid){
        continue;
      }

      if (!pKFi->isBad()) {
        const int leftIndex = std::get<0>(mit->second);
        cv::KeyPoint kpUn;

        if (leftIndex != -1 && pKFi->mvuRight[std::get<0>(mit->second)] < 0)  // Monocular observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          g2o::OptimizableGraph::Vertex* VP =
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                  optimizer.vertex(pKFi->mnId));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
        } else if (leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0)  // stereo observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          g2o::OptimizableGraph::Vertex* VP =
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                  optimizer.vertex(pKFi->mnId));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
        }

        if (pKFi->mpCamera2) {  // Monocular right observation
          int rightIndex = std::get<1>(mit->second);

          if (rightIndex != -1 && rightIndex < static_cast<int>(pKFi->mvKeysRight.size())) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            kpUn = pKFi->mvKeysRight[rightIndex];
            obs << kpUn.pt.x, kpUn.pt.y;

            EdgeMono* e = new EdgeMono(1);

            g2o::OptimizableGraph::Vertex* VP =
                dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                    optimizer.vertex(pKFi->mnId));
            if (bAllFixed)
              if (!VP->fixed()) bAllFixed = false;

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(id)));
            e->setVertex(1, VP);
            e->setMeasurement(obs);
            const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
          }
        }
      }
    }
    if (bAllFixed) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // Recover optimized data
  // Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    if (nLoopId == 0) {
      Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                       VP->estimate().tcw[0].cast<float>());
      pKFi->SetPose(Tcw);
    } else {
      pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),
                                   VP->estimate().tcw[0].cast<float>());
      pKFi->mnBAGlobalForKF = nLoopId;
    }
    if (pKFi->bImu) {
      VertexVelocity* VV = static_cast<VertexVelocity*>(
          optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      if (nLoopId == 0) {
        pKFi->SetVelocity(VV->estimate().cast<float>());
      } else {
        pKFi->mVwbGBA = VV->estimate().cast<float>();
      }

      VertexGyroBias* VG;
      VertexAccBias* VA;
      if (!bInit) {
        VG = static_cast<VertexGyroBias*>(
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
        VA = static_cast<VertexAccBias*>(
            optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      } else {
        VG = static_cast<VertexGyroBias*>(optimizer.vertex(4 * maxKFid + 2));
        VA = static_cast<VertexAccBias*>(optimizer.vertex(4 * maxKFid + 3));
      }

      Vector6d vb;
      vb << VG->estimate(), VA->estimate();
      IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
      if (nLoopId == 0) {
        pKFi->SetNewBias(b);
      } else {
        pKFi->mBiasGBA = b;
      }
    }
  }

  // Points
  for (size_t i = 0; i < vpMPs.size(); i++) {
    if (vbNotIncludedMP[i]) continue;

    std::shared_ptr<MapPoint> pMP = vpMPs[i];
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + iniMPid + 1));

    if (nLoopId == 0) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopId;
    }
  }

  pMap->IncreaseChangeIndex();
}


}  // namespace MORB_SLAM
