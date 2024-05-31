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

void Optimizer::GlobalBundleAdjustemnt(std::shared_ptr<Map> pMap, int nIterations,
                                       bool* pbStopFlag,
                                       const unsigned long nLoopKF,
                                       const bool bRobust) {
  std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  std::vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
  BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const std::vector<std::shared_ptr<KeyFrame>>& vpKFs,
                                 const std::vector<MapPoint*>& vpMP, int nIterations,
                                 bool* pbStopFlag, const unsigned long nLoopKF,
                                 const bool bRobust) {
  std::vector<bool> vbNotIncludedMP;
  vbNotIncludedMP.resize(vpMP.size());

  std::shared_ptr<Map> pMap = vpKFs[0]->GetMap();

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

  const int nExpectedSize = (vpKFs.size()) * vpMP.size();

  std::vector<MORB_SLAM::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<MORB_SLAM::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<std::shared_ptr<KeyFrame>> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  // Set KeyFrame vertices

  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKF->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(pKF->mnId);
    vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKF->mnId > maxKFid) maxKFid = pKF->mnId;
  }

  const float thHuber2D = sqrt(5.99);
  const float thHuber3D = sqrt(7.815);

  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    MapPoint* pMP = vpMP[i];
    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    const int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>> observations = pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (std::map<std::shared_ptr<KeyFrame>, std::tuple<int, int>>::const_iterator mit =
             observations.begin();
         mit != observations.end(); mit++) {
      std::shared_ptr<KeyFrame> pKF = mit->first;
      if (pKF->isBad() || pKF->mnId > maxKFid) continue;
      if (optimizer.vertex(id) == nullptr || optimizer.vertex(pKF->mnId) == nullptr)
        continue;
      nEdges++;

      const int leftIndex = std::get<0>(mit->second);

      if (leftIndex != -1 && pKF->mvuRight[std::get<0>(mit->second)] < 0) {
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

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

        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);
        }

        e->pCamera = pKF->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMP);
      } else if (leftIndex != -1 &&
                 pKF->mvuRight[leftIndex] >= 0)  // Stereo observation
      {
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

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

        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber3D);
        }

        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;
        e->bf = pKF->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMP);
      }

      if (pKF->mpCamera2) {
        int rightIndex = std::get<1>(mit->second);

        if (rightIndex != -1 && rightIndex < static_cast<int>(pKF->mvKeysRight.size())) {
          rightIndex -= pKF->NLeft;

          Eigen::Matrix<double, 2, 1> obs;
          cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
          obs << kp.pt.x, kp.pt.y;

          MORB_SLAM::EdgeSE3ProjectXYZToBody* e =
              new MORB_SLAM::EdgeSE3ProjectXYZToBody();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKF->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);

          Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
          e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                 Trl.translation().cast<double>());

          e->pCamera = pKF->mpCamera2;

          optimizer.addEdge(e);
          vpEdgesBody.push_back(e);
          vpEdgeKFBody.push_back(pKF);
          vpMapPointEdgeBody.push_back(pMP);
        }
      }
    }

    if (nEdges == 0) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    } else {
      vbNotIncludedMP[i] = false;
    }
  }

  // Optimize!
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(nIterations);
  Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

  // Recover optimized data
  // Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));

    g2o::SE3Quat SE3quat = vSE3->estimate();
    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(),
                                SE3quat.translation().cast<float>()));
    } else {
      pKF->mTcwGBA =
          Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
      pKF->mnBAGlobalForKF = nLoopKF;

      Sophus::SE3f mTwc = pKF->GetPoseInverse();
      Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
      Eigen::Vector3f vector_dist = mTcGBA_c.translation();
      double dist = vector_dist.norm();
      if (dist > 1) {
        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        std::vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;

        for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++) {
          MORB_SLAM::EdgeSE3ProjectXYZ* e = vpEdgesMono[i2];
          MapPoint* pMP = vpMapPointEdgeMono[i2];
          std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) {
            continue;
          }

          if (pMP->isBad()) continue;

          if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            numMonoBadPoints++;

          } else {
            numMonoOptPoints++;
            vpMonoMPsOpt.push_back(pMP);
          }
        }

        for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++) {
          g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i2];
          MapPoint* pMP = vpMapPointEdgeStereo[i2];
          std::shared_ptr<KeyFrame> pKFedge = vpEdgeKFMono[i2];

          if (pKF != pKFedge) {
            continue;
          }

          if (pMP->isBad()) continue;

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            numStereoBadPoints++;
          } else {
            numStereoOptPoints++;
            vpStereoMPsOpt.push_back(pMP);
          }
        }
      }
    }
  }

  // Points
  for (size_t i = 0; i < vpMP.size(); i++) {
    if (vbNotIncludedMP[i]) continue;

    MapPoint* pMP = vpMP[i];

    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));

    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
}

}  // namespace MORB_SLAM
