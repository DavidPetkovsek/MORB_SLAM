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

void Optimizer::GlobalBundleAdjustemnt(std::shared_ptr<Map> pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust) {
  std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  std::vector<std::shared_ptr<MapPoint>> vpMP = pMap->GetAllMapPoints();

  std::vector<bool> vbNotIncludedMP;
  vbNotIncludedMP.resize(vpMP.size());

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;

  // Set KeyFrame vertices
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKF->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
    vSE3->setId(pKF->mnId);
    vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (pKF->mnId > maxKFid) maxKFid = pKF->mnId;
  }

  const float thHuber2D = sqrt(5.99);
  const float thHuber3D = sqrt(7.815);

  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    std::shared_ptr<MapPoint> pMP = vpMP[i];
    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    const int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>> observations = pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (std::map<std::weak_ptr<KeyFrame>, std::tuple<int, int>, std::owner_less<>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {
      if(std::shared_ptr<KeyFrame> pKF = (mit->first).lock()) {
        if (pKF->isBad() || pKF->mnId > maxKFid || optimizer.vertex(id) == nullptr || optimizer.vertex(pKF->mnId) == nullptr)
          continue;
        nEdges++;

        const int leftIndex = std::get<0>(mit->second);

        if (leftIndex != -1 && pKF->mvuRight[std::get<0>(mit->second)] < 0) {
          const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          MORB_SLAM::EdgeSE3ProjectXYZ* e = new MORB_SLAM::EdgeSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
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

        // Stereo observation
        } else if (leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) {
          const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKF->mvuRight[std::get<0>(mit->second)];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
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
        }

        if (pKF->mpCamera2) {
          int rightIndex = std::get<1>(mit->second);

          if (rightIndex != -1 && rightIndex < static_cast<int>(pKF->mvKeysRight.size())) {
            rightIndex -= pKF->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            MORB_SLAM::EdgeSE3ProjectXYZToBody* e = new MORB_SLAM::EdgeSE3ProjectXYZToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);

            Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
            e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

            e->pCamera = pKF->mpCamera2;

            optimizer.addEdge(e);
          }
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
  // KeyFrames
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));

    g2o::SE3Quat SE3quat = vSE3->estimate();
    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
    } else {
      pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
      pKF->mnBAGlobalForKF = nLoopKF;
    }
  }

  // Points
  for (size_t i = 0; i < vpMP.size(); i++) {
    if (vbNotIncludedMP[i]) continue;

    std::shared_ptr<MapPoint> pMP = vpMP[i];

    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));

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
