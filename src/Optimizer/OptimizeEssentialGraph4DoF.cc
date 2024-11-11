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

#include "MORB_SLAM/InertialOdometry/InertialOptimizer.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <complex>
#include <mutex>
#include <unsupported/Eigen/MatrixFunctions>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/G2oTypes.h"
#include "MORB_SLAM/OptimizableTypes.h"

namespace MORB_SLAM {


void InertialOptimizer::OptimizeEssentialGraph4DoF(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF, const LoopClosing::KeyFrameAndPose& NonCorrectedSim3, const LoopClosing::KeyFrameAndPose& CorrectedSim3, const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>& LoopConnections) {
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  const std::vector<std::shared_ptr<MapPoint>> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);

  const int minFeat = 100;
  // Set KeyFrame vertices
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];
    if (pKF->isBad()) continue;

    VertexPose4DoF* V4DoF;

    const int nIDi = pKF->mnId;

    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      const g2o::Sim3 Swc = it->second.inverse();
      Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
      Eigen::Vector3d twc = Swc.translation();
      V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
    } else {
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

      vScw[nIDi] = Siw;
      V4DoF = new VertexPose4DoF(pKF);
    }

    if (pKF == pLoopKF) V4DoF->setFixed(true);

    V4DoF->setId(nIDi);
    V4DoF->setMarginalized(false);

    optimizer.addVertex(V4DoF);
  }
  std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // Edge used in posegraph has still 6Dof, even if updates of camera poses are just in 4DoF
  Eigen::Matrix<double, 6, 6> matLambda = Eigen::Matrix<double, 6, 6>::Identity();
  matLambda(0, 0) = 1e3;
  matLambda(1, 1) = 1e3;

  // Set Loop edges
  for (std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++) {
    std::shared_ptr<KeyFrame> pKF = mit->first;
    const long unsigned int nIDi = pKF->mnId;
    const std::set<std::shared_ptr<KeyFrame>>& spConnections = mit->second;
    const g2o::Sim3 Siw = vScw[nIDi];

    for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++) {
      const long unsigned int nIDj = (*sit)->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sij = Siw * Sjw.inverse();
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));

      e->information() = matLambda;
      optimizer.addEdge(e);

      sInsertedEdges.insert(std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
    }
  }

  // 1. Set normal edges
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];

    const int nIDi = pKF->mnId;

    g2o::Sim3 Siw;

    // Use noncorrected poses for posegraph edges
    LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Siw = iti->second;
    else
      Siw = vScw[nIDi];

    // 1.1.0 Spanning tree edge
    std::shared_ptr<KeyFrame> pParentKF = nullptr;
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.1.1 Inertial edges
    std::shared_ptr<KeyFrame> prevKF = pKF->mPrevKF;
    if (prevKF) {
      int nIDj = prevKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(prevKF);

      if (itj != NonCorrectedSim3.end())
        Swj = (itj->second).inverse();
      else
        Swj = vScw[nIDj].inverse();

      g2o::Sim3 Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.2 Loop edges
    const std::set<std::shared_ptr<KeyFrame>> sLoopEdges = pKF->GetLoopEdges();
    for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
      std::shared_ptr<KeyFrame> pLKF = *sit;
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Swl;

        LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Swl = itl->second.inverse();
        else
          Swl = vScw[pLKF->mnId].inverse();

        g2o::Sim3 Sil = Siw * Swl;
        Eigen::Matrix4d Til;
        Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
        Til.block<3, 1>(0, 3) = Sil.translation();
        Til(3, 3) = 1.;

        Edge4DoF* e = new Edge4DoF(Til);
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
        e->information() = matLambda;
        optimizer.addEdge(e);
      }
    }

    // 1.3 Covisibility graph edges
    const std::vector<std::shared_ptr<KeyFrame>> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
    for (std::vector<std::shared_ptr<KeyFrame>>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
      std::shared_ptr<KeyFrame> pKFn = *vit;
      if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(std::make_pair(std::min(pKF->mnId, pKFn->mnId), std::max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Swn;

          LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Swn = itn->second.inverse();
          else
            Swn = vScw[pKFn->mnId].inverse();

          g2o::Sim3 Sin = Siw * Swn;
          Eigen::Matrix4d Tin;
          Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
          Tin.block<3, 1>(0, 3) = Sin.translation();
          Tin(3, 3) = 1.;
          Edge4DoF* e = new Edge4DoF(Tin);
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
          e->information() = matLambda;
          optimizer.addEdge(e);
        }
      }
    }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);

  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];

    const int nIDi = pKFi->mnId;

    VertexPose4DoF* Vi = static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
    Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
    Eigen::Vector3d ti = Vi->estimate().tcw[0];

    g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    std::shared_ptr<MapPoint> pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    int nIDr;

    if(std::shared_ptr<KeyFrame> pRefKF = (pMP->GetReferenceKeyFrame()).lock()) {
      nIDr = pRefKF->mnId;

      g2o::Sim3 Srw = vScw[nIDr];
      g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

      Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
      Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
      pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

      pMP->UpdateNormalAndDepth();      
    }
  }
  pMap->IncreaseChangeIndex();
}

}  // namespace MORB_SLAM
