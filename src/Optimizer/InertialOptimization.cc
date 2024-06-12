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

  
// used in LocalMapping::InitializeIMU()
void Optimizer::InertialOptimization(std::shared_ptr<Map> pMap, Eigen::Matrix3d& Rwg,
                                     double& scale, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, bool bMono,
                                     bool bFixedVel, bool bGauss, 
                                     ImuInitializater::ImuInitType priorG,
                                     ImuInitializater::ImuInitType priorA) {
  Verbose::PrintMess("start inertial optimization", Verbose::VERBOSITY_NORMAL);
  int its = 200;
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  if (priorG != ImuInitializater::ImuInitType::VIBA2_G) solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (fixed poses and optimizable velocities)
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + (pKFi->mnId) + 1);
    if (bFixedVel)
      VV->setFixed(true);
    else
      VV->setFixed(false);

    optimizer.addVertex(VV);
  }

  // Biases
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  if (bFixedVel)
    VG->setFixed(true);
  else
    VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  if (bFixedVel)
    VA->setFixed(true);
  else
    VA->setFixed(false);

  optimizer.addVertex(VA);
  // prior acc bias
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  double infoPriorA = priorA;
  epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);
  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  double infoPriorG = priorG;
  epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(scale);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(!bMono);  // Fixed for stereo case
  optimizer.addVertex(VS);

  // Graph edges
  // IMU links with gravity and scale
  std::vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  std::vector<std::pair<std::shared_ptr<KeyFrame>, std::shared_ptr<KeyFrame>>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());
  // std::cout << "build optimization graph" << std::endl;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
      if (!pKFi->mpImuPreintegrated) {
        std::cout << "Not preintegrated measurement" << std::endl;
        continue;
      }

      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        std::cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA
             << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS
             << std::endl;

        continue;
      }
      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);

      vppUsedKF.push_back(std::make_pair(pKFi->mPrevKF, pKFi));
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  scale = VS->estimate();

  // Recover optimized data
  // Biases
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();
  scale = VS->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
  Rwg = VGDir->estimate().Rwg;

  // Keyframes velocities and biases
  const size_t N = vpKFs.size();
  for (size_t i = 0; i < N; i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;

    VertexVelocity* VV = static_cast<VertexVelocity*>(
        optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate();  // Velocity is scaled after
    pKFi->SetVelocity(Vw.cast<float>());

    if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      pKFi->SetNewBias(b);
      if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
    } else
      pKFi->SetNewBias(b);
  }
  Verbose::PrintMess("end inertial optimization", Verbose::VERBOSITY_NORMAL);
}

// used in LoopClosing
void Optimizer::InertialOptimization(std::shared_ptr<Map> pMap, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, ImuInitializater::ImuInitType priorG,
                                     ImuInitializater::ImuInitType priorA) {
  int its = 200;  // Check number of iterations
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (fixed poses and optimizable velocities)
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + (pKFi->mnId) + 1);
    VV->setFixed(false);

    optimizer.addVertex(VV);
  }

  // Biases
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  VG->setFixed(false);
  optimizer.addVertex(VG);

  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  VA->setFixed(false);

  optimizer.addVertex(VA);
  // prior acc bias
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  double infoPriorA = priorA;
  epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);
  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  double infoPriorG = priorG;
  epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(true);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(1.0);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(true);  // Fixed since scale is obtained from already well initialized map
  optimizer.addVertex(VS);

  // Graph edges
  // IMU links with gravity and scale
  std::vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  std::vector<std::pair<std::shared_ptr<KeyFrame>, std::shared_ptr<KeyFrame>>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());

  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        std::cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA
             << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS
             << std::endl;

        continue;
      }
      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);

      vppUsedKF.push_back(std::make_pair(pKFi->mPrevKF, pKFi));
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // Recover optimized data
  // Biases
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

  // Keyframes velocities and biases
  const size_t N = vpKFs.size();
  for (size_t i = 0; i < N; i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;

    VertexVelocity* VV = static_cast<VertexVelocity*>(
        optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate();
    pKFi->SetVelocity(Vw.cast<float>());

    if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      pKFi->SetNewBias(b);
      if (pKFi->mpImuPreintegrated) pKFi->mpImuPreintegrated->Reintegrate();
    } else
      pKFi->SetNewBias(b);
  }
}

// used in LocalMapping::ScaleRefinement()
void Optimizer::InertialOptimization(std::shared_ptr<Map> pMap, Eigen::Matrix3d& Rwg,
                                     double& scale) {
  int its = 10;
  long unsigned int maxKFid = pMap->GetMaxKFid();
  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (all variables are fixed)
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
    if (pKFi->mnId > maxKFid) continue;
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(pKFi);
    VV->setId(maxKFid + 1 + (pKFi->mnId));
    VV->setFixed(true);
    optimizer.addVertex(VV);

    // Vertex of fixed biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
    VG->setFixed(true);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
    VA->setFixed(true);
    optimizer.addVertex(VA);
  }

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(4 * (maxKFid + 1));
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(scale);
  VS->setId(4 * (maxKFid + 1) + 1);
  VS->setFixed(false);
  optimizer.addVertex(VS);

  // Graph edges
  int count_edges = 0;
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;

      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex((maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex((maxKFid + 1) + pKFi->mnId);
      g2o::HyperGraph::Vertex* VG =
          optimizer.vertex(2 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VA =
          optimizer.vertex(3 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(4 * (maxKFid + 1));
      g2o::HyperGraph::Vertex* VS = optimizer.vertex(4 * (maxKFid + 1) + 1);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        Verbose::PrintMess(
            "Error" + std::to_string(VP1->id()) + ", " + std::to_string(VV1->id()) +
                ", " + std::to_string(VG->id()) + ", " + std::to_string(VA->id()) + ", " +
                std::to_string(VP2->id()) + ", " + std::to_string(VV2->id()) + ", " +
                std::to_string(VGDir->id()) + ", " + std::to_string(VS->id()),
            Verbose::VERBOSITY_NORMAL);

        continue;
      }
      count_edges++;
      EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      ei->setRobustKernel(rk);
      rk->setDelta(1.f);
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  /*float err = */optimizer.activeRobustChi2();
  optimizer.optimize(its);
  optimizer.computeActiveErrors();
  /*float err_end = */optimizer.activeRobustChi2();
  // Recover optimized data
  scale = VS->estimate();
  Rwg = VGDir->estimate().Rwg;
}


int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  std::vector<EdgeMonoOnlyPose*> vpEdgesMono;
  std::vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  std::vector<size_t> vnIndexEdgeMono;
  std::vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      std::shared_ptr<MapPoint> pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;

        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          // if(Nleft == -1) continue;
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos()); 

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) { // else ?!
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }
  
  nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;
  // std::cout << "KeyFrame: " << nInitialMonoCorrespondences << ", " << nInitialStereoCorrespondences << std::endl;
  // if(nInitialCorrespondences < 5) {
  //   vpEdgesMono.clear();
  //   vpEdgesStereo.clear();
  //   vnIndexEdgeMono.clear();
  //   vnIndexEdgeStereo.clear();
  //   nInitialCorrespondences = 0;
  //   nInitialMonoCorrespondences = 0;
  //   nInitialStereoCorrespondences = 0;
  //   std::cout << "n too low!" << std::endl;
  // } else {
  //   for(int i = 0; i < nInitialMonoCorrespondences; i++){
  //     optimizer.addEdge(vpEdgesMono[i]);
  //   }
  //   for(int i = 0; i < nInitialStereoCorrespondences; i++){
  //     optimizer.addEdge(vpEdgesStereo[i]);
  //   }
  // }

  // Set KeyFrame Vertex
  std::shared_ptr<KeyFrame> pKF = pFrame->mpLastKeyFrame;

  VertexPose* VPk = new VertexPose(pKF);
  VPk->setId(4);
  VPk->setFixed(true);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pKF);
  VVk->setId(5);
  VVk->setFixed(true);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pKF);
  VGk->setId(6);
  VGk->setFixed(true);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pKF);
  VAk->setId(7);
  VAk->setFixed(true);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  // We perform 4 optimizations, after each optimization we classify observation
  // as inlier/outlier At the next optimization, outliers are not included, but
  // at the end they can be classified as inliers again.
  float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
  float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};
  int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    // For monocular observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    // For stereo observations
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);  // not included in next optimization
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // If not too much tracks, recover not too bad points
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize keyFframe states and generate new prior for
  // frame
  Eigen::Matrix<double, 15, 15> H;
  H.setZero();

  H.block<9, 9>(0, 0) += ei->GetHessian2();
  H.block<3, 3>(9, 9) += egr->GetHessian2();
  H.block<3, 3>(12, 12) += ear->GetHessian2();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  pFrame->mpcpi =
      std::make_shared<ConstraintPoseImu>(VP->estimate().Rwb, VP->estimate().twb,
                            VV->estimate(), VG->estimate(), VA->estimate(), H);

  return nInitialCorrespondences - nBad;
}

static std::shared_ptr<ConstraintPoseImu> oldMpcpi = nullptr;

int Optimizer::PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  std::vector<EdgeMonoOnlyPose*> vpEdgesMono;
  std::vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  std::vector<size_t> vnIndexEdgeMono;
  std::vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  {
    std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      std::shared_ptr<MapPoint> pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          // if(Nleft == -1) continue;
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;
  // std::cout << "Frame: " << nInitialMonoCorrespondences << ", " << nInitialStereoCorrespondences << std::endl;
  // if(nInitialCorrespondences < 5) {
  //   vpEdgesMono.clear();
  //   vpEdgesStereo.clear();
  //   vnIndexEdgeMono.clear();
  //   vnIndexEdgeStereo.clear();
  //   nInitialCorrespondences = 0;
  //   nInitialMonoCorrespondences = 0;
  //   nInitialStereoCorrespondences = 0;
  //   std::cout << "n too low!" << std::endl;
  // } else {
  //   for(int i = 0; i < nInitialMonoCorrespondences; i++){
  //     optimizer.addEdge(vpEdgesMono[i]);
  //   }
  //   for(int i = 0; i < nInitialStereoCorrespondences; i++){
  //     optimizer.addEdge(vpEdgesStereo[i]);
  //   }
  // }

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;
  if(!pFp || pFp->isPartiallyConstructed) {
    std::cout << "no prev frame in PIOLF" << std::endl;
    return 0;
  }

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (pFp->mpcpi == nullptr){
    std::cout << "NO MPCPI" << std::endl;
    return 0; // nInitialCorrespondences
  }
  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify observation
  // as inlier/outlier At the next optimization, outliers are not included, but
  // at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    // For monocular observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    // For stereo observations
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // If not too much tracks, recover not too bad points
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut) {
        pFrame->mvbOutlier[idx] = false;
        nInliers++;
      } else {
        nBad++;
      }
    }

    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut) {
        pFrame->mvbOutlier[idx] = false;
        nInliers++;
      } else {
        nBad++;
      }
    }
  }


  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize previous frame states and generate new prior
  // for frame
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += ei->GetHessian();

  Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += ep->GetHessian();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  H = Marginalize(H, 0, 14);

  pFrame->mpcpi = std::make_shared<ConstraintPoseImu>(
      VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
      VA->estimate(), H.block<15, 15>(15, 15));
  if (oldMpcpi == pFp->mpcpi) {
    std::cerr << "\033[22;34mSAME MPCPI\033[0m" << std::endl;
  } else {
    oldMpcpi = pFp->mpcpi;
    pFp->mpcpi = nullptr;
  }
  return nInitialCorrespondences - nBad;
}


}  // namespace MORB_SLAM
