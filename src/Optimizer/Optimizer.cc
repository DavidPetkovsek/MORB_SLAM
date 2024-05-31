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



int Optimizer::PoseOptimization(Frame* pFrame) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  Sophus::SE3<float> Tcw = pFrame->GetPose();
  vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                 Tcw.translation().cast<double>()));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // Set MapPoint vertices
  const int N = pFrame->N;

  std::vector<MORB_SLAM::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
  std::vector<MORB_SLAM::EdgeSE3ProjectXYZOnlyPoseToBody*> vpEdgesMono_FHR;
  std::vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
  vpEdgesMono.reserve(N);
  vpEdgesMono_FHR.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeRight.reserve(N);

  std::vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
  std::vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);
  const float deltaStereo = sqrt(7.815);

  {
    std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      std::shared_ptr<MapPoint> pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        // Conventional SLAM
        if (!pFrame->mpCamera2) {
          // Monocular observation
          if (pFrame->mvuRight[i] < 0) {
            continue;
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            MORB_SLAM::EdgeSE3ProjectXYZOnlyPose* e =
                new MORB_SLAM::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else  // Stereo observation
          {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 3, 1> obs;
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            const float& kp_ur = pFrame->mvuRight[i];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e =
                new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            e->fx = pFrame->fx;
            e->fy = pFrame->fy;
            e->cx = pFrame->cx;
            e->cy = pFrame->cy;
            e->bf = pFrame->mbf;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
          }
        }
        // SLAM with respect a rigid body
        else {
          nInitialCorrespondences++;

          cv::KeyPoint kpUn;

          if (i < pFrame->Nleft) {  // Left camera observation
            kpUn = pFrame->mvKeys[i];

            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            MORB_SLAM::EdgeSE3ProjectXYZOnlyPose* e =
                new MORB_SLAM::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else {
            kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            pFrame->mvbOutlier[i] = false;

            MORB_SLAM::EdgeSE3ProjectXYZOnlyPoseToBody* e =
                new MORB_SLAM::EdgeSE3ProjectXYZOnlyPoseToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera2;
            e->Xw = pMP->GetWorldPos().cast<double>();

            e->mTrl = g2o::SE3Quat(
                pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(),
                pFrame->GetRelativePoseTrl().translation().cast<double>());

            optimizer.addEdge(e);

            vpEdgesMono_FHR.push_back(e);
            vnIndexEdgeRight.push_back(i);
          }
        }
      }
    }
  }

  if (nInitialCorrespondences < 3) return 0;

  // We perform 4 optimizations, after each optimization we classify observation
  // as inlier/outlier At the next optimization, outliers are not included, but
  // at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  for (size_t it = 0; it < 4; it++) {
    Tcw = pFrame->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));

    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      MORB_SLAM::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
      MORB_SLAM::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

      const size_t idx = vnIndexEdgeRight[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        e->setLevel(0);
        pFrame->mvbOutlier[idx] = false;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    if (optimizer.edges().size() < 10) break;
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov =
      static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                          SE3quat_recov.translation().cast<float>());
  pFrame->SetPose(pose);

  return nInitialCorrespondences - nBad;
}

int Optimizer::OptimizeSim3(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame> pKF2,
                            std::vector<std::shared_ptr<MapPoint>>& vpMatches1, g2o::Sim3& g2oS12,
                            const float th2, const bool bFixScale,
                            Eigen::Matrix<double, 7, 7>& mAcumHessian,
                            const bool bAllPoints) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Camera poses
  const Eigen::Matrix3f R1w = pKF1->GetRotation();
  const Eigen::Vector3f t1w = pKF1->GetTranslation();
  const Eigen::Matrix3f R2w = pKF2->GetRotation();
  const Eigen::Vector3f t2w = pKF2->GetTranslation();

  // Set Sim3 vertex
  MORB_SLAM::VertexSim3Expmap* vSim3 = new MORB_SLAM::VertexSim3Expmap();
  vSim3->_fix_scale = bFixScale;
  vSim3->setEstimate(g2oS12);
  vSim3->setId(0);
  vSim3->setFixed(false);
  vSim3->pCamera1 = pKF1->mpCamera;
  vSim3->pCamera2 = pKF2->mpCamera;
  optimizer.addVertex(vSim3);

  // Set MapPoint vertices
  const int N = vpMatches1.size();
  const std::vector<std::shared_ptr<MapPoint>> vpMapPoints1 = pKF1->GetMapPointMatches();
  std::vector<MORB_SLAM::EdgeSim3ProjectXYZ*> vpEdges12;
  std::vector<MORB_SLAM::EdgeInverseSim3ProjectXYZ*> vpEdges21;
  std::vector<size_t> vnIndexEdge;
  std::vector<bool> vbIsInKF2;

  vnIndexEdge.reserve(2 * N);
  vpEdges12.reserve(2 * N);
  vpEdges21.reserve(2 * N);
  vbIsInKF2.reserve(2 * N);

  const float deltaHuber = sqrt(th2);

  int nCorrespondences = 0;
  int nBadMPs = 0;
  int nInKF2 = 0;
  int nOutKF2 = 0;
  int nMatchWithoutMP = 0;

  std::vector<int> vIdsOnlyInKF2;

  for (int i = 0; i < N; i++) {
    if (!vpMatches1[i]) continue;

    std::shared_ptr<MapPoint> pMP1 = vpMapPoints1[i];
    std::shared_ptr<MapPoint> pMP2 = vpMatches1[i];

    const int id1 = 2 * i + 1;
    const int id2 = 2 * (i + 1);

    const int i2 = std::get<0>(pMP2->GetIndexInKeyFrame(pKF2));

    Eigen::Vector3f P3D1c;
    Eigen::Vector3f P3D2c;

    if (pMP1 && pMP2) {
      if (!pMP1->isBad() && !pMP2->isBad()) {
        g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
        P3D1c = R1w * P3D1w + t1w;
        vPoint1->setEstimate(P3D1c.cast<double>());
        vPoint1->setId(id1);
        vPoint1->setFixed(true);
        optimizer.addVertex(vPoint1);

        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);
      } else {
        nBadMPs++;
        continue;
      }
    } else {
      nMatchWithoutMP++;

      // TODO The 3D position in KF1 doesn't exist
      if (!pMP2->isBad()) {
        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
        P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);

        vIdsOnlyInKF2.push_back(id2);
      }
      continue;
    }

    if (i2 < 0 && !bAllPoints) {
      Verbose::PrintMess("Remove point -> i2: " + std::to_string(i2) + "; bAllPoints: " + std::to_string(bAllPoints), Verbose::VERBOSITY_DEBUG);
      continue;
    }

    // the direction the camera is facing defines the positive Z direction. Therefore it should not be able to see MapPoints that are behind it
    if (P3D2c(2) <= 0) {
      Verbose::PrintMess("Sim3: Z coordinate is negative", Verbose::VERBOSITY_DEBUG);
      continue;
    }

    nCorrespondences++;

    // Set edge x1 = S12*X2
    Eigen::Matrix<double, 2, 1> obs1;
    const cv::KeyPoint& kpUn1 = pKF1->mvKeysUn[i];
    obs1 << kpUn1.pt.x, kpUn1.pt.y;

    MORB_SLAM::EdgeSim3ProjectXYZ* e12 = new MORB_SLAM::EdgeSim3ProjectXYZ();

    e12->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
    e12->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e12->setMeasurement(obs1);
    const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
    e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

    g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
    e12->setRobustKernel(rk1);
    rk1->setDelta(deltaHuber);
    optimizer.addEdge(e12);

    // Set edge x2 = S21*X1
    Eigen::Matrix<double, 2, 1> obs2;
    cv::KeyPoint kpUn2;
    bool inKF2;
    if (i2 >= 0) {
      kpUn2 = pKF2->mvKeysUn[i2];
      obs2 << kpUn2.pt.x, kpUn2.pt.y;
      inKF2 = true;

      nInKF2++;
    } else {
      float invz = 1 / P3D2c(2);
      float x = P3D2c(0) * invz;
      float y = P3D2c(1) * invz;

      obs2 << x, y;
      kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

      inKF2 = false;
      nOutKF2++;
    }

    MORB_SLAM::EdgeInverseSim3ProjectXYZ* e21 =
        new MORB_SLAM::EdgeInverseSim3ProjectXYZ();

    e21->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
    e21->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e21->setMeasurement(obs2);
    float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
    e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

    g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
    e21->setRobustKernel(rk2);
    rk2->setDelta(deltaHuber);
    optimizer.addEdge(e21);

    vpEdges12.push_back(e12);
    vpEdges21.push_back(e21);
    vnIndexEdge.push_back(i);

    vbIsInKF2.push_back(inKF2);
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Check inliers
  int nBad = 0;
  int nBadOutKF2 = 0;
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    MORB_SLAM::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
    MORB_SLAM::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) continue;

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = nullptr;
      optimizer.removeEdge(e12);
      optimizer.removeEdge(e21);
      vpEdges12[i] = static_cast<MORB_SLAM::EdgeSim3ProjectXYZ*>(nullptr);
      vpEdges21[i] = static_cast<MORB_SLAM::EdgeInverseSim3ProjectXYZ*>(nullptr);
      nBad++;

      if (!vbIsInKF2[i]) {
        nBadOutKF2++;
      }
      continue;
    }

    // Check if remove the robust adjustment improve the result
    e12->setRobustKernel(0);
    e21->setRobustKernel(0);
  }

  int nMoreIterations;
  if (nBad > 0)
    nMoreIterations = 10;
  else
    nMoreIterations = 5;

  if (nCorrespondences - nBad < 10) return 0;

  // Optimize again only with inliers
  optimizer.initializeOptimization();
  optimizer.optimize(nMoreIterations);

  int nIn = 0;
  mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    MORB_SLAM::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
    MORB_SLAM::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) continue;

    e12->computeError();
    e21->computeError();

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = nullptr;
    } else {
      nIn++;
    }
  }

  // Recover optimized Sim3
  g2o::VertexSim3Expmap* vSim3_recov =
      static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
  g2oS12 = vSim3_recov->estimate();

  return nIn;
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd& H,
                                       const int& start, const int& end) {
  // Goal
  // a  | ab | ac       a*  | 0 | ac*
  // ba | b  | bc  -->  0   | 0 | 0
  // ca | cb | c        ca* | 0 | c*

  // Size of block before block to marginalize
  const int a = start;
  // Size of block to marginalize
  const int b = end - start + 1;
  // Size of block after block to marginalize
  const int c = H.cols() - (end + 1);

  // Reorder as follows:
  // a  | ab | ac       a  | ac | ab
  // ba | b  | bc  -->  ca | c  | cb
  // ca | cb | c        ba | bc | b

  Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
    Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
    Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
  }
  if (a > 0 && c > 0) {
    Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
    Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
  }
  if (c > 0) {
    Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
    Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
    Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
  }
  Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

  // Perform marginalization (Schur complement)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv =
      svd.singularValues();
  for (int i = 0; i < b; ++i) {
    if (singularValues_inv(i) > 1e-6)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else
      singularValues_inv(i) = 0;
  }
  Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() *
                          svd.matrixU().transpose();
  Hn.block(0, 0, a + c, a + c) =
      Hn.block(0, 0, a + c, a + c) -
      Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
  Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
  Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
  Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

  // Inverse reorder
  // a*  | ac* | 0       a*  | 0 | ac*
  // ca* | c*  | 0  -->  0   | 0 | 0
  // 0   | 0   | 0       ca* | 0 | c*
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
    res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
    res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
  }
  if (a > 0 && c > 0) {
    res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
    res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
  }
  if (c > 0) {
    res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
    res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
    res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
  }

  res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

  return res;
}

// MORBSLAM Special
// set state variables when skipping PoseOptimization (for when mState == RECENTLY_LOST)
void Optimizer::SkipPoseOptimization(Frame* pFrame) {
  pFrame->mImuBias = pFrame->mpLastKeyFrame->GetImuBias();
  pFrame->mpcpi = pFrame->mpPrevFrame->mpcpi;
}



}  // namespace MORB_SLAM
