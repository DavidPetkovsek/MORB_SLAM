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

#pragma once

#include <math.h>

#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/InertialOdometry/ExternalFrameData.hpp"
#include "MORB_SLAM/LoopClosing.h"
#include "MORB_SLAM/Map.h"
#include "MORB_SLAM/MapPoint.h"
#ifdef FactoryEngine
#include <apps/morb_g2o/g2o/core/block_solver.h>
#include <apps/morb_g2o/g2o/core/optimization_algorithm_gauss_newton.h>
#include <apps/morb_g2o/g2o/core/optimization_algorithm_levenberg.h>
#include <apps/morb_g2o/g2o/core/robust_kernel_impl.h>
#include <apps/morb_g2o/g2o/core/sparse_block_matrix.h>
#include <apps/morb_g2o/g2o/solvers/linear_solver_dense.h>
#include <apps/morb_g2o/g2o/solvers/linear_solver_eigen.h>
#include <apps/morb_g2o/g2o/types/types_seven_dof_expmap.h>
#include <apps/morb_g2o/g2o/types/types_six_dof_expmap.h>
#else
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include "g2o/types/types_six_dof_expmap.h"
#endif

namespace MORB_SLAM {


class LoopClosing;

class InertialOptimizer {
 public:
  
  void static FullInertialBA(std::shared_ptr<Map> pMap, int its, const bool bFixLocal = false, const unsigned long nLoopKF = 0, bool *pbStopFlag = nullptr, bool bInit = false,
                             ImuInitializater::ImuInitType priorG = ImuInitializater::ImuInitType::DEFAULT_G, ImuInitializater::ImuInitType priorA = ImuInitializater::ImuInitType::DEFAULT_A,
                             Eigen::VectorXd *vSingVal = nullptr, bool *bHess = nullptr);

  int static PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit = false);
  int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

  // For inertial loopclosing
  void static OptimizeEssentialGraph4DoF(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame>pLoopKF, std::shared_ptr<KeyFrame>pCurKF, const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                        const LoopClosing::KeyFrameAndPose &CorrectedSim3, const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>> > &LoopConnections);

  // For inertial systems
  void static LocalInertialBA(std::shared_ptr<KeyFrame> pKF, bool *pbStopFlag, std::shared_ptr<Map> pMap, bool bLarge = false, bool bRecInit = false);
  void static MergeInertialBA(std::shared_ptr<KeyFrame> pCurrKF, std::shared_ptr<KeyFrame> pMergeKF, bool *pbStopFlag, std::shared_ptr<Map> pMap, LoopClosing::KeyFrameAndPose &corrPoses);

  // Inertial pose-graph
  void static InertialOptimization(std::shared_ptr<Map> pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, bool bFixedVel = false, bool bGauss = false,
                                   ImuInitializater::ImuInitType priorG = ImuInitializater::ImuInitType::DEFAULT_G, ImuInitializater::ImuInitType priorA = ImuInitializater::ImuInitType::DEFAULT_A);

  void static InertialOptimization(std::shared_ptr<Map> pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, ImuInitializater::ImuInitType priorG = ImuInitializater::ImuInitType::DEFAULT_G, ImuInitializater::ImuInitType priorA = ImuInitializater::ImuInitType::DEFAULT_A);
  void static InertialOptimization(std::shared_ptr<Map> pMap, Eigen::Matrix3d &Rwg, double &scale);
};


}  // namespace MORB_SLAM

