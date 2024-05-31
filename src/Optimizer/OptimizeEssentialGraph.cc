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

void Optimizer::OptimizeEssentialGraph(
    std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>& LoopConnections,
    const bool& bFixScale) {
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType* linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  g2o::BlockSolver_7_3* solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  std::vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  std::vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1);  // For debugging
  Eigen::Vector3d z_vec;
  z_vec << 0.0, 0.0, 1.0;

  const int minFeat = 100;

  // Set KeyFrame vertices
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKF->mnId;

    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      VSim3->setEstimate(it->second);
    } else {
      Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
      g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);
    }

    if (pKF->mnId == pMap->GetInitKFid()) VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = bFixScale;

    optimizer.addVertex(VSim3);
    vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec;  // For debugging

    vpVertices[nIDi] = VSim3;
  }

  std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  // Set Loop edges
  int count_loop = 0;
  for (std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>::const_iterator
           mit = LoopConnections.begin(),
           mend = LoopConnections.end();
       mit != mend; mit++) {
    std::shared_ptr<KeyFrame> pKF = mit->first;
    const long unsigned int nIDi = pKF->mnId;
    const std::set<std::shared_ptr<KeyFrame>>& spConnections = mit->second;
    const g2o::Sim3 Siw = vScw[nIDi];
    const g2o::Sim3 Swi = Siw.inverse();

    for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = spConnections.begin(),
                                        send = spConnections.end();
         sit != send; sit++) {
      const long unsigned int nIDj = (*sit)->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(*sit) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sji = Sjw * Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);

      e->information() = matLambda;

      optimizer.addEdge(e);
      count_loop++;
      sInsertedEdges.insert(std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
    }
  }

  // Set normal edges
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    std::shared_ptr<KeyFrame> pKF = vpKFs[i];

    const int nIDi = pKF->mnId;

    g2o::Sim3 Swi;

    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Swi = (iti->second).inverse();
    else
      Swi = vScw[nIDi].inverse();

    std::shared_ptr<KeyFrame> pParentKF = pKF->GetParent();

    // Spanning tree edge
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Sjw;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end())
        Sjw = itj->second;
      else
        Sjw = vScw[nIDj];

      g2o::Sim3 Sji = Sjw * Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // Loop edges
    const std::set<std::shared_ptr<KeyFrame>> sLoopEdges = pKF->GetLoopEdges();
    for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = sLoopEdges.begin(),
                                        send = sLoopEdges.end();
         sit != send; sit++) {
      std::shared_ptr<KeyFrame> pLKF = *sit;
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Slw;

        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

        if (itl != NonCorrectedSim3.end())
          Slw = itl->second;
        else
          Slw = vScw[pLKF->mnId];

        g2o::Sim3 Sli = Slw * Swi;
        g2o::EdgeSim3* el = new g2o::EdgeSim3();
        el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                             optimizer.vertex(pLKF->mnId)));
        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                             optimizer.vertex(nIDi)));
        el->setMeasurement(Sli);
        el->information() = matLambda;
        optimizer.addEdge(el);
      }
    }

    // Covisibility graph edges
    const std::vector<std::shared_ptr<KeyFrame>> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);
    for (std::vector<std::shared_ptr<KeyFrame>>::const_iterator vit = vpConnectedKFs.begin();
         vit != vpConnectedKFs.end(); vit++) {
      std::shared_ptr<KeyFrame> pKFn = *vit;
      if (pKFn && pKFn != pParentKF &&
          !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(std::make_pair(std::min(pKF->mnId, pKFn->mnId),
                                             std::max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Snw;

          LoopClosing::KeyFrameAndPose::const_iterator itn =
              NonCorrectedSim3.find(pKFn);

          if (itn != NonCorrectedSim3.end())
            Snw = itn->second;
          else
            Snw = vScw[pKFn->mnId];

          g2o::Sim3 Sni = Snw * Swi;

          g2o::EdgeSim3* en = new g2o::EdgeSim3();
          en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(pKFn->mnId)));
          en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(nIDi)));
          en->setMeasurement(Sni);
          en->information() = matLambda;
          optimizer.addEdge(en);
        }
      }
    }

    // Inertial edges if inertial
    if (pKF->bImu && pKF->mPrevKF) {
      g2o::Sim3 Spw;
      LoopClosing::KeyFrameAndPose::const_iterator itp =
          NonCorrectedSim3.find(pKF->mPrevKF);
      if (itp != NonCorrectedSim3.end())
        Spw = itp->second;
      else
        Spw = vScw[pKF->mPrevKF->mnId];

      g2o::Sim3 Spi = Spw * Swi;
      g2o::EdgeSim3* ep = new g2o::EdgeSim3();
      ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                           optimizer.vertex(pKF->mPrevKF->mnId)));
      ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                           optimizer.vertex(nIDi)));
      ep->setMeasurement(Spi);
      ep->information() = matLambda;
      optimizer.addEdge(ep);
    }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);
  optimizer.computeActiveErrors();
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (size_t i = 0; i < vpKFs.size(); i++) {
    std::shared_ptr<KeyFrame> pKFi = vpKFs[i];

    const int nIDi = pKFi->mnId;

    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();

    Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                     CorrectedSiw.translation().cast<float>() / s);
    pKFi->SetPose(Tiw);
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    MapPoint* pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    int nIDr;
    if (pMP->mnCorrectedByKF == pCurKF->mnId) {
      nIDr = pMP->mnCorrectedReference;
    } else {
      std::shared_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
      nIDr = pRefKF->mnId;
    }

    g2o::Sim3 Srw = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }

  // TODO Check this changeindex
  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(std::shared_ptr<KeyFrame> pCurKF,
                                       std::vector<std::shared_ptr<KeyFrame>>& vpFixedKFs,
                                       std::vector<std::shared_ptr<KeyFrame>>& vpFixedCorrectedKFs,
                                       std::vector<std::shared_ptr<KeyFrame>>& vpNonFixedKFs,
                                       std::vector<MapPoint*>& vpNonCorrectedMPs) {
  Verbose::PrintMess("Opt_Essential: There are " +
                         std::to_string(vpFixedKFs.size()) +
                         " KFs fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         std::to_string(vpFixedCorrectedKFs.size()) +
                         " KFs fixed in the old map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         std::to_string(vpNonFixedKFs.size()) +
                         " KFs non-fixed in the merged map",
                     Verbose::VERBOSITY_DEBUG);
  Verbose::PrintMess("Opt_Essential: There are " +
                         std::to_string(vpNonCorrectedMPs.size()) +
                         " MPs non-corrected in the merged map",
                     Verbose::VERBOSITY_DEBUG);

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType* linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  g2o::BlockSolver_7_3* solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  std::shared_ptr<Map> pMap = pCurKF->GetMap();
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);
  std::vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

  std::vector<bool> vpGoodPose(nMaxKFid + 1);
  std::vector<bool> vpBadPose(nMaxKFid + 1);

  const int minFeat = 100;

  for (std::shared_ptr<KeyFrame> pKFi : vpFixedKFs) {
    if (pKFi->isBad()) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = true;

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = false;
  }
  Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded",
                     Verbose::VERBOSITY_DEBUG);

  std::set<unsigned long> sIdKF;
  for (std::shared_ptr<KeyFrame> pKFi : vpFixedCorrectedKFs) {
    if (pKFi->isBad()) continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKFi->mnId;

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
    vScw[nIDi] =
        g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

    VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi] = true;
  }

  for (std::shared_ptr<KeyFrame> pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    if (sIdKF.count(nIDi))  // It has already added in the corrected merge KFs
      continue;

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
    g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vScw[nIDi] = Siw;
    VSim3->setEstimate(Siw);

    VSim3->setFixed(false);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = false;
    vpBadPose[nIDi] = true;
  }

  std::vector<std::shared_ptr<KeyFrame>> vpKFs;
  vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() +
                vpNonFixedKFs.size());
  vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
  vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(),
               vpFixedCorrectedKFs.end());
  vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
  std::set<std::shared_ptr<KeyFrame>> spKFs(vpKFs.begin(), vpKFs.end());

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7, 7>::Identity();

  for (std::shared_ptr<KeyFrame> pKFi : vpKFs) {
    int num_connections = 0;
    const int nIDi = pKFi->mnId;

    g2o::Sim3 correctedSwi;
    g2o::Sim3 Swi;

    if (vpGoodPose[nIDi]) correctedSwi = vCorrectedSwc[nIDi];
    if (vpBadPose[nIDi]) Swi = vScw[nIDi].inverse();

    std::shared_ptr<KeyFrame> pParentKFi = pKFi->GetParent();

    // Spanning tree edge
    if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
      int nIDj = pParentKFi->mnId;

      g2o::Sim3 Sjw;
      bool bHasRelation = false;

      if (vpGoodPose[nIDi] && vpGoodPose[nIDj]) {
        Sjw = vCorrectedSwc[nIDj].inverse();
        bHasRelation = true;
      } else if (vpBadPose[nIDi] && vpBadPose[nIDj]) {
        Sjw = vScw[nIDj];
        bHasRelation = true;
      }

      if (bHasRelation) {
        g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;
        optimizer.addEdge(e);
        num_connections++;
      }
    }

    // Loop edges
    const std::set<std::shared_ptr<KeyFrame>> sLoopEdges = pKFi->GetLoopEdges();
    for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = sLoopEdges.begin(),
                                        send = sLoopEdges.end();
         sit != send; sit++) {
      std::shared_ptr<KeyFrame> pLKF = *sit;
      if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
        g2o::Sim3 Slw;
        bool bHasRelation = false;

        if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId]) {
          Slw = vCorrectedSwc[pLKF->mnId].inverse();
          bHasRelation = true;
        } else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId]) {
          Slw = vScw[pLKF->mnId];
          bHasRelation = true;
        }

        if (bHasRelation) {
          g2o::Sim3 Sli = Slw * Swi;
          g2o::EdgeSim3* el = new g2o::EdgeSim3();
          el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(pLKF->mnId)));
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                               optimizer.vertex(nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
          num_connections++;
        }
      }
    }

    // Covisibility graph edges
    const std::vector<std::shared_ptr<KeyFrame>> vpConnectedKFs =
        pKFi->GetCovisiblesByWeight(minFeat);
    for (std::vector<std::shared_ptr<KeyFrame>>::const_iterator vit = vpConnectedKFs.begin();
         vit != vpConnectedKFs.end(); vit++) {
      std::shared_ptr<KeyFrame> pKFn = *vit;
      if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) &&
          !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end()) {
        if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {
          g2o::Sim3 Snw = vScw[pKFn->mnId];
          bool bHasRelation = false;

          if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId]) {
            Snw = vCorrectedSwc[pKFn->mnId].inverse();
            bHasRelation = true;
          } else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId]) {
            Snw = vScw[pKFn->mnId];
            bHasRelation = true;
          }

          if (bHasRelation) {
            g2o::Sim3 Sni = Snw * Swi;

            g2o::EdgeSim3* en = new g2o::EdgeSim3();
            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(pKFn->mnId)));
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
            num_connections++;
          }
        }
      }
    }

    if (num_connections == 0) {
      Verbose::PrintMess(
          "Opt_Essential: KF " + std::to_string(pKFi->mnId) + " has 0 connections",
          Verbose::VERBOSITY_DEBUG);
    }
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (std::shared_ptr<KeyFrame> pKFi : vpNonFixedKFs) {
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    g2o::VertexSim3Expmap* VSim3 =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
    double s = CorrectedSiw.scale();
    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation() / s);

    pKFi->mTcwBefMerge = pKFi->GetPose();
    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
    pKFi->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (MapPoint* pMPi : vpNonCorrectedMPs) {
    if (pMPi->isBad()) continue;

    std::shared_ptr<KeyFrame> pRefKF = pMPi->GetReferenceKeyFrame();
    while (pRefKF->isBad()) {
      if (!pRefKF) {
        Verbose::PrintMess(
            "MP " + std::to_string(pMPi->mnId) + " without a valid reference KF",
            Verbose::VERBOSITY_DEBUG);
        break;
      }

      pMPi->EraseObservation(pRefKF);
      pRefKF = pMPi->GetReferenceKeyFrame();
    }

    if (vpBadPose[pRefKF->mnId]) {
      Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
      Sophus::SE3f Twr = pRefKF->GetPoseInverse();

      Eigen::Vector3f eigCorrectedP3Dw =
          Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
      pMPi->SetWorldPos(eigCorrectedP3Dw);

      pMPi->UpdateNormalAndDepth();
    } else {
      std::cout << "ERROR: MapPoint has a reference KF from another map" << std::endl;
    }
  }
}



void Optimizer::OptimizeEssentialGraph4DoF(
    std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF,
    const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose& CorrectedSim3,
    const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>& LoopConnections) {
  // typedef g2o::BlockSolver<g2o::BlockSolverTraits<4, 4>> BlockSolver_4_4; // UNUSED

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolverX::LinearSolverType* linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();
  const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
      nMaxKFid + 1);

  std::vector<VertexPose4DoF*> vpVertices(nMaxKFid + 1);

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
    vpVertices[nIDi] = V4DoF;
  }
  std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // Edge used in posegraph has still 6Dof, even if updates of camera poses are
  // just in 4DoF
  Eigen::Matrix<double, 6, 6> matLambda =
      Eigen::Matrix<double, 6, 6>::Identity();
  matLambda(0, 0) = 1e3;
  matLambda(1, 1) = 1e3;

  // Set Loop edges
  // Edge4DoF* e_loop; // UNUSED
  for (std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>::const_iterator
           mit = LoopConnections.begin(),
           mend = LoopConnections.end();
       mit != mend; mit++) {
    std::shared_ptr<KeyFrame> pKF = mit->first;
    const long unsigned int nIDi = pKF->mnId;
    const std::set<std::shared_ptr<KeyFrame>>& spConnections = mit->second;
    const g2o::Sim3 Siw = vScw[nIDi];

    for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = spConnections.begin(),
                                        send = spConnections.end();
         sit != send; sit++) {
      const long unsigned int nIDj = (*sit)->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
          pKF->GetWeight(*sit) < minFeat)
        continue;

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sij = Siw * Sjw.inverse();
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3) = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));

      e->information() = matLambda;
      // e_loop = e; // UNUSED
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
    LoopClosing::KeyFrameAndPose::const_iterator iti =
        NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())
      Siw = iti->second;
    else
      Siw = vScw[nIDi];

    // 1.1.0 Spanning tree edge
    std::shared_ptr<KeyFrame> pParentKF = nullptr;
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(pParentKF);

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
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.1.1 Inertial edges
    std::shared_ptr<KeyFrame> prevKF = pKF->mPrevKF;
    if (prevKF) {
      int nIDj = prevKF->mnId;

      g2o::Sim3 Swj;

      LoopClosing::KeyFrameAndPose::const_iterator itj =
          NonCorrectedSim3.find(prevKF);

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
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                          optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.2 Loop edges
    const std::set<std::shared_ptr<KeyFrame>> sLoopEdges = pKF->GetLoopEdges();
    for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = sLoopEdges.begin(),
                                        send = sLoopEdges.end();
         sit != send; sit++) {
      std::shared_ptr<KeyFrame> pLKF = *sit;
      if (pLKF->mnId < pKF->mnId) {
        g2o::Sim3 Swl;

        LoopClosing::KeyFrameAndPose::const_iterator itl =
            NonCorrectedSim3.find(pLKF);

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
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(nIDi)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pLKF->mnId)));
        e->information() = matLambda;
        optimizer.addEdge(e);
      }
    }

    // 1.3 Covisibility graph edges
    const std::vector<std::shared_ptr<KeyFrame>> vpConnectedKFs =
        pKF->GetCovisiblesByWeight(minFeat);
    for (std::vector<std::shared_ptr<KeyFrame>>::const_iterator vit = vpConnectedKFs.begin();
         vit != vpConnectedKFs.end(); vit++) {
      std::shared_ptr<KeyFrame> pKFn = *vit;
      if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF &&
          !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
          if (sInsertedEdges.count(std::make_pair(std::min(pKF->mnId, pKFn->mnId),
                                             std::max(pKF->mnId, pKFn->mnId))))
            continue;

          g2o::Sim3 Swn;

          LoopClosing::KeyFrameAndPose::const_iterator itn =
              NonCorrectedSim3.find(pKFn);

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
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(nIDi)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(pKFn->mnId)));
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

  // Correct points. Transform to "non-optimized" reference keyframe pose and
  // transform back with optimized pose
  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    MapPoint* pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    int nIDr;

    std::shared_ptr<KeyFrame> pRefKF = pMP->GetReferenceKeyFrame();
    nIDr = pRefKF->mnId;

    g2o::Sim3 Srw = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
        correctedSwr.map(Srw.map(eigP3Dw));
    pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    pMP->UpdateNormalAndDepth();
  }
  pMap->IncreaseChangeIndex();
}

}  // namespace MORB_SLAM
