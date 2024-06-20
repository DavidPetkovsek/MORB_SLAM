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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/MapPoint.h"
#ifdef FactoryEngine
#include <apps/morb_sophus/sim3.hpp>
#else
#include "sophus/sim3.hpp"
#endif

namespace MORB_SLAM {

class ORBmatcher {
 public:
  ORBmatcher(float nnratio = 0.6, bool checkOri = true);

  // Computes the Hamming distance between two ORB descriptors
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

  // Search matches between Frame keypoints and projected MapPoints. Returns
  // number of matches Used to track the local map (Tracking)
  int SearchByProjection(Frame &F, const std::vector<std::shared_ptr<MapPoint>> &vpMapPoints, const float th = 3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

  // Project MapPoints tracked in last frame into the current frame and search
  // matches. Used to track from previous frame (Tracking)
  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

  // Project MapPoints seen in KeyFrame into the Frame and search matches.
  // Used in relocalisation (Tracking)
  int SearchByProjection(Frame &CurrentFrame, std::shared_ptr<KeyFrame> pKF, const std::set<std::shared_ptr<MapPoint>> &sAlreadyFound, const float th, const int ORBdist);

  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in loop detection (Loop Closing)
  int SearchByProjection(std::shared_ptr<KeyFrame> pKF, Sophus::Sim3<float> &Scw, const std::vector<std::shared_ptr<MapPoint>> &vpPoints, std::vector<std::shared_ptr<MapPoint>> &vpMatched, int th, float ratioHamming = 1.0);

  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in Place Recognition (Loop Closing and Merging)
  int SearchByProjection(std::shared_ptr<KeyFrame> pKF, Sophus::Sim3<float> &Scw, const std::vector<std::shared_ptr<MapPoint>> &vpPoints, const std::vector<std::shared_ptr<KeyFrame>> &vpPointsKFs,
                         std::vector<std::shared_ptr<MapPoint>> &vpMatched, std::vector<std::shared_ptr<KeyFrame>> &vpMatchedKF, int th, float ratioHamming = 1.0);

  // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
  // Brute force constrained to ORB that belong to the same vocabulary node (at
  // a certain level) Used in Relocalisation and Loop Detection
  int SearchByBoW(std::shared_ptr<KeyFrame> pKF, Frame &F, std::vector<std::shared_ptr<MapPoint>> &vpMapPointMatches);
  int SearchByBoW(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame> pKF2, std::vector<std::shared_ptr<MapPoint>> &vpMatches12);

  // Matching for the Map Initialization (only used in the monocular case)
  int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize = 10);

  // Matching to triangulate new MapPoints. Check Epipolar Constraint.
  int SearchForTriangulation(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame> pKF2, std::vector<std::pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

  // Project MapPoints into KeyFrame and search for duplicated MapPoints.
  int Fuse(std::shared_ptr<KeyFrame> pKF, const std::vector<std::shared_ptr<MapPoint>> &vpMapPoints, const float th = 3.0, const bool bRight = false);

  // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
  int Fuse(std::shared_ptr<KeyFrame> pKF, Sophus::Sim3f &Scw, const std::vector<std::shared_ptr<MapPoint>> &vpPoints, float th, std::vector<std::shared_ptr<MapPoint>> &vpReplacePoint);

 public:
  static const int TH_LOW;
  static const int TH_HIGH;
  
 protected:
  static const int HISTO_LENGTH;

  float RadiusByViewingCos(const float &viewCos);

  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

  float mfNNratio;
  bool mbCheckOrientation;
};

}  // namespace MORB_SLAM

