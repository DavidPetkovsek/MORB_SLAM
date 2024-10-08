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

#include "MORB_SLAM/ORBmatcher.h"

#include <limits.h>
#include <cinttypes>

#include <opencv2/core/core.hpp>

#ifdef FactoryEngine
#include <apps/morb_dbow2/DBoW2/FeatureVector.h>
#else
#include "DBoW2/FeatureVector.h"
#endif


namespace MORB_SLAM {

// TODO: Make these Settings?
const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri) : mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

int ORBmatcher::SearchByProjection(Frame &F, const std::vector<std::shared_ptr<MapPoint>> &vpMapPoints, const float th, const bool bFarPoints, const float thFarPoints) {
  int nmatches = 0, left = 0, right = 0;

  const bool bFactor = th != 1.0;

  for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
    std::shared_ptr<MapPoint>pMP = vpMapPoints[iMP];

    if (!pMP->mbTrackInView && !pMP->mbTrackInViewR) continue;
    if (bFarPoints && pMP->mTrackDepth > thFarPoints) continue;
    if (pMP->isBad()) continue;

    if (pMP->mbTrackInView) {
      const int &nPredictedLevel = pMP->mnTrackScaleLevel;

      // The size of the window will depend on the viewing direction
      float r = RadiusByViewingCos(pMP->mTrackViewCos);

      if (bFactor) r *= th;

      const std::vector<size_t> vIndices = F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r * F.mvScaleFactors[nPredictedLevel], nPredictedLevel - 1, nPredictedLevel);

      if (!vIndices.empty()) {
        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx = -1;

        // Get best and second matches with near keypoints
        for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
          const size_t idx = *vit;

          if (F.mvpMapPoints[idx] && F.mvpMapPoints[idx]->Observations() > 0) continue;

          if (F.Nleft == -1 && F.mvuRight[idx] > 0) {
            const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
            if (er > r * F.mvScaleFactors[nPredictedLevel]) continue;
          }

          const cv::Mat &d = F.mDescriptors.row(idx);

          const int dist = DescriptorDistance(MPdescriptor, d);

          if (dist < bestDist) {
            bestDist2 = bestDist;
            bestDist = dist;
            bestLevel2 = bestLevel;
            bestLevel = (F.Nleft == -1) ? F.mvKeysUn[idx].octave :
                        (static_cast<int>(idx) < F.Nleft) ? F.mvKeys[idx].octave : F.mvKeysRight[idx - F.Nleft].octave;
            bestIdx = idx;
          } else if (dist < bestDist2) {
            bestLevel2 = (F.Nleft == -1) ? F.mvKeysUn[idx].octave :
                          (static_cast<int>(idx) < F.Nleft) ? F.mvKeys[idx].octave : F.mvKeysRight[idx - F.Nleft].octave;
            bestDist2 = dist;
          }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if (bestDist <= TH_HIGH) {
          if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
            continue;

          if (bestLevel != bestLevel2 || bestDist <= mfNNratio * bestDist2) {
            F.mvpMapPoints[bestIdx] = pMP;
            // Also match with the stereo observation at right camera
            if (F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1) {
              F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
              nmatches++;
              right++;
            }
            nmatches++;
            left++;
          }
        }
      }
    }

    if (F.Nleft != -1 && pMP->mbTrackInViewR) {
      const int &nPredictedLevel = pMP->mnTrackScaleLevelR;
      if (nPredictedLevel != -1) {
        float r = RadiusByViewingCos(pMP->mTrackViewCosR);

        const std::vector<size_t> vIndices = F.GetFeaturesInArea(pMP->mTrackProjXR, pMP->mTrackProjYR, r * F.mvScaleFactors[nPredictedLevel], nPredictedLevel - 1, nPredictedLevel, true);

        if (vIndices.empty()) continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx = -1;

        // Get best and second matches with near keypoints
        for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
          const size_t idx = *vit;

          if (F.mvpMapPoints[idx + F.Nleft] && F.mvpMapPoints[idx + F.Nleft]->Observations() > 0) continue;

          const cv::Mat &d = F.mDescriptors.row(idx + F.Nleft);

          const int dist = DescriptorDistance(MPdescriptor, d);

          if (dist < bestDist) {
            bestDist2 = bestDist;
            bestDist = dist;
            bestLevel2 = bestLevel;
            bestLevel = F.mvKeysRight[idx].octave;
            bestIdx = idx;
          } else if (dist < bestDist2) {
            bestLevel2 = F.mvKeysRight[idx].octave;
            bestDist2 = dist;
          }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if (bestDist <= TH_HIGH) {
          if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
            continue;
          // Also match with the stereo observation at right camera
          if (F.Nleft != -1 && F.mvRightToLeftMatch[bestIdx] != -1) {
            F.mvpMapPoints[F.mvRightToLeftMatch[bestIdx]] = pMP;
            nmatches++;
            left++;
          }
          F.mvpMapPoints[bestIdx + F.Nleft] = pMP;
          nmatches++;
          right++;
        }
      }
    }
  }
  return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos) {
  return (viewCos > 0.998) ? 2.5 : 4.0;
}

int ORBmatcher::SearchByBoW(std::shared_ptr<KeyFrame> pKF, Frame &F, std::vector<std::shared_ptr<MapPoint>> &vpMapPointMatches) {
  const std::vector<std::shared_ptr<MapPoint>> vpMapPointsKF = pKF->GetMapPointMatches();

  vpMapPointMatches = std::vector<std::shared_ptr<MapPoint>>(F.N, nullptr);

  const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

  int nmatches = 0;

  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
  DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
  DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
  DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
  DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

  while (KFit != KFend && Fit != Fend) {
    if (KFit->first == Fit->first) {
      const std::vector<unsigned int> vIndicesKF = KFit->second;
      const std::vector<unsigned int> vIndicesF = Fit->second;

      for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {
        const unsigned int realIdxKF = vIndicesKF[iKF];

        std::shared_ptr<MapPoint>pMP = vpMapPointsKF[realIdxKF];

        if (!pMP || pMP->isBad()) continue;

        const cv::Mat &dKF = pKF->mDescriptors.row(realIdxKF);

        int bestDist1 = 256;
        int bestIdxF = -1;
        int bestDist2 = 256;

        int bestDist1R = 256;
        int bestIdxFR = -1;
        int bestDist2R = 256;

        for (size_t iF = 0; iF < vIndicesF.size(); iF++) {
          if (F.Nleft == -1) {
            const unsigned int realIdxF = vIndicesF[iF];

            if (vpMapPointMatches[realIdxF]) continue;

            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

            const int dist = DescriptorDistance(dKF, dF);

            if (dist < bestDist1) {
              bestDist2 = bestDist1;
              bestDist1 = dist;
              bestIdxF = realIdxF;
            } else if (dist < bestDist2) {
              bestDist2 = dist;
            }
          } else {
            const unsigned int realIdxF = vIndicesF[iF];

            if (vpMapPointMatches[realIdxF]) continue;

            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

            const int dist = DescriptorDistance(dKF, dF);

            if (static_cast<int>(realIdxF) < F.Nleft && dist < bestDist1) {
              bestDist2 = bestDist1;
              bestDist1 = dist;
              bestIdxF = realIdxF;
            } else if (static_cast<int>(realIdxF) < F.Nleft && dist < bestDist2) {
              bestDist2 = dist;
            }

            if (static_cast<int>(realIdxF) >= F.Nleft && dist < bestDist1R) {
              bestDist2R = bestDist1R;
              bestDist1R = dist;
              bestIdxFR = realIdxF;
            } else if (static_cast<int>(realIdxF) >= F.Nleft && dist < bestDist2R) {
              bestDist2R = dist;
            }
          }
        }

        if (bestDist1 <= TH_LOW) {
          if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
            vpMapPointMatches[bestIdxF] = pMP;

            const cv::KeyPoint &kp = (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                      (static_cast<int>(realIdxKF) >= pKF->NLeft) ? pKF->mvKeysRight[realIdxKF - pKF->NLeft] : pKF->mvKeys[realIdxKF];

            if (mbCheckOrientation) {
              cv::KeyPoint &Fkp = (!pKF->mpCamera2 || F.Nleft == -1) ? F.mvKeys[bestIdxF] :
                                  (static_cast<int>(bestIdxF) >= F.Nleft) ? F.mvKeysRight[bestIdxF - F.Nleft] : F.mvKeys[bestIdxF];

              float rot = kp.angle - Fkp.angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(bestIdxF);
            }
            nmatches++;
          }

          if (bestDist1R <= TH_LOW) {
            //TODO: test if we actually want this if statement (w/o the || true)
            if (static_cast<float>(bestDist1R) < mfNNratio * static_cast<float>(bestDist2R) || true) {
              vpMapPointMatches[bestIdxFR] = pMP;

              const cv::KeyPoint &kp = (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF] :
                                        (static_cast<int>(realIdxKF) >= pKF->NLeft) ? pKF->mvKeysRight[realIdxKF - pKF->NLeft] : pKF->mvKeys[realIdxKF];

              if (mbCheckOrientation) {
                cv::KeyPoint &Fkp = (!F.mpCamera2) ? F.mvKeys[bestIdxFR] :
                                    (static_cast<int>(bestIdxFR) >= F.Nleft) ? F.mvKeysRight[bestIdxFR - F.Nleft] : F.mvKeys[bestIdxFR];

                float rot = kp.angle - Fkp.angle;
                if (rot < 0.0) rot += 360.0f;
                int bin = round(rot * factor);
                if (bin == HISTO_LENGTH) bin = 0;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                rotHist[bin].push_back(bestIdxFR);
              }
              nmatches++;
            }
          }
        }
      }
      KFit++;
      Fit++;
    } else if (KFit->first < Fit->first) {
      KFit = vFeatVecKF.lower_bound(Fit->first);
    } else {
      Fit = F.mFeatVec.lower_bound(KFit->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vpMapPointMatches[rotHist[i][j]] = nullptr;
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(std::shared_ptr<KeyFrame> pKF, Sophus::Sim3f &Scw, const std::vector<std::shared_ptr<MapPoint>> &vpPoints, std::vector<std::shared_ptr<MapPoint>> &vpMatched, int th, float ratioHamming) {

  Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Set of MapPoints already found in the KeyFrame
  std::set<std::shared_ptr<MapPoint>> spAlreadyFound(vpMatched.begin(), vpMatched.end());
  spAlreadyFound.erase(nullptr);

  int nmatches = 0;

  // For each Candidate MapPoint Project and Match
  for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
    std::shared_ptr<MapPoint>pMP = vpPoints[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

    // Get 3D Coords.
    Eigen::Vector3f p3Dw = pMP->GetWorldPos();

    // Transform into Camera Coords.
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0) continue;

    // Project into Image
    const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

    // Point must be inside the image
    if (!pKF->IsInImage(uv(0), uv(1))) continue;

    // Depth must be inside the scale invariance region of the point
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist = PO.norm();

    if (dist < minDistance || dist > maxDistance) continue;

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist) continue;

    int nPredictedLevel = pMP->PredictScale(dist, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const std::vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0), uv(1), radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx = -1;
    for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
      const size_t idx = *vit;
      if (vpMatched[idx]) continue;

      const int &kpLevel = pKF->mvKeysUn[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_LOW * ratioHamming) {
      vpMatched[bestIdx] = pMP;
      nmatches++;
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(std::shared_ptr<KeyFrame> pKF, Sophus::Sim3<float> &Scw, const std::vector<std::shared_ptr<MapPoint>> &vpPoints, const std::vector<std::shared_ptr<KeyFrame>> &vpPointsKFs,
                                   std::vector<std::shared_ptr<MapPoint>> &vpMatched, std::vector<std::shared_ptr<KeyFrame>> &vpMatchedKF, int th, float ratioHamming) {
  // Get Calibration Parameters for later projection
  const float &fx = pKF->fx;
  const float &fy = pKF->fy;
  const float &cx = pKF->cx;
  const float &cy = pKF->cy;

  Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Set of MapPoints already found in the KeyFrame
  std::set<std::shared_ptr<MapPoint>> spAlreadyFound(vpMatched.begin(), vpMatched.end());
  spAlreadyFound.erase(nullptr);

  int nmatches = 0;

  // For each Candidate MapPoint Project and Match
  for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
    std::shared_ptr<MapPoint>pMP = vpPoints[iMP];
    std::shared_ptr<KeyFrame> pKFi = vpPointsKFs[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

    // Get 3D Coords.
    Eigen::Vector3f p3Dw = pMP->GetWorldPos();

    // Transform into Camera Coords.
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0) continue;

    // Project into Image
    const float invz = 1 / p3Dc(2);
    const float x = p3Dc(0) * invz;
    const float y = p3Dc(1) * invz;

    const float u = fx * x + cx;
    const float v = fy * y + cy;

    // Point must be inside the image
    if (!pKF->IsInImage(u, v)) continue;

    // Depth must be inside the scale invariance region of the point
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist = PO.norm();

    if (dist < minDistance || dist > maxDistance) continue;

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist) continue;

    int nPredictedLevel = pMP->PredictScale(dist, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const std::vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx = -1;
    for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
      const size_t idx = *vit;
      if (vpMatched[idx]) continue;

      const int &kpLevel = pKF->mvKeysUn[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_LOW * ratioHamming) {
      vpMatched[bestIdx] = pMP;
      vpMatchedKF[bestIdx] = pKFi;
      nmatches++;
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize) {
  int nmatches = 0;
  vnMatches12 = std::vector<int>(F1.mvKeysUn.size(), -1);

  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  std::vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
  std::vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

  for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++) {
    cv::KeyPoint kp1 = F1.mvKeysUn[i1];
    int level1 = kp1.octave;
    if (level1 > 0) continue;

    std::vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

    if (vIndices2.empty()) continue;

    cv::Mat d1 = F1.mDescriptors.row(i1);

    int bestDist = INT_MAX;
    int bestDist2 = INT_MAX;
    int bestIdx2 = -1;

    for (std::vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
      size_t i2 = *vit;

      cv::Mat d2 = F2.mDescriptors.row(i2);

      int dist = DescriptorDistance(d1, d2);

      if (vMatchedDistance[i2] <= dist) continue;

      if (dist < bestDist) {
        bestDist2 = bestDist;
        bestDist = dist;
        bestIdx2 = i2;
      } else if (dist < bestDist2) {
        bestDist2 = dist;
      }
    }

    if (bestDist <= TH_LOW) {
      if (bestDist < (float)bestDist2 * mfNNratio) {
        if (vnMatches21[bestIdx2] >= 0) {
          vnMatches12[vnMatches21[bestIdx2]] = -1;
          nmatches--;
        }
        vnMatches12[i1] = bestIdx2;
        vnMatches21[bestIdx2] = i1;
        vMatchedDistance[bestIdx2] = bestDist;
        nmatches++;

        if (mbCheckOrientation) {
          float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
          if (rot < 0.0) rot += 360.0f;
          int bin = round(rot * factor);
          if (bin == HISTO_LENGTH) bin = 0;
          assert(bin >= 0 && bin < HISTO_LENGTH);
          rotHist[bin].push_back(i1);
        }
      }
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        int idx1 = rotHist[i][j];
        if (vnMatches12[idx1] >= 0) {
          vnMatches12[idx1] = -1;
          nmatches--;
        }
      }
    }
  }

  // Update prev matched
  for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
    if (vnMatches12[i1] >= 0)
      vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;

  return nmatches;
}

int ORBmatcher::SearchByBoW(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame> pKF2, std::vector<std::shared_ptr<MapPoint>> &vpMatches12) {
  const std::vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
  const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
  const std::vector<std::shared_ptr<MapPoint>> vpMapPoints1 = pKF1->GetMapPointMatches();
  const cv::Mat &Descriptors1 = pKF1->mDescriptors;

  const std::vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
  const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
  const std::vector<std::shared_ptr<MapPoint>> vpMapPoints2 = pKF2->GetMapPointMatches();
  const cv::Mat &Descriptors2 = pKF2->mDescriptors;

  vpMatches12 = std::vector<std::shared_ptr<MapPoint>>(vpMapPoints1.size(), nullptr);
  std::vector<bool> vbMatched2(vpMapPoints2.size(), false);

  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);

  const float factor = 1.0f / HISTO_LENGTH;

  int nmatches = 0;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];
        if (pKF1->NLeft != -1 && idx1 >= pKF1->mvKeysUn.size()) {
          continue;
        }

        std::shared_ptr<MapPoint>pMP1 = vpMapPoints1[idx1];
        if (!pMP1) continue;
        if (pMP1->isBad()) continue;

        const cv::Mat &d1 = Descriptors1.row(idx1);

        int bestDist1 = 256;
        int bestIdx2 = -1;
        int bestDist2 = 256;

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          const size_t idx2 = f2it->second[i2];

          if (pKF2->NLeft != -1 && idx2 >= pKF2->mvKeysUn.size()) {
            continue;
          }

          std::shared_ptr<MapPoint>pMP2 = vpMapPoints2[idx2];

          if (vbMatched2[idx2] || !pMP2 || pMP2->isBad()) continue;

          const cv::Mat &d2 = Descriptors2.row(idx2);

          int dist = DescriptorDistance(d1, d2);

          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdx2 = idx2;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }

        if (bestDist1 < TH_LOW) {
          if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
            vpMatches12[idx1] = vpMapPoints2[bestIdx2];
            vbMatched2[bestIdx2] = true;

            if (mbCheckOrientation) {
              float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(idx1);
            }
            nmatches++;
          }
        }
      }
      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vpMatches12[rotHist[i][j]] = nullptr;
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForTriangulation(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame> pKF2, std::vector<std::pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse) {
  const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
  const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

  // Compute epipole in second image
  Sophus::SE3f T1w = pKF1->GetPose();
  Sophus::SE3f T2w = pKF2->GetPose();
  Sophus::SE3f Tw2 = pKF2->GetPoseInverse();  // for convenience
  Eigen::Vector3f Cw = pKF1->GetCameraCenter();
  Eigen::Vector3f C2 = T2w * Cw;

  Eigen::Vector2f ep = pKF2->mpCamera->project(C2);
  Sophus::SE3f T12;
  Sophus::SE3f Tll, Tlr, Trl, Trr;
  Eigen::Matrix3f R12;  // for fastest computation
  Eigen::Vector3f t12;  // for fastest computation

  std::shared_ptr<const GeometricCamera> pCamera1 = pKF1->mpCamera, pCamera2 = pKF2->mpCamera;

  if (!pKF1->mpCamera2 && !pKF2->mpCamera2) {
    T12 = T1w * Tw2;
    R12 = T12.rotationMatrix();
    t12 = T12.translation();
  } else {
    Sophus::SE3f Tr1w = pKF1->GetRightPose();
    Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
    Tll = T1w * Tw2;
    Tlr = T1w * Twr2;
    Trl = Tr1w * Tw2;
    Trr = Tr1w * Twr2;
  }

  Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr = Tlr.rotationMatrix(), Rrl = Trl.rotationMatrix(), Rrr = Trr.rotationMatrix();
  Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(), trl = Trl.translation(), trr = Trr.translation();

  // Find matches between not tracked keypoints
  // Matching speed-up by ORB Vocabulary
  // Compare only ORB that share the same node
  int nmatches = 0;
  std::vector<bool> vbMatched2(pKF2->N, false);
  std::vector<int> vMatches12(pKF1->N, -1);

  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);

  const float factor = 1.0f / HISTO_LENGTH;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];

        std::shared_ptr<MapPoint>pMP1 = pKF1->GetMapPoint(idx1);

        // If there is already a MapPoint skip
        if (pMP1) {
          continue;
        }

        const bool bStereo1 = (!pKF1->mpCamera2 && pKF1->mvuRight[idx1] >= 0);

        if (bOnlyStereo && !bStereo1) continue;

        const cv::KeyPoint &kp1 = (pKF1->NLeft == -1) ? pKF1->mvKeysUn[idx1] :
                                  (static_cast<int>(idx1) < pKF1->NLeft) ? pKF1->mvKeys[idx1] : pKF1->mvKeysRight[idx1 - pKF1->NLeft];

        const bool bRight1 = (pKF1->NLeft != -1 && static_cast<int>(idx1) >= pKF1->NLeft);

        const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

        int bestDist = TH_LOW;
        int bestIdx2 = -1;

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          size_t idx2 = f2it->second[i2];

          std::shared_ptr<MapPoint>pMP2 = pKF2->GetMapPoint(idx2);

          // If we have already matched or there is a MapPoint skip
          if (vbMatched2[idx2] || pMP2) continue;

          const bool bStereo2 = (!pKF2->mpCamera2 && pKF2->mvuRight[idx2] >= 0);

          if (bOnlyStereo && !bStereo2) continue;

          const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

          const int dist = DescriptorDistance(d1, d2);

          if (dist > TH_LOW || dist > bestDist) continue;

          const cv::KeyPoint &kp2 = (pKF2->NLeft == -1) ? pKF2->mvKeysUn[idx2] :
                                    (static_cast<int>(idx2) < pKF2->NLeft) ? pKF2->mvKeys[idx2] : pKF2->mvKeysRight[idx2 - pKF2->NLeft];
          const bool bRight2 = (pKF2->NLeft != -1 && static_cast<int>(idx2) >= pKF2->NLeft);

          if (!bStereo1 && !bStereo2 && !pKF1->mpCamera2) {
            const float distex = ep(0) - kp2.pt.x;
            const float distey = ep(1) - kp2.pt.y;
            if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave]) continue;
          }

          if (pKF1->mpCamera2 && pKF2->mpCamera2) {
            if (bRight1 && bRight2) {
              R12 = Rrr;
              t12 = trr;
              T12 = Trr;

              pCamera1 = pKF1->mpCamera2;
              pCamera2 = pKF2->mpCamera2;
            } else if (bRight1 && !bRight2) {
              R12 = Rrl;
              t12 = trl;
              T12 = Trl;

              pCamera1 = pKF1->mpCamera2;
              pCamera2 = pKF2->mpCamera;
            } else if (!bRight1 && bRight2) {
              R12 = Rlr;
              t12 = tlr;
              T12 = Tlr;

              pCamera1 = pKF1->mpCamera;
              pCamera2 = pKF2->mpCamera2;
            } else {
              R12 = Rll;
              t12 = tll;
              T12 = Tll;

              pCamera1 = pKF1->mpCamera;
              pCamera2 = pKF2->mpCamera;
            }
          }

          // MODIFICATION_2
          if (bCoarse || pCamera1->epipolarConstrain(pCamera2, kp1, kp2, R12, t12, pKF1->mvLevelSigma2[kp1.octave], pKF2->mvLevelSigma2[kp2.octave])) {
            bestIdx2 = idx2;
            bestDist = dist;
          }
        }

        if (bestIdx2 >= 0) {
          const cv::KeyPoint &kp2 = (pKF2->NLeft == -1) ? pKF2->mvKeysUn[bestIdx2] :
                                    (bestIdx2 < pKF2->NLeft) ? pKF2->mvKeys[bestIdx2] : pKF2->mvKeysRight[bestIdx2 - pKF2->NLeft];
          vMatches12[idx1] = bestIdx2;
          nmatches++;

          if (mbCheckOrientation) {
            float rot = kp1.angle - kp2.angle;
            if (rot < 0.0) rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH) bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(idx1);
          }
        }
      }
      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vMatches12[rotHist[i][j]] = -1;
        nmatches--;
      }
    }
  }

  vMatchedPairs.clear();
  vMatchedPairs.reserve(nmatches);

  for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
    if (vMatches12[i] < 0) continue;
    vMatchedPairs.push_back(std::make_pair(i, vMatches12[i]));
  }

  return nmatches;
}

int ORBmatcher::Fuse(std::shared_ptr<KeyFrame> pKF, const std::vector<std::shared_ptr<MapPoint>> &vpMapPoints, const float th, const bool bRight) {
  std::shared_ptr<const GeometricCamera> pCamera;
  Sophus::SE3f Tcw;
  Eigen::Vector3f Ow;

  if (bRight) {
    Tcw = pKF->GetRightPose();
    Ow = pKF->GetRightCameraCenter();
    pCamera = pKF->mpCamera2;
  } else {
    Tcw = pKF->GetPose();
    Ow = pKF->GetCameraCenter();
    pCamera = pKF->mpCamera;
  }

  const float &bf = pKF->mbf;
  int nFused = 0;
  const int nMPs = vpMapPoints.size();

  // For debbuging
  for (int i = 0; i < nMPs; i++) {
    std::shared_ptr<MapPoint>pMP = vpMapPoints[i];

    if (!pMP || pMP->isBad() || pMP->IsInKeyFrame(pKF)) continue;

    Eigen::Vector3f p3Dw = pMP->GetWorldPos();
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0f) continue;

    const float invz = 1 / p3Dc(2);

    const Eigen::Vector2f uv = pCamera->project(p3Dc);

    // Point must be inside the image
    if (!pKF->IsInImage(uv(0), uv(1))) continue;

    const float ur = uv(0) - bf * invz;

    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist3D = PO.norm();

    // Depth must be inside the scale pyramid of the image
    if (dist3D < minDistance || dist3D > maxDistance) continue;

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist3D) continue; 

    int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const std::vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0), uv(1), radius, bRight);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius

    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx = -1;
    for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
      size_t idx = *vit;
      const cv::KeyPoint &kp = (pKF->NLeft == -1) ? pKF->mvKeysUn[idx] :
                                (!bRight) ? pKF->mvKeys[idx] : pKF->mvKeysRight[idx];

      const int &kpLevel = kp.octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      if (pKF->mvuRight[idx] >= 0) {
        // Check reprojection error in stereo
        const float &kpx = kp.pt.x;
        const float &kpy = kp.pt.y;
        const float &kpr = pKF->mvuRight[idx];
        const float ex = uv(0) - kpx;
        const float ey = uv(1) - kpy;
        const float er = ur - kpr;
        const float e2 = ex * ex + ey * ey + er * er;

        if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 7.8) continue;
      } else {
        const float &kpx = kp.pt.x;
        const float &kpy = kp.pt.y;
        const float ex = uv(0) - kpx;
        const float ey = uv(1) - kpy;
        const float e2 = ex * ex + ey * ey;

        if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99) continue;
      }

      if (bRight) idx += pKF->NLeft;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    // If there is already a MapPoint replace otherwise add new measurement
    if (bestDist <= TH_LOW) {
      std::shared_ptr<MapPoint>pMPinKF = pKF->GetMapPoint(bestIdx);
      if (pMPinKF) {
        if (!pMPinKF->isBad()) {
          if (pMPinKF->Observations() > pMP->Observations())
            pMP->Replace(pMPinKF);
          else
            pMPinKF->Replace(pMP);
        }
      } else {
        pMP->AddObservation(pKF, bestIdx);
        pKF->AddMapPoint(pMP, bestIdx);
      }
      nFused++;
    }
  }

  return nFused;
}

int ORBmatcher::Fuse(std::shared_ptr<KeyFrame> pKF, Sophus::Sim3f &Scw, const std::vector<std::shared_ptr<MapPoint>> &vpPoints, float th, std::vector<std::shared_ptr<MapPoint>> &vpReplacePoint) {
  // Decompose Scw
  Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Set of MapPoints already found in the KeyFrame
  const std::set<std::shared_ptr<MapPoint>> spAlreadyFound = pKF->GetMapPoints();

  int nFused = 0;

  const int nPoints = vpPoints.size();

  // For each candidate MapPoint project and match
  for (int iMP = 0; iMP < nPoints; iMP++) {
    std::shared_ptr<MapPoint>pMP = vpPoints[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

    // Get 3D Coords.
    Eigen::Vector3f p3Dw = pMP->GetWorldPos();

    // Transform into Camera Coords.
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0f) continue;

    // Project into Image
    const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

    // Point must be inside the image
    if (!pKF->IsInImage(uv(0), uv(1))) continue;

    // Depth must be inside the scale pyramid of the image
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist3D = PO.norm();

    if (dist3D < minDistance || dist3D > maxDistance) continue;

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist3D) continue;

    // Compute predicted scale level
    const int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const std::vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0), uv(1), radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius

    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = INT_MAX;
    int bestIdx = -1;
    for (std::vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); vit++) {
      const size_t idx = *vit;
      const int &kpLevel = pKF->mvKeysUn[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    // If there is already a MapPoint replace otherwise add new measurement
    if (bestDist <= TH_LOW) {
      std::shared_ptr<MapPoint>pMPinKF = pKF->GetMapPoint(bestIdx);
      if (pMPinKF) {
        if (!pMPinKF->isBad()) vpReplacePoint[iMP] = pMPinKF;
      } else {
        pMP->AddObservation(pKF, bestIdx);
        pKF->AddMapPoint(pMP, bestIdx);
      }
      nFused++;
    }
  }

  return nFused;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono) {
  int nmatches = 0;

  // Rotation Histogram (to check rotation consistency)
  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  const Sophus::SE3f Tcw = CurrentFrame.GetPose();
  const Eigen::Vector3f twc = Tcw.inverse().translation();

  const Sophus::SE3f Tlw = LastFrame.GetPose();
  const Eigen::Vector3f tlc = Tlw * twc;

  const bool bForward = tlc(2) > CurrentFrame.mb && !bMono;
  const bool bBackward = -tlc(2) > CurrentFrame.mb && !bMono;

  for (int i = 0; i < LastFrame.N; i++) {
    std::shared_ptr<MapPoint> pMP = LastFrame.mvpMapPoints[i];
    if(!pMP || LastFrame.mvbOutlier[i])
      continue;
    // Project
    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
    Eigen::Vector3f x3Dc = Tcw * x3Dw;

    const float invzc = 1.0 / x3Dc(2);

    if (invzc < 0) continue; // If the currentFrame should not be able to see the point, ignore the point

    Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

    if (uv(0) < CurrentFrame.mnMinX || uv(0) > CurrentFrame.mnMaxX || uv(1) < CurrentFrame.mnMinY || uv(1) > CurrentFrame.mnMaxY) continue;

    int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

    // Search in a window. Size depends on scale
    float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

    std::vector<size_t> vIndices2;

    if (bForward)
      vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave);
    else if (bBackward)
      vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, 0, nLastOctave);
    else
      vIndices2 = CurrentFrame.GetFeaturesInArea( uv(0), uv(1), radius, nLastOctave - 1, nLastOctave + 1);

    if (vIndices2.empty()) continue;

    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx2 = -1;

    for (std::vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend; vit++) {
      const size_t i2 = *vit;

      if (CurrentFrame.mvpMapPoints[i2] && CurrentFrame.mvpMapPoints[i2]->Observations() > 0) continue;

      if (CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2] > 0) {
        const float ur = uv(0) - CurrentFrame.mbf * invzc;
        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
        if (er > radius) continue;
      }

      const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

      const int dist = DescriptorDistance(dMP, d);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx2 = i2;
      }
    }

    if (bestDist <= TH_HIGH) {
      CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
      nmatches++;

      if (mbCheckOrientation) {
        cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i] :
                            (i < LastFrame.Nleft) ? LastFrame.mvKeys[i] : LastFrame.mvKeysRight[i - LastFrame.Nleft];

        cv::KeyPoint kpCF = (CurrentFrame.Nleft == -1) ? CurrentFrame.mvKeysUn[bestIdx2] :
                            (bestIdx2 < CurrentFrame.Nleft) ? CurrentFrame.mvKeys[bestIdx2] : CurrentFrame.mvKeysRight[bestIdx2 - CurrentFrame.Nleft];
        float rot = kpLF.angle - kpCF.angle;
        if (rot < 0.0) rot += 360.0f;
        int bin = round(rot * factor);
        if (bin == HISTO_LENGTH) bin = 0;
        assert(bin >= 0 && bin < HISTO_LENGTH);
        rotHist[bin].push_back(bestIdx2);
      }
    }
    if (CurrentFrame.Nleft != -1) {
      Eigen::Vector3f x3Dr = CurrentFrame.GetRelativePoseTrl() * x3Dc;
      Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dr);

      int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

      // Search in a window. Size depends on scale
      float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

      std::vector<size_t> vIndices2;

      if (bForward)
        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave, -1, true);
      else if (bBackward)
        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, 0, nLastOctave, true);
      else
        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave - 1, nLastOctave + 1, true);

      const cv::Mat dMP = pMP->GetDescriptor();

      int bestDist = 256;
      int bestIdx2 = -1;

      for (std::vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend; vit++) {
        const size_t i2 = *vit;

        if (CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft] && CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft]->Observations() > 0)
          continue;

        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2 + CurrentFrame.Nleft);

        const int dist = DescriptorDistance(dMP, d);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdx2 = i2;
        }
      }

      if (bestDist <= TH_HIGH) {
        CurrentFrame.mvpMapPoints[bestIdx2 + CurrentFrame.Nleft] = pMP;
        nmatches++;
        if (mbCheckOrientation) {
          cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i] :
                              (i < LastFrame.Nleft) ? LastFrame.mvKeys[i] : LastFrame.mvKeysRight[i - LastFrame.Nleft];

          cv::KeyPoint kpCF = CurrentFrame.mvKeysRight[bestIdx2];

          float rot = kpLF.angle - kpCF.angle;
          if (rot < 0.0) rot += 360.0f;
          int bin = round(rot * factor);
          if (bin == HISTO_LENGTH) bin = 0;
          assert(bin >= 0 && bin < HISTO_LENGTH);
          rotHist[bin].push_back(bestIdx2 + CurrentFrame.Nleft);
        }
      }
    }
  }

  // Apply rotation consistency
  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.mvpMapPoints[rotHist[i][j]] = nullptr;
          nmatches--;
        }
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, std::shared_ptr<KeyFrame> pKF, const std::set<std::shared_ptr<MapPoint>> &sAlreadyFound, const float th, const int ORBdist) {
  int nmatches = 0;

  const Sophus::SE3f Tcw = CurrentFrame.GetPose();
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Rotation Histogram (to check rotation consistency)
  std::vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  const std::vector<std::shared_ptr<MapPoint>> vpMPs = pKF->GetMapPointMatches();

  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    std::shared_ptr<MapPoint>pMP = vpMPs[i];

    if (pMP && !pMP->isBad() && !sAlreadyFound.count(pMP)) {
      // Project
      Eigen::Vector3f x3Dw = pMP->GetWorldPos();
      Eigen::Vector3f x3Dc = Tcw * x3Dw;

      const Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

      if (uv(0) < CurrentFrame.mnMinX || uv(0) > CurrentFrame.mnMaxX || uv(1) < CurrentFrame.mnMinY || uv(1) > CurrentFrame.mnMaxY)
        continue;

      // Compute predicted scale level
      Eigen::Vector3f PO = x3Dw - Ow;
      float dist3D = PO.norm();

      const float maxDistance = pMP->GetMaxDistanceInvariance();
      const float minDistance = pMP->GetMinDistanceInvariance();

      // Depth must be inside the scale pyramid of the image
      if (dist3D < minDistance || dist3D > maxDistance)
        continue;

      int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

      // Search in a window
      const float radius = th * CurrentFrame.mvScaleFactors[nPredictedLevel];

      const std::vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nPredictedLevel - 1, nPredictedLevel + 1);

      if (vIndices2.empty())
        continue;

      const cv::Mat dMP = pMP->GetDescriptor();

      int bestDist = 256;
      int bestIdx2 = -1;

      for (std::vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
        const size_t i2 = *vit;
        if (CurrentFrame.mvpMapPoints[i2])
          continue;

        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);
        const int dist = DescriptorDistance(dMP, d);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdx2 = i2;
        }
      }

      if (bestDist <= ORBdist) {
        CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
        nmatches++;

        if (mbCheckOrientation) {
          float rot = pKF->mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
          if (rot < 0.0) rot += 360.0f;
          int bin = round(rot * factor);
          if (bin == HISTO_LENGTH) bin = 0;
          assert(bin >= 0 && bin < HISTO_LENGTH);
          rotHist[bin].push_back(bestIdx2);
        }
      }
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.mvpMapPoints[rotHist[i][j]] = nullptr;
          nmatches--;
        }
      }
    }
  }
  return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3) {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    } else if (s > max3) {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1) {
    ind2 = -1;
    ind3 = -1;
  } else if (max3 < 0.1f * (float)max1) {
    ind3 = -1;
  }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();
  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }
  return dist;
}

}  // namespace MORB_SLAM
