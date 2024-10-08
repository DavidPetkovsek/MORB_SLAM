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

#include "MORB_SLAM/MapDrawer.h"

#include <pangolin/pangolin.h>

#include <stdexcept>
#include <mutex>

#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/MapPoint.h"

namespace MORB_SLAM {

MapDrawer::MapDrawer(const Atlas_ptr &pAtlas, const Settings& settings): mpAtlas(pAtlas){
  newParameterLoader(settings);
}

void MapDrawer::newParameterLoader(const Settings &settings) {
  mKeyFrameSize = settings.keyFrameSize();
  mKeyFrameLineWidth = settings.keyFrameLineWidth();
  mGraphLineWidth = settings.graphLineWidth();
  mPointSize = settings.pointSize();
  mCameraSize = settings.cameraSize();
  mCameraLineWidth = settings.cameraLineWidth();
}

void MapDrawer::DrawMapPoints() {
  std::shared_ptr<Map> pActiveMap = mpAtlas->GetCurrentMap();
  if (!pActiveMap) return;

  const std::vector<std::shared_ptr<MapPoint>> &vpMPs = pActiveMap->GetAllMapPoints();
  const std::vector<std::shared_ptr<MapPoint>> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

  std::set<std::shared_ptr<MapPoint>> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

  if (vpMPs.empty()) return;

  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(0.0, 0.0, 0.0);

  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])) continue;
    Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
    glVertex3f(pos(0), pos(1), pos(2));
  }
  glEnd();

  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0.0, 0.0);

  for (std::set<std::shared_ptr<MapPoint>>::iterator sit = spRefMPs.begin(), send = spRefMPs.end();
       sit != send; sit++) {
    if ((*sit)->isBad()) continue;
    Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
    glVertex3f(pos(0), pos(1), pos(2));
  }

  glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba) {
  const float &w = mKeyFrameSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  std::shared_ptr<Map> pActiveMap = mpAtlas->GetCurrentMap();
  // DEBUG LBA
  std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
  std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

  if (!pActiveMap) return;

  const std::vector<std::shared_ptr<KeyFrame>> vpKFs = pActiveMap->GetAllKeyFrames();

  if (bDrawKF) {
    for (size_t i = 0; i < vpKFs.size(); i++) {
      std::shared_ptr<KeyFrame> pKF = vpKFs[i];
      Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();

      glPushMatrix();

      glMultMatrixf((GLfloat *)Twc.data());

      // It is the first KF in the map
      if (!pKF->GetParent()) {
        glLineWidth(mKeyFrameLineWidth * 5);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
      } else {
        glLineWidth(mKeyFrameLineWidth);
        if (bDrawOptLba) {
          if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
            glColor3f(0.0f, 1.0f, 0.0f);  // Green -> Opt KFs
          } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
            glColor3f(1.0f, 0.0f, 0.0f);  // Red -> Fixed KFs
          } else {
            glColor3f(0.0f, 0.0f, 1.0f);  // Basic color
          }
        } else {
          glColor3f(0.0f, 0.0f, 1.0f);  // Basic color
        }
        glBegin(GL_LINES);
      }

      glVertex3f(0, 0, 0);
      glVertex3f(w, h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, h, z);

      glVertex3f(w, h, z);
      glVertex3f(w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(-w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);

      glVertex3f(-w, -h, z);
      glVertex3f(w, -h, z);
      glEnd();

      glPopMatrix();

      glEnd();
    }
  }

  if (bDrawGraph) {
    glLineWidth(mGraphLineWidth);
    glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
    glBegin(GL_LINES);

    for (size_t i = 0; i < vpKFs.size(); i++) {
      // Covisibility Graph
      const std::vector<std::shared_ptr<KeyFrame>> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
      Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
      if (!vCovKFs.empty()) {
        for (std::vector<std::shared_ptr<KeyFrame>>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++) {
          if ((*vit)->mnId < vpKFs[i]->mnId) continue;
          Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
          glVertex3f(Ow(0), Ow(1), Ow(2));
          glVertex3f(Ow2(0), Ow2(1), Ow2(2));
        }
      }

      // Spanning tree
      std::shared_ptr<KeyFrame>pParent = vpKFs[i]->GetParent();
      if (pParent) {
        Eigen::Vector3f Owp = pParent->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owp(0), Owp(1), Owp(2));
      }

      // Loops
      std::set<std::shared_ptr<KeyFrame>> sLoopKFs = vpKFs[i]->GetLoopEdges();
      for (std::set<std::shared_ptr<KeyFrame>>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
        if ((*sit)->mnId < vpKFs[i]->mnId) continue;
        Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owl(0), Owl(1), Owl(2));
      }
    }

    glEnd();
  }

  if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
    glLineWidth(mGraphLineWidth);
    glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
    glBegin(GL_LINES);

    // Draw inertial links
    for (size_t i = 0; i < vpKFs.size(); i++) {
      std::shared_ptr<KeyFrame> pKFi = vpKFs[i];
      Eigen::Vector3f Ow = pKFi->GetCameraCenter();
      std::shared_ptr<KeyFrame>pNext = pKFi->mNextKF;
      if (pNext) {
        Eigen::Vector3f Owp = pNext->GetCameraCenter();
        glVertex3f(Ow(0), Ow(1), Ow(2));
        glVertex3f(Owp(0), Owp(1), Owp(2));
      }
    }

    glEnd();
  }

  std::vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();

  if (bDrawKF) {
    for (std::shared_ptr<Map> pMap : vpMaps) {
      if (pMap == pActiveMap) continue;

      std::vector<std::shared_ptr<KeyFrame>> vpKFs = pMap->GetAllKeyFrames();

      for (size_t i = 0; i < vpKFs.size(); i++) {
        std::shared_ptr<KeyFrame> pKF = vpKFs[i];
        Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
        unsigned int index_color = pKF->mnOriginMapId;

        glPushMatrix();

        glMultMatrixf((GLfloat *)Twc.data());

        // It is the first KF in the map
        if (!vpKFs[i]->GetParent()) {
          glLineWidth(mKeyFrameLineWidth * 5);
          glColor3f(1.0f, 0.0f, 0.0f);
          glBegin(GL_LINES);
        } else {
          glLineWidth(mKeyFrameLineWidth);
          glColor3f(mfFrameColors[index_color][0], mfFrameColors[index_color][1], mfFrameColors[index_color][2]);
          glBegin(GL_LINES);
        }

        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
      }
    }
  }
}

float MapDrawer::getCameraSize() const { return mCameraSize; }
float MapDrawer::getCameraLineWidth() const { return mCameraLineWidth; }

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw) {
  std::unique_lock<std::mutex> lock(mMutexCamera);
  mCameraPose = Tcw.inverse();
}

Eigen::Matrix4f MapDrawer::getCameraPose(){
  Eigen::Matrix4f Twc;
  {
    std::unique_lock<std::mutex> lock(mMutexCamera);
    Twc = mCameraPose.matrix();
  }
  return Twc;
}

}  // namespace MORB_SLAM
