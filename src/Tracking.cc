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

#include "MORB_SLAM/Tracking.h"

#include "MORB_SLAM/ImprovedTypes.hpp"
#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/G2oTypes.h"
#include "MORB_SLAM/GeometricTools.h"
#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/MLPnPsolver.h"
#include "MORB_SLAM/ORBmatcher.h"
#include "MORB_SLAM/Optimizer.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"



namespace MORB_SLAM {

Tracking::Tracking(std::shared_ptr<ORBVocabulary> pVoc, const Atlas_ptr &pAtlas,
                   std::shared_ptr<KeyFrameDatabase> pKFDB, const CameraType sensor, std::shared_ptr<Settings> settings)
    : mState(TrackingState::NO_IMAGES_YET),
      mLastProcessedState(TrackingState::NO_IMAGES_YET),
      mSensor(sensor),
      mbOnlyTracking(false),
      mbMapUpdated(false),
      notEnoughMatchPoints_trackOnlyMode(false),
      mpORBVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      mbReadyToInitialize(false),
      mpAtlas(pAtlas),
      mpLastKeyFrame(nullptr),
      mnLastRelocFrameId(0),
      time_recently_lost(1),
      mnFirstFrameId(0),
      mnInitialFrameId(0),
      mbCreatedMap(false),
      mpCamera2(nullptr),
      mForcedLost(false),
      mTeleported(false),
      mLockPreTeleportTranslation(false),
      mStereoInitDefaultPose(Sophus::SE3f()),
      mbReset(false),
      mbResetActiveMap(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false) {
  // Load camera parameters from settings file
  newParameterLoader(*settings);

  initID = 0;
  lastID = 0;

  std::vector<std::shared_ptr<const GeometricCamera>> vpCams = mpAtlas->GetAllCameras();
  std::cout << "There are " << vpCams.size() << " camera(s) in the atlas" << std::endl;
  for (std::shared_ptr<const GeometricCamera> pCam : vpCams) {
    std::cout << "Camera " << pCam->GetId();
    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      std::cout << " is pinhole" << std::endl;
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      std::cout << " is fisheye" << std::endl;
    } else {
      std::cout << " is unknown" << std::endl;
    }
  }

  mBaseTranslation.setZero();
  mPreTeleportTranslation.setZero();

#ifdef REGISTER_TIMES
  vdRectStereo_ms.clear();
  vdResizeImage_ms.clear();
  vdORBExtract_ms.clear();
  vdStereoMatch_ms.clear();
  vdIMUInteg_ms.clear();
  vdPosePred_ms.clear();
  vdLMTrack_ms.clear();
  vdNewKF_ms.clear();
  vdTrackTotal_ms.clear();
#endif
}

#ifdef REGISTER_TIMES
double calcAverage(std::vector<double> v_times) {
  double accum = 0;
  for (double value : v_times) {
    accum += value;
  }

  return accum / v_times.size();
}

double calcDeviation(std::vector<double> v_times, double average) {
  double accum = 0;
  for (double value : v_times) {
    accum += pow(value - average, 2);
  }
  return sqrt(accum / v_times.size());
}

double calcAverage(std::vector<int> v_values) {
  double accum = 0;
  int total = 0;
  for (double value : v_values) {
    if (value == 0) continue;
    accum += value;
    total++;
  }

  return accum / total;
}

double calcDeviation(std::vector<int> v_values, double average) {
  double accum = 0;
  int total = 0;
  for (double value : v_values) {
    if (value == 0) continue;
    accum += pow(value - average, 2);
    total++;
  }
  return sqrt(accum / total);
}

void Tracking::LocalMapStats2File() {
  ofstream f;
  f.open("LocalMapTimeStats.txt");
  f << fixed << setprecision(6);
  f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF "
       "culling[ms], Total[ms]"
    << std::endl;
  for (int i = 0; i < mpLocalMapper->vdLMTotal_ms.size(); ++i) {
    f << mpLocalMapper->vdKFInsert_ms[i] << ","
      << mpLocalMapper->vdMPCulling_ms[i] << ","
      << mpLocalMapper->vdMPCreation_ms[i] << ","
      << mpLocalMapper->vdLBASync_ms[i] << ","
      << mpLocalMapper->vdKFCullingSync_ms[i] << ","
      << mpLocalMapper->vdLMTotal_ms[i] << std::endl;
  }

  f.close();

  f.open("LBA_Stats.txt");
  f << fixed << setprecision(6);
  f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << std::endl;
  for (int i = 0; i < mpLocalMapper->vdLBASync_ms.size(); ++i) {
    f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i]
      << "," << mpLocalMapper->vnLBA_KFfixed[i] << ","
      << mpLocalMapper->vnLBA_MPs[i] << "," << mpLocalMapper->vnLBA_edges[i]
      << std::endl;
  }

  f.close();
}

void Tracking::TrackStats2File() {
  ofstream f;
  f.open("SessionInfo.txt");
  f << fixed;
  f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
  f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << std::endl;

  f << "OpenCV version: " << CV_VERSION << std::endl;

  f.close();

  f.open("TrackingTimeStats.txt");
  f << fixed << setprecision(6);

  f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU "
       "preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
    << std::endl;

  for (int i = 0; i < vdTrackTotal_ms.size(); ++i) {
    double stereo_rect = 0.0;
    if (!vdRectStereo_ms.empty()) {
      stereo_rect = vdRectStereo_ms[i];
    }

    double resize_image = 0.0;
    if (!vdResizeImage_ms.empty()) {
      resize_image = vdResizeImage_ms[i];
    }

    double stereo_match = 0.0;
    if (!vdStereoMatch_ms.empty()) {
      stereo_match = vdStereoMatch_ms[i];
    }

    double imu_preint = 0.0;
    if (!vdIMUInteg_ms.empty()) {
      imu_preint = vdIMUInteg_ms[i];
    }

    f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << ","
      << stereo_match << "," << imu_preint << "," << vdPosePred_ms[i] << ","
      << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i]
      << std::endl;
  }

  f.close();
}

void Tracking::PrintTimeStats() {
  // Save data in files
  TrackStats2File();
  LocalMapStats2File();

  ofstream f;
  f.open("ExecMean.txt");
  f << fixed;
  // Report the mean and std of each one
  std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
  f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;
  f << "OpenCV version: " << CV_VERSION << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
  f << "---------------------------" << std::endl;
  f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
  double average, deviation;
  if (!vdRectStereo_ms.empty()) {
    average = calcAverage(vdRectStereo_ms);
    deviation = calcDeviation(vdRectStereo_ms, average);
    std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Stereo Rectification: " << average << "$\\pm$" << deviation
      << std::endl;
  }

  if (!vdResizeImage_ms.empty()) {
    average = calcAverage(vdResizeImage_ms);
    deviation = calcDeviation(vdResizeImage_ms, average);
    std::cout << "Image Resize: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
  }

  average = calcAverage(vdORBExtract_ms);
  deviation = calcDeviation(vdORBExtract_ms, average);
  std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

  if (!vdStereoMatch_ms.empty()) {
    average = calcAverage(vdStereoMatch_ms);
    deviation = calcDeviation(vdStereoMatch_ms, average);
    std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
  }

  if (!vdIMUInteg_ms.empty()) {
    average = calcAverage(vdIMUInteg_ms);
    deviation = calcDeviation(vdIMUInteg_ms, average);
    std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "IMU Preintegration: " << average << "$\\pm$" << deviation
      << std::endl;
  }

  average = calcAverage(vdPosePred_ms);
  deviation = calcDeviation(vdPosePred_ms, average);
  std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdLMTrack_ms);
  deviation = calcDeviation(vdLMTrack_ms, average);
  std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
  f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdNewKF_ms);
  deviation = calcDeviation(vdNewKF_ms, average);
  std::cout << "New KF decision: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdTrackTotal_ms);
  deviation = calcDeviation(vdTrackTotal_ms, average);
  std::cout << "Total Tracking: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

  // Local Mapping time stats
  std::cout << std::endl << std::endl << std::endl;
  std::cout << "Local Mapping" << std::endl << std::endl;
  f << std::endl << "Local Mapping" << std::endl << std::endl;

  average = calcAverage(mpLocalMapper->vdKFInsert_ms);
  deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
  std::cout << "KF Insertion: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdMPCulling_ms);
  deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
  std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
  f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdMPCreation_ms);
  deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
  std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
  f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdLBA_ms);
  deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
  std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
  f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdKFCulling_ms);
  deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
  std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
  f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vdLMTotal_ms);
  deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
  std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

  // Local Mapping LBA complexity
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
  f << "---------------------------" << std::endl;
  f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_edges);
  deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
  std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
  f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_KFopt);
  deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
  std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
  deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
  std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(mpLocalMapper->vnLBA_MPs);
  deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
  std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
            << std::endl;
  f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

  std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
  std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
  f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
  f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

  // Map complexity
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "Map complexity" << std::endl;
  std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
  std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
  f << "---------------------------" << std::endl;
  f << std::endl << "Map complexity" << std::endl;
  std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
  Map* pBestMap = vpMaps[0];
  for (int i = 1; i < vpMaps.size(); ++i) {
    if (pBestMap->GetAllKeyFrames().size() <
        vpMaps[i]->GetAllKeyFrames().size()) {
      pBestMap = vpMaps[i];
    }
  }

  f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
  f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

  f << "---------------------------" << std::endl;
  f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdDataQuery_ms);
  deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
  f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Database Query: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdEstSim3_ms);
  deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
  f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdPRTotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
  f << "Total Place Recognition: " << average << "$\\pm$" << deviation
    << std::endl
    << std::endl;
  std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
  deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
  f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
  deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
  f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Essential Graph: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
  f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
  std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
  average = calcAverage(mpLoopClosing->vnLoopKFs);
  deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;

  f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
  deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
  f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
  deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
  f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
  deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
  f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
  f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
  std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
  average = calcAverage(mpLoopClosing->vnMergeKFs);
  deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vnMergeMPs);
  deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
  f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
            << std::endl;

  f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
  average = calcAverage(mpLoopClosing->vdGBA_ms);
  deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
  f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
  deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
  f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
  deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
  f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
  std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
  f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
  std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
  average = calcAverage(mpLoopClosing->vnGBAKFs);
  deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(mpLoopClosing->vnGBAMPs);
  deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
  f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
            << std::endl;

  f.close();
}

#endif

Tracking::~Tracking() {}

void Tracking::newParameterLoader(Settings& settings) {
  mpCamera = settings.camera1();
  mpCamera = mpAtlas->AddCamera(mpCamera);

  if (settings.needToUndistort()) {
    mDistCoef = settings.camera1DistortionCoef();
  } else {
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  }

  // TODO: missing image scaling and rectification
  mImageScale = 1.0f;

  mK = cv::Mat::eye(3, 3, CV_32F);
  mK.at<float>(0, 0) = mpCamera->getParameter(0);
  mK.at<float>(1, 1) = mpCamera->getParameter(1);
  mK.at<float>(0, 2) = mpCamera->getParameter(2);
  mK.at<float>(1, 2) = mpCamera->getParameter(3);

  mK_.setIdentity();
  mK_(0, 0) = mpCamera->getParameter(0);
  mK_(1, 1) = mpCamera->getParameter(1);
  mK_(0, 2) = mpCamera->getParameter(2);
  mK_(1, 2) = mpCamera->getParameter(3);

  if (mSensor.hasMulticam() &&
      settings.cameraModelType() == Settings::KannalaBrandt) {
    mpCamera2 = settings.camera2();
    mpCamera2 = mpAtlas->AddCamera(mpCamera2);

    mTlr = settings.Tlr();
  }

  if (mSensor.hasMulticam()) {
    mbf = settings.bf();
    mThDepth = settings.b() * settings.thDepth();
  }

  if (mSensor == CameraType::RGBD || mSensor == CameraType::IMU_RGBD) {
    mDepthMapFactor = settings.depthMapFactor();
    if (fabs(mDepthMapFactor) < 1e-5)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }

  mMinFrames = 0;
  mMaxFrames = settings.fps();
  // might be pointless
  mnFramesToResetIMU = mMaxFrames;

  // ORB parameters
  int nFeatures = settings.nFeatures();
  int nLevels = settings.nLevels();
  int fIniThFAST = settings.initThFAST();
  int fMinThFAST = settings.minThFAST();
  float fScaleFactor = settings.scaleFactor();

  mpORBextractorLeft = std::make_shared<ORBextractor>(nFeatures, fScaleFactor, nLevels,
                                        fIniThFAST, fMinThFAST);

  if (mSensor == CameraType::STEREO || mSensor == CameraType::IMU_STEREO)
    mpORBextractorRight = std::make_shared<ORBextractor>(nFeatures, fScaleFactor, nLevels,
                                           fIniThFAST, fMinThFAST);

  if (mSensor == CameraType::MONOCULAR || mSensor == CameraType::IMU_MONOCULAR)
    mpIniORBextractor = std::make_shared<ORBextractor>(5 * nFeatures, fScaleFactor, nLevels,
                                         fIniThFAST, fMinThFAST);

  mFastInit = settings.fastIMUInit();
  mStationaryInit = settings.stationaryIMUInit();

  // IMU parameters
  Sophus::SE3f Tbc = settings.Tbc();
  mInsertKFsLost = settings.insertKFsWhenLost();
  mImuFreq = settings.imuFrequency();
  //mImuPer = 0.001;  // 1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
  float Ng = settings.noiseGyro();
  float Na = settings.noiseAcc();
  float Ngw = settings.gyroWalk();
  float Naw = settings.accWalk();

  const float sf = sqrt(mImuFreq);
  mpImuCalib = std::make_shared<IMU::Calib>(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

  mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
}

void Tracking::SetLocalMapper(std::shared_ptr<LocalMapping> pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(std::shared_ptr<LoopClosing> pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

StereoPacket Tracking::GrabImageStereo(const cv::Mat& imRectLeft,
                                       const cv::Mat& imRectRight,
                                       const double& timestamp,
                                       const Camera_ptr &cam) {
  // std::cout << "GrabImageStereo" << std::endl;

  cv::Mat imGrayLeft = imRectLeft;
  cv::Mat imGrayRight = imRectRight;

  if (imGrayLeft.channels() == 3) {
    // std::cout << "Image with 3 channels" << std::endl;
    cvtColor(imGrayLeft, imGrayLeft, cv::COLOR_BGR2GRAY);
    cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
  } else if (imGrayLeft.channels() == 4) {
    // std::cout << "Image with 4 channels" << std::endl;
    cvtColor(imGrayLeft, imGrayLeft, cv::COLOR_BGRA2GRAY);
    cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
  }

  // std::cout << "Incoming frame creation" << std::endl;

  if (mSensor == CameraType::STEREO && !mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
  else if (mSensor == CameraType::STEREO && mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr);
  else if (mSensor == CameraType::IMU_STEREO && !mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
  else if (mSensor == CameraType::IMU_STEREO && mpCamera2)
    mCurrentFrame = Frame(cam, imGrayLeft, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

  // std::cout << "Incoming frame ended" << std::endl;

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
  vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

  Track();

  if(mState != TrackingState::OK && mState != TrackingState::NOT_INITIALIZED)
    std::cout << "Current state on Frame " << mCurrentFrame.mnId << ": " << mState << std::endl;
  
  //if state isnt lost, its still possible that it is lost if it trails to infinity - note if its in lost state no keyframes will be produced, but if its in OK state, keyframe will show
  //if mLastFrame.GetPose() from stereo is not close enough to IMU pose, then set to lost
  if (mState != TrackingState::LOST && mState != TrackingState::RECENTLY_LOST && !mReturnPose.translation().isZero(0) && !mForcedLost)
    return StereoPacket(mReturnPose, imGrayLeft, imGrayRight);
    // return StereoPacket(mCurrentFrame.GetPose(), imGrayLeft, imGrayRight);
    

  return StereoPacket(imGrayLeft, imGrayRight); // we do not have a new pose to report
}

RGBDPacket Tracking::GrabImageRGBD(const cv::Mat& imRGB, const cv::Mat& imD,
                                     const double& timestamp, const Camera_ptr &cam) {
  cv::Mat mImGray = imRGB;
  cv::Mat imDepth = imD;

  if (mImGray.channels() == 3) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  if (mSensor == CameraType::RGBD)
    mCurrentFrame = Frame(cam, mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary,
                          mK, mDistCoef, mbf, mThDepth, mpCamera);
  else if (mSensor == CameraType::IMU_RGBD)
    mCurrentFrame = Frame(cam, mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary,
                          mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

  Track();

  //if state isnt lost, its still possible that it is lost if it trails to infinity - note if its in lost state no keyframes will be produced, but if its in OK state, keyframe will show
  //if mLastFrame.GetPose() from stereo is not close enough to IMU pose, then set to lost
  if (mState != TrackingState::LOST && mState != TrackingState::RECENTLY_LOST)
    return RGBDPacket(mCurrentFrame.GetPose(), mImGray, imDepth);
  return RGBDPacket(mImGray, imDepth);
}

MonoPacket Tracking::GrabImageMonocular(const cv::Mat& im, const double& timestamp, const Camera_ptr &cam) {

  cv::Mat mImGray = im;
  if (mImGray.channels() == 3) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  if (mSensor == CameraType::MONOCULAR) {
    if (mState == TrackingState::NOT_INITIALIZED || mState == TrackingState::NO_IMAGES_YET ||
        (lastID - initID) < mMaxFrames)
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpIniORBextractor, mpORBVocabulary,
                            mpCamera, mDistCoef, mbf, mThDepth);
    else
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary,
                            mpCamera, mDistCoef, mbf, mThDepth);
  } else if (mSensor == CameraType::IMU_MONOCULAR) {
    if (mState == TrackingState::NOT_INITIALIZED || mState == TrackingState::NO_IMAGES_YET) {
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpIniORBextractor, mpORBVocabulary,
                            mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
    } else
      mCurrentFrame = Frame(cam, mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary,
                            mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
  }

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

  lastID = mCurrentFrame.mnId;
  Track();

  //if state isnt lost, its still possible that it is lost if it trails to infinity - note if its in lost state no keyframes will be produced, but if its in OK state, keyframe will show
  //if mLastFrame.GetPose() from stereo is not close enough to IMU pose, then set to lost
  if (mState != TrackingState::LOST && mState != TrackingState::RECENTLY_LOST)
    return MonoPacket(mCurrentFrame.GetPose(), mImGray);

  return MonoPacket(mImGray);
}

void Tracking::GrabImuData(const std::vector<IMU::Point>& imuMeasurements) {
  std::scoped_lock<std::mutex> lock(mMutexImuQueue);
  for(auto &point : imuMeasurements)
    mlQueueImuData.emplace_back(point); // copy ctor
}

void Tracking::PreintegrateIMU() {
  //PRIMU1
  if (!mCurrentFrame.mpPrevFrame || mCurrentFrame.mpPrevFrame->isPartiallyConstructed) {
    // Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
    mCurrentFrame.setIntegrated();
    return;
  }

  if (mlQueueImuData.size() == 0) {
    Verbose::PrintMess("No IMU data in mlQueueImuData!!",
                       Verbose::VERBOSITY_NORMAL);
    mCurrentFrame.setIntegrated();
    return;
  }

  //PRIMU2
  std::vector<IMU::Point> frameIMUDataList;
  frameIMUDataList.reserve(mlQueueImuData.size());
  // Fill frameIMUDataList with data for this frame from the global imu data queue
  {
    std::scoped_lock<std::mutex> lock(mMutexImuQueue);
    auto itr = mlQueueImuData.begin();
    // iterate until the end of the queue or until we hit a timestamp that is newer than current frame
    for(; itr != mlQueueImuData.end() && itr->t < mCurrentFrame.mTimeStamp; ++itr)
      if(itr->t >= mCurrentFrame.mpPrevFrame->mTimeStamp) // ignore measurements before the previous frame
        frameIMUDataList.emplace_back(*itr); // add to local vector (via copy)
    // If there are points to remove (we found imu measurements to use)
    if(!frameIMUDataList.empty() && itr != mlQueueImuData.begin())
      mlQueueImuData.erase(mlQueueImuData.begin(), --itr); // erase them from the global queue since they are now copied locally
  }

  //PRIMU3
  // Fails if there's 0 or 1 measurement
  const int n = static_cast<int>(frameIMUDataList.size()) - 1;
  if (n <= 0) { // 0 or 1 measurement
    std::cout << "Empty IMU measurements vector!!!" << std::endl;
    mCurrentFrame.setIntegrated();
    return;
  }

  IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

  // SUS math is being used here
  // points are not equally distributed and are being interpolated. There may not be any benefit to anything this if statement is doing
  // in the below for loop

  //PRIMU4
  // we get here only if there are at least 2 measurements
  for (int i = 0; i < n; i++) { // iterates until the second to last element
    float tstep;
    float dt = frameIMUDataList[i + 1].t - frameIMUDataList[i].t;
    Eigen::Vector3f acc, angVel;
    if ((i == 0) && i < (n - 1)) { // first iteration but not the last iteration
      float timeFromLastFrameToFirstIMUFrame = frameIMUDataList[0].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
      acc = (frameIMUDataList[0].a + frameIMUDataList[1].a -
             (frameIMUDataList[1].a - frameIMUDataList[0].a) *
                 (timeFromLastFrameToFirstIMUFrame / dt)) * 0.5f;
      angVel = (frameIMUDataList[0].w + frameIMUDataList[1].w -
                (frameIMUDataList[1].w - frameIMUDataList[0].w) *
                    (timeFromLastFrameToFirstIMUFrame / dt)) * 0.5f;
      tstep = frameIMUDataList[1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
    } else if (i < (n - 1)) { // not the first nor the last iteration
      acc = (frameIMUDataList[i].a + frameIMUDataList[i + 1].a) * 0.5f;
      angVel = (frameIMUDataList[i].w + frameIMUDataList[i + 1].w) * 0.5f;
      tstep = dt;
    } else if ((i > 0) && (i == (n - 1))) { // not the first but is the last iteration
      float timeFromeLastIMUFrameToCurrentFrame = frameIMUDataList[i + 1].t - mCurrentFrame.mTimeStamp;
      acc = (frameIMUDataList[i].a + frameIMUDataList[i + 1].a -
             (frameIMUDataList[i + 1].a - frameIMUDataList[i].a) *
                 (timeFromeLastIMUFrameToCurrentFrame / dt)) * 0.5f;
      angVel = (frameIMUDataList[i].w + frameIMUDataList[i + 1].w -
                (frameIMUDataList[i + 1].w - frameIMUDataList[i].w) *
                    (timeFromeLastIMUFrameToCurrentFrame / dt)) * 0.5f;
      tstep = mCurrentFrame.mTimeStamp - frameIMUDataList[i].t;
    } else if ((i == 0) && (i == (n - 1))) { // both the first and the last iteration
      // there are two measurements but we ignore the second for fun
      acc = frameIMUDataList[0].a;
      angVel = frameIMUDataList[0].w;
      tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
    }

    if (dt == 0 || tstep == 0) {
      std::cout << "WARNING: a PreintegrateIMU datapoint has dt=0, skipping iteration" << std::endl;
      continue;
    }

    // Temporarily remove to clean up log output
    // std::cout << "accel mag: " << std::sqrt((std::pow(acc(0,0), 2) + std::pow(acc(1,0), 2) + std::pow(acc(2,0), 2))) << "direction: " << acc(0,0) << " , " << acc(1,0) << " , " << acc(2,0) << std::endl;
    
    //PRIMU5
    //Prediction steps of the EKF
    mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
    pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
  }

  mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
  mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
  mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;
  mCurrentFrame.setIntegrated();

  // Verbose::PrintMess("Preintegration is finished!! ",
  // Verbose::VERBOSITY_DEBUG);
}

bool Tracking::PredictStateIMU() {
  //Is it even possible to get here with no previous frame? Maybe through LocalMappingDisabled shenanigans?
  if (!mCurrentFrame.mpPrevFrame || mCurrentFrame.mpPrevFrame->isPartiallyConstructed) {
    Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  //PSIMU1
  //If the map was merged or loop was closed on the last Frame use mpLastKeyFrame, otherwise use mCurrentFrame
  if (mbMapUpdated && mpLastKeyFrame) {
    const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mpImuPreintegratedFromLastKF->dT;
    IMU::Bias b = mpLastKeyFrame->GetImuBias();

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(b));
    Eigen::Vector3f twb2 =
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(b);
    Eigen::Vector3f Vwb2 =
        Vwb1 + t12 * Gz +
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(b);
    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    // Eigen::Vector3f test_displacement = Vwb2; //twb2-twb1; //Rwb2*mCurrentFrame.GetVelocity()*(mCurrentFrame.timestamp - mLastFrame.timestamp);
    // std::cout << "mbMapUpdated, using KeyFrame" << std::endl;
    // if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
    //   std::cout << "bias" << test_displacement.transpose() << std::endl;
    // else
    //   std::cout << "bias0 0 0" << std::endl;

    mCurrentFrame.mImuBias = b;
    mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
    return true;
    //PSIMU3
  } else if (!mbMapUpdated && mCurrentFrame.mpImuPreintegratedFrame) {
    const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();

    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;
    IMU::Bias b = mLastFrame.mImuBias;

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(b));
    Eigen::Vector3f twb2 =
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(b);
    Eigen::Vector3f Vwb2 =
        Vwb1 + t12 * Gz +
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(b);

    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    // Eigen::Vector3f test_displacement = Vwb2; //twb2-twb1; //Rwb2*mCurrentFrame.GetVelocity()*(mCurrentFrame.timestamp - mLastFrame.timestamp);
    // if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
    //   std::cout << "bias" << test_displacement.transpose() << std::endl;
    // else
    //   std::cout << "bias0 0 0" << std::endl;

    mCurrentFrame.mImuBias = b;
    mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
    return true;
    //PSIMU2
  } else
    std::cout << "not IMU prediction!!" << std::endl;
  // only happens if there was no IMU data when PreintegrateIMU() was called this frame
  return false;
}

// There was no comment on what this was supposed to do, but if it follows the same logic as UpdateFrameIMU(), 
// this function was supposed to reset mCurrentFrame and mLastFrame's mBias, mVw, mTcw, PoseMatrices, and some bool flags
void Tracking::ResetFrameIMU() {
  // TODO To implement...
}

void Tracking::Track() {
  //TK1
  if (mpLocalMapper->mbBadImu) {
    mForcedLost = false;
    std::cout << "TRACK: Reset map because local mapper set the bad imu flag " << std::endl;
    RequestResetActiveMap();
    return;
  }
  // std::cout << "TK1" << std::endl;

  //TK2
  std::shared_ptr<Map> pCurrentMap = mpAtlas->GetCurrentMap(false);
  if (!pCurrentMap) {
    std::cout << "ERROR: There is not an active map in the atlas" << std::endl;
    mForcedLost = false;
    return;
  }
  // std::cout << "TK2" << std::endl;

  //TK3
  if (mState != TrackingState::NO_IMAGES_YET) {
    if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
      mForcedLost = false;
      std::cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << std::endl;
      std::scoped_lock<std::mutex> lock(mMutexImuQueue);
      mlQueueImuData.clear();
      CreateMapInAtlas();
      return;
    }
  }
  // std::cout << "TK3" << std::endl;

  
  if (mSensor.isInertial() && mpLastKeyFrame) {
    mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());
  }

  if (mState == TrackingState::NO_IMAGES_YET) {
    mState = TrackingState::NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  if (mSensor.isInertial() && !mbCreatedMap) {
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartPreIMU =
        std::chrono::steady_clock::now();
#endif
    //TK4
    PreintegrateIMU();
    // std::cout << "TK4" << std::endl;
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndPreIMU =
        std::chrono::steady_clock::now();

    double timePreImu =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndPreIMU - time_StartPreIMU)
            .count();
    vdIMUInteg_ms.push_back(timePreImu);
#endif
  }
  mbCreatedMap = false;

  // Get Map Mutex -> Map cannot be changed
  std::unique_lock<std::mutex> lock(pCurrentMap->mMutexMapUpdate);

  mbMapUpdated = false;

  int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
  if (nCurMapChangeIndex > pCurrentMap->GetLastMapChange()) {
    pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
    mbMapUpdated = true;
  }
  //TK5
  if (mState == TrackingState::NOT_INITIALIZED) {
    if (mSensor.hasMulticam()) {
      StereoInitialization();
    } else {
      MonocularInitialization();
    }

    // mpFrameDrawer->Update(this);

    if (mState != TrackingState::OK) {  // If rightly initialized, mState=OK
      mLastFrame = Frame(mCurrentFrame);
      mForcedLost = false;
      return;
    }

    if (mpAtlas->GetAllMaps().size() == 1) {
      mnFirstFrameId = mCurrentFrame.mnId;
    }
  } else {
    // System is initialized. Track Frame.
    bool bOK = false;
    // std::cout << "TK5" << std::endl;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartPosePred =
        std::chrono::steady_clock::now();
#endif

    // Initial camera pose estimation using motion model or relocalization (if
    // tracking is lost)
    if (!mbOnlyTracking) {
      // State OK
      // Local Mapping is activated. This is the normal behaviour, unless
      // you explicitly activate the "only tracking" mode.
      //TK6
      if (mState == TrackingState::OK) {
        //TK6A
        // Local Mapping might have changed some MapPoints tracked in last frame
        CheckReplacedInLastFrame();

        // If the state is not LOST and wasn't reset on the previous frame, use the motion model
        if((imuMotionModelPrepedAfterRecentlyLostTracking || pCurrentMap->isImuInitialized()) && mCurrentFrame.mnId > mnLastRelocFrameId + 1){
          //TK6B
          // std::cout << "Pre-TK6B" << std::endl;
          bOK = TrackWithMotionModel();
          // std::cout << "Post-TK6B" << std::endl;

        }
        // std::cout << "Pre-Pre-TK6C" << std::endl;
        // If the state was lost/reset on the previous Frame, or of TrackWithMotionModel() failed, use the KF
        if (!bOK) {
          // std::cout << "Pre-TK6C" << std::endl;
          //TK6C
          bOK = TrackReferenceKeyFrame();
          // std::cout << "Post-TK6C" << std::endl;
        }
        // std::cout << "Post-Post-TK6C" << std::endl;

        //TK6D
        // If both Track helper functions failed, we are lost
        if (!bOK) {
          std::cout << "TrackReferenceKeyFrame failed, is LOST" << std::endl;
          
          // if there's enough KeyFrames in the map we're recently lost, if not we're lost
          if (pCurrentMap->KeyFramesInMap() > 10 && (mCurrentFrame.mnId > (mnLastRelocFrameId + mnFramesToResetIMU) || !mSensor.isInertial())) {
            mState = TrackingState::RECENTLY_LOST;
            mTimeStampLost = mCurrentFrame.mTimeStamp;
          } else{
            mState = TrackingState::LOST;
          }

        }
        //TK7
      } else if (mState == TrackingState::RECENTLY_LOST) {
        // Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

        bOK = true;
        if (mSensor.isInertial()) { // tried setting to false, still goes to inf
          //TK7A
          if (pCurrentMap->isImuInitialized())
            bOK = PredictStateIMU();
          else
            bOK = false;
          //TK7B
          if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost || mForcedLost) {
            if(mForcedLost) {
              std::cout << "BONK! TrackingState forcefully set to LOST" << std::endl;
              mForcedLost = false;
            } else {
              Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
            }
            mState = TrackingState::LOST;
            bOK = false;
          }
        } else {
          // Relocalization
          bOK = Relocalization();
          // std::cout << "mCurrentFrame.mTimeStamp:" <<
          // std::to_string(mCurrentFrame.mTimeStamp) << std::endl; std::cout <<
          // "mTimeStampLost:" << std::to_string(mTimeStampLost) << std::endl;
          if (mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f && !bOK) {
            mState = TrackingState::LOST;
            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
            bOK = false;
          }
        }
        //TK8 (I don't think a Track() loop can start with the state set to LOST, unless thru non-stereoinertial runs?)
      } else if (mState == TrackingState::LOST) {
        if(mForcedLost) {
          std::cout << "BONK! TrackingState forcefully set to LOST" << std::endl;
          mForcedLost = false;
        }
        Verbose::PrintMess("A new map is started...",
                            Verbose::VERBOSITY_NORMAL);
        //TK8A
        if (pCurrentMap->KeyFramesInMap() < 10) { // Resets maps because they might be garabge KF
          RequestResetActiveMap();
          Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
        //TK8B
        } else {
          setStereoInitDefaultPose(mpLastKeyFrame->GetPose());
          CreateMapInAtlas();
        }

        if (mpLastKeyFrame) mpLastKeyFrame = nullptr;

        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
        //TK8C
        return;
      }

    } else {
      // Localization Mode: Local Mapping is deactivated (TODO Not available in
      // inertial mode)
      if (mState == TrackingState::LOST) {
        if (mSensor.isInertial())
          Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
        bOK = Relocalization();
      } else {
        if (!notEnoughMatchPoints_trackOnlyMode) {
          // In last frame we tracked enough MapPoints in the map
          if (imuMotionModelPrepedAfterRecentlyLostTracking) {
            bOK = TrackWithMotionModel();
          } else {
            bOK = TrackReferenceKeyFrame();
          }
        } else {
          // In last frame we tracked mainly "visual odometry" points.

          // We compute two camera poses, one from motion model and one doing
          // relocalization. If relocalization is sucessfull we choose that
          // solution, otherwise we retain the "visual odometry" solution.

          bool bOKMM = false;
          bool bOKReloc = false;
          std::vector<MapPoint*> vpMPsMM;
          std::vector<bool> vbOutMM;
          Sophus::SE3f TcwMM;
          if (imuMotionModelPrepedAfterRecentlyLostTracking) {
            bOKMM = TrackWithMotionModel();
            vpMPsMM = mCurrentFrame.mvpMapPoints;
            vbOutMM = mCurrentFrame.mvbOutlier;
            TcwMM = mCurrentFrame.GetPose();
          }
          bOKReloc = Relocalization();

          if (bOKMM && !bOKReloc) {
            mCurrentFrame.SetPose(TcwMM);
            mCurrentFrame.mvpMapPoints = vpMPsMM;
            mCurrentFrame.mvbOutlier = vbOutMM;

            if (notEnoughMatchPoints_trackOnlyMode) {
              for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] &&
                    !mCurrentFrame.mvbOutlier[i]) {
                  mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                }
              }
            }
          } else if (bOKReloc) {
            notEnoughMatchPoints_trackOnlyMode = false;
          }

          bOK = bOKReloc || bOKMM;
        }
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndPosePred =
        std::chrono::steady_clock::now();

    double timePosePred =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndPosePred - time_StartPosePred)
            .count();
    vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartLMTrack =
        std::chrono::steady_clock::now();
#endif
    // If we have an initial estimation of the camera pose and matching. Track
    // the local map.
    if (!mbOnlyTracking) {
      if (bOK) {
        //TK9
        bOK = TrackLocalMap();
      } else {
        std::cout << "Fail to track local map!" << std::endl;
      }
    } else {
      // notEnoughMatchPoints_trackOnlyMode true means that there are few matches to MapPoints in the map. We
      // cannot retrieve a local map and therefore we do not perform
      // TrackLocalMap(). Once the system relocalizes the camera we will use the
      // local map again.
      if (bOK && !notEnoughMatchPoints_trackOnlyMode) bOK = TrackLocalMap();
    }
    //TK10
    if (bOK)
      mState = TrackingState::OK;
    else if (mState == TrackingState::OK) {
      if (mSensor.isInertial()) {
        // Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
        //TK11
        if (!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2()) {
          std::cout << "IMU is not or recently initialized. Reseting active map..." << std::endl;
          mForcedLost = false;
          RequestResetActiveMap();
        }
        //TK10
        mState = TrackingState::RECENTLY_LOST;
      } else
        mState = TrackingState::RECENTLY_LOST;  // visual to lost

      /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
      {*/
      mTimeStampLost = mCurrentFrame.mTimeStamp;
      //}
    }

    /* MEMORY LEAK????? WHY?????
    // Save frame if recent relocalization, since they are used for IMU reset
    // (as we are making copy, it shluld be once mCurrFrame is completely modified)
    if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&
        (static_cast<int>(mCurrentFrame.mnId) > mnFramesToResetIMU) &&
        mSensor.isInertial() && pCurrentMap->isImuInitialized()) {
      // TODO check this situation
      Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
      Frame* pF = new Frame(mCurrentFrame);
      pF->mpPrevFrame = new Frame(mLastFrame);

      // Load preintegration
      pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
    }
    */

// TODO: Implement ResetFrameIMU()
/*
    if (pCurrentMap->isImuInitialized()) {
      if (bOK) {
        if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU)) {
          std::cout << "RESETING FRAME!!!" << std::endl;
          ResetFrameIMU();
        } else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
          mLastBias = mCurrentFrame.mImuBias;
      }
    }
*/
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndLMTrack =
        std::chrono::steady_clock::now();

    double timeLMTrack =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndLMTrack - time_StartLMTrack)
            .count();
    vdLMTrack_ms.push_back(timeLMTrack);
#endif


    if (bOK || mState == TrackingState::RECENTLY_LOST) {
      //TK12
      // Update motion model
      if (mLastFrame.isSet() && mCurrentFrame.isSet()) {
        Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
        mVelocity = mCurrentFrame.GetPose() * LastTwc;
        imuMotionModelPrepedAfterRecentlyLostTracking = true;
      } else {
        imuMotionModelPrepedAfterRecentlyLostTracking = false;
      }

      //TK13
      // Clean VO matches
      for (int i = 0; i < mCurrentFrame.N; i++) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations() < 1) {
            // Why don't these actually get deleted?
            // Because they use indexes for matching MapPoints b/t frames, and deleting any of them will offset those indexes
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i] = nullptr;
          }
      }

      //TK14
      // Delete temporal MapPoints
      for (std::list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(),
                                     lend = mlpTemporalPoints.end();
           lit != lend; lit++) {
        MapPoint* pMP = *lit;
        delete pMP;
      }
      mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_StartNewKF =
          std::chrono::steady_clock::now();
#endif
      //TK15
      // Check if we need to insert a new keyframe
      if (mSensor.isInertial() && NeedNewKeyFrame()) {
        CreateNewKeyFrame();
      }

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_EndNewKF =
          std::chrono::steady_clock::now();

      double timeNewKF =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndNewKF - time_StartNewKF)
              .count();
      vdNewKF_ms.push_back(timeNewKF);
#endif

      //TK16
      // We allow points with high innovation (considererd outliers by the Huber
      // Function) pass to the new keyframe, so that bundle adjustment will
      // finally decide if they are outliers or not. We don't want next frame to
      // estimate its position with those points so we discard them in the
      // frame. Only has effect if lastframe is tracked
      for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i] = nullptr;
      }
    }

    //TK19
    // Reset if the camera get lost soon after initialization
    if(mState == TrackingState::LOST) {
      mForcedLost = false;
      if (pCurrentMap->KeyFramesInMap() <= 10) {
        RequestResetActiveMap();
        return;
      }
      if (mSensor.isInertial()) {
        if (!pCurrentMap->isImuInitialized()) {
          Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
          RequestResetActiveMap();
          return;
        }
      }

      setStereoInitDefaultPose(mpLastKeyFrame->GetPose());
      CreateMapInAtlas();

      return;
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    if(!mTeleported && !mLockPreTeleportTranslation) {
      mPreTeleportTranslation = mpReferenceKF->GetRotation().transpose()*mpReferenceKF->GetTranslation();
    } else if(mTeleported) {
      mTeleported = false;
      mLockPreTeleportTranslation = false;
      mBaseTranslation -= (mpReferenceKF->GetRotation().transpose()*mpReferenceKF->GetTranslation()) - mPreTeleportTranslation;
      mPreTeleportTranslation = mpReferenceKF->GetRotation().transpose()*mpReferenceKF->GetTranslation();
      if(pCurrentMap->GetIniertialBA2())
        mpLocalMapper->setIsDoneVIBA(true);
    }

    if(mpLocalMapper->getIsDoneVIBA()) {
      Eigen::Vector3f translation_print = mCurrentFrame.GetPose().rotationMatrix().inverse()*mCurrentFrame.GetPose().translation();
      mReturnPose = Sophus::SE3f(mCurrentFrame.GetPose().rotationMatrix(), mCurrentFrame.GetPose().rotationMatrix()*(translation_print+mBaseTranslation));
    } else {
      Eigen::Vector3f zero;
      zero.setZero();
      mReturnPose = Sophus::SE3f(Eigen::Matrix3f::Identity(), zero);
    }

    mLastFrame = Frame(mCurrentFrame);
  }

  //TK17
  if (mState == TrackingState::OK || mState == TrackingState::RECENTLY_LOST) {
    // Store frame pose information to retrieve the complete camera trajectory
    // afterwards.
    if (mCurrentFrame.isSet()) {
      Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() *
                          mCurrentFrame.mpReferenceKF->GetPoseInverse();
      mlRelativeFramePoses.push_back(Tcr_);
      mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
      // only gets to this scope if mState != TrackingState::LOST
      mlbLost.push_back(mState == TrackingState::LOST);
    } else {
      // This can happen if tracking is lost
      mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
      mlpReferences.push_back(mlpReferences.back());
      // only gets to this scope if mState != TrackingState::LOST
      mlbLost.push_back(mState == TrackingState::LOST);
    }
  }


#ifdef REGISTER_LOOP
  if (Stop()) {
    // Safe area to stop
    while (isStopped()) {
      usleep(3000);
    }
  }
#endif
}

void Tracking::StereoInitialization() {
  //SI1
  // If there aren't enough keypoints, can't initialize and return
  if (mCurrentFrame.N <= 500) {
    std::cout << "There aren't enough KeyPoints in the Frame to initialize the Map" << std::endl;
    return;
  }

  if (mSensor.isInertial()) {
    if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
      // std::cout << "There's no IMU measurements in the previous Frame" << std::endl;
      return;
    }

    if (!stationaryIMUInitEnabled() && (mpAtlas->CountMaps() <= 1) && (mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA).norm() < 0.5) {
      std::cout << "More acceleration is required to initialize the Map" << std::endl;
      return;
    }

    if (mpImuPreintegratedFromLastKF) delete mpImuPreintegratedFromLastKF;

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
  }
  //SI2
  // Set Frame pose to the default pose
  mCurrentFrame.SetPose(getStereoInitDefaultPose());
  if (mSensor.isInertial()) {
    // Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
    // Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
    Eigen::Vector3f Vwb0;
    Vwb0.setZero();
    mCurrentFrame.SetVelocity(Vwb0);
    // mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
  } //else
    // mCurrentFrame.SetPose(Sophus::SE3f());

  // Create KeyFrame
  KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  // Insert KeyFrame in the map
  // Why is this an Atlas function?
  mpAtlas->AddKeyFrame(pKFini);

  // Create MapPoints and asscoiate to KeyFrame
  if (!mpCamera2) {
    for (int i = 0; i < mCurrentFrame.N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        Eigen::Vector3f x3D;
        mCurrentFrame.UnprojectStereo(i, x3D);
        MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
        pNewMP->AddObservation(pKFini, i);
        pKFini->AddMapPoint(pNewMP, i);
        pNewMP->ComputeDistinctiveDescriptors();
        pNewMP->UpdateNormalAndDepth();
        mpAtlas->AddMapPoint(pNewMP);

        mCurrentFrame.mvpMapPoints[i] = pNewMP;
      }
    }
  } else {
    //SI3
    for (int i = 0; i < mCurrentFrame.Nleft; i++) {
      int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
      if (rightIndex != -1) {
        //SI3a
        Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

        MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

        pNewMP->AddObservation(pKFini, i);
        pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

        pKFini->AddMapPoint(pNewMP, i);
        pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

        //SI3b
        pNewMP->ComputeDistinctiveDescriptors();
        //SI3c
        pNewMP->UpdateNormalAndDepth();
        //SI3d
        mpAtlas->AddMapPoint(pNewMP);

        mCurrentFrame.mvpMapPoints[i] = pNewMP;
        mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
      }
    }
  }

  Verbose::PrintMess("New Map created with " + std::to_string(mpAtlas->MapPointsInMap()) + " MapPoints", Verbose::VERBOSITY_QUIET);

  // std::cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << std::endl;

  //SI4
  mpLocalMapper->InsertKeyFrame(pKFini);

  mLastFrame = Frame(mCurrentFrame);
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFini;
  // mnLastRelocFrameId = mCurrentFrame.mnId;

  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
  mpReferenceKF = pKFini;
  mCurrentFrame.mpReferenceKF = pKFini;

  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

  // mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

  mState = TrackingState::OK;
}

void Tracking::MonocularInitialization() {
  if (!mbReadyToInitialize) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() > 100) {
      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

      if (mSensor == CameraType::IMU_MONOCULAR) {
        if (mpImuPreintegratedFromLastKF) delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
      }

      mbReadyToInitialize = true;

      return;
    }
  } else {
    if (((int)mCurrentFrame.mvKeys.size() <= 100) ||
        ((mSensor == CameraType::IMU_MONOCULAR) &&
         (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0))) {
      mbReadyToInitialize = false;

      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(
        mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches < 100) {
      mbReadyToInitialize = false;
      return;
    }

    Sophus::SE3f Tcw;
    std::vector<bool> vbTriangulated;  // Triangulated Correspondences (mvIniMatches)

    if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,
                                          mCurrentFrame.mvKeysUn, mvIniMatches,
                                          Tcw, mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i] = -1;
          nmatches--;
        }
      } 

      // Set Frame Poses
      mInitialFrame.SetPose(Sophus::SE3f());
      mCurrentFrame.SetPose(Tcw);

      CreateInitialMapMonocular();
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  KeyFrame* pKFini =
      new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
  KeyFrame* pKFcur =
      new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mSensor == CameraType::IMU_MONOCULAR)
    pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(nullptr);

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  // Insert KFs in the map
  mpAtlas->AddKeyFrame(pKFini);
  mpAtlas->AddKeyFrame(pKFcur);

  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0) continue;

    // Create MapPoint.
    Eigen::Vector3f worldPos;
    worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
    MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    // Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    // Add to Map
    mpAtlas->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  std::set<MapPoint*> sMPs;
  sMPs = pKFini->GetMapPoints();

  // Bundle Adjustment
  Verbose::PrintMess("New Map created with " +
                         std::to_string(mpAtlas->MapPointsInMap()) + " points",
                     Verbose::VERBOSITY_QUIET);
  Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth;
  if (mSensor == CameraType::IMU_MONOCULAR)
    invMedianDepth = 4.0f / medianDepth;  // 4.0f
  else
    invMedianDepth = 1.0f / medianDepth;

  // TODO Check, originally 100 tracks
  if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50) {
    Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
    RequestResetActiveMap();
    return;
  }

  // Scale initial baseline
  Sophus::SE3f Tc2w = pKFcur->GetPose();
  Tc2w.translation() *= invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  std::vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
  for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
      MapPoint* pMP = vpAllMapPoints[iMP];
      pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      pMP->UpdateNormalAndDepth();
    }
  }

  if (mSensor == CameraType::IMU_MONOCULAR) {
    pKFcur->mPrevKF = pKFini;
    pKFini->mNextKF = pKFcur;
    pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);
  mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;
  // mnLastRelocFrameId = mInitialFrame.mnId;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  // Compute here initial velocity
  std::vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

  Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
  imuMotionModelPrepedAfterRecentlyLostTracking = false;
  Eigen::Vector3f phi = deltaT.so3().log();

  double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) / (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
  phi *= aux;

  mLastFrame = Frame(mCurrentFrame);

  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  // mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

  mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

  mState = TrackingState::OK;

  initID = pKFcur->mnId;
}

void Tracking::CreateMapInAtlas() {

  mpAtlas->CreateNewMap();  

  mnInitialFrameId = mCurrentFrame.mnId + 1;

  mState = TrackingState::NO_IMAGES_YET;

  // Reset the variables with information about the last KF
  imuMotionModelPrepedAfterRecentlyLostTracking = false;
  notEnoughMatchPoints_trackOnlyMode = false;

  if (mSensor.isInertial() && mpImuPreintegratedFromLastKF) {
    delete mpImuPreintegratedFromLastKF;
    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
  }

  if (mpLastKeyFrame) mpLastKeyFrame = nullptr;
  if (mpReferenceKF) mpReferenceKF = nullptr;

  // Reset the current and last Frames
  mLastFrame = Frame();
  mCurrentFrame = Frame();

  // prevents PreintegrateIMU() from being called in the next frame
  mbCreatedMap = true;

  // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the
  // current id, because it is the new starting point for new map
  Verbose::PrintMess("First frame id in map: " + std::to_string(mCurrentFrame.mnId + 1), Verbose::VERBOSITY_NORMAL);

  // only used by monocular tracking
  mbReadyToInitialize = false;
  
  // only used by the viewer or monocular tracking
  mvIniMatches.clear();
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i = 0; i < mLastFrame.N; i++) {
    MapPoint* pMP = mLastFrame.mvpMapPoints[i];

    if (pMP) {
      MapPoint* pRep = pMP->GetReplaced();
      if (pRep) {
        mLastFrame.mvpMapPoints[i] = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  // Compute Bag of Words vector
  mCurrentFrame.ComputeBoW();

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.7, true);
  std::vector<MapPoint*> vpMapPointMatches;

  int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

  if (nmatches < 15) {
    std::cout << "TRACK_REF_KF: Less than 15 matches!!" << std::endl;
    return false;
  }

  mCurrentFrame.mvpMapPoints = vpMapPointMatches;
  mCurrentFrame.SetPose(mLastFrame.GetPose());

  // std::cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw <<
  // std::endl;
  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    // if(i >= mCurrentFrame.Nleft) break;
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = nullptr;
        mCurrentFrame.mvbOutlier[i] = false;
        if (i < mCurrentFrame.Nleft) {
          pMP->mbTrackInView = false;
        } else {
          pMP->mbTrackInViewR = false;
        }
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  return (mSensor.isInertial() || (nmatchesMap >= 10));
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  // For StereoInertial Tracking, these next 2 lines do nothing but add rounding error (I think)
  Sophus::SE3f Tlr = mlRelativeFramePoses.back();
  mLastFrame.SetPose(Tlr * mLastFrame.mpReferenceKF->GetPose());

  //ULF1
  if (mnLastKeyFrameId == mLastFrame.mnId || !mSensor.hasMulticam() || !mbOnlyTracking)
    return;

  //ULF2
  // Create "visual odometry" MapPoints
  // We sort points according to their measured depth by the stereo/RGB-D sensor
  std::vector<std::pair<float, int>> vDepthIdx;
  const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
  vDepthIdx.reserve(Nfeat);
  for (int i = 0; i < Nfeat; i++) {
    float z = mLastFrame.mvDepth[i];
    if (z > 0) {
      vDepthIdx.emplace_back(z, i);
    }
  }

  if (vDepthIdx.empty()) return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // We insert all close points (depth<mThDepth)
  // If more than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;

    // if the observed point isn't in the MapPoints already, add it
    if (!mLastFrame.mvpMapPoints[i] || mLastFrame.mvpMapPoints[i]->Observations() < 1) {
      Eigen::Vector3f x3D;

      if (mLastFrame.Nleft == -1) {
        mLastFrame.UnprojectStereo(i, x3D);
      } else {
        // rotates/translates the MapPoint coords to be from the reference from of the origin
        x3D = mLastFrame.UnprojectStereoFishEye(i);
      }

      MapPoint* pNewMP = new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
      mLastFrame.mvpMapPoints[i] = pNewMP;
      mlpTemporalPoints.push_back(pNewMP);
    }
    nPoints++;

    if (vDepthIdx[j].first > mThDepth && nPoints > 100) break;
  }
}

bool Tracking::TrackWithMotionModel() {
  ORBmatcher matcher(0.9, true);

  //TWMM1
  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points if in Localization Mode
  UpdateLastFrame();

  //TWMM2
  if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
    // Predict state with IMU if it is initialized and it doesnt need reset
    return PredictStateIMU();
  }
  //TWMM3
  //No IMU, so assume the pose changed by the same amount it changed by last Frame
  mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());

  //TWMM4
  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);

  // Project points seen in previous frame
  int th = (mSensor == CameraType::STEREO) ? 7 : 15;

  //TWMM5
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, !mSensor.hasMulticam());

  //TWMM6
  // If few matches, uses a higher-tolerance search
  if (nmatches < 20) {
    Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);

    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, !mSensor.hasMulticam());
    Verbose::PrintMess("Matches with wider search: " + std::to_string(nmatches), Verbose::VERBOSITY_NORMAL);
  }

  if (nmatches < 20) {
    Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
    return mSensor.isInertial();
  }

  //TWMM7
  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      //TWMM8
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = nullptr;
        mCurrentFrame.mvbOutlier[i] = false;
        if (i < mCurrentFrame.Nleft) {
          pMP->mbTrackInView = false;
        } else {
          pMP->mbTrackInViewR = false;
        }
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
        //TWMM9
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  // Should these be the same number?
  if (mbOnlyTracking) {
    notEnoughMatchPoints_trackOnlyMode = nmatchesMap < 10;
    return nmatches > 20;
  }
  //TWMM10
  return (mSensor.isInertial() || (nmatchesMap >= 10));
}

bool Tracking::TrackLocalMap() {
  // We have an estimation of the camera pose and some map points tracked in the
  // frame. We retrieve the local map and try to find matches to points in the
  // local map.

  //TLM1
  UpdateLocalMap();
  SearchLocalPoints();

/* // UNUSED
  // TOO check outliers before PO
  int aux1 = 0, aux2 = 0;
  for (int i = 0; i < mCurrentFrame.N; i++)
    if (mCurrentFrame.mvpMapPoints[i]) {
      aux1++;
      if (mCurrentFrame.mvbOutlier[i]) aux2++;
    }
*/
  // int inliers; // UNUSED

  //TLM2
  if (!mpAtlas->isImuInitialized() || mCurrentFrame.mpImuPreintegratedFrame == nullptr || mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
    // Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
    Optimizer::PoseOptimization(&mCurrentFrame);
  } else if(mbMapUpdated) {
    Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame);
  } else {
    Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame);
  }
  
/* // UNUSED
  int aux1 = 0, aux2 = 0;
  for (int i = 0; i < mCurrentFrame.N; i++)
    if (mCurrentFrame.mvpMapPoints[i]) {
      aux1++;
      if (mCurrentFrame.mvbOutlier[i]) aux2++;
    }
*/

  mnMatchesInliers = 0;

  //TLM3
  // Update MapPoints Statistics
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        if (mbOnlyTracking || mCurrentFrame.mvpMapPoints[i]->Observations() > 0) 
          mnMatchesInliers++;
      // SUS why only stereo? No stereo_inertial
      } else if (mSensor == CameraType::STEREO)
        mCurrentFrame.mvpMapPoints[i] = nullptr;
    }
  }

  //TLM4
  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently
  mpLocalMapper->mnMatchesInliers = mnMatchesInliers;

  if(mForcedLost) {
    std::cout << "BONK queued for the next frame" << std::endl;
    return false;
  }

  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames) return mnMatchesInliers >= 50;

  if (mState == TrackingState::RECENTLY_LOST) return mnMatchesInliers >= 150;

  if(mSensor.isInertial()){
    if (!mSensor.hasMulticam()) {
      return !((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) || (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized()));
    } else {
      return mnMatchesInliers >= 15;
    } 
  } else {
    return mnMatchesInliers >= 30;
    }
}

bool Tracking::NeedNewKeyFrame() {
  if (mSensor.isInertial() && !mpAtlas->GetCurrentMap()->isImuInitialized()) 
    return (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25;

  if (mbOnlyTracking) return false;

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

  const int nKFs = mpAtlas->KeyFramesInMap();

  // Do not insert keyframes if not enough frames have passed from last
  // relocalisation
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
    return false;

  // Tracked MapPoints in the reference keyframe
  int nMinObs = (nKFs <= 2) ? 2 : 3;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  // Local Mapping accept keyframes?
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Check how many "close" points are being tracked and how many could be
  // potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;

  if (mSensor.hasMulticam()) {
    int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
    for (int i = 0; i < N; i++) {
      if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
          nTrackedClose++;
        else
          nNonTrackedClose++;
      }
    }
    // Verbose::PrintMess("[NEEDNEWKF]-> closed points: " +
    // std::to_string(nTrackedClose) + "; non tracked closed points: " +
    // std::to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);//
    // Verbose::VERBOSITY_DEBUG);
  }

  bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

  // Thresholds
  float thRefRatio = 0.75f;
  if (nKFs < 2) thRefRatio = 0.4f;

  /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
  const int thStereoClosedPoints = 15;
  if(nClosedPoints < thStereoClosedPoints && (mSensor==CameraType::STEREO ||
  mSensor==CameraType::IMU_STEREO))
  {
      //Pseudo-monocular, there are not enough close points to be confident
  about the stereo observations. thRefRatio = 0.9f;
  }*/

  if (mSensor == CameraType::MONOCULAR) thRefRatio = 0.9f;

  if (mpCamera2) thRefRatio = 0.75f;

  if (mSensor == CameraType::IMU_MONOCULAR) {
    if (mnMatchesInliers > 350)  // Points tracked from the local map
      thRefRatio = 0.75f;
    else
      thRefRatio = 0.90f;
  }

  // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) && bLocalMappingIdle);  // mpLocalMapper->KeyframesInQueue() < 2);
  // Condition 1c: tracking is weak
  const bool c1c = mSensor.hasMulticam() && !mSensor.isInertial() && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
  // Condition 2: Few tracked points compared to reference keyframe. Lots of
  // visual odometry compared to map matches.
  const bool c2 = (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) && mnMatchesInliers > 15);

  // std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
  // Temporal condition for Inertial cases
  bool c3 = mpLastKeyFrame && mSensor.isInertial() && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5;

  bool c4 = (((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) || mState == TrackingState::RECENTLY_LOST) && (mSensor == CameraType::IMU_MONOCULAR);
  // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==TrackingState::RECENTLY_LOST) && ((mSensor == CameraType::IMU_MONOCULAR)))

  if (((c1a || c1b || c1c) && c2) || c3 || c4) {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle || mpLocalMapper->IsInitializing()) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      if (mSensor.hasMulticam() && (mpLocalMapper->KeyframesInQueue() < 3)) {
        return true;
      } else {
        // std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
        return false;
      }
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame() {
  if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized()) return;

  if (!mpLocalMapper->SetNotStop(true)) return;

  KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mpAtlas->isImuInitialized())  //  || mpLocalMapper->IsInitializing())
    pKF->bImu = true;

  pKF->SetNewBias(mCurrentFrame.mImuBias);
  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  if (mpLastKeyFrame) {
    pKF->mPrevKF = mpLastKeyFrame;
    mpLastKeyFrame->mNextKF = pKF;
  } else
    Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

  // Reset preintegration from last KF (Create new object)
  if (mSensor.isInertial())
    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);

  if (mSensor.hasMulticam()){  // TODO check if incluide imu_stereo
    mCurrentFrame.UpdatePoseMatrices();
    // std::cout << "create new MPs" << std::endl;
    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    int maxPoint = 100;

    std::vector<std::pair<float, int>> vDepthIdx;
    int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
    vDepthIdx.reserve(mCurrentFrame.N);
    for (int i = 0; i < N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        vDepthIdx.emplace_back(z, i);
      }
    }

    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());

      int nPoints = 0;
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP)
          bCreateNew = true;
        else if (pMP->Observations() < 1) {
          bCreateNew = true;
          mCurrentFrame.mvpMapPoints[i] = nullptr;
        }

        if (bCreateNew) {
          Eigen::Vector3f x3D;

          if (mCurrentFrame.Nleft == -1) {
            mCurrentFrame.UnprojectStereo(i, x3D);
          } else {
            x3D = mCurrentFrame.UnprojectStereoFishEye(i);
          }

          MapPoint* pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
          pNewMP->AddObservation(pKF, i);

          // Check if it is a stereo observation in order to not
          // duplicate mappoints
          if (mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
            mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
            pNewMP->AddObservation(pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
            pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
          }

          pKF->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
        }
        nPoints++;

        if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
          break;
        }
      }
      // Verbose::PrintMess("new mps for stereo KF: " + std::to_string(nPoints),
      // Verbose::VERBOSITY_NORMAL);
    }
  }

  mpLocalMapper->InsertKeyFrame(pKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  for (MapPoint*& pMP : mCurrentFrame.mvpMapPoints) {
    if (pMP) {
      if (pMP->isBad()) {
        pMP = nullptr;
      } else {
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
      }
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (MapPoint* pMP : mvpLocalMapPoints) {
    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId || pMP->isBad()) continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }

  if (nToMatch > 0) {
    ORBmatcher matcher(0.8);
    int th = 1; // this ultimately is a multiplier on the size of a square search area in a frame
    if (mSensor == CameraType::RGBD || mSensor == CameraType::IMU_RGBD) th = 3;
    if (mpAtlas->isImuInitialized()) {
      if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
        th = 2;
      else
        th = 6;
    } else if (!mpAtlas->isImuInitialized() && mSensor.isInertial()) {
      th = 10;
    }

    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2) th = 5;

    if (mState == TrackingState::LOST || mState == TrackingState::RECENTLY_LOST)  // Lost for less than 1 second
      th = 15;

    /*int matches = */matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();

  for (KeyFrame* pKF : mvpLocalKeyFrames) {
    for (MapPoint* pMP : pKF->GetMapPointMatches()) {
      if (!pMP || pMP->isBad() || pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId) continue;
      mvpLocalMapPoints.push_back(pMP);
      pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  std::map<KeyFrame*, int> keyframeCounter;
  if (!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId < mnLastRelocFrameId + 2)) {
    for (int i = 0; i < mCurrentFrame.N; i++) {
      MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!pMP->isBad()) {
          const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
          for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          mCurrentFrame.mvpMapPoints[i] = nullptr;
        }
      }
    }
  } else {
    for (int i = 0; i < mLastFrame.N; i++) {
      // Using lastframe since current frame has not matches yet
      if (mLastFrame.mvpMapPoints[i]) {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP) continue;
        if (!pMP->isBad()) {
          const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
          for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          // MODIFICATION
          mLastFrame.mvpMapPoints[i] = nullptr;
        }
      }
    }
  }

  int max = 0;
  KeyFrame* pKFmax = nullptr;

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also
  // check which keyframe shares most points
  for (auto &pair : keyframeCounter) {
    KeyFrame* pKF = pair.first;

    if (pKF->isBad()) continue;

    if (pair.second > max) {
      max = pair.second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(pKF);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }

  // Include also some not-already-included keyframes that are neighbors to
  // already-included keyframes
  for (KeyFrame* pKF : mvpLocalKeyFrames) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80) break;

    for (KeyFrame* pNeighKF : pKF->GetBestCovisibilityKeyFrames(10)) {
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    for (KeyFrame* pChildKF : pKF->GetChilds()) {
      if (pChildKF && !pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    KeyFrame* pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }
  }

  // Add 20 last temporal KFs (mainly for IMU)
  if (mSensor.isInertial() && mvpLocalKeyFrames.size() < 80 && !mCurrentFrame.isPartiallyConstructed) {
    KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

    const int Nd = 20;
    for (int i = 0; i < Nd; i++) {
      if (!tempKeyFrame || tempKeyFrame->isPartiallyConstructed) break;
      if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(tempKeyFrame);
        tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        tempKeyFrame = tempKeyFrame->mPrevKF;
      }
    }
  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
  // Compute Bag of Words Vector
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for
  // relocalisation
  std::vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

  if (vpCandidateKFs.empty()) {
    Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  const int nKFs = vpCandidateKFs.size();

  // We perform first an ORB matching with each candidate
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.75, true); 

  std::vector<MLPnPsolver*> vpMLPnPsolvers;
  vpMLPnPsolvers.resize(nKFs);

  std::vector<std::vector<MapPoint*>> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  std::vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame* pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
      if (nmatches < 15) {
        vbDiscarded[i] = true;
        continue;
      } else {
        MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991);  // This solver needs at least 6 points
        vpMLPnPsolvers[i] = pSolver;
        nCandidates++;
      }
    }
  }

  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  ORBmatcher matcher2(0.9, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i]) continue;

      // Perform 5 Ransac Iterations
      std::vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      MLPnPsolver* pSolver = vpMLPnPsolvers[i];
      Eigen::Matrix4f eigTcw;
      bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      if (!bTcw) std::cout << "camera pose not calculated" << std::endl;

      // If a Camera Pose is computed, optimize
      if (bTcw) {
        Sophus::SE3f Tcw(eigTcw);
        mCurrentFrame.SetPose(Tcw);
        // Tcw.copyTo(mCurrentFrame.mTcw);

        std::set<MapPoint*> sFound;

        for (size_t j = 0; j < vbInliers.size(); j++) {
          if (vbInliers[j]) {
            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
            sFound.insert(vvpMapPointMatches[i][j]);
          } else
            mCurrentFrame.mvpMapPoints[j] = nullptr;
        }

        int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        if (nGood < 10) continue;

        for (int io = 0; io < mCurrentFrame.N; io++)
          if (mCurrentFrame.mvbOutlier[io])
            mCurrentFrame.mvpMapPoints[io] = nullptr;

        // If few inliers, search by projection in a coarse window and optimize again
        if (nGood < 50) {
          int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

          if (nadditional + nGood >= 50) {
            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

            // If many inliers but still not enough, search by projection again
            // in a narrower window the camera has been already optimized with
            // many points
            if (nGood > 30 && nGood < 50) {
              sFound.clear();
              for (int ip = 0; ip < mCurrentFrame.N; ip++)
                if (mCurrentFrame.mvpMapPoints[ip])
                  sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
              nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

              // Final optimization
              if (nGood + nadditional >= 50) {
                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                for (int io = 0; io < mCurrentFrame.N; io++)
                  if (mCurrentFrame.mvbOutlier[io])
                    mCurrentFrame.mvpMapPoints[io] = nullptr;
              }
            }
          }
        }

        // If the pose is supported by enough inliers stop ransacs and continue
        if (nGood >= 50) {
          bMatch = true;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    return false;
  } else {
    mnLastRelocFrameId = mCurrentFrame.mnId;
    std::cout << "Relocalized!!" << std::endl;
    return true;
  }
}

void Tracking::Reset(bool bLocMap) {
  std::unique_lock<std::mutex> lock(mMutexReset);

  Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

  // Reset Local Mapping
  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
    mpLocalMapper->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  mpLoopClosing->RequestReset();
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
  mpKeyFrameDB->clear();
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearAtlas();
  mpAtlas->CreateNewMap();
  mnInitialFrameId = 0;

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = TrackingState::NO_IMAGES_YET;

  mbReadyToInitialize = false;

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlbLost.clear();
  mCurrentFrame = Frame();
  mnLastRelocFrameId = 0;
  mLastFrame = Frame();
  mpReferenceKF = nullptr;
  mpLastKeyFrame = nullptr;
  mvIniMatches.clear();

  imuMotionModelPrepedAfterRecentlyLostTracking = false;
  mbReset = false;
  mbResetActiveMap = false;

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap) {
  std::unique_lock<std::mutex> lock(mMutexReset);
  
  Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);

  std::shared_ptr<Map> pMap = mpAtlas->GetCurrentMap();

  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
    mpLocalMapper->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  mpLoopClosing->RequestResetActiveMap(pMap);
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
  mpKeyFrameDB->clearMap(pMap);  // Only clear the active map references
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearMap();

  // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
  // Frame::nNextId = mnLastInitFrameId;
  //mnLastInitFrameId = Frame::nNextId;
  // mnLastRelocFrameId = mnLastInitFrameId;
  mState = TrackingState::NO_IMAGES_YET;  // TrackingState::NOT_INITIALIZED;

  mbReadyToInitialize = false;

  std::list<bool> lbLost;
  // lbLost.reserve(mlbLost.size());
  unsigned int index = mnFirstFrameId;
  std::cout << "mnFirstFrameId = " << mnFirstFrameId << std::endl;
  for (std::shared_ptr<Map> pMap : mpAtlas->GetAllMaps()) {
    if (pMap->GetAllKeyFrames().size() > 0) {
      if (index > pMap->GetLowerKFID()) index = pMap->GetLowerKFID();
    }
  }

  // std::cout << "First Frame id: " << index << std::endl;
  int num_lost = 0;
  std::cout << "mnInitialFrameId = " << mnInitialFrameId << std::endl;

  for (std::list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++) {
    if (index < mnInitialFrameId)
      lbLost.push_back(*ilbL);
    else {
      lbLost.push_back(true);
      num_lost += 1;
    }

    index++;
  }
  std::cout << num_lost << " Frames set to lost" << std::endl;

  mlbLost = lbLost;

  mnInitialFrameId = mCurrentFrame.mnId;
  mnLastRelocFrameId = mCurrentFrame.mnId;

  mCurrentFrame = Frame();
  mLastFrame = Frame();
  mpReferenceKF = nullptr;
  mpLastKeyFrame = nullptr;
  mvIniMatches.clear();

  imuMotionModelPrepedAfterRecentlyLostTracking = false;
  mbResetActiveMap = false;

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::InformOnlyTracking(const bool& flag) { mbOnlyTracking = flag; }

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias& b, KeyFrame* pCurrentKeyFrame) {
  std::shared_ptr<Map> pMap = pCurrentKeyFrame->GetMap();
  // unsigned int index = mnFirstFrameId; // UNUSED
  std::list<MORB_SLAM::KeyFrame*>::iterator lRit = mlpReferences.begin();
  std::list<bool>::iterator lbL = mlbLost.begin();

  if(s != 1.0f) {
    for (auto lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lbL++) {
      if (*lbL) continue;

      KeyFrame* pKF = *lRit;

      while (pKF && pKF->isBad()) {
        pKF = pKF->GetParent();
      }

      if(pKF == nullptr) continue;
      if (pKF->GetMap() == pMap) {
        (*lit).translation() *= s;
      }
    }
  }
  //mLastBias = b;

  mpLastKeyFrame = pCurrentKeyFrame;

  mLastFrame.SetNewBias(b);
  mCurrentFrame.SetNewBias(b);

  while (!mCurrentFrame.imuIsPreintegrated()) {
    usleep(500);
  }

  if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
    mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(), mLastFrame.mpLastKeyFrame->GetImuPosition(), mLastFrame.mpLastKeyFrame->GetVelocity());
  } else {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mLastFrame.mpImuPreintegrated->dT;

    mLastFrame.SetImuPoseVelocity(
      IMU::NormalizeRotation(Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
      twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
      Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }

  if (mCurrentFrame.mpImuPreintegrated) {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mCurrentFrame.mpImuPreintegrated->dT;

    mCurrentFrame.SetImuPoseVelocity(
        IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
        Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }
}

int Tracking::GetMatchesInliers() { return mnMatchesInliers; }

float Tracking::GetImageScale() { return mImageScale; }

void Tracking::setForcedLost(bool forceLost) { mForcedLost = forceLost; }

void Tracking::setStereoInitDefaultPose(const Sophus::SE3f default_pose) {
  mStereoInitDefaultPose = default_pose;
}

Sophus::SE3f Tracking::GetPoseRelativeToBase(Sophus::SE3f initialPose) {
  Eigen::Vector3f translation0 = initialPose.rotationMatrix().transpose()*initialPose.translation();
  Eigen::Vector3f translation1 = mCurrentFrame.GetPose().rotationMatrix().transpose()*mCurrentFrame.GetPose().translation()+mBaseTranslation;
  
  if(!mpLocalMapper->getIsDoneVIBA())
    translation1.setZero();
  return Sophus::SE3f(initialPose.rotationMatrix(), translation1);
}

void Tracking::RequestReset() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbReset = true;
}

void Tracking::RequestResetActiveMap() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbResetActiveMap = true;
}

// bool Tracking::ResetRequested() {
//   std::unique_lock<std::mutex> lock(mMutexReset);
//   return mbReset;
// }

// bool Tracking::ResetActiveMapRequested() {
//   std::unique_lock<std::mutex> lock(mMutexReset);
//   return mbResetActiveMap;
// }

void Tracking::ActivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void Tracking::DeactivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

void Tracking::CheckTrackingModeChanged() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  if (mbActivateLocalizationMode) {
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped()) {
      usleep(1000);
    }

    InformOnlyTracking(true);
    mbActivateLocalizationMode = false;
  }
  if (mbDeactivateLocalizationMode) {
    InformOnlyTracking(false);
    mpLocalMapper->Release();
    mbDeactivateLocalizationMode = false;
  }
}

void Tracking::CheckTrackingReset() {
  std::scoped_lock<std::mutex> lock(mMutexReset);
  if (mbReset) {
    Reset();
    mbReset = false;
    mbResetActiveMap = false;
  } else if (mbResetActiveMap) {
    ResetActiveMap();
    mbResetActiveMap = false;
  }
}

#ifdef REGISTER_LOOP
void Tracking::RequestStop() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  mbStopRequested = true;
}

bool Tracking::Stop() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  if (mbStopRequested && !mbNotStop) {
    mbStopped = true;
    std::cout << "Tracking STOP" << std::endl;
    return true;
  }
  return false;
}

bool Tracking::stopRequested() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  return mbStopRequested;
}

bool Tracking::isStopped() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  return mbStopped;
}

void Tracking::Release() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  mbStopped = false;
  mbStopRequested = false;
}
#endif

}  // namespace MORB_SLAM
