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

#include "MORB_SLAM/System.h"

#include <openssl/evp.h>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>
#include <string>
#include <iostream>
#include <filesystem>

#include "MORB_SLAM/Converter.h"

namespace MORB_SLAM {

Verbose::eLevel Verbose::th = Verbose::INFO;

System::System(const std::string &strVocFile, std::shared_ptr<SystemSettings> sysSettings, std::shared_ptr<CameraSettings> camSettings, const std::shared_ptr<Odometry> &odomSource)
    : mSensor(camSettings->cameraType()),
      mpAtlas(std::make_shared<Atlas>(0)),
      mTrackingState(TrackingState::SYSTEM_NOT_READY),
      mpCamSettings(camSettings),
      mpSysSettings(sysSettings) {
  
  Verbose::SetTh(Verbose::DEBUG);

  cameras.push_back(std::make_shared<Camera>(mSensor)); // for now just hard code the sensor we are using, TODO make multicam
  // Output welcome message
  Verbose::Log(Verbose::INFO, "Input sensor was set to: ", mSensor);
  
  // We're legally obligated to keep this line
  Verbose::Log(Verbose::INFO, "\n\nORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.\nORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.\nThis program comes with ABSOLUTELY NO WARRANTY;\nThis is free software, and you are welcome to redistribute it\nunder certain conditions. See LICENSE.txt.\n");
  
  mStrLoadAtlasFromFile = mpSysSettings->atlasLoadFile();
  mStrSaveAtlasToFile = mpSysSettings->atlasSaveFile();

  bool activeLC = mpSysSettings->activeLoopClosing();

  mStrVocabularyFilePath = strVocFile;

  bool isRead = false;

  // Load ORB Vocabulary
  Verbose::Log(Verbose::INFO, "Loading ORB Vocabulary. This could take a while...");

  mpVocabulary = std::make_shared<ORBVocabulary>();
  bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  if (!bVocLoad) {
    Verbose::Log(Verbose::FATAL, "Failed to open vocabulary at: ", strVocFile);
    throw std::invalid_argument("Failed to open at: " + strVocFile);
  }
  Verbose::Log(Verbose::SUCCESS, "Vocabulary loaded!");

  // Create KeyFrame Database
  mpKeyFrameDatabase = std::make_shared<KeyFrameDatabase>(mpVocabulary);

  if (mStrLoadAtlasFromFile.empty()) {
    Verbose::Log(Verbose::INFO, "Initialization of Atlas from scratch");
  } else {
    // Load the file with an earlier session
    Verbose::Log(Verbose::INFO, "Initialization of Atlas from file: ", mStrLoadAtlasFromFile);
    isRead = LoadAtlas(FileType::BINARY_FILE);

    if (!isRead) {
      Verbose::Log(Verbose::FATAL, "Error to load the file, please try with other session file or vocabulary file");
      throw std::invalid_argument("Error to load the file, please try with other session file or vocabulary file");
    }
    mpAtlas->CreateNewMap();
  }

  // Initialize the Tracking thread (it will live in the main thread of execution, the one that called this constructor)
  mpTracker = std::make_shared<Tracking>(mpVocabulary, mpAtlas, mpKeyFrameDatabase, mSensor, mpSysSettings, mpCamSettings, odomSource);

  mpLocalMapper = std::make_shared<LocalMapping>(mpAtlas, mSensor == CameraType::MONOCULAR || mSensor == CameraType::IMU_MONOCULAR, odomSource);
  
  // Do not axis flip when loading from existing atlas
  if (isRead) {
    mpLocalMapper->setIsDoneBA(true);
  }

  mpLocalMapper->mThFarPoints = mpSysSettings->thFarPoints();
  if (mpLocalMapper->mThFarPoints != 0) {
    Verbose::Log(Verbose::INFO, "Discard points further than ", mpLocalMapper->mThFarPoints, " m from current camera");
    mpLocalMapper->mbFarPoints = true;
  } else {
    mpLocalMapper->mbFarPoints = false;
  }

  // Initialize the Loop Closing thread and launch
  mpLoopCloser = std::make_shared<LoopClosing>(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor != CameraType::MONOCULAR, activeLC, odomSource);

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  // Set up Odometry
  if(odomSource) {
    odomSource->SetLocalMapper(mpLocalMapper);
    odomSource->SetTracker(mpTracker);
    odomSource->SetAtlas(mpAtlas);
  }

  Verbose::Log(Verbose::DEBUG, "Creating LocalMapping thread");
  mptLocalMapping = std::jthread(&MORB_SLAM::LocalMapping::Run, mpLocalMapper);

  Verbose::Log(Verbose::DEBUG, "Creating LoopClosing thread");
  mptLoopClosing = std::jthread(&MORB_SLAM::LoopClosing::Run, mpLoopCloser);
}

StereoPacket System::TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, double timestamp) {
  if (mSensor != CameraType::STEREO && mSensor != CameraType::IMU_STEREO) {
    Verbose::Log(Verbose::FATAL, "You called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial.");
    throw std::invalid_argument("ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial.");
  }

  cv::Mat imLeftToFeed, imRightToFeed;
  if (mpCamSettings && mpCamSettings->needToRectify()) {
    const cv::Mat &M1l = mpCamSettings->M1l();
    const cv::Mat &M2l = mpCamSettings->M2l();
    const cv::Mat &M1r = mpCamSettings->M1r();
    const cv::Mat &M2r = mpCamSettings->M2r();

    cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
  } else if (mpCamSettings && mpCamSettings->needToResize()) {
    cv::resize(imLeft, imLeftToFeed, mpCamSettings->newImSize());
    cv::resize(imRight, imRightToFeed, mpCamSettings->newImSize());
  } else {
    imLeftToFeed = imLeft;
    imRightToFeed = imRight;
  }

  // Check mode change
  mpTracker->CheckTrackingModeChanged();
  // Check reset
  mpTracker->CheckTrackingReset();

  StereoPacket Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, cameras[0]); // for now we know cameras[0] is providing the image

  mTrackingState = mpTracker->mState;
  return Tcw;
}

RGBDPacket System::TrackRGBD(const cv::Mat& im, const cv::Mat& depthmap, double timestamp) {
  if (mSensor != CameraType::RGBD && mSensor != CameraType::IMU_RGBD) {
    Verbose::Log(Verbose::FATAL, "You called TrackRGBD but input sensor was not set to RGBD.");
    throw std::invalid_argument("ERROR: you called TrackRGBD but input sensor was not set to RGBD.");
  }

  cv::Mat imToFeed = im.clone();
  cv::Mat imDepthToFeed = depthmap.clone();
  if (mpCamSettings && mpCamSettings->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, mpCamSettings->newImSize());
    imToFeed = resizedIm;
    cv::resize(depthmap, imDepthToFeed, mpCamSettings->newImSize());
  }

  // Check mode change
  mpTracker->CheckTrackingModeChanged();
  // Check reset
  mpTracker->CheckTrackingReset();

  RGBDPacket Tcw = mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, cameras[0]); // for now we know cameras[0] is providing the image

  mTrackingState = mpTracker->mState;
  return Tcw;
}

MonoPacket System::TrackMonocular(const cv::Mat& im, double timestamp) {

  if (mSensor != CameraType::MONOCULAR && mSensor != CameraType::IMU_MONOCULAR) {
    Verbose::Log(Verbose::FATAL, "You called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial.");
    throw std::invalid_argument("ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial.");
  }

  cv::Mat imToFeed = im.clone();
  if (mpCamSettings && mpCamSettings->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, mpCamSettings->newImSize());
    imToFeed = resizedIm;
  }

  // Check mode change
  mpTracker->CheckTrackingModeChanged();
  // Check reset
  mpTracker->CheckTrackingReset();

  MonoPacket Tcw = mpTracker->GrabImageMonocular(imToFeed, timestamp, cameras[0]); // for now we know cameras[0] is providing the image

  mTrackingState = mpTracker->mState;
  return Tcw;
}

bool System::MapChanged() {
  static int n = 0;
  int curn = mpAtlas->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else {
    return false;
  }
}

bool System::isMapMature() const {
  return mpAtlas->GetCurrentMap()->isMature();
}

System::~System() {
  Verbose::Log(Verbose::DEBUG, "Shutdown");

  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();

  if (mptLocalMapping.joinable()) {
    mptLocalMapping.join();
  }
  if (mptLoopClosing.joinable()) {
    mptLoopClosing.join();
  }
}

TrackingState System::GetTrackingState() { return mTrackingState; }

void System::SaveAtlas(int type) const {
  Verbose::Log(Verbose::DEBUG, "Thread ID is: ",  std::this_thread::get_id(),  ". Trying to save");
  if (!mStrSaveAtlasToFile.empty()) {
    Verbose::Log(Verbose::INFO, "Atlas saving to file ", mStrSaveAtlasToFile);
    // Save the current session
    mpAtlas->PreSave();
    Verbose::Log(Verbose::DEBUG, "presaved");
    std::string pathSaveFileName = mStrSaveAtlasToFile;  

    // Create the folder if it does not exist
    std::filesystem::path fsPath = pathSaveFileName;
    fsPath = fsPath.parent_path();
    if(!fsPath.empty() && !std::filesystem::exists(fsPath)){
      std::filesystem::create_directory(fsPath);
    }

    auto time = std::chrono::system_clock::now();
    std::time_t time_time = std::chrono::system_clock::to_time_t(time);
    std::string str_time = std::ctime(&time_time);
    pathSaveFileName = pathSaveFileName.append(".osa");

    
    Verbose::Log(Verbose::DEBUG, "About to Calculate");

    std::string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

    Verbose::Log(Verbose::DEBUG, "Vocab checksum`", strVocabularyChecksum);
    std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
    std::string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);
    Verbose::Log(Verbose::DEBUG, "Type is of: ", type);

    if (type == TEXT_FILE) {
      Verbose::Log(Verbose::DEBUG, "Starting to write the save text file");

      int rval = std::remove(pathSaveFileName.c_str());  // Deletes the file
      Verbose::Log(Verbose::DEBUG, "remove's output is: ", rval);

      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::text_oarchive oa(ofs);

      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << SERIALIZED_ATLAS_FORMAT_VERSION;
      oa << *mpAtlas;
      Verbose::Log(Verbose::DEBUG, "End to write the save text file");
    } else if (type == BINARY_FILE) {
      Verbose::Log(Verbose::DEBUG, "Starting to write the save binary file");
      int rval = std::remove(pathSaveFileName.c_str());  // Deletes the file
      Verbose::Log(Verbose::DEBUG, "remove's output is: ", rval);
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      Verbose::Log(Verbose::DEBUG, "big boostin' time");
      boost::archive::binary_oarchive oa(ofs);
      Verbose::Log(Verbose::DEBUG, "streaming");
      oa << strVocabularyName;
      Verbose::Log(Verbose::DEBUG, "streamed name");
      oa << strVocabularyChecksum;
      Verbose::Log(Verbose::DEBUG, "streamed checksum");
      oa << SERIALIZED_ATLAS_FORMAT_VERSION;
      Verbose::Log(Verbose::DEBUG, "streamed atlas format version");
      oa << *mpAtlas;
      Verbose::Log(Verbose::SUCCESS, "Atlas saved to file ", mStrSaveAtlasToFile);
    } else {
      Verbose::Log(Verbose::CRITICAL, "Invalid Atlas Save File Type");
    }
  } else 
    Verbose::Log(Verbose::CRITICAL, "No Atlas Save File is Set");
}

bool System::LoadAtlas(int type) {
  std::string strFileVoc, strVocChecksum, strSerializedAtlasFormatVersion;
  bool isRead = false;

  std::string pathLoadFileName = mStrLoadAtlasFromFile;
  pathLoadFileName = pathLoadFileName.append(".osa");

  if (type == TEXT_FILE) {
    Verbose::Log(Verbose::INFO, "Reading the saved Atlas text file");
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      Verbose::Log(Verbose::CRITICAL, "Load file not found");
      return false;
    }
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> strSerializedAtlasFormatVersion;
    if(strSerializedAtlasFormatVersion != SERIALIZED_ATLAS_FORMAT_VERSION) {
      Verbose::Log(Verbose::FATAL, "The format of the loaded Atlas is ", strSerializedAtlasFormatVersion, ", while the format required by this version of MORB-SLAM is ", SERIALIZED_ATLAS_FORMAT_VERSION);
      throw std::invalid_argument("Error to load the file, please try with other session file or vocabulary file");
    }
    ia >> *mpAtlas;
    Verbose::Log(Verbose::SUCCESS, "Atlas text file loaded" );
    isRead = true;
  } else if (type == BINARY_FILE) {
    Verbose::Log(Verbose::INFO, "Reading the saved Atlas binary file");
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      Verbose::Log(Verbose::CRITICAL, "Load file not found");
      return false;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> strSerializedAtlasFormatVersion;
    if(strSerializedAtlasFormatVersion != SERIALIZED_ATLAS_FORMAT_VERSION) {
      Verbose::Log(Verbose::FATAL, "The format of the loaded Atlas is ", strSerializedAtlasFormatVersion, ", while the format required by this version of MORB-SLAM is ", SERIALIZED_ATLAS_FORMAT_VERSION);
      throw std::invalid_argument("Error to load the file, please try with other session file or vocabulary file");
    }
    ia >> *mpAtlas;
    Verbose::Log(Verbose::SUCCESS, "Atlas binary file loaded");
    isRead = true;
  }

  if (isRead) {
    // Check if the vocabulary is the same
    std::string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      Verbose::Log(Verbose::CRITICAL, "The vocabulary load isn't the same as when it was created");
      Verbose::Log(Verbose::DEBUG, "-Vocabulary name: ", strFileVoc);
      return false;
    }

    mpAtlas->SetKeyFrameDatabase(mpKeyFrameDatabase);
    mpAtlas->SetORBVocabulary(mpVocabulary);
    mpAtlas->PostLoad();

    return true;
  }
  return false;
}

std::string System::CalculateCheckSum(std::string filename, int type) const {
  std::string checksum = "";

  unsigned int md5_digest_len = EVP_MD_size(EVP_md5());
  unsigned char *md5_digest;

  std::ios_base::openmode flags = std::ios::in;
  if (type == BINARY_FILE)  // Binary file
    flags = std::ios::in | std::ios::binary;

  Verbose::Log(Verbose::DEBUG, "start checksum");

  std::ifstream f(filename.c_str(), flags);
  if (!f.is_open()) {
    Verbose::Log(Verbose::CRITICAL, "Unable to open the in file ", filename, " for Md5 hash.");
    return checksum;
  }

  EVP_MD_CTX *mdctx;
  
  char buffer[1024];

  mdctx = EVP_MD_CTX_new();
  EVP_DigestInit_ex(mdctx, EVP_md5(), NULL);

  Verbose::Log(Verbose::DEBUG, "just initialized MD5");
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    EVP_DigestUpdate(mdctx, buffer, count);
  }
  Verbose::Log(Verbose::DEBUG, "about to close");

  f.close();

  md5_digest = (unsigned char *)OPENSSL_malloc(md5_digest_len);
  EVP_DigestFinal_ex(mdctx, md5_digest, &md5_digest_len);
  EVP_MD_CTX_free(mdctx);

  for (int i = 0; i < md5_digest_len; i++) {
    char aux[10];
    sprintf(aux, "%02x", md5_digest[i]);
    checksum = checksum + aux;
  }

  return checksum;
}

bool System::getHasMergedLocalMap() { 
  return mpLoopCloser->hasMergedLocalMap; 
}

bool System::getIsDoneBA() {
  return mpLocalMapper->getIsDoneBA();
}

std::shared_ptr<SystemSettings> System::getSysSettings() const { return mpSysSettings; }
std::shared_ptr<CameraSettings> System::getCamSettings() const { return mpCamSettings; }

// Bonk
void System::ForceLost() { mpTracker->setForcedLost(true); }

bool System::getIsLoopClosed() { return mpLoopCloser->loopClosed; }

void System::setIsLoopClosed(bool isLoopClosed) { mpLoopCloser->loopClosed = isLoopClosed; }

void System::RequestSystemReset() { mpTracker->RequestSystemReset(); }

Sophus::SE3f System::GetInitialFramePose() { return mpTracker->mInitialFramePose; }

bool System::HasInitialFramePose() { return mpTracker->mHasGlobalOriginPose; }

}  // namespace MORB_SLAM
