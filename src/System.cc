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

#include "MORB_SLAM/ImprovedTypes.hpp"
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

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const std::string &strVocFile, std::shared_ptr<CameraSettings> camSettings)
    : mSensor(camSettings->cameraType()),
      mpAtlas(std::make_shared<Atlas>(0)),
      mTrackingState(TrackingState::SYSTEM_NOT_READY),
      settings(camSettings) {

  cameras.push_back(std::make_shared<Camera>(mSensor)); // for now just hard code the sensor we are using, TODO make multicam
  // Output welcome message
  std::cout << "Input sensor was set to: " << mSensor << std::endl;
  
  // We're legally obligated to keep this line
  std::cout << std::endl << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << std::endl << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << std::endl << "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl << "This is free software, and you are welcome to redistribute it" << std::endl << "under certain conditions. See LICENSE.txt." << std::endl << std::endl;
  
  mStrLoadAtlasFromFile = settings->atlasLoadFile();
  mStrSaveAtlasToFile = settings->atlasSaveFile();

  bool activeLC = settings->activeLoopClosing();

  mStrVocabularyFilePath = strVocFile;

  bool isRead = false;

  // Load ORB Vocabulary
  std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;

  mpVocabulary = std::make_shared<ORBVocabulary>();
  bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  if (!bVocLoad) {
    std::cerr << "Wrong path to vocabulary. " << std::endl;
    std::cerr << "Failed to open at: " << strVocFile << std::endl;
    throw std::invalid_argument("Failed to open at: " + strVocFile);
  }
  std::cout << "Vocabulary loaded!" << std::endl << std::endl;

  // Create KeyFrame Database
  mpKeyFrameDatabase = std::make_shared<KeyFrameDatabase>(mpVocabulary);

  if (mStrLoadAtlasFromFile.empty()) {
    std::cout << "Initialization of Atlas from scratch " << std::endl;
  } else {
    // Load the file with an earlier session
    std::cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << std::endl;
    isRead = LoadAtlas(FileType::BINARY_FILE);

    if (!isRead) {
      std::cout << "Error to load the file, please try with other session file or vocabulary file" << std::endl;
      throw std::invalid_argument("Error to load the file, please try with other session file or vocabulary file");
    }
    mpAtlas->CreateNewMap();
  }

  mpTracker = std::make_shared<Tracking>(mpVocabulary, mpAtlas, mpKeyFrameDatabase, mSensor, settings);

  // Initialize the Tracking thread (it will live in the main thread of execution, the one that called this constructor)
  mpLocalMapper = std::make_shared<LocalMapping>(mpAtlas, mSensor == CameraType::MONOCULAR || mSensor == CameraType::IMU_MONOCULAR, mSensor.isInertial());
  
  // Do not axis flip when loading from existing atlas
  if (isRead) {
    mpLocalMapper->setIsDoneVIBA(true);
  }

  mpLocalMapper->mThFarPoints = settings->thFarPoints();
  if (mpLocalMapper->mThFarPoints != 0) {
    std::cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << std::endl;
    mpLocalMapper->mbFarPoints = true;
  } else {
    mpLocalMapper->mbFarPoints = false;
  }

  // Initialize the Loop Closing thread and launch
  mpLoopCloser = std::make_shared<LoopClosing>(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor != CameraType::MONOCULAR, activeLC, mSensor.isInertial());

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  std::cout << "Creating LocalMapping thread" << std::endl;
  mptLocalMapping = std::jthread(&MORB_SLAM::LocalMapping::Run, mpLocalMapper);

  std::cout << "Creating LoopClosing thread" << std::endl;
  mptLoopClosing = std::jthread(&MORB_SLAM::LoopClosing::Run, mpLoopCloser);

  // Fix verbosity
  Verbose::SetTh(Verbose::VERBOSITY_QUIET);
}

StereoPacket System::TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, double timestamp, const std::vector<IMU::Point>& vImuMeas) {
  if (mSensor != CameraType::STEREO && mSensor != CameraType::IMU_STEREO) {
    std::cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << std::endl;
    throw std::invalid_argument("ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial.");
  }

  cv::Mat imLeftToFeed, imRightToFeed;
  if (settings && settings->needToRectify()) {
    const cv::Mat &M1l = settings->M1l();
    const cv::Mat &M2l = settings->M2l();
    const cv::Mat &M1r = settings->M1r();
    const cv::Mat &M2r = settings->M2r();

    cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
  } else if (settings && settings->needToResize()) {
    cv::resize(imLeft, imLeftToFeed, settings->newImSize());
    cv::resize(imRight, imRightToFeed, settings->newImSize());
  } else {
    imLeftToFeed = imLeft;
    imRightToFeed = imRight;
  }

  // Check mode change
  mpTracker->CheckTrackingModeChanged();
  // Check reset
  mpTracker->CheckTrackingReset();

  if (mSensor == CameraType::IMU_STEREO)
    mpTracker->GrabImuData(vImuMeas);

  StereoPacket Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, cameras[0]); // for now we know cameras[0] is providing the image

  mTrackingState = mpTracker->mState;
  return Tcw;
}

RGBDPacket System::TrackRGBD(const cv::Mat& im, const cv::Mat& depthmap, double timestamp, const std::vector<IMU::Point>& vImuMeas) {
  if (mSensor != CameraType::RGBD && mSensor != CameraType::IMU_RGBD) {
    std::cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << std::endl;
    throw std::invalid_argument("ERROR: you called TrackRGBD but input sensor was not set to RGBD.");
  }

  cv::Mat imToFeed = im.clone();
  cv::Mat imDepthToFeed = depthmap.clone();
  if (settings && settings->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings->newImSize());
    imToFeed = resizedIm;
    cv::resize(depthmap, imDepthToFeed, settings->newImSize());
  }

  // Check mode change
  mpTracker->CheckTrackingModeChanged();
  // Check reset
  mpTracker->CheckTrackingReset();

  if (mSensor == CameraType::IMU_RGBD)
    mpTracker->GrabImuData(vImuMeas);

  RGBDPacket Tcw = mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, cameras[0]); // for now we know cameras[0] is providing the image

  mTrackingState = mpTracker->mState;
  return Tcw;
}

MonoPacket System::TrackMonocular(const cv::Mat& im, double timestamp, const std::vector<IMU::Point>& vImuMeas) {

  if (mSensor != CameraType::MONOCULAR && mSensor != CameraType::IMU_MONOCULAR) {
    std::cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << std::endl;
    throw std::invalid_argument("ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial.");
  }

  cv::Mat imToFeed = im.clone();
  if (settings && settings->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings->newImSize());
    imToFeed = resizedIm;
  }

  // Check mode change
  mpTracker->CheckTrackingModeChanged();
  // Check reset
  mpTracker->CheckTrackingReset();

  if (mSensor == CameraType::IMU_MONOCULAR)
    mpTracker->GrabImuData(vImuMeas);

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

System::~System() {
  std::cout << "Shutdown" << std::endl;

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
  std::cout << "Thread ID is: " << std::this_thread::get_id() << std::endl << "trying to save " << std::endl;
  if (!mStrSaveAtlasToFile.empty()) {
    Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_DEBUG);
    // Save the current session
    mpAtlas->PreSave();
    std::cout << "presaved" << std::endl;
    std::string pathSaveFileName = mStrSaveAtlasToFile;  

    // Create the folder if it does not exist
    std::filesystem::path fsPath = pathSaveFileName;
    fsPath = fsPath.parent_path();
    if(!std::filesystem::exists(fsPath)){
      std::filesystem::create_directory(fsPath);
    }

    auto time = std::chrono::system_clock::now();
    std::time_t time_time = std::chrono::system_clock::to_time_t(time);
    std::string str_time = std::ctime(&time_time);
    pathSaveFileName = pathSaveFileName.append(".osa");

    std::cout << "About to Calculate " << std::endl;

    std::string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

    std::cout << "Vocab checksum`" << strVocabularyChecksum << std::endl;
    std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
    std::string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);
    std::cout << "Type is of: " << type << std::endl;

    if (type == TEXT_FILE) {
      std::cout << "Starting to write the save text file " << std::endl;

      int rval = std::remove(pathSaveFileName.c_str());  // Deletes the file
      std::cout << "remove's output is: " << rval << std::endl;

      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::text_oarchive oa(ofs);

      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << SERIALIZED_ATLAS_FORMAT_VERSION;
      oa << *mpAtlas;
      std::cout << "End to write the save text file" << std::endl;
    } else if (type == BINARY_FILE) {
      std::cout << "Starting to write the save binary file" << std::endl;
      int rval = std::remove(pathSaveFileName.c_str());  // Deletes the file
      std::cerr << errno << std::endl;
      std::cout << "remove's output is: " << rval << std::endl;
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      std::cout << "big boostin' time" << std::endl;
      boost::archive::binary_oarchive oa(ofs);
      std::cout << "streaming" << std::endl;
      oa << strVocabularyName;
      std::cout << "streamed name" << std::endl;
      oa << strVocabularyChecksum;
      std::cout << "streamed checksum" << std::endl;
      oa << SERIALIZED_ATLAS_FORMAT_VERSION;
      std::cout << "streamed atlas format version" << std::endl;
      oa << *mpAtlas;
      std::cout << "End to write save binary file" << std::endl;
    } else {
      std::cout << "no file to be saved I guess lul" << std::endl;
    }
  }
}

bool System::LoadAtlas(int type) {
  std::string strFileVoc, strVocChecksum, strSerializedAtlasFormatVersion;
  bool isRead = false;

  std::string pathLoadFileName = "/";
  pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
  pathLoadFileName = pathLoadFileName.append(".osa");

  if (type == TEXT_FILE) {
    std::cout << "Starting to read the save text file " << std::endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      std::cout << "Load file not found" << std::endl;
      return false;
    }
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> strSerializedAtlasFormatVersion;
    if(strSerializedAtlasFormatVersion != SERIALIZED_ATLAS_FORMAT_VERSION) {
      std::cout << "ERROR: The format of the loaded Atlas is " << strSerializedAtlasFormatVersion << ", while the format required by this version of MORB-SLAM is " << SERIALIZED_ATLAS_FORMAT_VERSION << std::endl;
      throw std::invalid_argument("Error to load the file, please try with other session file or vocabulary file");
    }
    ia >> *mpAtlas;
    std::cout << "End to load the save text file " << std::endl;
    isRead = true;
  } else if (type == BINARY_FILE) {
    std::cout << "Starting to read the save binary file" << std::endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      std::cout << "Load file not found" << std::endl;
      return false;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> strSerializedAtlasFormatVersion;
    if(strSerializedAtlasFormatVersion != SERIALIZED_ATLAS_FORMAT_VERSION) {
      std::cout << "ERROR: The format of the loaded Atlas is " << strSerializedAtlasFormatVersion << ", while the format required by this version of MORB-SLAM is " << SERIALIZED_ATLAS_FORMAT_VERSION << std::endl;
      throw std::invalid_argument("Error to load the file, please try with other session file or vocabulary file");
    }
    ia >> *mpAtlas;
    std::cout << "End to load the save binary file" << std::endl;
    isRead = true;
  }

  if (isRead) {
    // Check if the vocabulary is the same
    std::string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      std::cout << "The vocabulary load isn't the same which the load session was created " << std::endl;
      std::cout << "-Vocabulary name: " << strFileVoc << std::endl;
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

  std::cout << "inside" << std::endl;

  std::ifstream f(filename.c_str(), flags);
  if (!f.is_open()) {
    std::cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << std::endl;
    return checksum;
  }

  EVP_MD_CTX *mdctx;
  
  char buffer[1024];

  mdctx = EVP_MD_CTX_new();
  EVP_DigestInit_ex(mdctx, EVP_md5(), NULL);

  std::cout << "just initialized MD5" << std::endl;
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    EVP_DigestUpdate(mdctx, buffer, count);
  }
  std::cout << "about to close" << std::endl;

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

bool System::getIsDoneVIBA() {
  return mpLocalMapper->getIsDoneVIBA();
}

std::shared_ptr<CameraSettings> System::getSettings() const { return settings; }

// Bonk
void System::ForceLost() { mpTracker->setForcedLost(true); }

bool System::getIsLoopClosed() { return mpLoopCloser->loopClosed; }

void System::setIsLoopClosed(bool isLoopClosed) { mpLoopCloser->loopClosed = isLoopClosed; }

void System::RequestSystemReset() { mpTracker->RequestSystemReset(); }

Sophus::SE3f System::GetInitialFramePose() { return mpTracker->mInitialFramePose; }

bool System::HasInitialFramePose() { return mpTracker->mHasGlobalOriginPose; }

}  // namespace MORB_SLAM
