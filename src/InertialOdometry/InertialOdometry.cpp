#include "MORB_SLAM/InertialOdometry/InertialOdometry.hpp"
#include "MORB_SLAM/Verbose.h"
#include "MORB_SLAM/InertialOdometry/InertialOptimizer.hpp"
#include "MORB_SLAM/Optimizer.h"
#include "MORB_SLAM/Atlas.h"
#include <assert.h>


#include <iostream>

namespace MORB_SLAM {


InertialOdometry::InertialOdometry(std::shared_ptr<InertialOdometrySettings> settings, const CameraType &cam)
    : mbMonocular(cam == CameraType::MONOCULAR || cam == CameraType::IMU_MONOCULAR),
      mRwg(Eigen::Matrix3d::Identity()),
      mScale(1.0),
      mbStationaryInitEnabled(settings->stationaryIMUInit()),
      mbFastInit(settings->fastIMUInit()) {
    newParameterLoader(*settings);
}

void InertialOdometry::newParameterLoader(InertialOdometrySettings &settings) {
    Sophus::SE3f Tbc = settings.Tbc();
    float Ng = settings.noiseGyro();
    float Na = settings.noiseAcc();
    float Ngw = settings.gyroWalk();
    float Naw = settings.accWalk();

    const float sf_a = sqrt(settings.accFrequency());
    const float sf_g = sqrt(settings.gyroFrequency());
    mpImuCalib = std::make_shared<IMU::Calib>(Tbc, Ng * sf_g, Na * sf_a, Ngw / sf_g, Naw / sf_a);

    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
}

std::shared_ptr<ExternalFrameData> InertialOdometry::DefaultExternalFrameData()  {
    return std::make_shared<InertialFrameData>(*mpImuCalib);
}

std::shared_ptr<ExternalFrameData> InertialOdometry::DefaultExternalFrameData(std::shared_ptr<KeyFrame> p_latest_kf) {
    std::shared_ptr<InertialFrameData> ed = std::make_shared<InertialFrameData>(*mpImuCalib);
    
    // If the ExternalData on the KeyFrame exists, copy over the necessary data
    if(std::shared_ptr<InertialKeyFrameData> external_kf_data = p_latest_kf->External<InertialKeyFrameData>()) {
        ed->SetNewBias(external_kf_data->GetImuBias());
    }
    
    return ed;
}

std::shared_ptr<ExternalKeyFrameData> InertialOdometry::DefaultExternalKeyFrameData(Frame& curr_frame) {
    // Construct ExternalKeyFrameData using ExternalFrameData if it exists
    if(std::shared_ptr<InertialFrameData> ed = curr_frame.External<InertialFrameData>())
        return std::make_shared<InertialKeyFrameData>(*ed);
    else
        return std::make_shared<InertialKeyFrameData>(*mpImuCalib);
}

void InertialOdometry::AddAccel(const Eigen::Vector3f &accel_meas, const double timestamp_s) {
    std::scoped_lock lock(mMutexAccel);
    mvAccelQueue.push_back(accel_meas);
    mvAccelTimestampQueue.push_back(timestamp_s);
}


void InertialOdometry::AddAccel(const std::vector<Eigen::Vector3f> &v_accel_meas, const std::vector<double> v_timestamp_s) {
    std::scoped_lock lock(mMutexAccel);
    if(v_accel_meas.size() != v_timestamp_s.size()) {
        std::cerr << "ERROR: Could not add batch of accel measurements, the number of timestamps and accel measurements don't match" << std::endl;
        return;
    }
    mvAccelQueue.insert(mvAccelQueue.end(), v_accel_meas.begin(), v_accel_meas.end());
    mvAccelTimestampQueue.insert(mvAccelTimestampQueue.end(), v_timestamp_s.begin(), v_timestamp_s.end());
}

void InertialOdometry::AddGyro(const Eigen::Vector3f &gyro_meas, const double timestamp_s) {
    std::scoped_lock lock(mMutexGyro);
    mvGyroQueue.push_back(gyro_meas);
    mvGyroTimestampQueue.push_back(timestamp_s);
}

void InertialOdometry::AddGyro(const std::vector<Eigen::Vector3f> &v_gyro_meas, const std::vector<double> v_timestamp_s) {
    std::scoped_lock lock(mMutexGyro);
    if(v_gyro_meas.size() != v_timestamp_s.size()) {
        std::cerr << "ERROR: Could not add batch of gyro measurements, the number of timestamps and gyro measurements don't match" << std::endl;
        return;
    }
    mvGyroQueue.insert(mvGyroQueue.end(), v_gyro_meas.begin(), v_gyro_meas.end());
    mvGyroTimestampQueue.insert(mvGyroTimestampQueue.end(), v_timestamp_s.begin(), v_timestamp_s.end());
}

bool InertialOdometry::GrabOdom(double curr_timestamp, double prev_timestamp) {      
    mvImuBatch.clear();
    std::vector<Eigen::Vector3f> curr_accel_batch;
    std::vector<double> curr_accel_timestamp_batch;
    std::vector<Eigen::Vector3f> curr_gyro_batch;
    std::vector<double> curr_gyro_timestamp_batch;
    {
        std::scoped_lock (mMutexAccel, mMutexGyro);

        // Ensure the queues are synchronized... This if statement never returns true.
        if(mvAccelQueue.size() != mvAccelTimestampQueue.size()|| mvGyroQueue.size() != mvGyroTimestampQueue.size()) {
            std::cerr << "ERROR: Could not grab odom measurements. The number of timestamps and IMU measurements don't match. Number of accel measurements: " << mvAccelQueue.size() << ". Number of accel timestamps: " << mvAccelTimestampQueue.size()
                        << ". Number of gyro measurements: " << mvGyroQueue.size() << ". Number of gyro timestamps: " << mvGyroTimestampQueue.size() << "." << std::endl;
            mvAccelQueue.clear();
            mvAccelTimestampQueue.clear();
            mvGyroQueue.clear();
            mvGyroTimestampQueue.clear();
            return false;
        }

        // Grab accel measurements from the queue that occurred between the prev_timestamp and curr_timestamp
        auto accel_timestamp_lower_it = std::lower_bound(mvAccelTimestampQueue.begin(), mvAccelTimestampQueue.end(), prev_timestamp);
        auto accel_timestamp_upper_it = std::upper_bound(mvAccelTimestampQueue.begin(), mvAccelTimestampQueue.end(), curr_timestamp);
        auto accel_lower_idx = std::distance(mvAccelTimestampQueue.begin(), accel_timestamp_lower_it);
        auto accel_upper_idx = std::distance(mvAccelTimestampQueue.begin(), accel_timestamp_upper_it);
        curr_accel_batch.resize(accel_upper_idx - accel_lower_idx);
        curr_accel_timestamp_batch.resize(accel_upper_idx - accel_lower_idx);
        std::move(mvAccelQueue.begin() + accel_lower_idx, mvAccelQueue.begin() + accel_upper_idx, curr_accel_batch.begin());
        std::move(accel_timestamp_lower_it, accel_timestamp_upper_it, curr_accel_timestamp_batch.begin());
        mvAccelQueue.erase(mvAccelQueue.begin(), mvAccelQueue.begin() + accel_upper_idx);
        mvAccelTimestampQueue.erase(mvAccelTimestampQueue.begin(), accel_timestamp_upper_it);

        // Grab gyro measurements from the queue that occurred between the prev_timestamp and curr_timestamp
        auto gyro_timestamp_lower_it = std::lower_bound(mvGyroTimestampQueue.begin(), mvGyroTimestampQueue.end(), prev_timestamp);
        auto gyro_timestamp_upper_it = std::upper_bound(mvGyroTimestampQueue.begin(), mvGyroTimestampQueue.end(), curr_timestamp);
        auto gyro_lower_idx = std::distance(mvGyroTimestampQueue.begin(), gyro_timestamp_lower_it);
        auto gyro_upper_idx = std::distance(mvGyroTimestampQueue.begin(), gyro_timestamp_upper_it);
        curr_gyro_batch.resize(gyro_upper_idx - gyro_lower_idx);
        curr_gyro_timestamp_batch.resize(gyro_upper_idx - gyro_lower_idx);
        std::move(mvGyroQueue.begin() + gyro_lower_idx, mvGyroQueue.begin() + gyro_upper_idx, curr_gyro_batch.begin());
        std::move(gyro_timestamp_lower_it, gyro_timestamp_upper_it, curr_gyro_timestamp_batch.begin());
        mvGyroQueue.erase(mvGyroQueue.begin(), mvGyroQueue.begin() + gyro_upper_idx);
        mvGyroTimestampQueue.erase(mvGyroTimestampQueue.begin(), gyro_timestamp_upper_it);
    }

    size_t n_accel = curr_accel_batch.size();
    size_t n_gyro = curr_gyro_batch.size();
    size_t n_timestamp_accel = curr_accel_timestamp_batch.size();
    size_t n_timestamp_gyro = curr_gyro_timestamp_batch.size();

    if(n_accel == 0 || n_gyro == 0) {
        std::cerr << "ERROR: No accel measurements or gyro measurements occured between the previous and current frame." << std::endl; // atleast one accel AND gyro need to be provided
        return false;
    }

    std::vector<IMU::Point> v_accel_interpolated = interpolateImu(curr_accel_batch, curr_accel_timestamp_batch, curr_timestamp, prev_timestamp, true);
    std::vector<IMU::Point> v_gyro_interpolated = interpolateImu(curr_gyro_batch, curr_gyro_timestamp_batch, curr_timestamp, prev_timestamp, false);
    if(v_accel_interpolated.empty() || v_gyro_interpolated.empty()) {
        std::cerr << "ERROR: Interpolation of IMU measurements failed." << std::endl;
        return false;
    }

    combineImu(v_accel_interpolated, v_gyro_interpolated, mvImuBatch);

    return true;
}

std::vector<IMU::Point> InertialOdometry::interpolateImu(const std::vector<Eigen::Vector3f> &v_imu, const std::vector<double> &v_imu_timestamp_s, const double &curr_frame_timestamp_s, const double &prev_frame_timestamp_s, const bool &is_accel) const {
    std::vector<IMU::Point> imu_interpolated; // to return
    std::string imu_type = is_accel ? "Accel" : "Gyro";
    size_t n_imu = v_imu.size();

    if(n_imu == 1) {
        double t_step = curr_frame_timestamp_s - prev_frame_timestamp_s;
        imu_interpolated.push_back(IMU::Point(v_imu[0], t_step, is_accel));
        return imu_interpolated;
    } else if (n_imu == 2) {
        double t_step = curr_frame_timestamp_s - prev_frame_timestamp_s;
        imu_interpolated.push_back(IMU::Point((v_imu[0] + v_imu[n_imu-1]) * 0.5, t_step, is_accel));
        return imu_interpolated;
    }

    for(int i = 0; i < n_imu-1; ++i) {
        double t_ab = v_imu_timestamp_s[i+1] -  v_imu_timestamp_s[i]; // time between curr IMU meas and the next IMU meas

        if(t_ab == 0) {
            std::cout << "WARNING: Two consecutive " << imu_type << " measurements have the same timestamp. Skipping iteration..." << std::endl;
            continue;
        }

        double t_step;
        Eigen::Vector3f data;

        if(i == 0) { // first iter
            double t_ini = v_imu_timestamp_s[i] - prev_frame_timestamp_s; // time from prev camera frame to curr IMU meas
            data = (v_imu[i+1] + v_imu[i] - (v_imu[i+1] - v_imu[i])*(t_ini/t_ab)) * 0.5;
            t_step = v_imu_timestamp_s[i+1] - prev_frame_timestamp_s;
        } else if(i < n_imu-2) {
            data = (v_imu[i+1] + v_imu[i]) * 0.5;
            t_step = t_ab;
        } else { // last iter
            double t_end = curr_frame_timestamp_s - v_imu_timestamp_s[i+1]; // time from next IMU meas to curr camera frame
            data = (v_imu[i+1] + v_imu[i] - (v_imu[i+1] - v_imu[i])*(t_end/t_ab)) * 0.5;
            t_step = curr_frame_timestamp_s - v_imu_timestamp_s[i];
        }

        imu_interpolated.push_back(IMU::Point(data, t_step, is_accel));
    }

    return std::move(imu_interpolated);
}

void InertialOdometry::combineImu(std::vector<IMU::Point> &v_accel, std::vector<IMU::Point>& v_gyro, std::vector<IMU::Point> &v_imu_combined) {
    size_t n_accel = v_accel.size();
    size_t n_gyro = v_gyro.size();
    v_imu_combined.resize(n_accel + n_gyro);

    size_t accel_idx = 0, gyro_idx = 0, imu_combined_idx = 0;

    while(accel_idx < n_accel && gyro_idx < n_gyro) {
        if(v_accel[accel_idx].t <= v_gyro[gyro_idx].t) {
            v_imu_combined[imu_combined_idx] = v_accel[accel_idx];
            ++accel_idx;
        } else {
            v_imu_combined[imu_combined_idx] = v_gyro[gyro_idx];
            ++gyro_idx;
        }
        ++imu_combined_idx;
    }

    while(accel_idx < n_accel) {
        v_imu_combined[imu_combined_idx] = v_accel[accel_idx];
        ++accel_idx;
        ++imu_combined_idx;
    }

    while(gyro_idx < n_gyro) {
        v_imu_combined[imu_combined_idx] = v_gyro[gyro_idx];
        ++gyro_idx;
        ++imu_combined_idx;
    }
}

bool InertialOdometry::PreintegrateOdom(Frame &curr_frame, Frame &last_frame, std::shared_ptr<KeyFrame> last_kf) {
  // This should never return true because ExternalData is assigned to the Frame at the start of the tracking loop.
  // The only case it returns true is if Odometry::DefaultExternalFrameData() returns a nullptr
  auto curr_frame_ed = curr_frame.External<InertialFrameData>();
  assert(curr_frame_ed != nullptr);
  
  if (!curr_frame.mpPrevFrame || curr_frame.mpPrevFrame->isPartiallyConstructed) {
    curr_frame_ed->setIntegrated();
    return false;
  }

  if (mvImuBatch.size() == 0) {
    Verbose::PrintMess("No IMU data in mvImuBatch!! Did not preintegrate.", Verbose::VERBOSITY_NORMAL);
    curr_frame_ed->setIntegrated();
    return false;
  }
  
  auto last_frame_ed = last_frame.External<InertialFrameData>();
  assert(last_frame_ed != nullptr);

  std::shared_ptr<IMU::Preintegrated> pImuPreintegratedFromLastFrame = std::make_shared<IMU::Preintegrated>(last_frame_ed->mImuBias, *mpImuCalib);
  bool hasPreintKF = pImuPreintegratedFromLastFrame->IntegrateMeasurements(mvImuBatch);

  if(hasPreintKF) {
    mpImuPreintegratedFromLastKF->IntegrateMeasurements(mvImuBatch);
    curr_frame_ed->mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    curr_frame_ed->mpImuPreintegrated = mpImuPreintegratedFromLastKF;
  } else {
    Verbose::PrintMess("mvImuBatch is missing either accel or gyro stream", Verbose::VERBOSITY_NORMAL);
  }
  curr_frame_ed->setIntegrated();
  return hasPreintKF;
}

bool InertialOdometry::ReadyForStereoInitialization(Frame &curr_frame, Frame &last_frame) {
    auto last_ed = last_frame.External<InertialFrameData>();
    auto curr_ed = curr_frame.External<InertialFrameData>();
    assert(curr_ed!=nullptr && last_ed!=nullptr);

    if (!curr_ed->mpImuPreintegrated || !last_ed->mpImuPreintegrated)
      return false;

    if (!mbStationaryInitEnabled && (mpAtlas->CountMaps() <= 1) && (curr_ed->mpImuPreintegratedFrame->avgA - last_ed->mpImuPreintegratedFrame->avgA).norm() < 0.5) {
      std::cout << "More acceleration is required to initialize the Map" << std::endl;
      return false;
    }

    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
    curr_ed->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

    return true;
}

bool InertialOdometry::ReadyForMonocularInitialization(Frame &curr_frame, Frame &last_frame) {
    // TODO: Monocular case.
    assert(curr_frame.External<InertialFrameData>()->mpImuPreintegrated!=nullptr);
    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
    curr_frame.External<InertialFrameData>()->mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    return true;
}

void InertialOdometry::InitialMapMonocular(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> initial_kf) {
    // TODO: Monocular case.
    auto curr_kf_ed = curr_kf->External<InertialKeyFrameData>();
    auto initial_kf_ed = initial_kf->External<InertialKeyFrameData>();
    assert(curr_kf_ed!=nullptr && initial_kf!=nullptr);

    initial_kf_ed->mpImuPreintegrated = (std::shared_ptr<IMU::Preintegrated>)(nullptr);
    curr_kf->mPrevKF = initial_kf;
    initial_kf->mNextKF = curr_kf;
    curr_kf_ed->mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(curr_kf_ed->mpImuPreintegrated->GetUpdatedBias(), *mpImuCalib);
}

bool InertialOdometry::PredictStateOdom(Frame &curr_frame, Frame &last_frame, std::shared_ptr<KeyFrame> last_kf, bool map_updated) {
  //Is it even possible to get here with no previous frame? Maybe through LocalMappingDisabled shenanigans?
  if (!curr_frame.mpPrevFrame || curr_frame.mpPrevFrame->isPartiallyConstructed) {
    Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  std::shared_ptr<InertialFrameData> ed = curr_frame.External<InertialFrameData>();
  std::shared_ptr<InertialFrameData> last_ed = last_frame.External<InertialFrameData>();
  assert(ed!=nullptr && last_ed!=nullptr);

  const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

  //If the map was merged or loop was closed on the last Frame use Tracking::mpLastKeyFrame, otherwise use Tracking::mLastFrame
  if (map_updated && last_kf) {
    const Eigen::Vector3f twb1 = getImuPosition(last_kf);
    const Eigen::Matrix3f Rwb1 = getImuRotation(last_kf);
    const Eigen::Vector3f Vwb1 = last_kf->GetVelocity();

    const float t12 = mpImuPreintegratedFromLastKF->dT;
    IMU::Bias b = last_kf->External<InertialKeyFrameData>()->GetImuBias();

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(b));
    Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(b);
    Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(b);

    Sophus::SE3f Twb(Rwb2, twb2);
    Sophus::SE3f Tbw = Twb.inverse();
    Sophus::SE3f Tcw = mpImuCalib->mTcb * Tbw;
    curr_frame.SetPose(Tcw);
    curr_frame.SetVelocity(Vwb2);

    ed->mImuBias = b;
    return true;

  } else if (!map_updated && ed->mpImuPreintegratedFrame) {
    const Eigen::Vector3f twb1 = last_frame.GetRwc() * mpImuCalib->mTcb.translation() + last_frame.GetOw();
    const Eigen::Matrix3f Rwb1 = last_frame.GetRwc() * mpImuCalib->mTcb.rotationMatrix();
    const Eigen::Vector3f Vwb1 = last_frame.GetVelocity();

    const float t12 = ed->mpImuPreintegratedFrame->dT;
    IMU::Bias b = last_ed->mImuBias;

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * ed->mpImuPreintegratedFrame->GetDeltaRotation(b));
    Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * ed->mpImuPreintegratedFrame->GetDeltaPosition(b);
    Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * ed->mpImuPreintegratedFrame->GetDeltaVelocity(b);

    Sophus::SE3f Twb(Rwb2, twb2);
    Sophus::SE3f Tbw = Twb.inverse();
    Sophus::SE3f Tcw = mpImuCalib->mTcb * Tbw;
    curr_frame.SetPose(Tcw);
    curr_frame.SetVelocity(Vwb2);

    ed->mImuBias = b;
    return true;
  }

  std::cout << "not IMU prediction!!" << std::endl;
  return false;
}

void InertialOdometry::TrackLocalMapPoseOptimization(Frame &curr_frame, bool &b_map_updated, bool reloc_recently) {
    std::shared_ptr<InertialFrameData> ed = curr_frame.External<InertialFrameData>();
    std::shared_ptr<InertialFrameData> last_ed = curr_frame.mpPrevFrame->External<InertialFrameData>();
    assert(ed!=nullptr && last_ed!=nullptr);

    if (!mpAtlas->isOdomInitialized() || ed->mpImuPreintegratedFrame == nullptr || reloc_recently) {
        Optimizer::PoseOptimization(&curr_frame);
    } else if(b_map_updated || last_ed->mpcpi == nullptr) {
        InertialOptimizer::PoseInertialOptimizationLastKeyFrame(&curr_frame);
    } else {
        InertialOptimizer::PoseInertialOptimizationLastFrame(&curr_frame);
    }
}

void InertialOdometry::NewKeyFrameEvent(std::shared_ptr<KeyFrame> ref_kf) {
    assert(ref_kf->External<InertialKeyFrameData>()!=nullptr);
    mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(ref_kf->External<InertialKeyFrameData>()->GetImuBias(), *mpImuCalib);
}

void InertialOdometry::NewMapEvent() {
    if(mpImuPreintegratedFromLastKF) {
        mpImuPreintegratedFromLastKF = std::make_shared<IMU::Preintegrated>(IMU::Bias(), *mpImuCalib);
    }
}

void InertialOdometry::LocalBundleAdjustment(std::shared_ptr<KeyFrame> curr_kf, bool &b_abortBA) {
    float dist = (curr_kf->mPrevKF->GetCameraCenter() - curr_kf->GetCameraCenter()).norm() +
        (curr_kf->mPrevKF->mPrevKF->GetCameraCenter() - curr_kf->mPrevKF->GetCameraCenter()).norm();

    if (mbStationaryInitEnabled || dist > 0.05)
        LocalMappingIncrementTimeInit(curr_kf->mTimeStamp - curr_kf->mPrevKF->mTimeStamp);

    int tracking_matches_inliers;
    if (std::shared_ptr<Tracking> pTracker = mwpTracker.lock())
        tracking_matches_inliers = pTracker->GetMatchesInliers();

    bool b_large = ((tracking_matches_inliers > 75) && mbMonocular) || ((tracking_matches_inliers > 100) && !mbMonocular);
    InertialOptimizer::LocalInertialBA(curr_kf, &b_abortBA, curr_kf->GetMap(), b_large, !curr_kf->GetMap()->isMature());  
}

void InertialOdometry::InitializeOdom() {
    if(std::shared_ptr<Tracking> pTracker = mwpTracker.lock()) {
        pTracker->mLockPreTeleportTranslation = true;
        if (mbMonocular)
            initializeIMU(ImuInitializater::ImuInitType::MONOCULAR_INIT_G, ImuInitializater::ImuInitType::MONOCULAR_INIT_A, true);
        else
            initializeIMU(ImuInitializater::ImuInitType::STEREO_INIT_G, ImuInitializater::ImuInitType::STEREO_INIT_A, true);
        pTracker->mTeleported = true;
    }
}

void InertialOdometry::initializeIMU(ImuInitializater::ImuInitType priorG, ImuInitializater::ImuInitType priorA, bool bFIBA) {
    if (LocalMappingResetRequested()) return;

    std::shared_ptr<Tracking> pTracker = mwpTracker.lock();
    if(!pTracker) return;

    float minTime = mbMonocular ? 2.0 : 1.0;
    size_t nMinKF = 10;

    int numMoreFramesNeeded = nMinKF - mpAtlas->KeyFramesInMap();
    if (numMoreFramesNeeded > 0) {
        if(numMoreFramesNeeded == 1)
            std::cout << "Waiting for 1 more KeyFrame before IMU initialization" << std::endl;
        else
            std::cout << "Waiting for " << numMoreFramesNeeded << " more KeyFrames before IMU initialization" << std::endl;
        return;
    }

    std::shared_ptr<KeyFrame> curr_kf = LocalMappingGetCurrentKeyFrame();
    assert(curr_kf->External<InertialKeyFrameData>()!=nullptr);


    // Retrieve all keyframe in temporal order
    std::list<std::shared_ptr<KeyFrame>> lpKF;
    std::shared_ptr<KeyFrame> pKF = curr_kf;
    while (pKF->mPrevKF) {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    std::vector<std::shared_ptr<KeyFrame>> vpKF(lpKF.begin(), lpKF.end());

    if (vpKF.size() < nMinKF) {
        std::cout << "cannot initialize, not enough frames in map vpKF?" << std::endl;
        return; // condition could be here too
    }

    if (!curr_kf->GetMap()->isOdomInitialized())
        std::cout << "start IMU initialization" << std::endl;

    double first_ts = vpKF.front()->mTimeStamp;
    if (curr_kf->mTimeStamp - first_ts < minTime) return;

    LocalMappingSetInitializing(true);

    LocalMappingProcessKeyFramesInQueue(vpKF);

    const int N = vpKF.size();
    IMU::Bias b(0, 0, 0, 0, 0, 0);

    // Compute and KF velocities mRwg estimation
    if (!mpAtlas->UseGravityDirectionFromLastMap() && !curr_kf->GetMap()->isOdomInitialized()) {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();

        for (std::vector<std::shared_ptr<KeyFrame>>::iterator itKF = vpKF.begin(); itKF != vpKF.end(); itKF++) {

            auto itKF_eKFd = (*itKF)->External<InertialKeyFrameData>();
            assert(itKF_eKFd!=nullptr);

            if (!itKF_eKFd->mpImuPreintegrated || !(*itKF)->mPrevKF) continue;

            dirG -= getImuRotation((*itKF)->mPrevKF) * itKF_eKFd->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            Eigen::Vector3f _vel = (getImuPosition(*itKF) - getImuPosition((*itKF)->mPrevKF))/itKF_eKFd->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        dirG = dirG / dirG.norm();
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f v = gI.cross(dirG);
        const float nv = v.norm();

        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);

        Eigen::Vector3f vzg = v * ang / nv;
        Rwg = Sophus::SO3f::exp(vzg).matrix();
        mRwg = Rwg.cast<double>();
        LocalMappingSetTimeInit(curr_kf->mTimeStamp - first_ts);
        LocalMappingSetPoseReverseAxisFlip(Sophus::SE3f(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero()));
    } else if(mpAtlas->UseGravityDirectionFromLastMap() && !curr_kf->GetMap()->isOdomInitialized()) {
        for (std::vector<std::shared_ptr<KeyFrame>>::iterator itKF = vpKF.begin(); itKF != vpKF.end(); itKF++) {
            auto itKF_eKFd = (*itKF)->External<InertialKeyFrameData>();
            assert(itKF_eKFd!=nullptr);

            if (!itKF_eKFd->mpImuPreintegrated || !(*itKF)->mPrevKF) continue;

            Eigen::Vector3f _vel = (getImuPosition(*itKF) - getImuPosition((*itKF)->mPrevKF))/itKF_eKFd->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        mRwg = Eigen::Matrix3d::Identity();
        LocalMappingSetTimeInit(curr_kf->mTimeStamp - first_ts);
        auto curr_eKFd = curr_kf->External<InertialKeyFrameData>();
        mbg = curr_eKFd->GetGyroBias().cast<double>();
        mba = curr_eKFd->GetAccBias().cast<double>();
    } else {
        mRwg = Eigen::Matrix3d::Identity();
        auto curr_eKFd = curr_kf->External<InertialKeyFrameData>();
        mbg = curr_eKFd->GetGyroBias().cast<double>();
        mba = curr_eKFd->GetAccBias().cast<double>();
    }

    mScale = 1.0;

    InertialOptimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, false, false, priorG, priorA);

    if (mScale < 1e-1) {
        std::cout << "scale too small" << std::endl;
        LocalMappingSetInitializing(false);
        return;
    }

    // Before this line we are not changing the std::map
    {
        std::scoped_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        if ((fabs(mScale - 1.f) > 0.00001) || !mbMonocular) {
            Sophus::SE3f Tgw(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw, mScale, true);
            pTracker->UpdateScale(mScale);
            pTracker->UpdateLastKeyFrame(curr_kf);
            updateFrameIMU(vpKF[0]->External<InertialKeyFrameData>()->GetImuBias());
        }

        // Check if initialization OK
        if (!mpAtlas->isOdomInitialized()) {
            for (int i = 0; i < N; i++) {
                std::shared_ptr<KeyFrame> pKF2 = vpKF[i];
                pKF2->bOdom = true;
            }
        }
    }

    pTracker->UpdateLastKeyFrame(curr_kf);
    updateFrameIMU(vpKF[0]->External<InertialKeyFrameData>()->GetImuBias());

    if (!mpAtlas->isOdomInitialized()) {
        mpAtlas->SetOdomInitialized();
        curr_kf->bOdom = true;
    }

    if(bFIBA) {
        Verbose::PrintMess("start Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);
        if (priorA != ImuInitializater::ImuInitType::VIBA2_A) {
            InertialOptimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, curr_kf->mnId, nullptr, true, priorG, priorA);
        } else {
            InertialOptimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, curr_kf->mnId, nullptr, false);
            mpAtlas->setUseGravityDirectionFromLastMap(mbFastInit);
            LocalMappingSetPoseReverseAxisFlip(curr_kf->GetPose());
        }  

        Verbose::PrintMess("end Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);
    }

    // Get Map Mutex
    std::scoped_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    unsigned long GBAid = curr_kf->mnId;

    // Process keyframes in the queue
    LocalMappingProcessKeyFramesInQueue(vpKF);
    
    curr_kf = LocalMappingGetCurrentKeyFrame();

    // Correct keyframes starting at map first keyframe
    std::list<std::shared_ptr<KeyFrame>> lpKFtoCheck(
        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.begin(),
        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.end());

    while (!lpKFtoCheck.empty()) {
        std::shared_ptr<KeyFrame> pKF = lpKFtoCheck.front();
        const std::set<std::shared_ptr<KeyFrame>> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        for (std::set<std::shared_ptr<KeyFrame>>::const_iterator sit = sChilds.begin();
            sit != sChilds.end(); sit++) {
            std::shared_ptr<KeyFrame> pChild = *sit;
            if (!pChild || pChild->isBad()) continue;

            if (pChild->mnBAGlobalForKF != GBAid) {
                Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

                Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                if (pChild->isVelocitySet()) {
                    pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                } else {
                    Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);
                }

                pChild->mpExternalKeyFrameData->UpdateChildSpanningTree();

                pChild->mnBAGlobalForKF = GBAid;
            }
            lpKFtoCheck.push_back(pChild);
        }

        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);

        if (pKF->bOdom) {
            pKF->mVwbBefGBA = pKF->GetVelocity();
            pKF->SetVelocity(pKF->mVwbGBA);
            
            pKF->mpExternalKeyFrameData->UpdateParentSpanningTree();
        } else {
            std::cout << "KF " << pKF->mnId << " not set to inertial!! " << std::endl;
        }

        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const std::vector<std::shared_ptr<MapPoint>> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();

    for (size_t i = 0; i < vpMPs.size(); i++) {
        std::shared_ptr<MapPoint> pMP = vpMPs[i];

        if (pMP->isBad()) continue;

        if (pMP->mnBAGlobalForKF == GBAid) {
            // If optimized by Global BA, just update
            pMP->SetWorldPos(pMP->mPosGBA);
        // Update according to the correction of its reference keyframe
        } else if(std::shared_ptr<KeyFrame> pRefKF = (pMP->GetReferenceKeyFrame()).lock()) {
            if (pRefKF->mnBAGlobalForKF != GBAid) continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

            // Backproject using corrected camera
            pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
    }

    Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    LocalMappingSetNewKeyFramesBad();

    pTracker->mState = TrackingState::OK;
    
    LocalMappingSetInitializing(false);

    curr_kf->GetMap()->IncreaseChangeIndex();

    if(!curr_kf->GetMap()->isPartialMature())
        std::cout << "end IMU initialization" << std::endl;
    
}

void InertialOdometry::PostInitializeOdom() {
    if(std::shared_ptr<Tracking> pTracker = mwpTracker.lock()) {
        const float timerVIBA2 = mbFastInit ? 10 : 15;

        std::shared_ptr<KeyFrame> curr_kf = LocalMappingGetCurrentKeyFrame();
        float mTinit = LocalMappingGetTimeInit();
        if ((mTinit < 50.0f)) {
            if (curr_kf->GetMap()->isOdomInitialized() && pTracker->mState == TrackingState::OK) {  // Enter here everytime local-mapping is called
                if (!curr_kf->GetMap()->isPartialMature() && mTinit > 5.0f) {
                    pTracker->mLockPreTeleportTranslation = true;
                    std::cout << "start VIBA 1" << std::endl;
                    curr_kf->GetMap()->SetPartialMature();
                    initializeIMU(ImuInitializater::ImuInitType::VIBA1_G, ImuInitializater::ImuInitType::VIBA1_A, true);
                    pTracker->mTeleported = true;
                    std::cout << "end VIBA 1" << std::endl;
                } else if (!curr_kf->GetMap()->isMature() && mTinit > timerVIBA2) {
                    pTracker->mLockPreTeleportTranslation = true;
                    std::cout << "start VIBA 2" << std::endl;
                    curr_kf->GetMap()->SetMature();
                    initializeIMU(ImuInitializater::ImuInitType::VIBA2_G, ImuInitializater::ImuInitType::VIBA2_A, true);
                    pTracker->mTeleported = true;
                    std::cout << "end VIBA 2" << std::endl;
                }

                // scale refinement
                if (mbMonocular && ((mpAtlas->KeyFramesInMap()) <= 200) &&
                    ((mTinit > 25.0f && mTinit < 25.5f) || (mTinit > 35.0f && mTinit < 35.5f) || (mTinit > 45.0f && mTinit < 45.5f))) {
                    scaleRefinement();
                }
            }
        }
    }
}

bool InertialOdometry::ReadyForKeyFrameCulling(const std::shared_ptr<KeyFrame> &curr_kf) {
    if(mbStationaryInitEnabled && !curr_kf->GetMap()->isMature())
        return false;
    else
        return true;
}

void InertialOdometry::scaleRefinement() {
    if (LocalMappingResetRequested()) return;

    std::shared_ptr<Tracking> pTracker = mwpTracker.lock();
    if(!pTracker) return;

    std::shared_ptr<KeyFrame> curr_kf = LocalMappingGetCurrentKeyFrame();

    // Retrieve all keyframes in temporal order
    std::list<std::shared_ptr<KeyFrame>> lpKF;
    std::shared_ptr<KeyFrame> pKF = curr_kf;
    while (pKF->mPrevKF) {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    std::vector<std::shared_ptr<KeyFrame>> vpKF(lpKF.begin(), lpKF.end());

    LocalMappingProcessKeyFramesInQueue(vpKF);

    curr_kf = LocalMappingGetCurrentKeyFrame();

    mRwg = Eigen::Matrix3d::Identity();
    mScale = 1.0;

    InertialOptimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);

    if (mScale < 1e-1)  // 1e-1
    {
        std::cout << "scale too small" << std::endl;
        LocalMappingSetInitializing(false);
        return;
    }

    Sophus::SO3d so3wg(mRwg);
    // Before this line we are not changing the map
    std::scoped_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    if ((fabs(mScale - 1.f) > 0.002) || !mbMonocular) {
    Sophus::SE3f Tgw(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw, mScale, true);
        pTracker->UpdateScale(mScale);
        pTracker->UpdateLastKeyFrame(curr_kf);
        updateFrameIMU(curr_kf->External<InertialKeyFrameData>()->GetImuBias());
    }

    LocalMappingSetNewKeyFramesBad();
    
    // To perform pose-inertial opt w.r.t. last keyframe
    curr_kf->GetMap()->IncreaseChangeIndex();

    return;
}

void InertialOdometry::InitializeMergeMap(const std::shared_ptr<Map> &curr_map) {
    // Map is not completly initialized
    Eigen::Vector3d bg, ba;
    bg << 0., 0., 0.;
    ba << 0., 0., 0.;
    InertialOptimizer::InertialOptimization(curr_map, bg, ba);
    IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
    std::unique_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    if(std::shared_ptr<Tracking> pTracker = mwpTracker.lock())
        updateFrameIMU(b);

    // Set map initialized
    curr_map->SetMature();
    curr_map->SetPartialMature();
    curr_map->SetOdomInitialized();
}

void InertialOdometry::MergeLocalBundleAdjustment(std::shared_ptr<KeyFrame> curr_kf, std::shared_ptr<KeyFrame> merge_kf, std::shared_ptr<Map> curr_map, KeyFrameAndPose& corr_poses) {
  bool bStopFlag = false;
  InertialOptimizer::MergeInertialBA(curr_kf, merge_kf, &bStopFlag, curr_map, corr_poses);
}

void InertialOdometry::MergeLocalUpdateTrackingFrame(std::shared_ptr<KeyFrame> pCurrentKF) {
    assert(pCurrentKF->External<InertialKeyFrameData>()!=nullptr);
    updateFrameIMU(pCurrentKF->External<InertialKeyFrameData>()->GetImuBias());
}

void InertialOdometry::LoopClosingOptimizeEssentialGraph(std::shared_ptr<Map> pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF, const KeyFrameAndPose& NonCorrectedSim3, const KeyFrameAndPose& CorrectedSim3, const std::map<std::shared_ptr<KeyFrame>, std::set<std::shared_ptr<KeyFrame>>>& LoopConnections) {
    InertialOptimizer::OptimizeEssentialGraph4DoF(pMap, pLoopKF, pCurKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
}

void InertialOdometry::updateFrameIMU(const IMU::Bias& b) {
  if (std::shared_ptr<Tracking> pTracker = mwpTracker.lock()) {
    auto curr_ed = pTracker->mCurrentFrame.External<InertialFrameData>();
    auto last_ed = pTracker->mLastFrame.External<InertialFrameData>();
    assert(curr_ed!=nullptr && last_ed!=nullptr);

    last_ed->SetNewBias(b);
    curr_ed->SetNewBias(b);

    while (!curr_ed->imuIsPreintegrated()) {
        usleep(500);
    }

    if (pTracker->mLastFrame.mnId == pTracker->mLastFrame.mpLastKeyFrame->mnFrameId) {
        Sophus::SE3f Twb(getImuRotation(pTracker->mLastFrame.mpLastKeyFrame), getImuPosition(pTracker->mLastFrame.mpLastKeyFrame));
        Sophus::SE3f Tbw = Twb.inverse();
        Sophus::SE3f Tcw = mpImuCalib->mTcb * Tbw;
        pTracker->mCurrentFrame.SetPose(Tcw);
        pTracker->mCurrentFrame.SetVelocity(pTracker->mLastFrame.mpLastKeyFrame->GetVelocity());

    } else {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = getImuPosition(pTracker->mLastFrame.mpLastKeyFrame);
        const Eigen::Matrix3f Rwb1 = getImuRotation(pTracker->mLastFrame.mpLastKeyFrame);
        const Eigen::Vector3f Vwb1 = pTracker->mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = last_ed->mpImuPreintegrated->dT;

        Sophus::SE3f Twb(IMU::NormalizeRotation(Rwb1 * last_ed->mpImuPreintegrated->GetUpdatedDeltaRotation()), twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * last_ed->mpImuPreintegrated->GetUpdatedDeltaPosition());
        Sophus::SE3f Tbw = Twb.inverse();
        Sophus::SE3f Tcw = mpImuCalib->mTcb * Tbw;
        pTracker->mCurrentFrame.SetPose(Tcw);
        pTracker->mCurrentFrame.SetVelocity(Vwb1 + Gz * t12 + Rwb1 * last_ed->mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    std::shared_ptr<IMU::Preintegrated> currFramePreintegrated = curr_ed->mpImuPreintegrated;
    if (currFramePreintegrated) {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = getImuPosition(pTracker->mCurrentFrame.mpLastKeyFrame);
        const Eigen::Matrix3f Rwb1 = getImuRotation(pTracker->mCurrentFrame.mpLastKeyFrame);
        const Eigen::Vector3f Vwb1 = pTracker->mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = currFramePreintegrated->dT;

        Sophus::SE3f Twb(IMU::NormalizeRotation(Rwb1 * currFramePreintegrated->GetUpdatedDeltaRotation()), twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * currFramePreintegrated->GetUpdatedDeltaPosition());
        Sophus::SE3f Tbw = Twb.inverse();
        Sophus::SE3f Tcw = mpImuCalib->mTcb * Tbw;
        pTracker->mCurrentFrame.SetPose(Tcw);
        pTracker->mCurrentFrame.SetVelocity(Vwb1 + Gz * t12 + Rwb1 * currFramePreintegrated->GetUpdatedDeltaVelocity());
        
    }
  }
}

void InertialOdometry::GlobalBundleAdjustment(std::shared_ptr<Map> pMap, const long unsigned int nLoopId, bool &mbStopGBA) {
  if (!pMap->isOdomInitialized())
    Optimizer::GlobalBundleAdjustemnt(pMap, 10, &mbStopGBA, nLoopId, false);
  else
    InertialOptimizer::FullInertialBA(pMap, 7, false, nLoopId, &mbStopGBA);
}

} //namespace MORB_SLAM