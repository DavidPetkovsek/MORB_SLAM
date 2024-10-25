#include "MORB_SLAM/InertialOdometry.hpp"
#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/Verbose.h"
#include "MORB_SLAM/Optimizer.h"

#include <iostream>

namespace MORB_SLAM {


InertialOdometry::InertialOdometry(std::shared_ptr<CameraSettings> settings) {
    newParameterLoader(*settings);
}

void InertialOdometry::newParameterLoader(CameraSettings &settings) {
    Sophus::SE3f Tbc = settings.Tbc();
    float Ng = settings.noiseGyro();
    float Na = settings.noiseAcc();
    float Ngw = settings.gyroWalk();
    float Naw = settings.accWalk();

    const float sf_a = sqrt(settings.accFrequency());
    const float sf_g = sqrt(settings.gyroFrequency());
    mpImuCalib = std::make_shared<IMU::Calib>(Tbc, Ng * sf_g, Na * sf_a, Ngw / sf_g, Naw / sf_a);
}

void InertialOdometry::AddAccel(const Eigen::Vector3f &accel_meas, const double timestamp_s) {
    std::scoped_lock lock(mMutexAccel);
    mvAccelQueue.push_back(accel_meas);
    mvAccelTimestampQueue.push_back(timestamp_s);
    // std::cout << std::fixed << "Added Accel with timestamp " <<  timestamp_s << std::endl;
}

void InertialOdometry::AddAccel(const std::vector<Eigen::Vector3f> &v_accel_meas, const std::vector<double> v_timestamp_s) {
    std::scoped_lock lock(mMutexAccel);
    if(v_accel_meas.size() != v_timestamp_s.size()) {
        std::cerr << "ERROR: Could not add batch of accel measurements, the number of timestamps don't match" << std::endl;
        return;
    }
    mvAccelQueue.insert(mvAccelQueue.end(), v_accel_meas.begin(), v_accel_meas.end());
    mvAccelTimestampQueue.insert(mvAccelTimestampQueue.end(), v_timestamp_s.begin(), v_timestamp_s.end());
}

void InertialOdometry::AddGyro(const Eigen::Vector3f &gyro_meas, const double timestamp_s) {
    std::scoped_lock lock(mMutexGyro);
    mvGyroQueue.push_back(gyro_meas);
    mvGyroTimestampQueue.push_back(timestamp_s);
    // std::cout << std::fixed << "Added gyro with timestamp " <<  timestamp_s << std::endl;
}

void InertialOdometry::AddGyro(const std::vector<Eigen::Vector3f> &v_gyro_meas, const std::vector<double> v_timestamp_s) {
    std::scoped_lock lock(mMutexGyro);
    if(v_gyro_meas.size() != v_timestamp_s.size()) {
        std::cerr << "ERROR: Could not add batch of gyro measurements, the number of timestamps don't match" << std::endl;
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

    return imu_interpolated;
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

bool InertialOdometry::PreintegrateOdom(Frame &curr_frame, Frame &prev_frame) {      
    return true;
}


} //namespace MORB_SLAM