/**
 * This file is not part of ORB-SLAM3
 *
 * Copyright (C) 2024 Sean B.G. O'Rourke
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
#include "MORB_SLAM/G2oTypes.h"

namespace MORB_SLAM{
namespace IMUProcessor{

/**
 * Processes all IMU data for a frame into a format which can be directly used by SLAM's TrackStereo/TrackRGBD/TrackMonocular functions.
 * This function uses InterpolateIMU and CombineIMU to process the data.
 *
 * @param accel_measurements a vector of accelerometer measurements. Each datapoint must have an associated timestamp.
 *                           This vector gets mutated, any measurement before prev_frame_timestamp gets erased.

 * @param accel_timestamps a vector of timestamps associated with each accelerometer measurement. These must be sorted in ascending order.
 *                         This vector gets mutated, any timestamp before prev_frame_timestamp gets erased.

 * @param gyro_measurements a vector of gyro measurements. See accel_measurements.
 * @param gyro_timestamps a vector of timestamps associated with each gyro datapoint. See accel_timestamps.
 * @param prev_frame_timestamp the timestamp of the previous frame. Must have the same units as the other timestamps.
 * @param curr_frame_timestamp the timestamp of the current frame. Must have the same units as the other timestamps.
 * @param timeConversion the conversion factor that changes all timestamp units into seconds. (e.x. if the timestamps are in ms, timeConversion = 1/1000). Default value is 1.
 *
 * @return a pair of the current frame's timestamp in seconds, and a vector of all accelerometer and gyro points created by CombineIMU
 */
std::pair<double, std::vector<MORB_SLAM::IMU::Point>> ProcessIMU(std::vector<Eigen::Vector3f>& accel_measurements, std::vector<double>& accel_timestamps, std::vector<Eigen::Vector3f>& gyro_measurements, std::vector<double>& gyro_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp, const double timeConversion=1.0);
// Overload that supports combining the accel and gyro measurements
std::pair<double, std::vector<MORB_SLAM::IMU::Point>> ProcessIMU(std::vector<Vector6f>& imu_measurements, std::vector<double>& imu_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp, const double timeConversion=1.0);

/**
 * Interpolates the IMU data between datapoints.
 *
 * @param all_data_measurements a vector of IMU measurements. Must be either all accelerometer or all gyro readings. Each measurement must have an associated timestamp.
 *                              This vector gets mutated, any measurement before prev_frame_timestamp gets erased.
 *
 * @param all_timestamps a vector of timestamps associated with each IMU measurement. These must be sorted in ascending order.
 *                       This vector gets mutated, any timestamp before prev_frame_timestamp gets erased.
 *
 * @param prev_frame_timestamp the timestamp of the previous frame. Must have the same units as all_timestamps.
 * @param curr_frame_timestamp the timestamp of the current frame. Must have the same units as all_timestamps.
 * @param isAccel a flag that is set to true if the IMU measurements are accelerometer readings, and false if they're gyro readings.
 *
 * @return a vector of the interpolated IMU points. The points are in chronological order. Returns an empty vector if there are no valid IMU Measurements.
 */
std::vector<MORB_SLAM::IMU::Point> InterpolateIMU(std::vector<Eigen::Vector3f>& all_data_measurements, std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp, const bool isAccel);
// Overload that supports combining the accel and gyro measurements
std::vector<MORB_SLAM::IMU::Point> InterpolateIMU(std::vector<Vector6f>& all_data_measurements, std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp);

/**
 * Combines the accelerometer and gyro measurement vectors into one vector.
 *
 * @param accel_points a vector of accelerometer points. The points are assumed to be in chronological order.
 * @param gyro_points a vector of gyro points. The points are assumed to be in chronological order.
 * @param timeConversion the conversion factor that changes the points' timestep units into seconds. (e.x. if the timesteps are in ms, timeConversion = 1/1000). Default value is 1.
 * 
 * @return a vector of all accelerometer and gyro points. The points are in chronological order. Returns an empty vector if either accel_points or gyro_points is empty.
 */
std::vector<MORB_SLAM::IMU::Point> CombineIMU(std::vector<MORB_SLAM::IMU::Point>& accel_points, std::vector<MORB_SLAM::IMU::Point>& gyro_points, const double timeConversion=1.0);

/**
 * Prints the timestamps of the previous frame, each IMU measurement, and the current frame. For debugging.
 *
 * @param all_timestamps
 * @param prev_frame_timestamp
 * @param curr_frame_timestamp
 */
void dumpInfo(std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp);

} // namespace IMUProcessor
} // namespace MORB_SLAM