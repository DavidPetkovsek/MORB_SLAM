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

#include "MORB_SLAM/ExternalIMUProcessor.h"

namespace MORB_SLAM{
namespace IMUProcessor{

std::pair<double, std::vector<MORB_SLAM::IMU::Point>> ProcessIMU(std::vector<Eigen::Vector3f>& accel_measurements, std::vector<double>& accel_timestamps, std::vector<Eigen::Vector3f>& gyro_measurements, std::vector<double>& gyro_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp, const double timeConversion) {
    std::pair<double, std::vector<MORB_SLAM::IMU::Point>> imu_data;

    std::vector<MORB_SLAM::IMU::Point> accel_points = InterpolateIMU(accel_measurements, accel_timestamps, prev_frame_timestamp, curr_frame_timestamp, true);
    std::vector<MORB_SLAM::IMU::Point> gyro_points = InterpolateIMU(gyro_measurements, gyro_timestamps, prev_frame_timestamp, curr_frame_timestamp, false);

    if(timeConversion == 1.0)
        imu_data.first = curr_frame_timestamp;
    else
        imu_data.first = curr_frame_timestamp*timeConversion;

    if(!accel_points.empty() && !gyro_points.empty())
        imu_data.second = CombineIMU(accel_points, gyro_points, timeConversion);

    return imu_data;
}

std::pair<double, std::vector<MORB_SLAM::IMU::Point>> ProcessIMU(std::vector<Vector6f>& imu_measurements, std::vector<double>& imu_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp, const double timeConversion) {
    std::pair<double, std::vector<MORB_SLAM::IMU::Point>> imu_data;

    imu_data.second = InterpolateIMU(imu_measurements, imu_timestamps, prev_frame_timestamp, curr_frame_timestamp);

    if(timeConversion == 1.0) {
        imu_data.first = curr_frame_timestamp;
    } else {
        imu_data.first = curr_frame_timestamp*timeConversion;
        for(int i = 0; i < imu_data.second.size(); i++)
            imu_data.second[i].t *= timeConversion;
    }
    
    return imu_data;
}

std::vector<MORB_SLAM::IMU::Point> InterpolateIMU(std::vector<Eigen::Vector3f>& all_data_measurements, std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp, const bool isAccel) {
    std::vector<MORB_SLAM::IMU::Point> output;

    size_t n = all_timestamps.size();

    if(n != all_data_measurements.size()) {
        std::string imuType = isAccel ? "Accel" : "Gyro";
        std::cout << "ERROR: " << imuType << " data passed into InterpolateIMU has " << n << " timestamps and " << all_data_measurements.size() << " datapoints" << std::endl;
        return output;
    }

    if(n == 0) {
        std::string imuType = isAccel ? "Accel" : "Gyro";
        std::cout << "No " << imuType << " measurements" << std::endl;
        return output;
    }

    std::vector<Eigen::Vector3f> data_measurements;
    std::vector<double> timestamps;
    bool preMeasurement = false;
    int idx = 0;

    // set idx to the first index with a timestamp after the previous frame
    for(; idx < n && all_timestamps[idx] <= prev_frame_timestamp; idx++) {}

    if(idx == n) {
        std::string imuType = isAccel ? "Accel" : "Gyro";
        std::cout << "All " << imuType << " measurements are before the previous frame" << std::endl;
        // dumpInfo(all_timestamps, prev_frame_timestamp, curr_frame_timestamp);
        all_data_measurements.clear();
        all_timestamps.clear();
        return output;
    }

    // add the IMU measurement before the previous frame timestamp to interpolate IMU measurement
    if(idx > 0) {
        timestamps.push_back(all_timestamps[idx-1]);
        data_measurements.push_back(all_data_measurements[idx-1]);
        preMeasurement = true;
    }

    // add all IMU measurements between the two frames
    for(; idx < n && all_timestamps[idx] < curr_frame_timestamp; idx++) {
        timestamps.push_back(all_timestamps[idx]);
        data_measurements.push_back(all_data_measurements[idx]);
        // num_valid_measurements++;
    }

    // Case where all imu measurements occur after the current frame's timestep
    if(timestamps.empty()) {
        std::string imuType = isAccel ? "Accel" : "Gyro";
        std::cout << "All " << imuType << " measurements are after the current frame" << std::endl;
        // dumpInfo(all_timestamps, prev_frame_timestamp, curr_frame_timestamp);
        return output;
    } else if(timestamps.size() == 1 && preMeasurement) {
        std::string imuType = isAccel ? "Accel" : "Gyro";
        std::cout << "No " << imuType << " measurements are between the previous and current frames" << std::endl;
        // dumpInfo(all_timestamps, prev_frame_timestamp, curr_frame_timestamp);
        all_timestamps.erase(all_timestamps.begin(), std::next(all_timestamps.begin(), idx));
        all_data_measurements.erase(all_data_measurements.begin(), std::next(all_data_measurements.begin(), idx));
        return output;
    }

    // add the IMU measurement after the current frame timestamp to interpolate IMU measurement
    if(idx != n) {
        timestamps.push_back(all_timestamps[idx]);
        data_measurements.push_back(all_data_measurements[idx]);
    }

    // remove all measurements from the global list that are before the IMU measurement right before the current frame
    if(idx > 1) {
        all_timestamps.erase(all_timestamps.begin(), std::next(all_timestamps.begin(), idx-1));
        all_data_measurements.erase(all_data_measurements.begin(), std::next(all_data_measurements.begin(), idx-1));
    }

    Eigen::Vector3f data = data_measurements[0];
    double tstep = curr_frame_timestamp-prev_frame_timestamp;
    n = timestamps.size();

    if(n == 1) {
        output.push_back(MORB_SLAM::IMU::Point(data, tstep, isAccel));
        return output;
    } else if(n == 2) {
        // point-slope form of the line between the two IMU points, evaluated at t = avg(t_prevFrame, t_currFrame)
        // data = data_measurements[0] + (data_measurements[1] - data_measurements[0])*(prev_frame_timestamp + tstep/2 - timestamps[0])/(timestamps[1]-timestamps[0]);
        data = (data_measurements[0] + data_measurements[1]) * 0.5f;
        output.push_back(MORB_SLAM::IMU::Point(data, tstep, isAccel));
        return output;
    }

    for (int i = 0; i < n-1; i++) {
        tstep = timestamps[i+1] - timestamps[i];

        if(tstep == 0) {
            std::cout << "Warning: Two IMU points have the same timestamp." << std::endl;
            if (i == 0) { // first iteration
                data = (data_measurements[0] + data_measurements[1]) * 0.5f;
                tstep = timestamps[1] - prev_frame_timestamp;
            } else if (i < (n - 2)) {
                continue;
            } else { // last iteration
                data = (data_measurements[i] + data_measurements[i+1]) * 0.5f;
                tstep = curr_frame_timestamp - timestamps[i];
            }
        } else {
            if (i == 0) { // first iteration
                double timeFromLastFrameToFirstIMUTimestamp = timestamps[0] - prev_frame_timestamp;
                data = (data_measurements[1] + data_measurements[0] - (data_measurements[1] - data_measurements[0]) * (timeFromLastFrameToFirstIMUTimestamp / tstep)) * 0.5f;
                tstep = timestamps[1] - prev_frame_timestamp;
            } else if (i < (n - 2)) {
                data = (data_measurements[i] + data_measurements[i+1]) * 0.5f;
            } else { // last iteration
                double timeFromeLastIMUTimestampToCurrentFrame = curr_frame_timestamp - timestamps[i+1];
                data = (data_measurements[i+1] + data_measurements[i] + (data_measurements[i+1] - data_measurements[i]) * (timeFromeLastIMUTimestampToCurrentFrame / tstep)) * 0.5f;
                tstep = curr_frame_timestamp - timestamps[i];
            }
        }

        if(tstep == 0) {
            std::cout << "An IMU point has the same timestamp as an image. Skipping iteration." << std::endl;
            continue;
        }

        output.push_back(MORB_SLAM::IMU::Point(data, tstep, isAccel));
    }

    return output;
}

std::vector<MORB_SLAM::IMU::Point> InterpolateIMU(std::vector<Vector6f>& all_data_measurements, std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp) {
    std::vector<MORB_SLAM::IMU::Point> output;

    size_t n = all_timestamps.size();

    if(n != all_data_measurements.size()) {
        std::cout << "ERROR: IMU data passed into InterpolateIMU has " << n << " timestamps and " << all_data_measurements.size() << " datapoints" << std::endl;
        return output;
    }

    if(n == 0) {
        std::cout << "No IMU measurements" << std::endl;
        return output;
    }

    std::vector<Vector6f> data_measurements;
    std::vector<double> timestamps;
    bool preMeasurement = false;
    int idx = 0;

    // set idx to the first index with a timestamp after the previous frame
    for(; idx < n && all_timestamps[idx] <= prev_frame_timestamp; idx++) {}

    if(idx == n) {
        std::cout << "All IMU measurements are before the previous frame" << std::endl;
        all_data_measurements.clear();
        all_timestamps.clear();
        return output;
    }

    // add the IMU measurement before the previous frame timestamp to interpolate IMU measurement
    if(idx > 0) {
        timestamps.push_back(all_timestamps[idx-1]);
        data_measurements.push_back(all_data_measurements[idx-1]);
        preMeasurement = true;
    }

    // add all IMU measurements between the two frames
    for(; idx < n && all_timestamps[idx] < curr_frame_timestamp; idx++) {
        timestamps.push_back(all_timestamps[idx]);
        data_measurements.push_back(all_data_measurements[idx]);
    }

    // Case where all imu measurements occur after the current frame's timestep
    if(timestamps.empty()) {
        std::cout << "All IMU measurements are after the current frame" << std::endl;
        return output;
    } else if(timestamps.size() == 1 && preMeasurement) {
        std::cout << "No IMU measurements are between the previous and current frames" << std::endl;
        all_timestamps.erase(all_timestamps.begin(), std::next(all_timestamps.begin(), idx));
        all_data_measurements.erase(all_data_measurements.begin(), std::next(all_data_measurements.begin(), idx));
        return output;
    }

    // add the IMU measurement after the current frame timestamp to interpolate IMU measurement
    if(idx != n) {
        timestamps.push_back(all_timestamps[idx]);
        data_measurements.push_back(all_data_measurements[idx]);
    }

    // remove all measurements from the global list that are before the IMU measurement right before the current frame
    if(idx > 1) {
        all_timestamps.erase(all_timestamps.begin(), std::next(all_timestamps.begin(), idx-1));
        all_data_measurements.erase(all_data_measurements.begin(), std::next(all_data_measurements.begin(), idx-1));
    }

    Vector6f data = data_measurements[0];
    double tstep = curr_frame_timestamp-prev_frame_timestamp;
    n = timestamps.size();

    if(n == 1) {
        output.push_back(MORB_SLAM::IMU::Point(data.head(3), data.tail(3), tstep));
        return output;
    } else if(n == 2) {
        data = (data_measurements[0] + data_measurements[1]) * 0.5f;
        output.push_back(MORB_SLAM::IMU::Point(data.head(3), data.tail(3), tstep));
        return output;
    }

    for (int i = 0; i < n-1; i++) {
        tstep = timestamps[i+1] - timestamps[i];

        if(tstep == 0) {
            std::cout << "Warning: Two IMU points have the same timestamp." << std::endl;
            if (i == 0) { // first iteration
                data = (data_measurements[0] + data_measurements[1]) * 0.5f;
                tstep = timestamps[1] - prev_frame_timestamp;
            } else if (i < (n - 2)) {
                continue;
            } else { // last iteration
                data = (data_measurements[i] + data_measurements[i+1]) * 0.5f;
                tstep = curr_frame_timestamp - timestamps[i];
            }
        } else {
            if (i == 0) { // first iteration
                double timeFromLastFrameToFirstIMUTimestamp = timestamps[0] - prev_frame_timestamp;
                data = (data_measurements[1] + data_measurements[0] - (data_measurements[1] - data_measurements[0]) * (timeFromLastFrameToFirstIMUTimestamp / tstep)) * 0.5f;
                tstep = timestamps[1] - prev_frame_timestamp;
            } else if (i < (n - 2)) {
                data = (data_measurements[i] + data_measurements[i+1]) * 0.5f;
            } else { // last iteration
                double timeFromeLastIMUTimestampToCurrentFrame = curr_frame_timestamp - timestamps[i+1];
                data = (data_measurements[i+1] + data_measurements[i] + (data_measurements[i+1] - data_measurements[i]) * (timeFromeLastIMUTimestampToCurrentFrame / tstep)) * 0.5f;
                tstep = curr_frame_timestamp - timestamps[i];
            }
        }

        if(tstep == 0) {
            std::cout << "An IMU point has the same timestamp as an image. Skipping iteration." << std::endl;
            continue;
        }

        output.push_back(MORB_SLAM::IMU::Point(data.head(3), data.tail(3), tstep));
    }

    return output;
}

std::vector<MORB_SLAM::IMU::Point> CombineIMU(std::vector<MORB_SLAM::IMU::Point>& accel_points, std::vector<MORB_SLAM::IMU::Point>& gyro_points, const double timeConversion) {
    std::vector<MORB_SLAM::IMU::Point> imu_points;

    auto accel_itr = accel_points.begin();
    auto gyro_itr = gyro_points.begin();
    double net_accel_timestamp = 0;
    double net_gyro_timestamp = 0;

    bool accel_next;
    const bool convertTimestamps = timeConversion != 1.0;

    while(accel_itr != accel_points.end() || gyro_itr != gyro_points.end()) {

        if(gyro_itr == gyro_points.end()) {
            accel_next = true;
        } else if(accel_itr == accel_points.end()) {
            accel_next = false;
        } else if(net_accel_timestamp == net_gyro_timestamp) {
            accel_next = accel_itr->t <= gyro_itr->t;
        } else {
            accel_next = net_accel_timestamp < net_gyro_timestamp;
        }

        if(accel_next) {
            if(convertTimestamps) accel_itr->t *= timeConversion;
            net_accel_timestamp += accel_itr->t;
            imu_points.push_back(*accel_itr);
            accel_itr++;
        } else {
            if(convertTimestamps) gyro_itr->t *= timeConversion;
            net_gyro_timestamp += gyro_itr->t;
            imu_points.push_back(*gyro_itr);
            gyro_itr++;
        }
    }

    return imu_points;
}

void dumpInfo(std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp) {
    std::cout << "Previous Timestamp: " << std::format("{}", prev_frame_timestamp) << std::endl;
    for(auto point : all_timestamps) {
        std::cout << std::format("{}", point) << std::endl;
    }
    std::cout << "Current Timestamp: " << std::format("{}", curr_frame_timestamp) << std::endl;
    std::cout << "_______________" << std::endl;
}

} // namespace IMUProcessor
} // namespace MORB_SLAM