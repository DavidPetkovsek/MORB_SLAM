#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocketServer.h>

#include <iostream>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include <MORB_SLAM/System.h>
#include <MORB_SLAM/Viewer.h>
#include <MORB_SLAM/ExternalMapViewer.h>

#include <Eigen/StdVector>

#include <csignal>

bool exitSLAM = false;

void sigHandler(int sigNum) {
    std::cout << "\nClosing SLAM w/ CTRL+C" << std::endl;
    exitSLAM = true;
    // exit(sigNum);
}

void dumpInfo(std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp) {
    std::cout << "Previous Timestamp: " << std::format("{}", prev_frame_timestamp) << std::endl;
    for(auto point : all_timestamps) {
        std::cout << std::format("{}", point) << std::endl;
    }
    std::cout << "Current Timestamp: " << std::format("{}", curr_frame_timestamp) << std::endl;
    std::cout << "_______________" << std::endl;
}

std::vector<MORB_SLAM::IMU::Point> InterpolateIMU(std::vector<Eigen::Vector3f>& all_data_measurements, std::vector<double>& all_timestamps, const double prev_frame_timestamp, const double curr_frame_timestamp, const bool isAccel) {
    std::vector<MORB_SLAM::IMU::Point> output;

    size_t n = all_timestamps.size();
    if(n == 0) {
        std::string imuType = isAccel ? "Accel" : "Gyro";
        std::cout << "No " << imuType << " measurements" << std::endl;
        std::cout << "_______________" << std::endl;
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

std::vector<MORB_SLAM::IMU::Point> CombineIMU(std::vector<MORB_SLAM::IMU::Point>& accel_points, std::vector<MORB_SLAM::IMU::Point>& gyro_points, const double timeConversion=1.0) {
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

int main(int argc, char **argv) {
    signal(SIGINT, sigHandler);

    {
    ix::initNetSystem();
    ix::WebSocket webSocket;
    //set to the address of the websocket host sending the IMU data
    std::string url("ws://172.26.0.1:8765");
    webSocket.setUrl(url);
    std::cout << "Connecting to " << url << std::endl;

    cv::Mat left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat right_img(cv::Size(848, 480), CV_8UC1);
    double img_timestamp;
    
    Eigen::Vector3f accel(3);
    Eigen::Vector3f gyro(3);
    double accel_timestamp;
    double gyro_timestamp;
    
    std::vector<Eigen::Vector3f> accel_measurements;
    std::vector<Eigen::Vector3f> gyro_measurements;
    std::vector<double> accel_timestamps;
    std::vector<double> gyro_timestamps;

    const int image_size = left_img.total()*left_img.elemSize();
    const int timestamp_size = sizeof(double);
    const int imu_size = sizeof(float)*3;

    bool connected = false;
    bool new_img = false;

    std::mutex imu_mutex;
    std::mutex accel_mutex;
    std::mutex gyro_mutex;
    std::condition_variable cond_image_rec;

    webSocket.setOnMessageCallback([&webSocket, &connected, &img_timestamp, &left_img, &right_img, &accel_timestamp, &accel_timestamps, &accel, &accel_measurements, &gyro_timestamp, &gyro_timestamps, &gyro, &gyro_measurements, timestamp_size, image_size, imu_size, &imu_mutex, &accel_mutex, &gyro_mutex, &cond_image_rec, &new_img](const ix::WebSocketMessagePtr& msg) {
            if(msg->type == ix::WebSocketMessageType::Message) {
                if(msg->str.data()[0] == 1) {
                    std::unique_lock<std::mutex> lock(imu_mutex);
                    std::memcpy(&img_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy((char *)left_img.data, msg->str.data()+1+timestamp_size, image_size);
                    std::memcpy((char *)right_img.data, msg->str.data()+1+image_size+timestamp_size, image_size);
                    new_img = true;

                    lock.unlock();
                    cond_image_rec.notify_all();
                } else if(msg->str.data()[0] == 2) {
                    std::unique_lock<std::mutex> lock(accel_mutex);
                    std::memcpy(&accel_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy(accel.data(), msg->str.data()+1+timestamp_size, imu_size);
                    accel_measurements.push_back(accel);
                    accel_timestamps.push_back(accel_timestamp);
                } else if(msg->str.data()[0] == 3) {
                    std::unique_lock<std::mutex> lock(gyro_mutex);
                    std::memcpy(&gyro_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy(gyro.data(), msg->str.data()+1+timestamp_size, imu_size);
                    gyro_measurements.push_back(gyro);
                    gyro_timestamps.push_back(gyro_timestamp);
                }
            } else if(msg->type == ix::WebSocketMessageType::Open) {
                std::cout << "Connected to the Realsense websocket" << std::endl;
                connected = true;
                webSocket.send("gimme");
            } else if(msg->type == ix::WebSocketMessageType::Error) {
                std::cout << "Connection error: " << msg->errorInfo.reason << std::endl;
            }
        }
    );
    
////////////////////////////////////////////////////////////////////////////////////////////////////////

    const std::string hostAddress = "0.0.0.0";
    const int portNumber = 9002;

    auto SLAM = std::make_shared<MORB_SLAM::System>(argv[1],argv[2], MORB_SLAM::CameraType::IMU_STEREO);
    auto viewer = std::make_shared<MORB_SLAM::Viewer>(SLAM);
     
    std::vector<Eigen::Vector3f> local_accel_measurements;
    std::vector<double> local_accel_timestamps;
    std::vector<Eigen::Vector3f> local_gyro_measurements;
    std::vector<double> local_gyro_timestamps;
    cv::Mat local_left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat local_right_img(cv::Size(848, 480), CV_8UC1);
    double local_img_timestamp;
    double prev_img_timestamp;

    double time_unit_to_seconds_conversion_factor = 0.001;
    double slam_timestamp;

    webSocket.start();

    while(!exitSLAM && !connected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    {
        std::unique_lock<std::mutex> lk(imu_mutex);
        while(!new_img)
            cond_image_rec.wait(lk);

        prev_img_timestamp = img_timestamp;

        new_img = false;
    }

    while(!exitSLAM) {
        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            while(!new_img)
                cond_image_rec.wait(lk);

            local_img_timestamp = img_timestamp;
            local_left_img = left_img.clone();
            local_right_img = right_img.clone();

            new_img = false;
        }

        {
            std::unique_lock<std::mutex> lk(accel_mutex);
            local_accel_measurements.insert(local_accel_measurements.end(), accel_measurements.begin(), accel_measurements.end());
            local_accel_timestamps.insert(local_accel_timestamps.end(), accel_timestamps.begin(), accel_timestamps.end());

            accel_measurements.clear();
            accel_timestamps.clear();
        }

        {
            std::unique_lock<std::mutex> lk(gyro_mutex);
            local_gyro_measurements.insert(local_gyro_measurements.end(), gyro_measurements.begin(), gyro_measurements.end());
            local_gyro_timestamps.insert(local_gyro_timestamps.end(), gyro_timestamps.begin(), gyro_timestamps.end());

            gyro_measurements.clear();
            gyro_timestamps.clear();
        }

        std::vector<MORB_SLAM::IMU::Point> accel_points = InterpolateIMU(local_accel_measurements, local_accel_timestamps, prev_img_timestamp, local_img_timestamp, true);
        std::vector<MORB_SLAM::IMU::Point> gyro_points = InterpolateIMU(local_gyro_measurements, local_gyro_timestamps, prev_img_timestamp, local_img_timestamp, false);

        std::vector<MORB_SLAM::IMU::Point> imu_points;
        if(!accel_points.empty() && !gyro_points.empty())
            imu_points = CombineIMU(accel_points, gyro_points, time_unit_to_seconds_conversion_factor);

        slam_timestamp = local_img_timestamp*time_unit_to_seconds_conversion_factor;
        MORB_SLAM::StereoPacket sophusPose = SLAM->TrackStereo(local_left_img, local_right_img, slam_timestamp, imu_points);

        viewer->update(sophusPose);

        prev_img_timestamp = local_img_timestamp;
        imu_points.clear();
    }

    std::cout << "Stopping WebSocket" << std::endl;
    webSocket.stop();
    std::cout << "Stopping Viewer" << std::endl;
    viewer.reset();
    std::cout << "Stopping SLAM" << std::endl;
    SLAM.reset();
    std::cout << "Done ( :" << std::endl;
    }
    std::cout << "Done 2 ( :" << std::endl;
    return 0;
}