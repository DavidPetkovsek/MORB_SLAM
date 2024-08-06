#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocketServer.h>

#include <iostream>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include <MORB_SLAM/System.h>
#include <MORB_SLAM/Viewer.h>
#include "MORB_SLAM/ExternalIMUProcessor.h"

#include <Eigen/StdVector>

#include <csignal>

bool exitSLAM = false;

void sigHandler(int sigNum) {
    std::cout << "\nClosing SLAM w/ CTRL+C" << std::endl;
    exitSLAM = true;
    // exit(sigNum);
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

    std::mutex img_mutex;
    std::mutex accel_mutex;
    std::mutex gyro_mutex;
    std::condition_variable cond_image_rec;

    webSocket.setOnMessageCallback([&webSocket, &connected, &img_timestamp, &left_img, &right_img, &accel_timestamp, &accel_timestamps, &accel, &accel_measurements, &gyro_timestamp, &gyro_timestamps, &gyro, &gyro_measurements, timestamp_size, image_size, imu_size, &img_mutex, &accel_mutex, &gyro_mutex, &cond_image_rec, &new_img](const ix::WebSocketMessagePtr& msg) {
            if(msg->type == ix::WebSocketMessageType::Message) {
                if(msg->str.data()[0] == 1) {
                    std::unique_lock<std::mutex> lock(img_mutex);
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

// Settings
////////////////////////////////////////////////////////////////////////////////////////////////////////
    const std::string hostAddress = "0.0.0.0";
    const int portNumber = 9002;
    const double time_unit_to_seconds_conversion_factor = 0.001;
////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto SLAM = std::make_shared<MORB_SLAM::System>(argv[1],argv[2], MORB_SLAM::CameraType::IMU_STEREO);
    auto viewer = std::make_shared<MORB_SLAM::Viewer>(SLAM);

    std::pair<double, std::vector<MORB_SLAM::IMU::Point>> slam_data;
     
    std::vector<Eigen::Vector3f> local_accel_measurements;
    std::vector<double> local_accel_timestamps;
    std::vector<Eigen::Vector3f> local_gyro_measurements;
    std::vector<double> local_gyro_timestamps;
    cv::Mat local_left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat local_right_img(cv::Size(848, 480), CV_8UC1);
    double local_img_timestamp;
    double prev_img_timestamp;

    webSocket.start();

    while(!exitSLAM && !connected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    {
        std::unique_lock<std::mutex> lk(img_mutex);
        while(!new_img)
            cond_image_rec.wait(lk);

        prev_img_timestamp = img_timestamp;

        new_img = false;
    }

    while(!exitSLAM) {
        {
            std::unique_lock<std::mutex> lk(img_mutex);
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

        slam_data = MORB_SLAM::IMUProcessor::ProcessIMU(local_accel_measurements, local_accel_timestamps, local_gyro_measurements, local_gyro_timestamps, prev_img_timestamp, local_img_timestamp, time_unit_to_seconds_conversion_factor);
        prev_img_timestamp = local_img_timestamp;

        MORB_SLAM::StereoPacket sophusPose = SLAM->TrackStereo(local_left_img, local_right_img, slam_data.first, slam_data.second);

        viewer->update(sophusPose);
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