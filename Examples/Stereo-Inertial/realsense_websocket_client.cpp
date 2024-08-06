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
    double img_timestamp = 0;
    
    MORB_SLAM::Vector6f imu(6);
    double imu_timestamp = 0;
    std::vector<MORB_SLAM::Vector6f> imu_measurements;
    std::vector<double> imu_timestamps;

    const int image_size = left_img.total()*left_img.elemSize();
    const int timestamp_size = sizeof(double);
    const int imu_size = sizeof(float)*6;

    bool connected = false;
    bool new_img = false;

    std::mutex img_mutex;
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    webSocket.setOnMessageCallback([&webSocket, &connected, &img_timestamp, &left_img, &right_img, &imu_timestamp, &imu_timestamps, &imu, &imu_measurements, timestamp_size, image_size, imu_size, &img_mutex, &imu_mutex, &cond_image_rec, &new_img](const ix::WebSocketMessagePtr& msg) {
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
                    std::unique_lock<std::mutex> lock(imu_mutex);
                    std::memcpy(&imu_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy(imu.data(), msg->str.data()+1+timestamp_size, imu_size);
                    imu_measurements.push_back(imu);
                    imu_timestamps.push_back(imu_timestamp);
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
        
    std::vector<MORB_SLAM::Vector6f> local_imu_measurements;
    std::vector<double> local_imu_timestamps;
    cv::Mat local_left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat local_right_img(cv::Size(848, 480), CV_8UC1);
    double local_img_timestamp;
    double prev_img_timestamp;

    bool isFirstLoop = true;

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
            std::unique_lock<std::mutex> lk(imu_mutex);
            local_imu_measurements.insert(local_imu_measurements.end(), imu_measurements.begin(), imu_measurements.end());
            local_imu_timestamps.insert(local_imu_timestamps.end(), imu_timestamps.begin(), imu_timestamps.end());

            imu_measurements.clear();
            imu_timestamps.clear();
        }

        slam_data = MORB_SLAM::IMUProcessor::ProcessIMU(local_imu_measurements, local_imu_timestamps, prev_img_timestamp, local_img_timestamp, time_unit_to_seconds_conversion_factor);
        prev_img_timestamp = local_img_timestamp;

        MORB_SLAM::StereoPacket sophusPose = SLAM->TrackStereo(local_left_img, local_right_img, slam_data.first, slam_data.second);

        if(isFirstLoop && SLAM->HasInitialFramePose()) {
            Sophus::SE3f outputPose = SLAM->GetInitialFramePose();
            std::cout << outputPose.rotationMatrix() << std::endl;
            std::cout << outputPose.translation() << std::endl;
            isFirstLoop = false;
        }

        viewer->update(sophusPose);
        // if(sophusPose.pose.has_value())
        //     Sophus::Vector3f pose_translation = sophusPose.pose->translation();
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