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

std::vector<MORB_SLAM::IMU::Point> InterpolateIMU(std::vector<Eigen::Vector3f>& all_data_measurements, std::vector<double>& all_timestamps, double prev_frame_timestamp, double curr_frame_timestamp, bool isAccel) {
    std::vector<MORB_SLAM::IMU::Point> output;

    std::vector<Eigen::Vector3f> data_measurements;
    std::vector<double> timestamps;
    
    size_t n = all_timestamps.size();
    int idx = 0;

    if(n >= 2 && all_timestamps[0] < prev_frame_timestamp && all_timestamps[1] >= prev_frame_timestamp) {
        timestamps.push_back(all_timestamps[0]);
        data_measurements.push_back(all_data_measurements[0]);
        timestamps.push_back(all_timestamps[1]);
        data_measurements.push_back(all_data_measurements[1]);
        idx = 2;
    }

    for(; idx < n && all_timestamps[idx] <= curr_frame_timestamp; idx++) {
        if(all_timestamps[idx] >= prev_frame_timestamp) {
            timestamps.push_back(all_timestamps[idx]);
            data_measurements.push_back(all_data_measurements[idx]);
        }
    }

    if(idx != n) {
        timestamps.push_back(all_timestamps[idx]);
        data_measurements.push_back(all_data_measurements[idx]);
    }

    if(idx > 1) {
        all_timestamps.erase(all_timestamps.begin(), std::next(all_timestamps.begin(), idx-1));
        all_data_measurements.erase(all_data_measurements.begin(), std::next(all_data_measurements.begin(), idx-1));
    }

    Eigen::Vector3f data;
    double tstep = curr_frame_timestamp-prev_frame_timestamp;
    n = timestamps.size();

    if(n == 0) {
        std::cout << "No datapoints" << std::endl;
        return output;
    } else if(n == 1) {
        output.push_back(MORB_SLAM::IMU::Point(data_measurements[0], tstep, isAccel));
        return output;
    } else if(n == 2) {
        data = (data_measurements[0] + data_measurements[1]) * 0.5f;
        output.push_back(MORB_SLAM::IMU::Point(data, tstep, isAccel));
        return output;
    }

    for (int i = 0; i < n-1; i++) {
        tstep = timestamps[i+1] - timestamps[i];

        if(tstep == 0) {
            std::cout << "Two IMU points have the same timestamp. Skipping iteration." << std::endl;
            continue;
        }

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

        if(tstep == 0) {
            std::cout << "An IMU point has the same timestamp as an image. Skipping iteration." << std::endl;
            continue;
        }

        output.push_back(MORB_SLAM::IMU::Point(data, tstep, isAccel));
    }

    return output;
}

std::vector<MORB_SLAM::IMU::Point> CombineIMU(std::vector<MORB_SLAM::IMU::Point> accel_points, std::vector<MORB_SLAM::IMU::Point> gyro_points) {
    std::vector<MORB_SLAM::IMU::Point> imu_points;

    auto accel_itr = accel_points.begin();
    auto gyro_itr = gyro_points.begin();
    double net_accel_timestamp = 0;
    double net_gyro_timestamp = 0;

    bool accel_next;

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
            imu_points.push_back(*accel_itr);
            net_accel_timestamp += accel_itr->t;
            accel_itr++;
        } else {
            imu_points.push_back(*gyro_itr);
            net_gyro_timestamp += gyro_itr->t;
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
    // TODO make img/imu timestamps std::chrono so you can read in either sec or ms
    // For now it's just seconds
    double img_timestamp;
    
    Eigen::Vector3f accel(3);
    Eigen::Vector3f gyro(3);
    double accel_timestamp;
    double gyro_timestamp;
    
    std::vector<Eigen::Vector3f> accel_measurements;
    std::vector<Eigen::Vector3f> gyro_measurements;
    std::vector<double> accel_timestamps;
    std::vector<double> gyro_timestamps;

    int image_size = left_img.total()*left_img.elemSize();
    int timestamp_size = sizeof(double);
    int imu_size = sizeof(float)*3;

    bool connected = false;
    bool new_img = false;

    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    webSocket.setOnMessageCallback([&webSocket, &connected, &img_timestamp, &left_img, &right_img, &accel_timestamp, &accel_timestamps, &accel, &accel_measurements, &gyro_timestamp, &gyro_timestamps, &gyro, &gyro_measurements, timestamp_size, image_size, imu_size, &imu_mutex, &cond_image_rec, &new_img](const ix::WebSocketMessagePtr& msg) {
            if(msg->type == ix::WebSocketMessageType::Message) {
                std::unique_lock<std::mutex> lock(imu_mutex);

                if(msg->str.data()[0] == 1) {
                    std::memcpy(&img_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy((char *)left_img.data, msg->str.data()+1+timestamp_size, image_size);
                    std::memcpy((char *)right_img.data, msg->str.data()+1+image_size+timestamp_size, image_size);
                    new_img = true;

                    lock.unlock();
                    cond_image_rec.notify_all();
                } else if(msg->str.data()[0] == 2) {
                    std::memcpy(&accel_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy(accel.data(), msg->str.data()+1+timestamp_size, imu_size);
                    accel_measurements.push_back(accel);
                    accel_timestamps.push_back(accel_timestamp);
                } else if(msg->str.data()[0] == 3) {
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

    std::string hostAddress = "0.0.0.0";
    int portNumber = 9002;

    auto SLAM = std::make_shared<MORB_SLAM::System>(argv[1],argv[2], MORB_SLAM::CameraType::IMU_STEREO);
    auto viewer = std::make_shared<MORB_SLAM::Viewer>(SLAM);
     
    std::vector<Eigen::Vector3f> local_accel_measurements;
    std::vector<double> local_accel_timestamps;
    std::vector<Eigen::Vector3f> local_gyro_measurements;
    std::vector<double> local_gyro_timestamps;
    cv::Mat local_left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat local_right_img(cv::Size(848, 480), CV_8UC1);
    double local_img_timestamp;

    bool doTheBonk = false;
    bool isFirstLoop = true;

    double prev_img_timestamp = -1;

    webSocket.start();

    while(!exitSLAM && !connected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    while(!exitSLAM) {
        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            while(!new_img)
                cond_image_rec.wait(lk);

            local_accel_measurements.insert(local_accel_measurements.end(), accel_measurements.begin(), accel_measurements.end());
            local_accel_timestamps.insert(local_accel_timestamps.end(), accel_timestamps.begin(), accel_timestamps.end());
            local_gyro_measurements.insert(local_gyro_measurements.end(), gyro_measurements.begin(), gyro_measurements.end());
            local_gyro_timestamps.insert(local_gyro_timestamps.end(), gyro_timestamps.begin(), gyro_timestamps.end());
            local_img_timestamp = img_timestamp;
            local_left_img = left_img.clone();
            local_right_img = right_img.clone();

            accel_measurements.clear();
            accel_timestamps.clear();
            gyro_measurements.clear();
            gyro_timestamps.clear();

            new_img = false;
        }


        std::vector<MORB_SLAM::IMU::Point> accel_points = InterpolateIMU(local_accel_measurements, local_accel_timestamps, prev_img_timestamp, local_img_timestamp, true);
        std::vector<MORB_SLAM::IMU::Point> gyro_points = InterpolateIMU(local_gyro_measurements, local_gyro_timestamps, prev_img_timestamp, local_img_timestamp, false);

        std::vector<MORB_SLAM::IMU::Point> imu_points = CombineIMU(accel_points, gyro_points);

        // std::cout << "PREVIOUS FRAME TIMESTAMP: " << prev_img_timestamp << std::endl;
        // for(auto itr = imu_points.begin(); itr != imu_points.end(); itr++) {
        //     std::string point_type = itr->hasAccel ? "Accel" : "Gyro";
        //     Eigen::Vector3f point_data = itr->hasAccel ? itr->a : itr->w;
            
        //     std::cout << point_type << " w/ dt = " << itr->t << ": " << point_data[0] << ", " << point_data[1] << ", " << point_data[2] << std::endl;
        // }
        // std::cout << "CURRENT FRAME TIMESTAMP: " << local_img_timestamp << std::endl;
        // std::cout << "______________________________" << std::endl << std::endl;

        MORB_SLAM::StereoPacket sophusPose = SLAM->TrackStereo(local_left_img, local_right_img, local_img_timestamp, imu_points);

        // if(isFirstLoop && SLAM->HasInitialFramePose()) {
        //     Sophus::SE3f outputPose = SLAM->GetInitialFramePose();
        //     std::cout << outputPose.rotationMatrix() << std::endl;
        //     std::cout << outputPose.translation() << std::endl;
        //     isFirstLoop = false;
        // }

        viewer->update(sophusPose);

        // if(sophusPose.pose.has_value()) {
        //     Sophus::Vector3f pose_translation = sophusPose.pose->translation();
        //     std::cout << pose_translation[0] << ", " << pose_translation[1] << ", " << pose_translation[2] << std::endl;
        // }

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