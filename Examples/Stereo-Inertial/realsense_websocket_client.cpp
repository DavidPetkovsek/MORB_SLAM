#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <iostream>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include <MORB_SLAM/System.h>
#include <MORB_SLAM/Viewer.h>

int main(int argc, char **argv) {
    ix::initNetSystem();
    ix::WebSocket webSocket;
    //set to the address of the websocket host sending the IMU data
    std::string url("ws://172.17.112.1:8765");
    webSocket.setUrl(url);
    std::cout << "Connecting to " << url << std::endl;

    cv::Mat left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat right_img(cv::Size(848, 480), CV_8UC1);
    // TODO make img/imu timestamps std::chrono so you can read in either sec or ms
    // For now it's just seconds
    double img_timestamp = 0;
    
    std::vector<float> imu(6);
    double imu_timestamp = 0;
    
    std::vector<std::vector<float>> imu_measurements;
    std::vector<double> imu_timestamps;
    std::vector<MORB_SLAM::IMU::Point> imu_points;

    int image_size = left_img.total()*left_img.elemSize();
    int timestamp_size = sizeof(double);
    int imu_size = sizeof(float)*6;

    bool connected = false;
    bool new_img = false;

    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;


    webSocket.setOnMessageCallback([&webSocket, &connected, &img_timestamp, &left_img, &right_img, &imu_timestamp, &imu_timestamps, &imu, &imu_measurements, timestamp_size, image_size, imu_size, &imu_mutex, &cond_image_rec, &new_img](const ix::WebSocketMessagePtr& msg) {
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
                    std::memcpy(&imu_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy(imu.data(), msg->str.data()+1+timestamp_size, imu_size);
                    imu_measurements.push_back(imu);
                    imu_timestamps.push_back(imu_timestamp);
                }
            }
            else if(msg->type == ix::WebSocketMessageType::Open) {
                std::cout << "Connection established" << std::endl;
                connected = true;
                webSocket.send("gimme");
            }
            else if(msg->type == ix::WebSocketMessageType::Error) {
                std::cout << "Connection error: " << msg->errorInfo.reason << std::endl;
            }
        }
    );
    
////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto SLAM = std::make_shared<MORB_SLAM::System>(argv[1],argv[2], MORB_SLAM::CameraType::IMU_STEREO);
    auto viewer = std::make_shared<MORB_SLAM::Viewer>(SLAM);

    float imageScale = SLAM->GetImageScale();
    
    std::vector<std::vector<float>> local_imu_measurements;
    std::vector<double> local_imu_timestamps;
    cv::Mat local_left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat local_right_img(cv::Size(848, 480), CV_8UC1);
    double local_img_timestamp;

    webSocket.start();

    while(!connected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    while(true) {
        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            while(!new_img)
                cond_image_rec.wait(lk);

            local_imu_measurements = imu_measurements;
            local_imu_timestamps = imu_timestamps;
            local_img_timestamp = img_timestamp;
            local_left_img = left_img.clone();
            local_right_img = right_img.clone();

            imu_measurements.clear();
            imu_timestamps.clear();

            new_img = false;
        }

        for(size_t i=0; i<local_imu_measurements.size(); i++) {
            MORB_SLAM::IMU::Point new_point(local_imu_measurements[i][0], local_imu_measurements[i][1], local_imu_measurements[i][2], local_imu_measurements[i][3], local_imu_measurements[i][4], local_imu_measurements[i][5], local_imu_timestamps[i]);
            imu_points.push_back(new_point);
        }

        if(imageScale != 1.f) {
            int width = local_left_img.cols * imageScale;
            int height = local_left_img.rows * imageScale;
            cv::resize(local_left_img, local_left_img, cv::Size(width, height));
            cv::resize(local_right_img, local_right_img, cv::Size(width, height));
        }


        MORB_SLAM::StereoPacket sophusPose = SLAM->TrackStereo(local_left_img, local_right_img, local_img_timestamp, imu_points);

        if(sophusPose.pose) {
            sophusPose.pose = sophusPose.pose->inverse();
        }
        
        viewer->update(sophusPose);
        //std::cout << sophusPose.pose->translation()[0] << ", " << sophusPose.pose->translation()[1] << ", " << sophusPose.pose->translation()[2] << std::endl;
        imu_points.clear();
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // while(true) {
    //     cv::imshow("LEFT", left_img);
    //     cv::imshow("RIGHT", right_img);
    //     std::cout << accel[0] << ", " << accel[1] << ", " << accel[2] << std::endl;
    //     char key = cv::waitKey(15);
    //     if(key == 'q') {
    //         webSocket.close();
    //         break;
    //     }
    // }

    return 0;
}