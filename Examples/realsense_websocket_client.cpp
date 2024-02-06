#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    ix::initNetSystem();
    ix::WebSocket webSocket;

    std::string url("ws://172.17.112.1:8765");
    webSocket.setUrl(url);

    std::cout << "Connecting to " << url << std::endl;

    cv::Mat left_img(cv::Size(848, 480), CV_8UC1);
    cv::Mat right_img(cv::Size(848, 480), CV_8UC1);
    uint64_t img_timestamp = 0;
    int image_size = left_img.total()*left_img.elemSize();
    int timestamp_size = sizeof(uint64_t);

    double accel[3];
    double gyro[3];
    uint64_t imu_timestamp = 0;
    int imu_size = sizeof(double)*3;

    webSocket.setOnMessageCallback([&webSocket, &img_timestamp, &left_img, &right_img, &imu_timestamp, &accel, &gyro, timestamp_size, image_size, imu_size](const ix::WebSocketMessagePtr& msg) {
            if(msg->type == ix::WebSocketMessageType::Message) {
                if(msg->str.data()[0] == 1) {
                    std::memcpy(&img_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy((char *)left_img.data, msg->str.data()+1+timestamp_size, image_size);
                    std::memcpy((char *)right_img.data, msg->str.data()+1+image_size+timestamp_size, image_size);
                } else if(msg->str.data()[0] == 2) {
                    std::memcpy(&imu_timestamp, msg->str.data()+1, timestamp_size);
                    std::memcpy((char *)accel, msg->str.data()+1+timestamp_size, imu_size);
                    std::memcpy((char *)gyro, msg->str.data()+1+imu_size+timestamp_size, imu_size);
                }
            }
            else if(msg->type == ix::WebSocketMessageType::Open) {
                std::cout << "Connection established" << std::endl;
                std::cout << "> " << std::flush;
                webSocket.send("gimme");
            }
            else if(msg->type == ix::WebSocketMessageType::Error) {
                std::cout << "Connection error: " << msg->errorInfo.reason << std::endl;
                std::cout << "> " << std::flush;
            }
        }
    );
    
    webSocket.start();

    while(true) {
        cv::imshow("LEFT", left_img);
        cv::imshow("RIGHT", right_img);
        std::cout << accel[0] << ", " << accel[1] << ", " << accel[2] << std::endl;
        char key = cv::waitKey(15);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}