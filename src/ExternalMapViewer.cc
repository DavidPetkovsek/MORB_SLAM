#include <MORB_SLAM/ExternalMapViewer.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <iostream>
#include <string>

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocketServer.h>

#include <condition_variable>
#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/System.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {

ExternalMapViewer::ExternalMapViewer(const System_ptr& pSystem, const std::string& _serverAddress, const int _serverPort):
    mpSystem(pSystem),
    // mpTracker(pSystem->mpTracker),
    serverAddress(_serverAddress),
    serverPort(_serverPort) {
        threadEMV = std::thread(&ExternalMapViewer::run, this);
    }

ExternalMapViewer::~ExternalMapViewer() {
    if(threadEMV.joinable()) threadEMV.join();
}

void ExternalMapViewer::pushValues(float x, float y, float z){

}

void ExternalMapViewer::run() {
    std::cout << "EXTERNALMAPVIEWER IS RUNNING!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

    ix::WebSocketServer server(serverPort, serverAddress);

    Tracking_ptr trackpointpointtracking(mpSystem->mpTracker);

    server.setOnClientMessageCallback([&trackpointpointtracking](std::shared_ptr<ix::ConnectionState> connectionState, ix::WebSocket & webSocket, const ix::WebSocketMessagePtr & msg) {
        if (msg->type == ix::WebSocketMessageType::Open) {
            Frame currentFrame;
            long unsigned int prevFrameID = 0;
            int message = 0; // TODO
            bool isKF;
            int state;
            Sophus::SE3f currentPose;

            // size_t outputSize = sizeof(float)*12 + sizeof(int)*2 + sizeof(bool);
            // std::vector<uint8_t> binaryOutput(outputSize);
            while(true) {
                if(prevFrameID != trackpointpointtracking->mLastFrame.mnId) {
                    currentFrame = Frame(trackpointpointtracking->mLastFrame, true);
                    prevFrameID = currentFrame.mnId;
                    state = trackpointpointtracking->mState.getID();

                    if(currentFrame.mpReferenceKF && currentFrame.mpReferenceKF->mnId == prevFrameID) {
                        isKF = true;
                    } else {
                        isKF = false;
                    }

                    // binaryOutput = dataToBinary(currentFrame.GetPose().rotationMatrix(), currentFrame.GetPose().translation(), state, message, isKF);
                    // std::cout << "SEND" << std::endl;
                    currentPose = currentFrame.GetPose();

                    webSocket.sendBinary(ExternalMapViewer::dataToBinary(currentPose.rotationMatrix(), currentPose.rotationMatrix().inverse()*currentPose.translation(), state, message, isKF));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    });

    auto res = server.listen();
    if (!res.first) {
        std::cerr << res.second << std::endl;
        return;
    }

    server.start();
    server.wait();
    std::cout << "______________________________ITS OVER_____________________________" << std::endl;
}

std::vector<uint8_t> ExternalMapViewer::dataToBinary(const Sophus::Matrix3f& rotationMatrix, const Sophus::Vector3f& translation, const int state, const int message, const bool KF) {
    
    size_t outputSize = sizeof(float)*12 + sizeof(int)*2 + sizeof(bool);
    std::vector<uint8_t> binaryOutput(outputSize);
    
    memcpy(binaryOutput.data(), rotationMatrix.data(), 9*sizeof(float));
    memcpy(binaryOutput.data() + 9*sizeof(float), translation.data(), 3*sizeof(float));
    memcpy(binaryOutput.data() + 12*sizeof(float), &state, sizeof(int));
    memcpy(binaryOutput.data() + 12*sizeof(float) + sizeof(int), &message, sizeof(int));
    memcpy(binaryOutput.data() + 12*sizeof(float) + 2*sizeof(int), &KF, sizeof(bool));

    return binaryOutput;
}

}