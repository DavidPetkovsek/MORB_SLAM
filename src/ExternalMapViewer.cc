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
    mpTracker(pSystem->mpTracker),
    serverAddress(_serverAddress),
    serverPort(_serverPort),
    valuesPushed(false) {
        std::cout << "Creating ExternalMapViewer thread" << std::endl;
        threadEMV = std::jthread(&ExternalMapViewer::run, this);
    }

ExternalMapViewer::~ExternalMapViewer() {
    if(threadEMV.joinable()) threadEMV.join();
}

void ExternalMapViewer::pushValues(float x, float y, float z) {
    pushedValues = {x,y,z};
    valuesPushed = true;
}

void ExternalMapViewer::run() {
    ix::WebSocketServer server(serverPort, serverAddress);

    std::cout << "ExternalMapViewer Started" << std::endl;

    server.setOnClientMessageCallback([this](std::shared_ptr<ix::ConnectionState> connectionState, ix::WebSocket & webSocket, const ix::WebSocketMessagePtr & msg) {
        if (msg->type == ix::WebSocketMessageType::Open) {
            Frame currentFrame;
            long unsigned int prevFrameID = 0;
            int message = 0; // TODO
            bool isKF;
            int state;
            Sophus::SE3f currentPose;

            while(true) {
                if(prevFrameID != this->mpTracker->mLastFrame.mnId) {
                    currentFrame = Frame(this->mpTracker->mLastFrame, true);
                    prevFrameID = currentFrame.mnId;
                    state = this->mpTracker->mState.getID();

                    if(currentFrame.mpReferenceKF && currentFrame.mpReferenceKF->mnId == prevFrameID) {
                        isKF = true;
                    } else {
                        isKF = false;
                    }
                    
                    currentPose = currentFrame.GetPose();
                    webSocket.sendBinary(ExternalMapViewer::poseToBinary(currentPose.rotationMatrix(), currentPose.rotationMatrix().inverse()*currentPose.translation(), state, message, isKF));
                } 

                if (valuesPushed) {
                    webSocket.sendBinary(ExternalMapViewer::coordsToBinary(pushedValues));
                    valuesPushed = false;
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
}

std::vector<uint8_t> ExternalMapViewer::poseToBinary(const Sophus::Matrix3f& rotationMatrix, const Sophus::Vector3f& translation, const int state, const int message, const bool KF) {
    
    size_t outputSize = sizeof(float)*12 + sizeof(int)*2 + sizeof(bool)*2;
    std::vector<uint8_t> binaryOutput(outputSize);
    bool isPose = true;
    
    memcpy(binaryOutput.data(), &isPose, sizeof(bool));
    memcpy(binaryOutput.data() + sizeof(bool), rotationMatrix.data(), 9*sizeof(float));
    memcpy(binaryOutput.data() + sizeof(bool) + 9*sizeof(float), translation.data(), 3*sizeof(float));
    memcpy(binaryOutput.data() + sizeof(bool) + 12*sizeof(float), &state, sizeof(int));
    memcpy(binaryOutput.data() + sizeof(bool) + 12*sizeof(float) + sizeof(int), &message, sizeof(int));
    memcpy(binaryOutput.data() + sizeof(bool) + 12*sizeof(float) + 2*sizeof(int), &KF, sizeof(bool));

    return binaryOutput;
}

std::vector<uint8_t> ExternalMapViewer::coordsToBinary(const std::vector<float>& coords) {
    size_t outputSize = sizeof(float)*3 + sizeof(bool);
        std::vector<uint8_t> binaryOutput(outputSize);
        bool isPose = false;
        
        memcpy(binaryOutput.data(), &isPose, sizeof(bool));
        memcpy(binaryOutput.data() + sizeof(bool), coords.data(), 3*sizeof(float));

        return binaryOutput;
}

}