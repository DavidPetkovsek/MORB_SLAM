#include <MORB_SLAM/ExternalMapViewer.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <iostream>
#include <string>

#include <condition_variable>
#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/System.h"
#include "MORB_SLAM/Tracking.h"

namespace MORB_SLAM {

ExternalMapViewer::ExternalMapViewer(const System_ptr& pSystem, const std::string& _serverAddress, const int _serverPort):
    mpTracker(pSystem->mpTracker),
    mServerAddress(_serverAddress),
    mServerPort(_serverPort),
    mServer(_serverPort, _serverAddress),
    mbValuesPushed(false),
    mbSlamUpdated(false),
    mbFirstClientConnected(false) {
        
        mServer.setOnClientMessageCallback([this](std::shared_ptr<ix::ConnectionState> connectionState, ix::WebSocket & webSocket, const ix::WebSocketMessagePtr & msg) {
            if (msg->type == ix::WebSocketMessageType::Open) {
                std::cout << "New client connected to EMV WebSocket server..." << std::endl;
                std::cout << "id: " << connectionState->getId() << std::endl;
                std::cout << "Uri: " << msg->openInfo.uri << std::endl;
                mbFirstClientConnected = true;
            }
        });

        auto res = mServer.listen();
        if (!res.first) {
            std::cerr << res.second << std::endl;
            return;
        }

        std::cout << "Starting ExternalMapViewer WebSocket server..." << std::endl;
        mServer.start();
        
        std::cout << "Creating ExternalMapViewer thread" << std::endl;
        threadEMV = std::jthread(&ExternalMapViewer::run, this);

        std::cout << "Waiting for atleast one client to connect to the ExternalMapViewer socket server before continuing..." << std::endl;
        while(!mbFirstClientConnected)
            usleep(1000);
    }

ExternalMapViewer::~ExternalMapViewer() {
    threadEMV.request_stop();
    mCondvarEMV.notify_all();

    if(threadEMV.joinable()) threadEMV.join();
    
    mServer.stop();
}

void ExternalMapViewer::pushValues(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(mMutexEMV);
    mPushedValues = {x,y,z};
    mbValuesPushed = true;
    mCondvarEMV.notify_all();
}

void ExternalMapViewer::updateSLAM(const Packet &packet) {
    std::lock_guard<std::mutex> lock(mMutexEMV);
    mSlamPacket = packet;
    mbSlamUpdated = true;
    mCondvarEMV.notify_all();
}

void ExternalMapViewer::run(std::stop_token token) {
    while(!token.stop_requested()) {
        std::unique_lock<std::mutex> lock(mMutexEMV);
        mCondvarEMV.wait(lock, [this, &token]{ return (mbSlamUpdated == true || mbValuesPushed == true || token.stop_requested()); });
        
        for(auto client : mServer.getClients()) {
            if (mbSlamUpdated) {
                client->sendBinary(ExternalMapViewer::slamDataToBinary(mSlamPacket));
                mbSlamUpdated = false;
            }

            if (mbValuesPushed) {
                client->sendBinary(ExternalMapViewer::coordsToBinary(mPushedValues));
                mbValuesPushed = false;
            }
        }
    }
}

std::vector<uint8_t> ExternalMapViewer::slamDataToBinary(const Packet &packet) {

    bool isPose = true; // TODO
    int state = 1; // TODO
    bool isKF = false; // TODO
    int message = 0; // TODO
    
    Sophus::SE3f currentPose = packet.mapPose.has_value() ? packet.mapPose.value().inverse() : Sophus::SE3f();
    Sophus::SE3f deltaPose = packet.deltaPose.has_value() ? packet.deltaPose.value().inverse() : Sophus::SE3f();
    Sophus::Matrix3f poseRotation = currentPose.rotationMatrix();
    Sophus::Vector3f poseTranslation = currentPose.translation();
    Sophus::Vector3f deltaPoseTranslation = deltaPose.translation();

    size_t outputSize = sizeof(float)*15 + sizeof(int)*2 + sizeof(bool)*2;
    std::vector<uint8_t> binaryOutput(outputSize);
    
    memcpy(binaryOutput.data(), &isPose, sizeof(bool));
    memcpy(binaryOutput.data() + sizeof(bool), poseRotation.data(), 9*sizeof(float));
    memcpy(binaryOutput.data() + sizeof(bool) + 9*sizeof(float), poseTranslation.data(), 3*sizeof(float));
    memcpy(binaryOutput.data() + sizeof(bool) + 12*sizeof(float), deltaPoseTranslation.data(), 3*sizeof(float));
    memcpy(binaryOutput.data() + sizeof(bool) + 15*sizeof(float), &state, sizeof(int));
    memcpy(binaryOutput.data() + sizeof(bool) + 15*sizeof(float) + sizeof(int), &message, sizeof(int));
    memcpy(binaryOutput.data() + sizeof(bool) + 15*sizeof(float) + 2*sizeof(int), &isKF, sizeof(bool));

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


} // namespace MORB_SLAM