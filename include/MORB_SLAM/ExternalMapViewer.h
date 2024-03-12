#pragma once

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

class ExternalMapViewer {
    public:
        ExternalMapViewer(const System_ptr &pSystem, const std::string& _serverAddress, const int _serverPort);
        virtual ~ExternalMapViewer();

        std::mutex mutexEMV;
        std::condition_variable condvarEMV;

        static std::vector<uint8_t> dataToBinary(const Sophus::Matrix3f& rotationMatrix, const Sophus::Vector3f& translation, const int state, const int message, const bool KF);
        void pushValues(float x, float y, float z);
    private:
        std::thread threadEMV;
        System_ptr mpSystem;
        // Tracking_ptr mpTracker;
        
        // Websocket host address
        const std::string serverAddress;
        // Websocket port
        const int serverPort;

        void run();
        
        int parseTrackingState(const TrackingState state);
};

}