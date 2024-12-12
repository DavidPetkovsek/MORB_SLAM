#pragma once

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

class ExternalMapViewer {
    public:
        ExternalMapViewer(const System_ptr &pSystem, const std::string& _serverAddress, const int _serverPort);
        virtual ~ExternalMapViewer();

        std::mutex mMutexEMV;
        std::condition_variable mCondvarEMV;

        static std::vector<uint8_t> slamDataToBinary(const Packet &packet);
        static std::vector<uint8_t> coordsToBinary(const std::vector<float>& coords);

        void pushValues(float x, float y, float z);
        void updateSLAM(const Packet &packet);

    private:
        std::jthread threadEMV;
        Tracking_ptr mpTracker;
        
        // Websocket Server
        ix::WebSocketServer mServer;
        const std::string mServerAddress;
        const int mServerPort;
        bool mbFirstClientConnected;
        
        std::vector<float> mPushedValues;
        bool mbValuesPushed;

        Packet mSlamPacket;
        bool mbSlamUpdated;

        
        void run(std::stop_token token);
};

}