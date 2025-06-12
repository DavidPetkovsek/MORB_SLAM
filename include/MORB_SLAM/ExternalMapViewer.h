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

#include "MORB_SLAM/Packet.hpp"
#include "MORB_SLAM/Verbose.h"

namespace MORB_SLAM {

class ExternalMapViewer {
    public:
        ExternalMapViewer(const std::string& _serverAddress, const int _serverPort);
        virtual ~ExternalMapViewer();

        std::mutex mMutexEMV;
        std::condition_variable mCondvarEMV;

        static std::vector<uint8_t> slamDataToBinary(const Packet &packet);
        static std::vector<uint8_t> coordsToBinary(const std::vector<float>& coords);

        void pushValues(float x, float y, float z);
        void updateSLAM(const Packet &packet);

    private:
        std::jthread threadEMV;
        
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