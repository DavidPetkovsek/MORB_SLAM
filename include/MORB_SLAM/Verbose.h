#pragma once
#include <iostream>

#ifdef FactoryEngine
#include <fe/Logger.hpp>
#endif

namespace MORB_SLAM
{

class Verbose {
 public:
    enum eLevel {
        SUCCESS=0,
        INFO=1,
        WARNING=2,
        ERROR=3,
        CRITICAL=4,
        FATAL=5,
        TODO=6,
        DEBUG=7
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev) {
        #ifdef FactoryEngine
            switch(lev){
                case SUCCESS:
                    fe::Logger::success(str);
                    break;
                case INFO:
                    fe::Logger::info(str);
                    break;
                case WARNING:
                    fe::Logger::warning(str);
                    break;
                case ERROR:
                    fe::Logger::error(str);
                    break;
                case CRITICAL:
                    fe::Logger::critical(str);
                    break;
                case FATAL:
                    fe::Logger::fatal(str);
                    break;
                case TODO:
                    fe::Logger::todo(str);
                    break;
                case DEBUG:
                    fe::Logger::debug(str);
                    break;
                default:
                    fe::Logger::info(str);
                    break;
            }
        #else
            std::cout << "Level: " << lev << " | " << str << std::endl;
        #endif
    }

    static void SetTh(eLevel _th) {
        th = _th;
    }
};

}