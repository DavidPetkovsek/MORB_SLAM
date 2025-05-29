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
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev) {
        #ifdef FactoryEngine
            fe::Logger::info("Level: ", lev, " | ", str);
        #else
            std::cout << "Level: " << lev << " | " << str << std::endl;
        #endif
    }

    static void SetTh(eLevel _th) {
        th = _th;
    }
};

}