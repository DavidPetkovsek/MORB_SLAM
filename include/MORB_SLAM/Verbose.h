#pragma once
#include <iostream>
#include <sstream>

#ifdef FactoryEngine
#include <fe/Logger.hpp>
#endif

namespace MORB_SLAM
{

template <class T>
concept IsPrintable = requires(std::ostream& os, const T &a){
    os << a;
};

class Verbose {
 public:
    enum eLevel {
        TODO=0,
        DEBUG=1,
        INFO=2,
        SUCCESS=3,
        WARNING=4,  // use when an non-essential part of a function is skipped
        ERROR=5,    // use when an essential part of a function is skipped
        CRITICAL=6, // use when a function is returned from early
        FATAL=7,    // use when an the system crashes
        NONE=8 
    };

    static eLevel th;

 public:
    template<typename... Args>
    static void Log(eLevel lev, Args... args) {
        if(lev < th) return;

        static_assert((IsPrintable<Args> && ...), "Error: The arguments you pass to the logger must be printable using std::cout or std::cerr!");

        std::stringstream ss{};
        (void)(ss << ... << args); // (void) ensures that the result of the fold expression is cast to void, which should eliminate the 'unused' compiler warning.
        std::string str = ss.str();

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