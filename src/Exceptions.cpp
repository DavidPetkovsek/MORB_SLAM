#include "MORB_SLAM/Exceptions.hpp"

namespace MORB_SLAM {
    
    ResetActiveMapSignal::ResetActiveMapSignal():
        std::runtime_error("Request Active Map Reset"){}

}