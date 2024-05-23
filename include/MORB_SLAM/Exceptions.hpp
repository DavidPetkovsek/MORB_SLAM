#pragma once

#include <stdexcept>

namespace MORB_SLAM {
    class ResetActiveMapSignal: public std::runtime_error {
    public:
        ResetActiveMapSignal();
    };
}