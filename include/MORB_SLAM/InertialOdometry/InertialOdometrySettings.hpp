#pragma once

#include "MORB_SLAM/Settings/Settings.h"

#include <sophus/se3.hpp>

namespace MORB_SLAM {


class InertialOdometrySettings : public Settings {
public:
    InertialOdometrySettings(const std::string& configFile);

    void SetTbc(Sophus::SE3f Tbc) { Tbc_ = Tbc; } // TODO: rework InertialSettings to account for stereo rectification in Tbc. Use this for now

    float noiseGyro() const { return noiseGyro_; }
    float noiseAcc() const { return noiseAcc_; }
    float gyroWalk() const { return gyroWalk_; }
    float accWalk() const { return accWalk_; }
    float accFrequency() const { return accFrequency_; }
    float gyroFrequency() const { return gyroFrequency_; }
    const Sophus::SE3f &Tbc() const { return Tbc_; }

    bool fastIMUInit() const { return fastIMUInit_; }
    bool stationaryIMUInit() const { return stationaryIMUInit_; }

    friend std::ostream& operator<<(std::ostream& output, const InertialOdometrySettings& s);

private:
    void readIMU(cv::FileStorage& fSettings);

    float noiseGyro_, noiseAcc_;
    float gyroWalk_, accWalk_;
    float accFrequency_;
    float gyroFrequency_;
    Sophus::SE3f Tbc_;

    bool fastIMUInit_;
    bool stationaryIMUInit_;
};


}