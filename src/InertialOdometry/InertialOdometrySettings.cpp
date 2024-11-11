#include "MORB_SLAM/InertialOdometry/InertialOdometrySettings.hpp"
#include "MORB_SLAM/Converter.h"

namespace MORB_SLAM {


InertialOdometrySettings::InertialOdometrySettings(const std::string &configFile) {
    cv::FileStorage fSettings = loadFile(configFile);
    readIMU(fSettings);
    std::cout << "\t-Loaded IMU calibration" << std::endl;
    std::cout << "----------------------------------" << std::endl;
}

void InertialOdometrySettings::readIMU(cv::FileStorage& fSettings) {
    bool found;
    noiseGyro_ = readParameter<float>(fSettings, "IMU.NoiseGyro", found);
    noiseAcc_ = readParameter<float>(fSettings, "IMU.NoiseAcc", found);
    gyroWalk_ = readParameter<float>(fSettings, "IMU.GyroWalk", found);
    accWalk_ = readParameter<float>(fSettings, "IMU.AccWalk", found);
    accFrequency_ = readParameter<float>(fSettings, "IMU.AccFrequency", found);
    gyroFrequency_ = readParameter<float>(fSettings, "IMU.GyroFrequency", found);

    cv::Mat cvTbc = readParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
    Tbc_ = Converter::toSophus(cvTbc);

    fastIMUInit_ = readParameter<bool>(fSettings, "IMU.FastIMUInit", found, false);
    if (!found) fastIMUInit_ = false;
    stationaryIMUInit_ = readParameter<bool>(fSettings, "IMU.StationaryIMUInit", found, false);
    if (!found) stationaryIMUInit_ = false;
}

std::ostream& operator<<(std::ostream& output, const InertialOdometrySettings& settings) {
    output << "IMU settings: " << std::endl;
    output << "\t-Gyro noise: " << settings.noiseGyro_ << std::endl;
    output << "\t-Accelerometer noise: " << settings.noiseAcc_ << std::endl;
    output << "\t-Gyro walk: " << settings.gyroWalk_ << std::endl;
    output << "\t-Accelerometer walk: " << settings.accWalk_ << std::endl;
    output << "\t-Gyro frequency: " << settings.gyroFrequency_ << std::endl;
    output << "\t-Accelerometer frequency: " << settings.accFrequency_ << std::endl;
    output << "\t-Fast IMU init enabled: " << settings.fastIMUInit_ << std::endl;
    output << "\t-Stationary IMU init enabled: " << settings.stationaryIMUInit_ << std::endl;
    
    return output;
}


} // namespace MORB_SLAM