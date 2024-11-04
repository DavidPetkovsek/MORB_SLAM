#include "MORB_SLAM/Settings/SystemSettings.hpp"

namespace MORB_SLAM {


SystemSettings::SystemSettings(const std::string& configFile) {
    cv::FileStorage fSettings = loadFile(configFile);
    readORB(fSettings);
    std::cout << "\t-Loaded ORB settings" << std::endl;
    readViewer(fSettings);
    std::cout << "\t-Loaded viewer settings" << std::endl;
    readLoadAndSave(fSettings);
    std::cout << "\t-Loaded Atlas settings" << std::endl;
    readOtherParameters(fSettings);
    std::cout << "\t-Loaded misc parameters" << std::endl;
    std::cout << "----------------------------------" << std::endl;
}

void SystemSettings::readORB(cv::FileStorage& fSettings) {
    bool found;
    nFeatures_ = readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
    scaleFactor_ = readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
    nLevels_ = readParameter<int>(fSettings, "ORBextractor.nLevels", found);
    initThFAST_ = readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
    minThFAST_ = readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
}

void SystemSettings::readViewer(cv::FileStorage& fSettings) {
    bool found;
    keyFrameSize_ = readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
    keyFrameLineWidth_ = readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
    graphLineWidth_ = readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
    pointSize_ = readParameter<float>(fSettings, "Viewer.PointSize", found);
    cameraSize_ = readParameter<float>(fSettings, "Viewer.CameraSize", found);
    cameraLineWidth_ = readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
    viewPointX_ = readParameter<float>(fSettings, "Viewer.ViewpointX", found);
    viewPointY_ = readParameter<float>(fSettings, "Viewer.ViewpointY", found);
    viewPointZ_ = readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
    viewPointF_ = readParameter<float>(fSettings, "Viewer.ViewpointF", found);
    imageViewerScale_ = readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

    if (!found) imageViewerScale_ = 1.0f;
}

void SystemSettings::readLoadAndSave(cv::FileStorage& fSettings) {
    bool found;
    sLoadFrom_ = readParameter<std::string>(fSettings, "System.LoadAtlasFromFile", found, false);
    sSaveto_ = readParameter<std::string>(fSettings, "System.SaveAtlasToFile", found, false);
}

void SystemSettings::readOtherParameters(cv::FileStorage& fSettings) {
    bool found;
    thFarPoints_ = readParameter<float>(fSettings, "System.thFarPoints", found, false);
    activeLoopClosing_ = readParameter<bool>(fSettings, "System.activeLoopClosing", found, false);
    if (!found) activeLoopClosing_ = true;
    newMapRelocalization_ = readParameter<bool>(fSettings, "System.NewMapRelocalization", found, false);
    if (!found) newMapRelocalization_ = false;
}

std::ostream& operator<<(std::ostream& output, const SystemSettings& settings) {
    output << "SLAM settings: " << std::endl;
    output << "\t-Features per image: " << settings.nFeatures_ << std::endl;
    output << "\t-ORB scale factor: " << settings.scaleFactor_ << std::endl;
    output << "\t-ORB number of scales: " << settings.nLevels_ << std::endl;
    output << "\t-Initial FAST threshold: " << settings.initThFAST_ << std::endl;
    output << "\t-Min FAST threshold: " << settings.minThFAST_ << std::endl;

    return output;
}


} //namespace MORB_SLAM