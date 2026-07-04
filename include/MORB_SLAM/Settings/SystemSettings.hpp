#pragma once

#include "MORB_SLAM/Settings/Settings.h"

namespace MORB_SLAM {


class SystemSettings : public Settings {
public:
    /* Constructor from file */
    SystemSettings(const std::string& configFile);

    /* Ostream operator overloading to dump settings to the terminal */
    friend std::ostream& operator<<(std::ostream& output, const SystemSettings& settings);

    /* Getter methods */
    int nFeatures() const { return nFeatures_; }
    int nLevels() const { return nLevels_; }
    float initThFAST() const { return initThFAST_; }
    float minThFAST() const { return minThFAST_; }
    float scaleFactor() const { return scaleFactor_; }

    float keyFrameSize() const { return keyFrameSize_; }
    float keyFrameLineWidth() const { return keyFrameLineWidth_; }
    float graphLineWidth() const { return graphLineWidth_; }
    float pointSize() const { return pointSize_; }
    float cameraSize() const { return cameraSize_; }
    float cameraLineWidth() const { return cameraLineWidth_; }
    float viewPointX() const { return viewPointX_; }
    float viewPointY() const { return viewPointY_; }
    float viewPointZ() const { return viewPointZ_; }
    float viewPointF() const { return viewPointF_; }
    float imageViewerScale() const { return imageViewerScale_; }

    const std::string &atlasLoadFile() const { return sLoadFrom_; }
    const std::string &atlasSaveFile() const { return sSaveto_; }

    float thFarPoints() const { return thFarPoints_; }
    bool activeLoopClosing() const { return activeLoopClosing_; }
    bool newMapRelocalization() const { return newMapRelocalization_; }

private:
    void readORB(cv::FileStorage& fSettings);
    void readViewer(cv::FileStorage& fSettings);
    void readLoadAndSave(cv::FileStorage& fSettings);
    void readOtherParameters(cv::FileStorage& fSettings);

    /* ORB stuff */
    int nFeatures_;
    float scaleFactor_;
    int nLevels_;
    int initThFAST_, minThFAST_;

    /* Viewer stuff */
    float keyFrameSize_;
    float keyFrameLineWidth_;
    float graphLineWidth_;
    float pointSize_;
    float cameraSize_;
    float cameraLineWidth_;
    float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
    float imageViewerScale_;

    /* Save & load maps */
    std::string sLoadFrom_, sSaveto_;

    /* Other stuff */
    float thFarPoints_;
    bool activeLoopClosing_;
    bool newMapRelocalization_;
};


} // namespace MORB_SLAM