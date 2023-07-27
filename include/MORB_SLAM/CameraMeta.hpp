#pragma once

namespace MORB_SLAM{
    class CameraMeta{
    protected:
        cv::Mat mK;
        Eigen::Matrix3f mK_;
        float fx;
        float fy;
        float cx;
        float cy;
        float invfx;
        float invfy;
        cv::Mat mDistCoef;
    };

    class StereoCameraMeta{
    protected:
        /** @brief = mb/fx*/
        float mbf;
        float mb;
    };
}