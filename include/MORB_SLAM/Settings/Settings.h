/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "MORB_SLAM/Verbose.h"
#include <opencv2/core/core.hpp>
#include <string>

namespace MORB_SLAM {

class Settings {
public:
  static cv::FileStorage loadFile(const std::string configFile);

  template <typename T>
  static T readParameter(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required = true);
};

// Specializations must be declared outside the class:
template <>
bool Settings::readParameter<bool>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required);

template <>
float Settings::readParameter<float>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required);

template <>
int Settings::readParameter<int>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required);

template <>
std::string Settings::readParameter<std::string>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required);

template <>
cv::Mat Settings::readParameter<cv::Mat>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required);

} // namespace MORB_SLAM
