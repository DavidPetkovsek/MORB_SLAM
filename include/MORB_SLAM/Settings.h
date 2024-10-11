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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/core/core.hpp>
#include <iostream>


namespace MORB_SLAM {


class Settings {
public:
  static cv::FileStorage loadFile(const std::string configFile) {
    cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
      std::cerr << "[ERROR]: could not open configuration file at: " << configFile << std::endl;
      std::cerr << "Aborting..." << std::endl;
      throw std::invalid_argument("[ERROR]: could not open configuration file at: " + configFile);
    } else {
      std::cout << "Loading settings from " << configFile << std::endl;
      return fSettings;
    }
  }

  template <typename T>
  static T readParameter(const cv::FileStorage& settings, const std::string& name, bool& found, const bool required = true) {
    cv::FileNode node = settings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
        throw std::invalid_argument(name + " required parameter does not exist, aborting...");
      } else {
        std::cerr << name << " optional parameter does not exist..." << std::endl;
        found = false;
        return T();
      }
    } else {
      found = true;
      return (T)node;
    }
  }

  template <>
  bool readParameter<bool>(const cv::FileStorage& settings, const std::string& name, bool& found, const bool required) {
    cv::FileNode node = settings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
        throw std::invalid_argument(name + " required parameter does not exist, aborting...");
      } else {
        std::cerr << name << " optional parameter does not exist..." << std::endl;
        found = false;
        return false;
      }
    } else if(node.isString()) {
      found = true;
      std::string s = node.string();
      if(s=="y"||s=="Y"||s=="yes"||s=="Yes"||s=="YES"||s=="true"||s=="True"||s=="TRUE"||s=="on"||s=="On"||s=="ON")
        return true;
      else if(s=="n"||s=="N"||s=="no"||s=="No"||s=="NO"||s=="false"||s=="False"||s=="FALSE"||s=="off"||s=="Off"||s=="OFF")
        return false;
    }
    throw std::invalid_argument(name + " bool setting was not set to a valid string");
  }

  template <>
  float readParameter<float>(const cv::FileStorage& settings, const std::string& name, bool& found, const bool required) {
    cv::FileNode node = settings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
        throw std::invalid_argument(name + " required parameter does not exist, aborting...");
      } else {
        std::cerr << name << " optional parameter does not exist..." << std::endl;
        found = false;
        return 0.0f;
      }
    } else if (!node.isReal()) {
      std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
      throw std::invalid_argument(name + " parameter must be a real number, aborting...");
    } else {
      found = true;
      return node.real();
    }
  }

  template <>
  int readParameter<int>(const cv::FileStorage& settings, const std::string& name, bool& found, const bool required) {
    cv::FileNode node = settings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
        throw std::invalid_argument(name + " required parameter does not exist, aborting...");
      } else {
        std::cerr << name << " optional parameter does not exist..." << std::endl;
        found = false;
        return 0;
      }
    } else if (!node.isInt()) {
      std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
      throw std::invalid_argument(name + " parameter must be an integer number, aborting...");
    } else {
      found = true;
      return node.operator int();
    }
  }

  template <>
  std::string readParameter<std::string>(const cv::FileStorage& settings, const std::string& name, bool& found, const bool required) {
    cv::FileNode node = settings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
        throw std::invalid_argument(name + " required parameter does not exist, aborting...");
      } else {
        std::cerr << name << " optional parameter does not exist..." << std::endl;
        found = false;
        return std::string();
      }
    } else if (!node.isString()) {
      std::cerr << name << " parameter must be a std::string, aborting..." << std::endl;
      throw std::invalid_argument(name + " parameter must be an integer number, aborting...");
    } else {
      found = true;
      return node.string();
    }
  }

  template <>
  cv::Mat readParameter<cv::Mat>(const cv::FileStorage& settings, const std::string& name, bool& found, const bool required) {
    cv::FileNode node = settings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
        throw std::invalid_argument(name + " required parameter does not exist, aborting...");
      } else {
        std::cerr << name << " optional parameter does not exist..." << std::endl;
        found = false;
        return cv::Mat();
      }
    } else {
      found = true;
      return node.mat();
    }
  }
  
};


}  // namespace MORB_SLAM

