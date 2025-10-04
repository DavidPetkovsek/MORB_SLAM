#include "MORB_SLAM/Settings/Settings.h"

#include <stdexcept>

namespace MORB_SLAM {

cv::FileStorage Settings::loadFile(const std::string configFile) {
  cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
      Verbose::Log(Verbose::FATAL, "Could not open configuration file at: ", configFile);
      throw std::invalid_argument("Could not open configuration file at: " + configFile);
    } else {
      Verbose::Log(Verbose::INFO, "Loading settings from ", configFile);
      return fSettings;
    }
}

template <typename T>
T Settings::readParameter(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required) {
  cv::FileNode node = settings[name];
  if (node.empty()) {
    if (required) {
      Verbose::Log(Verbose::FATAL, name, " required parameter does not exist, aborting...");
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      Verbose::Log(Verbose::WARNING, name, " optional parameter does not exist...");
      found = false;
      return T();
    }
  } else {
    found = true;
    return (T)node;
  }
}

template <>
bool Settings::readParameter<bool>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required) {
  cv::FileNode node = settings[name];
  if (node.empty()) {
    if (required) {
      Verbose::Log(Verbose::FATAL, name, " required parameter does not exist, aborting...");
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      Verbose::Log(Verbose::WARNING, name, " optional parameter does not exist...");
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
  Verbose::Log(Verbose::FATAL, name, " bool setting was not set to a valid string");
  throw std::invalid_argument(name + " bool setting was not set to a valid string");
}

template <>
float Settings::readParameter<float>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required) {
  cv::FileNode node = settings[name];
  if (node.empty()) {
    if (required) {
      Verbose::Log(Verbose::FATAL, name, " required parameter does not exist, aborting...");
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      Verbose::Log(Verbose::WARNING, name, " optional parameter does not exist...");
      found = false;
      return 0.0f;
    }
  } else if (!node.isReal()) {
    Verbose::Log(Verbose::FATAL, name, " parameter must be a real number, aborting...");
    throw std::invalid_argument(name + " parameter must be a real number, aborting...");
  } else {
    found = true;
    return node.real();
  }
}

template <>
int Settings::readParameter<int>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required) {
  cv::FileNode node = settings[name];
  if (node.empty()) {
    if (required) {
      Verbose::Log(Verbose::FATAL, name, " required parameter does not exist, aborting...");
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      Verbose::Log(Verbose::WARNING, name, " optional parameter does not exist...");
      found = false;
      return 0;
    }
  } else if (!node.isInt()) {
    Verbose::Log(Verbose::FATAL, name, " parameter must be an integer number, aborting...");
    throw std::invalid_argument(name + " parameter must be an integer number, aborting...");
  } else {
    found = true;
    return node.operator int();
  }
}

template <>
std::string
Settings::readParameter<std::string>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required) {
  cv::FileNode node = settings[name];
  if (node.empty()) {
    if (required) {
      Verbose::Log(Verbose::FATAL, name, " required parameter does not exist, aborting...");
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      Verbose::Log(Verbose::WARNING, name, " optional parameter does not exist...");
      found = false;
      return std::string();
    }
  } else if (!node.isString()) {
    Verbose::Log(Verbose::FATAL, name, " parameter must be a std::string, aborting...");
    throw std::invalid_argument(name + " parameter must be a std::string, aborting...");
  } else {
    found = true;
    return node.string();
  }
}

template <>
cv::Mat Settings::readParameter<cv::Mat>(const cv::FileStorage &settings, const std::string &name, bool &found, const bool required) {
  cv::FileNode node = settings[name];
  if (node.empty()) {
    if (required) {
      Verbose::Log(Verbose::FATAL, name, " required parameter does not exist, aborting...");
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      Verbose::Log(Verbose::WARNING, name, " optional parameter does not exist...");
      found = false;
      return cv::Mat();
    }
  } else {
    found = true;
    return node.mat();
  }
}

} // namespace MORB_SLAM
