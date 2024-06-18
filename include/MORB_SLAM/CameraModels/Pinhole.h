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

#include <assert.h>
#include <memory>
#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/TwoViewReconstruction.h"

namespace MORB_SLAM {
class Pinhole : public GeometricCamera {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& boost::serialization::base_object<GeometricCamera>(*this);
  }

 public:
  Pinhole() {
    mvParameters.resize(4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }
  Pinhole(const std::vector<float> _vParameters) : GeometricCamera(_vParameters) {
    assert(mvParameters.size() == 4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }

  Pinhole(Pinhole* pPinhole) : GeometricCamera(pPinhole->mvParameters) {
    assert(mvParameters.size() == 4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }

  virtual ~Pinhole() {}

  virtual cv::Point2f project(const cv::Point3f& p3D) const override;
  virtual Eigen::Vector2d project(const Eigen::Vector3d& v3D) const override;
  virtual Eigen::Vector2f project(const Eigen::Vector3f& v3D) const override;
  virtual Eigen::Vector2f projectMat(const cv::Point3f& p3D) const override;

  virtual Eigen::Vector3f unprojectEig(const cv::Point2f& p2D) const override;
  virtual cv::Point3f unproject(const cv::Point2f& p2D) const override;

  virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D) const override;

  virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int>& vMatches12, Sophus::SE3f& T21, std::vector<cv::Point3f>& vP3D, std::vector<bool>& vbTriangulated) const override;

  virtual cv::Mat toK() const override;
  virtual Eigen::Matrix3f toK_()const override;

  virtual bool epipolarConstrain(const std::shared_ptr<const GeometricCamera> &pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc) const override;

  friend std::ostream& operator<<(std::ostream& os, const Pinhole& ph);
  friend std::istream& operator>>(std::istream& os, Pinhole& ph);

  virtual bool IsEqual(const std::shared_ptr<GeometricCamera> &pCam) const override;
  virtual bool IsEqual(const std::shared_ptr<const GeometricCamera> &pCam) const override;

};
}  // namespace MORB_SLAM