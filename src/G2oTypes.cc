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

#include "MORB_SLAM/G2oTypes.h"


namespace MORB_SLAM {


// SO3 FUNCTIONS
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w) {
  return ExpSO3(w[0], w[1], w[2]);
}

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z) {
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);
  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5) {
    Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
    return NormalizeRotation(res);
  } else {
    Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0 - cos(d)) / d2;
    return NormalizeRotation(res);
  }
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R) {
  const double tr = R(0, 0) + R(1, 1) + R(2, 2);
  Eigen::Vector3d w;
  w << (R(2, 1) - R(1, 2)) / 2, (R(0, 2) - R(2, 0)) / 2, (R(1, 0) - R(0, 1)) / 2;
  const double costheta = (tr - 1.0) * 0.5f;
  if (costheta > 1 || costheta < -1) return w;
  const double theta = acos(costheta);
  const double s = sin(theta);
  if (fabs(s) < 1e-5)
    return w;
  else
    return theta * w / s;
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v) {
  const double x = v[0];
  const double y = v[1];
  const double z = v[2];
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);

  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5)
    return Eigen::Matrix3d::Identity();
  else
    return Eigen::Matrix3d::Identity() + W / 2 + W * W * (1.0 / d2 - (1.0 + cos(d)) / (2.0 * d * sin(d)));
}

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v) {
  const double x = v[0];
  const double y = v[1];
  const double z = v[2];
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);

  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5) {
    return Eigen::Matrix3d::Identity();
  } else {
    return Eigen::Matrix3d::Identity() - W * (1.0 - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
  }
}

}  // namespace MORB_SLAM
