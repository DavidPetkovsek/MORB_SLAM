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

#include "MORB_SLAM/ImuTypes.h"

#include <iostream>

#include "MORB_SLAM/Converter.h"
#include "MORB_SLAM/GeometricTools.h"

namespace MORB_SLAM {

namespace IMU {

const float eps = 1e-4;

Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R) {
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y,
                                 const float &z) {
  Eigen::Matrix3f I;
  I.setIdentity();
  const float d2 = x * x + y * y + z * z;
  const float d = sqrt(d2);
  Eigen::Vector3f v;
  v << x, y, z;
  Eigen::Matrix3f W = Sophus::SO3f::hat(v);
  if (d < eps) {
    return I;
  } else {
    return I - W * (1.0f - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
  }
}

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v) {
  return RightJacobianSO3(v(0), v(1), v(2));
}

Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y,
                                        const float &z) {
  Eigen::Matrix3f I;
  I.setIdentity();
  const float d2 = x * x + y * y + z * z;
  const float d = sqrt(d2);
  Eigen::Vector3f v;
  v << x, y, z;
  Eigen::Matrix3f W = Sophus::SO3f::hat(v);

  if (d < eps) {
    return I;
  } else {
    return I + W / 2 +
           W * W * (1.0f / d2 - (1.0f + cos(d)) / (2.0f * d * sin(d)));
  }
}

Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v) {
  return InverseRightJacobianSO3(v(0), v(1), v(2));
}

IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel,
                                       const Bias &imuBias, const float &time) {
  const float x = (angVel(0) - imuBias.bwx) * time;
  const float y = (angVel(1) - imuBias.bwy) * time;
  const float z = (angVel(2) - imuBias.bwz) * time;

  const float d2 = x * x + y * y + z * z;
  const float d = sqrt(d2);

  Eigen::Vector3f v;
  v << x, y, z;
  Eigen::Matrix3f W = Sophus::SO3f::hat(v);
  if (d < eps) {
    deltaR = Eigen::Matrix3f::Identity() + W;
    rightJ = Eigen::Matrix3f::Identity();
  } else {
    // Rodrigues' rotation formula
    deltaR = Eigen::Matrix3f::Identity() + W * sin(d) / d +
             W * W * (1.0f - cos(d)) / d2;
    rightJ = Eigen::Matrix3f::Identity() - W * (1.0f - cos(d)) / d2 +
             W * W * (d - sin(d)) / (d2 * d);
  }
}

unsigned int Preintegrated::NumObjects = 0;

Preintegrated::Preintegrated(const Bias &b_, const Calib &calib) {
  Nga = calib.Cov;
  NgaWalk = calib.CovWalk;
  Initialize(b_);
  NumObjects++;
}

Preintegrated::Preintegrated(){
  NumObjects++;
}

Preintegrated::~Preintegrated(){
  NumObjects--;
}

// Copy constructor
Preintegrated::Preintegrated(Preintegrated *pImuPre)
    : dT(pImuPre->dT),
      C(pImuPre->C),
      Info(pImuPre->Info),
      Nga(pImuPre->Nga),
      NgaWalk(pImuPre->NgaWalk),
      b(pImuPre->b),
      dR(pImuPre->dR),
      dV(pImuPre->dV),
      dP(pImuPre->dP),
      JRg(pImuPre->JRg),
      JVg(pImuPre->JVg),
      JVa(pImuPre->JVa),
      JPg(pImuPre->JPg),
      JPa(pImuPre->JPa),
      avgA(pImuPre->avgA),
      avgW(pImuPre->avgW),
      bu(pImuPre->bu),
      db(pImuPre->db),
      mvMeasurements(pImuPre->mvMeasurements) {NumObjects++;}

void Preintegrated::CopyFrom(std::shared_ptr<Preintegrated> pImuPre) {
  dT = pImuPre->dT;
  C = pImuPre->C;
  Info = pImuPre->Info;
  Nga = pImuPre->Nga;
  NgaWalk = pImuPre->NgaWalk;
  b.CopyFrom(pImuPre->b);
  dR = pImuPre->dR;
  dV = pImuPre->dV;
  dP = pImuPre->dP;
  JRg = pImuPre->JRg;
  JVg = pImuPre->JVg;
  JVa = pImuPre->JVa;
  JPg = pImuPre->JPg;
  JPa = pImuPre->JPa;
  avgA = pImuPre->avgA;
  avgW = pImuPre->avgW;
  bu.CopyFrom(pImuPre->bu);
  db = pImuPre->db;
  mvMeasurements = pImuPre->mvMeasurements;
}

void Preintegrated::Initialize(const Bias &b_) {
  dR.setIdentity();
  dV.setZero();
  dP.setZero();
  JRg.setZero();
  JVg.setZero();
  JVa.setZero();
  JPg.setZero();
  JPa.setZero();
  C.setZero();
  Info.setZero();
  db.setZero();
  b = b_;
  bu = b_;
  avgA.setZero();
  avgW.setZero();
  dT = 0.0f;
  mvMeasurements.clear();
}

void Preintegrated::Reintegrate() {
  std::unique_lock<std::mutex> lock(mMutex);
  const std::vector<integrable> aux = mvMeasurements;
  Initialize(bu);
  for (size_t i = 0; i < aux.size(); i++)
    IntegrateNewMeasurement(aux[i].a, aux[i].w, aux[i].t);
}

void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration,
                                            const Eigen::Vector3f &angVel,
                                            const float &dt) {
  mvMeasurements.push_back(integrable(acceleration, angVel, dt));

  // A is the Jacobian Matrix of the the robot state variables wrt each other (θ_x, θ_y, θ_z, v_x, v_y, v_z, p_x, p_y, p_z)
  /* (each term is a 3x3 block)
  A = | ∂θ/∂θ   ∂θ/∂v   ∂θ/∂p |
      | ∂v/∂θ   ∂v/∂v   ∂v/∂p |
      | ∂p/∂θ   ∂p/∂v   ∂p/∂p |
  */
  Eigen::Matrix<float, 9, 9> A;
  // ∂v/∂v = ∂p/∂p = I (Identity Matrix), since dx/dx = 1, dx/dy = 0, etc.
  // ∂θ/∂v = ∂θ/∂p = ∂v/∂p = 0 (Null Matrix)
  A.setIdentity();

  // B is the Jacobian Matrix of the the state variables wrt the noise vars (noise_gyro * 3, noise_accel * 3)
  /* (each term is a 3x3 block)
  B = | ∂θ/∂ng   ∂θ/∂na |
      | ∂v/∂ng   ∂v/∂na |
      | ∂v/∂ng   ∂p/∂na |
  */
  Eigen::Matrix<float, 9, 6> B;
  // ∂θ/∂na = ∂v/∂ng = ∂v/∂ng = 0 (Null Matrix)
  B.setZero();

  Eigen::Vector3f a, w;
  a << acceleration(0) - b.bax, acceleration(1) - b.bay, acceleration(2) - b.baz;
  w << angVel(0) - b.bwx, angVel(1) - b.bwy, angVel(2) - b.bwz;

  // the robot can only move in the direction it's facing, therefore the acceleration relative to the origin is
  //   the acceleration vector a rotated by the current heading rotation matrix dR
  avgA = (dT * avgA + dt * dR * a) / (dT + dt);
  // similarly, the real displacement is dR*d
  // note: this uses the previous value of dV
  dP = dP + dV * dt + dR * (0.5f * a * dt * dt);
  // the real velocity is dR*v
  dV = dV + dR * (a * dt);

  avgW = (dT * avgW + dt * w) / (dT + dt);

  // a_hat is effectively a matrix representation of a, allowing for matrix multiplication
  Eigen::Matrix<float, 3, 3> a_hat = Sophus::SO3f::hat(a);

  // the ∂/∂θ blocks are the variable's Jacobian wrt to the origin's rotation, not to the robot
  // dR_inverse = -dR is the heading of the origin relative to the robot
  // therefore the robot's velocity changes wrt the origin's rotation at -dR*(a_hat*dt)
  // ∂v/∂θ
  A.block<3, 3>(3, 0) = -dR * (a_hat * dt);
  // ∂p/∂θ
  A.block<3, 3>(6, 0) = -dR * (0.5f * a_hat * dt * dt);
  // ∂p/∂v(x = v_x*dt -> ∂x/∂v_x = dt, ...)
  A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<float, 3>(dt, dt, dt);

  // B represents how the state variables change with a small change in accel or angular velo.
  // ∂v/∂na = dR*dt, since a small change in acceleration changes velocity by a factor of dt
  B.block<3, 3>(3, 3) = dR * dt;
  // ∂v/∂na
  B.block<3, 3>(6, 3) = dR * 0.5f * dt * dt;

  // Update position and velocity jacobians wrt bias correction
  JPa = JPa + JVa * dt - dR * (0.5f * dt * dt);
  JPg = JPg + JVg * dt - dR * a_hat * JRg * (0.5f * dt * dt);
  JVa = JVa - dR * dt;
  JVg = JVg - dR * a_hat * JRg * dt;

  // dRi.deltaR is a rotation matrix resulting from Rodrigues' Rotation Formula, which represents a change in rotation with angVel and dt
  IntegratedRotation dRi(angVel, b, dt);
  // update the robot's heading. NormalizeRotation strips any rounding errors from the update step (ensures the result's a valid rotation matrix) 
  dR = NormalizeRotation(dR * dRi.deltaR);

  // the change in the robot's heading wrt a change in the origin's heading is the inverse of how the robot's heading changes
  // ∂θ/∂θ (the transpose of a rotation matrix is its inverse)
  A.block<3, 3>(0, 0) = dRi.deltaR.transpose();

  // dRi.rightJ is the right Jacobian associated with deltaR, which represents how the robot's rotation changes with a small change in the angular velo.
  // ∂θ/∂ng = rightJ*dt, since a small change in angular velo changes the robot's heading by a factor of rightJ*dt
  B.block<3, 3>(0, 0) = dRi.rightJ * dt;

  // Update covariance, which represents the uncertainty in the estimated state
  // Nga is a 6x6 diagonal matrix with the first 3 values being gyro_noise^2, last 3 being accel_noise^2
  C.block<9, 9>(0, 0) = A * C.block<9, 9>(0, 0) * A.transpose() + B * Nga * B.transpose();
  // same as Nga but with the walk noises
  C.block<6, 6>(9, 9) += NgaWalk;

  // Update rotation jacobian wrt bias correction
  // ∂θ/∂bg = -(deltaR*JRg + rightJ*dt)
  JRg = dRi.deltaR.transpose() * JRg - dRi.rightJ * dt;
  // Note: there is no JRa because the rotation is constant wrt. a change in accelerometer bias. If we had one, it would always just be 0

  // Total integrated time
  dT += dt;
}

void Preintegrated::MergePrevious(std::shared_ptr<Preintegrated> pPrev) {
  if (pPrev == shared_from_this()) return;

  std::unique_lock<std::mutex> lock1(mMutex);
  std::unique_lock<std::mutex> lock2(pPrev->mMutex);
  Bias bav;
  bav.bwx = bu.bwx;
  bav.bwy = bu.bwy;
  bav.bwz = bu.bwz;
  bav.bax = bu.bax;
  bav.bay = bu.bay;
  bav.baz = bu.baz;

  const std::vector<integrable> aux1 = pPrev->mvMeasurements;
  const std::vector<integrable> aux2 = mvMeasurements;

  Initialize(bav);
  for (size_t i = 0; i < aux1.size(); i++)
    IntegrateNewMeasurement(aux1[i].a, aux1[i].w, aux1[i].t);
  for (size_t i = 0; i < aux2.size(); i++)
    IntegrateNewMeasurement(aux2[i].a, aux2[i].w, aux2[i].t);
}

void Preintegrated::SetNewBias(const Bias &bu_) {
  std::unique_lock<std::mutex> lock(mMutex);
  bu = bu_;

  db(0) = bu_.bwx - b.bwx;
  db(1) = bu_.bwy - b.bwy;
  db(2) = bu_.bwz - b.bwz;
  db(3) = bu_.bax - b.bax;
  db(4) = bu_.bay - b.bay;
  db(5) = bu_.baz - b.baz;
}

IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  return IMU::Bias(b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz,
                   b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz);
}

Eigen::Matrix3f Preintegrated::GetDeltaRotation(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  Eigen::Vector3f dbg;
  dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
  return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
}

Eigen::Vector3f Preintegrated::GetDeltaVelocity(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  Eigen::Vector3f dbg, dba;
  dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
  dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
  return dV + JVg * dbg + JVa * dba;
}

Eigen::Vector3f Preintegrated::GetDeltaPosition(const Bias &b_) {
  std::unique_lock<std::mutex> lock(mMutex);
  Eigen::Vector3f dbg, dba;
  dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
  dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
  return dP + JPg * dbg + JPa * dba;
}

Eigen::Matrix3f Preintegrated::GetUpdatedDeltaRotation() {
  std::unique_lock<std::mutex> lock(mMutex);
  return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * db.head(3)).matrix());
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaVelocity() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dV + JVg * db.head(3) + JVa * db.tail(3);
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaPosition() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dP + JPg * db.head(3) + JPa * db.tail(3);
}

Eigen::Matrix3f Preintegrated::GetOriginalDeltaRotation() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dR;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaVelocity() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dV;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaPosition() {
  std::unique_lock<std::mutex> lock(mMutex);
  return dP;
}

Bias Preintegrated::GetOriginalBias() {
  std::unique_lock<std::mutex> lock(mMutex);
  return b;
}

Bias Preintegrated::GetUpdatedBias() {
  std::unique_lock<std::mutex> lock(mMutex);
  return bu;
}

Eigen::Matrix<float, 6, 1> Preintegrated::GetDeltaBias() {
  std::unique_lock<std::mutex> lock(mMutex);
  return db;
}

void Bias::CopyFrom(Bias &b) {
  bax = b.bax;
  bay = b.bay;
  baz = b.baz;
  bwx = b.bwx;
  bwy = b.bwy;
  bwz = b.bwz;
}

std::ostream &operator<<(std::ostream &out, const Bias &b) {
  if (b.bwx > 0) out << " ";
  out << b.bwx << ",";
  if (b.bwy > 0) out << " ";
  out << b.bwy << ",";
  if (b.bwz > 0) out << " ";
  out << b.bwz << ",";
  if (b.bax > 0) out << " ";
  out << b.bax << ",";
  if (b.bay > 0) out << " ";
  out << b.bay << ",";
  if (b.baz > 0) out << " ";
  out << b.baz;

  return out;
}

void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng,
                const float &na, const float &ngw, const float &naw) {
  mbIsSet = true;
  // ng = Gyro Noise, na = Accel. Noise, ngw = Gyro Walk Noise, naw = Accel. Walk Noise
  const float ng2 = ng * ng;
  const float na2 = na * na;
  const float ngw2 = ngw * ngw;
  const float naw2 = naw * naw;

  // Sophus/Eigen
  mTbc = sophTbc;
  mTcb = mTbc.inverse();
  Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
  CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}

Calib::Calib(const Calib &calib) {
  mbIsSet = calib.mbIsSet;
  // Sophus/Eigen parameters
  mTbc = calib.mTbc;
  mTcb = calib.mTcb;
  Cov = calib.Cov;
  CovWalk = calib.CovWalk;
}

}  // namespace IMU

}  // namespace MORB_SLAM
