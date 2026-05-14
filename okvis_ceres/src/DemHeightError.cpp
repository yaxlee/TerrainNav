/**
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENSE file for details
 */

/**
 * @file DemHeightError.cpp
 * @brief Implementation of the DEM height constraint error term.
 */

#include <okvis/ceres/DemHeightError.hpp>
#include <okvis/kinematics/operators.hpp>

namespace okvis {
namespace ceres {

bool DemHeightError::EvaluateWithMinimalJacobians(
    double const* const* parameters,
    double* residuals,
    double** jacobians,
    double** jacobiansMinimal) const {

  // --- Extract T_WS from parameter block ---
  Eigen::Map<const Eigen::Vector3d> r_WS(&parameters[0][0]);
  const Eigen::Quaterniond q_WS(parameters[0][6], parameters[0][3],
                                 parameters[0][4], parameters[0][5]);
  const Eigen::Matrix3d C_WS = q_WS.toRotationMatrix();

  // --- Compute antenna/sensor position in G frame ---
  // p_antenna_W = r_WS + C_WS * r_SA
  // p_antenna_G = C_GW * p_antenna_W + r_GW
  const Eigen::Vector3d p_antenna_W = r_WS + C_WS * r_SA_;
  const Eigen::Vector3d p_antenna_G = C_GW_ * p_antenna_W + r_GW_;

  // --- Residual: difference in height (z component) ---
  residuals[0] = (p_antenna_G.z() - h_sensor_) / sigma_h_;

  // --- Jacobians ---
  if (jacobians != nullptr && jacobians[0] != nullptr) {
    // Minimal Jacobian (1 x 6): [d/d(r_WS) | d/d(phi)]
    //
    // d(p_G.z)/d(r_WS) = C_GW.row(2)            (1 x 3)
    // d(p_G.z)/d(phi)  = C_GW.row(2) * d(C_WS * r_SA)/d(phi)
    //                  = C_GW.row(2) * (-C_WS * [r_SA]x)   (1 x 3)
    //   because d(C_WS * r_SA)/d(phi) = -C_WS * crossMx(r_SA)

    Eigen::Matrix<double, 1, 6> J_min;
    J_min.head<3>() = C_GW_.row(2);
    J_min.tail<3>() = -C_GW_.row(2) * C_WS * okvis::kinematics::crossMx(r_SA_);
    J_min /= sigma_h_;

    // Lift from minimal (6) to non-minimal (7) representation
    Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
    PoseManifold::minusJacobian(parameters[0], J_lift.data());

    // Full non-minimal Jacobian (1 x 7)
    Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J(jacobians[0]);
    J = J_min * J_lift;

    if (jacobiansMinimal != nullptr && jacobiansMinimal[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J_m(jacobiansMinimal[0]);
      J_m = J_min;
    }
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
