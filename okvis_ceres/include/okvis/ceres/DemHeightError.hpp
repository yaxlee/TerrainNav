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
 * @file ceres/DemHeightError.hpp
 * @brief Header for the DEM height constraint error term.
 */

#ifndef INCLUDE_OKVIS_CERES_DEMHEIGHTERROR_HPP_
#define INCLUDE_OKVIS_CERES_DEMHEIGHTERROR_HPP_

#include <ceres/sized_cost_function.h>
#include <Eigen/Core>

#include <okvis/kinematics/Transformation.hpp>
#include <okvis/ceres/ErrorInterface.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>

namespace okvis {
namespace ceres {

/**
 * @brief 1-DOF height constraint from a Digital Elevation Model (DEM).
 *
 * Constrains the z-component of the sensor position in the GPS/global frame {G}
 * to match a height queried from a DEM. T_GW must be fixed before adding this factor.
 *
 * Residual dimension: 1
 * Parameter block:   T_WS (7 DOF)
 */
class DemHeightError
    : public ::ceres::SizedCostFunction<1 /* residuals */,
                                        7 /* T_WS */>,
      public ErrorInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ::ceres::SizedCostFunction<1, 7> base_t;
  static const int kNumResiduals = 1;

  /**
   * @brief Constructor.
   * @param h_sensor   Expected height of the sensor in G frame [m]
   *                   = h_DEM + d_above_ground.
   * @param sigma_h    1-sigma height uncertainty [m].
   * @param r_SA       Antenna/sensor offset in IMU frame (reused as body offset).
   * @param T_GW       Fixed transformation from World to GPS/global frame.
   */
  DemHeightError(double h_sensor,
                 double sigma_h,
                 const Eigen::Vector3d& r_SA,
                 const okvis::kinematics::Transformation& T_GW)
      : h_sensor_(h_sensor),
        sigma_h_(sigma_h),
        r_SA_(r_SA),
        C_GW_(T_GW.C()),
        r_GW_(T_GW.r()) {}

  virtual ~DemHeightError() = default;

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const override {
    return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
  }

  virtual bool EvaluateWithMinimalJacobians(double const* const* parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const override;

  // ErrorInterface
  int residualDim() const override { return kNumResiduals; }
  int parameterBlocks() const override { return 1; }
  int parameterBlockDim(int /*idx*/) const override { return 7; }
  std::string typeInfo() const override { return "DemHeightError"; }

 private:
  double h_sensor_;          ///< Expected sensor height in G frame [m].
  double sigma_h_;           ///< Height uncertainty (1-sigma) [m].
  Eigen::Vector3d r_SA_;     ///< Sensor-to-antenna offset in IMU (S) frame.
  Eigen::Matrix3d C_GW_;     ///< Rotation part of T_GW (fixed).
  Eigen::Vector3d r_GW_;     ///< Translation part of T_GW (fixed).
};

}  // namespace ceres
}  // namespace okvis

#endif  // INCLUDE_OKVIS_CERES_DEMHEIGHTERROR_HPP_
