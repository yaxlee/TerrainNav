/**
 * DGVI - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich 
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENESE file for details
 */

/**
 * @file ErrorInterface.hpp
 * @brief Header file for the ErrorInterface class. A simple interface class that
          other error classes should inherit from.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_DGVI_CERES_ERRORINTERFACE_HPP_
#define INCLUDE_DGVI_CERES_ERRORINTERFACE_HPP_

#include <Eigen/Core>
#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include <dgvi/assert_macros.hpp>

/// \brief dgvi Main namespace of this package.
namespace dgvi {
/// \brief ceres Namespace for ceres-related functionality implemented in dgvi.
namespace ceres {

/// @brief Simple interface class the errors implemented here should inherit from.
class ErrorInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DGVI_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief Constructor
  ErrorInterface() = default;
  /// @brief Destructor (does nothing).
  virtual ~ErrorInterface() = default;

  /// @name Sizes
  /// @{

  /// @brief Get dimension of residuals.
  /// @return The residual dimension.
  virtual int residualDim() const = 0;

  /// @brief Get the number of parameter blocks this is connected to.
  /// @return The number of parameter blocks.
  virtual int parameterBlocks() const = 0;

  /**
   * @brief get the dimension of a parameter block this is connected to.
   * @param parameterBlockId The ID of the parameter block of interest.
   * @return Its dimension.
   */
  virtual int parameterBlockDim(int parameterBlockId) const = 0;

  /// @}
  // Error and Jacobian computation
  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  virtual bool EvaluateWithMinimalJacobians(
      double const* const * parameters, double* residuals, double** jacobians,
      double** jacobiansMinimal) const = 0;

  /// @brief Residual block type as string
  virtual std::string typeInfo() const = 0;
};

/// \brief Jacobian checker.
/// \param problem Ceres problem.
/// \param id Ceres residual pointer.
/// \param tolerance Tolerated deviation.
/// \return True if correct.
bool jacobiansCorrect(const ::ceres::Problem* problem, const ::ceres::ResidualBlockId id,
                      double tolerance = 1.0e-6);

}  // namespace ceres
}  // namespace dgvi

#endif /* INCLUDE_DGVI_CERES_ERRORINTERFACE_HPP_ */
