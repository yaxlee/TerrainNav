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
 * @file SpeedAndBiasParameterBlock.cpp
 * @brief Source file for the SpeedAndBiasParameterBlock class.
 * @author Stefan Leutenegger
 */

#include <dgvi/ceres/SpeedAndBiasParameterBlock.hpp>

/// \brief dgvi Main namespace of this package.
namespace dgvi {
/// \brief ceres Namespace for ceres-related functionality implemented in dgvi.
namespace ceres {

// Default constructor (assumes not fixed).
SpeedAndBiasParameterBlock::SpeedAndBiasParameterBlock()
    : base_t::ParameterBlockSized() {
  setFixed(false);
}

// Constructor with estimate and time.
SpeedAndBiasParameterBlock::SpeedAndBiasParameterBlock(
    const SpeedAndBias& speedAndBias, uint64_t id,
    const dgvi::Time& timestamp) {
  setEstimate(speedAndBias);
  setId(id);
  setTimestamp(timestamp);
  setFixed(false);
}

}  // namespace ceres
}  // namespace dgvi
