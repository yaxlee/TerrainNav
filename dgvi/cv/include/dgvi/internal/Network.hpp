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
 * @file dgvi/internal/Network.hpp
 * @brief Wraps Torch.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_DGVI_INTERNAL_NETWORK_HPP_
#define INCLUDE_DGVI_INTERNAL_NETWORK_HPP_

#ifdef DGVI_USE_NN
#include <torch/torch.h>
#include <torch/script.h>
#endif

namespace dgvi {

/// \brief Importing Torch Network (or treat as dummy if not built with Torch).
#ifdef DGVI_USE_NN
class Network : public ::torch::jit::Module {
 public:
  Network() = default;
  Network(torch::jit::Module&& module) : ::torch::jit::Module(module) {};
};
#else
class Network {
  // dummy
};
#endif

}

#endif /* INCLUDE_DGVI_INTERNAL_NETWORK_HPP_ */
