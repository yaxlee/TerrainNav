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
 * @file TestNCameraSystem.cpp
 * @brief Runs NCameraSystem tests.
 * @author Stefan Leutenegger
 */

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <brisk/brisk.h>
#pragma GCC diagnostic pop

#include "dgvi/cameras/PinholeCamera.hpp"
#include "dgvi/cameras/NoDistortion.hpp"
#include "dgvi/cameras/RadialTangentialDistortion.hpp"
#include "dgvi/cameras/EquidistantDistortion.hpp"
#include "dgvi/cameras/NCameraSystem.hpp"

TEST(NCameraSystem, functions)
{

  // instantiate all possible versions of test cameras
  std::vector<std::shared_ptr<const dgvi::cameras::CameraBase> > cameras;
  std::vector<dgvi::cameras::NCameraSystem::DistortionType> distortions;
  cameras.push_back(
      dgvi::cameras::PinholeCamera<dgvi::cameras::NoDistortion>::createTestObject());
  distortions.push_back(dgvi::cameras::NCameraSystem::NoDistortion);
  cameras.push_back(
      dgvi::cameras::PinholeCamera<dgvi::cameras::RadialTangentialDistortion>
        ::createTestObject());
  distortions.push_back(dgvi::cameras::NCameraSystem::RadialTangential);
  cameras.push_back(
      dgvi::cameras::PinholeCamera<dgvi::cameras::EquidistantDistortion>::createTestObject());
  distortions.push_back(dgvi::cameras::NCameraSystem::Equidistant);

  // the mounting transformations. The third one is opposite direction
  std::vector<std::shared_ptr<const dgvi::kinematics::Transformation>> T_SC;
  T_SC.push_back(
      std::shared_ptr<dgvi::kinematics::Transformation>(
          new dgvi::kinematics::Transformation(Eigen::Vector3d(0.1, 0.1, 0.1),
                                                Eigen::Quaterniond(1, 0, 0, 0))));
  T_SC.push_back(
      std::shared_ptr<dgvi::kinematics::Transformation>(
          new dgvi::kinematics::Transformation(
              Eigen::Vector3d(0.1, -0.1, -0.1), Eigen::Quaterniond(1, 0, 0, 0))));
  T_SC.push_back(
      std::shared_ptr<dgvi::kinematics::Transformation>(
          new dgvi::kinematics::Transformation(
              Eigen::Vector3d(0.1, -0.1, -0.1), Eigen::Quaterniond(0, 0, 1, 0))));

  dgvi::cameras::NCameraSystem nCameraSystem;
  dgvi::cameras::NCameraSystem::CameraType cameraType;
  cameraType.isColour = false;
  for(size_t i=0; i<3; ++i) {
      nCameraSystem.addCamera(T_SC.at(i), cameras.at(i), distortions.at(i),
                              true, cameraType); // comp. overlaps
  }

  // verify self overlaps
  DGVI_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(0, 0), "No self overlap?")
  DGVI_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(1, 1), "No self overlap?")
  DGVI_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(2, 2), "No self overlap?")

  // verify 0 and 1 overlap
  DGVI_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(0, 1), "No overlap?")
  DGVI_ASSERT_TRUE(std::runtime_error, nCameraSystem.hasOverlap(1, 0), "No overlap?")

  // verify 1 and 2 do not overlap
  DGVI_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(1, 2), "Overlap?")
  DGVI_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(2, 1), "Overlap?")

  // verify 0 and 2 do not overlap
  DGVI_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(0, 2), "Overlap?")
  DGVI_ASSERT_TRUE(std::runtime_error, !nCameraSystem.hasOverlap(2, 0), "Overlap?")

}

