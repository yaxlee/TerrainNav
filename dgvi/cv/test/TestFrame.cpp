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
 * @file TestFrame.cpp
 * @brief Runs Frame tests.
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
#include "dgvi/Frame.hpp"
#include "dgvi/internal/Network.hpp"

TEST(Frame, functions)
{

  // instantiate all possible versions of test cameras
  std::vector<std::shared_ptr<dgvi::cameras::CameraBase> > cameras;
  cameras.push_back(
      dgvi::cameras::PinholeCamera<dgvi::cameras::NoDistortion>::createTestObject());
  cameras.push_back(
      dgvi::cameras::PinholeCamera<dgvi::cameras::RadialTangentialDistortion>::createTestObject());
  cameras.push_back(
      dgvi::cameras::PinholeCamera<dgvi::cameras::EquidistantDistortion>::createTestObject());

  for (size_t c = 0; c < cameras.size(); ++c) {

#ifdef __ARM_NEON__
   std::shared_ptr<cv::FeatureDetector> detector(
        new brisk::BriskFeatureDetector(34, 2));
#else
   std::shared_ptr<cv::FeatureDetector> detector(
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            34, 2, 800, 450));
#endif
 
    std::shared_ptr<cv::DescriptorExtractor> extractor(
        new cv::BriskDescriptorExtractor(true, false));

    // create a stupid random image
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> eigenImage(752,480);
    eigenImage.setRandom();
    cv::Mat image(480, 752, CV_8UC1, eigenImage.data());
    std::shared_ptr<dgvi::Network> network(new dgvi::Network());
    dgvi::Frame frame(image, cameras.at(c), detector, extractor, network);

    // run
    frame.detect();
    frame.describe();
  }
}

