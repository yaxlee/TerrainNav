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
 * @file ViFrontendInterface.hpp
 * @brief Header file for the VioFrontendInterface class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_DGVI_VIOFRONTENDINTERFACE_HPP_
#define INCLUDE_DGVI_VIOFRONTENDINTERFACE_HPP_

#include <vector>
#include <memory>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <dgvi/kinematics/Transformation.hpp>

#include <dgvi/Measurements.hpp>
#include <dgvi/Parameters.hpp>
#include <dgvi/Time.hpp>
#include <dgvi/FrameTypedefs.hpp>
#include <dgvi/MultiFrame.hpp>

/// \brief dgvi Main namespace of this package.
namespace dgvi {

class ViSlamBackend;

/// \brief The VI-SLAM estimator class.
using Estimator = ViSlamBackend;

/**
 * @brief The VioFrontendInterface class is an interface for frontends.
 */
class ViFrontendInterface {
 public:
  /// \brief Default constructor.
  ViFrontendInterface() = default;

  /// \brief Default destructor.
  virtual ~ViFrontendInterface() = default;

  /// @name
  /// In the derived class, the following methods (and nothing else) have to be implemented:
  ///@{
  /**
   * @brief Detection and descriptor extraction on a per image basis.
   * @param cameraIndex   Index of camera to do detection and description.
   * @param frameOut      Multiframe containing the frames.
   *                      Resulting keypoints and descriptors are saved in here.
   * @param T_WC          Pose of camera with index cameraIndex at image capture time.
   * @param[in] keypoints If the keypoints are already available from a different source, provide
   *                      them here in order to skip detection.
   * @return True if successful.
   */
  virtual bool detectAndDescribe(
      size_t cameraIndex, std::shared_ptr<dgvi::MultiFrame> frameOut,
      const dgvi::kinematics::Transformation& T_WC,
      const std::vector<cv::KeyPoint> * keypoints) = 0;

  /**
   * @brief Matching as well as initialization of landmarks and state.
   * @param estimator       Estimator.
   * @param params          Configuration parameters.
   * @param framesInOut     Multiframe including the descriptors of all the keypoints.
   * @param kfPrior         A prior to trigger new keyframe from other criteria (e.g. LiDAR overlap)
   * @param[out] asKeyframe Should the frame be a keyframe?
   * @return True if successful.
   */
  virtual bool dataAssociationAndInitialization(
      Estimator& estimator,
      const dgvi::ViParameters & params,
      std::shared_ptr<dgvi::MultiFrame> framesInOut, bool kfPrior, bool* asKeyframe) = 0;

  /**
   * @brief Propagates pose, speeds and biases with given IMU measurements.
   * @see dgvi::ceres::ImuError::propagation()
   * @param[in] imuMeasurements All the IMU measurements.
   * @param[in] imuParams The parameters to be used.
   * @param[inout] T_WS_propagated Start pose.
   * @param[inout] speedAndBiases Start speed and biases.
   * @param[in] t_start Start time.
   * @param[in] t_end End time.
   * @param[out] covariance Covariance for GIVEN start states.
   * @param[out] jacobian Jacobian w.r.t. start states.
   * @return True on success.
   */
  virtual bool propagation(const dgvi::ImuMeasurementDeque & imuMeasurements,
                           const dgvi::ImuParameters & imuParams,
                           dgvi::kinematics::Transformation& T_WS_propagated,
                           dgvi::SpeedAndBias & speedAndBiases,
                           const dgvi::Time& t_start, const dgvi::Time& t_end,
                           Eigen::Matrix<double, 15, 15>* covariance,
                           Eigen::Matrix<double, 15, 15>* jacobian) const = 0;

  ///@}
};

}

#endif /* INCLUDE_DGVI_VIOFRONTENDINTERFACE_HPP_ */
