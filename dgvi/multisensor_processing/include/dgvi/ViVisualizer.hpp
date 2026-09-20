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
 * @file ViVisualizer.hpp
 * @brief Header file for the VioVisualizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_DGVI_VIOVISUALIZER_HPP_
#define INCLUDE_DGVI_VIOVISUALIZER_HPP_

#include <dgvi/assert_macros.hpp>

#include <dgvi/Parameters.hpp>
#include <dgvi/MultiFrame.hpp>
#include <dgvi/FrameTypedefs.hpp>

/// \brief dgvi Main namespace of this package.
namespace dgvi {

/**
 * @brief This class is responsible to visualize the matching results
 */
class ViVisualizer {
 public:

  /// @brief This struct contains the relevant data for visualizing
  struct VisualizationData {
    /// \brief Describe tracking quality (qualitatively).
    enum class TrackingQuality {
      Good,
      Marginal,
      Lost
    };

    /// \brief Pointer to VisualizationData.
    typedef std::shared_ptr<VisualizationData> Ptr;

    dgvi::ObservationVector observations;    ///< Vector containing all the keypoint observations.
    std::shared_ptr<dgvi::MultiFrame> currentFrames; ///< Current multiframe.
    dgvi::kinematics::Transformation T_WS;  ///< Pose of the current frame
    AlignedVector<kinematics::Transformation> T_SCi;  ///< Camera extrinsics (may change).
    bool isKeyframe = false; ///< Is it a keyframe?
    bool recognisedPlace = false; ///< is it a place recognition frame?
    TrackingQuality trackingQuality = TrackingQuality::Good; ///< The tracking quality.
  };

  DGVI_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Constructor.
   * @param parameters Parameters and settings.
   */
  ViVisualizer(dgvi::ViParameters& parameters);

  /// \brief Destructor.
  virtual ~ViVisualizer();

  /**
   * @brief Initialise parameters. Called in constructor.
   * @param parameters Parameters and settings.
   */
  void init(dgvi::ViParameters& parameters);

  /**
   * @brief Circles all keypoints in the current frame, links the matching ones to
   *        the current keyframe and returns the result.
   * @param data Visualization data.
   * @param image_number Index of the frame to display.
   * @return OpenCV matrix with the resulting image.
   */
  cv::Mat drawMatches(VisualizationData::Ptr& data, size_t image_number);
  
 private:
  /**
   * @brief Circles all keypoints in the current frame and returns the result.
   * @param data Visualization data.
   * @param cameraIndex Index of the frame to display.
   * @return OpenCV matrix with the resulting image.
   */
  cv::Mat drawKeypoints(VisualizationData::Ptr& data, size_t cameraIndex);

  /// Parameters and settings.
  dgvi::ViParameters parameters_;
};

} /* namespace dgvi */

#endif /* INCLUDE_DGVI_VIOVISUALIZER_HPP_ */
