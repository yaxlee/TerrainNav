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
 * @file ViSensorBase.hpp
 * @brief Header file for the ViSensorBase class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_DGVI_VISENSORBASE_HPP_
#define INCLUDE_DGVI_VISENSORBASE_HPP_

#include <Eigen/Core>
#include <map>

#include <opencv2/core.hpp>

#include <dgvi/assert_macros.hpp>
#include <dgvi/Time.hpp>
#include <dgvi/FrameTypedefs.hpp>


/// \brief dgvi Main namespace of this package.
namespace dgvi {

/// @brief Generic sensor interface for different VI sensors to derive from.
class ViSensorBase {
public:
  /// \brief Callback for receiving images.
  typedef std::function<
        bool(const dgvi::Time &,
             const std::map<size_t, cv::Mat> &,
             const std::map<size_t, cv::Mat> &)> ImageCallback;

  /// \brief Callback for receiving IMU measurements.
  typedef std::function<
        bool(const dgvi::Time &,
             const Eigen::Vector3d &, const Eigen::Vector3d & )> ImuCallback;
  typedef std::function<
        bool(const dgvi::Time &,
             const Eigen::Vector3d &, const Eigen::Vector3d &)> GpsCallback;
  typedef std::function<
          bool(const dgvi::Time &,
               const double &, const double &, const double &,
               const double &, const double &)> GeodeticGpsCallback;
  
  /// \brief Callback for receiving depth measurements.
  typedef std::function<
          bool(std::map<size_t, std::vector<dgvi::CameraMeasurement>>&)> DepthImageCallback;

  /// \brief Default destructor.
  virtual ~ViSensorBase() = default;

  /// @brief Set the images callback: can register multiple, will be executed in the order added.
  /// @param imagesCallback The images callback to register.
  virtual void setImagesCallback(const ImageCallback& imagesCallback) final {
    imagesCallbacks_.push_back(imagesCallback);
  }

  /// @brief Set IMU callback: can register multiple, will be executed in the order added.
  /// @param imuCallback The IMU callback to register.
  virtual void setImuCallback(const ImuCallback& imuCallback) final {
    imuCallbacks_.push_back(imuCallback);
  }

  /// @brief Set the GPS callback
  /// @param gpsCallback The GPS callback to register.
  virtual void setGpsCallback(const GpsCallback& gpsCallback) final {
    gpsCallback_ = gpsCallback;
  }

  /// @brief Set the Geodetic GPS callback
  /// @param geodeticGpsCallback The GPS callback to register.
  virtual void setGeodeticGpsCallback(const GeodeticGpsCallback & geodeticGpsCallback) final {
    geodeticGpsCallback_ = geodeticGpsCallback;
  }

  /// @brief Set the images callback
  /// @param imagesCallback The images callback to register.
  virtual void setDepthImageCallback(const DepthImageCallback & depthImageCallback) final {
    depthCallback_ = depthImageCallback;
  }


  /// @brief Starts streaming.
  /// @return True, if successful
  virtual bool startStreaming() = 0;

  /// @brief Stops streaming.
  /// @return True, if successful
  virtual bool stopStreaming() = 0;

  /// @brief Check if currently streaming.
  /// @return True, if streaming.
  virtual bool isStreaming() = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  AlignedVector<ImageCallback> imagesCallbacks_; ///< The registered images callbacks.
  AlignedVector<ImuCallback> imuCallbacks_; ///< The registered IMU callbacks.
  GpsCallback gpsCallback_; ///< The registered GPS callback. // ToDo: rename to cartesian
  GeodeticGpsCallback geodeticGpsCallback_; ///< The registered (geodetic) GPS callback.
  DepthImageCallback depthCallback_;  ///< The registered callback for the depth image.
};

/// @brief Reader class acting like a VI sensor.
/// @warning Make sure to use this in combination with synchronous
/// processing, as there is no throttling of the reading process.
class DatasetReaderBase : public ViSensorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DGVI_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// To account for GNSS-to-UTC offset (Leap seconds)
  const uint64 GNSS_LEAP_NANOSECONDS = 0;

  /// @brief (Re-)setting the dataset path.
  /// @param path The absolute or relative path to the dataset.
  /// @return True, if the dateset folder structure could be created.
  virtual bool setDatasetPath(const std::string & path) = 0;

  /// @brief Setting skip duration in the beginning.
  /// deltaT Duration [s] to skip in the beginning.
  virtual bool setStartingDelay(const dgvi::Duration & deltaT) = 0;

  /// @brief Get the completion fraction read already.
  /// @return Fraction read already.
  virtual double completion() const = 0;

};

} // dgvi

#endif
