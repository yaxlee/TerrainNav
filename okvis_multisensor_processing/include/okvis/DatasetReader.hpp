/**
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense 
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
 * @file DatasetReader.hpp
 * @brief Header file for the DatasetReader class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_DATASETREADER_HPP_
#define INCLUDE_OKVIS_DATASETREADER_HPP_

#include <fstream>
#include <atomic>
#include <thread>
#include <array>
#include <vector>

#include <glog/logging.h>

#include <okvis/assert_macros.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/ViSensorBase.hpp>
#include <gdal_priv.h>
#include <cpl_conv.h>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geoid.hpp>
#include <ogr_spatialref.h>
#include <memory>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief Reader class acting like a VI sensor.
/// @warning Make sure to use this in combination with synchronous
/// processing, as there is no throttling of the reading process.
class DatasetReader : public DatasetReaderBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief Disallow default construction.
  DatasetReader() = delete;

  /// @brief Construct pointing to dataset.
  /// @param path The absolute or relative path to the dataset.
  /// @param numCameras The total number of cameras.
  /// @param syncCameras Camera group to force synchronisation.
  /// @param deltaT Duration [s] to skip in the beginning.
  DatasetReader(const std::string& path, size_t numCameras, const std::set<size_t> & syncCameras,
                const Duration & deltaT = Duration(0.0),
                const std::optional<GpsParameters>& gpsParameters = std::nullopt,
                const std::optional<DemParameters>& demParameters = std::nullopt,
                const std::string& demPath = "");
  DatasetReader(const std::string& path, size_t numCameras, const std::set<size_t> & syncCameras,
                const Duration & deltaT,
                const std::optional<GpsParameters>& gpsParameters,
                const std::optional<DemParameters>& demParameters,
                const std::vector<std::string>& demPaths);

  /// @brief Destructor: stops streaming.
  virtual ~DatasetReader();

  /// @brief (Re-)setting the dataset path.
  /// @param path The absolute or relative path to the dataset.
  /// @return True, if the dateset folder structure could be created.
  virtual bool setDatasetPath(const std::string & path) final;

  /// @brief Setting skip duration in the beginning.
  /// deltaT Duration [s] to skip in the beginning.
  virtual bool setStartingDelay(const okvis::Duration & deltaT) final;

  /// @brief Starts reading the dataset.
  /// @return True, if successful
  virtual bool startStreaming() final;

  /// @brief Stops reading the dataset.
  /// @return True, if successful
  virtual bool stopStreaming() final;

  /// @brief Check if currently reading the dataset.
  /// @return True, if reading.
  virtual bool isStreaming() final;

  /// @brief Get the completion fraction read already.
  /// @return Fraction read already.
  virtual double completion() const final;

private:

  /// @brief Read the camera csv file, that maps image filenames to timestamps.
  int readCameraImageCsv(std::string folderString, size_t camIdx,
                         std::vector < std::pair<std::string, std::string> >& imageNames) const;

  /// @brief Main processing loop.
  void processing();
  std::thread processingThread_; ///< Thread running processing loop.

  std::string path_; ///< Dataset path.

  std::atomic_bool streaming_; ///< Are we streaming?
  std::atomic_int counter_; ///< Number of images read yet.
  size_t numImages_ = 0; ///< Number of images to read.

  std::ifstream imuFile_; ///< Imu csv file.

  /// @brief The times and names of all images (gray & rgb) by camera.
  std::map <size_t, std::vector < std::pair<std::string, std::string> > > allImageNames_;

  /// @brief The times and names of all depth images.
  std::map <size_t, std::vector < std::pair<std::string, std::string> > > allDepthImageNames_;

  size_t numCameras_ = 0; ///< Total number of cameras.
  std::set<size_t> syncCameras_; ///< Camera group to force synchronisation.

  Duration deltaT_ = okvis::Duration(0.0); ///< Skip duration [s].

  // gps stuff...
  std::atomic_bool gpsFlag_; ///< Flag specifying if gps data is available / used.
  std::string gpsDataType_; ///< GPS data type: "cartesian" | "geodetic" | "geodetic-leica"
  okvis::Time t_gps_; ///< Timestamp of the last gps signal received
  std::ifstream gpsFile_; ///< Gps csv file.

  double maxHErr_ = std::numeric_limits<double>::max(); ///< Max horizontal error threshold [m]
  double maxVErr_ = std::numeric_limits<double>::max(); ///< Max vertical error threshold [m]
  double maxGpsSpeed_ = std::numeric_limits<double>::max(); ///< Max horizontal GPS speed threshold [m/s]
  int minFixStatus_ = 0; ///< Minimum fix_status (0=disabled, 1=reject status=0)
  uint64_t lastGpsNs_ = 0; ///< Timestamp of last accepted GPS measurement [ns], for burst-duplicate filtering
  bool lastCartesianGpsValid_ = false; ///< True once a cartesian GPS point has passed reader-side filters.
  Eigen::Vector3d lastCartesianGpsPosition_ = Eigen::Vector3d::Zero(); ///< Last accepted cartesian GPS position [m].
  bool lastGeodeticGpsValid_ = false; ///< True once a geodetic GPS point has passed reader-side filters.
  Eigen::Vector3d lastGeodeticGpsPosition_ = Eigen::Vector3d::Zero(); ///< Last accepted geodetic GPS point [lat deg, lon deg, alt m].
  std::unique_ptr<GeographicLib::Geoid> geoid_; ///< Geoid model for ellipsoidal->orthometric correction

  // DEM-GPS integration
  bool useDemHeightForGps_ = false; ///< If true: replace GPS alt with DEM alt in reader; if false: blend in backend
  double demSigmaH_ = 2.0;          ///< Vertical uncertainty [m] used when DEM replaces GPS altitude

  struct DemDataset {
    std::string path;
    GDALDataset* dataset = nullptr;
    std::array<double, 6> geoTransform = {};
    bool hasNoData = false;
    double noDataValue = 0.0;
    OGRCoordinateTransformation* coordinateTransformation = nullptr;
  };

  bool loadDemDataset(const std::string& demPath);
  double getDemHeight(const DemDataset& demDataset, double lat, double lon) const;

  std::vector<DemDataset> demDatasets_; ///< Ordered DEM datasets used for fallback lookup.

public:
  /// @brief Query terrain height from DEM at a given geodetic coordinate.
  /// @param lat Latitude [degrees].
  /// @param lon Longitude [degrees].
  /// @return Height [m], or -1.0 if out of bounds / not loaded.
  double getDemHeight(double lat, double lon);
};

}

#endif // INCLUDE_OKVIS_DATASETREADER_HPP_
