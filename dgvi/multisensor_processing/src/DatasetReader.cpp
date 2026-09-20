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
 * @file DatasetReader.cpp
 * @brief Source file for the DatasetReader class.
 * @author Stefan Leutenegger
 */
 
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dgvi/ViInterface.hpp>
#include <dgvi/DatasetReader.hpp>
#include <gdal_priv.h>
#include <cpl_conv.h>


namespace dgvi {
namespace {
std::vector<std::string> findDemRasters(const std::string& datasetPath) {
  const boost::filesystem::path directory = boost::filesystem::path(datasetPath) / "dem0";
  std::vector<std::string> paths;
  if (!boost::filesystem::is_directory(directory)) {
    return paths;
  }
  for (const auto& entry : boost::filesystem::directory_iterator(directory)) {
    if (!boost::filesystem::is_regular_file(entry.path())) {
      continue;
    }
    std::string extension = entry.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (extension == ".tif" || extension == ".tiff" || extension == ".vrt") {
      paths.push_back(entry.path().string());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

double horizontalGeodeticDistanceMeters(double lat0, double lon0,
                                        double lat1, double lon1) {
  static constexpr double kEarthRadiusMeters = 6378137.0;
  static constexpr double kDegToRad = 0.017453292519943295;
  const double phi0 = lat0 * kDegToRad;
  const double phi1 = lat1 * kDegToRad;
  const double dphi = (lat1 - lat0) * kDegToRad;
  const double dlambda = (lon1 - lon0) * kDegToRad;
  const double sinDphi = std::sin(0.5 * dphi);
  const double sinDlambda = std::sin(0.5 * dlambda);
  const double a = sinDphi * sinDphi +
                   std::cos(phi0) * std::cos(phi1) * sinDlambda * sinDlambda;
  return 2.0 * kEarthRadiusMeters * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
}
}

DatasetReader::DatasetReader(
  const std::string& path, size_t numCameras, const std::set<size_t> &syncCameras,
  const Duration & deltaT, const std::optional<GpsParameters>& gpsParameters,
  const std::optional<DemParameters>& demParameters, const std::string& demPath) :
  DatasetReader(path, numCameras, syncCameras, deltaT, gpsParameters, demParameters,
                demPath.empty() ? std::vector<std::string>() : std::vector<std::string>{demPath}) {
}

DatasetReader::DatasetReader(
  const std::string& path, size_t numCameras, const std::set<size_t> &syncCameras,
  const Duration & deltaT, const std::optional<GpsParameters>& gpsParameters,
  const std::optional<DemParameters>& demParameters, const std::vector<std::string>& demPaths) :
  numCameras_(numCameras), syncCameras_(syncCameras), deltaT_(deltaT) {
  setDatasetPath(path);
  if (demParameters && demParameters->use) {
    DGVI_ASSERT_TRUE(Exception, gpsParameters && gpsParameters->type == "geodetic",
                     "DEM requires geodetic GNSS input")
    useDemHeightForGps_ = (*demParameters).useDemHeightForGps;
    demSigmaH_ = (*demParameters).sigma_h;
  }
 
  if(gpsParameters) {
    gpsFlag_ = true;
    gpsDataType_ = (*gpsParameters).type;
    DGVI_ASSERT_TRUE(Exception, gpsDataType_=="cartesian" || gpsDataType_=="geodetic" || gpsDataType_=="geodetic-leica",
                      "Unknown GPS data type specified")
    maxHErr_ = (*gpsParameters).maxHErr;
    maxVErr_ = (*gpsParameters).maxVErr;
    maxGpsSpeed_ = (*gpsParameters).gpsMaxSpeed;
    minFixStatus_ = (*gpsParameters).minFixStatus;
    const std::string& geoidModel = (*gpsParameters).geoidModel;
    if (!geoidModel.empty()) {
      try {
        geoid_ = std::make_unique<GeographicLib::Geoid>(geoidModel);
        LOG(INFO) << "[GPS] Geoid model loaded: " << geoidModel;
      } catch (const std::exception& e) {
        LOG(WARNING) << "[GPS] Failed to load geoid model '" << geoidModel << "': " << e.what()
                     << " -- geoid undulation correction disabled.";
      }
    }

    if (demParameters && demParameters->use) {
      const std::vector<std::string> paths = demPaths.empty() ? findDemRasters(path_) : demPaths;
      if (paths.empty()) {
        DGVI_THROW(Exception, "DEM is enabled, but no .tif/.tiff/.vrt rasters were found in "
                   << (boost::filesystem::path(path_) / "dem0").string());
      }
      GDALAllRegister();
      for (const std::string& demPath : paths) {
        loadDemDataset(demPath);
      }
      if (demDatasets_.empty()) {
        DGVI_THROW(Exception, "No DEM datasets could be loaded from " << paths.size() << " path(s).");
      } else {
        LOG(INFO) << "Loaded " << demDatasets_.size() << " DEM dataset(s); lookup uses input order.";
      }
    }
  }
  else {
    gpsFlag_ = false;
  }
  streaming_ = false;
  counter_ = 0;
  t_gps_ = dgvi::Time(0.0);
}

DatasetReader::~DatasetReader() {
  stopStreaming();
  for (DemDataset& demDataset : demDatasets_) {
    if (demDataset.coordinateTransformation) {
      OGRCoordinateTransformation::DestroyCT(demDataset.coordinateTransformation);
      demDataset.coordinateTransformation = nullptr;
      LOG(INFO) << "GDAL Coordinate Transformation object destroyed for " << demDataset.path;
    }

    if (demDataset.dataset) {
      GDALClose(demDataset.dataset);
      demDataset.dataset = nullptr;
      LOG(INFO) << "GDAL Dataset closed: " << demDataset.path;
    }
  }
}

bool DatasetReader::loadDemDataset(const std::string& demPath) {
  DemDataset demDataset;
  demDataset.path = demPath;
  demDataset.dataset = static_cast<GDALDataset*>(GDALOpen(demPath.c_str(), GA_ReadOnly));
  if (demDataset.dataset == nullptr) {
    LOG(ERROR) << "Failed to load DEM at " << demPath;
    return false;
  }

  if (demDataset.dataset->GetGeoTransform(demDataset.geoTransform.data()) != CE_None) {
    LOG(ERROR) << "Failed to read DEM geo transform at " << demPath;
    GDALClose(demDataset.dataset);
    return false;
  }

  OGRSpatialReference oSourceSRS, oTargetSRS;
  oSourceSRS.importFromEPSG(4326); // WGS84

  const char* pszProjection = demDataset.dataset->GetProjectionRef();
  if (pszProjection == nullptr || oTargetSRS.importFromWkt(pszProjection) != OGRERR_NONE) {
    LOG(ERROR) << "Failed to read DEM projection at " << demPath;
    GDALClose(demDataset.dataset);
    return false;
  }

  oSourceSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  oTargetSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

  demDataset.coordinateTransformation = OGRCreateCoordinateTransformation(&oSourceSRS, &oTargetSRS);
  if (demDataset.coordinateTransformation == nullptr) {
    LOG(ERROR) << "Failed to create DEM coordinate transformation at " << demPath;
    GDALClose(demDataset.dataset);
    return false;
  }

  GDALRasterBand* band = demDataset.dataset->GetRasterBand(1);
  if (band == nullptr) {
    LOG(ERROR) << "Failed to read DEM raster band at " << demPath;
    OGRCoordinateTransformation::DestroyCT(demDataset.coordinateTransformation);
    GDALClose(demDataset.dataset);
    return false;
  }

  int hasNoData = 0;
  demDataset.noDataValue = band->GetNoDataValue(&hasNoData);
  demDataset.hasNoData = hasNoData != 0;

  LOG(INFO) << "DEM loaded: " << demPath << ". Target CRS: " << oTargetSRS.GetName();
  demDatasets_.push_back(demDataset);
  return true;
}

double DatasetReader::getDemHeight(const DemDataset& demDataset, double lat, double lon) const {
  if (!demDataset.dataset || !demDataset.coordinateTransformation) return -std::numeric_limits<double>::infinity();

  double x = lon;
  double y = lat;
  if (!demDataset.coordinateTransformation->Transform(1, &x, &y)) {
    return -std::numeric_limits<double>::infinity();
  }

  const auto& geoTransform = demDataset.geoTransform;
  double d = geoTransform[1] * geoTransform[5] - geoTransform[2] * geoTransform[4];
  int pixel = static_cast<int>((geoTransform[5] * (x - geoTransform[0]) -
                                geoTransform[2] * (y - geoTransform[3])) / d);
  int line = static_cast<int>((geoTransform[1] * (y - geoTransform[3]) -
                               geoTransform[4] * (x - geoTransform[0])) / d);

  if (pixel < 0 || pixel >= demDataset.dataset->GetRasterXSize() ||
      line < 0 || line >= demDataset.dataset->GetRasterYSize()) {
    return -std::numeric_limits<double>::infinity();
  }

  float val;
  CPLErr err = demDataset.dataset->GetRasterBand(1)->RasterIO(
      GF_Read, pixel, line, 1, 1, &val, 1, 1, GDT_Float32, 0, 0);

  if (err != CE_None) {
    LOG(WARNING) << "Failed to read DEM value at " << demDataset.path
                 << " pixel " << pixel << ", line " << line;
    return -std::numeric_limits<double>::infinity();
  }

  if (demDataset.hasNoData &&
      (static_cast<double>(val) == demDataset.noDataValue ||
       (std::isnan(static_cast<double>(val)) && std::isnan(demDataset.noDataValue)))) {
    return -std::numeric_limits<double>::infinity();
  }

  return static_cast<double>(val);
}

// DEM Reader
double DatasetReader::getDemHeight(double lat, double lon) {
  for (const DemDataset& demDataset : demDatasets_) {
    const double height = getDemHeight(demDataset, lat, lon);
    if (std::isfinite(height) && height >= -100.0) {
      return height;
    }
  }

  return -std::numeric_limits<double>::infinity();
}

bool DatasetReader::setDatasetPath(const std::string & path) {
  boost::filesystem::path datasetPath(path);
  if (boost::filesystem::is_directory(datasetPath / "mav0")) {
    datasetPath /= "mav0";
  }
  path_ = datasetPath.string();
  return true;
}

bool DatasetReader::setStartingDelay(const Duration &deltaT)
{
  if(streaming_) {
    LOG(WARNING)<< "starting delay ignored, because streaming already started";
    return false;
  }
  deltaT_ = deltaT;
  return true;
}

bool DatasetReader::isStreaming()
{
  return streaming_;
}

double DatasetReader::completion() const {
  if(streaming_) {
    return double(counter_)/double(numImages_);
  }
  return 0.0;
}

bool DatasetReader::startStreaming() {
  DGVI_ASSERT_TRUE(Exception, !imagesCallbacks_.empty(), "no add image callback registered")
  DGVI_ASSERT_TRUE(Exception, !imuCallbacks_.empty(), "no add IMU callback registered")
  if(gpsFlag_)
          DGVI_ASSERT_TRUE(Exception, geodeticGpsCallback_ || gpsCallback_, "no add GPS callback registered")

  // open the IMU file
  std::string line;
  imuFile_.open(path_ + "/imu0/data.csv");
  DGVI_ASSERT_TRUE(Exception, imuFile_.good(), "no imu file found at " << path_+"/imu0/data.csv");
  int number_of_lines = 0;
  while (std::getline(imuFile_, line))
    ++number_of_lines;
  LOG(INFO)<< "No. IMU measurements: " << number_of_lines-1;
  if (number_of_lines - 1 <= 0) {
    LOG(ERROR)<< "no imu messages present in " << path_+"/imu0/data.csv";
    return -1;
  }
  // set reading position to second line
  imuFile_.clear();
  imuFile_.seekg(0, std::ios::beg);
  std::getline(imuFile_, line);

  if(gpsFlag_)
  {
      // open the GPS file
      std::string gline;
      if(gpsDataType_ =="cartesian")
        gpsFile_.open(path_ + "/gps0/data.csv");
      else if(gpsDataType_=="geodetic")
        gpsFile_.open(path_ + "/gps0/data_raw.csv");
      else if(gpsDataType_ == "geodetic-leica")
        gpsFile_.open(path_+"gnss.csv");
      DGVI_ASSERT_TRUE(Exception, gpsFile_.good(), "no gps file found at " << path_+"/gps0/");
      int gnumber_of_lines = 0;
      while (std::getline(gpsFile_, gline))
        ++gnumber_of_lines;
      LOG(INFO)<< "No. GPS measurements: " << gnumber_of_lines-1;
      if (gnumber_of_lines - 1 <= 0) {
        LOG(ERROR)<< "no gps messages present in " << path_+"/gps0/data.csv";
        return -1;
      }
      // set reading position to second line
      gpsFile_.clear();
      gpsFile_.seekg(0, std::ios::beg);
      std::getline(gpsFile_, gline);

  }

  // now open camera files
  std::vector<dgvi::Time> times;
  dgvi::Time latest(0);
  for(size_t i=0; i<numCameras_; ++i) {
    std::vector < std::pair<std::string, std::string> > imageNames;
    // first try gray
    int num_camera_images = readCameraImageCsv("cam", i, imageNames);
    if (num_camera_images <= 0) {
      // try also RGB
      num_camera_images = readCameraImageCsv("rgb", i, imageNames);
      DGVI_ASSERT_TRUE(Exception, num_camera_images>0, "no images found for camera " << i)
      LOG(INFO)<< "No. cam " << i << " RGB images: " << num_camera_images;
    } else {
      LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;
    }
    if(i==0) {
      numImages_ = num_camera_images;
    }
    allImageNames_[i] = imageNames;

    // now also see if there might be depth images
    std::vector < std::pair<std::string, std::string> > depthImageNames;
    int num_depth_camera_images = readCameraImageCsv("depth", i, depthImageNames);
    if(num_depth_camera_images>0) {
      allDepthImageNames_[i] = depthImageNames;
      LOG(INFO)<< "No. cam " << i << " depth images: " << num_depth_camera_images;
    }
  }

  counter_ = 0;
  streaming_ = true;
  processingThread_ = std::thread(&DatasetReader::processing, this);

  return true;
}

int DatasetReader::readCameraImageCsv(std::string folderString, size_t camIdx,
    std::vector < std::pair<std::string, std::string> >& imageNames) const
{
  const std::string filename = path_ + "/" + folderString + std::to_string(camIdx) + "/data.csv";
  std::ifstream camDataFile(filename);
  std::string line;
  if(!camDataFile.good()) {
    return -1;
  }

  int num_camera_images = 0;
  std::getline(camDataFile, line);
  while (std::getline(camDataFile, line)) {
    ++num_camera_images;
    std::stringstream stream(line);
    std::string s0, s1;
    if (!std::getline(stream, s0, ',')) {
      break;
    }
    if (!std::getline(stream, s1)) {
      break;
    }
    if(s1[0]==' ') {
      s1 = s1.substr(1,s1.size()); // handle stupid extra whitespace in some datasets...!
    }
    if(s1[s1.size()-1]=='\r') {
      s1 = s1.substr(0,s1.size()-1); // handle stupid Windows file endings...!
    }
    imageNames.push_back(
      std::make_pair(s0, path_ + "/" + folderString + std::to_string(camIdx) + "/data/" + s1));
  }
  return num_camera_images;
}

bool DatasetReader::stopStreaming() {
  // Stop the pipeline
  if(processingThread_.joinable()) {
    processingThread_.join();
    streaming_ = false;
  }
  return true;
}

/// \brief Helper struct for image iterators.
struct ImageIterators {
  /// \brief Camera iterators.
  std::vector<std::vector<std::pair<std::string, std::string>>::iterator> cam_iterators;
  /// \brief Depth camera iterators.
  std::map<size_t, std::vector<std::pair<std::string, std::string>>::iterator> depthCam_iterators;
  /// \brief Camera iterator ends.
  std::vector<std::vector<std::pair<std::string, std::string>>::iterator> cam_ends;
  /// \brief Depth camera iterator ends.
  std::map<size_t, std::vector<std::pair<std::string, std::string>>::iterator> depthCam_ends;
  const uint64_t tolNSec = 10000000; ///< Sync time tolerance in nano-seconds.

  /// \brief Arg min of the timestamps in current iterators.
  /// @param[out] timestamp The smallest timestamp in current iterators.
  /// @param[out] isDepth If it is a depth camera.
  /// \return The arg min (idx).
  int argMinTime(uint64_t& timestamp, bool& isDepth) {
    timestamp = std::numeric_limits<uint64_t>::max();
    int i_min = -1;
    for(size_t i = 0; i<cam_iterators.size(); ++i) {
      if(cam_iterators.at(i) == cam_ends.at(i)) continue;
      uint64_t timestamp_tmp = std::atol(cam_iterators.at(i)->first.c_str());
      if(timestamp_tmp < timestamp) {
        i_min = i;
        timestamp = timestamp_tmp;
        isDepth = false;
      }
      /// \todo Fixme: reading async depth only frames is broken with the following:
      /*if (depthCam_iterators.count(i)) {
        if (depthCam_iterators.at(i) != depthCam_ends.at(i)) {
          uint64_t timestamp_tmp = std::atol(depthCam_iterators.at(i)->first.c_str());
          if (timestamp_tmp < timestamp) {
            i_min = i;
            timestamp = timestamp_tmp;
            isDepth = true;
          }
        }
      }*/
    }
    return i_min;
  }

  /// \brief Arg min of the timestamps in current iterators.
  /// @param[in] syncGroup Set of camera indices to treat as synced.
  /// @param[out] timestamp The smallest timestamp in current iterators.
  /// @param[out] isDepth If it is a depth camera.
  /// \return The arg min (idx).
  int argMinTime(const std::set<size_t>& syncGroup, uint64_t& timestamp, bool& isDepth) {
    timestamp = std::numeric_limits<uint64_t>::max();
    size_t i_min = *syncGroup.begin();
    for(size_t i = 0; i<cam_iterators.size(); ++i) {
      if(!syncGroup.count(i)) continue;
      if(cam_iterators.at(i) == cam_ends.at(i)) return -1;
      uint64_t timestamp_tmp = std::atol(cam_iterators.at(i)->first.c_str());
      if(timestamp_tmp < timestamp) {
        i_min = i;
        timestamp = timestamp_tmp;
        isDepth = false;
      }
    }
    return i_min;
  }

  /// \brief Check if sync group is indeed synced.
  /// @param[in] syncGroup Set of camera indices to treat as synced.
  /// \return True if synced.
  bool isSynched(const std::set<size_t>& syncGroup) {
    uint64_t timestamp_min;
    bool isDepth_min;
    int i_min = argMinTime(syncGroup, timestamp_min, isDepth_min);
    if(i_min == -1) return false;
    for(size_t i : syncGroup) {
      if(cam_iterators.at(i) == cam_ends.at(i)) return false;
      uint64_t timestamp = std::atol(cam_iterators.at(i)->first.c_str());
      if(timestamp-timestamp_min > tolNSec) {
        return false;
      }
    }
    return true;
  }
};

void  DatasetReader::processing() {
  std::string line;
  dgvi::Time start(0.0);
  const size_t numCameras = allImageNames_.size();
  ImageIterators iterators;
  iterators.cam_iterators.resize(numCameras);
  iterators.cam_ends.resize(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    iterators.cam_iterators.at(i) = allImageNames_.at(i).begin();
    iterators.cam_ends.at(i) = allImageNames_.at(i).end();
    if(allDepthImageNames_.count(i)){
      iterators.depthCam_iterators[i] = allDepthImageNames_.at(i).begin();
      iterators.depthCam_ends[i] = allDepthImageNames_.at(i).end();
    }
  }

  while (streaming_) {

    // outer sync loop: add whatever has the smallest timestamp, unless sync needed
    int i_min = 0;
    bool needSync = false;
    uint64_t timestamp_min = 0;
    bool isDepthCam = false;
    do {
      i_min = iterators.argMinTime(timestamp_min, isDepthCam);
      if(i_min == -1) {
        streaming_ = false;
        return; // all processed, finished
      }
      if(syncCameras_.count(i_min)) {
        needSync = !iterators.isSynched(syncCameras_);
      }
      if(allDepthImageNames_.count(i_min)) {
        std::set<size_t> depthGroup;
        depthGroup.insert(i_min);
        needSync |= !iterators.isSynched(depthGroup);
      }
      if(needSync) {
        if(isDepthCam) {
          LOG(WARNING)
              << "depth image at t=" << timestamp_min << " without correspondence -- dropping";
          iterators.depthCam_iterators.at(i_min)++;
          if(iterators.depthCam_iterators.at(i_min) == allDepthImageNames_.at(i_min).end()) {
            streaming_ = false;
            return; // finished!
          }
        } else {
          LOG(WARNING)
              << "image at t=" << timestamp_min << " without correspondence -- dropping";
          iterators.cam_iterators.at(i_min)++;
          if(iterators.cam_iterators.at(i_min) == allImageNames_.at(i_min).end()) {
            streaming_ = false;
            return; // finished!
          }
        }
      }
    } while (needSync);

    // add (unless at the end)...
    std::map<size_t, cv::Mat> images;
    std::map<size_t, cv::Mat> depthImages;
    if(syncCameras_.count(i_min)) {
      if(iterators.cam_iterators.at(i_min) == allImageNames_.at(i_min).end()) {
        streaming_ = false;
        return; // finished!
      }
      for(size_t i : syncCameras_) {
        const std::string & filename = iterators.cam_iterators.at(i)->second;
        cv::Mat filtered;
        boost::filesystem::path p(filename);
        std::string directory = p.parent_path().parent_path().filename().string();
        if(directory.size() > 3 && directory.substr(0,3).compare("cam") == 0) {
          filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE); // force gray already here
        } else {
          filtered = cv::imread(filename); // colour
        }
        DGVI_ASSERT_TRUE(
              Exception, !filtered.empty(),
              "cam " << i << " missing image :" << std::endl << filename)
        images[i] = filtered;
        iterators.cam_iterators.at(i)++; // advance to next
      }
    } else {
      const std::string & filename = iterators.cam_iterators.at(i_min)->second;
      cv::Mat filtered;
      boost::filesystem::path p(filename);
      std::string directory = p.parent_path().parent_path().filename().string();
      if(directory.size() > 3 && directory.substr(0,3).compare("cam") == 0) {
        filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE); // force gray already here
      } else {
        filtered = cv::imread(filename); // colour
      }
      DGVI_ASSERT_TRUE(
        Exception, !filtered.empty(),
        "cam " << i_min << " missing image :" << std::endl << filename)
      images[i_min] = filtered;
      iterators.cam_iterators.at(i_min)++; // advance to next
    }
    if(allDepthImageNames_.count(i_min)) {
      if(iterators.depthCam_iterators.at(i_min) == allDepthImageNames_.at(i_min).end()) {
        streaming_ = false;
        return; // finished!
      }
      const std::string & depthFilename = iterators.depthCam_iterators.at(i_min)->second;
      cv::Mat depthImg = cv::imread(depthFilename, cv::IMREAD_UNCHANGED);
      DGVI_ASSERT_TRUE(
            Exception, !depthImg.empty(),
            "cam " << i_min << " missing depth image :" << std::endl << depthFilename)

      // convert back
      if(depthImg.type() == CV_32FC1) {
        logImageTransform(depthImg);
      } else {
        depthImg.convertTo(depthImg,  CV_32F, 0.001);
      }

      depthImages[i_min] = depthImg;
      iterators.depthCam_iterators.at(i_min)++; // advance to next


      // next also read synced image, if not already added from sync group above.
      if(!syncCameras_.count(i_min)) {
        const std::string & filename = iterators.cam_iterators.at(i_min)->second;
        cv::Mat filtered;
        if(filename.substr(filename.size()-3,filename.size()).compare("png") == 0) {
          filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE); // force gray already here
        } else {
          filtered = cv::imread(filename); // colour
        }

        DGVI_ASSERT_TRUE(
            Exception, !filtered.empty(),
            "cam " << i_min << " missing image :" << std::endl << filename)
        images[i_min] = filtered;
        iterators.cam_iterators.at(i_min)++; // advance to next
      }
    }

    // time
    Time t;
    t.fromNSec(timestamp_min);
    if (start == dgvi::Time(0.0)) {
      start = t;
    }

    // get all IMU measurements till then
    dgvi::Time t_imu = start;
    do {
      if (!std::getline(imuFile_, line)) {
        streaming_ = false;
        return;
      }

      std::stringstream stream(line);
      std::string s;
      std::getline(stream, s, ',');
      uint64_t nanoseconds = std::stol(s.c_str());

      Eigen::Vector3d gyr;
      for (int j = 0; j < 3; ++j) {
        std::getline(stream, s, ',');
        gyr[j] = std::stof(s);
      }

      Eigen::Vector3d acc;
      for (int j = 0; j < 3; ++j) {
        std::getline(stream, s, ',');
        acc[j] = std::stof(s);
      }

      t_imu.fromNSec(nanoseconds);

      // add the IMU measurement for (blocking) processing
      if (t_imu - start + dgvi::Duration(1.0) > deltaT_) {
        for (auto &imuCallback : imuCallbacks_) {
          imuCallback(t_imu, acc, gyr);
        }
      }

    } while (t_imu <= t + dgvi::Duration(0.021));

      if(gpsFlag_){
          std::string gline;
          while(t_gps_ <= t)/*do*/ {
            if (!std::getline(gpsFile_, gline)) {
              streaming_ = false;
              return;
            }

            std::stringstream gstream(gline);
            std::string gs;

            // Distinguish GPS data type
            if(gpsDataType_ == "cartesian"){

              std::getline(gstream, gs, ',');
              uint64_t gnanoseconds = std::stol(gs.c_str()) - GNSS_LEAP_NANOSECONDS;

              // Filter burst-duplicate measurements (GPS receiver outputs stale buffered
              // fixes after a dropout gap, all within a few ms with impossible velocities).
              static constexpr uint64_t kMinGpsIntervalNs = 100000000ULL; // 0.1s
              if (lastGpsNs_ > 0 && gnanoseconds > lastGpsNs_ &&
                  gnanoseconds - lastGpsNs_ < kMinGpsIntervalNs) {
                LOG(WARNING) << "[GPS filter] Burst duplicate dropped: dt="
                             << (gnanoseconds - lastGpsNs_) / 1e6 << "ms < 100ms";
                t_gps_.fromNSec(gnanoseconds);
                continue;
              }

              Eigen::Vector3d pos;
              for (int j = 0; j < 3; ++j) {
                std::getline(gstream, gs, ',');
                pos[j] = std::stof(gs);
              }

              Eigen::Vector3d err;
              for (int j = 0; j < 3; ++j) {
                std::getline(gstream, gs, ',');
                err[j] = std::stof(gs);
              }

              if (maxGpsSpeed_ > 0.0 && lastCartesianGpsValid_ &&
                  lastGpsNs_ > 0 && gnanoseconds > lastGpsNs_) {
                const double dt = static_cast<double>(gnanoseconds - lastGpsNs_) * 1.0e-9;
                const double horizontalDistance =
                    (pos.head<2>() - lastCartesianGpsPosition_.head<2>()).norm();
                const double speed = horizontalDistance / dt;
                if (speed > maxGpsSpeed_) {
                  LOG(WARNING) << "[GPS filter] Cartesian GPS speed jump rejected: speed="
                               << speed << " m/s > max=" << maxGpsSpeed_
                               << " m/s, distance=" << horizontalDistance
                               << " m, dt=" << dt << " s, t=" << gnanoseconds;
                  t_gps_.fromNSec(gnanoseconds);
                  continue;
                }
              }

              lastGpsNs_ = gnanoseconds;
              lastCartesianGpsPosition_ = pos;
              lastCartesianGpsValid_ = true;
              t_gps_.fromNSec(gnanoseconds);

              // add the GPS measurement for (blocking) processing
              if (t_gps_ - start + dgvi::Duration(1.0) > deltaT_) {
                gpsCallback_(t_gps_, pos, err);
              }

            }
            else if (gpsDataType_ == "geodetic"){

              // 1st entry timestamp
              std::getline(gstream, gs, ',');
              uint64_t gnanoseconds = std::stol(gs.c_str()) - GNSS_LEAP_NANOSECONDS;

              // 2nd entry latitude
              std::getline(gstream, gs, ',');
              double lat = std::stod(gs.c_str());

              // 3rd entry longitude
              std::getline(gstream, gs, ',');
              double lon = std::stod(gs.c_str());

              // 4th entry height / altitude
              std::getline(gstream, gs, ',');
              double alt_raw = std::stod(gs.c_str());
              double alt = alt_raw; // default: use GPS altitude

              // 5th horizontal error
              std::getline(gstream, gs, ',');
              double hErr = std::stod(gs.c_str());

              // 6th vertical error
              std::getline(gstream, gs, ',');
              double vErr = std::stod(gs.c_str());

              // 7th-12th: vx, vy, vz, vel_dt, cov_type, fix_status (optional extra columns)
              std::string remaining;
              std::getline(gstream, remaining);
              if (!remaining.empty() && remaining.back() == '\r') remaining.pop_back();

              int fixStatus = -1;
              if (!remaining.empty()) {
                std::istringstream rem(remaining);
                std::string col;
                int colIdx = 0;
                while (std::getline(rem, col, ',')) {
                  if (colIdx == 5) { // 6th extra column = fix_status
                    if (!col.empty() && col.back() == '\r') col.pop_back();
                    if (!col.empty()) fixStatus = std::stoi(col);
                    break;
                  }
                  ++colIdx;
                }
              }

              // filter by fix_status
              if (fixStatus >= 0 && minFixStatus_ > 0 && fixStatus < minFixStatus_) {
                LOG(WARNING) << "[GPS filter] fix_status=" << fixStatus << " < min=" << minFixStatus_
                             << " t=" << gnanoseconds;
                continue;
              }

              // skip measurements with zero hErr (invalid covariance)
              if (hErr == 0.0) {
                LOG(WARNING) << "[GPS filter] hErr=0 (invalid covariance) t=" << gnanoseconds;
                continue;
              }

              // filter bad GPS points by error thresholds
              if (hErr > maxHErr_ || vErr > maxVErr_) {
                LOG(WARNING) << "[GPS filter] hErr=" << hErr << " vErr=" << vErr
                             << " exceeds max (hErr_max=" << maxHErr_ << " vErr_max=" << maxVErr_
                             << ") t=" << gnanoseconds;
                continue;
              }

              if (maxGpsSpeed_ > 0.0 && lastGeodeticGpsValid_ &&
                  lastGpsNs_ > 0 && gnanoseconds > lastGpsNs_) {
                const double dt = static_cast<double>(gnanoseconds - lastGpsNs_) * 1.0e-9;
                const double horizontalDistance = horizontalGeodeticDistanceMeters(
                    lastGeodeticGpsPosition_[0], lastGeodeticGpsPosition_[1], lat, lon);
                const double speed = horizontalDistance / dt;
                if (speed > maxGpsSpeed_) {
                  LOG(WARNING) << "[GPS filter] Geodetic GPS speed jump rejected: speed="
                               << speed << " m/s > max=" << maxGpsSpeed_
                               << " m/s, distance=" << horizontalDistance
                               << " m, dt=" << dt << " s, t=" << gnanoseconds;
                  t_gps_.fromNSec(gnanoseconds);
                  continue;
                }
              }

              // altitude resolution: geoid correction, then optionally DEM replacement
              if (geoid_) {
                double undulation = (*geoid_)(lat, lon);
                alt = alt_raw - undulation; // ellipsoidal -> orthometric
                static bool geoidLoggedOnce = false;
                if (!geoidLoggedOnce) {
                  LOG(INFO) << "[GPS geoid] First correction: undulation=" << undulation
                            << "m at (lat=" << lat << ", lon=" << lon
                            << "), alt_ellipsoidal=" << alt_raw << "m -> alt_orthometric=" << alt << "m";
                  geoidLoggedOnce = true;
                }
              }
              if (useDemHeightForGps_) {
                double h_dem = getDemHeight(lat, lon);
                if (h_dem >= -100.0) {
                  alt = h_dem;      // DEM replaces GPS altitude
                  vErr = demSigmaH_; // DEM sigma replaces GPS vErr
                }
              }

              t_gps_.fromNSec(gnanoseconds);
              lastGpsNs_ = gnanoseconds;
              lastGeodeticGpsPosition_ = Eigen::Vector3d(lat, lon, alt);
              lastGeodeticGpsValid_ = true;

              // add the GPS measurement for (blocking) processing
              if (t_gps_ - start + dgvi::Duration(1.0) > deltaT_) {
                geodeticGpsCallback_(t_gps_, lat, lon, alt, hErr, vErr);
              }
            }
            else if (gpsDataType_ == "geodetic-leica"){

              // 1st entry timestamp
              std::getline(gstream, gs, ',');
              uint64_t gnanoseconds = std::stol(gs.c_str()) - GNSS_LEAP_NANOSECONDS;

              // 2nd entry date
              std::getline(gstream, gs, ',');
              // 3rd entry time
              std::getline(gstream, gs, ',');
              // 4th entry fix
              std::getline(gstream, gs, ',');
              // 5th entry rtk
              std::getline(gstream, gs, ',');
              // 6th entry num_sv
              std::getline(gstream, gs, ',');

              // 7th entry latitude
              std::getline(gstream, gs, ',');
              double lat = std::stod(gs.c_str());

              // 8th entry longitude
              std::getline(gstream, gs, ',');
              double lon = std::stod(gs.c_str());

              // 9th entry height / altitude
              std::getline(gstream, gs, ',');
              double alt = std::stod(gs.c_str());

              // 10th entry hmsl
              std::getline(gstream, gs, ',');

              // 11th horizontal error
              std::getline(gstream, gs, ',');
              double hErr = std::stod(gs.c_str());

              // 12th vertical error
              std::getline(gstream, gs, ',');
              double vErr = std::stod(gs.c_str());

              if (maxGpsSpeed_ > 0.0 && lastGeodeticGpsValid_ &&
                  lastGpsNs_ > 0 && gnanoseconds > lastGpsNs_) {
                const double dt = static_cast<double>(gnanoseconds - lastGpsNs_) * 1.0e-9;
                const double horizontalDistance = horizontalGeodeticDistanceMeters(
                    lastGeodeticGpsPosition_[0], lastGeodeticGpsPosition_[1], lat, lon);
                const double speed = horizontalDistance / dt;
                if (speed > maxGpsSpeed_) {
                  LOG(WARNING) << "[GPS filter] Leica GPS speed jump rejected: speed="
                               << speed << " m/s > max=" << maxGpsSpeed_
                               << " m/s, distance=" << horizontalDistance
                               << " m, dt=" << dt << " s, t=" << gnanoseconds;
                  t_gps_.fromNSec(gnanoseconds);
                  continue;
                }
              }

              t_gps_.fromNSec(gnanoseconds);
              lastGpsNs_ = gnanoseconds;
              lastGeodeticGpsPosition_ = Eigen::Vector3d(lat, lon, alt);
              lastGeodeticGpsValid_ = true;

              // add the GPS measurement for (blocking) processing
              if (t_gps_ - start + dgvi::Duration(1.0) > deltaT_) {
                geodeticGpsCallback_(t_gps_, lat, lon, alt, hErr, vErr);
              }
            }

          } /*while (t_gps <= t);*/
      }

    // finally we are ready to call the callback
    for (auto& imagesCallback : imagesCallbacks_) {
      imagesCallback(t, images, depthImages);
    }
    if(images.count(0) && !images.at(0).empty()) {
      ++counter_; // reference for counter is always image 0.
    }
  }

  return;
}

}
