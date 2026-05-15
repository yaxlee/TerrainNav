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
 * @file XDatasetReader.cpp
 * @brief Source file for the XDatasetReader class. 99% taken of DatasetReader.cpp
 * @author Simon Boche
 */

#include <algorithm>
#include <cmath>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/XDatasetReader.hpp>

namespace okvis {
namespace {
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

XDatasetReader::XDatasetReader(const std::string & path,
                             const Duration & deltaT, ViParameters parameters, bool use_lidar,
                             bool use_stereo_network, bool use_depth, bool use_rgb) :
        deltaT_(deltaT), lidarFlag_(use_lidar), networkFlag_(use_stereo_network), depthFlag_(use_depth), rgbFlag_(use_rgb), imuFlag_(parameters.imu.use){

  if(parameters.gps) {
    gpsFlag_ = true;
    gpsDataType_ = (*parameters.gps).type;
    OKVIS_ASSERT_TRUE(Exception, gpsDataType_=="cartesian" || gpsDataType_=="geodetic" || gpsDataType_=="geodetic-leica",
                      "Unknown GPS data type specified")
    maxHErr_ = (*parameters.gps).maxHErr;
    maxVErr_ = (*parameters.gps).maxVErr;
    maxGpsSpeed_ = (*parameters.gps).gpsMaxSpeed;
    minFixStatus_ = (*parameters.gps).minFixStatus;
    const std::string& geoidModel = (*parameters.gps).geoidModel;
    if (!geoidModel.empty()) {
      try {
        geoid_ = std::make_unique<GeographicLib::Geoid>(geoidModel);
        LOG(INFO) << "[GPS] Geoid model loaded: " << geoidModel;
      } catch (const std::exception& e) {
        LOG(WARNING) << "[GPS] Failed to load geoid model '" << geoidModel << "': " << e.what()
                     << " -- geoid undulation correction disabled.";
      }
    }
  }
  else {
    gpsFlag_ = false;
  }

  numCameras_ = parameters.nCameraSystem.numCameras();
  streaming_ = false;
  setDatasetPath(path);
  counter_ = 0;
  t_gps_ = okvis::Time(0.0);

  // Find the depth camera ids.
  if(use_depth) {
    for(int camId = 0; camId < numCameras_; camId ++) {
      if(parameters.nCameraSystem.isDepthCamera(camId)) {
        depthCameraId_ = camId;
      }
    }
  }
}

XDatasetReader::~XDatasetReader() {
  stopStreaming();
}

bool XDatasetReader::setDatasetPath(const std::string & path) {
  path_ = path;
  return true;
}

bool XDatasetReader::setStartingDelay(const Duration &deltaT)
{
  if(streaming_) {
    LOG(WARNING)<< "starting delay ignored, because streaming already started";
    return false;
  }
  deltaT_ = deltaT;
  return true;
}

bool XDatasetReader::isStreaming()
{
  return streaming_;
}

double XDatasetReader::completion() const {
  if(streaming_) {
    return double(counter_)/double(numImages_);
  }
  return 0.0;
}

bool XDatasetReader::startStreaming() {
  OKVIS_ASSERT_TRUE(Exception, !imagesCallbacks_.empty(), "no add image callback registered")
  if(imuFlag_)
    OKVIS_ASSERT_TRUE(Exception, !imuCallbacks_.empty(), "no add IMU callback registered")
  if(gpsFlag_)
    OKVIS_ASSERT_TRUE(Exception, geodeticGpsCallback_ || gpsCallback_, "no add GPS callback registered")
  if(lidarFlag_)
    OKVIS_ASSERT_TRUE(Exception, lidarCallback_, "no add Lidar callback registered")
  if(networkFlag_)
    OKVIS_ASSERT_TRUE(Exception, imagesNetworkCallback_, "no add stereo network callback registered")
  // if(depthFlag_)
  //   OKVIS_ASSERT_TRUE(Exception, depthCallback_, "No add depth callback registered");

  // open the IMU file
  std::string line;
  if(imuFlag_) {
    imuFile_.open(path_ + "/imu0/data.csv");
    OKVIS_ASSERT_TRUE(Exception, imuFile_.good(), "no imu file found at " << path_+"/imu0/data.csv");
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
  }

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
    OKVIS_ASSERT_TRUE(Exception, gpsFile_.good(), "no gps file found at " << path_+"/gps0/");
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

  if(lidarFlag_){
    // Open LiDAR file
    std::string lidarLine;
    lidarFile_.open(path_ + "/lidar0/data.csv");
    OKVIS_ASSERT_TRUE(Exception, lidarFile_.good(), "no lidar file found at " << path_+"/lidar0/data.csv");
    /*int number_of_lidar_lines = 0;
    while (std::getline(lidarFile_, lidarLine))
      ++number_of_lidar_lines;
    LOG(INFO)<< "No. LiDAR measurements: " << number_of_lidar_lines-1;
    if (number_of_lidar_lines - 1 <= 0) {
      LOG(ERROR)<< "no lidar messages present in " << path_+"/lidar0/data.csv";
      return -1;
    }
    // set reading position to second line
    lidarFile_.clear();
    lidarFile_.seekg(0, std::ios::beg);*/
    std::getline(lidarFile_, lidarLine);
  }

  if(depthFlag_){
    // Open depth files
    int num_depth_images = 0;
    std::ifstream depthDataFile(path_ + "/depth0/data.csv");
    std::pair<std::string, std::string> depthNames;
    std::getline(depthDataFile, line);
    while (std::getline(depthDataFile, line)) {
      ++num_depth_images;
      std::stringstream stream(line);
      std::string s0, s1;
      std::getline(stream, s0, ',');
      std::getline(stream, s1);
      depthNames = std::make_pair(s0,s1);
      allDepthNames_.push_back(depthNames);
    }
    LOG(INFO) << "No. depth images: " << num_depth_images;
  }


  // now open camera files
  std::vector<okvis::Time> times;
  okvis::Time latest(0);
  int num_camera_images = 0;
  int i = 0;
  while (i<numCameras_) {
    std::ifstream camDataFile(path_ + "/cam" + std::to_string(i) + "/data.csv");
    if(!camDataFile.good()) {
      OKVIS_ASSERT_TRUE(Exception, i>0, "No camera data found");
      break;
    }
    num_camera_images = 0;
    std::vector < std::pair<std::string, std::string> > imageNames;
    std::getline(camDataFile, line);
    while (std::getline(camDataFile, line)) {
      ++num_camera_images;
      std::stringstream stream(line);
      std::string s0, s1;
      std::getline(stream, s0, ',');
      std::getline(stream, s1, ',');
      // If the file is written in Windows. (\r\n)
      if (s1.back() == '\r') {
        s1 = s1.substr(0, s1.size() - 1);
      }
      imageNames.push_back(std::make_pair(s0,s1));
    }
    allImageNames_.push_back(imageNames);
    LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;
    if(i==0) {
      numImages_ = num_camera_images;
    }
    ++i; // try next camera
  }

  if(rgbFlag_) {
    // Open depth files
    int num_rgb_images = 0;
    std::ifstream rgbDataFile(path_ + "/rgb0/data.csv");
    std::vector < std::pair<std::string, std::string> > rgbNames;
    std::getline(rgbDataFile, line);
    while (std::getline(rgbDataFile, line)) {
      ++num_rgb_images;
      std::stringstream stream(line);
      std::string s0, s1;
      std::getline(stream, s0, ',');
      std::getline(stream, s1);
      // If the file is written in Windows. (\r\n)
      if (s1.back() == '\r') {
        s1 = s1.substr(0, s1.size() - 1);
      }
      rgbNames.push_back(std::make_pair(s0,s1));
    }
    LOG(INFO) << "No. rgb images: " << num_rgb_images;
    allImageNames_.push_back(rgbNames);
  }


  counter_ = 0;
  streaming_ = true;
  processingThread_ = std::thread(&XDatasetReader::processing, this);

  return true;
}

bool XDatasetReader::stopStreaming() {
  // Stop the pipeline
  if(processingThread_.joinable()) {
    processingThread_.join();
    streaming_ = false;
  }
  return true;
}

void  XDatasetReader::processing() {
  std::string line;
  okvis::Time start(0.0);
  const size_t numCameras = allImageNames_.size();
  std::vector < std::vector < std::pair<std::string, std::string> > ::iterator
  > cam_iterators(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    cam_iterators.at(i) = allImageNames_.at(i).begin();
  }

  std::vector < std::pair<std::string, std::string> > ::iterator depth_iterators;
  depth_iterators = allDepthNames_.begin();

  const uint64_t tolNSec = 10000000; // 0.01 sec
  while (streaming_) {

    // sync and check if at the end
    bool synched = false;
    while(!synched) {
      uint64_t max_timestamp = 0;
      for (size_t i = 0; i < numCameras; ++i) {
        if(cam_iterators.at(i) == allImageNames_.at(i).end()) {
          streaming_ = false;
          return; // finished!
        }
        uint64_t tcam = std::atol(cam_iterators.at(i)->first.c_str());
        if(tcam>max_timestamp) {
          max_timestamp = tcam;
        }
      }
      synched=true;
      for (size_t i = 0; i < numCameras; ++i) {
        uint64_t tcam = std::atol(cam_iterators.at(i)->first.c_str());
        if(tcam < max_timestamp - tolNSec) {
          LOG(INFO) << "Out of sync by " << max_timestamp - tcam;
          cam_iterators.at(i)++; // not in tolerance, advance;
          synched = false;
        }
      }
    }

    // check synched (I am paranoid)
    for (size_t i = 0; i < numCameras; ++i) {
      if(i>0) {
        int64_t tcam = std::atol(cam_iterators.at(0)->first.c_str());
        int64_t tcam_i = std::atol(cam_iterators.at(i)->first.c_str());
        OKVIS_ASSERT_TRUE(Exception, abs(tcam-tcam_i)<int64_t(tolNSec),
                          "timestamp mismatch " << double(abs(tcam-tcam_i))/1.0e6 << " ms at " << tcam_i);
      }
    }

    // add images
    okvis::Time t;
    std::map<size_t, cv::Mat> images;
    std::map<size_t, cv::Mat> depthImages;
    std::map<size_t, std::pair<okvis::Time, cv::Mat>> network_images;
    std::map<size_t, std::pair<okvis::Time, cv::Mat>> network_depthImages;
    for (size_t i = 0; i < numCameras; ++i) {
      cv::Mat filtered;
      std::pair<okvis::Time, cv::Mat> timestamp_filtered;
      std::string filename;
      if(rgbFlag_ && i == (numCameras - 1)) {
        filename = path_ + "/rgb0/data/" + cam_iterators.at(i)->second;
        filtered = cv::imread(filename, cv::IMREAD_COLOR);
        cv::cvtColor(filtered, filtered, cv::COLOR_BGR2RGB);
      } else {
        filename = path_ + "/cam" + std::to_string(i) + "/data/" + cam_iterators.at(i)->second;
        filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE);
      }

      OKVIS_ASSERT_TRUE(
              Exception, !filtered.empty(),
              "cam " << i << " missing image :" << std::endl << filename)

      t.fromNSec(std::atol(cam_iterators.at(i)->first.c_str()));
      timestamp_filtered = std::make_pair(t, filtered);

      if (start == okvis::Time(0.0)) {
        start = t;
      }

      if(lidarFlag_){
        // get all LiDAR measurements till then
        okvis::Time t_lidar = start;
        std::string lidarLine;
        do {
          if (!std::getline(lidarFile_, lidarLine)) {
            streaming_ = false;
            return;
          }

          std::stringstream lidarStream(lidarLine);
          std::string sl;
          // 1st entry timestamp
          std::getline(lidarStream, sl, ',');
          uint64_t lidarNanoseconds = std::stol(sl.c_str());

          // 2nd - 4th entry: xyz coordinates in LiDAR frame
          Eigen::Vector3d ptXYZ;
          for(int j = 0; j < 3; ++j){
            std::getline(lidarStream, sl, ',');
            ptXYZ[j] = std::stod(sl);
          }
          // 5th entry: intensity
          std::getline(lidarStream, sl);

          t_lidar.fromNSec(lidarNanoseconds);

          // add the lidar measurement for (blocking) processing
          if (t_lidar - start + okvis::Duration(1.0) > deltaT_) {
            lidarCallback_(t_lidar, ptXYZ);
          }

        } while (t_lidar <= t);
      }

      if (depthFlag_) {
        // get all depth image till then
        okvis::Time t_depth = start;
        while(true) {
          if (depth_iterators == allDepthNames_.end()) {
            LOG(INFO) << "depth stop streaming";
            streaming_ = false;
            return;
          }
          uint64_t depthNanoseconds = std::stol(depth_iterators->first);
          t_depth.fromNSec(depthNanoseconds);
          if (t_depth > t) break;
          std::string filename = path_ + "/depth0/data/" + depth_iterators->second;
          cv::Mat depth = cv::imread(filename, cv::IMREAD_UNCHANGED);
          if (t_depth - start + okvis::Duration(1.0) > deltaT_) {
            if (depthFlag_) {
              okvis::CameraMeasurement camMeasurement;
              camMeasurement.timeStamp = t_depth;
              depth.convertTo(depth, CV_32F);
              camMeasurement.measurement.depthImage = depth;
              std::map<size_t, std::vector<okvis::CameraMeasurement>> si_data;
              si_data[depthCameraId_] = {camMeasurement};
              depthImages.emplace(std::make_pair(depthCameraId_, depth.clone()));
              network_depthImages.emplace(std::make_pair(depthCameraId_, std::make_pair(t_depth, depth.clone())));
              if(depthCallback_) {
                depthCallback_(si_data);
              }
            }
            // LOG(INFO) << "Depth callback from streaming :" << filename << ", t_depth = " << t_depth << ", t = " << t;
          }

          depth_iterators ++;

        }
      }

      // get all IMU measurements till then
      if(imuFlag_){
        okvis::Time t_imu = start;
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
          if (t_imu - start + okvis::Duration(1.0) > deltaT_) {
            for (auto &imuCallback : imuCallbacks_) {
              imuCallback(t_imu, acc, gyr);
            }
          }

        } while (t_imu <= t);
      }

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
            if (t_gps_ - start + okvis::Duration(1.0) > deltaT_) {
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
            double alt = std::stod(gs.c_str());

            // 5th horizontal error
            std::getline(gstream, gs, ',');
            double hErr = std::stod(gs.c_str());

            // 6th vertical error
            std::getline(gstream, gs, ',');
            double vErr = std::stod(gs.c_str());

            // 7th-12th: vx, vy, vz, vel_dt, cov_type, fix_status (optional extra columns)
            // Read remaining line and parse fix_status only if it exists.
            std::string remaining;
            std::getline(gstream, remaining);
            if (!remaining.empty() && remaining.back() == '\r') remaining.pop_back();

            int fixStatus = -1; // -1 = unknown (CSV has no extra columns)
            if (!remaining.empty()) {
              // Count commas to find fix_status position (5 skips + 1 value)
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

            // filter by fix_status (only when column is present)
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

            // apply geoid undulation correction: convert ellipsoidal -> orthometric height
            if (geoid_) {
              alt -= (*geoid_)(lat, lon);
            }

            t_gps_.fromNSec(gnanoseconds);
            lastGpsNs_ = gnanoseconds;
            lastGeodeticGpsPosition_ = Eigen::Vector3d(lat, lon, alt);
            lastGeodeticGpsValid_ = true;

            // add the GPS measurement for (blocking) processing
            if (t_gps_ - start + okvis::Duration(1.0) > deltaT_) {
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
                LOG(WARNING) << "[GPS filter] Leica GPS speed jump rejected: speed="
                             << speed << " m/s > max=" << maxGpsSpeed_
                             << " m/s, distance=" << horizontalDistance
                             << " m, dt=" << dt << " s, t=" << gnanoseconds;
                t_gps_.fromNSec(gnanoseconds);
                continue;
              }
            }

            // apply geoid undulation correction: convert ellipsoidal -> orthometric height
            if (geoid_) {
              alt -= (*geoid_)(lat, lon);
            }

            t_gps_.fromNSec(gnanoseconds);
            lastGpsNs_ = gnanoseconds;
            lastGeodeticGpsPosition_ = Eigen::Vector3d(lat, lon, alt);
            lastGeodeticGpsValid_ = true;

            // add the GPS measurement for (blocking) processing
            if (t_gps_ - start + okvis::Duration(1.0) > deltaT_) {
              geodeticGpsCallback_(t_gps_, lat, lon, alt, hErr, vErr);
            }
          }

        } /*while (t_gps <= t);*/


      }

      // add the image to the frontend for (blocking) processing
      if (t - start > deltaT_) {
        images[i] = filtered;
        network_images[i] = timestamp_filtered;
      }

      cam_iterators[i]++;

    }
    if(images.size() == numCameras) {
      for (auto& imagesCallback : imagesCallbacks_) {
        imagesCallback(t, images, depthImages);
      }
      if (networkFlag_)
        imagesNetworkCallback_(network_images, network_depthImages);
      ++counter_;
    }
  }

  return;
}

}
