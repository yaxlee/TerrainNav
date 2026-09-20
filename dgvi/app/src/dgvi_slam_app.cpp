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
 * @file dgvi_slam_app.cpp
 * @brief This file processes a dataset.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <algorithm>
#include <cctype>
#include <vector>

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma GCC diagnostic pop
#include <dgvi/ViParametersReader.hpp>
#include <dgvi/ThreadedSlam.hpp>
#include <dgvi/DatasetReader.hpp>
#include <dgvi/RpgDatasetReader.hpp>
#include <dgvi/TrajectoryOutput.hpp>
#include <boost/filesystem.hpp>

#include <execinfo.h>


/// \brief Main
/// \param argc argc.
/// \param argv argv.

int main(int argc, char **argv)
{

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_minloglevel = 0;

  // Optional output directory, DEM rasters, and inherited RPG reader.
  if (argc < 3) {
    LOG(ERROR) <<
    "Usage: " << argv[0] << " configuration-yaml-file dataset-folder [save-path] [dem-path ...] [-rpg]";
    return EXIT_FAILURE;
  }

  dgvi::Duration deltaT(0.0);
  bool rpg = false;
  std::string savePath = "results";
  std::vector<std::string> demPaths;

  auto isDemPath = [](const std::string& path) {
    std::string extension = boost::filesystem::path(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return extension == ".tif" || extension == ".tiff" || extension == ".vrt";
  };

  // Collect DEM rasters and reader selection.
  for (int i = 3; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "-rpg") {
      rpg = true;
    } else if (isDemPath(arg)) {
      demPaths.push_back(arg);
    }
  }

  // Select the optional output directory.
  if (argc >= 4) {
    for (int i = 3; i < argc; ++i) {
      std::string arg(argv[i]);
      if (arg != "-rpg" && !isDemPath(arg)) {
        savePath = arg;
        break;
      }
    }
  }

  // Create savePath directory if it does not exist.
  if(!savePath.empty()) {
    boost::filesystem::create_directories(savePath);
  }

  // read configuration file
  std::string configFilename(argv[1]);

  dgvi::ViParametersReader viParametersReader(configFilename);
  dgvi::ViParameters parameters;
  viParametersReader.getParameters(parameters);

  const bool useDem = parameters.dem && parameters.dem->use;
  if (useDem && (!parameters.gps || parameters.gps->type != "geodetic" || rpg || demPaths.empty())) {
    LOG(ERROR) << "DEM requires the geodetic DatasetReader and at least one .tif/.tiff/.vrt raster.";
    return EXIT_FAILURE;
  }
  if (!useDem) {
    demPaths.clear();
  }

  // dataset reader
  // the folder path
  std::string path(argv[2]);
  std::shared_ptr<dgvi::DatasetReaderBase> datasetReader;
  if(rpg){
    datasetReader.reset(new dgvi::RpgDatasetReader(
                          path, deltaT, int(parameters.nCameraSystem.numCameras())));
  } else {
    datasetReader.reset(new dgvi::DatasetReader(
                          path, int(parameters.nCameraSystem.numCameras()),
                          parameters.camera.sync_cameras, deltaT, parameters.gps, parameters.dem, demPaths));
  }

  // also check DBoW2 vocabulary
  boost::filesystem::path executable(argv[0]);
  std::string dBowVocDir = executable.remove_filename().string();
  std::ifstream infile(dBowVocDir+"/small_voc.yml.gz");
  if(!infile.good()) {
     LOG(ERROR)<<"DBoW2 vocaublary " << dBowVocDir << "/small_voc.yml.gz not found.";
     return EXIT_FAILURE;
  }

  dgvi::ThreadedSlam estimator(parameters, dBowVocDir);
  estimator.setBlocking(true);

  // write logs
  std::string mode = "slam";
  if(!parameters.estimator.do_loop_closures) {
    mode = "vio";
  }
  if(parameters.camera.online_calibration.do_extrinsics) {
    mode = mode+"-calib";
  }

  const bool isWriteRpg = false;
  dgvi::TrajectoryOutput writer(savePath+"/dgvi-slam-" + mode + "_trajectory.csv", isWriteRpg, parameters.output.display_topview);
  estimator.setOptimisedGraphCallback(
        std::bind(&dgvi::TrajectoryOutput::processState, &writer,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4));
  estimator.setFinalTrajectoryCsvFile(savePath+"/dgvi-slam-" + mode + "-final_trajectory.csv", isWriteRpg);
  estimator.setMapCsvFile(savePath+"/dgvi-slam-" + mode + "-final_map.csv");

  // connect reader to estimator
  datasetReader->setImuCallback(
        std::bind(&dgvi::ThreadedSlam::addImuMeasurement, &estimator,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  datasetReader->setImagesCallback(
        std::bind(&dgvi::ThreadedSlam::addImages, &estimator, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
  if(parameters.gps) {
    if ((*parameters.gps).type == "cartesian") {
      datasetReader->setGpsCallback(
              std::bind(&dgvi::ThreadedSlam::addGpsMeasurement, &estimator,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    } else if ((*parameters.gps).type == "geodetic" || (*parameters.gps).type == "geodetic-leica") {
      datasetReader->setGeodeticGpsCallback(
              std::bind(&dgvi::ThreadedSlam::addGeodeticGpsMeasurement, &estimator,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                        std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
    } else {
      LOG(ERROR) << "Unknown GPS data type.";
      return EXIT_FAILURE;
    }

    // Register DEM callback for high-frequency height constraints (after T_GW is fixed)
    if(useDem && !demPaths.empty()) {
      auto* datasetReaderPtr = dynamic_cast<dgvi::DatasetReader*>(datasetReader.get());
      if(datasetReaderPtr) {
        const double sigma_h = (*parameters.dem).sigma_h;
        const double d_above_ground = (*parameters.dem).d_above_ground;
        const Eigen::Vector3d r_SA = (*parameters.dem).r_SA;
        const bool use_dem_height_for_gps = (*parameters.dem).useDemHeightForGps;
        const double dem_fusion_alpha = (*parameters.dem).demFusionAlpha;
        estimator.setDemCallback(
            [datasetReaderPtr](double lat, double lon) -> double {
              return datasetReaderPtr->getDemHeight(lat, lon);
            },
            sigma_h, d_above_ground, r_SA, use_dem_height_for_gps, dem_fusion_alpha);
        LOG(INFO) << "DEM callback registered (sigma_h=" << sigma_h
                  << "m, d_above_ground=" << d_above_ground
                  << "m, dem_count=" << demPaths.size() << ").";
      }
    }
  }

  // start
  dgvi::Time startTime = dgvi::Time::now();
  datasetReader->startStreaming();
  int progress = 0;
  while (true) {
    estimator.processFrame();
    std::map<std::string, cv::Mat> images;
    estimator.display(images);
    for(const auto & image : images) {
      cv::imshow(image.first, image.second);
    }
    cv::Mat topView;
    writer.drawTopView(topView);
    if(!topView.empty()) {
      cv::imshow("DGVI-SLAM Top View", topView);
    }
    if(!images.empty() || !topView.empty()) {
      char b = cv::waitKey(2);
      if (b == 's') {
        cv::imwrite("saved.png", topView);
      }
    }

    // check if done
    if(!datasetReader->isStreaming()) {
      estimator.stopThreading();
      LOG(INFO) << "Finished!" << std::endl;
      estimator.writeFinalTrajectoryCsv();
      if(parameters.gps){
        estimator.writeGlobalTrajectoryCsv(savePath+"/dgvi-slam-" + mode + "-global-final_trajectory.csv");
      }
      estimator.setFinalTrajectoryCsvFile(savePath+"/dgvi-slam-" + mode + "-final-ba_trajectory.csv", isWriteRpg);
      if(parameters.estimator.do_final_ba) {
        LOG(INFO) << "final full BA...";
        cv::Mat topView;
        estimator.doFinalBa();
        writer.drawTopView(topView);
        if (!topView.empty()) {
          cv::imshow("DGVI-SLAM Top View Final", topView);
          cv::imwrite("dgvi_slam_final_ba.png", topView);
        }
        cv::waitKey(1000);
      }
      estimator.writeFinalTrajectoryCsv();
      if(parameters.gps){
        estimator.writeGlobalTrajectoryCsv(savePath+"/dgvi-slam-" + mode + "-global-final-ba_trajectory.csv");
      }
      if(parameters.estimator.do_final_ba) {
        estimator.saveMap();
      }
      LOG(INFO) <<"total processing time " << (dgvi::Time::now() - startTime) << " s" << std::endl;
      break;
    }

    // display progress
    int newProgress = int(datasetReader->completion()*100.0);
#ifndef DEACTIVATE_TIMERS
    if (newProgress>progress) {
      LOG(INFO) << dgvi::timing::Timing::print();
    }
#endif
    if (newProgress>progress) {
      progress = newProgress;
      LOG(INFO) << "Progress: " << progress << "% ";
    }
  }
  return EXIT_SUCCESS;
}
