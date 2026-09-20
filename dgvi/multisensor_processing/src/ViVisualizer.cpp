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
 * @file ViVisualizer.cpp
 * @brief Source file for the ViVisualizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */


#include <dgvi/kinematics/Transformation.hpp>

#include <dgvi/cameras/NCameraSystem.hpp>
#include <dgvi/FrameTypedefs.hpp>

#include "dgvi/ViVisualizer.hpp"

// cameras and distortions
#include <dgvi/cameras/PinholeCamera.hpp>
#include <dgvi/cameras/EquidistantDistortion.hpp>
#include <dgvi/cameras/RadialTangentialDistortion.hpp>
#include <dgvi/cameras/RadialTangentialDistortion8.hpp>
#include <dgvi/cameras/EucmCamera.hpp>

/// \brief dgvi Main namespace of this package.
namespace dgvi {

ViVisualizer::ViVisualizer(ViParameters &parameters)
    : parameters_(parameters) {
  if (parameters.nCameraSystem.numCameras() > 0) {
    init(parameters);
  }
}

ViVisualizer::~ViVisualizer() {
}

void ViVisualizer::init(ViParameters &parameters) {
  parameters_ = parameters;
}

cv::Mat ViVisualizer::drawMatches(VisualizationData::Ptr& data,
                                   size_t image_number) {

  const kinematics::Transformation T_WS = data->T_WS;
  const kinematics::Transformation T_SC = data->T_SCi.at(image_number);
  //const kinematics::Transformation T_SC = *parameters_.nCameraSystem.T_SC(image_number);
  const kinematics::Transformation T_WC = T_WS*T_SC;
  const kinematics::Transformation T_CW = T_WC.inverse();

  std::shared_ptr<dgvi::MultiFrame> frame = data->currentFrames;

  // allocate an image
  const int im_cols = frame->image(image_number).cols;
  const int im_rows = frame->image(image_number).rows;

  cv::Mat outimg(im_rows, im_cols, CV_8UC3);
  cv::Mat current = outimg;

  cv::cvtColor(frame->image(image_number), current, cv::COLOR_GRAY2BGR);

  // loop-closure frame?
  cv::Mat tmp(im_rows, im_cols, CV_8UC3);
  if(data->recognisedPlace) {
    cv::Scalar colour(255,0,0);
    tmp.setTo(colour);
    cv::addWeighted(tmp, 0.5, current, 1.0 - 0.5, 0, current);
    cv::putText(current, "Place recognised", cv::Point2f(200,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                colour, 1, cv::LINE_AA);
  }

  // keyframe?
  if(data->isKeyframe) {
    cv::Scalar colour(255,255,0);
    tmp.setTo(colour);
    cv::addWeighted(tmp, 0.3, current, 1.0 - 0.3, 0, current);
    cv::putText(current, "Keyframe", cv::Point2f(120,10), cv::FONT_HERSHEY_COMPLEX, 0.3, colour, 1,
                cv::LINE_AA);
  }

  // Quality
  cv::Scalar trackingColour;
  if(data->trackingQuality == VisualizationData::TrackingQuality::Lost) {
    trackingColour = cv::Scalar(0,0,255);
    cv::putText(current, "TRACKING LOST", cv::Point2f(5,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                trackingColour, 1, cv::LINE_AA);
  } else if (data->trackingQuality == VisualizationData::TrackingQuality::Marginal) {
    trackingColour = cv::Scalar(0,255,255);
    cv::putText(current, "Tracking marginal", cv::Point2f(5,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                trackingColour, 1, cv::LINE_AA);
  } else {
    trackingColour = cv::Scalar(0,255,0);
    cv::putText(current, "Tracking good", cv::Point2f(5,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                trackingColour, 1, cv::LINE_AA);
  }

  // find distortion type
  dgvi::cameras::NCameraSystem::DistortionType distortionType = parameters_.nCameraSystem
      .distortionType(0);
  for (size_t i = 1; i < parameters_.nCameraSystem.numCameras(); ++i) {
    DGVI_ASSERT_TRUE(Exception,
                      distortionType == parameters_.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet")
  }

  for (auto it = data->observations.begin(); it != data->observations.end();
      ++it) {
    if (it->cameraIdx != image_number)
      continue;

    cv::Scalar color;

    if (it->landmarkId != 0) {
      color = cv::Scalar(255, 0, 0);  // blue
    } else {
      color = cv::Scalar(0, 0, 255);  // red
    }

    // draw matches
    auto keypoint = it->keypointMeasurement;
    if (fabs(it->landmark_W[3]) > 1.0e-8) {
      Eigen::Vector4d hPoint = it->landmark_W;
      if (it->isInitialized) {
        color = cv::Scalar(0, 255, 0);  // green
        if (it->classification == 10 || it->classification == 11) {
            color = cv::Scalar(0, 80, 0);  // dark green
        }
      } else {
        color = cv::Scalar(0, 255, 255);  // yellow
        if (it->classification == 10 || it->classification == 11) {
            color = cv::Scalar(0, 50, 50);  // dark yellew -> olive
        }
      }
      Eigen::Vector2d projection;
      bool isVisible = false;
      Eigen::Vector4d hP_C = T_CW * hPoint;
      switch (distortionType) {
        case dgvi::cameras::NCameraSystem::RadialTangential: {
          if (frame
              ->geometryAs<
                  dgvi::cameras::PinholeCamera<
                      dgvi::cameras::RadialTangentialDistortion>>(image_number)
              ->projectHomogeneous(hP_C, &projection)
              == dgvi::cameras::ProjectionStatus::Successful)
            isVisible = true;
          break;
        }
        case dgvi::cameras::NCameraSystem::Equidistant: {
          if (frame
              ->geometryAs<
                  dgvi::cameras::PinholeCamera<
                      dgvi::cameras::EquidistantDistortion>>(image_number)
              ->projectHomogeneous(hP_C, &projection)
              == dgvi::cameras::ProjectionStatus::Successful)
            isVisible = true;
          break;
        }
        case dgvi::cameras::NCameraSystem::RadialTangential8: {
          if (frame
              ->geometryAs<
                  dgvi::cameras::PinholeCamera<
                      dgvi::cameras::RadialTangentialDistortion8>>(
              image_number)->projectHomogeneous(hP_C, &projection)
              == dgvi::cameras::ProjectionStatus::Successful)
            isVisible = true;
          break;
        }
        case dgvi::cameras::NCameraSystem::NoDistortion: {
          if (frame->geometryAs<dgvi::cameras::EucmCamera>(image_number)->projectHomogeneous(hP_C, &projection)
          == dgvi::cameras::ProjectionStatus::Successful)
            isVisible = true;
          break;
        }
        default:
          DGVI_THROW(Exception, "Unsupported distortion type.")
          break;
      }
      if (fabs(hP_C[3]) > 1.0e-8) {
        if (hP_C[2] / hP_C[3] < 0.03) {
          isVisible = false;
        }
      }

      // draw projection
      if(isVisible) {
        cv::Point2f projectionCv(projection[0], projection[1]);
        cv::line(current, projectionCv,
            cv::Point2f(float(keypoint[0]), float(keypoint[1])), color, 1, cv::LINE_AA);
        cv::circle(current, projectionCv, 2,
            cv::Scalar(255,255,0), cv::FILLED, cv::LINE_AA);
        std::stringstream idText;
        idText << it->landmarkId;
        cv::putText(current, idText.str(),
                    cv::Point2f(float(projection[0]), float(projection[1])) + cv::Point2f(10,5),
                     cv::FONT_HERSHEY_COMPLEX, 0.35, cv::Scalar(255,255,0), 1, cv::LINE_AA);
      }
    }
    // draw keypoint
    const double r = 0.5 * it->keypointSize;
    cv::circle(current, cv::Point2f(float(keypoint[0]), float(keypoint[1])), int(r), color, 1,
        cv::LINE_AA);
    cv::KeyPoint cvKpt;
    frame->getCvKeypoint(it->cameraIdx, it->keypointIdx, cvKpt);
    const float angle = cvKpt.angle / 180.0f * float(M_PI);
    cv::line(outimg, cvKpt.pt,
             cv::Point2f(cvKpt.pt.x + float(r) * cos(angle), cvKpt.pt.y + float(r) * sin(angle)),
             color, 1, cv::LINE_AA);
  }
  return outimg;
}


cv::Mat ViVisualizer::drawKeypoints(VisualizationData::Ptr& data,
                                     size_t cameraIndex) {

  std::shared_ptr<dgvi::MultiFrame> currentFrames = data->currentFrames;
  cv::Mat currentImage = currentFrames->image(cameraIndex);

  cv::Mat outimg;
  cv::cvtColor(currentImage, outimg, cv::COLOR_GRAY2BGR);
  cv::Scalar greenColor(0, 255, 0);  // green

  cv::KeyPoint keypoint;
  for (size_t k = 0; k < currentFrames->numKeypoints(cameraIndex); ++k) {
    currentFrames->getCvKeypoint(cameraIndex, k, keypoint);

    const float radius = keypoint.size;
    const float angle = keypoint.angle / 180.0f * float(M_PI);

    cv::circle(outimg, keypoint.pt, int(radius), greenColor);
    cv::line(
        outimg,
        keypoint.pt,
        cv::Point2f(keypoint.pt.x + radius * cos(angle),
                    keypoint.pt.y - radius * sin(angle)),
        greenColor);
  }

  return outimg;
}

} /* namespace dgvi */
