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
 * @file FrameRelativeAdapter.cpp
 * @brief Source file for the FrameRelativeAdapter class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <opengv/relative_pose/FrameRelativeAdapter.hpp>
#include <dgvi/FrameTypedefs.hpp>
#include <dgvi/MultiFrame.hpp>

// cameras and distortions
#include <dgvi/cameras/PinholeCamera.hpp>
#include <dgvi/cameras/EquidistantDistortion.hpp>
#include <dgvi/cameras/RadialTangentialDistortion.hpp>
#include <dgvi/cameras/RadialTangentialDistortion8.hpp>
#include <dgvi/cameras/EucmCamera.hpp>

// Constructor.
opengv::relative_pose::FrameRelativeAdapter::FrameRelativeAdapter(const Estimator &estimator,
    const dgvi::cameras::NCameraSystem & nCameraSystem, uint64_t multiFrameIdA,
    size_t camIdA, uint64_t multiFrameIdB, size_t camIdB) {

  std::shared_ptr<dgvi::MultiFrame> frameAPtr =
      estimator.multiFrame(dgvi::StateId(multiFrameIdA));
  std::shared_ptr<dgvi::MultiFrame> frameBPtr =
      estimator.multiFrame(dgvi::StateId(multiFrameIdB));

  // determine type
  dgvi::cameras::NCameraSystem::DistortionType distortionTypeA =
      nCameraSystem.distortionType(camIdA);
  dgvi::cameras::NCameraSystem::DistortionType distortionTypeB =
      nCameraSystem.distortionType(camIdB);

  double fu1 = 0;
  size_t numKeypointsA = frameAPtr->numKeypoints(camIdA);
  switch (distortionTypeA) {
    case dgvi::cameras::NCameraSystem::RadialTangential: {
      fu1 = frameAPtr
          ->geometryAs<
              dgvi::cameras::PinholeCamera<
                  dgvi::cameras::RadialTangentialDistortion> >(camIdA)
          ->focalLengthU();
      break;
    }
    case dgvi::cameras::NCameraSystem::Equidistant: {
      fu1 =
          frameAPtr
              ->geometryAs<
                  dgvi::cameras::PinholeCamera<
                      dgvi::cameras::EquidistantDistortion> >(camIdA)
              ->focalLengthU();
      break;
    }
    case dgvi::cameras::NCameraSystem::RadialTangential8: {
      fu1 = frameAPtr
          ->geometryAs<
              dgvi::cameras::PinholeCamera<
                  dgvi::cameras::RadialTangentialDistortion8> >(camIdA)
          ->focalLengthU();
      break;
    }
    case dgvi::cameras::NCameraSystem::NoDistortion: {
      fu1 = frameAPtr->geometryAs<dgvi::cameras::EucmCamera>(camIdA)->focalLengthU();
      break;
    }
    default:
      DGVI_THROW(Exception, "Unsupported distortion type")
      break;
  }
  double fu2 = 0.0;
  size_t numKeypointsB = frameBPtr->numKeypoints(camIdB);
  switch (distortionTypeB) {
    case dgvi::cameras::NCameraSystem::RadialTangential: {
      fu2 = frameAPtr
          ->geometryAs<
              dgvi::cameras::PinholeCamera<
                  dgvi::cameras::RadialTangentialDistortion> >(camIdB)
          ->focalLengthU();
      break;
    }
    case dgvi::cameras::NCameraSystem::Equidistant: {
      fu2 =
          frameAPtr
              ->geometryAs<
                  dgvi::cameras::PinholeCamera<
                      dgvi::cameras::EquidistantDistortion> >(camIdB)
              ->focalLengthU();
      break;
    }
    case dgvi::cameras::NCameraSystem::RadialTangential8: {
      fu2 = frameAPtr
          ->geometryAs<
              dgvi::cameras::PinholeCamera<
                  dgvi::cameras::RadialTangentialDistortion8> >(camIdB)
          ->focalLengthU();
      break;
    }
    case dgvi::cameras::NCameraSystem::NoDistortion: {
      fu2 = frameAPtr->geometryAs<dgvi::cameras::EucmCamera>(camIdB)->focalLengthU();
      break;
    }
    default:
      DGVI_THROW(Exception, "Unsupported distortion type")
      break;
  }

  // resize members
  bearingVectors1_.resize(numKeypointsA);
  bearingVectors2_.resize(numKeypointsB);
  sigmaAngles1_.resize(numKeypointsA);
  sigmaAngles2_.resize(numKeypointsB);

  matches_.reserve(std::min(numKeypointsA, numKeypointsB));
  std::map<uint64_t, size_t> idMap;
  for (size_t k = 0; k < numKeypointsB; ++k) {

    // get landmark id, if set
    uint64_t lmId = frameBPtr->landmarkId(camIdB, k);
    if (lmId == 0)
      continue;

    // check, if existing
    if (!estimator.isLandmarkAdded(dgvi::LandmarkId(lmId)))
      continue;

    // remember it
    idMap.insert(std::pair<uint64_t, size_t>(lmId, k));
  }

  for (size_t k = 0; k < numKeypointsA; ++k) {
    // get landmark id, if set
    uint64_t lmId = frameAPtr->landmarkId(camIdA, k);
    if (lmId == 0)
      continue;

    std::map<uint64_t, size_t>::const_iterator it = idMap.find(lmId);
    if (it != idMap.end()) {
      // whohoo, let's insert it.
      matches_.push_back(dgvi::Match(k, it->second, 0.0));
    }
  }

  // precompute
  for (size_t k = 0; k < matches_.size(); ++k) {
    const size_t idx1 = matches_[k].idxA;
    const size_t idx2 = matches_[k].idxB;
    Eigen::Vector2d keypoint;
    double keypointStdDev;
    frameAPtr->getKeypoint(camIdA, idx1, keypoint);
    frameAPtr->getKeypointSize(camIdA, idx1, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;
    sigmaAngles1_[idx1] = sqrt(2) * keypointStdDev * keypointStdDev
        / (fu1 * fu1);
    if(!frameAPtr->getBackProjection(camIdA, idx1, bearingVectors1_[idx1])) {
      bearingVectors1_[idx1] = Eigen::Vector3d(1,0,0);
      /// \todo think of a better way to deal with unsuccessful backpr.
    }
    bearingVectors1_[idx1].normalize();

    frameBPtr->getKeypoint(camIdB, idx2, keypoint);
    frameBPtr->getKeypointSize(camIdB, idx2, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;
    sigmaAngles2_[idx2] = sqrt(2) * keypointStdDev * keypointStdDev
        / (fu2 * fu2);
    if(!frameBPtr->getBackProjection(camIdB, idx2, bearingVectors2_[idx2])) {
      bearingVectors1_[idx1] = Eigen::Vector3d(1,0,0);
      /// \todo think of a better way to deal with unsuccessful backpr.
    }
    bearingVectors2_[idx2].normalize();
  }
}

// Retrieve the bearing vector of a correspondence in viewpoint 1.
opengv::bearingVector_t opengv::relative_pose::FrameRelativeAdapter::getBearingVector1(
    size_t index) const {
  return bearingVectors1_[matches_[index].idxA];
}

// Retrieve the bearing vector of a correspondence in viewpoint 2.
opengv::bearingVector_t opengv::relative_pose::FrameRelativeAdapter::getBearingVector2(
    size_t index) const {
  return bearingVectors2_[matches_[index].idxB];
}

// Retrieve the position of a camera of a correspondence in viewpoint 1 seen from the origin of the
// viewpoint.
opengv::translation_t opengv::relative_pose::FrameRelativeAdapter::getCamOffset1(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Vector3d::Zero();
}

// Retrieve the rotation from a camera of a correspondence in viewpoint 1 to the viewpoint origin.
opengv::rotation_t opengv::relative_pose::FrameRelativeAdapter::getCamRotation1(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Matrix3d::Identity();
}

// Retrieve the position of a camera of a correspondence in viewpoint 2 seen from the origin of the
// viewpoint.
opengv::translation_t opengv::relative_pose::FrameRelativeAdapter::getCamOffset2(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Vector3d::Zero();
}

// Retrieve the rotation from a camera of a correspondence in viewpoint 2 to the viewpoint origin.
opengv::rotation_t opengv::relative_pose::FrameRelativeAdapter::getCamRotation2(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Matrix3d::Identity();
}

// Retrieve the number of correspondences.
size_t opengv::relative_pose::FrameRelativeAdapter::getNumberCorrespondences() const {
  return matches_.size();
}

// Obtain the angular standard deviation of the correspondence in frame 1 in [rad].
double opengv::relative_pose::FrameRelativeAdapter::getSigmaAngle1(
    size_t index) {
  return sigmaAngles1_[matches_[index].idxA];
}

// Obtain the angular standard deviation of the correspondence in frame 2 in [rad].
double opengv::relative_pose::FrameRelativeAdapter::getSigmaAngle2(
    size_t index) {
  return sigmaAngles2_[matches_[index].idxB];
}
