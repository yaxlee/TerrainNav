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

#include "glog/logging.h"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ceres/cost_function.h>
#include <ceres/crs_matrix.h>
#include <ceres/evaluation_callback.h>
#include <ceres/iteration_callback.h>
#include <ceres/loss_function.h>
#include <ceres/manifold.h>
#include <ceres/ordered_groups.h>
#include <ceres/problem.h>
#include <ceres/product_manifold.h>
#include <ceres/sized_cost_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <ceres/version.h>
#pragma GCC diagnostic pop

#include <dgvi/cameras/PinholeCamera.hpp>
#include <dgvi/cameras/EquidistantDistortion.hpp>
#include <dgvi/ceres/HomogeneousPointError.hpp>
#include <dgvi/ceres/ReprojectionError.hpp>
#include <dgvi/ceres/PoseParameterBlock.hpp>
#include <dgvi/ceres/PoseLocalParameterization.hpp>
#include <dgvi/ceres/HomogeneousPointLocalParameterization.hpp>
#include <dgvi/ceres/HomogeneousPointParameterBlock.hpp>
#include <dgvi/kinematics/Transformation.hpp>
#include <dgvi/Time.hpp>
#include <dgvi/FrameTypedefs.hpp>
#include <dgvi/assert_macros.hpp>

TEST(dgviTestSuite, ReprojectionError){
	// initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

	// Build the problem.
	::ceres::Problem problem;

	// set up a random geometry
	std::cout<<"set up a random geometry... "<<std::flush;
	dgvi::kinematics::Transformation T_WS; // world to sensor
	T_WS.setRandom(10.0,M_PI);
	dgvi::kinematics::Transformation T_disturb;
	T_disturb.setRandom(1,0.01);
	dgvi::kinematics::Transformation T_WS_init=T_WS*T_disturb; // world to sensor
	dgvi::kinematics::Transformation T_SC; // sensor to camera
	T_SC.setRandom(0.2,M_PI);
	dgvi::ceres::PoseParameterBlock poseParameterBlock(T_WS_init,1,dgvi::Time(0));
	dgvi::ceres::PoseParameterBlock extrinsicsParameterBlock(T_SC,2,dgvi::Time(0));
  problem.AddParameterBlock(poseParameterBlock.parameters(),
                            dgvi::ceres::PoseParameterBlock::Dimension);
  problem.AddParameterBlock(extrinsicsParameterBlock.parameters(),
                            dgvi::ceres::PoseParameterBlock::Dimension);
	problem.SetParameterBlockVariable(poseParameterBlock.parameters()); // optimize this...
  problem.SetParameterBlockConstant(extrinsicsParameterBlock.parameters()); // do not optimize this
	std::cout<<" [ OK ] "<<std::endl;

	// set up a random camera geometry
  std::cout << "set up a random camera geometry... " << std::flush;
  typedef dgvi::cameras::PinholeCamera<dgvi::cameras::NoDistortion>
      DistortedPinholeCameraGeometry;
  std::shared_ptr<const DistortedPinholeCameraGeometry> cameraGeometry =
      std::static_pointer_cast<const DistortedPinholeCameraGeometry>(
        DistortedPinholeCameraGeometry::createTestObject());
  std::cout << " [ OK ] " << std::endl;

	// let's use our own local quaternion perturbation
	std::cout<<"setting local parameterization for pose... "<<std::flush;
  dgvi::ceres::PoseManifold* poseLocalParameterization = new dgvi::ceres::PoseManifold;

  problem.SetManifold(poseParameterBlock.parameters(),poseLocalParameterization);
  problem.SetManifold(extrinsicsParameterBlock.parameters(),poseLocalParameterization);

  Eigen::Matrix<double,7,6,Eigen::RowMajor> J, J_numDiff;
  if(!poseLocalParameterization->verifyJacobianNumDiff(
       poseParameterBlock.parameters(), J.data(), J_numDiff.data())) {
    std::cout<<" [ FAIL ] "<<std::endl;
    DGVI_THROW(std::runtime_error, "manifold check failed")
  }
  if(!poseLocalParameterization->verifyJacobianNumDiff(
       extrinsicsParameterBlock.parameters(), J.data(), J_numDiff.data())) {
    std::cout<<" [ FAIL ] "<<std::endl;
    DGVI_THROW(std::runtime_error, "manifold check failed")
  }

	std::cout<<" [ OK ] "<<std::endl;

	// and the parameterization for points:
  ::ceres::Manifold* homogeneousPointLocalParameterization =
      new dgvi::ceres::HomogeneousPointManifold;

  // get some random points and build error terms -- check Jacobians on the fly
  DGVI_DEFINE_EXCEPTION(Exception, std::runtime_error)
	const size_t N=100;
  std::cout << "create N=" << N
            << " visible points and add respective reprojection error terms... " << std::flush;
	for (size_t i=1; i<100; ++i){

	  Eigen::Vector4d point = cameraGeometry->createRandomVisibleHomogeneousPoint(double(i%10)*3+2.0);
	  dgvi::ceres::HomogeneousPointParameterBlock* homogeneousPointParameterBlock_ptr =
			  new dgvi::ceres::HomogeneousPointParameterBlock(T_WS*T_SC*point,i+2);
    problem.AddParameterBlock(homogeneousPointParameterBlock_ptr->parameters(),
                              dgvi::ceres::HomogeneousPointParameterBlock::Dimension);
	  problem.SetParameterBlockConstant(homogeneousPointParameterBlock_ptr->parameters());
    problem.SetManifold(homogeneousPointParameterBlock_ptr->parameters(),
                        homogeneousPointLocalParameterization);

	  // get a randomized projection
	  Eigen::Vector2d kp;
	  cameraGeometry->projectHomogeneous(point,&kp);
	  kp += Eigen::Vector2d::Random();

	  // Set up the only cost function (also known as residual).
	  Eigen::Matrix2d information=Eigen::Matrix2d::Identity();
    ::ceres::CostFunction* cost_function =
        new dgvi::ceres::ReprojectionError<DistortedPinholeCameraGeometry>(
			  cameraGeometry,1, kp,information);
    auto id = problem.AddResidualBlock(
          cost_function, nullptr, poseParameterBlock.parameters(),
          homogeneousPointParameterBlock_ptr->parameters(), extrinsicsParameterBlock.parameters());

    // check Jacobians
    DGVI_ASSERT_TRUE(
          Exception, dgvi::ceres::jacobiansCorrect(&problem, id), "Jacobian verification failed")

	}
	std::cout<<" [ OK ] "<<std::endl;

	// Run the solver!
	std::cout<<"run the solver... "<<std::endl;
	::ceres::Solver::Options options;
	::FLAGS_stderrthreshold=google::WARNING; // enable console warnings (Jacobian verification)
	::ceres::Solver::Summary summary;
  ::ceres::Solve(options, &problem, &summary);

	// print some infos about the optimization
	std::cout << "initial T_WS : " << T_WS_init.T() << "\n"
			<< "optimized T_WS : " << poseParameterBlock.estimate().T() << "\n"
			<< "correct T_WS : " << T_WS.T() << "\n";

	// make sure it converged
  DGVI_ASSERT_TRUE(Exception,
                    2*(T_WS.q()*poseParameterBlock.estimate().q().inverse()).vec().norm()<1e-2,
                    "quaternions not close enough")
  DGVI_ASSERT_TRUE(Exception,
                    (T_WS.r()-poseParameterBlock.estimate().r()).norm()<1e-1,
                    "translation not close enough")
}
