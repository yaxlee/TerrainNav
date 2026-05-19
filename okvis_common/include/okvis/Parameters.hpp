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
 * @file Parameters.hpp
 * @brief This file contains struct definitions that encapsulate parameters and settings.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_PARAMETERS_HPP_
#define INCLUDE_OKVIS_PARAMETERS_HPP_

#include <set>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/core.hpp>
#pragma GCC diagnostic pop
#include <Eigen/Dense>
#include <okvis/Time.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <optional>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief Struct that contains all the camera calibration information.
struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  okvis::kinematics::Transformation T_SC;   ///< Transformation from camera to sensor (IMU) frame.
  Eigen::Vector2i imageDimension;           ///< Image dimension. [pixels]
  Eigen::VectorXd distortionCoefficients;   ///< Distortion Coefficients.
  Eigen::Vector2d focalLength;              ///< Focal length.
  Eigen::Vector2d principalPoint;           ///< Principal point.
  std::string cameraModel;                  ///< camera model. ('pinhole' 'eucm')
  Eigen::Vector2d eucmParameters;           ///< alpha, beta eucm parameters

  /// \brief Distortion type. ('radialtangential' 'radialtangential8' 'equdistant')
  std::string distortionType;

  cameras::NCameraSystem::CameraType cameraType; ///< Some additional info about the camera.
};

/*!
 * \brief Camera parameters.
 *
 * A simple struct to specify properties of a Camera.
 *
 */
struct CameraParameters{
  double timestamp_tolerance; ///< Stereo frame out-of-sync tolerance. [s]
  std::set<size_t> sync_cameras; ///< The cameras that will be synchronised.
  std::vector<size_t> stereo_indices; ///< camera indices for the left and right camera for the stereo network

  /// \brief Image timestamp error. [s] timestamp_camera_correct = timestamp_camera - image_delay.
  double image_delay;

  /**
   * @brief Some parameters to set the online calibrator.
   */
  struct OnlineCalibrationParameters {
    bool do_extrinsics; ///< Do we online-calibrate extrinsics?
    bool do_extrinsics_final_ba; ///< Do we calibrate extrinsics in final BA?
    double sigma_r; ///< T_SCi position prior stdev [m]
    double sigma_alpha; ///< T_SCi orientation prior stdev [rad]
    double sigma_r_final_ba; ///< T_SCi position prior stdev in final BA [m]
    double sigma_alpha_final_ba; ///< T_SCi orientation prior stdev in final BA [rad]
  };

  OnlineCalibrationParameters online_calibration; ///< Online calibration parameters.
};

/*!
 * \brief IMU parameters.
 *
 * A simple struct to specify properties of an IMU.
 *
 */
struct ImuParameters{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool use; ///< Use the IMU at all?
  okvis::kinematics::Transformation T_BS; ///< Transform from Body frame to IMU (sensor frame S).
  double a_max;  ///< Accelerometer saturation. [m/s^2]
  double g_max;  ///< Gyroscope saturation. [rad/s]
  double sigma_g_c;  ///< Gyroscope noise density.
  double sigma_bg;  ///< Initial gyroscope bias.
  double sigma_a_c;  ///< Accelerometer noise density.
  double sigma_ba;  ///< Initial accelerometer bias
  double sigma_gw_c; ///< Gyroscope drift noise density.
  double sigma_aw_c; ///< Accelerometer drift noise density.
  Eigen::Vector3d g0;  ///< Mean of the prior gyro bias.
  Eigen::Vector3d a0;  ///< Mean of the prior accelerometer bias.
  double g;  ///< Earth acceleration.
  Eigen::Vector3d s_a; ///< Scale factor for accelerometer measurements
};

/**
 * @brief Parameters for detection etc.
 */
struct FrontendParameters {
  double detection_threshold; ///< Detection threshold. By default the uniformity radius in pixels.
  double absolute_threshold; ///< Absolute Harris corner threshold (noise floor).
  double matching_threshold; ///< BRISK descriptor matching threshold.
  int octaves; ///< Number of octaves for detection. 0 means single-scale at highest resolution.
  int max_num_keypoints; ///< Restrict to a maximum of this many keypoints per img (strongest ones).
  double keyframe_overlap; ///< Minimum field-of-view overlap.
  bool use_cnn; ///< Use the CNN (if available) to filter out dynamic content / sky.
  bool parallelise_detection; ///< Run parallel detect & describe.
  int num_matching_threads; ///< Parallelise matching with this number of threads.
  /// Rectangular regions (x, y, width, height) excluded from feature detection.
  /// Applied to all cameras. Useful to mask out vehicle body parts visible in the image.
  std::vector<cv::Rect> mask_rects;
};

/**
 * @brief Parameters regarding the estimator.
 */
struct EstimatorParameters {
  int num_keyframes; ///< Number of keyframes in optimisation window.
  int num_loop_closure_frames; ///< Number of loop closure frames in optimisation window.
  int num_imu_frames; ///< Number of frames linked by most recent nonlinear IMU error terms.
  bool do_loop_closures; ///< Whether to do VI-SLAM or VIO.
  bool do_final_ba; ///< Whether to run a final full BA.
  bool enforce_realtime; ///< Whether to limit the time budget for optimisation.
  int realtime_min_iterations; ///< Minimum number of iterations always performed.
  int realtime_max_iterations; ///< Never do more than these, even if not converged.
  double realtime_time_limit; ///< Time budget for realtime optimisation. [s]
  int realtime_num_threads; ///< Number of threads for the realtime optimisation.
  int full_graph_iterations; ///< Don't do more than these for the full (background) optimisation.
  int full_graph_num_threads; ///< Number of threads for the full (background) optimisation.
  double p_dbow; ///< Match threshold for dBoW -- unfortunately this varies with setups.
  double drift_percentage_heuristic; ///< % allowed drift in loop closures rel. to dist. travelled.
};

/**
 * @brief Some options for how and what to output.
 */
struct OutputParameters {
    bool display_topview; ///< Displays top view (Non-causal part might be slow).
    bool display_matches; ///< Displays debug video and matches. May be slow.
    bool display_overhead; ///< Debug overhead image. Is slow.
    bool enable_submapping; //< Whether or not is submapping enabled
};
/**
  * @brief Struct to specify parameters of GPS sensor
  */
struct GpsParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string type; ///< Format of GPS data: "cartesian" | "geodetic" | "geodetic-leica":
    Eigen::Vector3d r_SA; ///< Translation IMU sensor to GPS antenna; known from calibration
    double yawErrorThreshold; /// < Threshold on maximum estimated yaw error [degree] for initialization
    bool robustGpsInit; /// < Flag if robust initialization is needed (low-grade GPS sensor)
    double gpsSigmaScale; ///< Multiplier on GPS sigmas inside the optimizer (>1 reduces weight, slows correction)
    double gpsLossScale; ///< Cauchy robust loss scale for GPS factors [m]; <=0 disables robust loss.
    double gpsOutlierScale; ///< Multiplier on the 3-sigma outlier rejection threshold (>1 relaxes rejection, useful for fast platforms with tight reported sigmas)
    double gpsDropoutThreshold; ///< Minimum VIO time [s] since last GPS state before re-init is triggered (prevents false re-init after GPS loop closure)
    bool gpsEnableReInit; ///< Enable post-initialisation GPS T_GW re-initialisation after dropout.
    double gpsVelocitySigma; ///< Horizontal GPS-derived velocity prior sigma [m/s] (<=0 disables).
    double gpsVelocityMinDt; ///< Minimum GPS time delta [s] for velocity prior.
    double gpsVelocityMaxDt; ///< Maximum GPS time delta [s] for velocity prior.
    double gpsReinitPositionSigmaScale; ///< Additional sigma scale for weak GPS position factors during ReInitialising (<=0 disables).
    double gpsReinitPositionLossScale; ///< Cauchy loss scale for weak ReInitialising GPS position factors [m]; <=0 disables robust loss.
    bool gpsEnableLegacyPositionAlignment; ///< Enable old dropout/re-init position-only alignment path.
    bool gpsBoundedRecoveryEnabled; ///< Enable small bounded GPS residual corrections on the local window.
    double gpsBoundedRecoveryResidualThreshold; ///< Horizontal GPS/VIO residual [m] above which bounded recovery can start.
    double gpsBoundedRecoveryExitThreshold; ///< Horizontal residual [m] below which bounded recovery state is reset.
    int gpsBoundedRecoveryConsecutive; ///< Number of consecutive large residuals before applying bounded correction.
    double gpsBoundedRecoveryMaxStep; ///< Maximum local-window correction per GPS update [m].
    double gpsBoundedRecoveryMaxTotal; ///< Maximum accumulated bounded correction distance before residual recovers [m]; <=0 disables the cap.
    bool gpsBoundedRecoveryHorizontalOnly; ///< Apply only horizontal bounded corrections.
    bool gpsBoundedRecoveryApplyInInitialised; ///< Allow bounded correction in normal GPS Initialised mode.
    bool gpsBoundedRecoveryApplyInReInitialising; ///< Allow bounded correction during GPS ReInitialising mode.
    double gpsBoundedRecoveryStationarySpeedThreshold; ///< Skip bounded recovery when GPS horizontal speed is below this [m/s]; <=0 disables.
    double gpsMaxCorrection; ///< Maximum allowed T_GW translation correction [m] for GPS loop closure; larger corrections are rejected as likely bad Umeyama estimates
    double gpsMaxYawCorrection; ///< Maximum allowed T_GW yaw correction [deg] for GPS loop closure (<=0 disables).
    int gpsMinInitPoints; ///< Minimum number of GPS points required for Umeyama alignment (higher = more robust, especially during sharp turns)
    int gpsMinReInitPoints; ///< Minimum GPS points required for re-initialization after GPS dropout.
    double gpsMaxSpeed; ///< Maximum accepted GPS horizontal speed [m/s] between consecutive accepted reader-side measurements (<=0 disables)

    double maxHErr; ///< Maximum horizontal error [m] for GPS bad-point filtering (reader-side)
    double maxVErr; ///< Maximum vertical error [m] for GPS bad-point filtering (reader-side)
    int minFixStatus; ///< Minimum fix_status to accept (0=no filter, 1=reject status 0, 2=require RTK)
    std::string geoidModel; ///< GeographicLib geoid model name for undulation correction (e.g. "egm96-5")

    /// Default Constructor (no GPS)
    GpsParameters() : type("none"), r_SA(Eigen::Vector3d(0., 0., 0.)),
                      yawErrorThreshold(0.), robustGpsInit(false),
                      gpsSigmaScale(1.0), gpsLossScale(3.0), gpsOutlierScale(1.0),
                      gpsDropoutThreshold(3.0),
                      gpsEnableReInit(true),
                      gpsVelocitySigma(0.0),
                      gpsVelocityMinDt(0.2),
                      gpsVelocityMaxDt(5.0),
                      gpsReinitPositionSigmaScale(5.0),
                      gpsReinitPositionLossScale(15.0),
                      gpsEnableLegacyPositionAlignment(false),
                      gpsBoundedRecoveryEnabled(false),
                      gpsBoundedRecoveryResidualThreshold(8.0),
                      gpsBoundedRecoveryExitThreshold(3.0),
                      gpsBoundedRecoveryConsecutive(2),
                      gpsBoundedRecoveryMaxStep(0.75),
                      gpsBoundedRecoveryMaxTotal(30.0),
                      gpsBoundedRecoveryHorizontalOnly(true),
                      gpsBoundedRecoveryApplyInInitialised(true),
                      gpsBoundedRecoveryApplyInReInitialising(true),
                      gpsBoundedRecoveryStationarySpeedThreshold(0.0),
                      gpsMaxCorrection(50.0), gpsMaxYawCorrection(0.0),
                      gpsMinInitPoints(10),
                      gpsMinReInitPoints(10),
                      gpsMaxSpeed(1e9),
                      maxHErr(1e9), maxVErr(1e9), minFixStatus(0), geoidModel("")
                      {}
};


/**
 * @brief Parameters for DEM-based height constraints.
 */
struct DemParameters {
  bool use = false;                  ///< Enable DEM height factors.
  double sigma_h = 2.0;             ///< Height constraint uncertainty (1-sigma) [m].
  double d_above_ground = 0.0;      ///< Sensor height above ground [m] (constant, e.g. for ground vehicles).
  Eigen::Vector3d r_SA = Eigen::Vector3d::Zero(); ///< Sensor-to-body offset in IMU frame.
  bool useDemHeightForGps = false;  ///< Fuse GPS altitude with DEM height (requires geodetic data_type).
  double demFusionAlpha = 0.0;      ///< GPS weight in altitude fusion: 0=full DEM, 1=full GPS, 0.5=equal blend.
};

/// @brief  Struct to specify the parameters of a LiDAR sensor
struct LidarParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    okvis::kinematics::Transformation T_SL;   ///< Transformation from LiDAR to sensor (IMU) frame.
    float elevation_resolution_angle; ///< Resolution angle for the elevation of the LiDAR sensor
    float azimuth_resolution_angle; ///< Resolution angle for the azimuth of the LiDAR sensor
};


/// @brief Struct to combine all parameters and settings.
/**
 * @brief Parameters for visual stationarity detection and constraints.
 */
struct StationaryParameters {
  bool   enabled                       = true;  ///< Enable visual stationarity constraints.
  int    entry_frames                  = 5;     ///< Consecutive visually-stationary frames required to enter.
  int    exit_frames                   = 3;     ///< Consecutive visually-moving frames required to exit.
  int    min_landmarks                 = 20;    ///< Minimum repeated 3D landmarks for a valid decision.
  double max_median_pixel_displacement = 0.45;  ///< [px] Median repeated-landmark displacement threshold.
  double max_mean_pixel_displacement   = 0.90;  ///< [px] Mean repeated-landmark displacement threshold.
  double sigma_v                       = 0.02;  ///< [m/s] Zero-velocity constraint 1-sigma.
  double sigma_position                = 0.03;  ///< [m] Relative no-motion position constraint 1-sigma.
  double sigma_orientation             = 0.01;  ///< [rad] Relative no-motion orientation constraint 1-sigma.
  int    gps_recovery_pause_after_exit_frames = 0; ///< Keep GPS bounded recovery paused for this many frames after stationary exit.
};

struct ViParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  okvis::cameras::NCameraSystem nCameraSystem;  ///< Camera extrinsics and intrinsics.
  CameraParameters camera; ///< Camera parameters.
  ImuParameters imu; ///< Imu parameters.
  std::optional<GpsParameters> gps; ///< Gps parameters.
  std::optional<DemParameters> dem; ///< DEM height constraint parameters.
  std::optional<LidarParameters> lidar; ///< LiDAR parameters
  FrontendParameters frontend; ///< Frontend parameters.
  EstimatorParameters estimator; ///< Estimator parameters.
  StationaryParameters stationary; ///< Visual stationarity parameters.
  OutputParameters output; ///< Output parameters.
  CameraCalibration rgb;  ///< RGB parameters.
};

} // namespace okvis

#endif // INCLUDE_OKVIS_PARAMETERS_HPP_
