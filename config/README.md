# DGVI-SLAM configuration

Profiles use OpenCV FileStorage YAML (`%YAML:1.0`). The camera calibration, IMU noise, image timing, feature masks, optimization settings, and active GNSS/DEM values were preserved from the existing profiles. Verify them against the actual sequence and sensor setup before running.

## File migration

| Previous file | Current file |
|---|---|
| `config/okvis2.yaml` | `field/dgvi_slam.yaml` |
| `config/fp_dem.yaml` | `fusionportable/dgvi_slam.yaml` |
| `config/fp_gps.yaml` | `fusionportable/gnss_only.yaml` |
| `config/urbanloco.yaml` | `urbanloco/dgvi_slam.yaml` |

Unused EuRoC, GVINS, Hilti, RealSense, and VBR example profiles were removed. `rviz2/` is retained because the optional ROS launch files reference it. The offline application does not require an RViz or Supereight configuration.

## Parameter groups

| Group | Check before use |
|---|---|
| `cameras` | Intrinsics, distortion, image dimensions, and camera-to-IMU transform `T_SC` |
| `camera_parameters` | Synchronization group, image delay in seconds, extrinsic-calibration switches |
| `imu_parameters` | Noise densities, bias priors, gravity, scale factors, and `T_BS` |
| `frontend_parameters` | Detection/matching thresholds and image-space feature masks |
| `stationary_parameters` | Stationarity thresholds and recovery pause after motion resumes |
| `estimator_parameters` | Window sizes, loop closure, final BA, and optimizer budgets |
| `output_parameters` | Optional runtime displays; submapping is disabled in the supplied profiles |
| `gps_parameters` | Fix type, antenna lever arm, uncertainty scaling, screening, initialization, and recovery |
| `dem_parameters` | DEM enable switch, height uncertainty, sensor height, lever arm, and altitude handling |

## GNSS and recovery

- `r_SA` is the antenna lever arm in the IMU frame, in meters.
- `gps_sigma_scale` scales the reported GNSS uncertainty. It changes the factor weight and should reflect the input uncertainty convention.
- `gps_velocity_sigma` is the horizontal velocity-prior standard deviation in m/s; nonpositive values disable it. `gps_velocity_min_dt` and `gps_velocity_max_dt` bound accepted intervals in seconds.
- `gps_min_init_points` and `gps_max_init_buffer_points` control the persistent initialization correspondence buffer.
- `gps_bounded_recovery_residual_threshold` and `gps_bounded_recovery_exit_threshold` are horizontal distances in meters. `gps_bounded_recovery_consecutive` controls the persistence requirement.
- `gps_bounded_recovery_max_step` bounds the translation applied per GNSS update. `gps_bounded_recovery_max_total <= 0` disables the accumulated-distance cap; the per-update bound still applies.
- `max_h_err`, `max_v_err`, and `gps_max_speed` screen the GNSS input. `min_fix_status` must match the status codes in the converted CSV, not an assumed universal GNSS status convention.

All four profiles explicitly disable legacy reinitialization (`gps_enable_reinit: false`) and legacy position alignment. Seven settings that only tune that disabled path were removed from the profiles and made optional in the parser: `gps_dropout_threshold`, `gps_reinit_position_sigma_scale`, `gps_reinit_position_loss_scale`, `gps_bounded_recovery_apply_in_reinitialising`, `gps_max_correction`, `gps_max_yaw_correction`, and `gps_min_reinit_points`. The library defaults and implementations remain available for users who deliberately re-enable legacy behavior.

## DEM and vertical conventions

`dem_parameters.use` controls both DEM-aided GNSS heights and the independent DEM factors. With `use: false`, DEM paths are ignored by the offline application. With `use: true`, provide at least one readable georeferenced raster and geodetic GNSS input.

| Parameter | Meaning |
|---|---|
| `sigma_h` | DEM height standard deviation in meters; use a positive value |
| `d_above_ground` | Constant sensor/antenna height above terrain for the independent height factor, in meters |
| `r_SA` | Lever arm used by the height factor, in the IMU frame |
| `use_dem_height_for_gps: true` | Replace GNSS altitude with DEM terrain height in the reader; `dem_fusion_alpha` is not used in this mode |
| `use_dem_height_for_gps: false` | Blend GNSS and DEM altitude in the backend using `dem_fusion_alpha` |
| `dem_fusion_alpha` | GNSS weight: `0.0` uses DEM height; `1.0` uses GNSS height; intermediate values blend them |
| `gps_parameters.geoid_model` | GeographicLib geoid grid, currently `egm96-5`; an empty string disables correction |

The independent height factor remains active when DEM is enabled even if `dem_fusion_alpha` is `1.0`. Thus alpha is not the full-system DEM ablation switch.

In the current implementation, DEM replacement/blending uses terrain height directly; `d_above_ground` is applied to the separate DEM height factor. This behavior is preserved. The DEM-to-global height relationship uses the manuscript's vertical-datum approximation. Check the raster datum and GNSS altitude convention before choosing the geoid correction or height offset. The reader logs a warning and disables geoid correction if the requested GeographicLib grid is unavailable; check the log before evaluating results. Missing raster coverage causes individual DEM queries to be skipped.

## Comparing profiles

The two FusionPortable profiles differ in `mask_rects`, `num_keyframes` (7 versus 11), and `num_imu_frames` (7 versus 15), in addition to DEM settings. They preserve existing experiments and should not be treated as a controlled DEM-only comparison. For that comparison, duplicate one profile and change only `dem_parameters.use`.

These profiles do not encode every sequence-specific setting reported in the paper. The run wrapper saves the actual YAML used for each invocation, which should accompany any new evaluation results.
