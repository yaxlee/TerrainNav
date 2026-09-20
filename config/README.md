# DGVI-SLAM configuration

Profiles use OpenCV FileStorage YAML (`%YAML:1.0`). The three included profiles are `field.yaml` (self-collected), `fp.yaml` (FusionPortableV2), and `urbanloco.yaml` (UrbanLoco). Each uses a monocular camera, IMU, and geodetic GNSS, with stationarity detection enabled. The files share two-space indentation and parameter-group ordering, with 4-by-4 transforms displayed across four rows. Calibration values and estimator settings retain their supplied values; `slam_use: dgvi` matches the current parser. No profile enables DEM.

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

All three profiles enable bounded recovery and disable legacy reinitialization and legacy position alignment.

## DEM and vertical conventions

`dem_parameters.use` controls both DEM-aided GNSS heights and the independent DEM factors. With `use: false` or no `dem_parameters` group, the reader ignores DEM data. With `use: true`, it automatically loads georeferenced `.tif`, `.tiff`, and `.vrt` files directly inside `mav0/dem0/`; extension matching is case-insensitive and rasters are tried in filename order. Geodetic GNSS input is required.

The standard reader accepts either the sequence directory containing `mav0/` or `mav0/` itself. For datasets whose sensor folders are directly in the supplied directory, it looks for `dem0/` there. No additional DEM path argument is needed:

```yaml
dem_parameters:
  use: true
```

Set the height uncertainty, sensor height, and lever arm below according to your sensor setup. If DEM is enabled but the directory is missing, contains no supported raster files, or none can be loaded, startup fails with an error. Explicit raster paths on the command line override automatic discovery and retain their supplied order.

| Parameter | Meaning |
|---|---|
| `sigma_h` | DEM height standard deviation in meters; use a positive value |
| `d_above_ground` | Constant sensor/antenna height above terrain for the independent height factor, in meters |
| `r_SA` | Lever arm used by the height factor, in the IMU frame |
| `use_dem_height_for_gps: true` | Replace GNSS altitude with DEM terrain height in the reader; `dem_fusion_alpha` is not used in this mode |
| `gps_parameters.geoid_model` | GeographicLib geoid grid, currently `egm96-5`; an empty string disables correction |
