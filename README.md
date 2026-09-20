# DGVI-SLAM

**DEM-Aided Visual–Inertial SLAM with Bounded Position-Fix GNSS Recovery in Variable-Elevation Urban Environments**

DGVI-SLAM combines camera and IMU measurements, position-fix GNSS, and a georeferenced digital elevation model (DEM). It extends the upstream visual–inertial backend and asynchronous GNSS factors with persistent global initialization, bounded trajectory recovery, and terrain-height constraints.

![DGVI-SLAM pipeline](docs/images/pipeline.png)

*System overview from Fig. 2 of the accompanying manuscript. Accepted GNSS fixes initialize the global frame; GNSS and DEM constraints support sliding-window estimation, with bounded recovery when GNSS and VIO positions persistently disagree.*

## What the system does

- Screens GNSS fixes using receiver status, reported uncertainty, and horizontal speed.
- Initializes the global reference from persistent GNSS–VIO position correspondences.
- Combines asynchronous GNSS position factors with horizontal-velocity regularization.
- Queries DEM tiles at estimated global positions and adds height constraints after global alignment is fixed.
- Applies bounded translations to active poses and landmarks during sustained GNSS–VIO disagreement.

We evaluate five public and six field sequences. On the field sequences, adding DEM reduces the median SE(3)-aligned position RMSE from 6.17 to 4.32 m (30.0%), relative to offline LIO-SAM reference trajectories.

## Build

```bash
cd /path/to/TerrainNav
git submodule update --init --recursive
```

On Ubuntu 22.04, the native dependencies are:

```bash
sudo apt update
sudo apt install build-essential cmake git libgoogle-glog-dev libgflags-dev \
  libatlas-base-dev libeigen3-dev libsuitesparse-dev \
  libboost-dev libboost-filesystem-dev libopencv-dev \
  libgeographic-dev libgdal-dev libpcl-dev
```

Use a CMake version compatible with the pinned Ceres submodule. The project uses C++17. The upstream build instructions use `libgeographiclib-dev` instead of `libgeographic-dev` on Ubuntu 24.04.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DUSE_NN=OFF -DUSE_GPU=OFF
cmake --build build --target dgvi_slam_app -j4
```

## Prepare input data

Use the converted dataset layout expected by `DatasetReader`:

| Path | Contents |
|---|---|
| `cam0/data.csv` | Image Data List |
| `cam0/data/` | Images Data |
| `imu0/data.csv` | IMU Data |
| `gps0/data.csv` | GNSS Data |
| `dem0/.tif` | DEM Data |

DEM rasters are supplied separately as GeoTIFF (`.tif`/`.tiff`). See [configuration guidance](config/README.md).

## Run

The executable is `dgvi_slam_app`. It accepts a dataset in the layout above,
an optional output directory, and optional DEM raster paths. `-rpg` selects the
inherited RPG reader (without DEM).

| Included profile | Purpose |
|---|---|
| [Field](config/field.yaml) | Self-collected monocular visual-inertial data with geodetic GNSS |
| [FusionPortableV2](config/fp.yaml) | FusionPortableV2 monocular visual-inertial data with geodetic GNSS |
| [UrbanLoco](config/urbanloco.yaml) | UrbanLoco monocular visual-inertial data with geodetic GNSS |

```bash
./build/dgvi_slam_app config/field.yaml /path/to/sequence results/field

# Camera + IMU + geodetic GNSS + DEM, using your calibrated configuration
./build/dgvi_slam_app /path/to/dgvi_slam.yaml \
  /path/to/sequence results/field /path/to/dem.tif
```

All three included profiles use geodetic GNSS and include stationarity settings.
Their calibration and estimator parameters are specific to each sensor setup.
DEM is not enabled in these profiles. See [configuration guidance](config/README.md).

The repository builds the standalone dataset application in `dgvi/app/` and its
supporting libraries in `dgvi/`. The shared estimator depends on `dgvi/mapping` and
`supereight2`. `USE_NN=ON` optionally enables frontend keypoint classification
using `resources/fast-scnn.pt` and requires LibTorch. Regression tests can be
enabled with `-DBUILD_TESTS=ON`.

## Source layout

```text
dgvi/
  app/src/                  Dataset application entry point
  ceres/                    Optimization backend and residuals
  common/                   Parameters and shared interfaces
  cv/                       Camera models and frames
  frontend/                 Feature matching and loop closure
  kinematics/               Transformations
  mapping/                  Mapping support
  multisensor_processing/   Dataset readers and SLAM orchestration
  time/                     Time and duration types
  timing/                   Performance timers
  util/                     Shared utilities
```

## Outputs

For SLAM mode, output filenames start with `dgvi-slam-slam`; disabling loop closure changes the mode to `vio`.

| Suffix | Contents |
|---|---|
| `_trajectory.csv` | Streamed optimized states in the local world frame |
| `-final_trajectory.csv` | Final local trajectory |
| `-global-final_trajectory.csv` | Global-frame trajectory when GNSS is configured and global output is available |
| `-final-ba_trajectory.csv` | Export after the optional final-BA stage; the filename can be written even when BA is disabled |
| `-global-final-ba_trajectory.csv` | Corresponding global export |
| `-final_map.csv` | Landmark map when final BA and map saving are enabled |

## Self-collected Dataset
Our self-collected datasets are available in anonymous huggingface repository: https://anonymous-hf.com/a/4vgrxcfti2mp/.
