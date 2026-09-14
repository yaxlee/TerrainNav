# DGVI-SLAM

**DEM-Aided Visual–Inertial SLAM with Bounded Position-Fix GNSS Recovery in Variable-Elevation Urban Environments**

DGVI-SLAM combines camera and IMU measurements, position-fix GNSS, and a georeferenced digital elevation model (DEM). It builds on [OKVIS2-X](https://github.com/ethz-mrl/OKVIS2-X), retaining its visual–inertial backend and asynchronous GNSS factors while adding persistent global initialization, bounded trajectory recovery, and terrain-height constraints.

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
git clone
cd DGVI-SLAM
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
  -DBUILD_ROS2=OFF -DHAVE_LIBREALSENSE=OFF \
  -DUSE_NN=OFF -DUSE_GPU=OFF -DBUILD_LEGACY_APPS=OFF
cmake --build build --target dgvi_slam_app -j4
```

## Prepare input data

Use the converted dataset layout expected by `DatasetReader`:

| Path | Contents |
|---|---|
| `cam0/data.csv` | Header, then `timestamp_ns,filename` |
| `cam0/data/` | Images referenced by the camera CSV |
| `imu0/data.csv` | Header, then `timestamp_ns,wx,wy,wz,ax,ay,az` |
| `gps0/data.csv` | Geodetic GNSS records described below |
| `dem0/.tif` | DEM |

DEM rasters are supplied separately as GeoTIFF (`.tif`/`.tiff`) or GDAL VRT files. Rasters must contain usable georeferencing and cover the route. Multiple rasters are queried in command-line order, using the first valid height. GNSS altitude, DEM height, sensor height, and the selected geoid model must follow a consistent vertical convention. See [configuration guidance](config/README.md).

## Run

Choose a profile and verify its calibration and acquisition settings:

| Profile | Purpose |
|---|---|
| [Field](config/field/dgvi_slam.yaml) | Field-system profile|
| [FusionPortableV2 + DEM](config/fusionportable/dgvi_slam.yaml) | FusionPortableV2 profile |
| [UrbanLoco](config/urbanloco/dgvi_slam.yaml) | UrbanLoco profile |


```bash
# Camera + IMU + GNSS + one or more DEM tiles
./build/dgvi_slam_app config/field/dgvi_slam.yaml \
  /path/to/sequence results/field /path/to/dem.tif
```

## Outputs and evaluation

For SLAM mode, output filenames start with `dgvi-slam-slam`; disabling loop closure changes the mode to `vio`.

| Suffix | Contents |
|---|---|
| `_trajectory.csv` | Streamed optimized states in the local world frame |
| `-final_trajectory.csv` | Final local trajectory |
| `-global-final_trajectory.csv` | Global-frame trajectory when GNSS is configured and global output is available |
| `-final-ba_trajectory.csv` | Export after the optional final-BA stage; the filename can be written even when BA is disabled |
| `-global-final-ba_trajectory.csv` | Corresponding global export |
| `-final_map.csv` | Landmark map when final BA and map saving are enabled |

The paper uses independent INS ground truth for FusionPortableV2, GNSS-fix consistency for UrbanLoco, and offline LiDAR–inertial reference trajectories for the field sequences. 
## Code map

| Location | Responsibility |
|---|---|
| `okvis_apps/src/dgvi_slam_app.cpp` | Offline application, sensor callbacks, DEM setup, trajectory output |
| `okvis_multisensor_processing/src/DatasetReader.cpp` | Input parsing, GNSS screening, geoid handling, raster queries |
| `okvis_ceres/src/ViGraph.cpp` | GNSS factors, global initialization, bounded recovery |
| `okvis_ceres/src/ViSlamBackend.cpp` | Backend coordination, height blending, DEM-factor refresh |
| `okvis_ceres/src/DemHeightError.cpp` | DEM height residual and Jacobians |
| `okvis_common/src/ViParametersReader.cpp` | Configuration parser |
| `config/` | Dataset profiles and parameter guidance |

The `okvis` C++ namespace, library targets, source directories, and upstream copyright notices are retained to preserve provenance and library compatibility. The CMake project and ROS package are named `dgvi_slam`. Inherited ROS node executable names and topic namespaces remain unchanged.
