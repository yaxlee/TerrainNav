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

We evaluate five public and six self-collected sequences. On the self-collected sequences, adding DEM reduces the median SE(3)-aligned position RMSE from 6.17 to 4.32 m (30.0%), relative to offline LIO-SAM reference trajectories.

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

Use the converted dataset layout expected by `DatasetReader`. Paths below are
relative to `mav0/` (or the supplied dataset directory when it directly contains
the sensor folders):

| Path | Contents |
|---|---|
| `cam0/data.csv` | Image Data List |
| `cam0/data/` | Images Data |
| `imu0/data.csv` | IMU Data |
| `gps0/data.csv` | GNSS Data |
| `dem0/*.tif`, `dem0/*.tiff`, `dem0/*.vrt` | Georeferenced DEM rasters |

With `dem_parameters.use: true`, DEM rasters are loaded automatically from `dem0/`.

## Run

| Included profile | Purpose |
|---|---|
| [Field](config/field.yaml) | Self-collected monocular visual-inertial data with geodetic GNSS |
| [FusionPortableV2](config/fp.yaml) | FusionPortableV2 monocular visual-inertial data with geodetic GNSS |
| [UrbanLoco](config/urbanloco.yaml) | UrbanLoco monocular visual-inertial data with geodetic GNSS |

```bash
./build/dgvi_slam_app config/field.yaml /path/to/sequence results/field

# Enable dem_parameters.use and place rasters in the sequence's mav0/dem0/
./build/dgvi_slam_app /path/to/field.yaml /path/to/self-collected-sequence /path/to/result
```

See [configuration guidance](config/README.md).

## Self-collected Dataset
Our self-collected datasets are available in anonymous huggingface repository: https://anonymous-hf.com/a/4vgrxcfti2mp/.
