# DGVI-SLAM 代码审查与清理明细

审查基线：`yaxlee/Terrian-SLAM` 的 `main`，提交 `25e41033456e9bda519cb34478de79793405a95b`。论文依据：用户提供的 `ICRA2027 (3).pdf`。本次修改作为独立分支提交，不直接修改主分支。

## 已完成的修改

| 范围 | 修改 | 原因与行为变化 |
|---|---|---|
| 项目命名 | CMake project、ROS package 改为 `dgvi_slam`；Doxygen 标题改为 DGVI-SLAM | 统一项目身份 |
| 主入口 | `okvis_app_synchronous.cpp` → `dgvi_slam_app.cpp`，可执行程序 `dgvi_slam_app` | 明确论文对应的离线 GNSS/DEM 入口 |
| 输出 | 主入口的轨迹/地图文件前缀改为 `dgvi-slam-`；默认输出目录改为当前目录的 `results` | 避免默认将结果写入数据集目录；现有后处理命令需更新文件名 |
| 默认构建 | ROS 2、RealSense、神经网络/GPU 默认关闭；增加 `BUILD_LEGACY_APPS=OFF` | 默认只构建论文使用的离线应用；继承的可选应用仍可显式开启 |
| 安装 | 提前加载 GNUInstallDirs；修正 ROS 头文件安装路径；将词袋文件安装至主程序目录 | 修正已有安装路径问题，保证主程序安装后可找到词袋 |
| 配置 | 4 份主配置按数据集重新组织；25 份 YAML 收敛为 4 份 | 移除与本文数据集无关的示例、SE2 参数和旧实验注释 |
| DEM 开关 | 主入口和 DatasetReader 均检查 `dem_parameters.use` | `false` 不再加载 DEM、替换/融合 GNSS 高度或注册 DEM 因子 |
| DEM 输入 | 启用 DEM 时要求 geodetic 输入、普通 DatasetReader 和 DEM 路径；所有栅格均加载失败则报错 | 防止请求 DEM 的运行静默退化为未加载 DEM |
| 参数解析 | DEM 高度与融合参数接受整数和浮点数；7 个旧重初始化参数改为可选 | 解决整数值被忽略的问题，并允许清理被禁用路径的参数 |
| 运行脚本 | 重写 `scripts/run_and_log.sh`，使用正确主入口 | 保存唯一运行目录、配置、命令、日志与退出码；保留程序/日志管道错误 |
| 轨迹转换 | 精简 `tools/convert_to_tum.py` | 去掉 OpenCV/NumPy 依赖，保留纳秒精度，验证输入并避免覆盖已有输出 |
| 文档 | 重写 README，新增配置说明、本明细；提取论文 Fig. 2 至 `docs/images/pipeline.png` | 说明项目贡献、构建、数据格式、DEM 约定、运行、输出和引用 |
| 忽略规则 | 统一数据集、结果、缓存和权重忽略项，允许文档 PNG | 防止再次混入实验产物 |
| ROS 兼容 | 更新 launch 文件中的 package/resource 路径；删除指向未构建节点的旧 synchronous launch | 与 `dgvi_slam` 包名一致 |

## 配置迁移

| 旧路径 | 新路径 |
|---|---|
| `config/okvis2.yaml` | `config/field/dgvi_slam.yaml` |
| `config/fp_dem.yaml` | `config/fusionportable/dgvi_slam.yaml` |
| `config/fp_gps.yaml` | `config/fusionportable/gnss_only.yaml` |
| `config/urbanloco.yaml` | `config/urbanloco/dgvi_slam.yaml` |

四份配置的有效字段已与基线逐项对比：标定、IMU 噪声、图像延迟、feature mask、窗口大小和启用的 GNSS/DEM 数值一致。GNSS-only 配置新增显式 `dem_parameters.use: false`；`dem_fusion_alpha: 0` 规范为 `0.0`。原配置里的 5 m 单次恢复上限被保留，本次清理没有重新调参。

移除的 7 个旧路径参数为：`gps_dropout_threshold`、`gps_reinit_position_sigma_scale`、`gps_reinit_position_loss_scale`、`gps_bounded_recovery_apply_in_reinitialising`、`gps_max_correction`、`gps_max_yaw_correction`、`gps_min_reinit_points`。四份配置都显式关闭旧重初始化；解析器保留这些字段及其默认值，便于有意启用该功能的用户使用。

## 审查发现

1. **DEM 关闭开关没有完整生效，已修复。** 原主入口只检查 DEM 参数对象存在和路径非空，DatasetReader 同样未检查 `use`。设置 `use: false` 并传入栅格仍可能启用 DEM 辅助。
2. **整数型 DEM 参数被静默忽略，已修复。** 原解析器仅接受 `isReal()`。整数形式的高度、噪声和融合系数会保持默认值，现在同时接受 `isInt()`。
3. **原运行脚本调用错误入口并可能吞掉失败，已修复。** 旧脚本需要 Supereight 配置并运行稠密建图应用；缺少 `pipefail` 时，程序失败但 `tee` 成功会被误报为成功。
4. **FusionPortable 的两份配置不是单变量 DEM 消融，已在文档明确。** 两者还存在 mask 和窗口尺寸差异。没有擅自覆盖这些实验参数。新消融应复制同一份配置，仅切换 DEM 开关。
5. **当前 main 没有跟踪原始 bag、DEM 栅格或轨迹结果 CSV。** 因此未声称删除不存在的结果文件。主要删除项是旧演示/转换脚本、旧论文草稿、未使用配置和约 9.77 MB 的上游展示 GIF；Git 历史中的文件仍可恢复，历史仓库体积不会因此立即缩小。
6. **高度约定需要随数据说明。** 保留现有 terrain height 替换/融合与独立 sensor-height 因子的行为；README 说明 geoid 网格缺失时的日志与回退，以及 DEM datum approximation。没有将本次仓库清理描述为算法重新验证。

## 有意保留的部分

- `okvis` 命名空间、库名、头文件路径和版权声明，保留上游来源和链接兼容性。
- `okvis_mapping`、Supereight2 及其 pinned submodules：它们仍是当前后端构建依赖，不能仅因稠密建图默认关闭而删除。
- 词袋、模型权重、ROS mesh 和 RViz 配置：仍被保留的构建/可选运行路径引用。
- GNSS 因子、初始化、bounded recovery 和 DEM residual/Jacobian 的数值实现：本次没有重写估计器。
- 原始单元测试及上游贡献者和 LICENSE 文件。

## 验证结果与边界

已完成：

- 读取的 289 份文本源码与 GitHub blob SHA 全部一致。
- 四份主配置的有效值对照、必要 GNSS 键完整性检查通过。
- Bash 语法、成功/失败退出码、带空格路径、配置与日志保存测试通过。
- TUM 转换的时间精度、位姿顺序、非法/非有限值拒绝与防覆盖测试通过。
- README 相对链接、CMake 文件引用、ROS XML 与包名一致性、Git whitespace 检查。
- 论文 Fig. 2 提取图已目视检查，内容完整清晰。

新增 `TestDemConfiguration.cpp`，覆盖四份配置解析、整数 DEM 参数以及关闭/开启 DEM 时无效栅格的行为。这些 C++ 回归测试尚未运行。

当前执行环境没有 `cmake`，也没有完整 C++ SLAM 依赖、子模块检出和测试数据。配置尝试在 `cmake: command not found` 处终止。因此**未完成 C++ 编译、CTest 或真实序列精度/运行时间回归**，不把静态检查当作编译通过。建议在已有开发环境合并前运行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON \
  -DBUILD_ROS2=OFF -DHAVE_LIBREALSENSE=OFF -DUSE_NN=OFF -DUSE_GPU=OFF
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

## 仓库名称剩余操作

代码、构建入口、输出和文档已使用 DGVI-SLAM。当前 GitHub 连接未提供 repository rename / settings 写入接口，GitHub 仓库本身仍叫 `Terrian-SLAM`。需要仓库管理员在 Settings → General → Repository name 中改为 `DGVI-SLAM`；之后可同步更新 README clone URL。没有更改仓库可见性或直接合并主分支。

## 删除与迁移的完整路径清单

以下路径从新版本树移除；配置和主入口的替代位置见上文。均可从基线提交恢复。

- `config/okvis2.yaml`
- `config/fp_dem.yaml`
- `config/fp_gps.yaml`
- `config/urbanloco.yaml`
- `config/rsD455/se2.yaml`
- `config/rsD455/okvis2.yaml`
- `config/hilti22/se2.yaml`
- `config/hilti22/se2-lidar.yaml`
- `config/hilti22/okvis2-postcalib.yaml`
- `config/hilti22/okvis2-lidar.yaml`
- `config/hilti22/okvis2.yaml`
- `config/vbr/se2-lidar-handheld.yaml`
- `config/vbr/se2.yaml`
- `config/vbr/se2-lidar-driving.yaml`
- `config/vbr/okvis2_diy.yaml`
- `config/vbr/okvis2-lidar-driving.yaml`
- `config/vbr/se2-lidar-handheld-spagna.yaml`
- `config/vbr/se2-driving.yaml`
- `config/vbr/se2-lidar-driving-ciamp0.yaml`
- `config/vbr/okvis2.yaml`
- `config/vbr/okvis2-lidar-handheld.yaml`
- `config/gvins/okvis2.yaml`
- `config/euroc/se2.yaml`
- `config/euroc/okvis2_eu.yaml`
- `config/euroc/okvis2.yaml`
- `cnn/demo.py`
- `okvis_apps/scripts/bag_creator.py`
- `okvis_apps/src/dbow2_test.cpp`
- `okvis_apps/src/nn_test.cpp`
- `scripts/halve_framerate.py`
- `tools/hilti_bag2mrl.py`
- `tools/vbr_bag2mrl.py`
- `tools/okvis_to_vtk.sh`
- `trim_csv.py`
- `paper/introduction_related_work.tex`
- `paper/references.bib`
- `resources/readme/okvis2x-showcase.gif`
- `okvis_multisensor_processing/okvisConfig.hpp`
- `okvis_ros2/launch/okvis_node_synchronous.launch.xml`
- `okvis_apps/src/okvis_app_synchronous.cpp`
