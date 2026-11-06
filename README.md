# Point-LIO for Unitree Go2 (ROS2 Humble)

把 [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar)（原版是 ROS1/catkin）
的 Point-LIO 适配到 **ROS2 Humble**，并针对 **Unitree Go2** 做了话题/参数/启动文件的适配。

节点**不启动任何雷达驱动**，直接订阅 Go2 自己发布的点云和 IMU：

- `/utlidar/cloud`：L1 点云（`sensor_msgs/PointCloud2`，约 14.7 Hz）
- `/utlidar/imu`：L1 内置 IMU（`sensor_msgs/Imu`，约 250 Hz），基础方案
- `/lowstate`：Go2 机身 IMU（`unitree_go/LowState`，500 Hz），经 `scripts/lowstate_to_imu.py`
  桥接成 `/go2/imu` 后使用，作为抗振动方案（实验性）

> 算法本身来自 HKU-MaRS 的 [Point-LIO](https://github.com/hku-mars/Point-LIO)，
> 本仓库是基于 ROS2 移植版做的 Go2 适配与实测，不是算法原仓库。

---

## 0. 状态速查（先看这个）

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| ROS2 Humble 编译 | ✅ | Ubuntu 22.04，`colcon build --symlink-install` 通过 |
| 静态 / 慢速建图（L1 内置 IMU） | ✅ | 74 s 静止回放，漂移 < 0.13 m |
| 走动建图（L1 内置 IMU） | ⚠️ | 开始走动后约 27 s 发散，见 §8.3。根因是 L1 内置 IMU 的振动/噪声，不是移植 bug |
| 走动建图（机身 IMU 桥接） | 🚧 | 实验性：明显延缓发散，短数据上有改善，但 3~4 min 以上仍会漂，尚未彻底解决，见 §8.4 |
| 离线 rosbag 回放 | ✅ | `go2_bag_replay*.launch.py` |
| PCD 地图保存 | ✅ | `pcd_save_en:=true`，Ctrl-C 后保存到源码树 `PCD/` |
| 官方 L1 / L2 等 launch | 📦 保留 | 上游原样保留，未在 Go2 上逐一验证，仅作参考 |

**一句话建议**：静态或慢速室内建图 → 直接用 §4.1；需要走动 → 先看 §8.3 / §8.4 的结论，
机身 IMU 方案可以明显改善但还不是"开箱即用"。

---

## 1. 仓库结构

```text
point_lio_unlidar/                      # colcon 工作区根目录（= 本仓库）
├── src/
│   └── point_lio_unilidar/             # ROS2 ament 包
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── LICENSE                     # GPL-2.0
│       ├── README.md                   # 上游 ROS2 移植版 README（参考）
│       ├── src/                        # C++ 源码
│       │   ├── laserMapping.cpp        # 主节点
│       │   ├── preprocess.cpp/.h       # 点云预处理（含 UNILIDAR 分支）
│       │   ├── Estimator.cpp/.h        # 状态估计
│       │   ├── parameters.cpp/.h
│       │   └── IMU_Processing.hpp
│       ├── include/                    # 依赖头文件（已内置，无需 submodule）
│       │   ├── common_lib.h / so3_math.h
│       │   ├── ikd-Tree/               # ikd-Tree（含 ikd_Tree.cpp）
│       │   ├── IKFoM/                  # IKFoM 工具包
│       │   └── FOV_Checker/
│       ├── config/
│       │   ├── go2_l1.yaml             # Go2 + L1 内置 IMU（基础方案）
│       │   ├── go2_body_imu.yaml       # Go2 + 机身 IMU（实验性方案）
│       │   ├── unilidar_l1.yaml / unilidar_l2.yaml
│       │   └── avia.yaml / mid360.yaml / horizon.yaml / ouster64.yaml / velody16.yaml
│       ├── launch/
│       │   ├── go2_l1.launch.py              # 在线：L1 内置 IMU
│       │   ├── go2_body_imu.launch.py        # 在线：机身 IMU（自动拉起桥接节点）
│       │   ├── go2_bag_replay.launch.py      # 离线回放：L1 内置 IMU
│       │   ├── go2_bag_replay_body_imu.launch.py
│       │   ├── mapping_go2.launch.py         # 早期简化版（等价 go2_l1，use_sim_time 默认 true）
│       │   └── mapping_*.launch.py           # 上游 L1/L2/Avia/Mid360/... 入口
│       ├── scripts/
│       │   └── lowstate_to_imu.py      # /lowstate -> /go2/imu 桥接节点
│       ├── rviz_cfg/loam_livox.rviz
│       ├── PCD/                        # 建图结果保存目录（*.pcd 不入库）
│       └── Log/                        # 上游调试脚本（运行时生成的 txt 不入库）
├── eval/                               # 本机测试/评测记录（不入库，见 §8.5）
├── build/ install/ log/                # colcon 构建产物（不入库）
├── README.md                           # 本文件
└── .gitignore
```

> `build/`、`install/`、`log/`、`eval/`、`*.pcd` 等都已写进 `.gitignore`：
> 它们要么是构建产物，要么是本机测试数据（`eval/` 约 639 MB），不需要上传。
> clone 后按 §3 重新编译即可。

---

## 2. 依赖

### 2.1 系统 / ROS 2

- Ubuntu 22.04
- ROS 2 Humble（`/opt/ros/humble`）
- PCL、Eigen3、OpenMP

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-rviz2 ros-humble-ros2bag \
  ros-humble-geometry-msgs ros-humble-nav-msgs \
  ros-humble-sensor-msgs ros-humble-visualization-msgs \
  libeigen3-dev
```

### 2.2 Unitree 相关

| 用途 | 需要的东西 |
| --- | --- |
| 在线连狗（两种方案都要） | Unitree 官方 [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)（CycloneDDS + Go2 话题），按其文档编译并 source |
| 机身 IMU 桥接 | `unitree_go` 消息（unitree_ros2 已提供）；同时需要 `/utlidar/imu` 作为时钟同步参考 |
| 官方 L1 / L2 雷达驱动 | `unilidar_sdk` / `unilidar_sdk2`（本仓库不绑定，Go2 上也不需要） |

> **Go2 用户注意**：Go2 自己就会发布 `/utlidar/cloud` 和 `/utlidar/imu`，
> **不要再启动 unilidar_sdk**，否则可能出现重复发布/抢占话题的问题。

本仓库在以下环境验证：Ubuntu 22.04 + ROS 2 Humble + 4 核 / 7.7 GiB 内存 + Go2（L1）。

---

## 3. 编译

```bash
# 1) 克隆（换成你自己的目录也可以）
cd ~
git clone https://github.com/hu-dc/point_lio_unlidar.git
cd point_lio_unlidar

# 2) source ROS 2
source /opt/ros/humble/setup.bash

# 3) 编译（单包工作区；4 核小内存机器建议 -j2 防 OOM）
MAKEFLAGS=-j2 colcon build --symlink-install

# 4) 生效
source install/setup.bash
```

- 全量编译约 3 分钟，增量编译约 20 秒（4 核 / 7.7 GiB 实测）。
- 若只改了 Python/launch/config，用 `--symlink-install` 时通常无需重新 build。
- 想重新编译干净：`rm -rf build install log` 后再执行上面的 build。

---

## 4. 运行

### 4.1 在线连狗（L1 内置 IMU，基础方案）

```bash
# 每个要跟 Go2 通信的终端都要先 source 这套环境
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh   # CycloneDDS + Go2 网卡 + unitree_go 消息

# 确认话题正常
ros2 topic hz /utlidar/cloud     # 约 14.7 Hz
ros2 topic hz /utlidar/imu       # 约 250 Hz

# 启动 Point-LIO（同一个终端继续 source 本工作区）
source ~/point_lio_unlidar/install/setup.bash
ros2 launch point_lio_unilidar go2_l1.launch.py rviz:=true
```

> source 顺序建议：ROS → unitree_ros2 → 本工作区，避免后面的把前面的环境覆盖掉。

启动后先静止 2~3 s 等 IMU 初始化完成，RViz 中应能看到点云和轨迹。

> **DDS 注意**：在线连狗时必须和 Go2 用同一套 DDS（通常是 unitree_ros2 提供的 CycloneDDS，
> 带 `CYCLONEDDS_URI=...ens37...`）。**不要**在这个终端里
> `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，否则和狗通信不上。

### 4.2 在线连狗（机身 IMU，实验性抗振动方案）

前提：能看到 `/lowstate`，且 `unitree_go` 消息可用（source unitree_ros2 即可）。

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh
source ~/point_lio_unlidar/install/setup.bash

ros2 topic hz /lowstate          # 约 500 Hz
ros2 launch point_lio_unilidar go2_body_imu.launch.py rviz:=true
```

这条 launch 会：

1. 拉起 `scripts/lowstate_to_imu.py`：订阅 `/lowstate`，用 `/utlidar/imu` 的时间戳做时钟标定，
   输出 `sensor_msgs/Imu` 到 `/go2/imu`；
2. 用 `config/go2_body_imu.yaml` 启动 `pointlio_mapping`（`use_imu_as_input=true`）。

桥接节点启动后会先缓存约 2 s 的 `/lowstate`，等时钟标定完成后再补发；
所以**必须保证 `/utlidar/imu` 也在发布**（Go2 默认会发），否则桥接节点无法标定。

如果你的 unitree_ros2 发布的是 `/lf/lowstate`，用参数覆盖：

```bash
ros2 launch point_lio_unilidar go2_body_imu.launch.py rviz:=true input_topic:=/lf/lowstate
```

### 4.3 离线 rosbag 回放

离线回放建议和"连狗"那套 DDS 环境隔离开（避免串扰，也方便得到可复现的结果）：
先 `unset CYCLONEDDS_URI`，再显式用 FastDDS + 独立 domain：

```bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

source /opt/ros/humble/setup.bash
source ~/point_lio_unlidar/install/setup.bash

# 方案 A：L1 内置 IMU。bag 需包含 /utlidar/cloud 和 /utlidar/imu
ros2 launch point_lio_unilidar go2_bag_replay.launch.py \
  bag_path:=$HOME/go2_bags/20261102_163913_static rviz:=true

# 方案 B：机身 IMU。bag 还需要包含 /lowstate
ros2 launch point_lio_unilidar go2_bag_replay_body_imu.launch.py \
  bag_path:=$HOME/go2_bags/20261105_095434_walk_308_4 rviz:=true
```

launch 会自动 `ros2 bag play ... --clock --topics ...`，**不要再手动 play 一遍**。
`rate` 默认 1.0，改大容易把时序搞乱，建议保持。

手动分步跑（等价）：

```bash
# 终端 1
ros2 launch point_lio_unilidar go2_l1.launch.py rviz:=true use_sim_time:=true

# 终端 2
ros2 bag play <bag目录> --clock --topics /utlidar/cloud /utlidar/imu
```

---

## 5. 话题与 RViz

### 5.1 订阅

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | L1 点云，约 14.7 Hz |
| `/utlidar/imu` | `sensor_msgs/Imu` | L1 内置 IMU，约 250 Hz |
| `/lowstate` | `unitree_go/LowState` | 机身 IMU 原始数据，约 500 Hz |
| `/go2/imu` | `sensor_msgs/Imu` | 桥接节点输出，供 Point-LIO 使用 |

点云字段（L1）：`x/y/z/intensity` FLOAT32、`ring` UINT16、`time` FLOAT32（秒，单帧内 0~0.065），
`point_step=32`，`frame_id=utlidar_lidar`。

### 5.2 发布

| 话题 | 说明 |
| --- | --- |
| `/cloud_registered` | 世界系配准后点云 |
| `/cloud_registered_body` | 机体系点云 |
| `/cloud_effected` | 参与匹配的有效点 |
| `/Laser_map` | 累积地图点云 |
| `/path` | 轨迹 |
| `/aft_mapped_to_init` | 里程计（`nav_msgs/Odometry`） |
| TF | `camera_init` → `aft_mapped` |

RViz 使用 `rviz_cfg/loam_livox.rviz`（`rviz:=true` 时自动加载），
**Fixed Frame 设为 `camera_init`**，否则可能看不到点云。

---

## 6. 参数说明

### 6.1 `config/go2_l1.yaml`（L1 内置 IMU）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `common.lid_topic` | `/utlidar/cloud` | 点云话题 |
| `common.imu_topic` | `/utlidar/imu` | IMU 话题 |
| `common.time_lag_imu_to_lidar` | `0.0` | Go2 上时间已对齐 |
| `preprocess.lidar_type` | `5` | UNILIDAR 分支 |
| `preprocess.scan_line` | `12` | L1 线数 |
| `preprocess.scan_rate` | `15` | 雷达转速相关 |
| `preprocess.timestamp_unit` | `0` | 单位：秒 |
| `preprocess.blind` | `0.3` | 最近盲区 |
| `mapping.imu_time_inte` | `0.004` | 约 250 Hz |
| `mapping.extrinsic_est_en` | `false` | 关闭外参在线估计 |
| `mapping.extrinsic_T` | `[0, 0, 0]` | L1 与内置 IMU 同模块 |
| `mapping.extrinsic_R` | 单位阵 | 同上 |
| `mapping.fov_degree` | `360` | L1 水平视场 |
| `mapping.det_range` | `50` | 有效距离（m） |
| `mapping.satu_acc` / `satu_gyro` | `30` / `35` | IMU 饱和阈值 |
| `odometry.publish_odometry_without_downsample` | `true` | 高频里程计 |

launch 中固定的关键参数：`use_imu_as_input=false`、`prop_at_freq_of_imu=true`、
`check_satu=true`、`init_map_size=10`、`point_filter_num=1`、`space_down_sample=true`、
`filter_size_surf=0.1`、`filter_size_map=0.1`、`cube_side_length=1000`。

### 6.2 `config/go2_body_imu.yaml`（机身 IMU，实验性）

与 L1 配置的主要差异：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `common.imu_topic` | `/go2/imu` | 桥接后的话题 |
| `mapping.imu_time_inte` | `0.003` | 协方差传播间隔（约 3 ms） |
| `mapping.extrinsic_T` | `[0.31502, 0.0, -0.089145]` | 雷达相对机身 IMU 的平移，来自 Ros2SLAM 的 Go2 标定 |
| `mapping.extrinsic_R` | `[-0.965512,0,0.260358, 0,1,0, -0.260358,0,-0.965512]` | 对应旋转 |
| `mapping.fov_degree` | `180` | 机身 IMU 方案下更保守 |
| `mapping.det_range` | `30` | |
| `preprocess.blind` | `0.5` | |
| launch 设置 | `use_imu_as_input=true` | |

> 上面的外参来自一台 Go2 的标定结果，换机器建议重新标定；
> 它直接影响 §8.4 里的残余漂移大小。

### 6.3 常用 launch 参数

| 参数 | 适用 launch | 默认 | 说明 |
| --- | --- | --- | --- |
| `rviz` | 全部 | `false` | 是否启动 RViz |
| `pcd_save_en` | 全部 | `false` | 是否保存 PCD |
| `config_file` | 全部 | 对应 yaml | 换配置 |
| `bag_path` | `go2_bag_replay*` | 必填 | bag 目录 |
| `rate` | `go2_bag_replay*` | `1.0` | 回放倍速 |
| `input_topic` | `go2_body_imu*` | `/lowstate` | 机身 IMU 原始话题 |
| `imu_topic_out` | `go2_body_imu*` | `/go2/imu` | 桥接输出话题 |
| `odom_header_frame_id` | Go2 系列 | `camera_init` | 里程计 frame |
| `check_satu` | Go2 系列 | `true` | IMU 饱和检查 |
| `use_imu_as_input` | Go2 系列 | 依 launch | `true` 用于机身 IMU |

---

## 7. 保存地图（PCD）

```bash
ros2 launch point_lio_unilidar go2_l1.launch.py rviz:=true pcd_save_en:=true
# 跑完后按 Ctrl-C，不要直接 kill
```

生成的文件在**源码树**里（`ROOT_DIR` 是编译期写死的源码目录）：

```text
~/point_lio_unilidar/src/point_lio_unilidar/PCD/scans.pcd
```

说明：

- `pcd_save_interval: -1` 表示所有帧存成一个文件；
- 必须正常 Ctrl-C 退出才会写盘；
- `PCD/*.pcd` 已在 `.gitignore` 中，不会误上传。

---

## 8. 验证记录与已知问题

> 以下数据来自本机实测；测试 bag 和评测记录（`eval/`，约 639 MB）没有上传到仓库，
> 见 §8.5。

### 8.1 编译 ✅

`colcon build --symlink-install` 通过（4 核 / 7.7 GiB 内存）。

### 8.2 静态回放 ✅

bag：`20261102_163913_static`，74 s，Go2 静止，L1 内置 IMU。

- IMU 初始化 100%，无 "No Effective Points!"
- `/cloud_registered` 约 15~21 Hz
- 相对起点漂移 < 0.13 m

### 8.3 走动（L1 内置 IMU）⚠️ 会发散

bag：`20261103_102932_walk`，176 s。前 ~20 s 正常，约 27 s 开始：
姿态压不住 → 高度一路下降 → 位置越跑越远（最终到 1e3 m 量级）。
另外 4 段 `walk_308` bag 也是类似时间点开始发散。

已排查（不是移植引入的 bug）：

- 点云字段/偏移/`point_step` 与 L1 实际对齐；
- `time` 字段、`timestamp_unit=0`（秒）语义正确；
- `/utlidar/imu` 实测约 249.7 Hz，无重复/乱序时间戳，与点云同步正常；
- 外参为单位阵与 L1 模块结构一致；
- 与上游 ROS1 版本的关键处理逐段对照过。

根因指向 L1 **内置 IMU 质量**：走动 bag 中加速度均值约 10.36 m/s²、标准差 4.63、
峰值 51.4 m/s²，60 s 内陀螺漂移 15~20°，Point-LIO 的传播在这种噪声下会失稳。
静态/慢速没问题，走动需要更高质量的 IMU → 所以有了 §8.4。

### 8.4 机身 IMU 桥接 🚧 实验性，尚未完全解决

Go2 的 `/lowstate`（`unitree_go/LowState`）里有 500 Hz 的机身 IMU，但：

- 没有 `header.stamp`，只有 32 位 MCU tick（2 ms 递增）；
- 用 DDS 接收时间当时间戳会因网络/批处理跳变，直接喂给 LIO 会崩。

`scripts/lowstate_to_imu.py` 的做法：

1. `stamp = tick * 0.001 + C`；
2. 用 `/utlidar/imu` 的时间戳在线估计常数 `C`（10 s 窗口取 p01，限制在标定值 ±10 ms 内，
   并做速率限制）；
3. 输出单调递增的 `sensor_msgs/Imu` 到 `/go2/imu`；
4. 复制加速度/角速度/四元数（四元数顺序 w,x,y,z），协方差置零。

配合 `config/go2_body_imu.yaml`（外参、`fov_degree=180`、`det_range=30`）和
`use_imu_as_input=true`。

**实测效果**（本机，不同 bag，非严格对比）：

- 短数据（约 45 s）上明显好于 L1 内置 IMU：中位误差约 1.4 m，最大约 9.4 m；
- 长数据（`walk_308`，约 4~6 min）上仍会在 3~4 min 后发散，
  表现为高度/位置持续漂移（几十米到千米量级，视运行而异）；
- 结论：**桥接把发散时间往后推了，但没有根治**。原因可能是剩余时钟抖动、
  外参/标定误差、L1 点云与机身 IMU 之间的时延、以及纯 LIO 无回环导致的累积漂移。

后续可以尝试的方向（欢迎 PR）：

- 更准的时间同步（改用 PPS/硬件触发，或对 `C` 做更稳的估计）；
- 重新标定 `extrinsic_T` / `extrinsic_R`（每台机器不同）；
- 调整 IMU 噪声协方差、`imu_time_inte`、`det_range` / `fov_degree`；
- 加回环/位姿图（LIO-SAM 式）或与轮式/腿式里程计融合。

### 8.5 复现与数据说明

- 测试 bag 在 Go2 上录制：`/utlidar/cloud`、`/utlidar/imu`，走动数据额外录 `/lowstate`，
  并另录一个参考轨迹（`*_sport.csv`，约 20 Hz）用于对比；
- 评测脚本/记录在 `eval/`（csv/pcd/log/report），体积约 639 MB，**没有上传**，
  已加入 `.gitignore`；
- 想复现：按 §4.3 回放自己的 bag，按 §7 保存地图。

---

## 9. 故障排查

| 现象 | 原因 / 处理 |
| --- | --- |
| 卡在 `IMU Initializing`，没有点云 | 点云没进来，或 QoS 不匹配。先 `ros2 topic hz /utlidar/cloud`；检查是否 source 了 unitree_ros2 / 是否连上狗 |
| `does not match the available interface` | 在线时 `CYCLONEDDS_URI` 的网卡名不存在，或 RMW 混用。在线用 CycloneDDS + 狗的网卡；离线先 `unset CYCLONEDDS_URI` 并用 `rmw_fastrtps_cpp` |
| RViz 里看不到点云 | Fixed Frame 不是 `camera_init`；或话题名不对；或 TF 没起来 |
| `No Effective Points!` 反复出现 | 参数/场景不匹配，调 `det_range`、`blind`、`filter_size_surf`；确认点云 frame 正确 |
| 轨迹缓慢漂移 | 纯 LIO 没有回环，长时间必然累积误差，属正常现象 |
| 走动后高度一路掉、越跑越远 | L1 内置 IMU 振动导致，见 §8.3；换机身 IMU 方案（§4.2） |
| `/go2/imu` 没有数据 | 没有 `unitree_go` 消息（source unitree_ros2）；`/lowstate` 话题名不对（试 `input_topic:=/lf/lowstate`）；或 `/utlidar/imu` 没发布导致无法标定 |
| `imu loop back, clear deque` | IMU 时间戳不单调。桥接节点已保证单调；如果换了自己的 IMU 源需自行处理 |
| `TF_REPEATED_DATA` | 同一 TF 被多个节点重复发布（例如同时开了别的发 TF 的节点），可忽略，或关掉其中一处 |
| `Failed to find match for field 'time'` | 点云没有逐点时间字段，`preprocess` 里对 `time` 的读取分支不匹配 |
| Ctrl-C 后没生成 PCD | 需要 `pcd_save_en:=true`，且正常 Ctrl-C 退出；文件在源码树 `PCD/` 下 |
| 离线回放跑飞 | `rate` 不要大于 1.0；确认 `use_sim_time=true` 和 `--clock` |

---

## 10. 来源、许可与差异

### 来源

- 算法：[hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO)（HKU-MaRS）
- ROS1 / Unitree 版：[unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar)
- ROS2 移植基础：[dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2)

### 许可

GPL-2.0，见 `src/point_lio_unilidar/LICENSE`（保留上游版权声明）。

> 注意：`package.xml` 里的 `<license>` 字段仍是上游遗留的 `BSD`，与实际许可文件不一致；
> 本仓库的实际许可以该 `LICENSE` 文件（GPL-2.0）为准。

### 相对上游的改动

1. 包名 `point_lio` → `point_lio_unilidar`，构建从 catkin 改为 ROS2 ament/colcon；
2. 新增 Go2 配置：`config/go2_l1.yaml`、`config/go2_body_imu.yaml`；
3. 新增 Go2 launch：`go2_l1`、`go2_body_imu`、`go2_bag_replay`、`go2_bag_replay_body_imu`、
   `mapping_go2`；
4. 新增 `scripts/lowstate_to_imu.py`（机身 IMU 桥接）及其时间戳方案；
5. 内置 `ikd-Tree` / `IKFoM` 源码（不依赖 git submodule）；
6. 去掉 `livox_ros_driver2` 依赖（在 CMake 中以注释保留），Go2 场景不需要；
7. 保留上游的 `mapping_*` / `correct_odom_*` / `gdb_debug_example` launch 作为参考。

### 致谢

感谢 HKU-MaRS 的 Point-LIO 算法工作，以及 Unitree 与 dfloreaa 的开源移植。
