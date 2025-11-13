# Fusion Perception C++

C++实现的相机和激光雷达融合感知系统，用于3D目标检测和跟踪。

## 项目简介

这是 `fusion_perception` Python 项目的 C++ 版本，提供相同的功能：
- 基于 YOLOv8 的 2D 目标检测（TensorRT 加速）
- 相机和 Lidar 数据融合
- 3D 边界框估计
- 多目标跟踪（卡尔曼滤波 + 匈牙利算法）
- 坐标系转换（Camera → Body）
- ROS2 集成

## 主要特性

✅ **高性能 C++ 实现** - 优化的算法和数据结构  
✅ **TensorRT 加速** - 目标检测 < 15ms  
✅ **Eigen 矩阵运算** - 高效的点云处理  
✅ **ROS2 集成** - 完整的话题订阅和发布  
✅ **与 Python 版本功能一致** - 相同的配置和接口  

## 系统要求

### 必需依赖
- **ROS2** (Humble或更高版本)
- **CMake** >= 3.8
- **C++17** 编译器
- **OpenCV** >= 4.2
- **Eigen3**
- **yaml-cpp**
- **livox_ros_driver2**

### 可选依赖（用于TensorRT加速）
- **CUDA** >= 11.0
- **TensorRT** >= 8.0

## 项目结构

```
fusion_cpp/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS2 包描述
├── build.sh                # 编译脚本
├── README.md               # 本文档
├── include/fusion_cpp/     # 头文件
│   ├── livox_parser.hpp
│   ├── sensor_fusion.hpp
│   ├── tracker.hpp
│   ├── detector_tensorrt.hpp
│   └── fusion_perception_node.hpp
├── src/                    # 源文件
│   ├── livox_parser.cpp
│   ├── sensor_fusion.cpp
│   ├── tracker.cpp
│   ├── detector_tensorrt.cpp
│   └── fusion_perception_node.cpp
├── launch/                 # Launch 文件
│   └── fusion_perception.launch.py
├── config/                 # 配置文件
│   └── fusion_config.yaml
└── calibration/            # 标定文件
    └── front_left.yaml
```

## 安装步骤

### 1. 安装系统依赖

```bash
# 安装基础依赖
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libyaml-cpp-dev \
    libopencv-dev

# 安装 ROS2 依赖
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport
```

### 2. 安装 Livox ROS2 驱动

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2
```

### 3. （可选）安装 TensorRT

如果需要使用 TensorRT 加速检测：

```bash
# Jetson 平台通常已预装 TensorRT
# 检查安装
dpkg -l | grep TensorRT

# 如果未安装，请从 NVIDIA 官网下载对应版本
```

### 4. 编译项目

```bash
cd /home/nvidia/liuwq/fusion_cpp
./build.sh
```

编译成功后，输出：
```
Build Successful!
To run the node, use:
  cd /home/nvidia/liuwq/fusion_cpp
  ./run.sh
```

## 使用方法

### 1. 配置文件

编辑 `config/fusion_config.yaml` 设置：
- 相机和 Lidar 话题名称
- 模型路径和参数
- 感知范围
- 跟踪参数

```yaml
sensors:
  lidar:
    topic: "/livox/lidar"
  cameras:
    - name: front_left
      topic: "/cr/camera/rgb/front_left_full"
      encoding: "rgb8"
      calibration: "calibration/front_left.yaml"
      image_size: [1920, 1080]
      projection_model: pinhole
    - name: front_right
      topic: "/cr/camera/rgb/front_right_full"
      encoding: "rgb8"
      calibration: "calibration/front_right.yaml"
      image_size: [1920, 1080]
      projection_model: pinhole
    - name: left
      topic: "/cr/camera/bgr/left_960_768"
      encoding: "bgr8"
      calibration: "calibration/left.yaml"
      image_size: [960, 768]
      projection_model: fisheye
    # right / rear cameras omitted for brevity

models:
  detector:
    backend: "tensorrt"
    model_path: "models/yolov8n_fp16.engine"
    confidence_threshold: 0.5
```

常用融合相关参数：

- `fusion.duplicate_merge_distance`：跨相机结果在车体坐标系下的合并半径，默认 0.9 m，可根据实际车宽调整。
- `fusion.ground_clamp_margin`：将 3D 框底面“吸附”到地面的最小距离，防止框沉入地面。
- `fusion.person_height_range`：约束行人高度的范围，减少点云稀疏导致的高度发散。
- `tracking.static_speed_threshold / smoothing_alpha_*`：控制静止或低速目标的输出平滑强度，减小晃动与轨迹分裂。

### 2. 标定文件

`calibration/` 目录下提供前左、前右、左右和后视5路相机的 `Tbc` 矩阵及内参。每个相机节点都会根据对应的 YAML 自动载入内参/畸变系数并生成去畸变映射，运行前请确认这些文件与车辆安装位置匹配，同时在 `config/fusion_config.yaml` 中填写 `extrinsic.lidar_to_body`（或提供参考相机的 `lidar_to_camera`）以生成所有相机的 `lidar→camera` 外参。

### 3. 运行节点

```bash
# 方式1: 使用运行脚本（推荐）
cd /home/nvidia/liuwq/fusion_cpp
./run.sh

# 方式2: 直接运行
source /opt/ros/humble/setup.bash
cd /home/nvidia/liuwq/fusion_cpp
export LD_LIBRARY_PATH=$PWD/install/lib:$LD_LIBRARY_PATH
./install/lib/fusion_cpp/fusion_perception_node
```

### 4. 查看输出

```bash
# 查看障碍物话题
ros2 topic echo /fusion_perception/obstacles

# 查看可视化图像
ros2 run rqt_image_view rqt_image_view /fusion_perception/visualization
```

## 话题接口

### 订阅话题
- 多路相机图像 (sensor_msgs/Image)：由 `config/fusion_config.yaml` 中 `sensors.cameras` 列表定义，例如 `/cr/camera/rgb/front_left_full`、`/cr/camera/bgr/left_960_768` 等
- `/livox/lidar` (livox_ros_driver2/CustomMsg) - Lidar 点云

### 发布话题
- `/fusion_perception/obstacles` (visualization_msgs/MarkerArray) - 3D 障碍物
- `/fusion_perception/visualization` (sensor_msgs/Image) - 可视化图像（可选）


## 多相机融合说明
- 每路相机启动时都会根据对应的标定文件自动生成去畸变映射，YOLO 检测在去畸变后的图像上运行。
- `SensorFusion` 为每个相机注册独立的内参/外参，先将 Livox 点云变换到相机坐标系后再进行图像投影匹配。
- 通过 `Tbc` 将检测结果转换到车体 (body) 坐标系，`MultiObjectTracker` 在 body 坐标系内实现跨相机的 360° 稳定跟踪。
- `config/fusion_config.yaml` 可通过 `enabled` 字段随时开启/关闭某路相机，也可以为不同分辨率设置单独的去畸变输出尺寸。

## 性能指标

在 NVIDIA Jetson 平台上的典型性能：

| 模块 | 处理时间 | 说明 |
|------|---------|------|
| YOLOv8 检测 (TensorRT) | ~10-15ms | FP16 推理 |
| 点云融合 | ~5-10ms | Eigen 优化 |
| 多目标跟踪 | ~2-5ms | 卡尔曼滤波 |
| **总处理时间** | **~20-30ms** | **输出频率: 10Hz** |

## 与 Python 版本对比

| 特性 | Python 版本 | C++ 版本 |
|------|------------|----------|
| 检测速度 | 10-15ms | 10-15ms |
| 总处理时间 | 30-50ms | 20-30ms ⚡ |
| 内存占用 | ~2GB | ~1GB ⚡ |
| CPU 占用 | 30-50% | 15-25% ⚡ |
| 代码复杂度 | 简单 | 中等 |

## 故障排除

### 编译错误

**问题**: `livox_ros_driver2 not found`
```bash
# 解决: 先编译 livox_ros_driver2
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

**问题**: `TensorRT headers not found`
```bash
# 解决: 安装 TensorRT 或禁用 TensorRT
# 在 CMakeLists.txt 中注释掉 find_package(TensorRT)
```

### 运行时错误

**问题**: `No camera data available`
```bash
# 检查相机话题是否发布
ros2 topic list | grep camera
ros2 topic hz /cr/camera/rgb/front_left_full
```

**问题**: `Camera data too old`
```bash
# 调整时间同步容差
# 在 config/fusion_config.yaml 中增加 sync_slop 值
performance:
  sync_slop: 0.5  # 增加到 500ms
```

## 开发说明

### 代码风格
- 遵循 Google C++ Style Guide
- 使用 C++17 标准
- 注释使用 Doxygen 格式

### 添加新功能
1. 在 `include/fusion_cpp/` 添加头文件
2. 在 `src/` 添加实现文件
3. 更新 `CMakeLists.txt`
4. 编译测试

## 许可证

Apache-2.0 License

## 作者

liuwq

## 参考

- [YOLOv8](https://github.com/ultralytics/ultralytics)
- [TensorRT](https://developer.nvidia.com/tensorrt)
- [ROS2](https://docs.ros.org/en/humble/)
- [Livox SDK](https://github.com/Livox-SDK/Livox-SDK2)

## 更新日志

### v1.0.0 (2024-11-01)
- ✅ 初始 C++ 实现
- ✅ 完整功能迁移自 Python 版本
- ✅ TensorRT 加速支持
- ✅ ROS2 Humble 兼容
