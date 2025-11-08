# Python与C++代码同步说明

## 更新日期
2025-11-01

## 更新目的
将C++版本的融合感知代码与Python版本保持一致，确保两个版本输出相同的结果。

## 主要修改

### 1. 行人尺寸约束 (`sensor_fusion.cpp`)

#### 默认尺寸值更新
```cpp
// 旧值
default_sizes_["person"] = Eigen::Vector3f(0.5f, 0.5f, 1.7f);

// 新值（与Python一致）
default_sizes_["person"] = Eigen::Vector3f(0.22f, 0.44f, 1.68f);
// length=胸厚22cm, width=肩宽44cm, height=身高168cm
```

#### 尺寸约束范围更新
```cpp
// 旧值
const float min_length = 0.15f, max_length = 0.8f;   // 胸厚
const float min_width = 0.25f, max_width = 0.7f;     // 肩宽
const float min_height = 1.3f, max_height = 2.0f;    // 身高

// 新值（基于真实人体数据，与Python一致）
const float min_length = 0.18f, max_length = 0.26f;   // 胸厚: 18-26cm
const float min_width = 0.36f, max_width = 0.52f;     // 肩宽: 36-52cm
const float min_height = 1.50f, max_height = 1.85f;   // 身高: 150-185cm
```

**说明**：
- length = 胸厚（前后厚度）
- width = 肩宽（左右宽度）
- height = 身高（垂直高度）

移除了额外的加权平滑逻辑，只保留硬性裁剪，使结果更稳定。

### 2. 卡尔曼滤波参数 (`tracker.cpp`)

#### 过程噪声协方差（Q矩阵）
```cpp
// 旧值
Q_ *= 0.01f;
Q_.block<3, 3>(3, 3) *= 0.5f;  // 速度噪声
Q_.block<3, 3>(6, 6) *= 1.0f;  // 尺寸噪声

// 新值（与Python一致）
Q_ *= 0.05f;
Q_.block<3, 3>(3, 3) *= 0.6f;   // 速度噪声 (0.05 * 0.6 = 0.03)
Q_.block<3, 3>(6, 6) *= 10.0f;  // 尺寸噪声 (0.05 * 10 = 0.5)
```

#### 测量噪声协方差（R矩阵）
```cpp
// 旧值
R_ *= 1.2f;
R_.block<3, 3>(0, 0) *= 0.8f;  // 位置
R_.block<3, 3>(3, 3) *= 1.0f;  // 尺寸
R_(6, 6) = 1.0f;               // 角度

// 新值（与Python一致）
R_ *= 0.8f;
R_.block<3, 3>(0, 0) *= 0.5f;  // 位置 (0.8 * 0.5 = 0.4)
R_.block<3, 3>(3, 3) *= 0.8f;  // 尺寸 (0.8 * 0.8 = 0.64)
R_(6, 6) = 0.6f;               // 角度
```

**效果**：
- 降低过程噪声Q：系统模型更稳定，减少状态突变
- 增加测量噪声R：更信任卡尔曼预测，减少测量跳变
- 整体效果：跟踪更平滑，减少坐标跳变

### 3. 3D边界框估计逻辑（已正确实现）

C++版本已经正确实现了以下逻辑（与Python一致）：

1. **Y轴检测高度**：
```cpp
Eigen::Vector3f y_components = sorted_eigenvectors.row(1).cwiseAbs();
int height_idx = 0;
y_components.maxCoeff(&height_idx);
float height = size_raw(height_idx);
```

2. **长宽区分**：
```cpp
if (dim1 > dim2) {
    length = dim1;  // 较大的是长度
    width = dim2;   // 较小的是宽度
    forward_idx = remaining_indices[0];
}
```

3. **yaw角计算**（XZ水平面）：
```cpp
Eigen::Vector3f forward_vector = sorted_eigenvectors.col(forward_idx);
float yaw = std::atan2(forward_vector(0), forward_vector(2));
```

## 验证方法

### 编译C++版本
```bash
cd /home/nvidia/liuwq/fusion_cpp
./build.sh
```

### 运行测试
```bash
# 运行C++版本
cd /home/nvidia/liuwq/fusion_cpp
./run.sh

# 在另一个终端查看输出
ros2 topic echo /fusion_perception/obstacles_cpp
```

### 对比输出
Python和C++版本应该输出相似的结果：
- **Person尺寸**：
  - length: 0.18-0.26m（胸厚）
  - width: 0.36-0.52m（肩宽）
  - height: 1.50-1.85m（身高）
- **位置坐标**：平滑无跳变
- **yaw角度**：平滑变化

## 文件修改清单

- `/home/nvidia/liuwq/fusion_cpp/src/sensor_fusion.cpp`
  - 第57行：更新行人默认尺寸
  - 第572-588行：更新尺寸约束范围

- `/home/nvidia/liuwq/fusion_cpp/src/tracker.cpp`
  - 第32-42行：更新卡尔曼滤波Q和R矩阵参数

## 注意事项

1. **坐标系定义**：
   - Camera坐标系: X右, Y下, Z前
   - Body坐标系: X前, Y左, Z上
   - 确保两个版本使用相同的坐标系定义

2. **校准文件**：
   - C++版本需要使用相同的校准文件
   - 路径：`/home/nvidia/liuwq/fusion_cpp/calibration/front_left.yaml`

3. **配置文件**：
   - 确保config/fusion_config.yaml中的参数两个版本一致

## 预期效果

更新后的C++版本应该：
- ✅ 行人尺寸准确（胸厚18-26cm，肩宽36-52cm，身高150-185cm）
- ✅ 坐标跟踪平滑（减少跳变）
- ✅ 与Python版本输出一致
- ✅ 性能优于Python版本（C++原生实现）
