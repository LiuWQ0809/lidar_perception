# Lidar坐标系高度过滤说明

## 修改内容

### 问题
之前的实现中，Lidar坐标系的高度过滤范围是从Camera感知范围转换而来的，导致高度过滤范围不够精确。

### 解决方案
在Lidar坐标系下**独立**设置高度过滤范围，不依赖于Camera感知范围的转换。

## 实现细节

### 1. 新增成员变量
在 `SensorFusion` 类中添加：
```cpp
// Lidar坐标系下的高度过滤范围（独立于感知范围）
float lidar_height_min_;  // -0.3m (地面以下)
float lidar_height_max_;  // 1.8m  (地面以上)
```

### 2. 初始化
在构造函数中设置固定值：
```cpp
lidar_height_min_ = -0.3f;  // 地面以下0.3米
lidar_height_max_ = 1.8f;   // 地面以上1.8米
```

### 3. 过滤逻辑
在 `filterPointsInLidarRange()` 函数中：
- **XY范围**: 使用 `lidar_range_` (从Camera感知范围转换而来)
- **Z轴（高度）**: 使用固定的 `lidar_height_min_` 和 `lidar_height_max_`

```cpp
if (x >= lidar_range_.x_min && x <= lidar_range_.x_max &&
    y >= lidar_range_.y_min && y <= lidar_range_.y_max &&
    z >= lidar_height_min_ && z <= lidar_height_max_) {
    // 保留此点
}
```

## 坐标系说明

### Lidar坐标系 (右手坐标系)
```
        Z (向上)
        |
        |
        o-------- X (向前)
       /
      /
     Y (向左)
```

- **X轴**: 向前（车辆前进方向）
- **Y轴**: 向左
- **Z轴**: 向上（垂直方向）

### 高度过滤范围
- **Z_min = -0.3m**: 地面以下0.3米（允许一些地面凹陷）
- **Z_max = 1.8m**: 地面以上1.8米（覆盖行人和车辆）

这个范围可以：
- ✅ 过滤掉天空中的噪点
- ✅ 过滤掉地面以下的噪点
- ✅ 保留行人高度范围（1.5-1.8m）
- ✅ 保留车辆高度范围（1.4-1.8m）

## 与Camera感知范围的关系

### Camera感知范围（config/fusion_config.yaml）
```yaml
perception_range:
  x_min: -5.0   # 左侧5米
  x_max: 5.0    # 右侧5米
  y_min: -0.3   # 下方0.3米
  y_max: 1.8    # 上方1.8米
  z_min: -5.0   # 后方5米
  z_max: 15.0   # 前方15米
```

### Lidar过滤范围
| 维度 | 来源 | 范围 | 说明 |
|------|------|------|------|
| X (前后) | 从Camera转换 | [-5.19, 14.90] | 保持Camera感知范围 |
| Y (左右) | 从Camera转换 | [-5.25, 4.93] | 保持Camera感知范围 |
| **Z (高度)** | **独立设置** | **[-0.30, 1.80]** | **不依赖Camera转换** |

## 日志输出示例

启动时会显示：
```
[INFO] [SensorFusion]: Lidar perception range: x[-5.19, 14.90] y[-5.25, 4.93] z[-0.61, 1.53]
[INFO] [SensorFusion]: Lidar height filtering: z[-0.30, 1.80] (independent of perception range)
```

注意：
- 第一行的 `z[-0.61, 1.53]` 是从Camera转换得到的（仅供参考）
- 第二行的 `z[-0.30, 1.80]` 是**实际使用**的高度过滤范围

## 效果预期

### 点云过滤效果
```
原始Lidar点云: ~100,000 点
    ↓ XY范围过滤
~50,000 点
    ↓ 高度过滤 [-0.3, 1.8]m ⭐
~30,000 点 (移除天空和地面以下的点)
    ↓ 后续处理...
```

### 优势
1. **精确控制**: 直接指定有意义的高度范围
2. **独立于相机**: 不受相机感知范围Y轴（下方/上方）的影响
3. **更高效**: 在Lidar坐标系下直接过滤，避免不必要的坐标转换
4. **更准确**: 基于实际物理高度，符合真实场景

## 代码位置

### 头文件
- `include/fusion_cpp/sensor_fusion.hpp`
  - 添加 `lidar_height_min_` 和 `lidar_height_max_` 成员变量

### 源文件
- `src/sensor_fusion.cpp`
  - 构造函数中初始化高度范围
  - `computeLidarPerceptionRange()` 添加日志输出
  - `filterPointsInLidarRange()` 使用独立的高度范围

## 参数调整

如需修改高度范围，编辑 `src/sensor_fusion.cpp` 中的：
```cpp
lidar_height_min_ = -0.3f;  // 根据实际需求调整
lidar_height_max_ = 1.8f;   // 根据实际需求调整
```

常见场景建议值：
- **只检测行人**: `[-0.3, 2.0]`
- **包含车辆**: `[-0.3, 3.5]` (卡车/公交车)
- **平坦地面**: `[0.0, 1.8]` (不需要地面以下容差)
- **起伏地形**: `[-0.5, 2.0]` (更大的地面容差)

## 测试验证

运行节点并观察日志：
```bash
cd /home/nvidia/liuwq/fusion_cpp
export LD_LIBRARY_PATH=/home/nvidia/liuwq/fusion_cpp/install/lib:$LD_LIBRARY_PATH
./install/lib/fusion_cpp/fusion_perception_node
```

查看启动日志中的：
```
[INFO] [SensorFusion]: Lidar height filtering: z[-0.30, 1.80]
```

确认高度范围设置正确。
