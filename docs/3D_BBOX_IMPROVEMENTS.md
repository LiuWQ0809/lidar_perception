# 3D边界框估计改进说明

## 问题描述
之前的实现中，行人的3D边界框尺寸明显偏大：
- **原始尺寸**: 长度 4.1m × 宽度 3.7m × 高度 1.2m
- **实际尺寸**: 长度 ~0.5m × 宽度 ~0.5m × 高度 ~1.7m

## 根本原因
2D检测框内的点云包含了大量背景点：
1. **地面点**: 行人脚下的地面反射点
2. **墙壁点**: 行人身后的墙壁点云
3. **其他背景**: 周围环境的噪声点

这些背景点导致PCA估计出的边界框尺寸远大于实际目标。

## 解决方案

### 改进的点云处理流程

```
原始点云 (Lidar坐标系)
    ↓
1. 坐标变换 (Lidar → Camera)
    ↓
2. 投影过滤 (2D bbox内的点)
    ↓
3. 深度过滤 (±0.8m from median)
    ↓
4. 地面移除 (RANSAC平面分割) ⭐ 新增
    ↓
5. 欧几里得聚类 (提取最大簇) ⭐ 新增
    ↓
6. 离群点移除 (median + 2*std + 0.5m)
    ↓
7. PCA估计3D边界框
```

### 关键算法

#### 1. 地面移除 (removeGroundPlane)
**算法**: RANSAC (随机采样一致性)
- **原理**: 迭代随机选择3个点拟合平面，找到最多内点的平面作为地面
- **参数**:
  - `distance_threshold`: 0.05m (5cm) - 点到平面的距离阈值
  - `max_iterations`: 50 - RANSAC迭代次数
- **效果**: 移除大量地面点，保留站立的目标

**平面方程**: ax + by + cz + d = 0
```cpp
// 用3个点计算法向量
normal = (p2 - p1) × (p3 - p1)
// 计算点到平面距离
distance = |a*x + b*y + c*z + d|
```

#### 2. 欧几里得聚类 (extractLargestCluster)
**算法**: 基于距离的区域生长聚类
- **原理**: 将点云分成多个连通簇，提取最大的簇（目标对象）
- **参数**:
  - `cluster_tolerance`: 0.3m - 行人使用较小值
  - `min_cluster_size`: 5 - 最小簇大小
- **效果**: 分离前景目标和分散的背景点

**聚类步骤**:
1. 选择一个未处理的点作为种子
2. 查找距离 < threshold 的邻近点
3. 递归扩展形成簇
4. 选择点数最多的簇作为目标

#### 3. 深度过滤 (已有)
- 计算深度中位数
- 只保留深度在 [median - 0.8m, median + 0.8m] 范围内的点
- 过滤远处背景

#### 4. 离群点移除 (已改进)
- 改用中位数代替均值（更鲁棒）
- 阈值: median_dist + 2*std_dist + 0.5m
- 与Python版本保持一致

## 实现细节

### 配置参数 (config/fusion_config.yaml)
```yaml
fusion:
  min_points_in_box: 5        # 最少点云数量
  outlier_threshold: 0.5      # 离群点阈值
  use_ground_plane: true      # 启用地面移除
```

### 关键函数

```cpp
// 1. 地面移除
Eigen::MatrixXf removeGroundPlane(
    const Eigen::MatrixXf& points,
    float distance_threshold = 0.05f,
    int max_iterations = 50
);

// 2. 聚类提取
Eigen::MatrixXf extractLargestCluster(
    const Eigen::MatrixXf& points,
    float cluster_tolerance = 0.3f,
    int min_cluster_size = 5
);

// 3. 离群点移除（已改进）
Eigen::MatrixXf removeOutliers(
    const Eigen::MatrixXf& points
);
```

## 预期效果

### 点云过滤效果
```
原始点云: 1400+ 点
    ↓ 深度过滤
深度过滤后: ~1400 点 (90%+保留)
    ↓ 地面移除  ⭐
地面移除后: ~400-600 点 (移除50-70%地面点)
    ↓ 聚类提取  ⭐
聚类后: ~300-400 点 (提取主簇)
    ↓ 离群点移除
最终: ~300 点
```

### 尺寸估计改进
```
之前:
  Size raw: [4.10, 3.70, 1.20]  ❌ 太大
  After adjust: [2.30, 2.12, 1.24]

预期改进后:
  Size raw: [0.8, 0.6, 1.5]  ✓ 接近真实
  After adjust: [0.5, 0.5, 1.7]  ✓ 合理范围
```

## 调试与验证

### 查看详细日志
```bash
cd /home/nvidia/liuwq/fusion_cpp
export LD_LIBRARY_PATH=/home/nvidia/liuwq/fusion_cpp/install/lib:$LD_LIBRARY_PATH

# 运行并查看调试信息
./install/lib/fusion_cpp/fusion_perception_node \
    --ros-args --log-level sensor_fusion:=DEBUG 2>&1 | \
    grep -E "Ground removal|Clustering|Size raw"
```

### 关键指标
- **地面移除率**: 应该在 50-70% (移除大量地面点)
- **聚类选择**: 最大簇应占剩余点的 70%+ 
- **最终尺寸**: 
  - 长度: 0.5-1.0m
  - 宽度: 0.4-0.6m
  - 高度: 1.5-1.8m

## 代码变更总结

### 新增文件
无

### 修改文件
1. **include/fusion_cpp/sensor_fusion.hpp**
   - 添加 `removeGroundPlane()` 函数声明
   - 添加 `use_ground_plane_` 成员变量

2. **src/sensor_fusion.cpp**
   - 实现 `removeGroundPlane()` - RANSAC地面分割
   - 改进 `removeOutliers()` - 使用中位数
   - 修改 `getPointsInBBox()` - 添加地面移除步骤
   - 添加 `<ctime>` 头文件

### 兼容性
- ✅ 完全向后兼容
- ✅ 通过配置文件控制 (`use_ground_plane`)
- ✅ 保持与Python版本一致的逻辑

## 性能影响

### 计算复杂度
- **地面移除**: O(iterations × N) ≈ O(50 × 1400) = 70K 操作
- **聚类**: O(N²) in worst case, 实际约 O(N×K) ≈ O(400 × 50)
- **总增加时间**: 预计 1-2ms

### 实时性
- 原处理时间: ~8ms
- 增加后: ~9-10ms
- 仍然远低于 100ms 的实时要求 ✓

## 下一步优化（可选）

1. **自适应参数**
   - 根据目标类别动态调整聚类阈值
   - 行人: 0.3m, 车辆: 0.5m

2. **地面假设优化**
   - 利用相机标定的地面平面先验
   - 只在 Y > 某阈值的点中搜索地面

3. **GPU加速**
   - 使用 PCL CUDA 加速聚类
   - 并行化 RANSAC 采样

4. **时序一致性**
   - 跟踪历史帧的地面平面
   - 提高稳定性

## 参考
- RANSAC: Fischler & Bolles, 1981
- Euclidean Clustering: PCL implementation
- Ground Plane Segmentation: 3D LIDAR常用方法
