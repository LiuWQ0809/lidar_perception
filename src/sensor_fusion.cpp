#include "fusion_cpp/sensor_fusion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <ctime>  // for std::time in RANSAC

namespace fusion_cpp {

SensorFusion::SensorFusion(const YAML::Node& config) {
    // 读取外参
    auto extrinsic = config["extrinsic"]["lidar_to_camera"];
    auto rotation = extrinsic["rotation"];
    auto translation = extrinsic["translation"];

    // 构建变换矩阵
    transform_matrix_ = Eigen::Matrix4f::Identity();
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transform_matrix_(i, j) = rotation[i][j].as<float>();
        }
        transform_matrix_(i, 3) = translation[i].as<float>();
    }

    // 读取相机内参
    auto intrinsic = config["intrinsic"];
    fx_ = intrinsic["fx"].as<float>();
    fy_ = intrinsic["fy"].as<float>();
    cx_ = intrinsic["cx"].as<float>();
    cy_ = intrinsic["cy"].as<float>();

    // 读取融合参数
    auto fusion_config = config["fusion"];
    min_points_in_box_ = fusion_config["min_points_in_box"].as<int>();
    outlier_threshold_ = fusion_config["outlier_threshold"].as<float>();
    use_ground_plane_ = fusion_config["use_ground_plane"].as<bool>();

    // 读取感知范围 (Lidar坐标系)
    // Lidar坐标系: X前, Y左, Z上
    auto range = config["perception_range"];
    lidar_range_.x_min = range["x_min"].as<float>();
    lidar_range_.x_max = range["x_max"].as<float>();
    lidar_range_.y_min = range["y_min"].as<float>();
    lidar_range_.y_max = range["y_max"].as<float>();
    lidar_range_.z_min = range["z_min"].as<float>();
    lidar_range_.z_max = range["z_max"].as<float>();

    // 设置默认尺寸 (长, 宽, 高)
    // 行人: length=胸厚, width=肩宽, height=身高
    default_sizes_["person"] = Eigen::Vector3f(0.22f, 0.44f, 1.68f);  // 中值: 胸厚22cm, 肩宽44cm, 身高168cm
    default_sizes_["bicycle"] = Eigen::Vector3f(1.8f, 0.6f, 1.3f);
    default_sizes_["car"] = Eigen::Vector3f(4.5f, 1.8f, 1.5f);
    default_sizes_["motorcycle"] = Eigen::Vector3f(2.0f, 0.8f, 1.3f);
    default_sizes_["bus"] = Eigen::Vector3f(10.0f, 2.5f, 3.0f);
    default_sizes_["truck"] = Eigen::Vector3f(7.0f, 2.3f, 2.5f);

    // 初始化平滑参数
    smoothing_alpha_ = 0.6f;  // 默认60%当前帧，40%历史帧

    RCLCPP_INFO(rclcpp::get_logger("SensorFusion"), 
                "Sensor fusion module initialized");
}

Eigen::MatrixXf SensorFusion::filterPointsInLidarRange(const Eigen::MatrixXf& points_lidar) {
    if (points_lidar.rows() == 0) {
        return points_lidar;
    }

    std::vector<int> valid_indices;
    valid_indices.reserve(points_lidar.rows());

    for (int i = 0; i < points_lidar.rows(); ++i) {
        const float x = points_lidar(i, 0);
        const float y = points_lidar(i, 1);
        const float z = points_lidar(i, 2);

        // 所有维度使用统一的lidar_range_参数（来自YAML配置）
        if (x >= lidar_range_.x_min && x <= lidar_range_.x_max &&
            y >= lidar_range_.y_min && y <= lidar_range_.y_max &&
            z >= lidar_range_.z_min && z <= lidar_range_.z_max) {
            valid_indices.push_back(i);
        }
    }

    if (valid_indices.empty()) {
        return Eigen::MatrixXf(0, 3);
    }

    Eigen::MatrixXf filtered(valid_indices.size(), 3);
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        filtered.row(i) = points_lidar.row(valid_indices[i]);
        
        // 计算实际范围
        float x = filtered(i, 0);
        float y = filtered(i, 1);
        float z = filtered(i, 2);
        
        x_min = std::min(x_min, x);
        x_max = std::max(x_max, x);
        y_min = std::min(y_min, y);
        y_max = std::max(y_max, y);
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
    }
    
    // 每隔一段时间输出实际有效范围
    static int filter_count = 0;
    filter_count++;

    return filtered;
}

Eigen::MatrixXf SensorFusion::removeGroundPlane(const Eigen::MatrixXf& points) {
    if (points.rows() < 10) {
        return points;  // 点太少，无法进行平面拟合
    }

    // RANSAC参数
    const int max_iterations = 50;           // 迭代次数
    const float distance_threshold = 0.05f;   // 点到平面距离阈值 (5cm)
    const int min_inliers = static_cast<int>(points.rows() * 0.3); // 至少30%的点是地面
    
    Eigen::Vector4f best_plane(0, 0, 1, 0);  // 默认水平平面 z=0
    int best_inlier_count = 0;
    
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    
    // RANSAC迭代
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选择3个点
        std::vector<int> indices;
        indices.reserve(3);
        for (int i = 0; i < 3; ++i) {
            int idx = std::rand() % points.rows();
            indices.push_back(idx);
        }
        
        // 计算平面方程 ax + by + cz + d = 0
        Eigen::Vector3f p1 = points.row(indices[0]);
        Eigen::Vector3f p2 = points.row(indices[1]);
        Eigen::Vector3f p3 = points.row(indices[2]);
        
        Eigen::Vector3f v1 = p2 - p1;
        Eigen::Vector3f v2 = p3 - p1;
        Eigen::Vector3f normal = v1.cross(v2);
        
        if (normal.norm() < 1e-6) {
            continue;  // 三点共线，跳过
        }
        
        normal.normalize();
        float d = -normal.dot(p1);
        
        // 确保法向量朝上（在Lidar坐标系，地面法向量应该主要朝+Z方向）
        if (normal(2) < 0) {
            normal = -normal;
            d = -d;
        }
        
        // 检查是否接近水平面（法向量Z分量应该接近1）
        if (normal(2) < 0.8f) {
            continue;  // 不是接近水平的平面，跳过
        }
        
        // 统计内点
        int inlier_count = 0;
        for (int i = 0; i < points.rows(); ++i) {
            float distance = std::abs(normal.dot(points.row(i)) + d);
            if (distance < distance_threshold) {
                inlier_count++;
            }
        }
        
        // 更新最佳平面
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_plane << normal(0), normal(1), normal(2), d;
        }
    }
    
    // 如果找到了有效的地面平面，移除地面点
    if (best_inlier_count >= min_inliers) {
        std::vector<int> non_ground_indices;
        non_ground_indices.reserve(points.rows());
        
        for (int i = 0; i < points.rows(); ++i) {
            float distance = std::abs(best_plane.head<3>().dot(points.row(i)) + best_plane(3));
            if (distance >= distance_threshold) {
                non_ground_indices.push_back(i);
            }
        }
        
        if (!non_ground_indices.empty()) {
            Eigen::MatrixXf non_ground(non_ground_indices.size(), 3);
            for (size_t i = 0; i < non_ground_indices.size(); ++i) {
                non_ground.row(i) = points.row(non_ground_indices[i]);
            }
            
            return non_ground;
        }
    }
    
    // 如果没有检测到地面或地面点太少，返回原始点云
    return points;
}

Eigen::MatrixXf SensorFusion::transformLidarToCamera(const Eigen::MatrixXf& points_lidar) {
    if (points_lidar.rows() == 0) {
        return Eigen::MatrixXf(0, 3);
    }

    // 先在lidar坐标系下过滤范围
    Eigen::MatrixXf filtered_points = filterPointsInLidarRange(points_lidar);
    
    // 移除地面点
    filtered_points = removeGroundPlane(filtered_points);
    
    if (filtered_points.rows() == 0) {
        return Eigen::MatrixXf(0, 3);
    }

    // 转换为齐次坐标
    Eigen::MatrixXf points_homo(filtered_points.rows(), 4);
    points_homo.leftCols(3) = filtered_points;
    points_homo.col(3).setOnes();

    // 应用变换
    Eigen::MatrixXf points_camera = (transform_matrix_ * points_homo.transpose()).transpose();

    return points_camera.leftCols(3);
}

void SensorFusion::projectPointsToImage(const Eigen::MatrixXf& points_camera,
                                       Eigen::MatrixXf& image_points,
                                       Eigen::VectorXf& depths,
                                       std::vector<int>& valid_indices) {
    if (points_camera.rows() == 0) {
        image_points = Eigen::MatrixXf(0, 2);
        depths = Eigen::VectorXf(0);
        valid_indices.clear();
        return;
    }

    // 过滤相机后面的点
    std::vector<int> valid_idx_temp;
    valid_idx_temp.reserve(points_camera.rows());

    for (int i = 0; i < points_camera.rows(); ++i) {
        if (points_camera(i, 2) > 0) {
            valid_idx_temp.push_back(i);
        }
    }

    if (valid_idx_temp.empty()) {
        image_points = Eigen::MatrixXf(0, 2);
        depths = Eigen::VectorXf(0);
        valid_indices.clear();
        return;
    }

    // 投影到图像平面
    const size_t num_valid = valid_idx_temp.size();
    image_points.resize(num_valid, 2);
    depths.resize(num_valid);
    valid_indices = valid_idx_temp;

    for (size_t i = 0; i < num_valid; ++i) {
        const int idx = valid_idx_temp[i];
        const float z = points_camera(idx, 2);
        const float x = points_camera(idx, 0) / z * fx_ + cx_;
        const float y = points_camera(idx, 1) / z * fy_ + cy_;

        image_points(i, 0) = x;
        image_points(i, 1) = y;
        depths(i) = z;
    }
}

Eigen::MatrixXf SensorFusion::getPointsInBBox(const Eigen::MatrixXf& points_camera,
                                              const std::vector<float>& bbox,
                                              int image_height,
                                              int image_width) {
    if (points_camera.rows() == 0 || bbox.size() != 4) {
        return Eigen::MatrixXf(0, 3);
    }

    // 投影到图像
    Eigen::MatrixXf image_points;
    Eigen::VectorXf depths;
    std::vector<int> valid_indices;
    projectPointsToImage(points_camera, image_points, depths, valid_indices);

    if (image_points.rows() == 0) {
        return Eigen::MatrixXf(0, 3);
    }

    const float x1 = bbox[0];
    const float y1 = bbox[1];
    const float x2 = bbox[2];
    const float y2 = bbox[3];

    // 找到框内的点
    std::vector<int> box_indices;
    std::vector<float> box_depths;  // 记录深度用于后续过滤
    box_indices.reserve(image_points.rows());
    box_depths.reserve(image_points.rows());

    for (int i = 0; i < image_points.rows(); ++i) {
        const float x = image_points(i, 0);
        const float y = image_points(i, 1);

        if (x >= x1 && x <= x2 && y >= y1 && y <= y2 &&
            x >= 0 && x < image_width && y >= 0 && y < image_height) {
            box_indices.push_back(valid_indices[i]);
            box_depths.push_back(depths(i));  // 记录对应的深度
        }
    }

    if (box_indices.empty()) {
        return Eigen::MatrixXf(0, 3);
    }

    // 深度过滤：只保留深度接近中位数的点（过滤背景）
    // 计算深度中位数
    std::vector<float> depths_sorted = box_depths;
    std::nth_element(depths_sorted.begin(), 
                     depths_sorted.begin() + depths_sorted.size() / 2,
                     depths_sorted.end());
    float median_depth = depths_sorted[depths_sorted.size() / 2];
    
    // 【关键修改】使用自适应深度阈值，距离越远，阈值越小（相对比例）
    // 近距离(1-2m): ±0.8m (相对误差40-80%)
    // 中距离(3-4m): ±0.6m (相对误差15-20%)
    // 远距离(>5m):  ±0.5m (相对误差<10%)
    float depth_threshold;
    if (median_depth < 2.5f) {
        depth_threshold = 0.8f;  // 近距离
    } else if (median_depth < 4.0f) {
        depth_threshold = 0.5f;  // 中距离：更严格
    } else {
        depth_threshold = 0.4f;  // 远距离：最严格
    }
    
    // 过滤深度异常的点
    std::vector<int> filtered_indices;
    filtered_indices.reserve(box_indices.size());
    
    for (size_t i = 0; i < box_indices.size(); ++i) {
        if (std::abs(box_depths[i] - median_depth) <= depth_threshold) {
            filtered_indices.push_back(box_indices[i]);
        }
    }
    
    if (filtered_indices.empty()) {
        // 如果过滤后没有点了，使用原始点集
        filtered_indices = box_indices;
    } else {
        RCLCPP_DEBUG(rclcpp::get_logger("SensorFusion"),
                     "Depth filtering: points after=%zu (removed %zu)",
                     filtered_indices.size(), box_indices.size() - filtered_indices.size());
    }

    // 提取框内的3D点
    Eigen::MatrixXf points_in_box(filtered_indices.size(), 3);
    for (size_t i = 0; i < filtered_indices.size(); ++i) {
        points_in_box.row(i) = points_camera.row(filtered_indices[i]);
    }

    // 禁用地面移除和聚类，与Python版本保持一致
    // Python版本直接使用所有框内点，不做额外过滤
    
    return points_in_box;
}

Eigen::MatrixXf SensorFusion::removeOutliers(const Eigen::MatrixXf& points) {
    if (points.rows() < 3) {
        return points;
    }

    // 计算中心
    Eigen::Vector3f center = points.colwise().mean();

    // 分别计算X,Y方向(横向)和Z方向(深度)的离群点
    // X,Y方向使用严格的过滤,Z方向使用更宽松的过滤
    std::vector<int> inlier_indices;
    inlier_indices.reserve(points.rows());
    
    // 1. 先过滤X,Y方向的离群点
    Eigen::VectorXf distances_xy(points.rows());
    for (int i = 0; i < points.rows(); ++i) {
        Eigen::Vector2f diff_xy(points(i, 0) - center(0), points(i, 1) - center(1));
        distances_xy(i) = diff_xy.norm();
    }
    
    // XY方向中位数和标准差
    std::vector<float> distances_xy_vec(distances_xy.data(), 
                                        distances_xy.data() + distances_xy.size());
    std::nth_element(distances_xy_vec.begin(), 
                     distances_xy_vec.begin() + distances_xy_vec.size() / 2,
                     distances_xy_vec.end());
    float median_dist_xy = distances_xy_vec[distances_xy_vec.size() / 2];
    float std_dist_xy = std::sqrt((distances_xy.array() - median_dist_xy).square().mean());
    
    // 2. 再过滤Z方向的离群点(更宽松)
    Eigen::VectorXf z_values = points.col(2);
    std::vector<float> z_vec(z_values.data(), z_values.data() + z_values.size());
    std::nth_element(z_vec.begin(), z_vec.begin() + z_vec.size() / 2, z_vec.end());
    float median_z = z_vec[z_vec.size() / 2];
    float std_z = std::sqrt((z_values.array() - median_z).square().mean());
    
    // XY方向阈值: median + 2*std + outlier_threshold
    float threshold_xy = median_dist_xy + 2.0f * std_dist_xy + outlier_threshold_;
    // Z方向阈值: median ± 3*std (更宽松,允许更大的深度范围)
    float threshold_z_min = median_z - 3.0f * std_z;
    float threshold_z_max = median_z + 3.0f * std_z;

    for (int i = 0; i < points.rows(); ++i) {
        bool xy_ok = distances_xy(i) <= threshold_xy;
        bool z_ok = (points(i, 2) >= threshold_z_min && points(i, 2) <= threshold_z_max);
        if (xy_ok && z_ok) {
            inlier_indices.push_back(i);
        }
    }

    if (inlier_indices.empty()) {
        return points;
    }

    Eigen::MatrixXf inliers(inlier_indices.size(), 3);
    for (size_t i = 0; i < inlier_indices.size(); ++i) {
        inliers.row(i) = points.row(inlier_indices[i]);
    }

    return inliers;
}

Eigen::MatrixXf SensorFusion::extractLargestCluster(const Eigen::MatrixXf& points,
                                                     float cluster_tolerance,
                                                     int min_cluster_size) {
    if (points.rows() < min_cluster_size) {
        return points;
    }

    const int num_points = points.rows();
    std::vector<bool> processed(num_points, false);
    std::vector<std::vector<int>> clusters;

    // 简化版欧几里得聚类
    for (int i = 0; i < num_points; ++i) {
        if (processed[i]) {
            continue;
        }

        std::vector<int> cluster;
        std::vector<int> seeds = {i};
        processed[i] = true;

        // 广度优先搜索
        for (size_t j = 0; j < seeds.size(); ++j) {
            int seed_idx = seeds[j];
            cluster.push_back(seed_idx);

            // 查找邻近点
            for (int k = 0; k < num_points; ++k) {
                if (processed[k]) {
                    continue;
                }

                // 计算欧几里得距离
                float dist = (points.row(seed_idx) - points.row(k)).norm();
                if (dist <= cluster_tolerance) {
                    seeds.push_back(k);
                    processed[k] = true;
                }
            }
        }

        // 只保留足够大的簇
        if (static_cast<int>(cluster.size()) >= min_cluster_size) {
            clusters.push_back(cluster);
        }
    }

    // 如果没有找到有效簇，返回原始点云
    if (clusters.empty()) {
        return points;
    }

    // 找到最大的簇（通常是目标对象）
    size_t max_cluster_idx = 0;
    size_t max_cluster_size = clusters[0].size();
    for (size_t i = 1; i < clusters.size(); ++i) {
        if (clusters[i].size() > max_cluster_size) {
            max_cluster_size = clusters[i].size();
            max_cluster_idx = i;
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("SensorFusion"),
                 "Clustering: found %zu clusters, largest has %zu points (%.1f%% of total)",
                 clusters.size(), max_cluster_size, 
                 100.0 * max_cluster_size / num_points);

    // 提取最大簇的点
    const std::vector<int>& largest_cluster = clusters[max_cluster_idx];
    Eigen::MatrixXf cluster_points(largest_cluster.size(), 3);
    for (size_t i = 0; i < largest_cluster.size(); ++i) {
        cluster_points.row(i) = points.row(largest_cluster[i]);
    }

    return cluster_points;
}

void SensorFusion::adjustSizeByClass(float& length, float& width, float& height,
                                    const std::string& class_name) {
    auto it = default_sizes_.find(class_name);
    if (it == default_sizes_.end()) {
        return;  // 未知类别，不调整
    }

    const Eigen::Vector3f& default_size = it->second;
    const float default_length = default_size(0);
    const float default_width = default_size(1);
    const float default_height = default_size(2);

    // 对行人使用精确的人体尺寸约束（与Python版本一致）
    if (class_name == "person") {
        // 行人尺寸约束范围（基于真实人体数据）
        // length = 胸厚: 18-26cm
        // width = 肩宽: 36-52cm
        // height = 身高: 150-185cm
        const float min_length = 0.18f, max_length = 0.26f;   // 胸厚
        const float min_width = 0.36f, max_width = 0.52f;     // 肩宽
        const float min_height = 1.50f, max_height = 1.85f;   // 身高
        
        // 硬性裁剪到物理合理范围
        length = std::clamp(length, min_length, max_length);
        width = std::clamp(width, min_width, max_width);
        height = std::clamp(height, min_height, max_height);
        
        RCLCPP_DEBUG(rclcpp::get_logger("SensorFusion"),
                     "Person size adjusted - L: %.3f, W: %.3f, H: %.3f",
                     length, width, height);
    } else {
        // 其他类别使用原来的逻辑
        const float weight = 0.7f;
        
        if (height < default_height * 0.5f || height > default_height * 2.0f) {
            height = weight * height + (1.0f - weight) * default_height;
        }
        
        if (length < default_length * 0.3f || length > default_length * 3.0f) {
            length = 0.5f * length + 0.5f * default_length;
        }
        
        if (width < default_width * 0.3f || width > default_width * 3.0f) {
            width = 0.5f * width + 0.5f * default_width;
        }
        
        // 确保最小尺寸
        length = std::max(length, 0.2f);
        width = std::max(width, 0.2f);
        height = std::max(height, 0.3f);
    }
}

bool SensorFusion::estimateFrom2DBBox(const std::vector<float>& bbox_2d,
                                     const std::string& class_name,
                                     BBox3D& bbox_3d) {
    /**
     * 纯2D框估计（当点云完全缺失时）
     * 匹配Python版本的 _estimate_from_2d_bbox 逻辑
     */
    
    if (bbox_2d.size() != 4) {
        return false;
    }
    
    float x1 = bbox_2d[0];
    float y1 = bbox_2d[1];
    float x2 = bbox_2d[2];
    float y2 = bbox_2d[3];
    
    // 简单的深度估计（根据2D框高度）
    float bbox_height_pixels = y2 - y1;
    float depth = 0.0f;
    
    if (class_name == "person") {
        // 假设人的实际高度约1.7米
        float real_height = 1.7f;
        // depth = (real_height * fy) / bbox_height_pixels
        depth = (real_height * fy_) / (bbox_height_pixels + 1e-6f);
    } else if (class_name == "car") {
        float real_height = 1.5f;
        depth = (real_height * fy_) / (bbox_height_pixels + 1e-6f);
    } else if (class_name == "bicycle") {
        float real_height = 1.2f;
        depth = (real_height * fy_) / (bbox_height_pixels + 1e-6f);
    } else {
        // 默认高度
        float real_height = 1.5f;
        depth = (real_height * fy_) / (bbox_height_pixels + 1e-6f);
    }
    
    // 深度范围检查
    if (depth < 0.5f || depth > 15.0f) {
        return false;
    }
    
    // 计算2D框中心在图像上的位置
    float u_center = (x1 + x2) / 2.0f;
    float v_center = (y1 + y2) / 2.0f;
    
    // 反投影到3D（camera坐标系）
    float x = (u_center - cx_) * depth / fx_;
    float y = (v_center - cy_) * depth / fy_;
    float z = depth;
    
    // 根据类别设置默认尺寸
    Eigen::Vector3f size;
    if (class_name == "person") {
        size = Eigen::Vector3f(0.6f, 0.6f, 1.7f);  // [l, w, h]
    } else if (class_name == "car") {
        size = Eigen::Vector3f(4.5f, 1.8f, 1.5f);
    } else if (class_name == "bicycle") {
        size = Eigen::Vector3f(1.8f, 0.6f, 1.2f);
    } else {
        size = Eigen::Vector3f(1.0f, 1.0f, 1.5f);
    }
    
    bbox_3d.center = Eigen::Vector3f(x, y, z);
    bbox_3d.size = size;
    bbox_3d.yaw = 0.0f;  // 默认朝向
    
    return true;
}

bool SensorFusion::estimate3DBBox(const Eigen::MatrixXf& points_3d,
                                 const std::string& class_name,
                                 BBox3D& bbox_3d) {
    // 匹配Python版本：直接使用min_points_in_box_作为阈值
    if (points_3d.rows() < min_points_in_box_) {
        RCLCPP_DEBUG(rclcpp::get_logger("SensorFusion"),
                     "Insufficient points: %ld < %d", points_3d.rows(), min_points_in_box_);
        return false;
    }

    // 移除离群点
    Eigen::MatrixXf filtered_points = removeOutliers(points_3d);

    if (filtered_points.rows() < min_points_in_box_) {
        return false;
    }

    // 使用中位数计算中心点（更鲁棒，抗离群点）
    Eigen::Vector3f center;
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<float> values(filtered_points.rows());
        for (int i = 0; i < filtered_points.rows(); ++i) {
            values[i] = filtered_points(i, axis);
        }
        std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
        center(axis) = values[values.size()/2];
    }

    // 中心化点云
    Eigen::MatrixXf points_centered = filtered_points.rowwise() - center.transpose();

    // 计算协方差矩阵
    Eigen::Matrix3f cov_matrix = (points_centered.transpose() * points_centered) / 
                                 static_cast<float>(points_centered.rows() - 1);

    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov_matrix);
    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues().real();
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors().real();

    // 按特征值从大到小排序
    std::vector<std::pair<float, int>> eigen_pairs;
    for (int i = 0; i < 3; ++i) {
        eigen_pairs.push_back(std::make_pair(eigenvalues(i), i));
    }
    std::sort(eigen_pairs.rbegin(), eigen_pairs.rend());

    // PCA稳定性检查：特征值应该有明显的主方向
    float eigen_ratio = eigen_pairs[0].first / (eigen_pairs[2].first + 1e-6f);
    if (eigen_ratio < 2.0f) {
        // 点云分布太均匀，使用默认尺寸
        auto it = default_sizes_.find(class_name);
        if (it != default_sizes_.end()) {
            bbox_3d.center = center;
            bbox_3d.size = it->second;
            bbox_3d.yaw = 0.0f;
            return true;
        }
        return false;
    }

    Eigen::Matrix3f sorted_eigenvectors;
    Eigen::Vector3f sorted_eigenvalues;
    for (int i = 0; i < 3; ++i) {
        sorted_eigenvalues(i) = eigen_pairs[i].first;
        sorted_eigenvectors.col(i) = eigenvectors.col(eigen_pairs[i].second);
    }

    // 将点转换到主方向坐标系
    Eigen::MatrixXf points_rotated = points_centered * sorted_eigenvectors;

    // 计算边界框（使用百分位数而不是极值，更鲁棒）
    Eigen::Vector3f min_coords, max_coords;
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<float> values(points_rotated.rows());
        for (int i = 0; i < points_rotated.rows(); ++i) {
            values[i] = points_rotated(i, axis);
        }
        std::sort(values.begin(), values.end());
        // 使用5%和95%百分位数，去除极端值
        int idx_5 = static_cast<int>(values.size() * 0.05);
        int idx_95 = static_cast<int>(values.size() * 0.95);
        min_coords(axis) = values[idx_5];
        max_coords(axis) = values[idx_95];
    }
    
    Eigen::Vector3f size_raw = max_coords - min_coords;

    // 识别高度维度 (Y轴分量最大的特征向量)
    Eigen::Vector3f y_components = sorted_eigenvectors.row(1).cwiseAbs();
    int height_idx = 0;
    y_components.maxCoeff(&height_idx);

    float height = size_raw(height_idx);

    // 剩余两个维度是长和宽
    std::vector<int> remaining_indices;
    for (int i = 0; i < 3; ++i) {
        if (i != height_idx) {
            remaining_indices.push_back(i);
        }
    }

    float dim1 = size_raw(remaining_indices[0]);
    float dim2 = size_raw(remaining_indices[1]);

    float length, width;
    int forward_idx;
    if (dim1 > dim2) {
        length = dim1;
        width = dim2;
        forward_idx = remaining_indices[0];
    } else {
        length = dim2;
        width = dim1;
        forward_idx = remaining_indices[1];
    }

    // 计算yaw角 (在XZ水平面，Camera坐标系)
    Eigen::Vector3f forward_vector = sorted_eigenvectors.col(forward_idx);
    float yaw = std::atan2(forward_vector(0), forward_vector(2));

    // 根据类别调整尺寸
    adjustSizeByClass(length, width, height, class_name);

    // 填充结果
    bbox_3d.center = center;
    bbox_3d.size = Eigen::Vector3f(length, width, height);
    bbox_3d.yaw = yaw;

    return true;
}

std::vector<Detection> SensorFusion::fuseDetectionsWithLidar(
    const std::vector<Detection>& detections,
    const Eigen::MatrixXf& points_lidar,
    int image_height,
    int image_width) {
    
    std::vector<Detection> fused_detections;

    // 转换点云到相机坐标系
    Eigen::MatrixXf points_camera = transformLidarToCamera(points_lidar);

    if (points_camera.rows() == 0) {
        RCLCPP_WARN(rclcpp::get_logger("SensorFusion"), 
                    "No valid points in camera frame");
        return fused_detections;
    }

    // 对每个检测框进行融合
    for (const auto& det : detections) {
        Detection fused_det = det;

        // 获取框内的3D点
        Eigen::MatrixXf points_in_box = getPointsInBBox(
            points_camera, det.bbox, image_height, image_width);

        if (points_in_box.rows() >= min_points_in_box_) {
            // 点云充足，使用点云估计3D边界框
            BBox3D bbox_3d;
            if (estimate3DBBox(points_in_box, det.class_name, bbox_3d)) {
                fused_det.bbox_3d = bbox_3d;
                fused_detections.push_back(fused_det);
            }
        } else {
            // 点云过少，尝试使用2D框估计（与Python版本一致）
            RCLCPP_DEBUG(rclcpp::get_logger("SensorFusion"),
                        "Insufficient points (%d < %d), trying 2D bbox estimation for %s",
                        (int)points_in_box.rows(), min_points_in_box_, det.class_name.c_str());
            
            BBox3D bbox_3d;
            if (estimateFrom2DBBox(det.bbox, det.class_name, bbox_3d)) {
                fused_det.bbox_3d = bbox_3d;
                fused_detections.push_back(fused_det);
            }
        }
    }

    return fused_detections;
}

} // namespace fusion_cpp
