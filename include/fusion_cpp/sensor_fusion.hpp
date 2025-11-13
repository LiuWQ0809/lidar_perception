#ifndef FUSION_CPP_SENSOR_FUSION_HPP_
#define FUSION_CPP_SENSOR_FUSION_HPP_

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace fusion_cpp {

/**
 * @brief 3D边界框结构
 */
struct BBox3D {
    Eigen::Vector3f center;  // 中心点 [x, y, z]
    Eigen::Vector3f size;    // 尺寸 [length, width, height]
    float yaw;               // 偏航角 (在水平面)
    Eigen::Vector3f velocity; // 速度 [vx, vy, vz]
    
    BBox3D() : center(Eigen::Vector3f::Zero()), 
               size(Eigen::Vector3f::Zero()), 
               yaw(0.0f),
               velocity(Eigen::Vector3f::Zero()) {}
};

/**
 * @brief 检测结果结构
 */
struct Detection {
    std::vector<float> bbox;  // 2D检测框 [x1, y1, x2, y2]
    float confidence;
    int class_id;
    std::string class_name;
    BBox3D bbox_3d;
    std::string camera_id;  // 来源相机
    
    Detection() : confidence(0.0f), class_id(-1) {}
};

/**
 * @brief 感知范围配置
 */
struct PerceptionRange {
    float x_min, x_max;
    float y_min, y_max;
    float z_min, z_max;
};

/**
 * @brief 传感器融合模块
 * 融合相机2D检测和Lidar点云，生成3D障碍物信息
 */
class SensorFusion {
public:
    struct CameraModel {
        Eigen::Matrix4f transform_matrix;  // lidar -> camera
        float fx;
        float fy;
        float cx;
        float cy;
    };

    /**
     * @brief 构造函数
     * @param config YAML配置
     */
    explicit SensorFusion(const YAML::Node& config);
    ~SensorFusion() = default;

    /**
     * @brief 注册相机模型（外参+内参）
     */
    void registerCamera(const std::string& camera_id, const CameraModel& model);

    /**
     * @brief 将Lidar点云从lidar坐标系转换到camera坐标系
     * @param points_lidar 输入点云 (N x 3)
     * @param model 相机模型
     * @return 转换后的点云 (M x 3)，可能经过感知范围过滤
     */
    Eigen::MatrixXf transformLidarToCamera(const Eigen::MatrixXf& points_lidar,
                                           const CameraModel& model);

    /**
     * @brief 将相机坐标系的3D点投影到图像平面
     * @param points_camera 相机坐标系点云 (N x 3)
     * @param model 相机模型
     * @param[out] image_points 投影后的图像坐标 (M x 2)
     * @param[out] depths 深度值 (M)
     * @param[out] valid_indices 有效点的原始索引 (M)
     */
    void projectPointsToImage(const Eigen::MatrixXf& points_camera,
                             const CameraModel& model,
                             Eigen::MatrixXf& image_points,
                             Eigen::VectorXf& depths,
                             std::vector<int>& valid_indices);

    /**
     * @brief 获取检测框内的3D点
     * @param points_camera 相机坐标系点云 (N x 3)
     * @param bbox 2D检测框 [x1, y1, x2, y2]
     * @param image_height 图像高度
     * @param image_width 图像宽度
     * @return 框内的3D点 (M x 3)
     */
    Eigen::MatrixXf getPointsInBBox(const Eigen::MatrixXf& points_camera,
                                    const std::vector<float>& bbox,
                                    int image_height,
                                    int image_width,
                                    const CameraModel& model);

    /**
     * @brief 基于3D点云估计3D边界框
     * @param points_3d 3D点云 (N x 3)
     * @param class_name 目标类别
     * @param[out] bbox_3d 输出的3D边界框
     * @return true if成功
     */
    bool estimate3DBBox(const Eigen::MatrixXf& points_3d,
                       const std::string& class_name,
                       BBox3D& bbox_3d);

    /**
     * @brief 从2D边界框估计3D边界框（当点云过少时的备用方案）
     * @param bbox_2d 2D边界框 [x1, y1, x2, y2]
     * @param class_name 目标类别
     * @param[out] bbox_3d 输出的3D边界框
     * @return true if成功
     */
    bool estimateFrom2DBBox(const std::vector<float>& bbox_2d,
                           const std::string& class_name,
                           const CameraModel& model,
                           BBox3D& bbox_3d);

    /**
     * @brief 融合多个检测结果与点云数据
     * @param camera_id 相机ID
     * @param detections 2D检测结果列表
     * @param points_lidar Lidar点云 (N x 3)
     * @param image_height 图像高度
     * @param image_width 图像宽度
     * @return 融合后的检测结果列表（包含3D信息）
     */
    std::vector<Detection> fuseDetectionsWithLidar(
        const std::string& camera_id,
        const std::vector<Detection>& detections,
        const Eigen::MatrixXf& points_lidar,
        int image_height,
        int image_width);

private:
    /**
     * @brief 在lidar坐标系下过滤感知范围内的点
     * @param points_lidar 输入点云
     * @return 过滤后的点云
     */
    Eigen::MatrixXf filterPointsInLidarRange(const Eigen::MatrixXf& points_lidar);

    /**
     * @brief 移除离群点
     * @param points 输入点云
     * @return 过滤后的点云
     */
    Eigen::MatrixXf removeOutliers(const Eigen::MatrixXf& points);

    /**
     * @brief 使用RANSAC算法检测并移除地面点
     * @param points 输入点云（Lidar坐标系）
     * @return 移除地面后的点云
     */
    Eigen::MatrixXf removeGroundPlane(const Eigen::MatrixXf& points);

    /**
     * @brief 根据类别调整尺寸
     * @param length, width, height 原始尺寸
     * @param class_name 类别名称
     */
    void adjustSizeByClass(float& length, float& width, float& height,
                          const std::string& class_name);

    /**
     * @brief 使用欧几里得聚类提取最大簇（前景目标）
     * @param points 输入点云
     * @param cluster_tolerance 聚类距离阈值（米）
     * @param min_cluster_size 最小簇大小
     * @return 最大簇的点云
     */
    Eigen::MatrixXf extractLargestCluster(const Eigen::MatrixXf& points,
                                          float cluster_tolerance = 0.3f,
                                          int min_cluster_size = 5);

private:
    // 已注册的相机模型
    std::unordered_map<std::string, CameraModel> camera_models_;

    // 融合参数
    int min_points_in_box_;
    float outlier_threshold_;
    bool use_ground_plane_;  // 是否使用地面移除

    // 感知范围（Lidar坐标系，从YAML配置文件读取）
    PerceptionRange lidar_range_;

    // 类别默认尺寸 (length, width, height)
    std::map<std::string, Eigen::Vector3f> default_sizes_;

    // 历史平滑：存储每个track_id的上一帧3D bbox
    std::map<int, BBox3D> history_bboxes_;
    
    // 时间平滑参数
    float smoothing_alpha_;  // 平滑系数，0=完全使用历史，1=完全使用当前
};

} // namespace fusion_cpp

#endif // FUSION_CPP_SENSOR_FUSION_HPP_
