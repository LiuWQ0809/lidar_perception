#ifndef FUSION_CPP_FUSION_PERCEPTION_NODE_HPP_
#define FUSION_CPP_FUSION_PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "fusion_cpp/livox_parser.hpp"
#include "fusion_cpp/sensor_fusion.hpp"
#include "fusion_cpp/tracker.hpp"

#ifdef USE_TENSORRT
#include "fusion_cpp/detector_tensorrt.hpp"
#endif

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <deque>

namespace fusion_cpp {

/**
 * @brief 融合感知ROS2节点
 * 订阅相机和Lidar数据，发布3D障碍物信息
 */
class FusionPerceptionNode : public rclcpp::Node {
public:
    FusionPerceptionNode();
    ~FusionPerceptionNode() = default;

private:
    /**
     * @brief 加载配置文件
     * @param config_path 配置文件路径
     */
    void loadConfig(const std::string& config_path);

    /**
     * @brief 加载camera到body的坐标系转换
     * @param project_root 项目根目录
     */
    void loadCameraToBodyTransform(const std::string& project_root);

    /**
     * @brief 将跟踪目标从camera坐标系转换到body坐标系
     * @param tracked_objects 输入的跟踪对象
     * @return 转换后的对象
     */
    std::vector<Detection> transformCameraToBody(
        const std::vector<Detection>& tracked_objects);

    /**
     * @brief 相机回调
     */
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Lidar驱动回调
     */
    void lidarDrivenCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    /**
     * @brief 发布障碍物标记
     */
    void publishObstacles(const std::vector<Detection>& tracked_objects);

    /**
     * @brief 可视化检测结果
     */
    cv::Mat visualizeResults(const cv::Mat& image,
                            const std::vector<Detection>& detections_2d,
                            const std::vector<Detection>& tracked_objects);

    /**
     * @brief 创建边界框标记
     */
    visualization_msgs::msg::Marker createBBoxMarker(
        const Detection& obj, int id, const std::string& frame_id);

private:
    // 配置
    YAML::Node config_;

    // 模块
    std::unique_ptr<LivoxParser> livox_parser_;
    std::unique_ptr<SensorFusion> fusion_;
    std::unique_ptr<MultiObjectTracker> tracker_;

#ifdef USE_TENSORRT
    std::unique_ptr<TensorRTDetector> detector_;
#endif

    // 坐标转换矩阵 (camera to body)
    Eigen::Matrix4f T_camera_to_body_;

    // 数据缓存
    cv::Mat latest_image_;
    rclcpp::Time image_timestamp_;
    
    // 跳帧计数
    int frame_skip_;
    int frame_counter_;

    // 性能统计
    int frame_count_;
    int lidar_received_count_;
    std::deque<double> process_times_;
    std::deque<double> publish_intervals_;
    double last_publish_time_;
    double max_time_diff_;

    // ROS订阅和发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viz_pub_;
};

} // namespace fusion_cpp

#endif // FUSION_CPP_FUSION_PERCEPTION_NODE_HPP_
