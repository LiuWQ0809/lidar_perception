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
#include <unordered_map>
#include <mutex>

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
    struct CameraContext {
        std::string name;
        std::string topic;
        std::string encoding;
        cv::Size image_size;
        cv::Size calibration_size;
        cv::Size undistort_size;
        cv::Size processed_size;
        bool enabled{true};
        bool is_fisheye{false};
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        cv::Mat undistort_map1;
        cv::Mat undistort_map2;
        Eigen::Matrix4f T_camera_to_body = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f T_body_to_camera = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f T_lidar_to_camera = Eigen::Matrix4f::Identity();
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
        cv::Mat latest_image;
        rclcpp::Time image_timestamp;
        bool has_image{false};
        std::mutex mutex;
    };

private:
    void loadConfig(const std::string& config_path);
    void loadExtrinsic();
    void initializeCameras(const std::string& project_root, int queue_size);
    bool loadCalibrationFile(const std::string& calib_path, CameraContext& camera);

    Detection transformDetectionToBody(const Detection& detection,
                                       const Eigen::Matrix4f& T_camera_to_body);

    void cameraCallback(const std::string& camera_id,
                        const sensor_msgs::msg::Image::SharedPtr msg);
    void lidarDrivenCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    bool getLatestCameraFrame(const std::shared_ptr<CameraContext>& camera,
                              cv::Mat& image,
                              rclcpp::Time& timestamp);

    cv::Mat undistortImage(const cv::Mat& image, CameraContext& camera);

    void publishObstacles(const std::vector<Detection>& tracked_objects);
    cv::Mat visualizeResults(const cv::Mat& image,
                             const std::vector<Detection>& detections_2d,
                             const std::vector<Detection>& tracked_objects);
    visualization_msgs::msg::Marker createBBoxMarker(
        const Detection& obj, int id, const std::string& frame_id);
    void enforceGroundConstraint(Detection& det) const;
    std::vector<Detection> suppressDuplicates(const std::vector<Detection>& detections);

private:
    YAML::Node config_;

    // 模块
    std::unique_ptr<LivoxParser> livox_parser_;
    std::unique_ptr<SensorFusion> fusion_;
    std::unique_ptr<MultiObjectTracker> tracker_;

#ifdef USE_TENSORRT
    std::unique_ptr<TensorRTDetector> detector_;
#endif

    // 坐标转换
    Eigen::Matrix4f T_lidar_to_body_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_body_to_lidar_ = Eigen::Matrix4f::Identity();
    bool has_fallback_extrinsic_ = false;
    Eigen::Matrix4f fallback_lidar_to_camera_ = Eigen::Matrix4f::Identity();
    std::string fallback_camera_name_ = "front_left";
    float ground_level_z_;
    float ground_clamp_margin_;
    float min_person_height_;
    float max_person_height_;
    float duplicate_merge_distance_;

    std::unordered_map<std::string, std::shared_ptr<CameraContext>> cameras_;
    std::vector<std::string> camera_order_;

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
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viz_pub_;
};

} // namespace fusion_cpp

#endif // FUSION_CPP_FUSION_PERCEPTION_NODE_HPP_
