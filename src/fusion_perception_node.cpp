#include "fusion_cpp/fusion_perception_node.hpp"
#include <filesystem>
#include <chrono>
#include <fstream>
#include <limits.h>
#include <unistd.h>

namespace fusion_cpp {

FusionPerceptionNode::FusionPerceptionNode()
    : Node("fusion_perception_node"),
      frame_skip_(0),
      frame_counter_(0),
      frame_count_(0),
      lidar_received_count_(0),
      last_publish_time_(0.0),
      max_time_diff_(0.2) {
    
    // 获取项目根目录 - 使用环境变量或默认路径
    std::string package_share_dir;
    const char* fusion_cpp_path = std::getenv("FUSION_CPP_PATH");
    if (fusion_cpp_path != nullptr) {
        package_share_dir = std::string(fusion_cpp_path);
    } else {
        package_share_dir = "/home/nvidia/liuwq/fusion_cpp";
    }
    
    RCLCPP_INFO(this->get_logger(), "Project directory: %s", package_share_dir.c_str());
    
    // 加载配置
    std::string config_path = package_share_dir + "/config/fusion_config.yaml";
    loadConfig(config_path);

    // 初始化各模块
    RCLCPP_INFO(this->get_logger(), "Initializing perception modules...");

    livox_parser_ = std::make_unique<LivoxParser>();
    fusion_ = std::make_unique<SensorFusion>(config_);
    tracker_ = std::make_unique<MultiObjectTracker>(config_);

    // 初始化检测器 (如果TensorRT可用)
#ifdef USE_TENSORRT
    try {
        auto detector_config = config_["models"]["detector"];
        std::string model_path = detector_config["model_path"].as<std::string>();
        
        // 转换为绝对路径
        if (model_path[0] != '/') {
            model_path = package_share_dir + "/" + model_path;
        }

        float conf_threshold = detector_config["confidence_threshold"].as<float>();
        float iou_threshold = detector_config["iou_threshold"].as<float>();
        
        std::vector<int> interested_classes;
        for (const auto& cls : detector_config["interested_classes"]) {
            interested_classes.push_back(cls.as<int>());
        }

        detector_ = std::make_unique<TensorRTDetector>(
            model_path, conf_threshold, iou_threshold, interested_classes);
        
        RCLCPP_INFO(this->get_logger(), "TensorRT detector initialized");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize detector: %s", e.what());
        throw;
    }
#else
    RCLCPP_WARN(this->get_logger(), "TensorRT not available, detector disabled");
#endif

    // 加载camera到body的坐标系转换
    loadCameraToBodyTransform(package_share_dir);

    // 读取配置参数
    frame_skip_ = config_["performance"]["frame_skip"].as<int>();
    max_time_diff_ = config_["performance"]["sync_slop"].as<double>();
    
    int queue_size = config_["performance"]["queue_size"].as<int>();
    queue_size = std::max(1, queue_size);

    // 订阅话题
    std::string camera_topic = config_["sensors"]["camera"]["topic"].as<std::string>();
    std::string lidar_topic = config_["sensors"]["lidar"]["topic"].as<std::string>();

    // Camera订阅 (缓存最新帧)
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, queue_size,
        std::bind(&FusionPerceptionNode::cameraCallback, this, std::placeholders::_1));

    // Lidar订阅 (主驱动)
    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic, queue_size,
        std::bind(&FusionPerceptionNode::lidarDrivenCallback, this, 
                 std::placeholders::_1));

    // 发布话题
    std::string pub_topic = config_["publisher"]["topic"].as<std::string>();
    obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        pub_topic, 10);

    // 可视化发布
    if (config_["publisher"]["visualize"].as<bool>()) {
        std::string viz_topic = config_["publisher"]["viz_topic"].as<std::string>();
        viz_pub_ = this->create_publisher<sensor_msgs::msg::Image>(viz_topic, 10);
    }

    RCLCPP_INFO(this->get_logger(), "Fusion Perception Node initialized successfully!");
    RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Lidar topic: %s", lidar_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Queue size: %d", queue_size);
    RCLCPP_INFO(this->get_logger(), "Processing mode: Lidar-driven (10Hz target)");
}

void FusionPerceptionNode::loadConfig(const std::string& config_path) {
    try {
        config_ = YAML::LoadFile(config_path);
        RCLCPP_INFO(this->get_logger(), "Loaded config from: %s", config_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
        throw;
    }
}

void FusionPerceptionNode::loadCameraToBodyTransform(const std::string& project_root) {
    std::string calibration_path = project_root + "/calibration/front_left.yaml";
    
    // 设置默认值
    T_camera_to_body_ = Eigen::Matrix4f::Identity();

    try {
        cv::FileStorage fs(calibration_path, cv::FileStorage::READ);
        
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Failed to open calibration file: %s", calibration_path.c_str());
            RCLCPP_WARN(this->get_logger(), 
                       "Using identity transform (no coordinate transformation)");
            return;
        }

        cv::Mat tbc_mat;
        fs["Tbc"] >> tbc_mat;

        if (tbc_mat.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Tbc matrix not found in calibration file");
            RCLCPP_WARN(this->get_logger(), 
                       "Using identity transform (no coordinate transformation)");
        } else {
            // 检查矩阵类型并转换为Eigen矩阵
            // YAML中dt:f表示float类型
            if (tbc_mat.type() != CV_32F) {
                tbc_mat.convertTo(tbc_mat, CV_32F);
            }
            
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    T_camera_to_body_(i, j) = tbc_mat.at<float>(i, j);
                }
            }
        }

        fs.release();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Error loading camera-to-body transform: %s", e.what());
        RCLCPP_WARN(this->get_logger(), 
                   "Using identity transform (no coordinate transformation)");
        T_camera_to_body_ = Eigen::Matrix4f::Identity();
    }
}

std::vector<Detection> FusionPerceptionNode::transformCameraToBody(
    const std::vector<Detection>& tracked_objects) {
    
    if (tracked_objects.empty()) {
        return {};
    }

    std::vector<Detection> transformed_objects;

    Eigen::Matrix3f R_camera_to_body = T_camera_to_body_.block<3, 3>(0, 0);

    for (const auto& obj : tracked_objects) {
        Detection obj_transformed = obj;
        
        // 转换中心点位置
        Eigen::Vector4f center_camera;
        center_camera << obj.bbox_3d.center(0), obj.bbox_3d.center(1), 
                        obj.bbox_3d.center(2), 1.0f;
        
        // 调试：打印camera坐标系下的位置
        RCLCPP_INFO(this->get_logger(),
                   "[TRANSFORM] Camera coords: X=%.2fm, Y=%.2fm, Z=%.2fm (纵向距离)",
                   obj.bbox_3d.center(0), obj.bbox_3d.center(1), obj.bbox_3d.center(2));
        
        Eigen::Vector4f center_body_homo = T_camera_to_body_ * center_camera;
        Eigen::Vector3f center_body = center_body_homo.head<3>();

        // 转换朝向 (yaw角度)
        float yaw_camera = obj.bbox_3d.yaw;
        
        // Camera坐标系中的朝向向量 (在XZ平面, 水平面)
        Eigen::Vector3f forward_camera;
        forward_camera << std::sin(yaw_camera), 0.0f, std::cos(yaw_camera);

        // 转换到body坐标系
        Eigen::Vector3f forward_body = R_camera_to_body * forward_camera;

        // 计算body坐标系中的yaw角度（在XY平面，水平面）
        float yaw_body = std::atan2(forward_body(1), forward_body(0));

        // 更新bbox_3d
        obj_transformed.bbox_3d.center = center_body;
        obj_transformed.bbox_3d.yaw = yaw_body;
        // size保持不变

        transformed_objects.push_back(obj_transformed);
    }

    return transformed_objects;
}

void FusionPerceptionNode::cameraCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        latest_image_ = cv_ptr->image;
        image_timestamp_ = msg->header.stamp;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Camera callback error: %s", e.what());
    }
}

void FusionPerceptionNode::lidarDrivenCallback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg) {
    
    lidar_received_count_++;

    // 帧跳过逻辑
    if (frame_skip_ > 0) {
        frame_counter_++;
        if (frame_counter_ % (frame_skip_ + 1) != 0) {
            return;
        }
    }

    // 检查是否有camera数据
    if (latest_image_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "No camera data available yet");
        // publishObstacles({});
        return;
    }

    // 检查camera数据时间戳是否太旧
    double lidar_time = lidar_msg->header.stamp.sec + 
                       lidar_msg->header.stamp.nanosec * 1e-9;
    double image_time = image_timestamp_.seconds();
    double time_diff = std::abs(lidar_time - image_time);

    if (time_diff > max_time_diff_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Camera data too old: %.3fs > %.3fs",
                            time_diff, max_time_diff_);
        // publishObstacles({});
        return;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        const int h = latest_image_.rows;
        const int w = latest_image_.cols;

        // 2D目标检测
        auto detection_start = std::chrono::high_resolution_clock::now();
        
        std::vector<Detection> detections_2d;
#ifdef USE_TENSORRT
        detections_2d = detector_->detect(latest_image_);
        
        // // 调试：输出每个2D检测框的详细信息
        // if (!detections_2d.empty()) {
        //     RCLCPP_INFO(this->get_logger(),
        //                "[DEBUG 2D] Got %zu detections:", detections_2d.size());
        //     for (size_t i = 0; i < detections_2d.size() && i < 10; ++i) {
        //         const auto& det = detections_2d[i];
        //         RCLCPP_INFO(this->get_logger(),
        //                    "  [%zu] %s conf=%.3f bbox=[%.1f,%.1f,%.1f,%.1f]",
        //                    i, det.class_name.c_str(), det.confidence,
        //                    det.bbox[0], det.bbox[1], det.bbox[2], det.bbox[3]);
        //     }
        // }
#endif
        
        auto detection_end = std::chrono::high_resolution_clock::now();

        // 解析点云和融合
        std::vector<Detection> detections_3d;
        
        if (!detections_2d.empty()) {
            Eigen::MatrixXf points = livox_parser_->parseCustomMsg(lidar_msg);
            
            if (points.rows() > 0) {
                detections_3d = fusion_->fuseDetectionsWithLidar(
                    detections_2d, points, h, w);
                // RCLCPP_INFO(this->get_logger(),
                //            "[DEBUG] 2D detections from model: %zu, 3D fused objects: %zu",
                //            detections_2d.size(), detections_3d.size());
            }
        }

        // 多目标跟踪
        std::vector<Detection> tracked_objects = tracker_->update(detections_3d);
        // RCLCPP_INFO(this->get_logger(),
        //            "[DEBUG] After tracking: %zu objects",
        //            tracked_objects.size());

        // 坐标转换到body系
        tracked_objects = transformCameraToBody(tracked_objects);

        // 发布结果
        publishObstacles(tracked_objects);

        // 可视化
        if (viz_pub_ && !detections_2d.empty()) {
            cv::Mat viz_image = visualizeResults(
                latest_image_.clone(), detections_2d, tracked_objects);
            
            auto viz_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "bgr8", viz_image).toImageMsg();
            viz_msg->header.stamp = this->now();
            viz_pub_->publish(*viz_msg);
        }

        // 性能统计
        auto end_time = std::chrono::high_resolution_clock::now();
        double process_time = std::chrono::duration<double, std::milli>(
            end_time - start_time).count() / 1000.0;
        
        process_times_.push_back(process_time);
        if (process_times_.size() > 30) {
            process_times_.pop_front();
        }

        frame_count_++;

        // 记录发布频率
        double current_time = this->now().seconds();
        if (last_publish_time_ > 0) {
            double interval = current_time - last_publish_time_;
            publish_intervals_.push_back(interval);
            if (publish_intervals_.size() > 30) {
                publish_intervals_.pop_front();
            }
        }
        last_publish_time_ = current_time;

        // 定期输出统计信息
        if (frame_count_ % 30 == 0) {
            double avg_time = 0.0;
            for (double t : process_times_) avg_time += t;
            avg_time /= process_times_.size();

            if (!publish_intervals_.empty()) {
                double avg_interval = 0.0;
                for (double i : publish_intervals_) avg_interval += i;
                avg_interval /= publish_intervals_.size();
            }

            // RCLCPP_INFO(this->get_logger(),
            //            "Processed %d frames | Lidar received: %d | "
            //            "Total: %.1fms (Det: %.1fms) | "
            //            "Processing FPS: %.1f | Output Hz: %.1f (Target: 10Hz) | "
            //            "Time diff: %.0fms | Det2D: %zu | Tracked: %zu",
            //            frame_count_, lidar_received_count_,
            //            avg_time * 1000, detection_time,
            //            processing_fps, actual_hz, time_diff * 1000,
            //            detections_2d.size(), tracked_objects.size());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Lidar-driven processing error: %s", e.what());
        // publishObstacles({});
    }
}

void FusionPerceptionNode::publishObstacles(
    const std::vector<Detection>& tracked_objects) {
    
    visualization_msgs::msg::MarkerArray marker_array;

    std::string frame_id = config_["publisher"]["frame_id"].as<std::string>();

    for (size_t i = 0; i < tracked_objects.size(); ++i) {
        auto marker = createBBoxMarker(tracked_objects[i], i, frame_id);
        marker_array.markers.push_back(marker);
    }

    // 只有在消息数组为空时才跳过发布
    if (marker_array.markers.empty()) {
        static int empty_count = 0;
        if (++empty_count % 50 == 0) {  // 每50次输出一次警告
            RCLCPP_WARN(this->get_logger(), 
                       "Empty marker array, skipping publish (count: %d)", empty_count);
        }
        return;
    }

    obstacle_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker FusionPerceptionNode::createBBoxMarker(
    const Detection& obj, int id, const std::string& frame_id) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "obstacles";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 位置
    marker.pose.position.x = obj.bbox_3d.center(0);
    marker.pose.position.y = obj.bbox_3d.center(1);
    marker.pose.position.z = obj.bbox_3d.center(2);

    // 朝向
    Eigen::Quaternionf q(Eigen::AngleAxisf(obj.bbox_3d.yaw, Eigen::Vector3f::UnitZ()));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // 尺寸
    marker.scale.x = obj.bbox_3d.size(0);  // length
    marker.scale.y = obj.bbox_3d.size(1);  // width
    marker.scale.z = obj.bbox_3d.size(2);  // height

    // 颜色 (根据类别)
    if (obj.class_name == "person") {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    } else if (obj.class_name == "car") {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    } else {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    }
    marker.color.a = 0.5f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    return marker;
}

cv::Mat FusionPerceptionNode::visualizeResults(
    const cv::Mat& image,
    const std::vector<Detection>& detections_2d,
    const std::vector<Detection>& tracked_objects) {
    
    cv::Mat vis = image.clone();

#ifdef USE_TENSORRT
    vis = detector_->visualize(vis, detections_2d);
#endif

    // 可以添加更多可视化信息（如3D框投影等）

    return vis;
}

} // namespace fusion_cpp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fusion_cpp::FusionPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
