#include "fusion_cpp/fusion_perception_node.hpp"

#include <filesystem>
#include <chrono>
#include <fstream>
#include <limits.h>
#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <system_error>

namespace fusion_cpp {

FusionPerceptionNode::FusionPerceptionNode()
    : Node("fusion_perception_node"),
      frame_skip_(0),
      frame_counter_(0),
      frame_count_(0),
      lidar_received_count_(0),
      last_publish_time_(0.0),
      max_time_diff_(0.2) {

    auto hasValidConfig = [](const std::string& dir) -> bool {
        if (dir.empty()) {
            return false;
        }
        std::error_code ec;
        return std::filesystem::exists(dir + "/config/fusion_config.yaml", ec) && !ec;
    };

    std::string package_share_dir;
    const char* fusion_cpp_path = std::getenv("FUSION_CPP_PATH");
    std::vector<std::string> candidates;
    if (fusion_cpp_path != nullptr) {
        candidates.emplace_back(fusion_cpp_path);
    }
    try {
        candidates.emplace_back(std::filesystem::current_path().string());
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to read current directory for config discovery: %s",
                    e.what());
    }
    candidates.emplace_back("/home/nvidia/liuwq/fusion_cpp");

    for (const auto& candidate : candidates) {
        if (hasValidConfig(candidate)) {
            package_share_dir = candidate;
            if (fusion_cpp_path != nullptr && candidate != fusion_cpp_path) {
                RCLCPP_WARN(this->get_logger(),
                            "FUSION_CPP_PATH points to '%s' but no config found. "
                            "Using '%s' instead.",
                            fusion_cpp_path, candidate.c_str());
            }
            break;
        }
    }

    if (package_share_dir.empty()) {
        throw std::runtime_error(
            "Unable to locate fusion_cpp project directory (missing config/fusion_config.yaml)");
    }

    RCLCPP_INFO(this->get_logger(), "Project directory: %s", package_share_dir.c_str());

    std::string config_path = package_share_dir + "/config/fusion_config.yaml";
    loadConfig(config_path);

    auto range_node = config_["perception_range"];
    ground_level_z_ = range_node && range_node["z_min"]
        ? range_node["z_min"].as<float>()
        : -0.1f;

    auto fusion_node = config_["fusion"];
    ground_clamp_margin_ = (fusion_node && fusion_node["ground_clamp_margin"])
        ? fusion_node["ground_clamp_margin"].as<float>()
        : 0.02f;
    duplicate_merge_distance_ = (fusion_node && fusion_node["duplicate_merge_distance"])
        ? fusion_node["duplicate_merge_distance"].as<float>()
        : 0.9f;

    if (fusion_node && fusion_node["person_height_range"] &&
        fusion_node["person_height_range"].IsSequence() &&
        fusion_node["person_height_range"].size() == 2) {
        min_person_height_ = fusion_node["person_height_range"][0].as<float>();
        max_person_height_ = fusion_node["person_height_range"][1].as<float>();
        if (min_person_height_ > max_person_height_) {
            std::swap(min_person_height_, max_person_height_);
        }
    } else {
        min_person_height_ = 1.45f;
        max_person_height_ = 1.85f;
    }

    livox_parser_ = std::make_unique<LivoxParser>();
    fusion_ = std::make_unique<SensorFusion>(config_);
    tracker_ = std::make_unique<MultiObjectTracker>(config_);

#ifdef USE_TENSORRT
    try {
        auto detector_config = config_["models"]["detector"];
        std::string model_path = detector_config["model_path"].as<std::string>();
        if (!model_path.empty() && model_path[0] != '/') {
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

    loadExtrinsic();

    frame_skip_ = config_["performance"]["frame_skip"].as<int>();
    max_time_diff_ = config_["performance"]["sync_slop"].as<double>();
    int queue_size = config_["performance"]["queue_size"].as<int>();
    queue_size = std::max(1, queue_size);

    initializeCameras(package_share_dir, queue_size);

    auto lidar_topic = config_["sensors"]["lidar"]["topic"].as<std::string>();
    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic, queue_size,
        std::bind(&FusionPerceptionNode::lidarDrivenCallback, this, std::placeholders::_1));

    std::string pub_topic = config_["publisher"]["topic"].as<std::string>();
    obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(pub_topic, 10);

    if (config_["publisher"]["visualize"].as<bool>()) {
        std::string viz_topic = config_["publisher"]["viz_topic"].as<std::string>();
        viz_pub_ = this->create_publisher<sensor_msgs::msg::Image>(viz_topic, 10);
    }

    RCLCPP_INFO(this->get_logger(), "Fusion Perception Node initialized successfully!");
    RCLCPP_INFO(this->get_logger(), "Lidar topic: %s", lidar_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", pub_topic.c_str());
    if (!camera_order_.empty()) {
        std::string camera_list;
        for (const auto& name : camera_order_) {
            const auto& ctx = cameras_.at(name);
            camera_list += name + "(" + ctx->topic + ") ";
        }
        RCLCPP_INFO(this->get_logger(), "Registered cameras: %s", camera_list.c_str());
    }
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

void FusionPerceptionNode::loadExtrinsic() {
    auto extrinsic = config_["extrinsic"];
    if (!extrinsic) {
        throw std::runtime_error("extrinsic section missing in config");
    }

    if (extrinsic["lidar_to_body"]) {
        auto node = extrinsic["lidar_to_body"];
        T_lidar_to_body_ = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                T_lidar_to_body_(i, j) = node["rotation"][i][j].as<float>();
            }
            T_lidar_to_body_(i, 3) = node["translation"][i].as<float>();
        }
        T_body_to_lidar_ = T_lidar_to_body_.inverse();
        has_fallback_extrinsic_ = false;
    } else if (extrinsic["lidar_to_camera"]) {
        auto node = extrinsic["lidar_to_camera"];
        fallback_lidar_to_camera_ = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                fallback_lidar_to_camera_(i, j) = node["rotation"][i][j].as<float>();
            }
            fallback_lidar_to_camera_(i, 3) = node["translation"][i].as<float>();
        }
        fallback_camera_name_ = extrinsic["reference_camera"]
            ? extrinsic["reference_camera"].as<std::string>()
            : std::string("front_left");
        has_fallback_extrinsic_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "extrinsic.lidar_to_camera is deprecated. Provide lidar_to_body instead.");
    } else {
        throw std::runtime_error("No lidar extrinsic provided");
    }
}

bool FusionPerceptionNode::loadCalibrationFile(const std::string& calib_path,
                                               CameraContext& camera) {
    cv::FileStorage fs(calib_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", calib_path.c_str());
        return false;
    }

    cv::FileNode proj = fs["projection_parameters"];
    double fx = proj["fx"].real();
    double fy = proj["fy"].real();
    double cx = proj["cx"].real();
    double cy = proj["cy"].real();
    camera.camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                                                      0.0, fy, cy,
                                                      0.0, 0.0, 1.0);

    cv::FileNode dist = fs["distortion_parameters"];
    std::vector<double> coeffs;
    static const std::vector<std::string> keys = {"k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"};
    for (const auto& key : keys) {
        if (dist[key].empty()) {
            coeffs.push_back(0.0);
        } else {
            coeffs.push_back(dist[key].real());
        }
    }
    camera.dist_coeffs = cv::Mat(coeffs).reshape(1, 1).clone();

    std::string camera_type;
    fs["CameraType"] >> camera_type;
    if (camera.is_fisheye == false) {
        camera.is_fisheye = (camera_type == "kb" || camera_type == "fisheye");
    }

    if (camera.calibration_size.width <= 0 || camera.calibration_size.height <= 0) {
        camera.calibration_size.width = fs["orign_x"].empty() ? camera.image_size.width : static_cast<int>(fs["orign_x"].real());
        camera.calibration_size.height = fs["orign_y"].empty() ? camera.image_size.height : static_cast<int>(fs["orign_y"].real());
    }

    cv::Mat Tbc;
    fs["Tbc"] >> Tbc;
    fs.release();

    if (Tbc.type() != CV_32F) {
        Tbc.convertTo(Tbc, CV_32F);
    }

    if (Tbc.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Calibration file missing Tbc: %s", calib_path.c_str());
        return false;
    }

    camera.T_camera_to_body = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            camera.T_camera_to_body(i, j) = Tbc.at<float>(i, j);
        }
    }
    camera.T_body_to_camera = camera.T_camera_to_body.inverse();
    return true;
}

void FusionPerceptionNode::initializeCameras(const std::string& project_root, int queue_size) {
    auto sensors = config_["sensors"];
    if (!sensors || !sensors["cameras"]) {
        throw std::runtime_error("sensors.cameras not configured");
    }

    auto cameras_node = sensors["cameras"];
    for (const auto& node : cameras_node) {
        if (!node || (node["enabled"] && !node["enabled"].as<bool>())) {
            continue;
        }

        auto camera = std::make_shared<CameraContext>();
        camera->name = node["name"].as<std::string>();
        camera->topic = node["topic"].as<std::string>();
        camera->encoding = node["encoding"] ? node["encoding"].as<std::string>() : std::string("bgr8");
        auto image_size = node["image_size"];
        camera->image_size = cv::Size(image_size[0].as<int>(), image_size[1].as<int>());
        if (node["calibration_size"]) {
            camera->calibration_size = cv::Size(node["calibration_size"][0].as<int>(),
                                                node["calibration_size"][1].as<int>());
        }
        if (node["undistort_size"]) {
            camera->undistort_size = cv::Size(node["undistort_size"][0].as<int>(),
                                              node["undistort_size"][1].as<int>());
        }
        camera->processed_size = camera->undistort_size.width > 0 ? camera->undistort_size : camera->image_size;
        if (node["projection_model"]) {
            std::string model = node["projection_model"].as<std::string>();
            camera->is_fisheye = (model == "fisheye");
        }

        std::string calib_path = node["calibration"].as<std::string>();
        if (!calib_path.empty() && calib_path[0] != '/') {
            calib_path = project_root + "/" + calib_path;
        }

        if (!loadCalibrationFile(calib_path, *camera)) {
            throw std::runtime_error("Failed to load calibration for camera " + camera->name);
        }

        double scale_x = (camera->calibration_size.width > 0)
            ? static_cast<double>(camera->image_size.width) / camera->calibration_size.width
            : 1.0;
        double scale_y = (camera->calibration_size.height > 0)
            ? static_cast<double>(camera->image_size.height) / camera->calibration_size.height
            : 1.0;

        camera->camera_matrix.at<double>(0, 0) *= scale_x;
        camera->camera_matrix.at<double>(1, 1) *= scale_y;
        camera->camera_matrix.at<double>(0, 2) *= scale_x;
        camera->camera_matrix.at<double>(1, 2) *= scale_y;

        cv::Size map_size = camera->processed_size;
        if (map_size.width <= 0 || map_size.height <= 0) {
            map_size = camera->image_size;
        }

        if (camera->is_fisheye) {
            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat new_K = camera->camera_matrix.clone();
            cv::Mat dist = camera->dist_coeffs.colRange(0, 4).clone();
            cv::fisheye::initUndistortRectifyMap(
                camera->camera_matrix, dist, R, new_K,
                map_size, CV_16SC2, camera->undistort_map1, camera->undistort_map2);
        } else {
            cv::initUndistortRectifyMap(
                camera->camera_matrix, camera->dist_coeffs, cv::Mat(),
                camera->camera_matrix, map_size, CV_16SC2,
                camera->undistort_map1, camera->undistort_map2);
        }

        cameras_[camera->name] = camera;
        camera_order_.push_back(camera->name);
    }

    if (cameras_.empty()) {
        throw std::runtime_error("No camera enabled in configuration");
    }

    if (has_fallback_extrinsic_) {
        auto it = cameras_.find(fallback_camera_name_);
        if (it == cameras_.end()) {
            throw std::runtime_error("Fallback camera " + fallback_camera_name_ + " not found");
        }
        T_lidar_to_body_ = it->second->T_camera_to_body * fallback_lidar_to_camera_;
        T_body_to_lidar_ = T_lidar_to_body_.inverse();
        has_fallback_extrinsic_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "Computed lidar_to_body from fallback camera %s",
                    fallback_camera_name_.c_str());
    }

    if (T_body_to_lidar_.isApprox(Eigen::Matrix4f::Identity())) {
        RCLCPP_WARN(this->get_logger(),
                    "Using identity transform for lidar_to_body. Please check calibration.");
    }

    for (auto& [name, camera] : cameras_) {
        camera->T_body_to_camera = camera->T_camera_to_body.inverse();
        camera->T_lidar_to_camera = camera->T_body_to_camera * T_lidar_to_body_;

        SensorFusion::CameraModel model;
        model.transform_matrix = camera->T_lidar_to_camera;
        model.fx = static_cast<float>(camera->camera_matrix.at<double>(0, 0));
        model.fy = static_cast<float>(camera->camera_matrix.at<double>(1, 1));
        model.cx = static_cast<float>(camera->camera_matrix.at<double>(0, 2));
        model.cy = static_cast<float>(camera->camera_matrix.at<double>(1, 2));
        fusion_->registerCamera(name, model);

        camera->subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            camera->topic, queue_size,
            [this, name](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->cameraCallback(name, msg);
            });
    }
}

cv::Mat FusionPerceptionNode::undistortImage(const cv::Mat& image, CameraContext& camera) {
    if (camera.undistort_map1.empty() || camera.undistort_map2.empty()) {
        return image.clone();
    }
    cv::Mat result;
    cv::remap(image, result, camera.undistort_map1, camera.undistort_map2, cv::INTER_LINEAR);
    return result;
}

bool FusionPerceptionNode::getLatestCameraFrame(const std::shared_ptr<CameraContext>& camera,
                                                cv::Mat& image,
                                                rclcpp::Time& timestamp) {
    std::lock_guard<std::mutex> lock(camera->mutex);
    if (!camera->has_image || camera->latest_image.empty()) {
        return false;
    }
    camera->latest_image.copyTo(image);
    timestamp = camera->image_timestamp;
    return true;
}

Detection FusionPerceptionNode::transformDetectionToBody(
    const Detection& detection,
    const Eigen::Matrix4f& T_camera_to_body) {
    Detection transformed = detection;
    Eigen::Vector4f center_camera;
    center_camera << detection.bbox_3d.center(0),
                     detection.bbox_3d.center(1),
                     detection.bbox_3d.center(2), 1.0f;

    Eigen::Vector4f center_body = T_camera_to_body * center_camera;
    Eigen::Matrix3f R_camera_to_body = T_camera_to_body.block<3, 3>(0, 0);

    Eigen::Vector3f forward_camera;
    forward_camera << std::sin(detection.bbox_3d.yaw),
                      0.0f,
                      std::cos(detection.bbox_3d.yaw);
    Eigen::Vector3f forward_body = R_camera_to_body * forward_camera;
    float yaw_body = std::atan2(forward_body(1), forward_body(0));

    transformed.bbox_3d.center = center_body.head<3>();
    transformed.bbox_3d.yaw = yaw_body;
    return transformed;
}

void FusionPerceptionNode::enforceGroundConstraint(Detection& det) const {
    float height = det.bbox_3d.size(2);
    if (det.class_name == "person") {
        height = std::clamp(height, min_person_height_, max_person_height_);
    } else {
        height = std::clamp(height, 0.5f, 4.0f);
    }
    det.bbox_3d.size(2) = height;

    const float half_height = height * 0.5f;
    const float min_bottom = ground_level_z_ + ground_clamp_margin_;
    float bottom = det.bbox_3d.center(2) - half_height;
    if (bottom < min_bottom) {
        det.bbox_3d.center(2) += (min_bottom - bottom);
    }
}

std::vector<Detection> FusionPerceptionNode::suppressDuplicates(
    const std::vector<Detection>& detections) {
    if (detections.size() <= 1) {
        return detections;
    }

    std::vector<Detection> merged_results;
    std::vector<bool> consumed(detections.size(), false);

    for (size_t i = 0; i < detections.size(); ++i) {
        if (consumed[i]) {
            continue;
        }

        const auto& base = detections[i];
        Eigen::Vector3f center_sum = base.bbox_3d.center;
        Eigen::Vector3f size_sum = base.bbox_3d.size;
        float sin_sum = std::sin(base.bbox_3d.yaw);
        float cos_sum = std::cos(base.bbox_3d.yaw);
        float best_conf = base.confidence;
        size_t best_idx = i;
        int count = 1;

        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (consumed[j]) {
                continue;
            }
            if (detections[j].class_name != base.class_name) {
                continue;
            }

            Eigen::Vector3f diff = detections[j].bbox_3d.center - base.bbox_3d.center;
            float planar_dist = diff.head<2>().norm();
            float height_diff = std::abs(diff(2));
            float dyn_thresh = std::max(duplicate_merge_distance_,
                                        0.25f * (detections[j].bbox_3d.size(0) + detections[j].bbox_3d.size(1) +
                                                 base.bbox_3d.size(0) + base.bbox_3d.size(1)));
            if (planar_dist > dyn_thresh || height_diff > 0.8f) {
                continue;
            }

            consumed[j] = true;
            center_sum += detections[j].bbox_3d.center;
            size_sum += detections[j].bbox_3d.size;
            sin_sum += std::sin(detections[j].bbox_3d.yaw);
            cos_sum += std::cos(detections[j].bbox_3d.yaw);
            if (detections[j].confidence > best_conf) {
                best_conf = detections[j].confidence;
                best_idx = j;
            }
            ++count;
        }

        Detection fused = detections[best_idx];
        fused.bbox_3d.center = center_sum / static_cast<float>(count);
        fused.bbox_3d.size = size_sum / static_cast<float>(count);
        if (std::abs(sin_sum) > 1e-3f || std::abs(cos_sum) > 1e-3f) {
            fused.bbox_3d.yaw = std::atan2(sin_sum, cos_sum);
        }
        enforceGroundConstraint(fused);
        merged_results.push_back(fused);
    }

    return merged_results;
}

void FusionPerceptionNode::cameraCallback(
    const std::string& camera_id,
    const sensor_msgs::msg::Image::SharedPtr msg) {
    auto it = cameras_.find(camera_id);
    if (it == cameras_.end()) {
        return;
    }
    auto& camera = it->second;

    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat image = cv_ptr->image;
        if (image.cols != camera->image_size.width || image.rows != camera->image_size.height) {
            cv::resize(image, image, camera->image_size);
        }
        cv::Mat undistorted = undistortImage(image, *camera);
        {
            std::lock_guard<std::mutex> lock(camera->mutex);
            camera->latest_image = undistorted;
            camera->image_timestamp = msg->header.stamp;
            camera->has_image = true;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Camera %s callback error: %s",
                     camera_id.c_str(), e.what());
    }
}

void FusionPerceptionNode::lidarDrivenCallback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg) {
    lidar_received_count_++;

    if (frame_skip_ > 0) {
        frame_counter_++;
        if (frame_counter_ % (frame_skip_ + 1) != 0) {
            return;
        }
    }

    double lidar_time = lidar_msg->header.stamp.sec +
                        lidar_msg->header.stamp.nanosec * 1e-9;

    auto start_time = std::chrono::high_resolution_clock::now();

    Eigen::MatrixXf points = livox_parser_->parseCustomMsg(lidar_msg);
    std::vector<Detection> detections_body;
    bool any_camera_ready = false;

    for (const auto& name : camera_order_) {
        auto camera_it = cameras_.find(name);
        if (camera_it == cameras_.end()) {
            continue;
        }
        auto& camera = camera_it->second;

        cv::Mat image;
        rclcpp::Time image_stamp;
        if (!getLatestCameraFrame(camera, image, image_stamp)) {
            continue;
        }

        double image_time = image_stamp.seconds();
        double time_diff = std::abs(lidar_time - image_time);
        if (time_diff > max_time_diff_) {
            continue;
        }
        any_camera_ready = true;

        std::vector<Detection> detections_2d;
#ifdef USE_TENSORRT
        detections_2d = detector_->detect(image);
#endif
        if (detections_2d.empty()) {
            continue;
        }
        for (auto& det : detections_2d) {
            det.camera_id = name;
        }

        std::vector<Detection> detections_3d = fusion_->fuseDetectionsWithLidar(
            name, detections_2d, points, image.rows, image.cols);

        for (auto& det3d : detections_3d) {
            det3d.camera_id = name;
            auto det_body = transformDetectionToBody(det3d, camera->T_camera_to_body);
            enforceGroundConstraint(det_body);
            detections_body.push_back(det_body);
        }

        if (viz_pub_ && !detections_2d.empty()) {
            cv::Mat viz_image = visualizeResults(image.clone(), detections_2d, detections_body);
            auto viz_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", viz_image).toImageMsg();
            viz_msg->header.stamp = image_stamp;
            viz_msg->header.frame_id = name;
            viz_pub_->publish(*viz_msg);
        }
    }

    if (!any_camera_ready) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No fresh camera data for fusion");
        return;
    }

    auto deduped_detections = suppressDuplicates(detections_body);
    auto tracked_objects = tracker_->update(deduped_detections);
    publishObstacles(tracked_objects);

    auto end_time = std::chrono::high_resolution_clock::now();
    double process_time = std::chrono::duration<double, std::milli>(
                              end_time - start_time)
                              .count() /
                          1000.0;
    process_times_.push_back(process_time);
    if (process_times_.size() > 30) {
        process_times_.pop_front();
    }

    frame_count_++;
    double current_time = this->now().seconds();
    if (last_publish_time_ > 0) {
        double interval = current_time - last_publish_time_;
        publish_intervals_.push_back(interval);
        if (publish_intervals_.size() > 30) {
            publish_intervals_.pop_front();
        }
    }
    last_publish_time_ = current_time;
}

void FusionPerceptionNode::publishObstacles(
    const std::vector<Detection>& tracked_objects) {
    visualization_msgs::msg::MarkerArray marker_array;
    std::string frame_id = config_["publisher"]["frame_id"].as<std::string>();

    for (size_t i = 0; i < tracked_objects.size(); ++i) {
        auto marker = createBBoxMarker(tracked_objects[i], static_cast<int>(i), frame_id);
        marker_array.markers.push_back(marker);
    }

    if (marker_array.markers.empty()) {
        static int empty_count = 0;
        if (++empty_count % 50 == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Empty marker array, skipping publish (count: %d)", empty_count);
        }
        return;
    }

    obstacle_pub_->publish(marker_array);
}

cv::Mat FusionPerceptionNode::visualizeResults(
    const cv::Mat& image,
    const std::vector<Detection>& detections_2d,
    const std::vector<Detection>& tracked_objects) {
    cv::Mat vis = image.clone();
#ifdef USE_TENSORRT
    vis = detector_->visualize(vis, detections_2d);
#endif
    (void)tracked_objects;
    return vis;
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

    marker.pose.position.x = obj.bbox_3d.center(0);
    marker.pose.position.y = obj.bbox_3d.center(1);
    marker.pose.position.z = obj.bbox_3d.center(2);

    Eigen::Quaternionf q(Eigen::AngleAxisf(obj.bbox_3d.yaw, Eigen::Vector3f::UnitZ()));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = obj.bbox_3d.size(0);
    marker.scale.y = obj.bbox_3d.size(1);
    marker.scale.z = obj.bbox_3d.size(2);

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

}  // namespace fusion_cpp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fusion_cpp::FusionPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
