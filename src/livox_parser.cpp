#include "fusion_cpp/livox_parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstring>

namespace fusion_cpp {

LivoxParser::LivoxParser() {
    // Constructor
}

Eigen::MatrixXf LivoxParser::parseCustomMsg(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
    
    if (msg->points.empty()) {
        return Eigen::MatrixXf(0, 3);
    }

    const size_t num_points = msg->points.size();
    
    // 预分配内存
    std::vector<Eigen::Vector3f> valid_points;
    valid_points.reserve(num_points);

    // 提取有效点
    for (const auto& point : msg->points) {
        if (isValidPoint(point.x, point.y, point.z)) {
            valid_points.emplace_back(point.x, point.y, point.z);
        }
    }

    // 转换为Eigen矩阵
    const size_t valid_count = valid_points.size();
    Eigen::MatrixXf points(valid_count, 3);
    
    for (size_t i = 0; i < valid_count; ++i) {
        points.row(i) = valid_points[i];
    }

    return points;
}

Eigen::MatrixXf LivoxParser::parsePointCloud2(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    
    if (msg->data.empty()) {
        return Eigen::MatrixXf(0, 3);
    }

    const size_t point_step = msg->point_step;
    const size_t num_points = msg->data.size() / point_step;

    if (num_points == 0) {
        return Eigen::MatrixXf(0, 3);
    }

    std::vector<Eigen::Vector3f> valid_points;
    valid_points.reserve(num_points);

    // 假设点云格式为 x, y, z (每个4字节float)
    const uint8_t* data_ptr = msg->data.data();
    
    for (size_t i = 0; i < msg->data.size(); i += point_step) {
        float x, y, z;
        std::memcpy(&x, data_ptr + i, sizeof(float));
        std::memcpy(&y, data_ptr + i + 4, sizeof(float));
        std::memcpy(&z, data_ptr + i + 8, sizeof(float));

        if (isValidPoint(x, y, z)) {
            valid_points.emplace_back(x, y, z);
        }
    }

    // 转换为Eigen矩阵
    const size_t valid_count = valid_points.size();
    Eigen::MatrixXf points(valid_count, 3);
    
    for (size_t i = 0; i < valid_count; ++i) {
        points.row(i) = valid_points[i];
    }

    return points;
}

} // namespace fusion_cpp
