#include "fusion_cpp/livox_parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstring>

namespace fusion_cpp {

LivoxParser::LivoxParser() {
    // Constructor
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

    // 直接预分配Eigen矩阵，避免中间vector
    Eigen::MatrixXf points(num_points / downsample_ratio_ + 1, 3);
    size_t valid_count = 0;

    const uint8_t* data_ptr = msg->data.data();
    const size_t step = point_step * downsample_ratio_;
    
    for (size_t i = 0; i < msg->data.size(); i += step) {
        float x, y, z;
        std::memcpy(&x, data_ptr + i, sizeof(float));
        std::memcpy(&y, data_ptr + i + 4, sizeof(float));
        std::memcpy(&z, data_ptr + i + 8, sizeof(float));

        if (isValidPoint(x, y, z)) {
            points(valid_count, 0) = x;
            points(valid_count, 1) = y;
            points(valid_count, 2) = z;
            valid_count++;
        }
    }

    if (valid_count < num_points) {
        return points.topRows(valid_count);
    }

    return points;
}

} // namespace fusion_cpp
