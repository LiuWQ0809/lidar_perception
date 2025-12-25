#ifndef FUSION_CPP_LIVOX_PARSER_HPP_
#define FUSION_CPP_LIVOX_PARSER_HPP_

#include <vector>
#include <Eigen/Dense>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace fusion_cpp {

/**
 * @brief Lidar点云数据解析器
 * 解析PointCloud2消息
 */
class LivoxParser {
public:
    LivoxParser();
    ~LivoxParser() = default;

    /**
     * @brief 解析标准PointCloud2消息
     * @param msg PointCloud2消息
     * @return 点云矩阵 (N x 3), 每行为 [x, y, z]
     */
    Eigen::MatrixXf parsePointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

private:
    /**
     * @brief 检查点是否有效
     * @param x, y, z 点坐标
     * @return true if valid
     */
    inline bool isValidPoint(float x, float y, float z) const {
        return !std::isnan(x) && !std::isnan(y) && !std::isnan(z) &&
               !std::isinf(x) && !std::isinf(y) && !std::isinf(z);
    }
};

} // namespace fusion_cpp

#endif // FUSION_CPP_LIVOX_PARSER_HPP_
