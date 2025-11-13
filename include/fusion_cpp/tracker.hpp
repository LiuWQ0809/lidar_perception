#ifndef FUSION_CPP_TRACKER_HPP_
#define FUSION_CPP_TRACKER_HPP_

#include "fusion_cpp/sensor_fusion.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace fusion_cpp {

/**
 * @brief 卡尔曼滤波器类
 * 状态向量: [x, y, z, vx, vy, vz, l, w, h, yaw] (10维)
 * 测量向量: [x, y, z, l, w, h, yaw] (7维)
 */
class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter() = default;

    void predict();
    void update(const Eigen::VectorXf& measurement);

    Eigen::VectorXf x;  // 状态向量 (10x1)
    Eigen::MatrixXf P;  // 协方差矩阵 (10x10)

private:
    Eigen::MatrixXf F_;  // 状态转移矩阵 (10x10)
    Eigen::MatrixXf H_;  // 测量矩阵 (7x10)
    Eigen::MatrixXf Q_;  // 过程噪声协方差 (10x10)
    Eigen::MatrixXf R_;  // 测量噪声协方差 (7x7)
    Eigen::MatrixXf I_;  // 单位矩阵 (10x10)
};

/**
 * @brief 单个跟踪目标
 */
class Track {
public:
    explicit Track(const Detection& detection, int track_id = -1);
    ~Track() = default;

    void predict();
    void update(const Detection& detection);
    Detection getState() const;

    int track_id;
    std::string class_name;
    int class_id;
    int age;
    int hits;
    int time_since_update;
    Detection last_detection;
    KalmanFilter kf_;

private:
    // 内部实现
};

/**
 * @brief 多目标跟踪器
 */
class MultiObjectTracker {
public:
    explicit MultiObjectTracker(const YAML::Node& config);
    ~MultiObjectTracker() = default;

    /**
     * @brief 更新跟踪
     * @param detections_3d 3D检测结果列表
     * @return 跟踪结果列表
     */
    std::vector<Detection> update(const std::vector<Detection>& detections_3d);

    /**
     * @brief 重置跟踪器
     */
    void reset();

private:
    /**
     * @brief 将检测与跟踪关联
     * @param detections 检测结果
     * @param tracks 跟踪列表
     * @param[out] matched 匹配对 (detection_idx, track_idx)
     * @param[out] unmatched_dets 未匹配的检测索引
     * @param[out] unmatched_trks 未匹配的跟踪索引
     */
    void associateDetectionsToTracks(
        const std::vector<Detection>& detections,
        const std::vector<std::shared_ptr<Track>>& tracks,
        std::vector<std::pair<int, int>>& matched,
        std::vector<int>& unmatched_dets,
        std::vector<int>& unmatched_trks);

    /**
     * @brief 计算检测与跟踪之间的距离
     * @param detection 检测结果
     * @param track 跟踪对象
     * @return 距离
     */
    float calculateDistance(const Detection& detection, const Track& track);

    /**
     * @brief 为未匹配的检测尝试重新绑定到陈旧轨迹
     */
    void recoverUnmatched(const std::vector<Detection>& detections,
                          std::vector<int>& unmatched_dets,
                          std::vector<int>& unmatched_trks);

    /**
     * @brief 根据运动速度对输出姿态平滑
     */
    Detection smoothTrackedState(const std::shared_ptr<Track>& track,
                                 const Detection& raw_state);

    /**
     * @brief 匈牙利算法求解最优分配
     * @param cost_matrix 代价矩阵
     * @param[out] assignment 分配结果
     */
    void hungarianAlgorithm(const Eigen::MatrixXf& cost_matrix,
                           std::vector<int>& assignment);

private:
    int max_age_;
    int min_hits_;
    float iou_threshold_;
    float max_distance_;
    float static_speed_thresh_;
    float smoothing_alpha_static_;
    float smoothing_alpha_dynamic_;

    std::vector<std::shared_ptr<Track>> tracks_;
    int frame_count_;
    static int next_track_id_;
    std::unordered_map<int, Detection> last_smoothed_states_;
};

} // namespace fusion_cpp

#endif // FUSION_CPP_TRACKER_HPP_
