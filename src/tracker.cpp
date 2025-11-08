#include "fusion_cpp/tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <limits>
#include <cmath>

namespace fusion_cpp {

// ================ KalmanFilter Implementation ================

KalmanFilter::KalmanFilter() : x(10), P(10, 10), F_(10, 10), H_(7, 10), 
                                Q_(10, 10), R_(7, 7), I_(10, 10) {
    // 初始化状态向量
    x.setZero();

    // 状态转移矩阵
    const float dt = 0.1f;  // 假设10Hz
    F_.setIdentity();
    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;

    // 测量矩阵
    H_.setZero();
    H_(0, 0) = 1.0f;  // x
    H_(1, 1) = 1.0f;  // y
    H_(2, 2) = 1.0f;  // z
    H_(3, 6) = 1.0f;  // l
    H_(4, 7) = 1.0f;  // w
    H_(5, 8) = 1.0f;  // h
    H_(6, 9) = 1.0f;  // yaw

    // 过程噪声协方差（降低以提高稳定性，与Python一致）
    Q_.setIdentity();
    Q_ *= 0.05f;  // 基础过程噪声
    Q_.block<3, 3>(3, 3) *= 0.6f;  // 速度噪声 (0.05 * 0.6 = 0.03)
    Q_.block<3, 3>(6, 6) *= 10.0f;  // 尺寸噪声 (0.05 * 10 = 0.5)

    // 测量噪声协方差（增加以信任预测，减少测量跳变，与Python一致）
    R_.setIdentity();
    R_ *= 0.8f;  // 基础测量噪声
    R_.block<3, 3>(0, 0) *= 0.5f;  // 位置测量噪声 (0.8 * 0.5 = 0.4)
    R_.block<3, 3>(3, 3) *= 0.8f;  // 尺寸测量噪声 (0.8 * 0.8 = 0.64)
    R_(6, 6) = 0.6f;  // 角度测量噪声

    // 初始协方差
    P.setIdentity();
    P *= 10.0f;

    // 单位矩阵
    I_.setIdentity();
}

void KalmanFilter::predict() {
    // 预测状态
    Eigen::VectorXf x_pred = F_ * x;
    
    // 物理约束：限制单帧位置变化（防止预测跳变）
    const float max_position_change = 2.0f;  // 单帧最大位移2米（10Hz下相当于20m/s）
    for (int i = 0; i < 3; ++i) {  // x, y, z
        float delta = x_pred(i) - x(i);
        if (std::abs(delta) > max_position_change) {
            x_pred(i) = x(i) + (delta > 0 ? max_position_change : -max_position_change);
        }
    }
    
    // 限制速度的突变（加速度约束）
    const float max_acceleration = 10.0f;  // 最大加速度 10 m/s^2
    const float dt = 0.1f;
    const float max_velocity_change = max_acceleration * dt;  // 1.0 m/s per frame
    for (int i = 3; i <= 5; ++i) {  // vx, vy, vz
        float delta_v = x_pred(i) - x(i);
        if (std::abs(delta_v) > max_velocity_change) {
            x_pred(i) = x(i) + (delta_v > 0 ? max_velocity_change : -max_velocity_change);
        }
    }
    
    x = x_pred;
    
    // 预测协方差
    P = F_ * P * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXf& measurement) {
    // 测量残差
    Eigen::VectorXf y = measurement - H_ * x;

    // 残差协方差
    Eigen::MatrixXf S = H_ * P * H_.transpose() + R_;

    // 卡尔曼增益
    Eigen::MatrixXf K = P * H_.transpose() * S.inverse();

    // 更新状态
    x = x + K * y;

    // 更新协方差
    P = (I_ - K * H_) * P;
    
    // 物理约束：限制速度和位置变化
    // 行人最大速度约10 m/s (极限跑步)，车辆约30 m/s
    const float max_velocity = 15.0f;  // m/s
    for (int i = 3; i <= 5; ++i) {  // vx, vy, vz
        if (std::abs(x(i)) > max_velocity) {
            x(i) = (x(i) > 0) ? max_velocity : -max_velocity;
        }
    }
    
    // 限制尺寸变化（物理尺寸不会突变）
    // 长、宽、高应该在合理范围内
    x(6) = std::max(0.1f, std::min(x(6), 15.0f));  // length [0.1, 15]m
    x(7) = std::max(0.1f, std::min(x(7), 5.0f));   // width [0.1, 5]m
    x(8) = std::max(0.3f, std::min(x(8), 5.0f));   // height [0.3, 5]m
}

// ================ Track Implementation ================

Track::Track(const Detection& detection, int track_id) 
    : track_id(track_id), class_name(detection.class_name),
      class_id(detection.class_id), age(1), hits(1), time_since_update(0),
      last_detection(detection) {
    
    // 初始化卡尔曼滤波器状态
    const auto& center = detection.bbox_3d.center;
    const auto& size = detection.bbox_3d.size;
    float yaw = detection.bbox_3d.yaw;

    kf_.x << center(0), center(1), center(2),  // x, y, z
             0.0f, 0.0f, 0.0f,                  // vx, vy, vz
             size(0), size(1), size(2),         // l, w, h
             yaw;                                 // yaw
}

void Track::predict() {
    kf_.predict();
    age++;
    time_since_update++;
}

void Track::update(const Detection& detection) {
    const auto& center = detection.bbox_3d.center;
    const auto& size = detection.bbox_3d.size;
    float yaw = detection.bbox_3d.yaw;

    // 测量向量
    Eigen::VectorXf z(7);
    z << center(0), center(1), center(2),
         size(0), size(1), size(2),
         yaw;

    kf_.update(z);
    time_since_update = 0;
    hits++;
    last_detection = detection;
}

Detection Track::getState() const {
    Detection state;
    
    const Eigen::VectorXf& x = kf_.x;
    
    state.bbox_3d.center = Eigen::Vector3f(x(0), x(1), x(2));
    state.bbox_3d.size = Eigen::Vector3f(x(6), x(7), x(8));
    state.bbox_3d.yaw = x(9);
    state.bbox_3d.velocity = Eigen::Vector3f(x(3), x(4), x(5));
    
    state.class_name = class_name;
    state.class_id = class_id;
    state.confidence = last_detection.confidence;
    state.bbox = last_detection.bbox;
    
    return state;
}

// ================ MultiObjectTracker Implementation ================

int MultiObjectTracker::next_track_id_ = 1;

MultiObjectTracker::MultiObjectTracker(const YAML::Node& config) 
    : frame_count_(0) {
    
    auto tracking_config = config["tracking"];
    max_age_ = tracking_config["max_age"].as<int>();
    min_hits_ = tracking_config["min_hits"].as<int>();
    iou_threshold_ = tracking_config["iou_threshold"].as<float>();
    max_distance_ = tracking_config["max_distance"].as<float>();

    RCLCPP_INFO(rclcpp::get_logger("MultiObjectTracker"), 
                "Multi-object tracker initialized");
}

std::vector<Detection> MultiObjectTracker::update(
    const std::vector<Detection>& detections_3d) {
    
    frame_count_++;

    // 预测所有跟踪的下一帧状态
    for (auto& track : tracks_) {
        track->predict();
    }

    // 数据关联
    if (!detections_3d.empty()) {
        std::vector<std::pair<int, int>> matched;
        std::vector<int> unmatched_dets;
        std::vector<int> unmatched_trks;

        associateDetectionsToTracks(detections_3d, tracks_, 
                                   matched, unmatched_dets, unmatched_trks);

        // 更新匹配的跟踪
        for (const auto& match : matched) {
            int det_idx = match.first;
            int trk_idx = match.second;
            tracks_[trk_idx]->update(detections_3d[det_idx]);
        }

        // 创建新跟踪
        for (int det_idx : unmatched_dets) {
            auto new_track = std::make_shared<Track>(
                detections_3d[det_idx], next_track_id_++);
            tracks_.push_back(new_track);
        }
    }

    // 删除长时间未更新的跟踪
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [this](const std::shared_ptr<Track>& t) {
                return t->time_since_update >= max_age_;
            }),
        tracks_.end()
    );

    // 返回稳定的跟踪结果
    std::vector<Detection> tracked_objects;
    for (const auto& track : tracks_) {
        if (track->hits >= min_hits_ || frame_count_ <= min_hits_) {
            tracked_objects.push_back(track->getState());
        }
    }

    return tracked_objects;
}

void MultiObjectTracker::associateDetectionsToTracks(
    const std::vector<Detection>& detections,
    const std::vector<std::shared_ptr<Track>>& tracks,
    std::vector<std::pair<int, int>>& matched,
    std::vector<int>& unmatched_dets,
    std::vector<int>& unmatched_trks) {
    
    matched.clear();
    unmatched_dets.clear();
    unmatched_trks.clear();

    if (tracks.empty()) {
        for (size_t i = 0; i < detections.size(); ++i) {
            unmatched_dets.push_back(i);
        }
        return;
    }

    if (detections.empty()) {
        for (size_t i = 0; i < tracks.size(); ++i) {
            unmatched_trks.push_back(i);
        }
        return;
    }

    // 计算代价矩阵
    Eigen::MatrixXf cost_matrix(detections.size(), tracks.size());
    for (size_t d = 0; d < detections.size(); ++d) {
        for (size_t t = 0; t < tracks.size(); ++t) {
            cost_matrix(d, t) = calculateDistance(detections[d], *tracks[t]);
        }
    }

    // 使用匈牙利算法求解
    std::vector<int> assignment(detections.size(), -1);
    hungarianAlgorithm(cost_matrix, assignment);

    // 初始化未匹配列表
    for (size_t i = 0; i < detections.size(); ++i) {
        unmatched_dets.push_back(i);
    }
    for (size_t i = 0; i < tracks.size(); ++i) {
        unmatched_trks.push_back(i);
    }

    // 过滤距离过大的匹配
    for (size_t d = 0; d < assignment.size(); ++d) {
        int t = assignment[d];
        if (t >= 0 && cost_matrix(d, t) < max_distance_) {
            matched.push_back({static_cast<int>(d), t});
            unmatched_dets.erase(
                std::remove(unmatched_dets.begin(), unmatched_dets.end(), d),
                unmatched_dets.end());
            unmatched_trks.erase(
                std::remove(unmatched_trks.begin(), unmatched_trks.end(), t),
                unmatched_trks.end());
        }
    }
}

float MultiObjectTracker::calculateDistance(const Detection& detection, 
                                            const Track& track) {
    const Eigen::Vector3f& det_center = detection.bbox_3d.center;
    Eigen::Vector3f trk_center = Eigen::Vector3f(
        track.kf_.x(0), track.kf_.x(1), track.kf_.x(2));
    
    // 位置距离
    float pos_distance = (det_center - trk_center).norm();
    
    // 考虑速度方向一致性（用于转弯场景）
    Eigen::Vector3f trk_velocity = Eigen::Vector3f(
        track.kf_.x(3), track.kf_.x(4), track.kf_.x(5));
    float velocity_norm = trk_velocity.norm();
    
    // 如果track有明显运动（>0.5m/s），考虑运动方向
    if (velocity_norm > 0.5f) {
        // 预测位置（考虑速度）
        Eigen::Vector3f predicted_center = trk_center + trk_velocity * 0.1f; // dt=0.1s
        float predicted_distance = (det_center - predicted_center).norm();
        
        // 使用位置距离和预测距离的较小值（更宽容）
        pos_distance = std::min(pos_distance, predicted_distance);
    }
    
    // 尺寸一致性（防止误匹配）
    const Eigen::Vector3f& det_size = detection.bbox_3d.size;
    Eigen::Vector3f trk_size = Eigen::Vector3f(
        track.kf_.x(6), track.kf_.x(7), track.kf_.x(8));
    
    float size_diff = (det_size - trk_size).norm();
    float avg_size = (det_size.norm() + trk_size.norm()) / 2.0f;
    float size_penalty = (avg_size > 0.1f) ? (size_diff / avg_size) : 0.0f;
    
    // 类别一致性
    float class_penalty = (detection.class_name == track.class_name) ? 0.0f : 5.0f;
    
    // 综合代价：位置 + 尺寸差异 + 类别惩罚
    return pos_distance + size_penalty * 0.5f + class_penalty;
}

void MultiObjectTracker::hungarianAlgorithm(const Eigen::MatrixXf& cost_matrix,
                                            std::vector<int>& assignment) {
    // 简化的匈牙利算法实现（贪婪近似）
    // 对于生产环境，建议使用完整的库实现
    
    const int rows = cost_matrix.rows();
    const int cols = cost_matrix.cols();
    
    assignment.resize(rows, -1);
    std::vector<bool> col_assigned(cols, false);

    // 贪婪匹配：对每一行找最小代价的未分配列
    for (int r = 0; r < rows; ++r) {
        int best_col = -1;
        float best_cost = std::numeric_limits<float>::max();

        for (int c = 0; c < cols; ++c) {
            if (!col_assigned[c] && cost_matrix(r, c) < best_cost) {
                best_cost = cost_matrix(r, c);
                best_col = c;
            }
        }

        if (best_col >= 0) {
            assignment[r] = best_col;
            col_assigned[best_col] = true;
        }
    }
}

void MultiObjectTracker::reset() {
    tracks_.clear();
    frame_count_ = 0;
    next_track_id_ = 1;
    RCLCPP_INFO(rclcpp::get_logger("MultiObjectTracker"), "Tracker reset");
}

} // namespace fusion_cpp
