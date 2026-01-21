#include "fusion_cpp/tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <limits>
#include <cmath>
#include <unordered_set>

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

    // 过程噪声协方差（平衡平滑度与响应速度）
    Q_.setIdentity();
    Q_ *= 0.005f;  // 极低的位置过程噪声，强制平滑
    Q_.block<3, 3>(3, 3) *= 4.0f;  // 允许速度有一定变化 (0.02)
    Q_.block<3, 3>(6, 6) *= 0.2f;  // 尺寸应该非常稳定

    // 测量噪声协方差（增大以过滤观测抖动，特别是运动时的中心跳变）
    R_.setIdentity();
    R_ *= 4.0f;  // 进一步显著增大测量噪声，更信任运动模型
    R_.block<3, 3>(0, 0) *= 2.0f;  // 位置测量噪声 (8.0)
    R_.block<3, 3>(3, 3) *= 2.0f;  // 尺寸测量噪声 (8.0)
    R_(6, 6) = 2.0f;  // 角度测量噪声

    // 初始协方差
    P.setIdentity();
    P *= 10.0f;

    // 单位矩阵
    I_.setIdentity();
}

void KalmanFilter::predict(float dt) {
    // 更新状态转移矩阵中的 dt
    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;

    // 预测状态
    Eigen::VectorXf x_pred = F_ * x;
    
    // 物理约束：限制单帧位置变化（防止预测跳变）
    const float max_velocity_limit = 20.0f; // 20m/s
    const float max_position_change = max_velocity_limit * dt;
    for (int i = 0; i < 3; ++i) {  // x, y, z
        float delta = x_pred(i) - x(i);
        if (std::abs(delta) > max_position_change) {
            x_pred(i) = x(i) + (delta > 0 ? max_position_change : -max_position_change);
        }
    }
    
    // 限制速度的突变（加速度约束）
    const float max_acceleration = 8.0f;  // 最大加速度 8 m/s^2
    const float max_velocity_change = max_acceleration * dt;
    for (int i = 3; i <= 5; ++i) {  // vx, vy, vz
        float delta_v = x_pred(i) - x(i);
        if (std::abs(delta_v) > max_velocity_change) {
            x_pred(i) = x(i) + (delta_v > 0 ? max_velocity_change : -max_velocity_change);
        }
    }
    
    x = x_pred;
    
    // 预测协方差 (考虑 dt 对过程噪声的影响)
    // 简单的 Q 缩放：dt 越大，不确定性增加越快
    Eigen::MatrixXf Q_scaled = Q_ * (dt / 0.1f);
    P = F_ * P * F_.transpose() + Q_scaled;
}

void KalmanFilter::update(const Eigen::VectorXf& measurement) {
    // 测量残差
    Eigen::VectorXf y = measurement - H_ * x;

    // 【关键修复】角度残差归一化
    while (y(6) > M_PI) y(6) -= 2.0f * M_PI;
    while (y(6) < -M_PI) y(6) += 2.0f * M_PI;

    // 残差协方差
    Eigen::MatrixXf S = H_ * P * H_.transpose() + R_;

    // 【优化】测量门限 (Gating)
    // 计算马氏距离，剔除物理上不可能的突变观测
    float mahalanobis_dist = y.transpose() * S.inverse() * y;
    if (mahalanobis_dist > 14.07f) { // 7自由度下95%置信度阈值
        // 如果偏差过大，可能是误检测或严重遮挡，减小更新权重
        // 这里不直接 return，而是通过增大 R 来降低本次更新的影响
        S += R_ * 2.0f;
    }

    // 卡尔曼增益
    Eigen::MatrixXf K = P * H_.transpose() * S.inverse();

    // 更新状态
    x = x + K * y;

    // 更新后再次归一化状态中的Yaw角
    while (x(9) > M_PI) x(9) -= 2.0f * M_PI;
    while (x(9) < -M_PI) x(9) += 2.0f * M_PI;

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

void Track::predict(float dt) {
    kf_.predict(dt);
    age++;
    time_since_update++;
}

void Track::update(const Detection& detection) {
    const auto& center = detection.bbox_3d.center;
    const auto& size = detection.bbox_3d.size;
    float yaw = detection.bbox_3d.yaw;

    // 【关键优化】针对行人的运动学约束和朝向估计
    if (class_name == "person") {
        // 1. 速度方向锁定朝向：行人运动时，朝向应趋向于速度方向
        Eigen::Vector2f velocity(kf_.x(3), kf_.x(4));
        if (velocity.norm() > 0.4f) { // 速度大于 0.4m/s 时更新朝向
            float velocity_yaw = std::atan2(velocity(1), velocity(0));
            // 将速度方向作为 yaw 的观测值，但给予较小的权重（通过增大 R 间接实现，或者直接平滑）
            yaw = velocity_yaw;
        } else {
            // 静止时，保持上一时刻朝向，不接受测量更新
            yaw = kf_.x(9);
        }
        
        // 2. 物理限制：行人不可能有极高的加速度或速度
        // 这里的限制会在 kf_.update 内部的物理约束中起作用
    }

    // 测量向量
    Eigen::VectorXf z(7);
    z << center(0), center(1), center(2),
         size(0), size(1), size(2),
         yaw;

    kf_.update(z);
    
    // 如果是行人，在更新后再次强化物理约束
    if (class_name == "person") {
        // 限制行人最大速度为 3m/s (约 10km/h)
        float speed = kf_.x.segment<2>(3).norm();
        if (speed > 3.0f) {
            kf_.x.segment<2>(3) *= (3.0f / speed);
        }
    }

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
    state.camera_id = last_detection.camera_id;
    
    return state;
}

// ================ MultiObjectTracker Implementation ================

int MultiObjectTracker::next_track_id_ = 1;

MultiObjectTracker::MultiObjectTracker(const YAML::Node& config) 
    : frame_count_(0), last_timestamp_(-1.0) {
    
    auto tracking_config = config["tracking"];
    max_age_ = tracking_config["max_age"].as<int>();
    min_hits_ = tracking_config["min_hits"].as<int>();
    iou_threshold_ = tracking_config["iou_threshold"].as<float>();
    max_distance_ = tracking_config["max_distance"].as<float>();
    static_speed_thresh_ = tracking_config["static_speed_threshold"]
        ? tracking_config["static_speed_threshold"].as<float>()
        : 0.3f;
    smoothing_alpha_static_ = tracking_config["smoothing_alpha_static"]
        ? tracking_config["smoothing_alpha_static"].as<float>()
        : 0.2f;
    smoothing_alpha_dynamic_ = tracking_config["smoothing_alpha_dynamic"]
        ? tracking_config["smoothing_alpha_dynamic"].as<float>()
        : 0.6f;

    RCLCPP_INFO(rclcpp::get_logger("MultiObjectTracker"), 
                "Multi-object tracker initialized");
}

std::vector<Detection> MultiObjectTracker::update(
    const std::vector<Detection>& detections_3d, double timestamp) {
    
    frame_count_++;

    // 计算时间增量 dt
    float dt = 0.1f; // 默认 10Hz
    if (last_timestamp_ > 0) {
        dt = static_cast<float>(timestamp - last_timestamp_);
    }
    // 限制 dt 范围，防止异常时间戳导致滤波器发散
    dt = std::clamp(dt, 0.01f, 0.5f);
    last_timestamp_ = timestamp;

    // 预测所有跟踪的下一帧状态
    for (auto& track : tracks_) {
        track->predict(dt);
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

        // 对未匹配对尝试恢复，避免分裂
        recoverUnmatched(detections_3d, unmatched_dets, unmatched_trks);

        // 创建新跟踪
        for (int det_idx : unmatched_dets) {
            auto new_track = std::make_shared<Track>(
                detections_3d[det_idx], next_track_id_++);
            tracks_.push_back(new_track);
        }
    }

    // 【关键优化】合并距离过近的跟踪，防止目标分裂
    mergeCloseTracks();

    // 删除长时间未更新的跟踪
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [this](const std::shared_ptr<Track>& t) {
                if (t->time_since_update >= max_age_) {
                    last_smoothed_states_.erase(t->track_id);
                    return true;
                }
                return false;
            }),
        tracks_.end()
    );

    // 返回稳定的跟踪结果
    std::vector<Detection> tracked_objects;
    for (const auto& track : tracks_) {
        if (track->hits >= min_hits_ || frame_count_ <= min_hits_) {
            Detection raw_state = track->getState();
            tracked_objects.push_back(smoothTrackedState(track, raw_state));
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
        if (t >= 0) {
            float dist_thresh = max_distance_;
            // 【优化】针对行人的匹配阈值收紧
            if (detections[d].class_name == "person") {
                dist_thresh = std::min(dist_thresh, 1.5f); // 行人匹配半径限制在 1.5m
            }
            
            if (cost_matrix(d, t) < dist_thresh) {
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
}

float MultiObjectTracker::calculateIoU(const std::vector<float>& box1, const std::vector<float>& box2) {
    if (box1.size() != 4 || box2.size() != 4) return 0.0f;
    float x1 = std::max(box1[0], box2[0]);
    float y1 = std::max(box1[1], box2[1]);
    float x2 = std::min(box1[2], box2[2]);
    float y2 = std::min(box1[3], box2[3]);
    float width = std::max(0.0f, x2 - x1);
    float height = std::max(0.0f, y2 - y1);
    float intersection = width * height;
    float area1 = (box1[2] - box1[0]) * (box1[3] - box1[1]);
    float area2 = (box2[2] - box2[0]) * (box2[3] - box2[1]);
    float union_area = area1 + area2 - intersection;
    return (union_area > 0) ? (intersection / union_area) : 0.0f;
}

float MultiObjectTracker::calculateDistance(const Detection& detection, 
                                            const Track& track) {
    const Eigen::Vector3f& det_center = detection.bbox_3d.center;
    Eigen::Vector3f trk_center = Eigen::Vector3f(
        track.kf_.x(0), track.kf_.x(1), track.kf_.x(2));
    
    // 1. 3D 位置距离
    float pos_distance = (det_center - trk_center).norm();
    
    // 2. 2D IoU 贡献 (ID 稳定性的关键)
    float iou_penalty = 1.0f;
    if (detection.camera_id == track.last_detection.camera_id) {
        float iou = calculateIoU(detection.bbox, track.last_detection.bbox);
        // IoU 越大，惩罚越小。如果 IoU > 0.5，大幅降低代价
        if (iou > 0.5f) {
            iou_penalty = 0.2f; 
        } else if (iou > 0.1f) {
            iou_penalty = 0.5f;
        } else {
            iou_penalty = 1.5f; // IoU 太小，增加惩罚
        }
    }
    
    // 3. 考虑速度方向一致性
    Eigen::Vector3f trk_velocity = Eigen::Vector3f(
        track.kf_.x(3), track.kf_.x(4), track.kf_.x(5));
    float velocity_norm = trk_velocity.norm();
    
    if (velocity_norm > 0.5f) {
        Eigen::Vector3f predicted_center = trk_center + trk_velocity * 0.1f;
        float predicted_distance = (det_center - predicted_center).norm();
        
        Eigen::Vector3f move_dir = (det_center - trk_center).normalized();
        Eigen::Vector3f vel_dir = trk_velocity.normalized();
        float cos_theta = move_dir.dot(vel_dir);
        
        if (cos_theta < 0.0f) {
            pos_distance += 1.5f; // 运动方向相反，重罚
        }
        
        pos_distance = 0.5f * pos_distance + 0.5f * predicted_distance;
    }
    
    // 4. 综合代价
    // 使用 IoU 惩罚来缩放位置距离，实现 2D-3D 混合关联
    float total_cost = pos_distance * iou_penalty;
    
    // 5. 类别一致性
    if (detection.class_name != track.class_name) {
        total_cost += 5.0f;
    }
    
    // 6. 针对行人的特殊处理
    if (detection.class_name == "person") {
        // 行人匹配更看重 2D 框的连续性
        if (detection.camera_id == track.last_detection.camera_id) {
            float iou = calculateIoU(detection.bbox, track.last_detection.bbox);
            if (iou > 0.6f) total_cost *= 0.5f; // 2D 框非常吻合，极力保持 ID
        }
    }
    
    return total_cost;
}

void MultiObjectTracker::hungarianAlgorithm(const Eigen::MatrixXf& cost_matrix,
                                            std::vector<int>& assignment) {
    // 【优化】使用全局最近邻 (Global Nearest Neighbor) 贪婪匹配
    // 相比原来的逐行贪婪，这里对所有可能的匹配按代价排序，优先选择全局最小代价的匹配
    // 这在无法使用标准匈牙利算法库的情况下，提供了更好的关联效果
    
    const int rows = cost_matrix.rows();
    const int cols = cost_matrix.cols();
    
    assignment.assign(rows, -1);
    std::vector<bool> row_occupied(rows, false);
    std::vector<bool> col_occupied(cols, false);

    struct Match {
        int r;
        int c;
        float cost;
        // 按代价从小到大排序
        bool operator<(const Match& other) const { return cost < other.cost; }
    };

    // 收集所有可能的匹配
    std::vector<Match> all_matches;
    all_matches.reserve(rows * cols);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            // 只考虑在最大距离阈值内的匹配，减少排序量
            // 注意：这里使用稍微宽松的阈值，具体的严格阈值判定在外部进行
            if (cost_matrix(r, c) < max_distance_ * 2.0f) {
                all_matches.push_back({r, c, cost_matrix(r, c)});
            }
        }
    }

    // 排序
    std::sort(all_matches.begin(), all_matches.end());

    // 贪婪分配
    for (const auto& m : all_matches) {
        if (!row_occupied[m.r] && !col_occupied[m.c]) {
            assignment[m.r] = m.c;
            row_occupied[m.r] = true;
            col_occupied[m.c] = true;
        }
    }
}

void MultiObjectTracker::recoverUnmatched(const std::vector<Detection>& detections,
                                          std::vector<int>& unmatched_dets,
                                          std::vector<int>& unmatched_trks) {
    if (unmatched_dets.empty() || unmatched_trks.empty()) {
        return;
    }

    std::vector<int> remaining_dets;
    std::unordered_set<int> recovered_tracks;

    for (int det_idx : unmatched_dets) {
        float best_dist = max_distance_ * 1.5f;
        int best_track = -1;
        for (int trk_idx : unmatched_trks) {
            if (recovered_tracks.count(trk_idx)) {
                continue;
            }
            const auto& track = tracks_[trk_idx];
            Eigen::Vector3f track_center(track->kf_.x(0), track->kf_.x(1), track->kf_.x(2));
            float dist = (detections[det_idx].bbox_3d.center - track_center).norm();
            if (dist < best_dist) {
                best_dist = dist;
                best_track = trk_idx;
            }
        }

        if (best_track >= 0) {
            tracks_[best_track]->update(detections[det_idx]);
            recovered_tracks.insert(best_track);
        } else {
            remaining_dets.push_back(det_idx);
        }
    }

    std::vector<int> remaining_trks;
    for (int trk_idx : unmatched_trks) {
        if (!recovered_tracks.count(trk_idx)) {
            remaining_trks.push_back(trk_idx);
        }
    }

    unmatched_dets.swap(remaining_dets);
    unmatched_trks.swap(remaining_trks);
}

void MultiObjectTracker::mergeCloseTracks() {
    if (tracks_.size() < 2) return;

    std::vector<bool> to_remove(tracks_.size(), false);
    
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (to_remove[i]) continue;
        
        for (size_t j = i + 1; j < tracks_.size(); ++j) {
            if (to_remove[j]) continue;
            
            // 只有同类别的才合并
            if (tracks_[i]->class_name != tracks_[j]->class_name) continue;
            
            Eigen::Vector3f pos_i(tracks_[i]->kf_.x(0), tracks_[i]->kf_.x(1), tracks_[i]->kf_.x(2));
            Eigen::Vector3f pos_j(tracks_[j]->kf_.x(0), tracks_[j]->kf_.x(1), tracks_[j]->kf_.x(2));
            
            float dist = (pos_i - pos_j).norm();
            
            // 【优化】针对行人的合并阈值放宽
            float merge_thresh = 0.8f;
            if (tracks_[i]->class_name == "person") {
                merge_thresh = 1.2f; // 1.2m 内的行人轨迹合并
            }

            if (dist < merge_thresh) {
                // 合并策略：保留 hits 更多、更成熟的轨迹
                if (tracks_[i]->hits >= tracks_[j]->hits) {
                    to_remove[j] = true;
                } else {
                    to_remove[i] = true;
                    break; 
                }
            }
        }
    }

    // 执行删除
    int removed_count = 0;
    for (int i = static_cast<int>(tracks_.size()) - 1; i >= 0; --i) {
        if (to_remove[i]) {
            last_smoothed_states_.erase(tracks_[i]->track_id);
            tracks_.erase(tracks_.begin() + i);
            removed_count++;
        }
    }
    
    if (removed_count > 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("MultiObjectTracker"), "Merged %d close tracks", removed_count);
    }
}

Detection MultiObjectTracker::smoothTrackedState(const std::shared_ptr<Track>& track,
                                                 const Detection& raw_state) {
    Detection smoothed = raw_state;
    
    // 优化：根据速度动态调整平滑系数
    // 速度越低，平滑越强（alpha越小），减少静止物体的抖动
    float planar_speed = raw_state.bbox_3d.velocity.head<2>().norm();
    float alpha = (planar_speed < static_speed_thresh_)
        ? smoothing_alpha_static_ : smoothing_alpha_dynamic_;
    
    // 【优化】针对行人的特殊平滑处理
    if (track->class_name == "person") {
        // 行人步态会导致中心点高频抖动，需要更强的平滑
        alpha *= 0.5f; 
        // 限制行人 alpha 范围在 [0.05, 0.2]
        alpha = std::clamp(alpha, 0.05f, 0.20f);
    } else {
        // 其他目标限制在 [0.1, 0.5]
        alpha = std::clamp(alpha, 0.1f, 0.5f); 
    }

    auto it = last_smoothed_states_.find(track->track_id);
    if (it != last_smoothed_states_.end()) {
        const auto& prev = it->second;
        
        // 【优化】位置平滑：考虑预测位移，减少滞后
        Eigen::Vector3f predicted_center = prev.bbox_3d.center + prev.bbox_3d.velocity * 0.1f;
        smoothed.bbox_3d.center = alpha * raw_state.bbox_3d.center +
                                  (1.0f - alpha) * predicted_center;
        
        // 尺寸平滑（尺寸通常不应变化，使用极小的 alpha）
        smoothed.bbox_3d.size = 0.05f * raw_state.bbox_3d.size +
                                0.95f * prev.bbox_3d.size;
        
        // 角度平滑
        float yaw_diff = std::atan2(std::sin(raw_state.bbox_3d.yaw - prev.bbox_3d.yaw),
                                    std::cos(raw_state.bbox_3d.yaw - prev.bbox_3d.yaw));
        smoothed.bbox_3d.yaw = prev.bbox_3d.yaw + alpha * yaw_diff;
        
        // 速度平滑：直接使用卡尔曼滤波估计的速度，而不是位置差分
        smoothed.bbox_3d.velocity = 0.3f * raw_state.bbox_3d.velocity + 
                                    0.7f * prev.bbox_3d.velocity;
    }

    last_smoothed_states_[track->track_id] = smoothed;
    return smoothed;
}

void MultiObjectTracker::reset() {
    tracks_.clear();
    frame_count_ = 0;
    last_timestamp_ = -1.0;
    next_track_id_ = 1;
    last_smoothed_states_.clear();
    RCLCPP_INFO(rclcpp::get_logger("MultiObjectTracker"), "Tracker reset");
}

} // namespace fusion_cpp
