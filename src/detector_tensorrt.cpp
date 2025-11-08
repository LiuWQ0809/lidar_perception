#include "fusion_cpp/detector_tensorrt.hpp"

#ifdef USE_TENSORRT

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <algorithm>
#include <cuda_runtime_api.h>

namespace fusion_cpp {

// TensorRT Logger实现
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};

// COCO类别名称
static const std::vector<std::string> COCO_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
};

TensorRTDetector::TensorRTDetector(
    const std::string& engine_path,
    float conf_threshold,
    float iou_threshold,
    const std::vector<int>& interested_classes)
    : engine_path_(engine_path),
      conf_threshold_(conf_threshold),
      iou_threshold_(iou_threshold),
      interested_classes_(interested_classes),
      runtime_(nullptr),
      engine_(nullptr),
      context_(nullptr),
      input_size_(640),
      output_size_(8400),
      input_buffer_(nullptr),
      output_buffer_(nullptr),
      stream_(nullptr),
      class_names_(COCO_CLASSES) {

    // 加载引擎
    if (!loadEngine(engine_path)) {
        throw std::runtime_error("Failed to load TensorRT engine");
    }

    // 分配GPU内存
    const size_t input_bytes = 1 * 3 * input_size_ * input_size_ * sizeof(float);
    const size_t output_bytes = 1 * 84 * output_size_ * sizeof(float);

    cudaMalloc(&input_buffer_, input_bytes);
    cudaMalloc(&output_buffer_, output_bytes);
    cudaStreamCreate(reinterpret_cast<cudaStream_t*>(&stream_));

    RCLCPP_INFO(rclcpp::get_logger("TensorRTDetector"),
                "TensorRT Detector initialized");
    RCLCPP_INFO(rclcpp::get_logger("TensorRTDetector"),
                "  Engine: %s", engine_path.c_str());
    RCLCPP_INFO(rclcpp::get_logger("TensorRTDetector"),
                "  Confidence threshold: %.2f", conf_threshold);
    RCLCPP_INFO(rclcpp::get_logger("TensorRTDetector"),
                "  IOU threshold: %.2f", iou_threshold);
}

TensorRTDetector::~TensorRTDetector() {
    // 释放资源
    if (input_buffer_) cudaFree(input_buffer_);
    if (output_buffer_) cudaFree(output_buffer_);
    if (stream_) cudaStreamDestroy(reinterpret_cast<cudaStream_t>(stream_));
    if (context_) delete context_;
    if (engine_) delete engine_;
    if (runtime_) delete runtime_;
}

bool TensorRTDetector::loadEngine(const std::string& engine_path) {
    // 读取引擎文件
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        RCLCPP_ERROR(rclcpp::get_logger("TensorRTDetector"),
                    "Failed to open engine file: %s", engine_path.c_str());
        return false;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);
    file.close();

    // 创建runtime和engine
    static Logger logger;
    runtime_ = nvinfer1::createInferRuntime(logger);
    
    if (!runtime_) {
        RCLCPP_ERROR(rclcpp::get_logger("TensorRTDetector"),
                    "Failed to create TensorRT runtime");
        return false;
    }

    engine_ = runtime_->deserializeCudaEngine(
        engine_data.data(), size);

    if (!engine_) {
        RCLCPP_ERROR(rclcpp::get_logger("TensorRTDetector"),
                    "Failed to deserialize CUDA engine");
        return false;
    }

    // 创建execution context
    context_ = engine_->createExecutionContext();

    if (!context_) {
        RCLCPP_ERROR(rclcpp::get_logger("TensorRTDetector"),
                    "Failed to create execution context");
        return false;
    }

    return true;
}

void TensorRTDetector::preprocess(const cv::Mat& image,
                                  std::vector<float>& input_data) {
    const int target_size = input_size_;
    
    // 计算缩放比例
    const float scale = static_cast<float>(target_size) / 
                       std::max(image.rows, image.cols);
    
    const int new_w = static_cast<int>(image.cols * scale);
    const int new_h = static_cast<int>(image.rows * scale);

    // Resize
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h));

    // Padding
    const int dw = target_size - new_w;
    const int dh = target_size - new_h;
    const int top = dh / 2;
    const int bottom = dh - top;
    const int left = dw / 2;
    const int right = dw - left;

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right,
                      cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    // 转换为float并归一化
    cv::Mat float_img;
    padded.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    // HWC to CHW
    input_data.resize(1 * 3 * target_size * target_size);
    
    std::vector<cv::Mat> channels(3);
    cv::split(float_img, channels);

    // 按CHW顺序排列: BGR -> RGB
    for (int c = 0; c < 3; ++c) {
        const int channel_idx = 2 - c;  // BGR to RGB
        std::memcpy(input_data.data() + c * target_size * target_size,
                   channels[channel_idx].data,
                   target_size * target_size * sizeof(float));
    }

    scale_ = scale;
    padded_size_ = cv::Size(new_w, new_h);
}

std::vector<Detection> TensorRTDetector::detect(const cv::Mat& image) {
    if (image.empty()) {
        return {};
    }

    // 预处理
    std::vector<float> input_data;
    preprocess(image, input_data);

    // 拷贝到GPU
    const size_t input_bytes = input_data.size() * sizeof(float);
    cudaMemcpyAsync(input_buffer_, input_data.data(), input_bytes,
                   cudaMemcpyHostToDevice,
                   reinterpret_cast<cudaStream_t>(stream_));

    // 推理
    void* bindings[] = {input_buffer_, output_buffer_};
    
    // 设置输入输出绑定
    context_->setTensorAddress("images", input_buffer_);
    context_->setTensorAddress("output0", output_buffer_);
    
    // 执行推理（使用 enqueueV3 for TensorRT 8.5+）
    context_->enqueueV3(reinterpret_cast<cudaStream_t>(stream_));

    // 拷贝结果到CPU
    std::vector<float> output_data(1 * 84 * output_size_);
    const size_t output_bytes = output_data.size() * sizeof(float);
    cudaMemcpyAsync(output_data.data(), output_buffer_, output_bytes,
                   cudaMemcpyDeviceToHost,
                   reinterpret_cast<cudaStream_t>(stream_));
    
    cudaStreamSynchronize(reinterpret_cast<cudaStream_t>(stream_));

    // 后处理
    std::vector<Detection> detections = postprocess(output_data.data(), image.size());

    // NMS
    std::vector<Detection> nms_result = applyNMS(detections);
    // RCLCPP_INFO(rclcpp::get_logger("TensorRTDetector"),
    //            "[NMS] %zu -> %zu detections", detections.size(), nms_result.size());
    
    // 额外的去重：如果两个框的IoU仍然很高（可能NMS没完全去除），再次过滤
    if (nms_result.size() > 1) {
        std::vector<Detection> final_result = removeDuplicateDetections(nms_result, 0.5f);
        // RCLCPP_INFO(rclcpp::get_logger("TensorRTDetector"),
        //            "[DEDUP] %zu -> %zu detections", nms_result.size(), final_result.size());
        return final_result;
    }
    
    return nms_result;
}

std::vector<Detection> TensorRTDetector::postprocess(
    const float* output,
    const cv::Size& original_shape) {
    
    std::vector<Detection> detections;

    // YOLOv8输出格式: (1, 84, 8400)
    // 需要转置为 (8400, 84) 来访问
    // 84 = 4(bbox) + 80(classes)
    
    // 数据在内存中的布局是 [batch, channels, detections]
    // 即 output[0 * 84 * 8400 + c * 8400 + i] 对应第i个检测框的第c个通道
    
    for (int i = 0; i < output_size_; ++i) {
        // 正确的访问方式：output[channel * output_size + detection_idx]
        const float cx = output[0 * output_size_ + i];
        const float cy = output[1 * output_size_ + i];
        const float w = output[2 * output_size_ + i];
        const float h = output[3 * output_size_ + i];

        // 获取类别和置信度
        float max_conf = 0.0f;
        int max_class_id = 0;
        
        for (int c = 0; c < 80; ++c) {
            const float conf = output[(4 + c) * output_size_ + i];
            if (conf > max_conf) {
                max_conf = conf;
                max_class_id = c;
            }
        }

        // 过滤低置信度和不感兴趣的类别
        if (max_conf < conf_threshold_) continue;
        
        if (!interested_classes_.empty() &&
            std::find(interested_classes_.begin(), interested_classes_.end(), 
                     max_class_id) == interested_classes_.end()) {
            continue;
        }

        // 转换坐标到原始图像
        const float x1 = (cx - w / 2.0f) / scale_;
        const float y1 = (cy - h / 2.0f) / scale_;
        const float x2 = (cx + w / 2.0f) / scale_;
        const float y2 = (cy + h / 2.0f) / scale_;

        Detection det;
        det.bbox = {x1, y1, x2, y2};
        det.confidence = max_conf;
        det.class_id = max_class_id;
        det.class_name = class_names_[max_class_id];

        detections.push_back(det);
        
        // 调试：输出每个检测框的信息
        // RCLCPP_DEBUG(rclcpp::get_logger("TensorRTDetector"),
        //             "[DETECT] cls=%d conf=%.3f bbox=[%.1f,%.1f,%.1f,%.1f]",
        //             max_class_id, max_conf, x1, y1, x2, y2);
    }
    
    // RCLCPP_INFO(rclcpp::get_logger("TensorRTDetector"),
    //            "[POSTPROCESS] Found %zu detections before NMS", detections.size());

    return detections;
}

std::vector<Detection> TensorRTDetector::applyNMS(
    const std::vector<Detection>& detections) {
    
    if (detections.empty()) {
        return {};
    }

    // 简化的NMS实现
    std::vector<Detection> result;
    std::vector<bool> suppressed(detections.size(), false);

    for (size_t i = 0; i < detections.size(); ++i) {
        if (suppressed[i]) continue;

        result.push_back(detections[i]);

        const auto& bbox_i = detections[i].bbox;
        const float area_i = (bbox_i[2] - bbox_i[0]) * (bbox_i[3] - bbox_i[1]);

        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (suppressed[j]) continue;
            if (detections[i].class_id != detections[j].class_id) continue;

            const auto& bbox_j = detections[j].bbox;
            
            // 计算IoU
            const float x1 = std::max(bbox_i[0], bbox_j[0]);
            const float y1 = std::max(bbox_i[1], bbox_j[1]);
            const float x2 = std::min(bbox_i[2], bbox_j[2]);
            const float y2 = std::min(bbox_i[3], bbox_j[3]);

            if (x2 > x1 && y2 > y1) {
                const float inter = (x2 - x1) * (y2 - y1);
                const float area_j = (bbox_j[2] - bbox_j[0]) * (bbox_j[3] - bbox_j[1]);
                const float iou = inter / (area_i + area_j - inter);

                if (iou > iou_threshold_) {
                    suppressed[j] = true;
                }
            }
        }
    }

    return result;
}

std::vector<Detection> TensorRTDetector::removeDuplicateDetections(
    const std::vector<Detection>& detections, float iou_threshold) {
    
    if (detections.size() <= 1) {
        return detections;
    }
    
    // 按置信度排序
    std::vector<Detection> sorted_dets = detections;
    std::sort(sorted_dets.begin(), sorted_dets.end(),
             [](const Detection& a, const Detection& b) {
                 return a.confidence > b.confidence;
             });
    
    std::vector<Detection> keep;
    
    for (const auto& det : sorted_dets) {
        bool is_duplicate = false;
        const auto& bbox1 = det.bbox;
        const float area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1]);
        
        for (const auto& kept_det : keep) {
            const auto& bbox2 = kept_det.bbox;
            
            // 计算IoU
            const float x1 = std::max(bbox1[0], bbox2[0]);
            const float y1 = std::max(bbox1[1], bbox2[1]);
            const float x2 = std::min(bbox1[2], bbox2[2]);
            const float y2 = std::min(bbox1[3], bbox2[3]);
            
            if (x2 > x1 && y2 > y1) {
                const float inter = (x2 - x1) * (y2 - y1);
                const float area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1]);
                const float iou = inter / (area1 + area2 - inter);
                
                // 如果IoU很高且是同一类别，认为是重复
                if (iou > iou_threshold && det.class_id == kept_det.class_id) {
                    is_duplicate = true;
                    break;
                }
            }
        }
        
        if (!is_duplicate) {
            keep.push_back(det);
        }
    }
    
    return keep;
}

cv::Mat TensorRTDetector::visualize(const cv::Mat& image,
                                   const std::vector<Detection>& detections) {
    cv::Mat vis = image.clone();

    for (const auto& det : detections) {
        const int x1 = static_cast<int>(det.bbox[0]);
        const int y1 = static_cast<int>(det.bbox[1]);
        const int x2 = static_cast<int>(det.bbox[2]);
        const int y2 = static_cast<int>(det.bbox[3]);

        cv::Scalar color = getColor(det.class_id);
        cv::rectangle(vis, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

        std::string label = det.class_name + ": " + 
                          std::to_string(det.confidence).substr(0, 4);
        
        int baseline;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                              0.5, 2, &baseline);
        
        cv::rectangle(vis, cv::Point(x1, y1 - label_size.height - 10),
                     cv::Point(x1 + label_size.width, y1), color, -1);
        
        cv::putText(vis, label, cv::Point(x1, y1 - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    }

    return vis;
}

cv::Scalar TensorRTDetector::getColor(int class_id) {
    static const std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),    // 蓝色
        cv::Scalar(0, 255, 0),    // 绿色
        cv::Scalar(0, 0, 255),    // 红色
        cv::Scalar(255, 255, 0),  // 青色
        cv::Scalar(255, 0, 255),  // 洋红
        cv::Scalar(0, 255, 255),  // 黄色
    };
    return colors[class_id % colors.size()];
}

} // namespace fusion_cpp

#endif // USE_TENSORRT
