#ifndef FUSION_CPP_DETECTOR_TENSORRT_HPP_
#define FUSION_CPP_DETECTOR_TENSORRT_HPP_

#ifdef USE_TENSORRT

#include "fusion_cpp/sensor_fusion.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <NvInfer.h>

namespace fusion_cpp {

/**
 * @brief TensorRT检测器
 * 使用TensorRT引擎进行YOLOv8目标检测
 */
class TensorRTDetector {
public:
    /**
     * @brief 构造函数
     * @param engine_path TensorRT引擎文件路径
     * @param conf_threshold 置信度阈值
     * @param iou_threshold NMS IOU阈值
     * @param interested_classes 感兴趣的类别ID列表
     */
    TensorRTDetector(const std::string& engine_path,
                     float conf_threshold = 0.5f,
                     float iou_threshold = 0.45f,
                     const std::vector<int>& interested_classes = {0});

    ~TensorRTDetector();

    /**
     * @brief 检测图像中的目标
     * @param image 输入图像 (BGR格式)
     * @return 检测结果列表
     */
    std::vector<Detection> detect(const cv::Mat& image);

    /**
     * @brief 可视化检测结果
     * @param image 原始图像
     * @param detections 检测结果
     * @return 可视化后的图像
     */
    cv::Mat visualize(const cv::Mat& image, 
                     const std::vector<Detection>& detections);

private:
    /**
     * @brief 加载TensorRT引擎
     * @param engine_path 引擎文件路径
     * @return true if成功
     */
    bool loadEngine(const std::string& engine_path);

    /**
     * @brief 预处理图像
     * @param image 输入图像
     * @param[out] input_data 预处理后的数据
     */
    void preprocess(const cv::Mat& image, std::vector<float>& input_data);

    /**
     * @brief 后处理检测结果
     * @param output 模型输出
     * @param original_shape 原始图像尺寸
     * @return 检测结果
     */
    std::vector<Detection> postprocess(const float* output,
                                       const cv::Size& original_shape);

    /**
     * @brief NMS (Non-Maximum Suppression)
     * @param detections 检测结果
     * @return 过滤后的检测结果
     */
    std::vector<Detection> applyNMS(const std::vector<Detection>& detections);

    /**
     * @brief 移除重复的检测结果（额外的去重步骤）
     * @param detections 检测结果
     * @param iou_threshold IoU阈值
     * @return 去重后的检测结果
     */
    std::vector<Detection> removeDuplicateDetections(
        const std::vector<Detection>& detections, float iou_threshold);

    /**
     * @brief 获取类别颜色
     * @param class_id 类别ID
     * @return BGR颜色
     */
    cv::Scalar getColor(int class_id);

private:
    std::string engine_path_;
    float conf_threshold_;
    float iou_threshold_;
    std::vector<int> interested_classes_;

    // TensorRT相关
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;

    // 输入输出尺寸
    int input_size_;  // 通常是640
    int output_size_; // 检测框数量

    // GPU内存
    void* input_buffer_;
    void* output_buffer_;

    // CUDA stream
    void* stream_;

    // 类别名称
    std::vector<std::string> class_names_;

    // 预处理参数
    float scale_;
    cv::Size padded_size_;
};

} // namespace fusion_cpp

#endif // USE_TENSORRT

#endif // FUSION_CPP_DETECTOR_TENSORRT_HPP_
