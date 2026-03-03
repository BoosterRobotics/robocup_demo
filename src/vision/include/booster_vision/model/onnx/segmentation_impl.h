#pragma once

#include "booster_vision/model/segmentor.h"

#include <string>
#include <vector>
#include <cstdio>
#include <memory>

#include <opencv2/opencv.hpp>

#include <onnxruntime_cxx_api.h>

namespace booster_vision {

class YoloV8SegmentorONNX : public booster_vision::YoloV8Segmentor {
public:
    YoloV8SegmentorONNX(const std::string &path, const float &conf) :
        booster_vision::YoloV8Segmentor(path, conf) {
        Init(path);
    }
    ~YoloV8SegmentorONNX();

    void Init(std::string model_path);
    std::vector<booster_vision::SegmentationRes> Inference(const cv::Mat &img) override;

private:
    template <typename T>
    std::vector<booster_vision::SegmentationRes> InferenceImpl(const cv::Mat &img);

    Ort::Env env_;
    std::shared_ptr<Ort::Session> session_;
    Ort::RunOptions options_;
    std::vector<const char *> input_node_names_;
    std::vector<const char *> output_node_names_;
    ONNXTensorElementDataType element_type_;
    void* data_buffer_ = nullptr;

    cv::Size model_input_size_ = {640, 640};  // Will be overridden by model's actual input size
};

} // namespace booster_vision
