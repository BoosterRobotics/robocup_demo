#pragma once

#include "booster_vision/model/detector.h"

#include <string>
#include <vector>
#include <cstdio>
#include <memory>

#include <opencv2/opencv.hpp>

#include <onnxruntime/onnxruntime_cxx_api.h>

namespace booster_vision {

class YoloV8DetectorONNX : public booster_vision::YoloV8Detector {
public:
    YoloV8DetectorONNX(const std::string &path) :
        booster_vision::YoloV8Detector(path) {
        Init(path);
    }
    ~YoloV8DetectorONNX();

    void Init(std::string model_path) override;
    std::vector<booster_vision::DetectionRes> Inference(const cv::Mat &img) override;

private:
    template <typename T>
    std::vector<booster_vision::DetectionRes> InferenceImpl(const cv::Mat &img);

    Ort::Env env_;
    std::shared_ptr<Ort::Session> session_;
    Ort::RunOptions options_;
    std::vector<const char *> input_node_names_;
    std::vector<const char *> output_node_names_;
    ONNXTensorElementDataType element_type_;
    void* data_buffer_ = nullptr;

    cv::Size model_input_size_ = {640, 640};
};

} // namespace booster_vision