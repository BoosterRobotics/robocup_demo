#pragma once

#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include "booster_vision/model//data_types.h"

namespace booster_vision {

class YoloV8Detector {
public:
    virtual ~YoloV8Detector() = default;
    virtual void Init(std::string model_path){};
    virtual std::vector<DetectionRes> Inference(const cv::Mat &img) = 0;

    void setConfidenceThreshold(float confidence_threshold) {
        confidence_ = confidence_threshold;
    }

    float getConfidenceThreshold() {
        return confidence_;
    }

    void setNMSThreshold(float nms_threshold) {
        nms_threshold_ = nms_threshold;
    }

    float getNMSThreshold() {
        return nms_threshold_;
    }

    std::string getModelPath() {
        return model_path_;
    }

    static std::shared_ptr<YoloV8Detector> CreateYoloV8Detector(const YAML::Node &node);
    static cv::Mat DrawDetection(const cv::Mat &img, const std::vector<DetectionRes> &detections);
    static const std::vector<std::string> kClassLabels;

protected:
    YoloV8Detector(const std::string &model_path) :
        model_path_(model_path) {
    }

    std::string model_path_;
    float confidence_ = 0.25f;
    float nms_threshold_ = 0.4f;
};

} // namespace booster_vision