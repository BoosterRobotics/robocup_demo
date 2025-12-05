#pragma once

#include <vector>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include "booster_vision/model//data_types.h"

namespace booster_vision {

class YoloV8Segmentor {
public:
    virtual ~YoloV8Segmentor() {
    }
    virtual std::vector<SegmentationRes> Inference(const cv::Mat &img) = 0;

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

    static std::shared_ptr<YoloV8Segmentor> CreateYoloV8Segmentor(const YAML::Node &node);
    static cv::Mat DrawSegmentation(const cv::Mat &img, const std::vector<SegmentationRes> &detections);
    static const std::vector<std::string> kClassLabels;

protected:
    YoloV8Segmentor(const std::string &name, const float &conf) :
        model_path_(name), confidence_(conf) {
    }
    float confidence_ = 0.2f;
    float nms_threshold_ = 0.4f;
    std::string model_path_;
};

} // namespace booster_vision