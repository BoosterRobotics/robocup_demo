#include "booster_vision/model/detector.h"

#include <stdexcept>
#include <filesystem>

#include "booster_vision/model/trt/impl.h"
#ifdef ENABLE_ONNX
#include "booster_vision/model/onnx/detection_impl.h"
#endif

namespace booster_vision {

const std::vector<std::string> YoloV8Detector::kClassLabels{"Ball", "Goalpost", "Person", "LCross",
                                                            "TCross", "XCross", "PenaltyPoint", "Opponent", "BRMarker"};

std::shared_ptr<YoloV8Detector> YoloV8Detector::CreateYoloV8Detector(const YAML::Node &node) {
    try {
        std::string model_path = node["model_path"].as<std::string>();
        float conf_thresh = node["confidence_threshold"].as<float>();
        float nms_thresh = node["nms_threshold"].as<float>();

        std::shared_ptr<YoloV8Detector> detector_ptr = nullptr;
        #ifdef ENABLE_ONNX
        if (model_path.find(".onnx") != std::string::npos) {
            detector_ptr = std::shared_ptr<YoloV8Detector>(new YoloV8DetectorONNX(model_path));
        } else {
            detector_ptr = std::shared_ptr<YoloV8Detector>(new YoloV8DetectorTRT(model_path));
        }
        #else
        detector_ptr = std::shared_ptr<YoloV8Detector>(new YoloV8DetectorTRT(model_path));
        #endif
        if (detector_ptr) {
            detector_ptr->setConfidenceThreshold(conf_thresh);
            detector_ptr->setNMSThreshold(nms_thresh);
        }
        return detector_ptr;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return nullptr;
    }
}

cv::Mat YoloV8Detector::DrawDetection(const cv::Mat &img, const std::vector<DetectionRes> &detections) {
    static std::unordered_map<int, cv::Scalar> class_colors;
    auto get_color_for_class = [&](int class_id) {
        if (class_colors.find(class_id) == class_colors.end()) {
            // Generate a random color for the class
            cv::RNG rng(class_id);
            class_colors[class_id] = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        }
        return class_colors[class_id];
    };

    cv::Mat img_out = img.clone();
    for (const auto &detection : detections) {
        cv::rectangle(img_out, detection.bbox, get_color_for_class(detection.class_id), 2);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << detection.confidence * 100;
        std::string conf(ss.str());
        std::string display_text = kClassLabels[detection.class_id] + ": " + conf;
        cv::putText(img_out, display_text, cv::Point(detection.bbox.x, detection.bbox.y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    }
    return img_out;
}

} // namespace booster_vision