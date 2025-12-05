#include "booster_vision/model//segmentor.h"

#include "booster_vision/model//trt/impl.h"

namespace booster_vision {

const std::vector<std::string> YoloV8Segmentor::kClassLabels = {"CircleLine", "Line"};

std::shared_ptr<YoloV8Segmentor> YoloV8Segmentor::CreateYoloV8Segmentor(const YAML::Node &node) {
    try {
        std::string model_path = node["model_path"].as<std::string>();
        float conf_thresh = node["confidence_threshold"].as<float>();

        return std::shared_ptr<YoloV8Segmentor>(new YoloV8SegmentorTRT(model_path, conf_thresh));
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return nullptr;
    }
}

cv::Mat YoloV8Segmentor::DrawSegmentation(const cv::Mat &img, const std::vector<SegmentationRes> &segmentations) {
    static std::unordered_map<int, cv::Scalar> class_colors;
    auto get_color_for_class = [&](int class_id) {
        if (class_colors.find(class_id) == class_colors.end()) {
            // Generate a random color for the class
            cv::RNG rng(class_id);
            class_colors[class_id] = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 256));
        }
        return class_colors[class_id];
    };

    cv::Mat img_out;
    cv::cvtColor(img, img_out, cv::COLOR_RGB2BGR);
    for (const auto &seg : segmentations) {
        auto display_color = get_color_for_class(seg.class_id);
        for (int y = seg.bbox.y; y < seg.bbox.y + seg.bbox.height; y++) {
            for (int x = seg.bbox.x; x < seg.bbox.x + seg.bbox.width; x++) {
                if (seg.mask.at<uchar>(y, x) > 127) {
                    for (int c = 0; c < 3; c++) {
                        // img_out.at<cv::Vec3b>(y, x)[c] = std::max(255, img_out.at<cv::Vec3b>(y, x)[c] + static_cast<int>(display_color[c]));
                        img_out.at<cv::Vec3b>(y, x)[c] = std::min(255, static_cast<int>(display_color[c]));
                    }
                }
            }
        }
        cv::rectangle(img_out, seg.bbox, display_color, 2);
        cv::putText(img_out, kClassLabels[seg.class_id] + " " + std::to_string(int(seg.confidence * 100)), cv::Point(seg.bbox.x, seg.bbox.y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
    }
    return img_out;
}

} // namespace booster_vision