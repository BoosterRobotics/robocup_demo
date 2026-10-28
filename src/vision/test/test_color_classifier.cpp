#include <filesystem>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "booster_vision/color_classifier.hpp"
#include "booster_vision/base/misc_utils.hpp"
#include "booster_vision/model/detector.h"

using namespace booster_vision;

int main(int argc, char **argv) {
    std::string cfg_template_path = argv[1];
    std::string cfg_path = argv[2];
    std::string img_path = argv[3];

    if (!std::filesystem::exists(cfg_template_path)) {
        // TODO(SS): throw exception here
        std::cerr << "Error: Configuration template file '" << cfg_template_path << "' does not exist." << std::endl;
        return -1;
    }

    YAML::Node node = YAML::LoadFile(cfg_template_path);
    if (!std::filesystem::exists(cfg_path)) {
        std::cout << "Warning: Configuration file empty!" << std::endl;
    } else {
        YAML::Node cfg_node = YAML::LoadFile(cfg_path);
        // merge input cfg to template cfg
        MergeYAML(node, cfg_node);
    }

    std::shared_ptr<ColorClassifier> color_classifier_;
    if (node["robot_color_classifier"]) {
        color_classifier_ = std::make_shared<ColorClassifier>();
        color_classifier_->Init(node["robot_color_classifier"]);
    }

    std::shared_ptr<YoloV8Detector> detector_;
    if (!node["detection_model"]) {
        std::cerr << "no detection model param here" << std::endl;
        return -1;
    } else {
        detector_ = YoloV8Detector::CreateYoloV8Detector(node["detection_model"], "");
        // classnames_ = node["detection_model"]["classnames"].as<std::vector<std::string>>();
        // detector post processing
        float default_threshold = as_or<float>(node["detection_model"]["confidence_threshold"], 0.2);
    }

    // loop img_path and load images
    std::vector<std::string> img_files;
    for (const auto &entry : std::filesystem::directory_iterator(img_path)) {
        if (entry.is_regular_file() && (entry.path().extension() == ".jpg" || entry.path().extension() == ".png")) {
            img_files.push_back(entry.path().string());
        }
    }

    // process image
    for (const auto &img_file : img_files) {
        cv::Mat color = cv::imread(img_file);
        if (color.empty()) {
            std::cerr << "Error: Could not read image file '" << img_file << "'." << std::endl;
            continue;
        }

        int cnt = 0;
        // process image
        auto detections = detector_->Inference(color);
        for (auto &detection : detections) {
            if ((color_classifier_ != nullptr) && detection.class_name == "Opponent") {
                // get a crop of the image given detection.bbox
                cv::Mat crop = color(detection.bbox);
                std::string robot_color_str = color_classifier_->Classify(crop);

                // draw bbox on the image and put the color string
                cv::rectangle(color, detection.bbox, cv::Scalar(0, 255, 0), 2);
                cv::putText(color, robot_color_str, cv::Point(detection.bbox.x + detection.bbox.width/2, detection.bbox.y + detection.bbox.height/2),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                cnt++;
            }
        }
        if (cnt == 0) continue;
        cv::imshow("Detection", color);
        cv::waitKey(0);
    }
    return 0;
}