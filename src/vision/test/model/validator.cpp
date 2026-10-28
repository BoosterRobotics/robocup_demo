#include "booster_vision/model/detector.h"

#include <fstream>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "booster_vision/base/misc_utils.hpp"

using booster_vision::YoloV8Detector;
using booster_vision::DetectionRes;

int main(int argc, char **argv) {
    std::string cfg_path = argv[1];
    std::string img_path = argv[2];

    YAML::Node node = YAML::LoadFile(cfg_path);
    std::shared_ptr<YoloV8Detector> detector = YoloV8Detector::CreateYoloV8Detector(node["detection_model"], "");

    // post processing
    bool enable_post_process_ = false;
    bool single_ball_assumption_ = false;
    std::vector<std::string> classnames_;
    std::map<std::string, float> confidence_map_;

    // init detector
    if (!node["detection_model"]) {
        std::cerr << "no detection model param here" << std::endl;
        return -1;
    } else {
        classnames_ = node["detection_model"]["classnames"].as<std::vector<std::string>>();
        // detector post processing
        float default_threshold = as_or<float>(node["detection_model"]["confidence_threshold"], 0.2);
        if (node["detection_model"]["post_process"]) {
            enable_post_process_ = true;
            single_ball_assumption_ = as_or<bool>(node["detection_model"]["post_process"]["single_ball_assumption"], false);
            if (node["detection_model"]["post_process"]["confidence_thresholds"]) {
                for (const auto &item : node["detection_model"]["post_process"]["confidence_thresholds"]) {
                    confidence_map_[item.first.as<std::string>()] = item.second.as<float>();
                }
                // set default confidence for other classes
                for (const auto &classname : classnames_) {
                    if (confidence_map_.find(classname) == confidence_map_.end()) {
                        confidence_map_[classname] = default_threshold;
                    }
                }
            } else {
                std::cout << "all class apply same default threshold: " << default_threshold << std::endl;
            }
        }
    }

    // get the parant directory of img_path
    std::string root_dir = img_path.substr(0, img_path.find_last_of("/\\"));
    std::string val_dir = root_dir + "/val";
    int cnt = 0;
    while (std::filesystem::exists(val_dir)) {
        val_dir = root_dir + "/val_" + std::to_string(cnt);
        cnt++;
    }
    std::filesystem::create_directories(val_dir);

    // list all files in the directory
    std::vector<std::string> files;
    for (const auto &entry : std::filesystem::directory_iterator(img_path)) {
        auto file = entry.path().filename().string();
        if (file.find(".jpg") != std::string::npos || file.find(".jpeg") != std::string::npos || file.find(".png") != std::string::npos) {
            files.push_back(entry.path().string());
        }
    }

    for (const auto &file : files) {
        cv::Mat img = cv::imread(file);

        std::vector<DetectionRes> objects = detector->Inference(img);

        std::vector<booster_vision::DetectionRes> filtered_detections;
        if (enable_post_process_) {
            // filter detections with different confidence
            if (!confidence_map_.empty()) {
                for (auto &detection : objects) {
                    auto classname = classnames_[detection.class_id];
                    if (detection.confidence < confidence_map_[classname]) {
                        continue;
                    }
                    filtered_detections.push_back(detection);
                }
            } else {
                filtered_detections = objects;
            }

            // keep the highest ball detections
            if (single_ball_assumption_) {
                std::vector<booster_vision::DetectionRes> ball_detections;
                std::vector<booster_vision::DetectionRes> filtered_detections_bk = filtered_detections;
                filtered_detections.clear();

                for (const auto& detection : filtered_detections_bk) {
                    if (classnames_[detection.class_id] == "Ball") {
                        ball_detections.push_back(detection);
                    } else {
                        filtered_detections.push_back(detection);
                    }
                }

                if (ball_detections.size() > 1) {
                    std::cout << "Multiple ball detections found, keeping the one with highest confidence." << std::endl;
                    auto max_ball_detection = *std::max_element(ball_detections.begin(), ball_detections.end(),
                                                                [](const booster_vision::DetectionRes &a, const booster_vision::DetectionRes &b) {
                                                                    return a.confidence < b.confidence;
                                                                });
                    filtered_detections.push_back(max_ball_detection);
                } else {
                    filtered_detections.insert(filtered_detections.end(), ball_detections.begin(), ball_detections.end());
                }
            }
        } else {
            filtered_detections = objects;
        }
        std::cout << "image file processed: " << file << ", number of objects detected: " << filtered_detections.size() << std::endl;
        for (const auto &obj : filtered_detections) {
            std::cout << "class: " << obj.class_name << " confidence: " << obj.confidence << " bbox: " << obj.bbox << std::endl;
        }
        std::cout << std::endl;

        // get file name of file strip the directory
        std::string file_name = file.substr(file.find_last_of("/\\") + 1);
        std::string validation_file_name = file_name.substr(0, file_name.find_last_of(".")) + ".txt";


        auto display_img = booster_vision::YoloV8Detector::DrawDetection(img, filtered_detections);
        // cv::imshow("Validation", display_img);
        // cv::waitKey(0);
        cv::imwrite(val_dir + "/" + file_name + "_val.jpg", display_img);
        // save class_id x y w h to a file
        // std::ofstream validation_file(val_dir + "/" + validation_file_name);
        // for (auto &obj : filtered_detections) {
        //     validation_file << obj.class_id << " " << obj.bbox.x / 1280.0 << " " << obj.bbox.y / 720.0
        //                     << " " << obj.bbox.width / 1280.0 << " " << obj.bbox.height / 720.0 << std::endl;
        // }
        // std::cout << "Validation file saved: " << root_dir + "/val/" + validation_file_name << std::endl;
    }
}