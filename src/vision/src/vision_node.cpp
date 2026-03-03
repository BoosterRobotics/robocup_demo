#include "booster_vision/vision_node.h"

#include <cstdlib>
#include <functional>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "vision_interface/msg/detected_object.hpp"
#include "vision_interface/msg/detections.hpp"
#include "vision_interface/msg/cal_param.hpp"

#include "booster_vision/base/data_syncer.hpp"
#include "booster_vision/base/data_logger.hpp"
#include "booster_vision/base/misc_utils.hpp"
#include "booster_vision/model//detector.h"
#include "booster_vision/model//segmentor.h"
#include "booster_vision/pose_estimator/pose_estimator.h"
#include "booster_vision/img_bridge.h"

namespace booster_vision {

VisionNode::VisionNode(const std::string &node_name, const rclcpp::NodeOptions &options) :
    rclcpp::Node(node_name, options) {
    this->declare_parameter<bool>("offline_mode", false);
    this->declare_parameter<bool>("show_det", false);
    this->declare_parameter<bool>("show_seg", false);
    this->declare_parameter<bool>("save_data", true);
    this->declare_parameter<bool>("save_depth", true);
    this->declare_parameter<int>("save_fps", 3);
    this->declare_parameter<std::string>("robot_name", "");
    this->declare_parameter<std::string>("detection_model_path", "");
    this->declare_parameter<std::string>("segmentation_model_path", "");
    this->declare_parameter<std::string>("color_topic", "");
    this->declare_parameter<std::string>("depth_topic", "");
    this->declare_parameter<std::string>("intrin_topic", "");
}

// TODO(GW): oneline offline
void VisionNode::Init(const std::string &cfg_template_path, const std::string &cfg_path) {
    if (!std::filesystem::exists(cfg_template_path)) {
        // TODO(SS): throw exception here
        std::cerr << "Error: Configuration template file '" << cfg_template_path << "' does not exist." << std::endl;
        return;
    }

    YAML::Node node = YAML::LoadFile(cfg_template_path);
    if (!std::filesystem::exists(cfg_path)) {
        std::cout << "Warning: Configuration file empty!" << std::endl;
    } else {
        YAML::Node cfg_node = YAML::LoadFile(cfg_path);
        // merge input cfg to template cfg
        MergeYAML(node, cfg_node);
    }

    std::cout << "loaded file: " << std::endl
              << node << std::endl;

    // Store config node for later use (e.g., when updating intrinsics)
    config_node_ = node;

    this->get_parameter<bool>("show_det", show_det_);
    this->get_parameter<bool>("show_seg", show_seg_);
    this->get_parameter<bool>("save_data", save_data_);
    this->get_parameter<bool>("save_depth", save_depth_);
    this->get_parameter<bool>("offline_mode", offline_mode_);
    this->get_parameter<std::string>("color_topic", color_topic_);
    this->get_parameter<std::string>("depth_topic", depth_topic_);
    this->get_parameter<std::string>("intrin_topic", intrin_topic_);
    this->get_parameter<std::string>("detection_model_path", detection_model_path);
    std::cout << "detection_model_path origin: " << detection_model_path << std::endl;

    if(!detection_model_path.empty()){
        if(detection_model_path[0] == '/') {
            // absolute path, do nothing
        } else {
            std::string package_path = ament_index_cpp::get_package_share_directory("vision");
            detection_model_path = (std::filesystem::path(package_path) / detection_model_path).string();
        }
    }


    this->get_parameter<std::string>("segmentation_model_path", segmentation_model_path);
    std::cout << "segmentation_model_path origin: " << segmentation_model_path << std::endl;

    if(!segmentation_model_path.empty()){
        if(segmentation_model_path[0] == '/') {
            // absolute path, do nothing
        } else {
            std::string package_path = ament_index_cpp::get_package_share_directory("vision");
            segmentation_model_path = (std::filesystem::path(package_path) / segmentation_model_path).string();
        }
    }
    
    int save_fps = 0;
    this->get_parameter<int>("save_fps", save_fps);
    save_depth_ = save_depth_ && save_data_;
    std::cout << "offline_mode: " << offline_mode_ << std::endl;
    std::cout << "show_det: " << show_det_ << std::endl;
    std::cout << "show_seg: " << show_seg_ << std::endl;
    std::cout << "save_data: " << save_data_ << std::endl;
    std::cout << "save_depth: " << save_depth_ << std::endl;
    std::cout << "save_fps: " << save_fps << std::endl;
    std::cout << "color_topic: " << color_topic_ << std::endl;
    std::cout << "depth_topic: " << depth_topic_ << std::endl;
    std::cout << "intrin_topic: " << intrin_topic_ << std::endl;
    std::cout << "detection_model_path: " << detection_model_path << std::endl;
    std::cout << "segmentation_model_path: " << segmentation_model_path << std::endl;
    save_every_n_frame_ = std::max(1, save_fps > 0 ? 30 / save_fps : 1);
    std::cout << "save_every_n_frame: " << save_every_n_frame_ << std::endl;

    // read camera param
    if (!node["camera"]) {
        // TODO(SS): throw exception here
        std::cerr << "no camera param found here" << std::endl;
        return;
    } else {
        if (color_topic_.empty())
        {
            color_topic_ = node["camera"]["color_topic"].as<std::string>();
        }
        if (intrin_topic_.empty())
        {
            intrin_topic_ = node["camera"]["intrin_topic"].as<std::string>();
        }
        if (depth_topic_.empty())
        {
            depth_topic_ = node["camera"]["depth_topic"].as<std::string>();
        }
        intr_ = Intrinsics(node["camera"]["intrin"]);
        p_eye2head_ = as_or<Pose>(node["camera"]["extrin"], Pose());

        float pitch_comp = as_or<float>(node["camera"]["pitch_compensation"], 0.0);
        float yaw_comp = as_or<float>(node["camera"]["yaw_compensation"], 0.0);
        float z_comp = as_or<float>(node["camera"]["z_compensation"], 0.0);

        p_headprime2head_ = Pose(0, 0, z_comp, 0, pitch_comp * M_PI / 180, yaw_comp * M_PI / 180);
    }

    // init detector
    if (!node["detection_model"]) {
        std::cerr << "no detection model param here" << std::endl;
        return;
    } else {
        detector_ = YoloV8Detector::CreateYoloV8Detector(node["detection_model"], detection_model_path);
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

    if (!node["segmentation_model"]) {
        std::cerr << "no segmentation model param found" << std::endl;
    } else {
        segmentor_ = YoloV8Segmentor::CreateYoloV8Segmentor(node["segmentation_model"], segmentation_model_path);
    }

    // add detector_ warmup

    // init data_syncer
    use_depth_ = as_or<bool>(node["use_depth"], false);
    data_syncer_ = std::make_shared<DataSyncer>(use_depth_);
    bool save_data_nonstationary = as_or<bool>(node["misc"]["save_data_nonstationary"], true);
    std::string log_root = std::string(std::getenv("HOME")) + "/Workspace/vision_log/" + getTimeString();
    data_logger_ = save_data_ ? std::make_shared<DataLogger>(log_root, save_data_nonstationary) : nullptr;
    data_logger_->LogYAML(node, "vision_local.yaml");
    seg_data_syncer_ = std::make_shared<DataSyncer>(false);


    std::string robot_name = as_or<std::string>(node["robot_name"], "");
    std::string topic_suffix = "";
    if (robot_name.empty()) {
        robot_name = "robot0";
        topic_suffix = "";
    } else {
        topic_suffix = "/" + robot_name;
    }

    // init robot color classifier
    if (node["robot_color_classifier"]) {
        color_classifier_ = std::make_shared<ColorClassifier>();
        color_classifier_->Init(node["robot_color_classifier"]);
    }

    // init pose estimator
    pose_estimator_ = std::make_shared<PoseEstimator>(intr_);
    pose_estimator_->Init(YAML::Node());
    pose_estimator_map_["default"] = pose_estimator_;

    if (node["ball_pose_estimator"]) {
        pose_estimator_map_["ball"] = std::make_shared<BallPoseEstimator>(intr_);
        pose_estimator_map_["ball"]->Init(node["ball_pose_estimator"]);
    }

    if (node["human_like_pose_estimator"]) {
        pose_estimator_map_["human_like"] = std::make_shared<HumanLikePoseEstimator>(intr_);
        pose_estimator_map_["human_like"]->Init(node["human_like_pose_estimator"]);
    }

    if (node["field_marker_pose_estimator"]) {
        pose_estimator_map_["field_marker"] = std::make_shared<FieldMarkerPoseEstimator>(intr_);
        pose_estimator_map_["field_marker"]->Init(node["field_marker_pose_estimator"]);

        line_segment_area_threshold_ = as_or<int>(node["field_marker_pose_estimator"]["line_segment_area_threshold"], 75);
    }

    // init ros related

    if (color_topic_.find("robot0_rgbd_camera") != std::string::npos && !robot_name.empty()) { // sim
        color_topic_ = color_topic_.replace(color_topic_.find("robot0_rgbd_camera"), std::string("robot0_rgbd_camera").length(), robot_name + "_rgbd_camera");
    }
    if (depth_topic_.find("robot0_rgbd_camera") != std::string::npos && !robot_name.empty()) { // sim
        depth_topic_ = depth_topic_.replace(depth_topic_.find("robot0_rgbd_camera"), std::string("robot0_rgbd_camera").length(), robot_name + "_rgbd_camera");
    }
    if (intrin_topic_.find("robot0_rgbd_camera") != std::string::npos && !robot_name.empty()) { // sim
        intrin_topic_ = intrin_topic_.replace(intrin_topic_.find("robot0_rgbd_camera"), std::string("robot0_rgbd_camera").length(), robot_name + "_rgbd_camera");
    }
    

    // Subscribe to camera info to update intrinsics dynamically
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        intrin_topic_, rclcpp::QoS(1).best_effort(),
        std::bind(&VisionNode::CameraInfoCallback, this, std::placeholders::_1));
    std::cout << "Subscribing to camera info topic: " << intrin_topic_ << std::endl;

    callback_group_sub_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub_3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub_4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_1 = rclcpp::SubscriptionOptions();
    sub_opt_1.callback_group = callback_group_sub_1_;
    auto sub_opt_2 = rclcpp::SubscriptionOptions();
    sub_opt_2.callback_group = callback_group_sub_2_;
    auto sub_opt_3 = rclcpp::SubscriptionOptions();
    sub_opt_3.callback_group = callback_group_sub_3_;
    auto sub_opt_4 = rclcpp::SubscriptionOptions();
    sub_opt_4.callback_group = callback_group_sub_4_;

    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_transport::TransportHints hints(this, "compressed");
    
    // Subscribe to color image (raw or compressed)
    if (color_topic_.find("compressed") != std::string::npos) {
        std::cout << "Subscribing to compressed color topic: " << color_topic_ << std::endl;
        compressed_color_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            color_topic_, 1, std::bind(&VisionNode::CompressedColorCallback, this, std::placeholders::_1), sub_opt_1);
    } else {
        std::cout << "Subscribing to raw color topic: " << color_topic_ << std::endl;
        color_sub_ = it_->subscribe(color_topic_, 1, &VisionNode::ColorCallback, this, nullptr, sub_opt_1);
    }
    
    // Subscribe to depth image (raw or compressed)
    if (use_depth_ && depth_topic_.find("compressed") != std::string::npos) {
        std::cout << "Subscribing to compressed depth topic: " << depth_topic_ << std::endl;
        compressed_depth_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            depth_topic_, 1, std::bind(&VisionNode::CompressedDepthCallback, this, std::placeholders::_1), sub_opt_3);
    } else if (use_depth_) {
        std::cout << "Subscribing to raw depth topic: " << depth_topic_ << std::endl;
        depth_sub_ = it_->subscribe(depth_topic_, 1, &VisionNode::DepthCallback, this, nullptr, sub_opt_3);
    }

    detection_pub_ = this->create_publisher<vision_interface::msg::Detections>("/booster_soccer/detection" + topic_suffix, rclcpp::QoS(1));

    if (node["segmentation_model"]) {
        std::cout << "create sub for segmentor" << std::endl;
        if (color_topic_.find("compressed") != std::string::npos) {
            compressed_color_seg_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                color_topic_, 1, std::bind(&VisionNode::CompressedSegmentationCallback, this, std::placeholders::_1), sub_opt_2);
        } else {
            color_seg_sub_ = it_->subscribe(color_topic_, 1, &VisionNode::SegmentationCallback, this, nullptr, sub_opt_2);
        }
        field_line_pub_ = this->create_publisher<vision_interface::msg::LineSegments>("/booster_soccer/line_segments" + topic_suffix, rclcpp::QoS(1));
    }
    ball_pub_ = this->create_publisher<vision_interface::msg::Ball>("/booster_soccer/ball" + topic_suffix, rclcpp::QoS(1));

    if (offline_mode_) {
        pose_tf_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/booster_soccer/t_head2base" + topic_suffix, 10, std::bind(&VisionNode::PoseTFCallBack, this, std::placeholders::_1));
    } else {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/head_pose" + topic_suffix, 10, std::bind(&VisionNode::PoseCallBack, this, std::placeholders::_1), sub_opt_4);
        calParam_sub_ = this->create_subscription<vision_interface::msg::CalParam>("/booster_soccer/cal_param" + topic_suffix, 10, std::bind(&VisionNode::CalParamCallback, this, std::placeholders::_1));
        pose_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/booster_soccer/t_head2base" + topic_suffix, rclcpp::QoS(10));
    }
}

void VisionNode::ProcessData(SyncedDataBlock &synced_data, vision_interface::msg::Detections &detection_msg) {
    double timestamp = synced_data.color_data.timestamp;
    double depth_time_diff = (timestamp - synced_data.depth_data.timestamp) * 1000;
    double pose_time_diff = (timestamp - synced_data.pose_data.timestamp) * 1000;
    if (use_depth_ && depth_time_diff > 40) {
        std::cerr << "color depth time diff: " << depth_time_diff << "ms" << std::endl;
    }
    if (pose_time_diff > 40) {
        std::cerr << "color pose time diff: " << pose_time_diff << " ms" << std::endl;
    }
    cv::Mat color = synced_data.color_data.data;
    cv::Mat depth = synced_data.depth_data.data;

    cv::Mat depth_float;
    if (!depth.empty() && depth.depth() == CV_16U) {
        depth.convertTo(depth_float, CV_32F, 0.001, 0);
    } else {
        depth_float = depth;
    }

    Pose p_head2base = synced_data.pose_data.data;
    Pose p_eye2base = p_head2base * p_headprime2head_ * p_eye2head_;
    std::cout << "det: p_eye2base: \n"
              << p_eye2base.toCVMat() << std::endl;

    // inference
    auto detections = detector_->Inference(color);
    std::cout << detections.size() << " objects detected." << std::endl;

    auto get_estimator = [&](const std::string &class_name) {
        if (class_name == "Ball") {
            return pose_estimator_map_.find("ball") != pose_estimator_map_.end() ? pose_estimator_map_["ball"] : pose_estimator_map_["default"];
        } else if (class_name == "Person" || class_name == "Opponent" || class_name == "Goalpost") {
            return pose_estimator_map_.find("human_like") != pose_estimator_map_.end() ? pose_estimator_map_["human_like"] : pose_estimator_map_["default"];
        } else if (class_name.find("Cross") != std::string::npos || class_name == "PenaltyPoint") {
            return pose_estimator_map_.find("field_marker") != pose_estimator_map_.end() ? pose_estimator_map_["field_marker"] : pose_estimator_map_["default"];
        } else {
            return pose_estimator_map_["default"];
        }
    };

    std::vector<booster_vision::DetectionRes> filtered_detections;
    if (enable_post_process_ && !detections.empty()) {
        // filter detections with different confidence
        if (!confidence_map_.empty()) {
            for (auto &detection : detections) {
                auto classname = classnames_[detection.class_id];
                if (detection.confidence < confidence_map_[classname]) {
                    continue;
                }
                filtered_detections.push_back(detection);
            }
        } else {
            filtered_detections = detections;
        }

        // keep the highest ball detections
        if (single_ball_assumption_) {
            std::vector<booster_vision::DetectionRes> ball_detections;
            std::vector<booster_vision::DetectionRes> filtered_detections_bk = filtered_detections;
            filtered_detections.clear();

            for (const auto &detection : filtered_detections_bk) {
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
        filtered_detections = detections;
    }

    std::vector<booster_vision::DetectionRes> detections_for_display;
    for (auto &detection : filtered_detections) {
        vision_interface::msg::DetectedObject detection_obj;

        detection.class_name = detector_->kClassLabels[detection.class_id];

        auto pose_estimator = get_estimator(detection.class_name);
        Pose pose_obj_by_color = pose_estimator->EstimateByColor(p_eye2base, detection, color);
        Pose pose_obj_by_depth = pose_estimator->EstimateByDepth(p_eye2base, detection, color, depth_float);

        // filter out incorrect ball detection
        if (pose_estimator->use_depth_ && detection.class_name == "Ball" && pose_obj_by_depth == Pose()) {
            std::cout << "filtered out ball detection by depth" << std::endl;
            continue;
        }
        detection_obj.position_projection = pose_obj_by_color.getTranslationVec();
        detection_obj.position = pose_obj_by_depth.getTranslationVec();

        auto xyz = p_head2base.getTranslationVec();
        auto rpy = p_head2base.getEulerAnglesVec();
        detection_obj.received_pos = {xyz[0], xyz[1], xyz[2],
                                      static_cast<float>(rpy[0] / CV_PI * 180), static_cast<float>(rpy[1] / CV_PI * 180), static_cast<float>(rpy[2] / CV_PI * 180)};

        detection_obj.confidence = detection.confidence * 100;
        detection_obj.xmin = detection.bbox.x;
        detection_obj.ymin = detection.bbox.y;
        detection_obj.xmax = detection.bbox.x + detection.bbox.width;
        detection_obj.ymax = detection.bbox.y + detection.bbox.height;
        detection_obj.label = detection.class_name;

        if ((color_classifier_ != nullptr) && (detection.class_name == "Opponent")) {
            // get a crop of the image given detection.bbox
            cv::Mat crop = color(detection.bbox);
            std::string robot_color_str = color_classifier_->Classify(crop);
            // add robot color to detection_obj
            detection_obj.color = robot_color_str;
        }

        // publish detection
        detection_msg.detected_objects.push_back(detection_obj);
        detections_for_display.push_back(detection);
    }

    // compute corner points positision
    std::vector<cv::Point2f> corner_uvs = {cv::Point2f(0, 0), cv::Point2f(color.cols - 1, 0),
                                           cv::Point2f(color.cols - 1, color.rows - 1), cv::Point2f(0, color.rows - 1),
                                           cv::Point2f(color.cols / 2.0, color.rows / 2.0)};
    for (auto &uv : corner_uvs) {
        auto corner_pos = CalculatePositionByIntersection(p_eye2base, uv, intr_);
        detection_msg.corner_pos.push_back(corner_pos.x);
        detection_msg.corner_pos.push_back(corner_pos.y);
    }

    // sync-radar measurements

    // publish msg
    detection_pub_->publish(detection_msg);
    std::cout << std::endl;

    {
        static double last_pub_time = -1.0;
        static uint64_t count = 0;
        double pub_ts = detection_msg.header.stamp.sec +
                        static_cast<double>(detection_msg.header.stamp.nanosec) * 1e-9;
        if (last_pub_time >= 0.0) {
            double diff_ms = (pub_ts - last_pub_time) * 1000.0;
            std::cout << "[Detections Pub Interval] #" << (count) 
                      << " -> #" << (count + 1) << ": " << diff_ms << " ms" << std::endl;
        }
        last_pub_time = pub_ts;
        count++;
    }

    // show vision results
    if (show_det_) {
        cv::Mat color_rgb;
        cv::cvtColor(color, color_rgb, cv::COLOR_BGR2RGB);
        cv::Mat img_out = YoloV8Detector::DrawDetection(color_rgb, detections_for_display);
        cv::imshow("Detection", img_out);

        // color jet depth_float and show
        // if (!depth_float.empty()) {
        //     cv::Mat depth_colormap;
        //     cv::normalize(depth_float, depth_float, 0, 255, cv::NORM_MINMAX);
        //     depth_float.convertTo(depth_float, CV_8U);
        //     cv::applyColorMap(depth_float, depth_colormap, cv::COLORMAP_JET);
        //     cv::imshow("Depth", depth_colormap);
        // }

        cv::waitKey(1);
    }

    if (save_data_) {
        save_cnt_++;
        if (save_cnt_ % save_every_n_frame_ != 0) {
            return;
        } else {
            save_cnt_ = 0;
        }
        data_logger_->LogDataBlock(synced_data);
    }
}

void VisionNode::ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    std::cout << "new color for det received" << std::endl;
    auto start = std::chrono::system_clock::now();
    if (!msg) {
        std::cerr << "empty image message." << std::endl;
        return;
    }

    cv::Mat img;
    try {
        img = toCVMat(*msg);
    } catch (std::exception &e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    if (msg->encoding == "rgb8") {
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    }

    vision_interface::msg::Detections detection_msg;
    detection_msg.header = msg->header;
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // get synced data
    SyncedDataBlock synced_data = data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    
    ProcessData(synced_data, detection_msg);
    auto end = std::chrono::system_clock::now();
    std::cout << "color callback takes: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;
}

void VisionNode::CompressedColorCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    std::cout << "new compressed color for det received" << std::endl;
    auto start = std::chrono::system_clock::now();
    if (!msg) {
        std::cerr << "empty compressed image message." << std::endl;
        return;
    }

    cv::Mat img;
    try {
        // Decode compressed image
        img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "Failed to decode compressed image" << std::endl;
            return;
        }
    } catch (std::exception &e) {
        std::cerr << "decode exception: " << e.what() << std::endl;
        return;
    }

    vision_interface::msg::Detections detection_msg;
    detection_msg.header = msg->header;
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // get synced data
    SyncedDataBlock synced_data = data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    
    ProcessData(synced_data, detection_msg);
    auto end = std::chrono::system_clock::now();
    std::cout << "compressed color callback takes: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;
}

void VisionNode::ProcessSegmentationData(SyncedDataBlock &synced_data, vision_interface::msg::LineSegments &field_line_segs_msg) {
    double timestamp = synced_data.color_data.timestamp;
    cv::Mat color = synced_data.color_data.data;
    Pose p_head2base = synced_data.pose_data.data;
    Pose p_eye2base = p_head2base * p_headprime2head_ * p_eye2head_;

    double time_diff = (timestamp - synced_data.pose_data.timestamp) * 1000;
    if (time_diff > 40) {
        std::cerr << "seg: color pose time diff: " << time_diff << " ms" << std::endl;
    }
    std::cout << "seg: p_eye2base: \n"
              << p_eye2base.toCVMat() << std::endl;

    // inference
    auto segmentations = segmentor_->Inference(color);
    std::vector<FieldLineSegment> field_line_segs;
    for (auto &seg : segmentations) {

        auto line_segs = FitFieldLineSegments(p_eye2base, intr_, seg.contour, line_segment_area_threshold_);
        for (auto line_seg : line_segs) {
            float inlier_precentage = static_cast<float>(line_seg.inlier_count) / line_seg.contour_2d_points.size();
            if (inlier_precentage < 0.25) {
                continue;
            }
            field_line_segs_msg.coordinates.push_back(line_seg.end_points_3d[0].x);
            field_line_segs_msg.coordinates.push_back(line_seg.end_points_3d[0].y);
            field_line_segs_msg.coordinates.push_back(line_seg.end_points_3d[1].x);
            field_line_segs_msg.coordinates.push_back(line_seg.end_points_3d[1].y);

            field_line_segs_msg.coordinates_uv.push_back(line_seg.end_points_2d[0].x);
            field_line_segs_msg.coordinates_uv.push_back(line_seg.end_points_2d[0].y);
            field_line_segs_msg.coordinates_uv.push_back(line_seg.end_points_2d[1].x);
            field_line_segs_msg.coordinates_uv.push_back(line_seg.end_points_2d[1].y);

            field_line_segs.push_back(line_seg);
        }
    }
    std::cout << segmentations.size() << " objects segmented." << std::endl;

    field_line_pub_->publish(field_line_segs_msg);
    if (show_seg_) {
        cv::Mat img_out = YoloV8Segmentor::DrawSegmentation(color, segmentations);
        img_out = DrawFieldLineSegments(img_out, field_line_segs);
        cv::imshow("Segmentation", img_out);
        cv::waitKey(1);
    }
}

void VisionNode::SegmentationCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (!segmentor_) {
        std::cerr << "no segmentor loaded." << std::endl;
        return;
    }
    std::cout << "new color for seg received" << std::endl;
    if (!msg) {
        std::cerr << "empty image message." << std::endl;
        return;
    }

    // cv_bridge::CvImagePtr cv_ptr; // make use of cv_bridge to convert ROS image message to OpenCV cv::Mat format
    cv::Mat img;
    try {
        // cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        img = toCVMat(*msg).clone();
    } catch (std::exception &e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    vision_interface::msg::LineSegments field_line_segs_msg;
    field_line_segs_msg.header = msg->header;
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // get synced data
    SyncedDataBlock synced_data = seg_data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    ProcessSegmentationData(synced_data, field_line_segs_msg);
}

void VisionNode::CompressedSegmentationCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    if (!segmentor_) {
        std::cerr << "no segmentor loaded." << std::endl;
        return;
    }
    std::cout << "new compressed color for seg received" << std::endl;
    if (!msg) {
        std::cerr << "empty compressed image message." << std::endl;
        return;
    }

    cv::Mat img;
    try {
        img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "Failed to decode compressed image" << std::endl;
            return;
        }
    } catch (std::exception &e) {
        std::cerr << "decode exception: " << e.what() << std::endl;
        return;
    }

    vision_interface::msg::LineSegments field_line_segs_msg;
    field_line_segs_msg.header = msg->header;
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // get synced data
    SyncedDataBlock synced_data = seg_data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    ProcessSegmentationData(synced_data, field_line_segs_msg);
}

void VisionNode::DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    std::cout << "new depth received" << std::endl;
    // cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    try {
        // TODO(SS): check if the image is 16-bit for zed camera
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        img = toCVMat(*msg).clone();
    } catch (std::exception &e) {
        std::cerr << "cv_bridge exception " << e.what() << std::endl;
        return;
    }

    if (img.empty()) {
        std::cerr << "empty image recevied." << std::endl;
        return;
    }

    // Check if the image is indeed 16-bit
    if (img.depth() != CV_16U && img.depth() != CV_32F) {
        std::cerr << "image is either 16u or 32f." << std::endl;
        return;
    }

    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    data_syncer_->AddDepth(DepthDataBlock(img, timestamp));
    // seg_data_syncer_->AddDepth(DepthDataBlock(img, timestamp));
}

void VisionNode::CompressedDepthCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    std::cout << "new compressed depth received" << std::endl;
    cv::Mat img;
    try {
        cv::Mat compressed_data = cv::Mat(msg->data);
        img = cv::imdecode(compressed_data, cv::IMREAD_ANYDEPTH);
        
        if (img.empty()) {
            std::cerr << "Failed to decode compressed depth image" << std::endl;
            return;
        }
    } catch (std::exception &e) {
        std::cerr << "decode exception " << e.what() << std::endl;
        return;
    }

    if (img.empty()) {
        std::cerr << "empty image received." << std::endl;
        return;
    }

    // Check if the image is indeed 16-bit or 32-bit float
    if (img.depth() != CV_16U && img.depth() != CV_32F) {
        std::cerr << "image must be either 16u or 32f." << std::endl;
        return;
    }

    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    data_syncer_->AddDepth(DepthDataBlock(img, timestamp));
    // seg_data_syncer_->AddDepth(DepthDataBlock(img, timestamp));
}

void VisionNode::PoseTFCallBack(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    data_syncer_->AddPose(PoseDataBlock(Pose(*msg), timestamp));
    seg_data_syncer_->AddPose(PoseDataBlock(Pose(*msg), timestamp));
}

void VisionNode::PoseCallBack(const geometry_msgs::msg::Pose::SharedPtr msg) {
    auto current_time = this->get_clock()->now();
    double timestamp = static_cast<double>(current_time.nanoseconds()) * 1e-9;

    float x = msg->position.x;
    float y = msg->position.y;
    float z = msg->position.z;
    float qx = msg->orientation.x;
    float qy = msg->orientation.y;
    float qz = msg->orientation.z;
    float qw = msg->orientation.w;
    auto pose = Pose(x, y, z, qx, qy, qz, qw);
    data_syncer_->AddPose(PoseDataBlock(pose, timestamp));
    seg_data_syncer_->AddPose(PoseDataBlock(pose, timestamp));

    if (!offline_mode_) {
        auto tf_msg = pose.toRosTFMsg();
        tf_msg.header.stamp = builtin_interfaces::msg::Time(current_time);
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "head_pose";

        pose_tf_pub_->publish(tf_msg);
    }
}

void VisionNode::CalParamCallback(const vision_interface::msg::CalParam::SharedPtr msg) {
    float pitch_comp = msg->pitch_compensation;
    float yaw_comp = msg->yaw_compensation;
    float z_comp = msg->z_compensation;
    std::cout << "calParams: " << pitch_comp << " " << yaw_comp << " " << z_comp << std::endl;
    p_headprime2head_ = Pose(0, 0, z_comp, 0, pitch_comp * M_PI / 180, yaw_comp * M_PI / 180);
}

void VisionNode::CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!msg) {
        std::cerr << "empty camera info message." << std::endl;
        return;
    }

    // Track if this is the first update
    static bool first_update = true;
    
    float fx = msg->k[0];
    float fy = msg->k[4];
    float cx = msg->k[2];
    float cy = msg->k[5];

    std::vector<float> distortion_coeffs(msg->d.begin(), msg->d.end());

    // Determine distortion model based on camera type
    Intrinsics::DistortionModel distortion_model;

    if (msg->d.empty())
    {
        distortion_model = Intrinsics::DistortionModel::kNone;
    } else {
        // Heuristic: if distortion coefficients are all zero, treat as no distortion
        float distortion_sum = 0.0;
        for (size_t i = 0; i < msg->d.size(); ++i) {
            distortion_sum += std::abs(msg->d[i]);
        }
        if (distortion_sum < 1e-6) {
            distortion_model = Intrinsics::DistortionModel::kNone;
        } else {
            if (msg->distortion_model == "plumb_bob") {
                distortion_model = Intrinsics::DistortionModel::kInverseBrownConrady;
            } else  {
                distortion_model = Intrinsics::DistortionModel::kBrownConrady;
            }
        }
    }
    // Check if distortion coefficients are effectively zero
    float distortion_sum = 0.0;
    for (auto coeff : distortion_coeffs) {
        distortion_sum += std::abs(coeff);
    }
    
    Intrinsics new_intr;
    if (distortion_sum < 1e-6 || distortion_coeffs.empty()) {
        new_intr = Intrinsics(fx, fy, cx, cy);
        std::cout << "Camera has no distortion" << std::endl;
    } else {
        new_intr = Intrinsics(fx, fy, cx, cy, distortion_coeffs, distortion_model);
        std::cout << "Camera has distortion model: " << static_cast<int>(distortion_model) << std::endl;
    }
    
    // Check if values have changed significantly (or if this is the first update)
    bool should_update = first_update || 
                        std::abs(intr_.fx - fx) > 0.1 || 
                        std::abs(intr_.fy - fy) > 0.1 || 
                        std::abs(intr_.cx - cx) > 0.1 || 
                        std::abs(intr_.cy - cy) > 0.1;
    
    if (should_update) {
        intr_ = new_intr;
        std::cout << "Camera intrinsics updated from topic:" << std::endl;
        std::cout << "  fx=" << fx << ", fy=" << fy << std::endl;
        std::cout << "  cx=" << cx << ", cy=" << cy << std::endl;
        std::cout << "  width=" << msg->width << ", height=" << msg->height << std::endl;
        std::cout << "  distortion_model=" << msg->distortion_model << std::endl;
        if (!distortion_coeffs.empty()) {
            std::cout << "  distortion_coeffs: [";
            for (size_t i = 0; i < distortion_coeffs.size(); ++i) {
                std::cout << distortion_coeffs[i];
                if (i < distortion_coeffs.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
        
        // Recreate all pose estimators with new intrinsics
        pose_estimator_map_.clear();
        
        pose_estimator_ = std::make_shared<PoseEstimator>(intr_);
        pose_estimator_->Init(YAML::Node());
        pose_estimator_map_["default"] = pose_estimator_;
        std::cout << "Created default pose estimator with updated intrinsics" << std::endl;
        
        if (config_node_["ball_pose_estimator"]) {
            pose_estimator_map_["ball"] = std::make_shared<BallPoseEstimator>(intr_);
            pose_estimator_map_["ball"]->Init(config_node_["ball_pose_estimator"]);
            std::cout << "Created ball pose estimator with updated intrinsics" << std::endl;
        }
        
        if (config_node_["human_like_pose_estimator"]) {
            pose_estimator_map_["human_like"] = std::make_shared<HumanLikePoseEstimator>(intr_);
            pose_estimator_map_["human_like"]->Init(config_node_["human_like_pose_estimator"]);
            std::cout << "Created human_like pose estimator with updated intrinsics" << std::endl;
        }
        
        if (config_node_["field_marker_pose_estimator"]) {
            pose_estimator_map_["field_marker"] = std::make_shared<FieldMarkerPoseEstimator>(intr_);
            pose_estimator_map_["field_marker"]->Init(config_node_["field_marker_pose_estimator"]);
            std::cout << "Created field_marker pose estimator with updated intrinsics" << std::endl;
        }
        
        first_update = false;
    }
    
    // After receiving several stable updates, unsubscribe to save resources
    static int received_count = 0;
    received_count++;
    if (received_count >= 5) {
        camera_info_sub_.reset();
        std::cout << "Camera info callback disabled after " << received_count << " messages" << std::endl;
        std::cout << "Final camera intrinsics:" << std::endl << intr_ << std::endl;
    }
}

} // namespace booster_vision
