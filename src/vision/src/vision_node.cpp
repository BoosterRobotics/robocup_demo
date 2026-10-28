#include "booster_vision/vision_node.h"

#include <cstdlib>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
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

namespace {

bool IntrinsicsEquivalent(const Intrinsics &lhs, const Intrinsics &rhs) {
    constexpr float kTolerance = 1e-5f;
    const auto nearly_equal = [kTolerance](float a, float b) {
        return std::abs(a - b) <= kTolerance * std::max(1.0f, std::max(std::abs(a), std::abs(b)));
    };
    if (!nearly_equal(lhs.fx, rhs.fx) || !nearly_equal(lhs.fy, rhs.fy) ||
        !nearly_equal(lhs.cx, rhs.cx) || !nearly_equal(lhs.cy, rhs.cy) ||
        lhs.model != rhs.model || lhs.distortion_coeffs.size() != rhs.distortion_coeffs.size()) {
        return false;
    }
    for (size_t i = 0; i < lhs.distortion_coeffs.size(); ++i) {
        if (!nearly_equal(lhs.distortion_coeffs[i], rhs.distortion_coeffs[i])) {
            return false;
        }
    }
    return true;
}

std::string NormalizeCameraInfoModel(std::string model) {
    std::transform(model.begin(), model.end(), model.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    model.erase(std::remove_if(model.begin(), model.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }), model.end());
    return model;
}

void DrawPositionLabels(cv::Mat &image,
                        const std::vector<DetectionRes> &detections,
                        const std::vector<cv::Point2f> &positions) {
    const size_t count = std::min(detections.size(), positions.size());
    constexpr double kFontScale = 0.5;
    constexpr int kTextThickness = 1;
    constexpr int kPadding = 3;

    for (size_t i = 0; i < count; ++i) {
        const auto &position = positions[i];
        if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
            continue;
        }

        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
               << "x=" << position.x << "m y=" << position.y << "m";
        const std::string label = stream.str();

        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
            label, cv::FONT_HERSHEY_SIMPLEX, kFontScale, kTextThickness, &baseline);
        const auto &bbox = detections[i].bbox;
        const int max_text_x = std::max(0, image.cols - text_size.width - 2 * kPadding);
        const int text_x = std::clamp(bbox.x, 0, max_text_x) + kPadding;

        int text_y = bbox.y + bbox.height + text_size.height + 2 * kPadding;
        if (text_y + baseline + kPadding >= image.rows) {
            text_y = bbox.y - 2 * kPadding;
        }
        text_y = std::clamp(
            text_y, text_size.height + kPadding,
            std::max(text_size.height + kPadding, image.rows - baseline - kPadding));

        const cv::Point background_top_left(
            text_x - kPadding, text_y - text_size.height - kPadding);
        const cv::Point background_bottom_right(
            text_x + text_size.width + kPadding, text_y + baseline + kPadding);
        cv::rectangle(image, background_top_left, background_bottom_right,
                      cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(image, label, cv::Point(text_x, text_y),
                    cv::FONT_HERSHEY_SIMPLEX, kFontScale,
                    cv::Scalar(255, 255, 255), kTextThickness, cv::LINE_AA);
    }
}

bool IsFiniteProjection(const std::vector<float> &position) {
    return position.size() >= 3 &&
           std::all_of(position.begin(), position.begin() + 3,
                       [](float value) { return std::isfinite(value); });
}

} // namespace

VisionNode::VisionNode(const std::string &node_name) :
    rclcpp::Node(node_name) {
    this->declare_parameter<bool>("offline_mode", false);
    this->declare_parameter<bool>("show_det", false);
    this->declare_parameter<bool>("show_seg", false);
    this->declare_parameter<bool>("save_data", true);
    this->declare_parameter<bool>("save_depth", true);
    this->declare_parameter<int>("save_fps", 3);
    this->declare_parameter<std::string>("detection_model_path", "");
    this->declare_parameter<std::string>("segmentation_model_path", "");
    this->declare_parameter<std::string>("camera_type", "");
    this->declare_parameter<std::string>("camera_info_topic", "");
    this->declare_parameter<double>("intrinsics_wait_timeout_sec", -1.0);
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

    this->get_parameter<bool>("show_det", show_det_);
    this->get_parameter<bool>("show_seg", show_seg_);
    this->get_parameter<bool>("save_data", save_data_);
    this->get_parameter<bool>("save_depth", save_depth_);
    this->get_parameter<bool>("offline_mode", offline_mode_);
    this->get_parameter<std::string>("camera_type", camera_type_);
    this->get_parameter<std::string>("detection_model_path", detection_model_path);
    std::cout << "detection_model_path origin: " << detection_model_path << std::endl;

    if(!detection_model_path.empty()){
        if(detection_model_path[0] == '/') {
            // absolute path, do nothing
        } else {
            std::string package_path = ament_index_cpp::get_package_share_directory("vision");
            detection_model_path = std::filesystem::path(package_path) / detection_model_path;
        }
    }


    this->get_parameter<std::string>("segmentation_model_path", segmentation_model_path);
    std::cout << "segmentation_model_path origin: " << segmentation_model_path << std::endl;

    if(!segmentation_model_path.empty()){
        if(segmentation_model_path[0] == '/') {
            // absolute path, do nothing
        } else {
            std::string package_path = ament_index_cpp::get_package_share_directory("vision");
            segmentation_model_path = std::filesystem::path(package_path) / segmentation_model_path;
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
    std::cout << "camera_type: " << camera_type_ << std::endl;
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
        if(camera_type_.empty())
        {
            std::cout << "camera type not overridden by launch file, using default: " << node["camera"]["type"].as<std::string>() << std::endl;
            camera_type_ = node["camera"]["type"].as<std::string>();
        }
        yaml_intr_ = Intrinsics(node["camera"]["intrin"]);
        intr_ = yaml_intr_;
        const std::string configured_camera_info_topic = as_or<std::string>(
            node["camera"]["camera_info_topic"], camera_info_topic_);
        if (!configured_camera_info_topic.empty()) {
            camera_info_topic_ = configured_camera_info_topic;
        }
        intrinsics_wait_timeout_sec_ = std::max(
            0.0, as_or<double>(
                node["camera"]["intrin_wait_timeout_sec"], intrinsics_wait_timeout_sec_));
        std::string camera_info_topic_override;
        double intrinsics_wait_timeout_override = -1.0;
        this->get_parameter<std::string>("camera_info_topic", camera_info_topic_override);
        this->get_parameter<double>(
            "intrinsics_wait_timeout_sec", intrinsics_wait_timeout_override);
        if (!camera_info_topic_override.empty()) {
            camera_info_topic_ = camera_info_topic_override;
        }
        if (std::isfinite(intrinsics_wait_timeout_override) &&
            intrinsics_wait_timeout_override >= 0.0) {
            intrinsics_wait_timeout_sec_ = intrinsics_wait_timeout_override;
        }
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
    if (data_logger_) {
        data_logger_->LogYAML(node, "vision_local.yaml");
    }
    seg_data_syncer_ = std::make_shared<DataSyncer>(false);

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

    intrinsics_wait_start_ = std::chrono::steady_clock::now();
    if (offline_mode_) {
        intrinsics_source_ = IntrinsicsSource::kYamlFallback;
        RCLCPP_INFO(
            get_logger(),
            "[vision][intrinsics] offline mode: using YAML intrinsics "
            "fx=%.6f fy=%.6f cx=%.6f cy=%.6f",
            yaml_intr_.fx, yaml_intr_.fy, yaml_intr_.cx, yaml_intr_.cy);
    } else {
        intrinsics_fallback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { ActivateYamlIntrinsicsIfTimedOut(); });
        RCLCPP_INFO(
            get_logger(),
            "[vision][intrinsics] waiting up to %.3f s for CameraInfo topic %s",
            intrinsics_wait_timeout_sec_, camera_info_topic_.c_str());
    }

    // init ros related

    std::cout << "current camera_type : " << camera_type_ << std::endl;
    std::string color_topic;
    std::string depth_topic;
    if (camera_type_.find("zed") != std::string::npos) {
        color_topic = "/boostercamera/head/rgb";
        depth_topic = "/boostercamera/head/depth";
    } else if (camera_type_ == "d-robotics") {
        color_topic = "/boostercamera/head/rgb";
        depth_topic = "/boostercamera/head/depth";
    } else if (camera_type_ == "orbbec") {
        color_topic = "/boostercamera/head/rgb";
        depth_topic = "/boostercamera/head/depth";
    } else {
        // realsense
        color_topic = "/boostercamera/head/rgb";
        depth_topic = "/boostercamera/head/depth";
    }

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
    // Subscribe to both raw and compressed image topics for color
    if (camera_type_.find("compressed") != std::string::npos) {
        color_sub_ = it_->subscribe(color_topic, 1, &VisionNode::ColorCallback, this, &hints, sub_opt_1);
    } else {
        color_sub_ = it_->subscribe(color_topic, 1, &VisionNode::ColorCallback, this, nullptr, sub_opt_1);
    } 
    if (use_depth_) {
        depth_sub_ = it_->subscribe(depth_topic, 1, &VisionNode::DepthCallback, this, nullptr, sub_opt_3);
    }
    if (camera_type_.find("compressed") != std::string::npos) {
        color_sub_ = it_->subscribe(color_topic, 2, &VisionNode::ColorCallback, this, &hints, sub_opt_1);
    } else {
        color_sub_ = it_->subscribe(color_topic, 2, &VisionNode::ColorCallback, this, nullptr, sub_opt_1);
    } 
    if (use_depth_) {
        depth_sub_ = it_->subscribe(depth_topic, 2, &VisionNode::DepthCallback, this, nullptr, sub_opt_3);
    }

    if (!offline_mode_) {
        const auto camera_info_qos = rclcpp::SensorDataQoS();
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, camera_info_qos,
            std::bind(&VisionNode::CameraInfoCallback, this, std::placeholders::_1), sub_opt_1);
    }

    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
    //     .reliability(rclcpp::ReliabilityPolicy::BestEffort)  // Use best effort for real-time performance
    //     .durability(rclcpp::DurabilityPolicy::Volatile);

    // color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     color_topic,
    //     qos_profile,
    //     std::bind(&VisionNode::ColorCallback, this, std::placeholders::_1),
    //     sub_opt_1);

    // depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     depth_topic,
    //     qos_profile,
    //     std::bind(&VisionNode::DepthCallback, this, std::placeholders::_1),
    //     sub_opt_3);

    detection_pub_ = this->create_publisher<vision_interface::msg::Detections>("/booster_vision/detection", rclcpp::QoS(1));

    if (node["segmentation_model"]) {
        std::cout << "create sub for segmentor" << std::endl;
        if (camera_type_.find("compressed") != std::string::npos) {
            color_seg_sub_ = it_->subscribe(color_topic, 1, &VisionNode::SegmentationCallback, this, &hints, sub_opt_2);
        } else {
            color_seg_sub_ = it_->subscribe(color_topic, 1, &VisionNode::SegmentationCallback, this, nullptr, sub_opt_2);
        } 
        field_line_pub_ = this->create_publisher<vision_interface::msg::LineSegments>("/booster_vision/line_segments", rclcpp::QoS(1));
    }
    ball_pub_ = this->create_publisher<vision_interface::msg::Ball>("/booster_vision/ball", rclcpp::QoS(1));

    if (offline_mode_) {
        pose_tf_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/booster_vision/t_head2base", 10, std::bind(&VisionNode::PoseTFCallBack, this, std::placeholders::_1));
    } else {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/head_pose", 10, std::bind(&VisionNode::PoseCallBack, this, std::placeholders::_1), sub_opt_4);
        calParam_sub_ = this->create_subscription<vision_interface::msg::CalParam>("/booster_vision/cal_param", 10, std::bind(&VisionNode::CalParamCallback, this, std::placeholders::_1));
        pose_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/booster_vision/t_head2base", rclcpp::QoS(10));
    }
}

void VisionNode::UpdatePoseEstimatorIntrinsicsLocked() {
    for (auto &item : pose_estimator_map_) {
        if (item.second) {
            item.second->SetIntrinsics(intr_);
        }
    }
}

void VisionNode::ApplyRuntimeIntrinsics(const Intrinsics &intrinsics, IntrinsicsSource source,
                                        uint32_t image_width, uint32_t image_height) {
    if (!std::isfinite(intrinsics.fx) || !std::isfinite(intrinsics.fy) ||
        !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy) ||
        intrinsics.fx <= 0.0f || intrinsics.fy <= 0.0f) {
        return;
    }

    bool changed = false;
    {
        std::unique_lock<std::shared_mutex> lock(intrinsics_mutex_);
        if (image_width > 0 && image_height > 0 && active_camera_image_width_ > 0 &&
            active_camera_image_height_ > 0 &&
            (active_camera_image_width_ != image_width || active_camera_image_height_ != image_height)) {
            return;
        }

        if (source == IntrinsicsSource::kTopic) {
            camera_info_width_ = image_width;
            camera_info_height_ = image_height;
        }
        changed = intrinsics_source_ != source || !IntrinsicsEquivalent(intr_, intrinsics);
        if (changed) {
            intr_ = intrinsics;
            UpdatePoseEstimatorIntrinsicsLocked();
            intrinsics_source_ = source;
        }
    }

    if (source == IntrinsicsSource::kTopic && intrinsics_fallback_timer_) {
        intrinsics_fallback_timer_->cancel();
    }
    if (changed) {
        RCLCPP_INFO(
            get_logger(),
            "[vision][intrinsics] using %s intrinsics: fx=%.6f fy=%.6f cx=%.6f cy=%.6f",
            source == IntrinsicsSource::kTopic ? "CameraInfo topic" : "YAML fallback",
            intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
    }
}

void VisionNode::UpdateActiveCameraImageSize(uint32_t image_width, uint32_t image_height) {
    if (image_width == 0 || image_height == 0) {
        return;
    }

    bool reverted_to_yaml = false;
    uint32_t info_width = 0;
    uint32_t info_height = 0;
    {
        std::unique_lock<std::shared_mutex> lock(intrinsics_mutex_);
        active_camera_image_width_ = image_width;
        active_camera_image_height_ = image_height;
        if (intrinsics_source_ == IntrinsicsSource::kTopic && camera_info_width_ > 0 &&
            camera_info_height_ > 0 &&
            (camera_info_width_ != image_width || camera_info_height_ != image_height)) {
            info_width = camera_info_width_;
            info_height = camera_info_height_;
            intr_ = yaml_intr_;
            UpdatePoseEstimatorIntrinsicsLocked();
            intrinsics_source_ = IntrinsicsSource::kYamlFallback;
            reverted_to_yaml = true;
        }
    }

    if (reverted_to_yaml) {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] CameraInfo size %ux%u does not match image size %ux%u; "
            "using YAML intrinsics for the current image",
            info_width, info_height, image_width, image_height);
    }
}

void VisionNode::ActivateYamlIntrinsicsIfTimedOut() {
    const double elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - intrinsics_wait_start_).count();
    if (elapsed_sec < intrinsics_wait_timeout_sec_) {
        return;
    }

    bool activated = false;
    {
        std::unique_lock<std::shared_mutex> lock(intrinsics_mutex_);
        if (intrinsics_source_ == IntrinsicsSource::kWaiting) {
            intr_ = yaml_intr_;
            UpdatePoseEstimatorIntrinsicsLocked();
            intrinsics_source_ = IntrinsicsSource::kYamlFallback;
            activated = true;
        }
    }
    if (intrinsics_fallback_timer_) {
        intrinsics_fallback_timer_->cancel();
    }
    if (activated) {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] no valid CameraInfo received within %.3f s; "
            "using YAML fallback (fx=%.6f fy=%.6f cx=%.6f cy=%.6f)",
            intrinsics_wait_timeout_sec_, yaml_intr_.fx, yaml_intr_.fy, yaml_intr_.cx, yaml_intr_.cy);
    }
}

bool VisionNode::EnsureIntrinsicsReady() {
    {
        std::shared_lock<std::shared_mutex> lock(intrinsics_mutex_);
        if (intrinsics_source_ != IntrinsicsSource::kWaiting) {
            return true;
        }
    }
    ActivateYamlIntrinsicsIfTimedOut();
    std::shared_lock<std::shared_mutex> lock(intrinsics_mutex_);
    return intrinsics_source_ != IntrinsicsSource::kWaiting;
}

void VisionNode::CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!msg) {
        return;
    }

    const float fx = static_cast<float>(msg->k[0]);
    const float fy = static_cast<float>(msg->k[4]);
    const float cx = static_cast<float>(msg->k[2]);
    const float cy = static_cast<float>(msg->k[5]);
    if (msg->width == 0 || msg->height == 0 ||
        !std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) ||
        fx <= 0.0f || fy <= 0.0f) {
        RCLCPP_WARN(get_logger(), "[vision][intrinsics] ignoring invalid CameraInfo dimensions or K matrix");
        return;
    }

    uint32_t image_width = 0;
    uint32_t image_height = 0;
    {
        std::shared_lock<std::shared_mutex> lock(intrinsics_mutex_);
        image_width = active_camera_image_width_;
        image_height = active_camera_image_height_;
    }
    if (image_width > 0 && image_height > 0 &&
        (image_width != msg->width || image_height != msg->height)) {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] ignoring CameraInfo size %ux%u for image size %ux%u",
            msg->width, msg->height, image_width, image_height);
        return;
    }

    const uint32_t effective_binning_x = msg->binning_x == 0 ? 1 : msg->binning_x;
    const uint32_t effective_binning_y = msg->binning_y == 0 ? 1 : msg->binning_y;
    const bool nontrivial_binning = effective_binning_x != 1 || effective_binning_y != 1;
    const bool roi_dimensions_set = msg->roi.width != 0 || msg->roi.height != 0;
    const bool roi_is_full_image =
        msg->roi.x_offset == 0 && msg->roi.y_offset == 0 &&
        (!roi_dimensions_set ||
         (msg->roi.width == msg->width && msg->roi.height == msg->height));
    const bool nontrivial_roi = !roi_is_full_image || msg->roi.do_rectify;
    if (nontrivial_binning || nontrivial_roi) {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] ignoring CameraInfo with binning/ROI that does not match "
            "the raw image (binning=%ux%u, roi=(%u,%u,%u,%u), rectify=%d)",
            effective_binning_x, effective_binning_y,
            msg->roi.x_offset, msg->roi.y_offset, msg->roi.width, msg->roi.height,
            msg->roi.do_rectify ? 1 : 0);
        return;
    }

    std::vector<float> distortion_coeffs(msg->d.begin(), msg->d.end());
    if (std::any_of(distortion_coeffs.begin(), distortion_coeffs.end(),
                    [](float coeff) { return !std::isfinite(coeff); })) {
        RCLCPP_WARN(get_logger(), "[vision][intrinsics] ignoring CameraInfo with non-finite distortion coefficients");
        return;
    }

    const std::string model_name = NormalizeCameraInfoModel(msg->distortion_model);
    const bool has_distortion = std::any_of(
        distortion_coeffs.begin(), distortion_coeffs.end(),
        [](float coeff) { return std::abs(coeff) > std::numeric_limits<float>::epsilon(); });
    Intrinsics::DistortionModel model = Intrinsics::DistortionModel::kNone;
    size_t minimum_coefficients = 0;
    if (model_name.empty() || model_name == "none") {
        model = Intrinsics::DistortionModel::kNone;
    } else if (model_name == "plumb_bob") {
        model = Intrinsics::DistortionModel::kBrownConrady;
        minimum_coefficients = 5;
    } else if (model_name == "rational_polynomial") {
        model = Intrinsics::DistortionModel::kBrownConrady;
        minimum_coefficients = 8;
    } else if (model_name == "inverse_brown_conrady") {
        model = Intrinsics::DistortionModel::kInverseBrownConrady;
        minimum_coefficients = 5;
    } else {
        RCLCPP_WARN(
            get_logger(), "[vision][intrinsics] unsupported CameraInfo distortion_model '%s'",
            msg->distortion_model.c_str());
        return;
    }
    if (has_distortion && (model == Intrinsics::DistortionModel::kNone ||
                           distortion_coeffs.size() < minimum_coefficients)) {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] distortion_model '%s' has insufficient coefficients (%zu, need %zu)",
            msg->distortion_model.c_str(), distortion_coeffs.size(), minimum_coefficients);
        return;
    }

    if (!has_distortion) {
        ApplyRuntimeIntrinsics(
            Intrinsics(fx, fy, cx, cy), IntrinsicsSource::kTopic, msg->width, msg->height);
    } else {
        ApplyRuntimeIntrinsics(
            Intrinsics(fx, fy, cx, cy, distortion_coeffs, model),
            IntrinsicsSource::kTopic, msg->width, msg->height);
    }
}

void VisionNode::ProcessData(SyncedDataBlock &synced_data, vision_interface::msg::Detections &detection_msg) {
    std::shared_lock<std::shared_mutex> intrinsics_lock(intrinsics_mutex_);
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
    std::vector<cv::Point2f> positions_for_display;
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
        const auto projected_position = pose_obj_by_color.getTranslationVec();
        if (pose_obj_by_color == Pose() || !IsFiniteProjection(projected_position)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "discarding %s detection with invalid ground intersection",
                detection.class_name.c_str());
            continue;
        }
        detection_obj.position_projection = projected_position;
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
        positions_for_display.emplace_back(projected_position[0], projected_position[1]);
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

    // 新增: 打印两次检测发布时间间隔
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

    vision_interface::msg::Ball ball_msg;
    ball_msg.header = detection_msg.header;
    ball_msg.confidence = 0;
    for (auto &detection : filtered_detections) {
        if (detection.class_name == "Ball" && detection.confidence > ball_msg.confidence) {
            auto pose_estimator = get_estimator(detection.class_name);
            Pose pose_obj_by_color = pose_estimator->EstimateByColor(p_eye2base, detection, color);
            if (pose_obj_by_color == Pose()) {
                continue;
            }
            auto value = pose_obj_by_color.getTranslationVec();
            if (!IsFiniteProjection(value) || value[0] < -2 || value[0] > 10 || value[1] < -5 || value[1] > 5) {
                continue;
            }
            ball_msg.x = value[0];
            ball_msg.y = value[1];
            ball_msg.confidence = detection.confidence;
            break;
        }
    }
    ball_pub_->publish(ball_msg);

    // show vision results
    if (show_det_) {
        cv::Mat color_rgb;
        cv::cvtColor(color, color_rgb, cv::COLOR_BGR2RGB);
        cv::Mat img_out = YoloV8Detector::DrawDetection(color_rgb, detections_for_display);
        DrawPositionLabels(img_out, detections_for_display, positions_for_display);
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

    // cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    try {
        // cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        img = toCVMat(*msg);
    } catch (std::exception &e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    UpdateActiveCameraImageSize(msg->width, msg->height);
    if (!EnsureIntrinsicsReady()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "[vision][intrinsics] dropping color frame while waiting for CameraInfo topic %s",
            camera_info_topic_.c_str());
        return;
    }

    if (camera_type_ == "realsense") {
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

void VisionNode::ProcessSegmentationData(SyncedDataBlock &synced_data, vision_interface::msg::LineSegments &field_line_segs_msg) {
    std::shared_lock<std::shared_mutex> intrinsics_lock(intrinsics_mutex_);
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
        // TODO: fit circle line
        if (seg.class_id == 0) continue;
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

    // cv_bridge::CvImagePtr cv_ptr; // 使用cv_bridge将ROS图像消息转换为OpenCV cv::Mat格式
    cv::Mat img;
    try {
        // cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        img = toCVMat(*msg).clone();
    } catch (std::exception &e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    UpdateActiveCameraImageSize(msg->width, msg->height);
    if (!EnsureIntrinsicsReady()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "[vision][intrinsics] dropping segmentation frame while waiting for CameraInfo topic %s",
            camera_info_topic_.c_str());
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

} // namespace booster_vision
