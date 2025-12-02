#include "booster_vision/vision_node.h"

#include <functional>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "vision_interface/msg/detected_object.hpp"
#include "vision_interface/msg/detections.hpp"

#include "booster_vision/base/data_syncer.hpp"
#include "booster_vision/model/detector.h"
#include "booster_vision/pose_estimator/pose_estimator.h"
#include "booster_vision/base/misc_utils.hpp"
#include "booster_vision/img_bridge.h"

namespace booster_vision {

void VisionNode::Init(const std::string &cfg_template_path, const std::string &cfg_path) {
    if (!std::filesystem::exists(cfg_template_path)) {
        std::cerr << "Error: Configuration template file '" << cfg_path << "' does not exist." << std::endl;
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

    std::cout << "loaded file: " << std::endl << node << std::endl;

    show_res_ = as_or<bool>(node["show_res"], false);

    this->declare_parameter<std::string>("robot_name", "");
    std::string robot_name = "";
    this->get_parameter("robot_name", robot_name);
    std::cout << "robot_name: " << robot_name << std::endl;
    std::string robot_prefix = robot_name != "" ? ("/" + robot_name) : "";
    std::cout << "robot_prefix: " << robot_prefix << std::endl;
    // read camera param
    if (!node["camera"]) {
        std::cerr << "no camera param found here" << std::endl;
        return;
    } else {
        intr_ = Intrinsics(node["camera"]["intrin"]);
        p_eye2head_ = as_or<Pose>(node["camera"]["extrin"], Pose());

        float pitch_comp = as_or<float>(node["camera"]["pitch_compensation"], 0.0);
        float yaw_comp = as_or<float>(node["camera"]["yaw_compensation"], 0.0);

        p_headprime2head_ = Pose(0, 0, 0, 0, pitch_comp * M_PI / 180, yaw_comp * M_PI / 180);
    }

    // init detector
    if (!node["detection_model"]) {
        std::cerr << "no detection model param here" << std::endl;
        return;
    } else {
        detector_ = YoloV8Detector::CreateYoloV8Detector(node["detection_model"]);
    }

    // init data_syncer
    use_depth_ = as_or<bool>(node["use_depth"], false);
    data_syncer_ = std::make_shared<DataSyncer>(use_depth_);

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

    // init ros related
    std::string color_topic = "/camera/camera/color/image_raw" + robot_prefix;
    std::string depth_topic = "/camera/camera/aligned_depth_to_color/image_raw" + robot_prefix;

    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    color_sub_ = it_->subscribe(color_topic, 1, std::bind(&VisionNode::ColorCallback, this, std::placeholders::_1));
    depth_sub_ = it_->subscribe(depth_topic, 1, std::bind(&VisionNode::DepthCallback, this, std::placeholders::_1));

    detection_pub_ = this->create_publisher<vision_interface::msg::Detections>("/booster_vision/detection" + robot_prefix, rclcpp::QoS(1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/head_pose" + robot_prefix, 10, std::bind(&VisionNode::PoseCallBack, this, std::placeholders::_1));
}

void VisionNode::ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    std::cout << "new color received" << std::endl;
    if (!msg) {
        std::cerr << "empty image message." << std::endl;
        return;
    }

    cv::Mat img;
    try {
        img = toCVMat(*msg);
    } catch (std::exception &e) {
        std::cerr << "converting msg to cv::Mat failed: " << e.what() << std::endl;
        return;
    }

    vision_interface::msg::Detections detection_msg;
    detection_msg.header = msg->header;
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // get synced data
    SyncedDataBlock synced_data = data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    cv::Mat color = synced_data.color_data.data;
    cv::Mat depth = synced_data.depth_data.data;
    Pose p_head2base = synced_data.pose_data.data;
    Pose p_eye2base = p_head2base * p_headprime2head_ * p_eye2head_;
    std::cout << "p_head2base: \n"
              << p_head2base.toCVMat() << std::endl;
    std::cout << "p_eye2base: \n"
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

    for (auto &detection : detections) {
        vision_interface::msg::DetectedObject detection_obj;

        detection.class_name = detector_->kClassLabels[detection.class_id];

        auto pose_estimator = get_estimator(detection.class_name);
        Pose pose_obj_by_color = pose_estimator->EstimateByColor(p_eye2base, detection, color);
        Pose pose_obj_by_depth = pose_estimator->EstimateByDepth(p_eye2base, detection, depth);
        detection_obj.position_projection = pose_obj_by_color.getTranslation();
        detection_obj.position = pose_obj_by_depth.getTranslation();

        auto xyz = p_head2base.getTranslation();
        auto rpy = p_head2base.getEulerAngles();
        std::vector<float> xyzrpy = {xyz[0], xyz[1], xyz[2], static_cast<float>(rpy[0] / CV_PI * 180), static_cast<float>(rpy[1] / CV_PI * 180), static_cast<float>(rpy[2] / CV_PI * 180)};
        detection_obj.received_pos = xyzrpy;

        detection_obj.confidence = detection.confidence * 100;
        detection_obj.xmin = detection.bbox.x;
        detection_obj.ymin = detection.bbox.y;
        detection_obj.xmax = detection.bbox.x + detection.bbox.width;
        detection_obj.ymax = detection.bbox.y + detection.bbox.height;
        detection_obj.label = detection.class_name;
        detection_msg.detected_objects.push_back(detection_obj);
    }

    // publish msg
    detection_pub_->publish(detection_msg);

    // show vision results
    if (show_res_) {
        cv::Mat img_out = YoloV8Detector::DrawDetection(color, detections);
        cv::imshow("Detection", img_out);
        cv::waitKey(1);
    }
}

void VisionNode::DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    std::cout << "new depth received" << std::endl;
    cv::Mat img;
    try {
        img = toCVMat(*msg);
    } catch (std::exception &e) {
        std::cerr << "cv_bridge exception " << e.what() << std::endl;
        return;
    }

    if (img.empty()) {
        std::cerr << "empty image recevied." << std::endl;
        return;
    }

    if (img.depth() != CV_16U) {
        std::cerr << "image is not 16-bit depth." << std::endl;
        return;
    }

    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    data_syncer_->AddDepth(DepthDataBlock(img, timestamp));
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
}

} // namespace booster_vision
