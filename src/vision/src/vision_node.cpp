#include "booster_vision/vision_node.h"

#include <cstdlib>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <functional>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <cstdio>
#include <regex>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <utility>

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

int64_t SteadyNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string ToLowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string ReadTextFile(const std::filesystem::path &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        return "";
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
}

std::string RunCommandCaptureStdout(const std::string &command) {
    std::array<char, 256> buffer{};
    std::string output;
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return output;
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        output += buffer.data();
    }
    pclose(pipe);
    return output;
}

std::string ExtractJsonStringField(
    const std::string &text, const std::string &field_name) {
    const std::regex quoted_field_regex(
        "\\\"" + field_name + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"",
        std::regex_constants::icase);
    std::smatch match;
    if (std::regex_search(text, match, quoted_field_regex) && match.size() >= 2) {
        return match[1].str();
    }

    // Keep a small fallback for booster-cli versions that print a human-readable
    // `Model: T2` line instead of JSON.
    const std::regex plain_field_regex(
        "(^|[\\n,])\\s*" + field_name + "\\s*[:=]\\s*([^,\\n}]+)",
        std::regex_constants::icase);
    if (std::regex_search(text, match, plain_field_regex) && match.size() >= 3) {
        std::string value = match[2].str();
        value.erase(value.begin(), std::find_if(value.begin(), value.end(),
                                                [](unsigned char c) {
                                                    return !std::isspace(c) && c != '\"';
                                                }));
        value.erase(std::find_if(value.rbegin(), value.rend(),
                                 [](unsigned char c) {
                                     return !std::isspace(c) && c != '\"';
                                 }).base(), value.end());
        return value;
    }
    return "";
}

bool ParseFloat3(const std::string &value, float *x, float *y, float *z) {
    std::istringstream iss(value);
    return static_cast<bool>(iss >> *x >> *y >> *z);
}

std::string ExtractXmlAttribute(const std::string &xml, const std::string &attr_name) {
    const std::regex attr_regex(attr_name + "\\s*=\\s*\"([^\"]*)\"");
    std::smatch match;
    if (std::regex_search(xml, match, attr_regex) && match.size() >= 2) {
        return match[1].str();
    }
    return "";
}

bool LoadFixedJointChildToParentPose(
    const std::filesystem::path &urdf_path,
    const std::string &child_link,
    Pose *child2parent) {
    if (child2parent == nullptr) {
        return false;
    }
    const std::string urdf = ReadTextFile(urdf_path);
    if (urdf.empty()) {
        std::cerr << "Failed to read URDF: " << urdf_path << std::endl;
        return false;
    }

    size_t search_pos = 0;
    while (true) {
        const size_t joint_start = urdf.find("<joint", search_pos);
        if (joint_start == std::string::npos) {
            break;
        }
        const size_t joint_end = urdf.find("</joint>", joint_start);
        if (joint_end == std::string::npos) {
            break;
        }
        search_pos = joint_end + std::string("</joint>").size();
        const std::string joint_xml = urdf.substr(joint_start, search_pos - joint_start);
        if (joint_xml.find("<child") == std::string::npos ||
            joint_xml.find("link=\"" + child_link + "\"") == std::string::npos) {
            continue;
        }

        const std::regex origin_regex("<origin\\b[^>]*/?>");
        std::smatch origin_match;
        if (!std::regex_search(joint_xml, origin_match, origin_regex)) {
            std::cerr << "URDF joint for child link " << child_link
                      << " has no origin." << std::endl;
            return false;
        }

        const std::string origin_xml = origin_match[0].str();
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        const std::string xyz = ExtractXmlAttribute(origin_xml, "xyz");
        if (!xyz.empty() && !ParseFloat3(xyz, &x, &y, &z)) {
            std::cerr << "Invalid xyz in URDF origin for child link "
                      << child_link << ": " << xyz << std::endl;
            return false;
        }
        const std::string rpy = ExtractXmlAttribute(origin_xml, "rpy");
        if (!rpy.empty() && !ParseFloat3(rpy, &roll, &pitch, &yaw)) {
            std::cerr << "Invalid rpy in URDF origin for child link "
                      << child_link << ": " << rpy << std::endl;
            return false;
        }

        *child2parent = Pose(x, y, z, roll, pitch, yaw);
        return true;
    }

    std::cerr << "URDF child link not found: " << child_link
              << " in " << urdf_path << std::endl;
    return false;
}

std::filesystem::path ResolveAutoCalibrationUrdfPath(
    const std::string &robot_model_lower, const std::string &urdf_name) {
    std::vector<std::filesystem::path> candidates;

    try {
        const std::filesystem::path package_path =
            ament_index_cpp::get_package_share_directory("vision");
        candidates.emplace_back(package_path / "config" / urdf_name);
    } catch (const std::exception &) {
        // The package share is unavailable in local/static checks; continue
        // with the source and robot-image paths below.
    }

    candidates.emplace_back(std::filesystem::path("config") / urdf_name);

    if (robot_model_lower.find("t2") != std::string::npos) {
        candidates.emplace_back(
            std::filesystem::path(
                "/home/yss/robocup_all/motion_train/booster_assets/robots/T2/T2_31dof") /
            urdf_name);
        candidates.emplace_back(
            std::filesystem::path(
                "/home/yss/robocup_all/motion_train/booster_motion/booster_config/"
                "robot_config/kidsize/booster_assets/robots/T2/T2_31dof") /
            urdf_name);
        candidates.emplace_back(
            std::filesystem::path("/opt/booster/booster_assets/robots/T2/T2_31dof") /
            urdf_name);
        candidates.emplace_back(
            std::filesystem::path("/opt/booster/robots/T2/T2_31dof") / urdf_name);
        // Production robot images may install the same model under the
        // firmware model directory and use a versioned filename.
        candidates.emplace_back(
            std::filesystem::path("/home/booster/etc/robot_models/T2") /
            urdf_name);
        candidates.emplace_back(
            std::filesystem::path("/home/booster/etc/robot_models/T2") /
            "T2_v1p2_31dof.urdf");
        candidates.emplace_back(
            std::filesystem::path("/opt/booster/robot_models/T2") /
            urdf_name);
        candidates.emplace_back(
            std::filesystem::path("/opt/booster/robot_models/T2") /
            "T2_v1p2_31dof.urdf");
    } else if (robot_model_lower.find("k1") != std::string::npos) {
        candidates.emplace_back(
            std::filesystem::path(
                "/home/yss/robocup_all/motion_train/booster_assets/robots/K1") /
            urdf_name);
        candidates.emplace_back(
            std::filesystem::path("/opt/booster/booster_assets/robots/K1") /
            urdf_name);
    } else if (robot_model_lower.find("t1") != std::string::npos) {
        candidates.emplace_back(
            std::filesystem::path(
                "/home/yss/robocup_all/motion_train/booster_assets/robots/T1") /
            urdf_name);
        candidates.emplace_back(
            std::filesystem::path("/opt/booster/booster_assets/robots/T1") /
            urdf_name);
    }

    for (const auto &candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return candidate;
        }
    }
    return {};
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
}

// TODO(GW): oneline offline
bool VisionNode::Init(
    const std::string &cfg_template_path,
    const std::vector<std::string> &cfg_paths) {
    if (!std::filesystem::exists(cfg_template_path)) {
        std::cerr << "Error: Configuration template file '" << cfg_template_path << "' does not exist." << std::endl;
        return false;
    }

    YAML::Node node = YAML::LoadFile(cfg_template_path);
    // Only camera/calibration sections from optional files may override the
    // complete local template. Ignore other sections even when the robot-side
    // vision.yaml contains a full runtime configuration.
    for (const auto &cfg_path : cfg_paths) {
        if (cfg_path.empty()) {
            continue;
        }
        if (!std::filesystem::exists(cfg_path)) {
            std::cout << "Warning: Configuration override file '"
                      << cfg_path << "' does not exist; skipping."
                      << std::endl;
            continue;
        }
        YAML::Node cfg_node = YAML::LoadFile(cfg_path);
        MergeCameraCalibrationYAML(node, cfg_node);
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

    if (detection_model_path.empty() && node["detection_model"] &&
        node["detection_model"]["model_path"]) {
        detection_model_path = node["detection_model"]["model_path"].as<std::string>();
        std::cout << "detection_model_path fallback from YAML: "
                  << detection_model_path << std::endl;
    }

    // Resolve relative engines next to the selected configuration first. This
    // is important on T2 where /opt/booster is commonly deployed as a complete
    // config+model bundle. Keep the installed package as a compatibility
    // fallback for configs that only contain a model filename.
    const auto resolve_model_path = [&](const std::string &configured_path) {
        if (configured_path.empty() || configured_path.front() == '/') {
            return configured_path;
        }
        const std::filesystem::path relative_path(configured_path);
        const std::filesystem::path package_dir =
            ament_index_cpp::get_package_share_directory("vision");
        std::vector<std::filesystem::path> config_dirs;
        for (auto it = cfg_paths.rbegin(); it != cfg_paths.rend(); ++it) {
            if (!it->empty()) {
                config_dirs.emplace_back(
                    std::filesystem::path(*it).parent_path());
            }
        }
        config_dirs.emplace_back(
            std::filesystem::path(cfg_template_path).parent_path());
        for (const auto &config_dir : config_dirs) {
            if (config_dir.empty()) continue;
            const std::filesystem::path config_candidate = config_dir / relative_path;
            if (std::filesystem::exists(config_candidate)) {
                return config_candidate.string();
            }
        }
        return (package_dir / relative_path).string();
    };

    detection_model_path = resolve_model_path(detection_model_path);
    if (detection_model_path.empty() ||
        !std::filesystem::is_regular_file(detection_model_path)) {
        std::cerr << "detection engine not found: "
                  << detection_model_path << std::endl;
        return false;
    }


    this->get_parameter<std::string>("segmentation_model_path", segmentation_model_path);
    enable_segmentation_ = node["segmentation_model"] &&
        as_or<bool>(node["segmentation_model"]["enabled"], false);
    std::cout << "segmentation_model_path origin: " << segmentation_model_path << std::endl;

    if (segmentation_model_path.empty() && enable_segmentation_ &&
        node["segmentation_model"] && node["segmentation_model"]["model_path"]) {
        segmentation_model_path = node["segmentation_model"]["model_path"].as<std::string>();
        std::cout << "segmentation_model_path fallback from YAML: "
                  << segmentation_model_path << std::endl;
    }

    if (enable_segmentation_) {
        segmentation_model_path = resolve_model_path(segmentation_model_path);
    }
    
    int save_fps = 0;
    this->get_parameter<int>("save_fps", save_fps);
    save_depth_ = save_depth_ && save_data_;
    show_seg_ = show_seg_ && enable_segmentation_;
    std::cout << "offline_mode: " << offline_mode_ << std::endl;
    std::cout << "show_det: " << show_det_ << std::endl;
    std::cout << "show_seg: " << show_seg_ << std::endl;
    std::cout << "save_data: " << save_data_ << std::endl;
    std::cout << "save_depth: " << save_depth_ << std::endl;
    std::cout << "save_fps: " << save_fps << std::endl;
    std::cout << "camera_type: " << camera_type_ << std::endl;
    std::cout << "detection_model_path: " << detection_model_path << std::endl;
    std::cout << "segmentation_model_path: " << segmentation_model_path << std::endl;
    std::cout << "enable_segmentation: " << enable_segmentation_ << std::endl;
    save_every_n_frame_ = std::max(1, save_fps > 0 ? 30 / save_fps : 1);
    std::cout << "save_every_n_frame: " << save_every_n_frame_ << std::endl;

    // read camera param
    if (!node["camera"]) {
        std::cerr << "no camera param found here" << std::endl;
        return false;
    } else {
        if (camera_type_.empty()) {
            camera_type_ = as_or<std::string>(node["camera"]["type"], "");
            std::cout << "camera type not overridden by launch file, using configuration: "
                      << (camera_type_.empty() ? "<unspecified>" : camera_type_)
                      << std::endl;
        }
        yaml_intr_ = Intrinsics(node["camera"]["intrin"]);
        intr_ = yaml_intr_;
        intrinsics_wait_timeout_sec_ = std::max(
            0.0, as_or<double>(node["camera"]["intrin_wait_timeout_sec"],
                               intrinsics_wait_timeout_sec_));
        p_eye2head_ = as_or<Pose>(node["camera"]["extrin"], Pose());
        const std::string extrin_direction = as_or<std::string>(
            node["camera"]["extrin_direction"], "camera_to_head");
        if (extrin_direction == "head_to_camera") {
            p_eye2head_ = p_eye2head_.inverse();
        } else if (extrin_direction != "camera_to_head") {
            throw std::invalid_argument(
                "camera.extrin_direction must be camera_to_head or head_to_camera");
        }

        float pitch_comp = as_or<float>(node["camera"]["pitch_compensation"], 0.0);
        float yaw_comp = as_or<float>(node["camera"]["yaw_compensation"], 0.0);
        float z_comp = as_or<float>(node["camera"]["z_compensation"], 0.0);
        ball_x_compensation_ = as_or<float>(
            node["camera"]["ball_x_compensation"], 0.0);
        ball_y_compensation_ = as_or<float>(
            node["camera"]["ball_y_compensation"], 0.0);

        p_headprime2head_ = Pose(0, 0, z_comp, 0, pitch_comp * M_PI / 180, yaw_comp * M_PI / 180);
    }

    if (!InitAutoCalibrationPoseAdapter(node)) {
        RCLCPP_ERROR(
            get_logger(),
            "Automatic calibration is enabled, but the robot/camera pose adapter could not be initialized.");
        return false;
    }

    depth_scale_16u_ = std::max(
        1e-9, as_or<double>(node["camera"]["depth_scale_16u"], 0.001));
    depth_scale_32f_ = std::max(
        1e-9, as_or<double>(node["camera"]["depth_scale_32f"], 0.001));
    std::cout << "depth scales: 16U=" << depth_scale_16u_
              << " 32F=" << depth_scale_32f_ << std::endl;

    // init detector
    if (!node["detection_model"]) {
        std::cerr << "no detection model param here" << std::endl;
        return false;
    } else {
        detector_ = YoloV8Detector::CreateYoloV8Detector(node["detection_model"], detection_model_path);
        if (!detector_) {
            std::cerr << "failed to initialize detection model" << std::endl;
            return false;
        }
        classnames_ = node["detection_model"]["classnames"].as<std::vector<std::string>>();
        if (classnames_ != YoloV8Detector::kClassLabels) {
            std::cerr << "detection_model.classnames must match the T2 engine "
                      << "class order" << std::endl;
            return false;
        }
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

    if (enable_segmentation_ && node["segmentation_model"]) {
        segmentor_ = YoloV8Segmentor::CreateYoloV8Segmentor(node["segmentation_model"], segmentation_model_path);
        if (!segmentor_) {
            enable_segmentation_ = false;
            show_seg_ = false;
            std::cerr << "failed to initialize segmentation model; segmentation disabled" << std::endl;
        }
    } else {
        std::cout << "segmentation model disabled by configuration" << std::endl;
    }

    // add detector_ warmup

    if (node["head_pose"]) {
        head_pose_topic_ = as_or<std::string>(
            node["head_pose"]["topic"], head_pose_topic_);
        raw_head_pose_fallback_topic_ = as_or<std::string>(
            node["head_pose"]["raw_fallback_topic"],
            raw_head_pose_fallback_topic_);
        pose_max_sync_diff_ms_ = std::max(
            0.0, as_or<double>(node["head_pose"]["max_sync_diff_ms"],
                               pose_max_sync_diff_ms_));
        pose_interpolation_max_gap_ms_ = std::max(
            0.0, as_or<double>(node["head_pose"]["max_interpolation_gap_ms"],
                               pose_interpolation_max_gap_ms_));
        pose_sync_wait_timeout_ms_ = std::max(
            0.0, as_or<double>(node["head_pose"]["sync_wait_timeout_ms"],
                               pose_sync_wait_timeout_ms_));
    }
    std::cout << "head pose: topic=" << head_pose_topic_
              << " max_sync_diff_ms=" << pose_max_sync_diff_ms_
              << " interpolation_max_gap_ms=" << pose_interpolation_max_gap_ms_
              << " sync_wait_timeout_ms=" << pose_sync_wait_timeout_ms_
              << std::endl;

    if (node["depth_sync"]) {
        depth_max_sync_diff_ms_ = std::max(
            0.0, as_or<double>(node["depth_sync"]["max_sync_diff_ms"],
                               depth_max_sync_diff_ms_));
        depth_sync_wait_timeout_ms_ = std::max(
            0.0, as_or<double>(node["depth_sync"]["sync_wait_timeout_ms"],
                               depth_sync_wait_timeout_ms_));
    }
    std::cout << "depth sync: max_sync_diff_ms="
              << depth_max_sync_diff_ms_
              << " sync_wait_timeout_ms=" << depth_sync_wait_timeout_ms_
              << std::endl;

    // init data_syncer
    use_depth_ = as_or<bool>(node["use_depth"], false);
    data_syncer_ = std::make_shared<DataSyncer>(
        use_depth_, pose_interpolation_max_gap_ms_, pose_sync_wait_timeout_ms_,
        depth_sync_wait_timeout_ms_);
    bool save_data_nonstationary = as_or<bool>(node["misc"]["save_data_nonstationary"], true);
    std::string log_root = std::string(std::getenv("HOME")) + "/Workspace/vision_log/" + getTimeString();
    data_logger_ = save_data_ ? std::make_shared<DataLogger>(log_root, save_data_nonstationary) : nullptr;
    if (data_logger_) {
        data_logger_->LogYAML(node, "vision_local.yaml");
    }
    if (enable_segmentation_) {
        seg_data_syncer_ = std::make_shared<DataSyncer>(
            false, pose_interpolation_max_gap_ms_, pose_sync_wait_timeout_ms_);
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

    intrinsics_wait_start_ = std::chrono::steady_clock::now();
    if (auto_calibrate_ || offline_mode_) {
        intrinsics_source_ = auto_calibrate_
            ? IntrinsicsSource::kCalibratedYaml
            : IntrinsicsSource::kYamlFallback;
        RCLCPP_INFO(
            get_logger(),
            "[vision][intrinsics] %s: using vision.yaml intrinsics "
            "fx=%.6f fy=%.6f cx=%.6f cy=%.6f",
            auto_calibrate_ ? "automatic calibration" : "offline mode",
            yaml_intr_.fx, yaml_intr_.fy, yaml_intr_.cx, yaml_intr_.cy);
    } else {
        intrinsics_fallback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { ActivateYamlIntrinsicsIfTimedOut(); });
        RCLCPP_INFO(
            get_logger(),
            "[vision][intrinsics] waiting up to %.3f s for runtime intrinsics from "
            "/boostercamera/head/rgb/camera_info",
            intrinsics_wait_timeout_sec_);
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
    // Keep one bounded queue per stream. Creating the same image_transport
    // subscriber twice briefly unsubscribes/resubscribes and can drop the
    // first synchronized frame on startup.
    if (camera_type_.find("compressed") != std::string::npos) {
        color_sub_ = it_->subscribe(color_topic, 2, &VisionNode::ColorCallback, this, &hints, sub_opt_1);
    } else {
        color_sub_ = it_->subscribe(color_topic, 2, &VisionNode::ColorCallback, this, nullptr, sub_opt_1);
    } 
    if (use_depth_) {
        depth_sub_ = it_->subscribe(depth_topic, 2, &VisionNode::DepthCallback, this, nullptr, sub_opt_3);
    }

    if (!offline_mode_ && !auto_calibrate_) {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/boostercamera/head/rgb/camera_info", 10,
            std::bind(&VisionNode::CameraInfoCallback, this, std::placeholders::_1),
            sub_opt_1);
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

    if (enable_segmentation_ && segmentor_) {
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
        // T2 publishes head pose with the sensor-data (best-effort) profile.
        // A reliable subscriber is incompatible with that publisher and would
        // leave the depth/pose synchronizer permanently empty.
        const auto pose_qos = rclcpp::SensorDataQoS().keep_last(100);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            head_pose_topic_, pose_qos,
            std::bind(&VisionNode::PoseStampedCallback, this, std::placeholders::_1),
            sub_opt_4);
        RCLCPP_INFO(
            get_logger(),
            "[vision][head pose subscription] topic=%s type=PoseStamped "
            "qos=best_effort/volatile depth=100",
            head_pose_topic_.c_str());
        if (!auto_calibrate_) {
            calParam_sub_ = this->create_subscription<vision_interface::msg::CalParam>(
                "/booster_vision/cal_param", 10,
                std::bind(&VisionNode::CalParamCallback, this, std::placeholders::_1));
        }
        pose_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/booster_vision/t_head2base", rclcpp::QoS(10));
    }

    vision_heartbeat_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        [this]() {
            const int64_t now_ms = SteadyNowMs();
            const int64_t last_received_ms =
                last_color_received_steady_ms_.load(std::memory_order_relaxed);
            const int64_t last_completed_ms =
                last_color_completed_steady_ms_.load(std::memory_order_relaxed);
            const int64_t last_pose_ms =
                last_head_pose_received_steady_ms_.load(std::memory_order_relaxed);
            const double received_age_sec = last_received_ms > 0
                ? static_cast<double>(now_ms - last_received_ms) * 1e-3
                : -1.0;
            const double completed_age_sec = last_completed_ms > 0
                ? static_cast<double>(now_ms - last_completed_ms) * 1e-3
                : -1.0;
            const double pose_age_sec = last_pose_ms > 0
                ? static_cast<double>(now_ms - last_pose_ms) * 1e-3
                : -1.0;
            const uint64_t received =
                color_frames_received_.load(std::memory_order_relaxed);
            const uint64_t processing_started =
                color_frames_processing_started_.load(
                    std::memory_order_relaxed);
            const uint64_t completed =
                color_frames_completed_.load(std::memory_order_relaxed);
            const uint64_t in_flight = processing_started >= completed
                ? processing_started - completed
                : 0;
            const ProcessingStage stage =
                processing_stage_.load(std::memory_order_relaxed);
            const char *stage_name = "unknown";
            switch (stage) {
            case ProcessingStage::kIdle:
                stage_name = "idle";
                break;
            case ProcessingStage::kWaitingForIntrinsics:
                stage_name = "waiting_intrinsics";
                break;
            case ProcessingStage::kDecodingColor:
                stage_name = "decoding_color";
                break;
            case ProcessingStage::kSynchronizing:
                stage_name = "synchronizing";
                break;
            case ProcessingStage::kInference:
                stage_name = "inference";
                break;
            case ProcessingStage::kPostProcessing:
                stage_name = "post_processing";
                break;
            }
            const auto sync_state_name = [](SyncState state) {
                switch (state) {
                    case SyncState::kSynchronized:
                        return "ok";
                    case SyncState::kStale:
                        return "stale";
                    case SyncState::kDisabled:
                        return "disabled";
                    case SyncState::kUnavailable:
                    default:
                        return "missing";
                }
            };
            RCLCPP_INFO(
                get_logger(),
                "[vision][heartbeat] color_received=%llu "
                "processing_started=%llu completed=%llu in_flight=%llu "
                "stage=%s depth_received=%llu head_pose_received=%llu "
                "last_color_age=%.2fs last_completed_age=%.2fs "
                "last_head_pose_age=%.2fs pipeline_ms=%lld "
                "pose_sync=%s(%.1fms) depth_sync=%s(%.1fms)",
                static_cast<unsigned long long>(received),
                static_cast<unsigned long long>(processing_started),
                static_cast<unsigned long long>(completed),
                static_cast<unsigned long long>(in_flight),
                stage_name,
                static_cast<unsigned long long>(
                    depth_frames_received_.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    head_pose_frames_received_.load(std::memory_order_relaxed)),
                received_age_sec, completed_age_sec, pose_age_sec,
                static_cast<long long>(
                    last_pipeline_duration_ms_.load(std::memory_order_relaxed)),
                sync_state_name(
                    last_pose_sync_state_.load(std::memory_order_relaxed)),
                last_pose_sync_diff_ms_.load(std::memory_order_relaxed),
                sync_state_name(
                    last_depth_sync_state_.load(std::memory_order_relaxed)),
                last_depth_sync_diff_ms_.load(std::memory_order_relaxed));
        });
    return true;
}

void VisionNode::UpdatePoseEstimatorIntrinsicsLocked() {
    for (auto &entry : pose_estimator_map_) {
        if (entry.second) {
            entry.second->SetIntrinsics(intr_);
        }
    }
}

void VisionNode::ApplyRuntimeIntrinsics(const Intrinsics &intrinsics) {
    if (!std::isfinite(intrinsics.fx) || !std::isfinite(intrinsics.fy) ||
        !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy) ||
        intrinsics.fx <= 0.0f || intrinsics.fy <= 0.0f) {
        return;
    }

    {
        std::unique_lock<std::shared_mutex> lock(intrinsics_mutex_);
        const bool same_intrinsics = intrinsics_source_ == IntrinsicsSource::kTopic &&
            std::abs(intr_.fx - intrinsics.fx) <= 1e-6f &&
            std::abs(intr_.fy - intrinsics.fy) <= 1e-6f &&
            std::abs(intr_.cx - intrinsics.cx) <= 1e-6f &&
            std::abs(intr_.cy - intrinsics.cy) <= 1e-6f &&
            intr_.model == intrinsics.model &&
            intr_.distortion_coeffs == intrinsics.distortion_coeffs;
        if (same_intrinsics) {
            return;
        }
        intr_ = intrinsics;
        UpdatePoseEstimatorIntrinsicsLocked();
        intrinsics_source_ = IntrinsicsSource::kTopic;
    }
    if (intrinsics_fallback_timer_) {
        intrinsics_fallback_timer_->cancel();
    }
    RCLCPP_INFO(
        get_logger(),
        "[vision][intrinsics] using CameraInfo: fx=%.6f fy=%.6f cx=%.6f cy=%.6f",
        intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
}

void VisionNode::ActivateYamlIntrinsicsIfTimedOut() {
    const double elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - intrinsics_wait_start_).count();
    if (elapsed_sec < intrinsics_wait_timeout_sec_) {
        return;
    }

    {
        std::unique_lock<std::shared_mutex> lock(intrinsics_mutex_);
        if (intrinsics_source_ != IntrinsicsSource::kWaiting) {
            return;
        }
        intr_ = yaml_intr_;
        UpdatePoseEstimatorIntrinsicsLocked();
        intrinsics_source_ = IntrinsicsSource::kYamlFallback;
    }
    RCLCPP_WARN(
        get_logger(),
        "[vision][intrinsics] no valid CameraInfo received within %.3f s; "
        "using vision.yaml fallback fx=%.6f fy=%.6f cx=%.6f cy=%.6f",
        intrinsics_wait_timeout_sec_, yaml_intr_.fx, yaml_intr_.fy,
        yaml_intr_.cx, yaml_intr_.cy);
    if (intrinsics_fallback_timer_) {
        intrinsics_fallback_timer_->cancel();
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

void VisionNode::CameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (!msg) {
        return;
    }

    const float fx = static_cast<float>(msg->k[0]);
    const float fy = static_cast<float>(msg->k[4]);
    const float cx = static_cast<float>(msg->k[2]);
    const float cy = static_cast<float>(msg->k[5]);
    if (!std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0.0f || fy <= 0.0f) {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] ignoring invalid CameraInfo "
            "(fx=%.6f fy=%.6f cx=%.6f cy=%.6f)",
            fx, fy, cx, cy);
        return;
    }

    std::vector<float> distortion_coeffs;
    distortion_coeffs.reserve(msg->d.size());
    for (double coefficient : msg->d) {
        if (!std::isfinite(coefficient)) {
            RCLCPP_WARN(
                get_logger(),
                "[vision][intrinsics] ignoring CameraInfo with non-finite distortion");
            return;
        }
        distortion_coeffs.push_back(static_cast<float>(coefficient));
    }

    if (msg->distortion_model.empty() || msg->distortion_model == "none" ||
        distortion_coeffs.empty()) {
        ApplyRuntimeIntrinsics(Intrinsics(fx, fy, cx, cy));
        return;
    }
    if (msg->distortion_model != "plumb_bob" &&
        msg->distortion_model != "rational_polynomial") {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] unsupported CameraInfo distortion model '%s'",
            msg->distortion_model.c_str());
        return;
    }
    if (distortion_coeffs.size() < 5) {
        RCLCPP_WARN(
            get_logger(),
            "[vision][intrinsics] CameraInfo distortion vector has %zu values; need at least 5",
            distortion_coeffs.size());
        return;
    }
    ApplyRuntimeIntrinsics(Intrinsics(
        fx, fy, cx, cy, distortion_coeffs, Intrinsics::kBrownConrady));
}

bool VisionNode::InitAutoCalibrationPoseAdapter(const YAML::Node &node) {
    auto_calibrate_ = as_or<bool>(node["calibration"]["auto_calibrate"], false);
    auto_calibrate_pose_ready_ = false;
    if (!auto_calibrate_) {
        std::cout << "auto_calibrate: false" << std::endl;
        return true;
    }

    // Offline data already contains the pose that should be used by the
    // synchronizer. Do not require booster-cli or a robot URDF in this mode.
    if (offline_mode_) {
        p_headprime2head_ = Pose();
        auto_calibrate_pose_ready_ = true;
        std::cout << "auto_calibrate: true (offline transform input)" << std::endl;
        return true;
    }

    std::string robot_info =
        RunCommandCaptureStdout("LD_LIBRARY_PATH='' booster-cli robot_info");
    auto_calibrate_robot_model_ = ExtractJsonStringField(robot_info, "Model");
    if (auto_calibrate_robot_model_.empty()) {
        robot_info = RunCommandCaptureStdout("booster-cli robot_info");
        auto_calibrate_robot_model_ = ExtractJsonStringField(robot_info, "Model");
    }
    const std::string robot_model_lower = ToLowerAscii(auto_calibrate_robot_model_);

    std::string urdf_name;
    if (robot_model_lower.find("k1") != std::string::npos) {
        urdf_name = "K1_22dof.urdf";
    } else if (robot_model_lower.find("t1") != std::string::npos) {
        urdf_name = "T1_23dof.urdf";
    } else if (robot_model_lower.find("t2") != std::string::npos) {
        // T2 uses the 31-DOF model from booster_assets, installed in the
        // vision config directory with absolute-path fallbacks for robot images.
        urdf_name = "T2_31dof.urdf";
    } else {
        std::cerr
            << "auto_calibrate enabled, but unsupported robot Model from "
               "booster-cli robot_info: "
            << auto_calibrate_robot_model_ << std::endl;
        return false;
    }

    const std::string perception_info = ReadTextFile("/opt/booster/perception_info.yaml");
    const std::string perception_info_lower = ToLowerAscii(perception_info);
    // A launch parameter is an explicit override. If it is empty, use the
    // merged YAML value; this also handles YAML files that omit camera.type.
    const std::string configured_camera_type = ToLowerAscii(
        camera_type_.empty()
            ? as_or<std::string>(node["camera"]["type"], "")
            : camera_type_);

    // T2_31dof contains the Booster stereo RGB link used by the T2 calibration
    // file. Select it from the robot model directly; T2's perception file may
    // be absent on development images and its unified topic name is generic.
    if (robot_model_lower.find("t2") != std::string::npos) {
        auto_calibrate_camera_link_ = "head_booster_stereo_rgb_link";
    // Prefer the robot image's perception description for K1/T1. The unified
    // camera topics do not identify the physical camera, so camera.type is
    // only a fallback.
    } else if (perception_info_lower.find("realsense") != std::string::npos) {
        auto_calibrate_camera_link_ = "head_realsense_rgb_link";
    } else if (perception_info_lower.find("boostermipi") != std::string::npos ||
               perception_info_lower.find("booster_mipi") != std::string::npos ||
               perception_info_lower.find("d-robotics") != std::string::npos ||
               perception_info_lower.find("drobotics") != std::string::npos) {
        auto_calibrate_camera_link_ = "head_booster_stereo_rgb_link";
    } else if (configured_camera_type.find("realsense") != std::string::npos) {
        auto_calibrate_camera_link_ = "head_realsense_rgb_link";
    } else if (configured_camera_type.find("boostermipi") != std::string::npos ||
               configured_camera_type.find("booster_mipi") != std::string::npos ||
               configured_camera_type.find("d-robotics") != std::string::npos ||
               configured_camera_type.find("drobotics") != std::string::npos) {
        auto_calibrate_camera_link_ = "head_booster_stereo_rgb_link";
    } else {
        std::cerr
            << "auto_calibrate enabled, but camera type is missing or unsupported in "
               "/opt/booster/perception_info.yaml and camera.type."
            << std::endl;
        return false;
    }

    if (robot_model_lower.find("t1") != std::string::npos &&
        auto_calibrate_camera_link_ == "head_booster_stereo_rgb_link") {
        std::cerr << "auto_calibrate enabled, but T1 only supports "
                     "head_realsense_rgb_link."
                  << std::endl;
        return false;
    }

    const std::filesystem::path urdf_path =
        ResolveAutoCalibrationUrdfPath(robot_model_lower, urdf_name);
    if (urdf_path.empty()) {
        std::cerr << "auto_calibrate enabled, but URDF '" << urdf_name
                  << "' could not be found. Checked the vision package, local "
                     "config directory, and booster_assets paths."
                  << std::endl;
        return false;
    }

    if (!LoadFixedJointChildToParentPose(
            urdf_path, auto_calibrate_camera_link_, &p_camera_link2pitchlink_)) {
        return false;
    }

    // In auto-calibration format camera.extrin is the residual hand-eye
    // transform. The nominal camera-link pose comes from the URDF above.
    p_headprime2head_ = Pose();
    auto_calibrate_pose_ready_ = true;
    std::cout << "auto_calibrate: true"
              << ", robot_model: " << auto_calibrate_robot_model_
              << ", urdf: " << urdf_path
              << ", camera_link: " << auto_calibrate_camera_link_
              << std::endl;
    std::cout << "auto_calibrate headpoint2pitchlink: "
              << p_headpoint2pitchlink_ << std::endl;
    std::cout << "auto_calibrate camera_link2pitchlink: "
              << p_camera_link2pitchlink_ << std::endl;
    return true;
}

Pose VisionNode::AdaptHeadPoseForAutoCalibration(
    const Pose &headpoint2base) const {
    if (!auto_calibrate_pose_ready_) {
        return headpoint2base;
    }
    return headpoint2base * p_headpoint2pitchlink_.inverse() *
        p_camera_link2pitchlink_;
}

void VisionNode::ProcessData(SyncedDataBlock &synced_data, vision_interface::msg::Detections &detection_msg) {
    std::shared_lock<std::shared_mutex> intrinsics_lock(intrinsics_mutex_);
    double timestamp = synced_data.color_data.timestamp;
    const bool depth_available = use_depth_ &&
        !synced_data.depth_data.data.empty() &&
        std::isfinite(synced_data.depth_data.timestamp) &&
        synced_data.depth_data.timestamp > 0.0;
    const double depth_time_diff = depth_available
        ? std::abs(timestamp - synced_data.depth_data.timestamp) * 1000.0
        : std::numeric_limits<double>::infinity();
    const bool usable_depth = depth_available &&
        depth_time_diff <= depth_max_sync_diff_ms_;
    if (!usable_depth) {
        // Do not let DataLogger persist a stale frame that was rejected for
        // this color image; an empty depth block makes the fallback explicit.
        synced_data.depth_data = DepthDataBlock();
    }
    const double pose_time_diff = synced_data.pose_valid
        ? synced_data.pose_data.nearest_source_diff * 1000.0
        : 0.0;
    const bool usable_pose = synced_data.pose_valid &&
        pose_time_diff <= pose_max_sync_diff_ms_;
    last_pose_sync_state_.store(
        !synced_data.pose_valid ? SyncState::kUnavailable
                                : (usable_pose ? SyncState::kSynchronized
                                               : SyncState::kStale),
        std::memory_order_relaxed);
    last_pose_sync_diff_ms_.store(
        synced_data.pose_valid ? pose_time_diff : -1.0,
        std::memory_order_relaxed);
    last_depth_sync_state_.store(
        !use_depth_ ? SyncState::kDisabled
                    : (!depth_available ? SyncState::kUnavailable
                                        : (usable_depth
                                               ? SyncState::kSynchronized
                                               : SyncState::kStale)),
        std::memory_order_relaxed);
    last_depth_sync_diff_ms_.store(
        depth_available ? depth_time_diff : -1.0,
        std::memory_order_relaxed);
    if (use_depth_ && !depth_available) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "No valid depth frame is available; continuing with color-only detection");
    } else if (use_depth_ && !usable_depth) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring unsynchronized depth for color frame (delta=%.1f ms)",
            depth_time_diff);
    }
    if (!synced_data.pose_valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "No valid head pose is available; continuing without 3D projections");
    } else if (!usable_pose) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring unsynchronized head pose for color frame (delta=%.1f ms)",
            pose_time_diff);
    }
    cv::Mat color = synced_data.color_data.data;
    cv::Mat depth = usable_depth
        ? synced_data.depth_data.data
        : cv::Mat();

    cv::Mat depth_float;
    if (!depth.empty() && depth.depth() == CV_16U) {
        depth.convertTo(depth_float, CV_32F, depth_scale_16u_, 0);
    } else if (!depth.empty() && depth.depth() == CV_32F) {
        depth.convertTo(depth_float, CV_32F, depth_scale_32f_, 0);
    }

    Pose p_head2base;
    Pose p_eye2base;
    if (usable_pose) {
        Pose camera_compensation;
        {
            std::lock_guard<std::mutex> lock(camera_extrinsics_mutex_);
            camera_compensation = p_headprime2head_;
        }
        p_head2base = synced_data.pose_data.data;
        p_eye2base = p_head2base * camera_compensation * p_eye2head_;
    }

    // inference
    processing_stage_.store(
        ProcessingStage::kInference, std::memory_order_relaxed);
    auto detections = detector_->Inference(color);
    processing_stage_.store(
        ProcessingStage::kPostProcessing, std::memory_order_relaxed);
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Detector produced %zu raw objects", detections.size());

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
                if (detection.class_id < 0 ||
                    detection.class_id >= static_cast<int>(classnames_.size())) {
                    continue;
                }
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
                const bool validClassId = detection.class_id >= 0 &&
                    detection.class_id < static_cast<int>(classnames_.size());
                if (validClassId && classnames_[detection.class_id] == "Ball") {
                    ball_detections.push_back(detection);
                } else if (validClassId) {
                    filtered_detections.push_back(detection);
                }
            }

            if (ball_detections.size() > 1) {
                RCLCPP_DEBUG_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Multiple ball detections found; keeping the highest confidence result");
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
    std::vector<cv::Point2f> detection_xy_for_display;
    if (show_det_) {
        detections_for_display.reserve(filtered_detections.size());
        detection_xy_for_display.reserve(filtered_detections.size());
    }

    auto clamp01 = [](float value) {
        return std::max(0.0f, std::min(1.0f, value));
    };
    auto estimate_robot_fallen = [&](const booster_vision::DetectionRes &detection,
                                     const vision_interface::msg::DetectedObject &detection_obj) {
        const float bboxWidth = static_cast<float>(std::max(1, detection.bbox.width));
        const float bboxHeight = static_cast<float>(std::max(1, detection.bbox.height));
        const float aspect = bboxWidth / bboxHeight;
        const float relativeHeight = bboxHeight /
            static_cast<float>(std::max(1, color.rows));

        float range = 0.0f;
        if (detection_obj.position_projection.size() >= 2) {
            range = std::hypot(
                detection_obj.position_projection[0],
                detection_obj.position_projection[1]);
        }

        float projectedHeight = 0.0f;
        if (range > 0.1f && intr_.fy > 1.0f) {
            projectedHeight = range * bboxHeight / intr_.fy;
        }

        float depthHeight = 0.0f;
        if (usable_pose && usable_depth && !depth_float.empty() &&
            depth_float.type() == CV_32FC1 &&
            intr_.fx > 1.0f && intr_.fy > 1.0f) {
            std::vector<float> heights;
            const cv::Mat eyeToBase = p_eye2base.toCVMat();
            const int step = std::max(
                2,
                std::min(detection.bbox.width, detection.bbox.height) / 8);
            const float scaleX = static_cast<float>(depth_float.cols) /
                static_cast<float>(std::max(1, color.cols));
            const float scaleY = static_cast<float>(depth_float.rows) /
                static_cast<float>(std::max(1, color.rows));
            const float depthFx = static_cast<float>(intr_.fx) * scaleX;
            const float depthFy = static_cast<float>(intr_.fy) * scaleY;
            const float depthCx = static_cast<float>(intr_.cx) * scaleX;
            const float depthCy = static_cast<float>(intr_.cy) * scaleY;

            for (int y = detection.bbox.y;
                 y < detection.bbox.y + detection.bbox.height;
                 y += step) {
                const int depthY = static_cast<int>(std::round(y * scaleY));
                if (depthY < 0 || depthY >= depth_float.rows) continue;

                for (int x = detection.bbox.x;
                     x < detection.bbox.x + detection.bbox.width;
                     x += step) {
                    const int depthX = static_cast<int>(std::round(x * scaleX));
                    if (depthX < 0 || depthX >= depth_float.cols) continue;

                    float depth = depth_float.at<float>(depthY, depthX);
                    if (!std::isfinite(depth) || depth <= 0.05f) continue;
                    if (depth > 20.0f) depth *= 0.001f;
                    if (depth <= 0.05f || depth > 8.0f) continue;

                    const float cameraX =
                        (static_cast<float>(depthX) - depthCx) * depth / depthFx;
                    const float cameraY =
                        (static_cast<float>(depthY) - depthCy) * depth / depthFy;
                    const float baseZ =
                        eyeToBase.at<float>(2, 0) * cameraX +
                        eyeToBase.at<float>(2, 1) * cameraY +
                        eyeToBase.at<float>(2, 2) * depth +
                        eyeToBase.at<float>(2, 3);
                    if (std::isfinite(baseZ) && baseZ > -0.2f && baseZ < 2.0f) {
                        heights.push_back(baseZ);
                    }
                }
            }

            if (heights.size() >= 12) {
                const size_t lowIndex = heights.size() / 10;
                const size_t highIndex = heights.size() * 9 / 10;
                std::nth_element(
                    heights.begin(), heights.begin() + lowIndex, heights.end());
                const float lowHeight = heights[lowIndex];
                std::nth_element(
                    heights.begin(), heights.begin() + highIndex, heights.end());
                depthHeight = std::max(0.0f, heights[highIndex] - lowHeight);
            }
        }

        const float aspectScore = clamp01((aspect - 0.85f) / 0.75f);
        const float projectedHeightScore = projectedHeight > 0.0f
            ? clamp01((0.72f - projectedHeight) / 0.35f)
            : 0.0f;
        const float depthHeightScore = depthHeight > 0.0f
            ? clamp01((0.55f - depthHeight) / 0.30f)
            : 0.0f;
        const float imageHeightScore = clamp01((0.32f - relativeHeight) / 0.18f);
        const float confidence = clamp01(
            0.35f * aspectScore +
            0.30f * std::max(projectedHeightScore, depthHeightScore) +
            0.20f * imageHeightScore +
            0.15f * clamp01(detection.confidence));
        const bool fallen = confidence >= 0.65f &&
            (aspect > 1.05f ||
             projectedHeightScore > 0.5f ||
             depthHeightScore > 0.5f);
        return std::pair<bool, float>(fallen, confidence);
    };

    // Preserve the visible ground footprint of robots without depending on
    // depth.  The bottom corners are more useful for collision geometry than
    // the bbox center alone.
    auto project_ground_point = [&](const cv::Point2f &pixel,
                                    std::vector<float> &projection) {
        if (!usable_pose) return false;
        const cv::Point3f point = CalculatePositionByIntersection(
            p_eye2base, pixel, intr_);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || point.x <= 0.0f) {
            return false;
        }
        projection = {point.x, point.y, point.z};
        return true;
    };

    for (auto &detection : filtered_detections) {
        vision_interface::msg::DetectedObject detection_obj;

        if (detection.class_id < 0 ||
            detection.class_id >= static_cast<int>(detector_->kClassLabels.size()) ||
            detection.class_id >= static_cast<int>(classnames_.size())) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Ignoring detector class id %d outside configured label range",
                detection.class_id);
            continue;
        }
        detection.class_name = detector_->kClassLabels[detection.class_id];

        if (usable_pose) {
            auto pose_estimator = get_estimator(detection.class_name);
            Pose pose_obj_by_color =
                pose_estimator->EstimateByColor(p_eye2base, detection, color);
            Pose pose_obj_by_depth = pose_obj_by_color;
            const bool can_use_depth = usable_depth && pose_estimator->use_depth_ &&
                !depth_float.empty();
            if (can_use_depth) {
                pose_obj_by_depth = pose_estimator->EstimateByDepth(
                    p_eye2base, detection, color, depth_float);
            }

            if (can_use_depth && detection.class_name == "Ball" &&
                pose_obj_by_depth == Pose()) {
                RCLCPP_DEBUG_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Ball detection rejected because depth estimation was invalid");
                continue;
            }
            detection_obj.position_projection = pose_obj_by_color.getTranslationVec();
            detection_obj.position = pose_obj_by_depth.getTranslationVec();

            if ((detection.class_name == "Opponent" ||
                 detection.class_name == "Person") &&
                color.cols > 0 && color.rows > 0) {
                std::vector<float> leftProjection;
                std::vector<float> rightProjection;
                const cv::Point2f leftPixel(
                    static_cast<float>(std::clamp(detection.bbox.x, 0, color.cols - 1)),
                    static_cast<float>(std::clamp(
                        detection.bbox.y + detection.bbox.height,
                        0, color.rows - 1)));
                const cv::Point2f rightPixel(
                    static_cast<float>(std::clamp(
                        detection.bbox.x + detection.bbox.width,
                        0, color.cols - 1)),
                    static_cast<float>(std::clamp(
                        detection.bbox.y + detection.bbox.height,
                        0, color.rows - 1)));
                if (project_ground_point(leftPixel, leftProjection) &&
                    project_ground_point(rightPixel, rightProjection)) {
                    detection_obj.position_projection_left = leftProjection;
                    detection_obj.position_projection_right = rightProjection;
                }
            }

            if (detection.class_name == "Ball") {
                if (detection_obj.position_projection.size() >= 2) {
                    detection_obj.position_projection[0] += ball_x_compensation_;
                    detection_obj.position_projection[1] += ball_y_compensation_;
                }
                if (detection_obj.position.size() >= 2) {
                    detection_obj.position[0] += ball_x_compensation_;
                    detection_obj.position[1] += ball_y_compensation_;
                }
            }

            auto xyz = p_head2base.getTranslationVec();
            auto rpy = p_head2base.getEulerAnglesVec();
            detection_obj.received_pos = {
                xyz[0], xyz[1], xyz[2],
                static_cast<float>(rpy[0] / CV_PI * 180),
                static_cast<float>(rpy[1] / CV_PI * 180),
                static_cast<float>(rpy[2] / CV_PI * 180)};
        }

        detection_obj.confidence = detection.confidence * 100;
        detection_obj.xmin = detection.bbox.x;
        detection_obj.ymin = detection.bbox.y;
        detection_obj.xmax = detection.bbox.x + detection.bbox.width;
        detection_obj.ymax = detection.bbox.y + detection.bbox.height;
        detection_obj.label = detection.class_name;

        if ((color_classifier_ != nullptr) && (detection.class_name == "Opponent")) {
            const cv::Rect imageRect(0, 0, color.cols, color.rows);
            const cv::Rect clippedBox = detection.bbox & imageRect;
            if (clippedBox.width > 0 && clippedBox.height > 0) {
                detection_obj.color = color_classifier_->Classify(
                    color(clippedBox));
            }
        }

        if (detection.class_name == "Opponent" ||
            detection.class_name == "Person") {
            const auto fallenEstimate =
                estimate_robot_fallen(detection, detection_obj);
            detection_obj.fallen = fallenEstimate.first;
            detection_obj.fallen_confidence = fallenEstimate.second;
        }

        // publish detection
        detection_msg.detected_objects.push_back(detection_obj);
        if (show_det_) {
            detections_for_display.push_back(detection);
            if (detection_obj.position_projection.size() >= 2) {
                detection_xy_for_display.emplace_back(
                    detection_obj.position_projection[0],
                    detection_obj.position_projection[1]);
            } else {
                const float nan_value = std::numeric_limits<float>::quiet_NaN();
                detection_xy_for_display.emplace_back(nan_value, nan_value);
            }
        }
    }

    // compute corner points positision
    std::vector<cv::Point2f> corner_uvs = {cv::Point2f(0, 0), cv::Point2f(color.cols - 1, 0),
                                           cv::Point2f(color.cols - 1, color.rows - 1), cv::Point2f(0, color.rows - 1),
                                           cv::Point2f(color.cols / 2.0, color.rows / 2.0)};
    if (usable_pose) {
        for (auto &uv : corner_uvs) {
            auto corner_pos = CalculatePositionByIntersection(p_eye2base, uv, intr_);
            detection_msg.corner_pos.push_back(corner_pos.x);
            detection_msg.corner_pos.push_back(corner_pos.y);
        }
    }

    // sync-radar measurements

    // publish msg
    detection_pub_->publish(detection_msg);

    vision_interface::msg::Ball ball_msg;
    ball_msg.header = detection_msg.header;
    ball_msg.confidence = 0;
    for (auto &detection : filtered_detections) {
        if (!usable_pose) {
            break;
        }
        if (detection.class_name == "Ball" && detection.confidence > ball_msg.confidence) {
            auto pose_estimator = get_estimator(detection.class_name);
            Pose pose_obj_by_color = pose_estimator->EstimateByColor(p_eye2base, detection, color);
            auto value = pose_obj_by_color.getTranslationVec();
            if (value[0] < -2 || value[0] > 10 || value[1] < -5 || value[1] > 5) {
                continue;
            }
            ball_msg.x = value[0] + ball_x_compensation_;
            ball_msg.y = value[1] + ball_y_compensation_;
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
        const size_t draw_count = std::min(
            detections_for_display.size(), detection_xy_for_display.size());
        for (size_t i = 0; i < draw_count; ++i) {
            const auto &xy = detection_xy_for_display[i];
            if (!std::isfinite(xy.x) || !std::isfinite(xy.y)) {
                continue;
            }

            const auto &bbox = detections_for_display[i].bbox;
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "x:" << xy.x << " y:" << xy.y;
            const std::string text = oss.str();

            int baseline = 0;
            const cv::Size text_size = cv::getTextSize(
                text, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
            int text_x = std::max(0, bbox.x);
            int text_y = bbox.y + bbox.height + text_size.height + 2;
            if (text_y >= img_out.rows) {
                text_y = std::max(text_size.height + 2, bbox.y - 4);
            }
            if (text_x + text_size.width >= img_out.cols) {
                text_x = std::max(0, img_out.cols - text_size.width - 1);
            }

            cv::putText(img_out, text, cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            cv::putText(img_out, text, cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
        }
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
    if (!msg) {
        RCLCPP_WARN(get_logger(), "Received null color image message");
        return;
    }
    color_frames_received_.fetch_add(1, std::memory_order_relaxed);
    last_color_received_steady_ms_.store(
        SteadyNowMs(), std::memory_order_relaxed);
    processing_stage_.store(
        ProcessingStage::kWaitingForIntrinsics, std::memory_order_relaxed);
    if (!EnsureIntrinsicsReady()) {
        processing_stage_.store(
            ProcessingStage::kIdle, std::memory_order_relaxed);
        return;
    }

    // cv_bridge::CvImagePtr cv_ptr;
    processing_stage_.store(
        ProcessingStage::kDecodingColor, std::memory_order_relaxed);
    cv::Mat img;
    try {
        // cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        img = toCVMat(*msg);
    } catch (std::exception &e) {
        RCLCPP_WARN(get_logger(), "Unable to decode color image: %s", e.what());
        processing_stage_.store(
            ProcessingStage::kIdle, std::memory_order_relaxed);
        return;
    }

    if (camera_type_ == "realsense") {
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    }

    vision_interface::msg::Detections detection_msg;
    detection_msg.header = msg->header;
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    // get synced data
    const int64_t pipeline_started_ms = SteadyNowMs();
    color_frames_processing_started_.fetch_add(1, std::memory_order_relaxed);
    processing_stage_.store(
        ProcessingStage::kSynchronizing, std::memory_order_relaxed);
    SyncedDataBlock synced_data = data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    
    ProcessData(synced_data, detection_msg);
    color_frames_completed_.fetch_add(1, std::memory_order_relaxed);
    const int64_t pipeline_completed_ms = SteadyNowMs();
    last_pipeline_duration_ms_.store(
        pipeline_completed_ms - pipeline_started_ms,
        std::memory_order_relaxed);
    last_color_completed_steady_ms_.store(
        pipeline_completed_ms, std::memory_order_relaxed);
    processing_stage_.store(
        ProcessingStage::kIdle, std::memory_order_relaxed);
}

void VisionNode::ProcessSegmentationData(SyncedDataBlock &synced_data, vision_interface::msg::LineSegments &field_line_segs_msg) {
    std::shared_lock<std::shared_mutex> intrinsics_lock(intrinsics_mutex_);
    cv::Mat color = synced_data.color_data.data;
    const double pose_time_diff = synced_data.pose_valid
        ? synced_data.pose_data.nearest_source_diff * 1000.0
        : 0.0;
    if (!synced_data.pose_valid || pose_time_diff > pose_max_sync_diff_ms_) {
        return;
    }
    Pose p_head2base = synced_data.pose_data.data;
    Pose camera_compensation;
    {
        std::lock_guard<std::mutex> lock(camera_extrinsics_mutex_);
        camera_compensation = p_headprime2head_;
    }
    Pose p_eye2base = p_head2base * camera_compensation * p_eye2head_;

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
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Segmentor produced %zu objects", segmentations.size());

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
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Segmentation frame received but no segmentor is loaded");
        return;
    }
    if (!msg) {
        RCLCPP_WARN(get_logger(), "Received null segmentation image message");
        return;
    }
    if (!EnsureIntrinsicsReady()) {
        return;
    }

    cv::Mat img;
    try {
        // cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        img = toCVMat(*msg).clone();
    } catch (std::exception &e) {
        RCLCPP_WARN(
            get_logger(), "Unable to decode segmentation image: %s", e.what());
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
    if (!msg) {
        RCLCPP_WARN(get_logger(), "Received null depth image message");
        return;
    }
    // cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    try {
        // TODO(SS): check if the image is 16-bit for zed camera
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        img = toCVMat(*msg).clone();
    } catch (std::exception &e) {
        RCLCPP_WARN(get_logger(), "Unable to decode depth image: %s", e.what());
        return;
    }

    if (img.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty depth image");
        return;
    }

    // Check if the image is indeed 16-bit
    if (img.depth() != CV_16U && img.depth() != CV_32F) {
        RCLCPP_WARN(
            get_logger(), "Unsupported depth encoding '%s'",
            msg->encoding.c_str());
        return;
    }

    const double timestamp = msg->header.stamp.sec +
        static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    if (!std::isfinite(timestamp) || timestamp <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring depth image with invalid timestamp %d.%09u",
            msg->header.stamp.sec, msg->header.stamp.nanosec);
        return;
    }
    data_syncer_->AddDepth(DepthDataBlock(img, timestamp));
    depth_frames_received_.fetch_add(1, std::memory_order_relaxed);
    // seg_data_syncer_->AddDepth(DepthDataBlock(img, timestamp));
}

void VisionNode::PoseTFCallBack(
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr msg) {
    if (!msg) {
        return;
    }
    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    data_syncer_->AddPose(PoseDataBlock(Pose(*msg), timestamp));
    if (seg_data_syncer_) {
        seg_data_syncer_->AddPose(PoseDataBlock(Pose(*msg), timestamp));
    }
    head_pose_frames_received_.fetch_add(1, std::memory_order_relaxed);
    last_head_pose_received_steady_ms_.store(
        SteadyNowMs(), std::memory_order_relaxed);
}

void VisionNode::PoseStampedCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    if (!msg) {
        return;
    }

    const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
    const auto &position = msg->pose.position;
    const auto &orientation = msg->pose.orientation;
    const double quaternion_norm_sq =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
    const bool finite_pose =
        std::isfinite(position.x) && std::isfinite(position.y) &&
        std::isfinite(position.z) && std::isfinite(orientation.x) &&
        std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
        std::isfinite(orientation.w) && std::isfinite(quaternion_norm_sq);
    if (stamp_ns <= 0 || !finite_pose || quaternion_norm_sq < 1e-12) {
        bool expected = false;
        if (invalid_head_pose_log_printed_.compare_exchange_strong(expected, true)) {
            RCLCPP_ERROR(
                get_logger(),
                "[vision][head pose] rejected invalid PoseStamped: stamp=%d.%09u "
                "finite=%s quaternion_norm_sq=%.9g",
                msg->header.stamp.sec, msg->header.stamp.nanosec,
                finite_pose ? "true" : "false", quaternion_norm_sq);
        }
        return;
    }

    const double timestamp = static_cast<double>(stamp_ns) * 1e-9;
    const double inv_quaternion_norm = 1.0 / std::sqrt(quaternion_norm_sq);
    Pose pose(
        static_cast<float>(position.x),
        static_cast<float>(position.y),
        static_cast<float>(position.z),
        static_cast<float>(orientation.x * inv_quaternion_norm),
        static_cast<float>(orientation.y * inv_quaternion_norm),
        static_cast<float>(orientation.z * inv_quaternion_norm),
        static_cast<float>(orientation.w * inv_quaternion_norm));
    pose = AdaptHeadPoseForAutoCalibration(pose);
    data_syncer_->AddPose(PoseDataBlock(pose, timestamp));
    if (seg_data_syncer_) {
        seg_data_syncer_->AddPose(PoseDataBlock(pose, timestamp));
    }
    head_pose_frames_received_.fetch_add(1, std::memory_order_relaxed);
    last_head_pose_received_steady_ms_.store(
        SteadyNowMs(), std::memory_order_relaxed);

    bool expected = false;
    if (head_pose_input_log_printed_.compare_exchange_strong(expected, true)) {
        RCLCPP_INFO(
            get_logger(),
            "[vision][head pose] using source timestamp from %s frame_id=%s "
            "stamp=%d.%09u",
            head_pose_topic_.c_str(), msg->header.frame_id.c_str(),
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    if (pose_tf_pub_) {
        auto tf_msg = pose.toRosTFMsg();
        tf_msg.header.stamp = msg->header.stamp;

        pose_tf_pub_->publish(tf_msg);
    }
}

void VisionNode::CalParamCallback(
    const vision_interface::msg::CalParam::ConstSharedPtr msg) {
    if (!msg || auto_calibrate_pose_ready_) {
        return;
    }
    float pitch_comp = msg->pitch_compensation;
    float yaw_comp = msg->yaw_compensation;
    float z_comp = msg->z_compensation;
    std::cout << "calParams: " << pitch_comp << " " << yaw_comp << " " << z_comp << std::endl;
    std::lock_guard<std::mutex> lock(camera_extrinsics_mutex_);
    p_headprime2head_ = Pose(
        0, 0, z_comp, 0,
        pitch_comp * M_PI / 180, yaw_comp * M_PI / 180);
}

} // namespace booster_vision
