#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"
#include "booster_vision/base/data_syncer.hpp"
#include "booster_vision/base/misc_utils.hpp"
#include "booster_vision/img_bridge.h"
#include "booster_vision/calibration/optimizor.hpp"
#include "booster_vision/calibration/calibration.h"
#include "booster_vision/calibration/board_detector.h"
#include "booster_vision/pose_estimator/pose_estimator.h"

namespace booster_vision {

// calibratiion node
class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode(const std::string &node_name);
    ~CalibrationNode() = default;

    void Init(const std::string cfg_path, bool offline_mode = false, std::string calibration_mode = "handeye");
    void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void PoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void RunOfflineCalibrationProcess();
    void RunExtrinsicCalibrationProcess(const SyncedDataBlock &data_block);

private:
    bool is_offline_ = false;
    bool new_log_path_ = true;

    int board_w_ = 0;
    int board_h_ = 0;
    float board_square_size_ = 0.05;
    std::string calibration_mode_ = "handeye";

    std::string input_cfg_path_;
    std::string log_path_;
    std::string camera_type_;

    YAML::Node cfg_node_;

    Intrinsics intr_;
    Pose p_eye2head_;
    std::shared_ptr<rclcpp::Node> nh_;
    SyncedDataBlock data_block_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber color_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    std::shared_ptr<DataSyncer> data_syncer_;
    std::vector<SyncedDataBlock> cali_data_;

    std::shared_ptr<BoardDetector> board_detector_;

    // for display
    cv::Mat board_position_mask_;

    // for offset calibration
    double exclude_distance_;
    bool zero_translation_;
};

CalibrationNode::CalibrationNode(const std::string &node_name) :
    rclcpp::Node(node_name) {
    this->declare_parameter<int>("board_w", 11);
    this->declare_parameter<int>("board_h", 8);
    this->declare_parameter<float>("board_square_size", 0.05);
}

void CalibrationNode::Init(const std::string cfg_path, bool is_offline, std::string calibration_mode) {
    if (!std::filesystem::exists(cfg_path)) {
        std::cerr << "Error: Configuration file '" << cfg_path << "' does not exist." << std::endl;
        return;
    }

    input_cfg_path_ = cfg_path;
    cfg_node_ = YAML::LoadFile(cfg_path);
    if (!cfg_node_["camera"]) {
        std::cerr << "no camera param found here" << std::endl;
        return;
    } else {
        intr_ = Intrinsics(cfg_node_["camera"]["intrin"]);
        p_eye2head_ = as_or<Pose>(cfg_node_["camera"]["extrin"], Pose());
    }
    std::cout << "intrinsics from yaml\n: " << intr_ << std::endl;

    calibration_mode_ = calibration_mode;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_1 = rclcpp::SubscriptionOptions();
    sub_opt_1.callback_group = callback_group_sub_1;
    auto sub_opt_2 = rclcpp::SubscriptionOptions();
    sub_opt_2.callback_group = callback_group_sub_2;

    std::string color_topic;
    std::string intrin_topic;
    if (camera_type_ == "zed") {
        color_topic = "/zed/zed_node/left/image_rect_color";
        intrin_topic = "/zed/zed_node/left/camera_info";
    } else {
        // realsense
        color_topic = "/camera/camera/color/image_raw";
        intrin_topic = "/camera/camera/color/camera_info";
    }

    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    color_sub_ = it_->subscribe(color_topic, 1, &CalibrationNode::ColorCallback, this, nullptr, sub_opt_1);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/head_pose", 10,
        std::bind(&CalibrationNode::PoseCallback, this, std::placeholders::_1), sub_opt_2);
    is_offline_ = is_offline;
    if (!is_offline_) {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            intrin_topic, 10,
            std::bind(&CalibrationNode::CameraInfoCallback, this, std::placeholders::_1));
    }

    this->get_parameter("board_w", board_w_);
    this->get_parameter("board_h", board_h_);
    this->get_parameter("board_square_size", board_square_size_);

    board_detector_ = std::make_shared<BoardDetector>(cv::Size(board_w_, board_h_), board_square_size_, intr_);

    data_syncer_ = std::make_shared<DataSyncer>(false);
    if (is_offline_) {
        std::string data_dir = cfg_path.substr(0, cfg_path.find_last_of("/"));
        data_syncer_->LoadData(data_dir);
        log_path_ = data_dir;
    }
}

void CalibrationNode::RunExtrinsicCalibrationProcess(const SyncedDataBlock &data_block) {
    if (!is_offline_ && new_log_path_) {
        log_path_ = std::string(std::getenv("HOME")) + "/Workspace/calibration_log/handeye/" + getTimeString();
        new_log_path_ = false;
    }
    auto img = data_block.color_data.data;
    double timestamp = data_block.color_data.timestamp;
    double color_pose_timediff = std::abs(data_block.pose_data.timestamp - timestamp) * 1000;
    if (color_pose_timediff > 40) {
        std::cerr << "color and pose data not synced, time diff: " << color_pose_timediff << std::endl;
        return;
    }

    // extract chessboard corners
    bool found = board_detector_->DetectBoard(img);
    std::vector<cv::Point2f> corners = board_detector_->getBoardUVs();

    // draw board history position
    double alpha = 0.25;
    cv::Mat display_img = img.clone();
    if (board_position_mask_.empty()) {
        board_position_mask_ = cv::Mat::zeros(img.size(), img.type());
    }
    cv::addWeighted(board_position_mask_, alpha, display_img, 1 - alpha, 0, display_img);

    // draw boards on image
    cv::drawChessboardCorners(display_img, cv::Size(board_w_, board_h_), corners, found);
    std::string progress_status_text = std::to_string(cali_data_.size()) + "/8 frames collected";
    cv::putText(display_img, progress_status_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    cv::putText(display_img, "press h for help info in terminal", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);

    // draw valid area
    cv::Rect valid_area(img.cols * 0.15, img.rows * 0.15, img.cols * 0.7, img.rows * 0.7);
    cv::rectangle(display_img, valid_area, cv::Scalar(0, 0, 255), 2);
    cv::imshow("chessboard", display_img);

    int wait_time = is_offline_ ? 0 : 10;
    const char key = cv::waitKey(wait_time);
    switch (key) {
    case 's': {
        std::cout << "select current snap short for calibration!" << std::endl;
        // TODO(GW): order check
        if (found) {
            // update board mask
            cv::Mat mask = board_detector_->getBoardMask(img);
            cv::putText(mask, std::to_string(cali_data_.size()), cv::Point(corners[0].x, corners[0].y + 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            cv::bitwise_or(mask, board_position_mask_, board_position_mask_);

            cali_data_.emplace_back(data_block);

            if (!is_offline_) {
                if (!std::filesystem::exists(log_path_)) {
                    std::filesystem::create_directories(log_path_);
                    std::cout << "creating log directory: " << log_path_ << std::endl;
                }
                // save img
                std::string img_name = log_path_ + "/color_" + std::to_string(timestamp) + ".jpg";
                cv::imwrite(img_name, img);
                // save pose
                YAML::Node pose_node;
                pose_node["pose"] = data_block.pose_data.data;
                std::string yaml_name = log_path_ + "/pose_" + std::to_string(timestamp) + ".yaml";
                std::ofstream pose_yaml(yaml_name);
                pose_yaml << pose_node;
                // save vision_local.yaml
                std::string vision_cfg_name = log_path_ + "/vision_local.yaml";
                std::ofstream vision_cfg(vision_cfg_name);
                vision_cfg << cfg_node_;
            }
        }
        break;
    }
    case 'c': {
        if (cali_data_.size() < 8) {
            std::cout << "not enough data for calibration, need at least 8 frames!" << std::endl;
            break;
        }
        // calibrate
        std::cout << "start calibration computation!" << std::endl;
        // prepare data for calibration
        std::vector<std::vector<cv::Point3f>> all_corners_3d_;
        std::vector<std::vector<cv::Point2f>> all_corners_2d_;
        std::vector<Pose> p_head2bases;
        std::vector<Pose> p_board2cameras;

        YAML::Node calib_data_node;
        for (int i = 0; i < cali_data_.size(); i++) {
            cv::Mat img = cali_data_[i].color_data.data.clone();
            if (board_detector_->DetectBoard(img)) {
                all_corners_3d_.push_back(board_detector_->getBoardPoints());
                all_corners_2d_.push_back(board_detector_->getBoardUVsSubpixel());
                p_head2bases.push_back(cali_data_[i].pose_data.data);
                p_board2cameras.push_back(board_detector_->getBoardPose());

                auto corner_3d = all_corners_3d_.back();
                auto corner_2d = all_corners_2d_.back();
                auto p_head2base = p_head2bases.back();
                auto p_board2camera = p_board2cameras.back();
                
                auto calib_res = YAML::Node();
                calib_res["corner_3d"] = YAML::Node();
                for (auto &corner : corner_3d) {
                    YAML::Node corner_node(YAML::NodeType::Sequence);
                    corner_node.push_back(corner.x);
                    corner_node.push_back(corner.y);
                    corner_node.push_back(corner.z);
                    calib_res["corner_3d"].push_back(corner_node);
                }
                calib_res["corner_2d"] = YAML::Node();
                for(auto &corner : corner_2d) {
                    YAML::Node corner_node(YAML::NodeType::Sequence);
                    corner_node.push_back(corner.x);
                    corner_node.push_back(corner.y);
                    calib_res["corner_2d"].push_back(corner_node);
                }
                calib_res["head2base"] = p_head2base;
                calib_res["board2camera"] = p_board2camera;
                calib_data_node[std::to_string(i)] = calib_res;
                
                std::cout << "number " << i << " th board detected!" << std::endl;
                std::cout << "board pose: \n"
                          << p_board2cameras.back() << std::endl;
                std::cout << "head pose: \n"
                          << p_head2bases.back() << std::endl;
            }
        }
        std::string date = getTimeString();
        std::string calib_data_log_yaml = log_path_ + "/calib_data_log.yaml." + date;
        std::ofstream calib_data_yaml(calib_data_log_yaml);
        calib_data_yaml << calib_data_node;


        double reprojection_error = std::numeric_limits<double>::max();
        Pose p_eye2head_best;
        Pose p_board2base_best;

        EyeInHandCalibration(&reprojection_error,
                             &p_eye2head_best, &p_board2base_best,
                             p_board2cameras, p_head2bases,
                             all_corners_3d_, all_corners_2d_, intr_);

        // save res
        std::cout << "old extrin: \n"
                  << cfg_node_["camera"]["extrin"].as<Pose>() << std::endl;
        std::cout << "new extrin: \n"
                  << p_eye2head_best << std::endl;
        std::cout << "new board2base: \n"
                  << p_board2base_best << std::endl;


        YAML::Node new_cfg_node = cfg_node_;
        new_cfg_node["camera"]["extrin"] = p_eye2head_best;

        // clear compensation
        new_cfg_node["camera"]["pitch_compensation"] = 0.0;
        new_cfg_node["camera"]["yaw_compensation"] = 0.0;
        new_cfg_node["camera"]["z_compensation"] = 0.0;

        YAML::Node cali_node = new_cfg_node["calibration"];
        if (!cali_node) {
            cali_node = YAML::Node();
        }
        if (!cali_node["handeye"]) {
            cali_node["handeye"] = YAML::Node();
        }
        cali_node["handeye"]["calibration_time"] = date;
        cali_node["handeye"]["reprojection_error"] = reprojection_error;

        std::string new_cfg_path = log_path_ + "/vision_local.yaml.calbration_res_" + date;
        std::cout << "saving calibration result to " << new_cfg_path << std::endl;
        std::ofstream new_cfg_yaml(new_cfg_path);
        new_cfg_yaml << new_cfg_node;

        if (!is_offline_) {
            // wait input to overwrite config
            std::cout << "overwrite input config with new config? y/n" << std::endl;
            // wait for input
            char key;
            std::cin >> key;
            if (key == 'y') {
                std::ofstream cfg_yaml(input_cfg_path_);
                cfg_yaml << new_cfg_node;
                std::cout << "overwriting input config with new config" << std::endl;

                // backup original config
                std::string backup_cfg_path = log_path_ + "/vision_local.yaml.input_backup_" + date;
                std::cout << "backuping original config to " << backup_cfg_path << std::endl;
                std::ofstream backup_cfg_yaml(backup_cfg_path);
                backup_cfg_yaml << cfg_node_;
            } else {
                std::cout << "not overwrite input config" << std::endl;
            }
        }
        std::cout << "finish extrinsics calibration process!!!" << std::endl;
    }
    case 'r': {
        std::cout << "rest extrinsics calibration process!!!" << std::endl;
        new_log_path_ = true;
        cali_data_.clear();
        board_position_mask_ = cv::Mat();
        break;
    }
    case 'q': {
        // exit
        std::cout << "exit extrinsics calibration process" << std::endl;
        rclcpp::shutdown();
        exit(0);
        break;
    }
    case 'h': {
        std::cout << std::endl
                  << "operation key binding:" << std::endl
                  << "s: save data for calibration" << std::endl
                  << "c: start calibration if data number exceeds 8" << std::endl
                  << "r: restart calibration process" << std::endl
                  << "q: exit" << std::endl;
        break;
    }
    default: {
        break;
    }
    }
}

void CalibrationNode::RunOfflineCalibrationProcess() {
    while (true) {
        if (calibration_mode_ == "handeye") {
            RunExtrinsicCalibrationProcess(data_syncer_->getSyncedDataBlock());
        }
    }
}

void CalibrationNode::ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    // std::cout << "new color received" << std::endl;
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

    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    auto data_block = data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    if (calibration_mode_ == "handeye") {
        RunExtrinsicCalibrationProcess(data_block);
    }
}

void CalibrationNode::PoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
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

void CalibrationNode::CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!msg) {
        std::cerr << "empty camera info message." << std::endl;
        return;
    }

    float fx = msg->k[0];
    float fy = msg->k[4];
    float cx = msg->k[2];
    float cy = msg->k[5];

    std::vector<float> distortion_coeffs(msg->d.begin(), msg->d.end());
    std::cout << "update camera intrinsics" << std::endl;
    intr_ = Intrinsics(fx, fy, cx, cy, distortion_coeffs, Intrinsics::DistortionModel::kBrownConrady);
    cfg_node_["camera"]["intrin"] = intr_;

    // if recevied count execeeds 10, disable this callback
    static int recevied_count = 0;
    recevied_count++;
    if (recevied_count > 10) {
        camera_info_sub_.reset();
        std::cout << "disable camera info callback" << std::endl;
        // update board_detector
        std::cout << "updated camera intrinsics: \n" << intr_ << std::endl;
        board_detector_ = std::make_shared<BoardDetector>(cv::Size(board_w_, board_h_), board_square_size_, intr_);
    }
}

} // namespace booster_vision

const std::string kArguments = "{help h usage ? |         | print this message}"
                               "{@mode          | handeye | calibration mode: handeye or offset}"
                               "{@config_file   | <none>  | config file path}"
                               "{offline of     |         | offline mode}";

int main(int argc, char **argv) {
    cv::CommandLineParser parser(argc, argv, kArguments);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    std::string config_path = parser.get<std::string>("@config_file");
    std::string mode = parser.get<std::string>("@mode");
    bool offline_mode = parser.has("offline");

    if (!parser.check()) {
        parser.printErrors();
        return -1;
    }

    if (mode != "handeye" && mode != "offset") {
        std::cerr << "invalid calibration mode: " << mode << std::endl;
        return -1;
    }

    rclcpp::init(argc, argv);

    std::string node_name = "calibration_node";
    auto node = std::make_shared<booster_vision::CalibrationNode>(node_name);

    std::cout << "offline mode: " << offline_mode << std::endl;

    node->Init(config_path, offline_mode, mode);
    std::cout << "calibration node initialized" << std::endl;

    if (offline_mode) {
        node->RunOfflineCalibrationProcess();
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}
