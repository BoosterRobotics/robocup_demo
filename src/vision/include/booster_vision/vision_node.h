#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "vision_interface/msg/detections.hpp"

#include "vision_interface/msg/line_segments.hpp"
#include "vision_interface/msg/cal_param.hpp"
#include "vision_interface/msg/ball.hpp"

#include <yaml-cpp/yaml.h>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"

#include "booster_vision/color_classifier.hpp"

namespace booster_vision {

class DataLogger;
class DataSyncer;
class PoseEstimator;
class YoloV8Detector;
class YoloV8Segmentor;
class SyncedDataBlock;

class VisionNode : public rclcpp::Node {
public:
    VisionNode(const std::string &node_name);
    ~VisionNode() = default;

    [[nodiscard]] bool Init(
        const std::string &cfg_template_path,
        const std::vector<std::string> &cfg_paths);
    void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void SegmentationCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void PoseTFCallBack(const geometry_msgs::msg::TransformStamped::ConstSharedPtr msg);
    void PoseStampedCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void CalParamCallback(const vision_interface::msg::CalParam::ConstSharedPtr msg);
    void ProcessData(SyncedDataBlock &synced_data, vision_interface::msg::Detections &detections);
    void ProcessSegmentationData(SyncedDataBlock &synced_data, vision_interface::msg::LineSegments &field_line_segs_msg);

private:
    enum class IntrinsicsSource {
        kWaiting,
        kYamlFallback,
        kCalibratedYaml,
        kTopic,
    };

    enum class ProcessingStage {
        kIdle,
        kWaitingForIntrinsics,
        kDecodingColor,
        kSynchronizing,
        kInference,
        kPostProcessing,
    };

    enum class SyncState {
        kUnavailable,
        kSynchronized,
        kStale,
        kDisabled,
    };

    void UpdatePoseEstimatorIntrinsicsLocked();
    void ApplyRuntimeIntrinsics(const Intrinsics &intrinsics);
    void ActivateYamlIntrinsicsIfTimedOut();
    bool EnsureIntrinsicsReady();

    bool use_depth_ = false;
    double depth_scale_16u_ = 0.001;
    double depth_scale_32f_ = 0.001;
    double depth_max_sync_diff_ms_ = 40.0;
    double depth_sync_wait_timeout_ms_ = 12.0;
    bool show_det_ = false;
    bool show_seg_ = false;
    bool save_data_ = false;
    bool save_depth_ = false;
    bool offline_mode_ = false;
    bool enable_segmentation_ = false;
    std::string detection_model_path;
    std::string segmentation_model_path;

    int save_cnt_ = 0;
    int save_every_n_frame_ = 0;

    std::string camera_type_;
    std::string head_pose_topic_ = "/head_pose_stamped";
    std::string raw_head_pose_fallback_topic_ = "/head_pose";
    std::string img_log_path_;

    Intrinsics intr_;
    Intrinsics yaml_intr_;
    IntrinsicsSource intrinsics_source_{IntrinsicsSource::kWaiting};
    std::chrono::steady_clock::time_point intrinsics_wait_start_;
    mutable std::shared_mutex intrinsics_mutex_;
    rclcpp::TimerBase::SharedPtr intrinsics_fallback_timer_;
    double intrinsics_wait_timeout_sec_ = 3.0;
    Pose p_eye2head_;
    Pose p_headprime2head_;
    Pose p_previous_head2base_;
    bool auto_calibrate_ = false;
    bool auto_calibrate_pose_ready_ = false;
    // /head_pose_stamped reports this head point relative to the robot base.
    // The value is shared by the T2 motion-state publisher configuration.
    Pose p_headpoint2pitchlink_{0.0613f, 0.0f, 0.108f, 0.0f, 0.0f, 0.0f};
    Pose p_camera_link2pitchlink_;
    std::string auto_calibrate_robot_model_;
    std::string auto_calibrate_camera_link_;
    std::mutex camera_extrinsics_mutex_;
    float z_compensation_ = 0;
    float ball_x_compensation_ = 0;
    float ball_y_compensation_ = 0;
    double pose_max_sync_diff_ms_ = 40.0;
    double pose_interpolation_max_gap_ms_ = 40.0;
    double pose_sync_wait_timeout_ms_ = 12.0;
    std::atomic<bool> head_pose_input_log_printed_{false};
    std::atomic<bool> invalid_head_pose_log_printed_{false};
    std::atomic<uint64_t> color_frames_received_{0};
    std::atomic<uint64_t> color_frames_processing_started_{0};
    std::atomic<uint64_t> color_frames_completed_{0};
    std::atomic<uint64_t> depth_frames_received_{0};
    std::atomic<uint64_t> head_pose_frames_received_{0};
    std::atomic<int64_t> last_color_received_steady_ms_{0};
    std::atomic<int64_t> last_color_completed_steady_ms_{0};
    std::atomic<int64_t> last_head_pose_received_steady_ms_{0};
    std::atomic<int64_t> last_pipeline_duration_ms_{-1};
    std::atomic<double> last_pose_sync_diff_ms_{-1.0};
    std::atomic<double> last_depth_sync_diff_ms_{-1.0};
    std::atomic<SyncState> last_pose_sync_state_{SyncState::kUnavailable};
    std::atomic<SyncState> last_depth_sync_state_{SyncState::kDisabled};
    std::atomic<ProcessingStage> processing_stage_{ProcessingStage::kIdle};
    rclcpp::TimerBase::SharedPtr vision_heartbeat_timer_;
    int line_segment_area_threshold_ = 10; // threshold for line segment detection

    // post processing
    bool enable_post_process_ = false;
    bool single_ball_assumption_ = false;
    std::vector<std::string> classnames_;
    // std::vector<float> duration_list_;
    // std::vector<std::string> duration_name_list_;
    std::map<std::string, float> confidence_map_;
    // std::string profiler_log_path_;
    // bool first_write_profiler_log_ = true;


    std::shared_ptr<rclcpp::Node> nh_;
    rclcpp::Publisher<vision_interface::msg::Detections>::SharedPtr detection_pub_;
    rclcpp::Publisher<vision_interface::msg::LineSegments>::SharedPtr field_line_pub_;
    rclcpp::Publisher<vision_interface::msg::Ball>::SharedPtr ball_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr segmentation_img_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pose_tf_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_tf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber color_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Subscriber color_seg_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<vision_interface::msg::CalParam>::SharedPtr calParam_sub_; // Sub for calibration params

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_3_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_4_;

    std::shared_ptr<ColorClassifier> color_classifier_;

    std::shared_ptr<DataLogger> data_logger_;
    std::shared_ptr<DataSyncer> data_syncer_;
    std::shared_ptr<DataSyncer> seg_data_syncer_;
    std::shared_ptr<YoloV8Detector> detector_;
    std::shared_ptr<YoloV8Segmentor> segmentor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::map<std::string, std::shared_ptr<PoseEstimator>> pose_estimator_map_;

    bool InitAutoCalibrationPoseAdapter(const YAML::Node &node);
    Pose AdaptHeadPoseForAutoCalibration(const Pose &headpoint2base) const;
};

} // namespace booster_vision
