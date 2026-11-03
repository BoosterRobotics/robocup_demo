#include <iostream>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include "booster_vision/base/misc_utils.hpp"

namespace booster_vision {
TEST(ConfigUtilsTest, MergeConfig) {
  YAML::Node template_node = YAML::Load("{\
    a: 1,          \
    b: {           \
      c: 2,        \
      d: 3         \
    }              \
  }");
  YAML::Node input_node = YAML::Load("{\
    b: {           \
      c: 4         \
    },             \
    e: 5           \
  }");

  auto merge_node = YAML::Clone(template_node);
  MergeYAML(merge_node, input_node);
  
  
  std::cout << "template node: " << template_node << std::endl;
  std::cout << "input node: " << input_node << std::endl;
  std::cout << "merged node: " << merge_node << std::endl;
}

TEST(ConfigUtilsTest, CameraCalibrationOverrideIgnoresOtherSections) {
  const auto template_node = YAML::Load(R"(
camera:
  type: d-robotics
  intrin_wait_timeout_sec: 3.0
  intrin: {fx: 200, fy: 201, cx: 272, cy: 224}
calibration:
  auto_calibrate: false
  offset: {exclude_distance: 2.0}
detection_model:
  model_path: model/T2_0804_digua.engine
  preprocess_backend: cuda
segmentation_model: {enabled: false}
head_pose: {topic: /head_pose_stamped}
use_depth: false
misc: {save_data_nonstationary: true}
)");
  const auto override_node = YAML::Load(R"(
camera:
  intrin: {fx: 208.71294}
  extrin: [[1, 0, 0, 0.01], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
  image_size: [544, 448]
calibration:
  auto_calibrate: true
  handeye: {reprojection_error: 1.80312649}
detection_model:
  backend: rhino
  model_path: model/T2_20260825_digua_det.ref
  preprocess_backend: cpu
segmentation_model: {enabled: true}
head_pose: {topic: /wrong_head_pose}
use_depth: true
misc: {save_data_nonstationary: false}
extra_runtime_section: {enabled: true}
)");

  auto merged = YAML::Clone(template_node);
  MergeCameraCalibrationYAML(merged, override_node);

  EXPECT_FLOAT_EQ(merged["camera"]["intrin"]["fx"].as<float>(), 208.71294f);
  EXPECT_FLOAT_EQ(merged["camera"]["intrin"]["fy"].as<float>(), 201.0f);
  EXPECT_EQ(merged["camera"]["type"].as<std::string>(), "d-robotics");
  EXPECT_DOUBLE_EQ(merged["camera"]["intrin_wait_timeout_sec"].as<double>(), 3.0);
  EXPECT_EQ(YAML::Dump(merged["camera"]["extrin"]), YAML::Dump(override_node["camera"]["extrin"]));
  EXPECT_EQ(YAML::Dump(merged["camera"]["image_size"]), "[544, 448]");
  EXPECT_TRUE(merged["calibration"]["auto_calibrate"].as<bool>());
  EXPECT_DOUBLE_EQ(merged["calibration"]["offset"]["exclude_distance"].as<double>(), 2.0);
  EXPECT_DOUBLE_EQ(merged["calibration"]["handeye"]["reprojection_error"].as<double>(), 1.80312649);
  for (const char *section : {"detection_model", "segmentation_model", "head_pose", "use_depth", "misc"}) {
    EXPECT_EQ(YAML::Dump(merged[section]), YAML::Dump(template_node[section])) << section;
  }
  EXPECT_FALSE(merged["extra_runtime_section"].IsDefined());
}

TEST(ConfigUtilsTest, CameraCalibrationOverridePreservesAbsentSections) {
  auto merged = YAML::Load(R"(
camera: {type: d-robotics, intrin: {fx: 200, fy: 201}}
calibration: {auto_calibrate: false}
use_depth: false
)");
  const auto original = YAML::Dump(merged);
  MergeCameraCalibrationYAML(merged, YAML::Load("{use_depth: true}"));
  EXPECT_EQ(YAML::Dump(merged), original);
  MergeCameraCalibrationYAML(merged, YAML::Load("{}"));
  EXPECT_EQ(YAML::Dump(merged), original);

  MergeCameraCalibrationYAML(merged, YAML::Load("{camera: {intrin: {fx: 210}}}"));
  EXPECT_FLOAT_EQ(merged["camera"]["intrin"]["fx"].as<float>(), 210.0f);
  EXPECT_FALSE(merged["calibration"]["auto_calibrate"].as<bool>());

  MergeCameraCalibrationYAML(merged, YAML::Load("{calibration: {auto_calibrate: true}}"));
  EXPECT_FLOAT_EQ(merged["camera"]["intrin"]["fx"].as<float>(), 210.0f);
  EXPECT_TRUE(merged["calibration"]["auto_calibrate"].as<bool>());
  EXPECT_FALSE(merged["use_depth"].as<bool>());
}

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
