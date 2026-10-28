#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "booster_vision/base/intrin.h"

namespace booster_vision {

class IntrinsicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up any common resources for tests
    }

    void TearDown() override {
        // Clean up any resources used by tests
    }
};

TEST_F(IntrinsicsTest, ConstructorFromYAMLNode) {
    YAML::Node node;
    node["fx"] = 500.0;
    node["fy"] = 500.0;
    node["cx"] = 320.0;
    node["cy"] = 240.0;
    node["distortion_coeffs"] = std::vector<float>{0.1f, 0.01f, 0.001f, 0.0001f, 0.00001f};
    node["distortion_model"] = 1; // kBrownConrady

    Intrinsics intrinsics(node);

    EXPECT_FLOAT_EQ(intrinsics.fx, 500.0);
    EXPECT_FLOAT_EQ(intrinsics.fy, 500.0);
    EXPECT_FLOAT_EQ(intrinsics.cx, 320.0);
    EXPECT_FLOAT_EQ(intrinsics.cy, 240.0);
    EXPECT_EQ(intrinsics.distortion_coeffs.size(), 5);
    EXPECT_EQ(intrinsics.model, Intrinsics::kBrownConrady);
}

TEST_F(IntrinsicsTest, ConstructorFromMatAndDistortionCoeffs) {
    cv::Mat intr = (cv::Mat_<float>(3, 3) << 500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1);
    std::vector<float> distortion_coeffs = {0.1f, 0.01f, 0.001f, 0.0001f, 0.00001f};
    Intrinsics intrinsics(intr, distortion_coeffs, Intrinsics::kBrownConrady);

    EXPECT_FLOAT_EQ(intrinsics.fx, 500.0);
    EXPECT_FLOAT_EQ(intrinsics.fy, 500.0);
    EXPECT_FLOAT_EQ(intrinsics.cx, 320.0);
    EXPECT_FLOAT_EQ(intrinsics.cy, 240.0);
    EXPECT_EQ(intrinsics.distortion_coeffs.size(), 5);
    EXPECT_EQ(intrinsics.model, Intrinsics::kBrownConrady);
}

TEST_F(IntrinsicsTest, ConstructorFromFloatValues) {
    std::vector<float> distortion_coeffs = {0.1f, 0.01f, 0.001f, 0.0001f, 0.00001f};
    Intrinsics intrinsics(500.0f, 500.0f, 320.0f, 240.0f, distortion_coeffs, Intrinsics::kBrownConrady);

    EXPECT_FLOAT_EQ(intrinsics.fx, 500.0);
    EXPECT_FLOAT_EQ(intrinsics.fy, 500.0);
    EXPECT_FLOAT_EQ(intrinsics.cx, 320.0);
    EXPECT_FLOAT_EQ(intrinsics.cy, 240.0);
    EXPECT_EQ(intrinsics.distortion_coeffs.size(), 5);
    EXPECT_EQ(intrinsics.model, Intrinsics::kBrownConrady);
}

TEST_F(IntrinsicsTest, Project) {
    Intrinsics intrinsics(500.0f, 500.0f, 320.0f, 240.0f);
    cv::Point3f point3D(1.0f, 1.0f, 1.0f);
    cv::Point2f projected = intrinsics.Project(point3D);

    EXPECT_FLOAT_EQ(projected.x, 820.0);
    EXPECT_FLOAT_EQ(projected.y, 740.0);
}

TEST_F(IntrinsicsTest, BackProject) {
    Intrinsics intrinsics(500.0f, 500.0f, 320.0f, 240.0f);
    cv::Point2f point2D(820.0f, 740.0f);
    cv::Point3f backProjected = intrinsics.BackProject(point2D, 1.0f);

    EXPECT_FLOAT_EQ(backProjected.x, 1.0);
    EXPECT_FLOAT_EQ(backProjected.y, 1.0);
    EXPECT_FLOAT_EQ(backProjected.z, 1.0);
}

TEST_F(IntrinsicsTest, UnDistort) {
    Intrinsics intrinsics(500.0f, 500.0f, 320.0f, 240.0f);
    cv::Point2f distortedPoint(820.0f, 740.0f);
    cv::Point2f undistortedPoint = intrinsics.UnDistort(distortedPoint);

    // Assuming no distortion, the undistorted point should be the same as the input point
    EXPECT_FLOAT_EQ(undistortedPoint.x, 820.0);
    EXPECT_FLOAT_EQ(undistortedPoint.y, 740.0);
}

} // namespace booster_vision