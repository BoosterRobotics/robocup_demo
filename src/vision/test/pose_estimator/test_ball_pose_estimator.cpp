#include <gtest/gtest.h>

#include <array>
#include <tuple>

#include "booster_vision/pose_estimator/pose_estimator.h"

namespace booster_vision {
namespace {

TEST(BallPoseEstimatorTest, AppliesConfiguredXYCompensationOnly) {
    const Intrinsics intrinsics(100.0f, 100.0f, 50.0f, 50.0f);
    BallPoseEstimator uncompensated_estimator(intrinsics);
    BallPoseEstimator compensated_estimator(intrinsics);

    YAML::Node uncompensated_config;
    uncompensated_estimator.Init(uncompensated_config);

    YAML::Node compensated_config;
    compensated_config["x_compensation"] = 0.12f;
    compensated_config["y_compensation"] = -0.08f;
    compensated_estimator.Init(compensated_config);

    // The camera is one metre above the ground and looks downward.
    const Pose camera_to_base(0.0f, 0.0f, 1.0f, 0.0f, CV_PI, 0.0f);
    DetectionRes detection;
    detection.bbox = cv::Rect(40, 40, 20, 20);

    const auto uncompensated =
        uncompensated_estimator.EstimateByColor(camera_to_base, detection, cv::Mat())
            .getTranslationVec();
    const auto compensated =
        compensated_estimator.EstimateByColor(camera_to_base, detection, cv::Mat())
            .getTranslationVec();

    EXPECT_NEAR(compensated[0] - uncompensated[0], 0.12f, 1e-5f);
    EXPECT_NEAR(compensated[1] - uncompensated[1], -0.08f, 1e-5f);
    EXPECT_NEAR(compensated[2], uncompensated[2], 1e-5f);
}

TEST(BallPoseEstimatorTest, IntersectsBBoxCenterRayWithConfiguredBallHeight) {
    const Intrinsics intrinsics(100.0f, 100.0f, 50.0f, 50.0f);
    const Pose camera_to_base(0.0f, 0.0f, 1.0f, 0.0f, CV_PI, 0.0f);
    DetectionRes detection;
    detection.bbox = cv::Rect(50, 50, 20, 20);

    struct BallCase {
        int size;
        float radius;
    };
    const std::array<BallCase, 3> ball_cases{{
        {3, 0.090f},
        {4, 0.095f},
        {5, 0.105f},
    }};

    for (const auto &ball : ball_cases) {
        BallPoseEstimator estimator(intrinsics);
        YAML::Node config;
        config["ball_size"] = ball.size;
        estimator.Init(config);

        const auto position =
            estimator.EstimateByColor(camera_to_base, detection, cv::Mat())
                .getTranslationVec();
        const float ray_scale = 1.0f - ball.radius;

        EXPECT_NEAR(position[0], -0.1f * ray_scale, 1e-5f);
        EXPECT_NEAR(position[1], 0.1f * ray_scale, 1e-5f);
        EXPECT_NEAR(position[2], ball.radius, 1e-5f);
    }
}

TEST(PoseEstimatorTest, GroundIntersectionRemainsAtZeroHeight) {
    const Intrinsics intrinsics(100.0f, 100.0f, 50.0f, 50.0f);
    const Pose camera_to_base(0.0f, 0.0f, 1.0f, 0.0f, CV_PI, 0.0f);

    const auto position = CalculatePositionByIntersection(
        camera_to_base, cv::Point2f(60.0f, 60.0f), intrinsics);

    EXPECT_NEAR(position.z, 0.0f, 1e-5f);
}

TEST(PoseEstimatorTest, RejectsRayPointingAwayFromGroundPlane) {
    const Intrinsics intrinsics(100.0f, 100.0f, 50.0f, 50.0f);
    const Pose camera_to_base(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    cv::Point3f position;

    EXPECT_FALSE(TryCalculatePositionByIntersection(
        camera_to_base, cv::Point2f(50.0f, 50.0f), intrinsics, 0.0f, position));
}

TEST(PoseEstimatorTest, FixedTargetIsInvariantAcrossHeadAngles) {
    const Intrinsics intrinsics(642.414673f, 641.951172f, 639.024414f, 353.080536f);
    cv::Mat eye_to_head_matrix = (cv::Mat_<float>(4, 4) <<
        -0.00851856079f, -0.297874361f, 0.954567015f, -0.0115087703f,
        -0.999961972f, 0.000737651193f, -0.00869347993f, 0.00595669867f,
        0.00188542693f, -0.954604805f, -0.297869325f, -0.0339295864f,
        0.0f, 0.0f, 0.0f, 1.0f);
    const Pose eye_to_head(eye_to_head_matrix);
    const cv::Point3f fixed_ball_center(3.0f, 0.35f, 0.09f);
    const std::array<std::tuple<float, float>, 6> head_angles{{
        {-10.0f, 0.0f},
        {0.0f, -15.0f},
        {0.0f, 0.0f},
        {0.0f, 15.0f},
        {10.0f, 0.0f},
        {20.0f, 0.0f},
    }};

    for (const auto &[pitch_deg, yaw_deg] : head_angles) {
        const Pose head_to_base(
            0.0f, 0.0f, 1.15f, 0.0f,
            pitch_deg * static_cast<float>(CV_PI) / 180.0f,
            yaw_deg * static_cast<float>(CV_PI) / 180.0f);
        const Pose eye_to_base = head_to_base * eye_to_head;
        const cv::Point3f ball_in_eye = eye_to_base.inverse() * fixed_ball_center;
        ASSERT_GT(ball_in_eye.z, 0.0f);

        const cv::Point2f ball_pixel = intrinsics.Project(ball_in_eye);
        const cv::Point3f reconstructed = CalculatePositionByIntersection(
            eye_to_base, ball_pixel, intrinsics, fixed_ball_center.z);

        EXPECT_NEAR(reconstructed.x, fixed_ball_center.x, 1e-4f);
        EXPECT_NEAR(reconstructed.y, fixed_ball_center.y, 1e-4f);
        EXPECT_NEAR(reconstructed.z, fixed_ball_center.z, 1e-5f);
    }
}

} // namespace
} // namespace booster_vision
