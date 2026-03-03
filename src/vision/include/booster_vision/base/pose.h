#pragma once

#include <iostream>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

// #include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace booster_vision {

struct Pose {
public:
    Pose() = default;
    Pose(const float &x, const float &y, const float &z,
         const float &roll, const float &pitch, const float &yaw);
    Pose(cv::Mat &pose) :
        mat_pose(pose){};
    Pose(const cv::Mat &rot, const cv::Mat &trans);
    Pose(float x, float y, float z,
         float qx, float qy, float qz, float qw);
    Pose(const geometry_msgs::msg::TransformStamped &msg);

    // inverse
    Pose inverse() const {
        Pose result;
        result.mat_pose = mat_pose.inv();
        return result;
    }

    cv::Mat toCVMat() const {
        return mat_pose;
    }
    geometry_msgs::msg::TransformStamped toRosTFMsg();

    // geters
    cv::Mat getRotationMatrix() const {
        return mat_pose(cv::Rect(0, 0, 3, 3)).clone();
    }

    cv::Mat getTranslationVecMatrix() const {
        return mat_pose(cv::Rect(3, 0, 1, 3)).clone();
    }

    std::vector<double> getQuaternionVec() const;  // x,y,z,w
    std::vector<float> getEulerAnglesVec() const; // roll, pitch, yaw
    std::vector<float> getTranslationVec() const {
        return {mat_pose.at<float>(0, 3), mat_pose.at<float>(1, 3), mat_pose.at<float>(2, 3)};
    }

    Pose operator*(const Pose &other) const {
        Pose result;
        result.mat_pose = mat_pose * other.mat_pose;
        return result;
    }

    cv::Point3f operator*(const cv::Point3f &point) const {
        cv::Mat mat_point = (cv::Mat_<float>(4, 1) << point.x, point.y, point.z, 1);
        cv::Mat result = mat_pose * mat_point;
        return cv::Point3f(result.at<float>(0, 0), result.at<float>(1, 0), result.at<float>(2, 0));
    }

    bool operator==(const Pose &other) const {
        return cv::countNonZero(mat_pose != other.mat_pose) == 0;
    }

    bool operator!=(const Pose &other) const {
        return !(*this == other);
    }

    friend std::ostream &operator<<(std::ostream &os, const Pose &pose) {
        os << pose.mat_pose;
        return os;
    }

private:
    cv::Mat mat_pose = cv::Mat::eye(4, 4, CV_32F);
};

} // namespace booster_vision

namespace YAML {
// Specialize the convert template for Pose
template <>
struct convert<booster_vision::Pose> {
    static Node encode(const booster_vision::Pose &pose) {
        Node node;
        cv::Mat mat = pose.toCVMat();
        for (int i = 0; i < 4; ++i) {
            Node row(NodeType::Sequence);
            for (int j = 0; j < 4; ++j) {
                row.push_back(mat.at<float>(i, j));
            }
            node.push_back(row);
        }
        return node;
    }

    static bool decode(const Node &node, booster_vision::Pose &pose) {
        if (node.size() != 4) {
            return false; // Or throw an exception
        }
        cv::Mat mat = cv::Mat::zeros(4, 4, CV_32F);
        for (size_t i = 0; i < 4; ++i) {
            const auto &row = node[i];
            if (!row.IsSequence() || row.size() != 4) {
                return false; // Or throw an exception
            }
            for (size_t j = 0; j < 4; ++j) {
                mat.at<float>(i, j) = row[j].as<float>();
            }
        }
        pose = booster_vision::Pose(mat);
        return true;
    }
};

} // namespace YAML
