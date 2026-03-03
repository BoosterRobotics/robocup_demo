#pragma once
#include <string>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace booster_vision {

struct Intrinsics {
    enum DistortionModel {
        kNone = 0,
        kBrownConrady = 1, // Opencv
        kInverseBrownConrady = 2
    };
    Intrinsics() = default;
    explicit Intrinsics(const YAML::Node &node);
    Intrinsics(const cv::Mat intr, const std::vector<float> &distortion_coeffs, const DistortionModel &model);
    Intrinsics(const cv::Mat intr) :
        Intrinsics(intr, std::vector<float>(), DistortionModel::kNone) {
    }
    Intrinsics(float fx, float fy, float cx, float cy, const std::vector<float> &distortion_coeffs, DistortionModel model);
    Intrinsics(float fx, float fy, float cx, float cy) :
        Intrinsics(fx, fy, cx, cy, std::vector<float>(), DistortionModel::kNone) {
    }

    cv::Point2f Project(const cv::Point3f &point) const;
    cv::Point3f BackProject(const cv::Point2f &point, float depth = 1.0) const;
    cv::Point2f UnDistort(const cv::Point2f &point) const;

    cv::Mat get_intrinsics_matrix() const {
        return (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    // overload << operator for printing
    friend std::ostream &operator<<(std::ostream &os, const Intrinsics &intr) {
        os << "fx: " << intr.fx << std::endl
           << "fy: " << intr.fy << std::endl
           << "cx: " << intr.cx << std::endl
           << "cy: " << intr.cy << std::endl;
        os << "distortion_model: " << static_cast<int>(intr.model) << std::endl;
        if (!intr.distortion_coeffs.empty()) {
            os << "distortion_coeffs: ";
            for (const auto &coeff : intr.distortion_coeffs) {
                os << coeff << " ";
            }
            os << std::endl;
        }
        return os;
    }

    float fx = 0;
    float fy = 0;
    float cx = 0;
    float cy = 0;
    std::vector<float> distortion_coeffs = {};
    DistortionModel model = DistortionModel::kNone;
};

} // namespace booster_vision

// encode and decode for converting between Intrinsics and YAML
namespace YAML {
// Specialize the convert template for Pose
template <>
struct convert<booster_vision::Intrinsics> {
    static Node encode(const booster_vision::Intrinsics &intrin) {
        Node node;
        node["fx"] = intrin.fx;
        node["fy"] = intrin.fy;
        node["cx"] = intrin.cx;
        node["cy"] = intrin.cy;
        node["distortion_model"] = static_cast<int>(intrin.model);
        node["distortion_coeffs"] = intrin.distortion_coeffs;
        return node;
    }

    static bool decode(const Node &node, booster_vision::Intrinsics &intr) {
        if (!node) {
            throw std::runtime_error("Intrinsics: Invalid YAML node");
        }
        intr.fx = node["fx"].as<float>();
        intr.fy = node["fy"].as<float>();
        intr.cx = node["cx"].as<float>();
        intr.cy = node["cy"].as<float>();
        intr.model = static_cast<booster_vision::Intrinsics::DistortionModel>(node["distortion_model"].as<int>());
        if (!node["distortion_coeffs"].IsSequence() || node["distortion_coeffs"].size() < 5) {
            return false;
        }
        intr.distortion_coeffs = node["distortion_coeffs"].as<std::vector<float>>();
        return true;
    }
};
} // namespace YAML