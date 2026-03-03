#pragma once

#include <vector>
#include <chrono>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

namespace booster_vision {

std::string getTimeString() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_c);
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    return ss.str();
}

}  // booster_vision

namespace YAML {
template <>
struct convert<std::vector<double>> {
    static bool decode(const Node &node, std::vector<double> &vec) {
        if (!node.IsSequence()) {
            return false; // Or throw an exception
        }
        vec.clear();
        for (const auto &item : node) {
            vec.push_back(item.as<double>());
        }
        return true;
    }
};

template <>
struct convert<std::vector<std::string>> {
    static bool decode(const Node &node, std::vector<std::string> &vec) {
        if (!node.IsSequence()) {
            return false; // Or throw an exception
        }
        vec.clear();
        for (const auto &item : node) {
            vec.push_back(item.as<std::string>());
        }
        return true;
    }
};

} // namespace YAML

template <typename T>
T as_or(const YAML::Node &node, const T &default_value) {
    if (node) {
        try {
            return node.as<T>();
        } catch (const YAML::Exception &) {
            // Handle conversion failure
            return default_value;
        }
    }
    return default_value;
}

void MergeYAML(YAML::Node a, const YAML::Node &b) {
    if (!b.IsMap()) {
        a = b;
        return;
    }

    for (const auto &it : b) {
        const std::string &key = it.first.as<std::string>();
        const YAML::Node &b_value = it.second;

        if (a[key]) {
            MergeYAML(a[key], b_value);
        } else {
            a[key] = b_value;
        }
    }
}
