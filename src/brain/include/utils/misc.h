#pragma once

#include "chrono"
#include <string>
#include <random>
#include <sstream>
#include <yaml-cpp/yaml.h>

// Calculate milliseconds elapsed since start_time
inline int msecsSince(std::chrono::high_resolution_clock::time_point start_time)
{
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}

inline std::string gen_uuid() {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> uni(0, 15);
    std::uniform_int_distribution<int> uni8(8, 11);

    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 8; i++) ss << uni(rng);
    ss << "-";
    for (int i = 0; i < 4; i++) ss << uni(rng);
    ss << "-4"; // The 13th character is '4', indicating this is a version 4 UUID
    for (int i = 0; i < 3; i++) ss << uni(rng);
    ss << "-";
    ss << uni8(rng); // The 17th character is 8, 9, A, or B
    for (int i = 0; i < 3; i++) ss << uni(rng);
    ss << "-";
    for (int i = 0; i < 12; i++) ss << uni(rng);

    return ss.str();
}

// generate a timestamp string in format "YYYYMMDD_HHMMSS"
inline std::string gen_timestamp_str() {
    auto now = std::chrono::system_clock::now();
    now += std::chrono::hours(0); // Force to UTC+0 timezone
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::gmtime(&now_time_t); 
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &now_tm);
    return std::string(timestamp);
}

// generate a timestamped file name under a given directory
inline std::string gen_timestamped_filename(const std::string& dir, const std::string& ext = "", bool addUUID = false) {
    auto filename = gen_timestamp_str();
    if (addUUID) filename += ("_" + gen_uuid());
    filename += ext;
    
    if (dir.back() == '/') return dir + filename;
    
    // else
    return dir + "/" + filename;
}

inline bool mkdir_if_not_exist(const std::string& dir) {
    std::string cmd = "mkdir -p " + dir;
    return system(cmd.c_str()) == 0;
}

inline void MergeYAML(YAML::Node a, const YAML::Node &b) {
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