#include "booster_vision/img_bridge.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rcpputils/endian.hpp"

#include <map>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

namespace enc = sensor_msgs::image_encodings;

class Exception : public std::runtime_error {
public:
    explicit Exception(const std::string &description) :
        std::runtime_error(description) {
    }
};

static int depthStrToInt(const std::string depth) {
    if (depth == "8U") {
        return 0;
    } else if (depth == "8S") {
        return 1;
    } else if (depth == "16U") {
        return 2;
    } else if (depth == "16S") {
        return 3;
    } else if (depth == "32S") {
        return 4;
    } else if (depth == "32F") {
        return 5;
    }
    return 6;
}

int getCvType(const std::string &encoding) {
    // Check for the most common encodings first
    if (encoding == enc::BGR8) { return CV_8UC3; }
    if (encoding == enc::MONO8) { return CV_8UC1; }
    if (encoding == enc::RGB8) { return CV_8UC3; }
    if (encoding == enc::MONO16) { return CV_16UC1; }
    if (encoding == enc::BGR16) { return CV_16UC3; }
    if (encoding == enc::RGB16) { return CV_16UC3; }
    if (encoding == enc::BGRA8) { return CV_8UC4; }
    if (encoding == enc::RGBA8) { return CV_8UC4; }
    if (encoding == enc::BGRA16) { return CV_16UC4; }
    if (encoding == enc::RGBA16) { return CV_16UC4; }

    // For bayer, return one-channel
    if (encoding == enc::BAYER_RGGB8) { return CV_8UC1; }
    if (encoding == enc::BAYER_BGGR8) { return CV_8UC1; }
    if (encoding == enc::BAYER_GBRG8) { return CV_8UC1; }
    if (encoding == enc::BAYER_GRBG8) { return CV_8UC1; }
    if (encoding == enc::BAYER_RGGB16) { return CV_16UC1; }
    if (encoding == enc::BAYER_BGGR16) { return CV_16UC1; }
    if (encoding == enc::BAYER_GBRG16) { return CV_16UC1; }
    if (encoding == enc::BAYER_GRBG16) { return CV_16UC1; }

    // Miscellaneous
    if (encoding == enc::YUV422) { return CV_8UC2; }
    if (encoding == enc::YUV422_YUY2) { return CV_8UC2; }

    // Check all the generic content encodings
    std::cmatch m;

    if (std::regex_match(encoding.c_str(), m,
                         std::regex("(8U|8S|16U|16S|32S|32F|64F)C([0-9]+)"))) {
        return CV_MAKETYPE(depthStrToInt(m[1].str()), atoi(m[2].str().c_str()));
    }

    if (std::regex_match(encoding.c_str(), m,
                         std::regex("(8U|8S|16U|16S|32S|32F|64F)"))) {
        return CV_MAKETYPE(depthStrToInt(m[1].str()), 1);
    }

    throw Exception("Unrecognized image encoding [" + encoding + "]");
}

namespace booster_vision {

cv::Mat toCVMat(const sensor_msgs::msg::Image &source) {
    // Special handling for MONO16 (depth images)
    if (source.encoding == enc::MONO16) {
        int width = source.width;
        int height = source.height;
        
        // Create a Mat and copy the data
        cv::Mat depth_mat(height, width, CV_16UC1);
        memcpy(depth_mat.data, source.data.data(), source.data.size());
        
        // Handle endianness if needed
        if ((rcpputils::endian::native == rcpputils::endian::big && !source.is_bigendian) ||
            (rcpputils::endian::native == rcpputils::endian::little && source.is_bigendian)) {
            // Swap bytes for each 16-bit value
            for (int i = 0; i < depth_mat.rows; i++) {
                uint16_t* row = depth_mat.ptr<uint16_t>(i);
                for (int j = 0; j < depth_mat.cols; j++) {
                    row[j] = (row[j] >> 8) | (row[j] << 8);
                }
            }
        }
        return depth_mat;
    }

    // Special handling for NV12 format
    if (source.encoding == "nv12") {
        // NV12 format: Y plane followed by interleaved UV plane
        int width = source.width;
        int height = source.height;
        
        // Create a cv::Mat for the NV12 data
        cv::Mat nv12(height * 3 / 2, width, CV_8UC1);
        std::memcpy(nv12.data, source.data.data(), source.data.size());
        
        // Convert NV12 to BGR format
        cv::Mat bgr_mat;
        cv::cvtColor(nv12, bgr_mat, cv::COLOR_YUV2BGR_NV12);
        return bgr_mat;
    }

    // special handling for bgra8
    if (source.encoding == enc::BGRA8) {
        int width = source.width;
        int height = source.height;

        // Create a cv::Mat for the BGRA data
        cv::Mat bgra_mat(height, width, CV_8UC4);
        std::memcpy(bgra_mat.data, source.data.data(), source.data.size());

        // Convert BGRA to BGR format
        cv::Mat bgr_mat;
        cv::cvtColor(bgra_mat, bgr_mat, cv::COLOR_BGRA2BGR);
        return bgr_mat;
    }

    int source_type = getCvType(source.encoding);
    int byte_depth = enc::bitDepth(source.encoding) / 8;
    int num_channels = enc::numChannels(source.encoding);

    if (source.step < source.width * byte_depth * num_channels) {
        std::stringstream ss;
        ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " << source.step << " != " << source.width << " * " << byte_depth << " * " << num_channels;
        throw Exception(ss.str());
    }

    if (source.height * source.step != source.data.size()) {
        std::stringstream ss;
        ss << "Image is wrongly formed: height * step != size  or  " << source.height << " * " << source.step << " != " << source.data.size();
        throw Exception(ss.str());
    }

    // If the endianness is the same as locally, share the data
    cv::Mat mat(source.height, source.width, source_type, const_cast<uchar *>(&source.data[0]),
                source.step);

    if ((rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) || (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian) || byte_depth == 1) {
        return mat.clone();
    }

    // Otherwise, reinterpret the data as bytes and switch the channels accordingly
    mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
                  const_cast<uchar *>(&source.data[0]), source.step);
    cv::Mat mat_swap(source.height, source.width, mat.type());

    std::vector<int> fromTo;
    fromTo.reserve(num_channels * byte_depth);
    for (int i = 0; i < num_channels; ++i) {
        for (int j = 0; j < byte_depth; ++j) {
            fromTo.push_back(byte_depth * i + j);
            fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
        }
    }
    cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

    // Interpret mat_swap back as the proper type
    mat_swap.reshape(num_channels);

    return mat_swap;
}

} // namespace booster_vision
