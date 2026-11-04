#include "booster_vision/img_bridge.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rcpputils/endian.hpp"

#include <map>
#include <memory>
#include <cstdint>
#include <cstring>
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

        const size_t row_bytes = static_cast<size_t>(width) * sizeof(uint16_t);
        const size_t step = source.step == 0 ? row_bytes : source.step;
        if (step < row_bytes || source.data.size() < step * static_cast<size_t>(height)) {
            throw Exception("MONO16 image has invalid step/data size");
        }

        // Create a tightly packed Mat; ROS image rows may contain padding.
        cv::Mat depth_mat(height, width, CV_16UC1);
        for (int row = 0; row < height; ++row) {
            std::memcpy(
                depth_mat.ptr<uint8_t>(row),
                source.data.data() + static_cast<size_t>(row) * step,
                row_bytes);
        }
        
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
        // NV12 has a Y plane followed by interleaved UV rows, using the same
        // stride. Copy pixels row by row; source.data can include padding.
        const int width = source.width;
        const int height = source.height;
        if (width <= 0 || height <= 0 || width % 2 != 0 || height % 2 != 0) {
            throw Exception("NV12 image requires positive, even width and height");
        }
        const size_t row_bytes = static_cast<size_t>(width);
        const size_t step = source.step == 0 ? row_bytes : source.step;
        const size_t rows = static_cast<size_t>(height) * 3 / 2;
        if (step < row_bytes || source.data.size() < step * rows) {
            throw Exception("NV12 image has invalid step/data size");
        }
        cv::Mat nv12(static_cast<int>(rows), width, CV_8UC1);
        for (size_t row = 0; row < rows; ++row) {
            std::memcpy(nv12.ptr<uint8_t>(static_cast<int>(row)),
                        source.data.data() + row * step, row_bytes);
        }
        
        // Convert NV12 to BGR format
        cv::Mat bgr_mat;
        cv::cvtColor(nv12, bgr_mat, cv::COLOR_YUV2BGR_NV12);
        return bgr_mat;
    }

    // special handling for bgra8
    if (source.encoding == enc::BGRA8) {
        const int width = source.width;
        const int height = source.height;
        const size_t row_bytes = static_cast<size_t>(width) * 4;
        const size_t step = source.step == 0 ? row_bytes : source.step;
        if (width <= 0 || height <= 0 || step < row_bytes ||
            source.data.size() < step * static_cast<size_t>(height)) {
            throw Exception("BGRA image has invalid dimensions/step/data size");
        }
        cv::Mat bgra_mat(height, width, CV_8UC4);
        for (int row = 0; row < height; ++row) {
            std::memcpy(bgra_mat.ptr<uint8_t>(row),
                        source.data.data() + static_cast<size_t>(row) * step,
                        row_bytes);
        }

        // Convert BGRA to BGR format
        cv::Mat bgr_mat;
        cv::cvtColor(bgra_mat, bgr_mat, cv::COLOR_BGRA2BGR);
        return bgr_mat;
    }

    int source_type = getCvType(source.encoding);
    int byte_depth = enc::bitDepth(source.encoding) / 8;
    int num_channels = enc::numChannels(source.encoding);

    const size_t row_bytes = static_cast<size_t>(source.width) *
        static_cast<size_t>(byte_depth) * static_cast<size_t>(num_channels);
    const size_t step = source.step == 0 ? row_bytes : source.step;
    if (step < row_bytes) {
        std::stringstream ss;
        ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  "
           << step << " < " << source.width << " * " << byte_depth << " * " << num_channels;
        throw Exception(ss.str());
    }

    if (source.data.size() < step * static_cast<size_t>(source.height)) {
        std::stringstream ss;
        ss << "Image is wrongly formed: data is shorter than height * step  or  "
           << source.data.size() << " < " << source.height << " * " << step;
        throw Exception(ss.str());
    }

    // Copy each row into a tightly packed Mat.  Camera drivers are allowed to
    // pad Image rows, while OpenCV algorithms and the depth estimators assume
    // a regular row stride.
    cv::Mat mat(source.height, source.width, source_type);
    for (size_t row = 0; row < source.height; ++row) {
        std::memcpy(
            mat.ptr<uint8_t>(static_cast<int>(row)),
            source.data.data() + row * step,
            row_bytes);
    }

    const bool same_endian =
        (rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) ||
        (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian);
    if (same_endian || byte_depth == 1) {
        return mat;
    }

    // Otherwise reverse bytes independently for every channel.  Doing this
    // explicitly avoids relying on mixChannels' channel numbering for a
    // multi-byte scalar, and preserves the original OpenCV depth exactly.
    cv::Mat converted(source.height, source.width, source_type);
    for (int row = 0; row < static_cast<int>(source.height); ++row) {
        const uint8_t *src_row = mat.ptr<uint8_t>(row);
        uint8_t *dst_row = converted.ptr<uint8_t>(row);
        for (size_t byte_offset = 0;
             byte_offset < row_bytes;
             byte_offset += static_cast<size_t>(byte_depth)) {
            for (int byte = 0; byte < byte_depth; ++byte) {
                dst_row[byte_offset + static_cast<size_t>(byte)] =
                    src_row[byte_offset + static_cast<size_t>(byte_depth - 1 - byte)];
            }
        }
    }
    return converted;
}

cv::Mat toBGRMat(const sensor_msgs::msg::Image &source) {
    cv::Mat image = toCVMat(source);
    if (image.empty()) {
        throw Exception("Color image is empty");
    }
    if (source.encoding == enc::RGB8) {
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    } else if (source.encoding == enc::RGBA8) {
        cv::cvtColor(image, image, cv::COLOR_RGBA2BGR);
    } else if (source.encoding == enc::MONO8) {
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    }
    // toCVMat already converts NV12/BGRA8 to BGR and preserves BGR8.
    if (image.type() != CV_8UC3) {
        throw Exception("Unsupported color image encoding [" + source.encoding + "]");
    }
    return image;
}

} // namespace booster_vision
