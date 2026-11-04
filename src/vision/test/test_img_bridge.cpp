#include <algorithm>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "booster_vision/img_bridge.h"

namespace booster_vision {
namespace {

sensor_msgs::msg::Image MakeImage(const std::string &encoding,
                                uint32_t step, bool nv12 = false) {
    sensor_msgs::msg::Image image;
    image.width = 16;
    image.height = 8;
    image.encoding = encoding;
    image.step = step;
    image.is_bigendian = false;
    image.data.resize(step * (nv12 ? image.height * 3 / 2 : image.height), 0);
    return image;
}

void ExpectUniformBGR(const cv::Mat &image, const cv::Scalar &pixel) {
    ASSERT_EQ(image.type(), CV_8UC3);
    ASSERT_EQ(image.size(), cv::Size(16, 8));
    EXPECT_EQ(cv::norm(image, cv::Mat(image.size(), image.type(), pixel), cv::NORM_INF), 0);
}

TEST(ImageBridgeTest, ColorEncodingDeterminesConversionAndPreservesInput) {
    for (const std::string encoding : {"bgr8", "rgb8", "bgra8", "rgba8", "mono8"}) {
        SCOPED_TRACE(encoding);
        const int channels = encoding == "mono8" ? 1 :
            (encoding == "bgra8" || encoding == "rgba8" ? 4 : 3);
        const bool rgb = encoding == "rgb8" || encoding == "rgba8";
        const cv::Scalar input = channels == 1 ? cv::Scalar(80) :
            (rgb ? cv::Scalar(160, 80, 20, 255) : cv::Scalar(20, 80, 160, 255));
        for (const bool padded : {false, true}) {
            SCOPED_TRACE(padded);
            auto msg = MakeImage(encoding, 16 * channels + (padded ? 64 : 0));
            cv::Mat pixels(msg.height, msg.width, CV_MAKETYPE(CV_8U, channels),
                           msg.data.data(), msg.step);
            pixels.setTo(input);
            const auto original = msg.data;
            const cv::Mat decoded = toBGRMat(msg);
            EXPECT_EQ(msg.data, original);
            std::fill(msg.data.begin(), msg.data.end(), 0);
            ExpectUniformBGR(decoded, channels == 1 ? cv::Scalar(80, 80, 80) :
                                                   cv::Scalar(20, 80, 160));
        }
    }
}

TEST(ImageBridgeTest, NV12HandlesPackedAndPaddedRows) {
    for (const uint32_t step : {16U, 32U}) {
        SCOPED_TRACE(step);
        auto msg = MakeImage("nv12", step, true);
        cv::Mat packed(msg.height * 3 / 2, msg.width, CV_8UC1, cv::Scalar(128));
        packed.rowRange(0, msg.height).setTo(cv::Scalar(90));
        for (int row = 0; row < packed.rows; ++row) {
            std::copy_n(packed.ptr<uint8_t>(row), msg.width,
                        msg.data.begin() + row * step);
        }
        const auto original = msg.data;
        cv::Mat expected;
        cv::cvtColor(packed, expected, cv::COLOR_YUV2BGR_NV12);
        const cv::Mat decoded = toBGRMat(msg);
        EXPECT_EQ(msg.data, original);
        EXPECT_EQ(cv::norm(decoded, expected, cv::NORM_INF), 0);
        EXPECT_GT(cv::mean(decoded)[0], 0);
    }
}

TEST(ImageBridgeTest, RejectsInvalidNV12AndBGRALayouts) {
    for (const bool nv12 : {false, true}) {
        SCOPED_TRACE(nv12);
        const uint32_t row_bytes = nv12 ? 16 : 64;
        auto truncated = MakeImage(nv12 ? "nv12" : "bgra8", row_bytes, nv12);
        truncated.data.pop_back();
        EXPECT_THROW(toCVMat(truncated), std::runtime_error);

        auto short_step = MakeImage(nv12 ? "nv12" : "bgra8", row_bytes - 1, nv12);
        EXPECT_THROW(toCVMat(short_step), std::runtime_error);
    }
    auto odd_width = MakeImage("nv12", 16, true);
    odd_width.width = 15;
    EXPECT_THROW(toCVMat(odd_width), std::runtime_error);
    auto odd_height = MakeImage("nv12", 16, true);
    odd_height.height = 7;
    EXPECT_THROW(toCVMat(odd_height), std::runtime_error);
}

TEST(ImageBridgeTest, ZeroStepUsesPackedLayout) {
    for (const bool nv12 : {false, true}) {
        auto msg = MakeImage(nv12 ? "nv12" : "bgra8", nv12 ? 16 : 64, nv12);
        std::fill(msg.data.begin(), msg.data.end(), 128);
        const cv::Mat expected = toBGRMat(msg);
        msg.step = 0;
        EXPECT_EQ(cv::norm(toBGRMat(msg), expected, cv::NORM_INF), 0);
    }
}

TEST(ImageBridgeTest, DepthConversionKeepsItsOriginalType) {
    auto msg = MakeImage("16UC1", 32);
    const cv::Mat depth = toCVMat(msg);
    EXPECT_EQ(depth.type(), CV_16UC1);
    EXPECT_THROW(toBGRMat(msg), std::runtime_error);
}

} // namespace
} // namespace booster_vision
