#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "booster_vision/base/latest_image_buffer.hpp"

namespace booster_vision {

TEST(LatestImageBufferTest, KeepsNewestSnapshotAndReleasesConsumedFrame) {
    LatestImageBuffer buffer;
    EXPECT_TRUE(buffer.Take().empty());
    cv::Mat frame(8, 16, CV_8UC3, cv::Scalar(20, 80, 160));
    buffer.Store(frame);
    frame.setTo(cv::Scalar(30, 90, 170));
    buffer.Store(frame);
    frame.setTo(cv::Scalar(0, 0, 0));

    const cv::Mat shown = buffer.Take();
    ASSERT_FALSE(shown.empty());
    EXPECT_EQ(shown.at<cv::Vec3b>(0, 0), cv::Vec3b(30, 90, 170));
    EXPECT_TRUE(buffer.Take().empty());
}

TEST(LatestImageBufferTest, EmptyInputDoesNotErasePendingFrame) {
    LatestImageBuffer buffer;
    buffer.Store(cv::Mat(2, 2, CV_8UC3, cv::Scalar(10, 20, 30)));
    buffer.Store(cv::Mat());
    ASSERT_FALSE(buffer.Take().empty());
}

TEST(LatestImageBufferTest, ExecutorThreadsCanSubmitWhileMainThreadConsumes) {
    LatestImageBuffer buffer;
    std::atomic<int> finished{0};
    std::vector<std::thread> producers;
    for (int id = 1; id <= 4; ++id) {
        producers.emplace_back([&, id]() {
            for (int frame = 0; frame < 100; ++frame) {
                cv::Mat pixels(8, 16, CV_8UC3, cv::Scalar(id, frame, 200));
                buffer.Store(pixels);
                pixels.setTo(cv::Scalar(0, 0, 0));
            }
            finished.fetch_add(1);
        });
    }
    int consumed = 0;
    for (;;) {
        const bool all_finished = finished.load() == 4;
        const cv::Mat shown = buffer.Take();
        if (!shown.empty()) {
            ++consumed;
            EXPECT_EQ(shown.type(), CV_8UC3);
            const auto pixel = shown.at<cv::Vec3b>(0, 0);
            EXPECT_GE(pixel[0], 1);
            EXPECT_LE(pixel[0], 4);
            EXPECT_EQ(pixel[2], 200);
            EXPECT_EQ(cv::norm(shown, cv::Mat(shown.size(), shown.type(),
                          cv::Scalar(pixel[0], pixel[1], pixel[2])), cv::NORM_INF), 0);
        }
        if (all_finished) break;
        std::this_thread::yield();
    }
    for (auto &producer : producers) producer.join();
    EXPECT_GT(consumed, 0);
}

} // namespace booster_vision
