#include <gtest/gtest.h>
#include <gtest/gtest.h>
#include "booster_vision/model/detector.h"
#include <opencv2/opencv.hpp>

using booster_vision::YoloV8Detector;
using booster_vision::DetectionRes;


std::string g_cfg_path = "";
std::string g_img_path = "";
class YoloV8DetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize the detector with a configuration node
        YAML::Node config = YAML::LoadFile(g_cfg_path);
        detector = YoloV8Detector::CreateYoloV8Detector(config["detection_model"], "");
    }

    std::shared_ptr<YoloV8Detector> detector;
};

TEST_F(YoloV8DetectorTest, SetAndGetParams) {
    float original_confidence_threshold = detector->getConfidenceThreshold();
    float original_nms_threshold = detector->getNMSThreshold();

    float confidence_threshold = 0.5f;
    detector->setConfidenceThreshold(confidence_threshold);
    float retrieved_threshold = detector->getConfidenceThreshold();
    ASSERT_EQ(retrieved_threshold, confidence_threshold);

    float nms_threshold = 0.2;
    detector->setNMSThreshold(nms_threshold);
    float retrieved_nms_threshold = detector->getNMSThreshold();
    ASSERT_EQ(retrieved_nms_threshold, nms_threshold);

    // Reset the original values
    detector->setConfidenceThreshold(original_confidence_threshold);
    detector->setNMSThreshold(original_nms_threshold);
}

TEST_F(YoloV8DetectorTest, DetectObjects) {
    float orginal_threshold = detector->getConfidenceThreshold();
    // read image
    cv::Mat img = cv::imread(g_img_path);

    std::vector<DetectionRes> objects = detector->Inference(img);
    int num_objs = static_cast<int>(objects.size());
    ASSERT_GT(num_objs, 0);

    auto display_img = booster_vision::YoloV8Detector::DrawDetection(img, objects);
    cv::imshow("Detected Objects", display_img);
    cv::waitKey(0);

    detector->setConfidenceThreshold(0.99f);
    objects = detector->Inference(img);
    int num_objs_new = static_cast<int>(objects.size());
    ASSERT_GE(num_objs, num_objs_new);

    detector->setConfidenceThreshold(orginal_threshold);
}

TEST_F(YoloV8DetectorTest, DetectObjectsOnEmptyImage) {
    cv::Mat img = cv::Mat::zeros(cv::Size(640, 640), CV_8UC3);
    std::vector<DetectionRes> objects = detector->Inference(img);
    ASSERT_EQ(objects.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    g_cfg_path = argv[1];
    g_img_path = argv[2];
    return RUN_ALL_TESTS();
}