#pragma once
#include "booster_vision/model//trt/config.h"

struct alignas(float) Detection {
    //center_x center_y w h
    float bbox[4];
    float conf;  // bbox_conf * cls_conf
    float class_id;
    float mask[32];
    float keypoints[kNumberOfPoints * 3];  // keypoints array with dynamic size based on kNumberOfPoints
    float angle;                           // obb angle
};

struct AffineMatrix {
    float value[6];
};

const int bbox_element =
        sizeof(AffineMatrix) / sizeof(float) + 1;  // left, top, right, bottom, confidence, class, keepflag
