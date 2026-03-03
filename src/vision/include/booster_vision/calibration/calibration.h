#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"

namespace booster_vision {

void EyeInHandCalibration(double *reprojection_error,
                          Pose *p_cam2head,
                          Pose *p_board2base,
                          const std::vector<Pose> &p_board2cameras,
                          const std::vector<Pose> &p_head2bases,
                          const std::vector<std::vector<cv::Point3f>> &all_corners_3d,
                          const std::vector<std::vector<cv::Point2f>> &all_corners_2d,
                          const Intrinsics &intr,
                          bool optimization = true);

void EyeInHandOffsetCalibration(double *euclidian_error,
                                Pose *p_head2head_prime,
                                const Pose &p_eye2head,
                                const std::vector<Pose> &p_head2bases,
                                const std::vector<cv::Point3f> &computed_3d_rays,
                                const std::vector<cv::Point3f> &gt_3ds,
                                const bool without_translation = true);

cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point3f &ray);

double Compute3dError(const std::vector<cv::Point3f> &rays,
                      const std::vector<cv::Point3f> &gts,
                      const std::vector<Pose> &p_head2bases,
                      const Pose &p_eye2head,
                      const Pose &p_head2head_prime);
} // namespace booster_vision