#pragma once
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"

namespace booster_vision {

void VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

void VisualizePointCloudandPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::vector<float> &coeffs, float x, float y, float z);

void VisualizePointCloudSphere(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::vector<std::vector<float>> &sphere_coeffs);

void CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &depth_image, const cv::Mat &rgb_image, const cv::Rect &bbox,

                      const booster_vision::Intrinsics &intrinsics);
void CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &depth_image, const cv::Mat &rgb_image,
                      const booster_vision::Intrinsics &intrinsics);

void DownSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &processed_cloud, const float leaf_size,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

void PointCloudNoiseRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &processed_cloud, const int neighbour_count,
                            const float multiplier,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

void ClusterPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clustered_clouds, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       const float cluster_distance_threshold);

void SphereFitting(std::vector<float> &sphere, float &confidence, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   const float &dist_threshold, const float &radius_threshold);

void PlaneFitting(std::vector<float> &plane, float &confidence,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const float &dist_threshold);

} // namespace booster_vision
