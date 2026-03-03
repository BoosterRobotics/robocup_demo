#include "booster_vision/pose_estimator/pose_estimator.h"

#include "booster_vision/base/misc_utils.hpp"
#include "booster_vision/base/pointcloud_process.h"

namespace booster_vision {

cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point2f target_uv, const Intrinsics &intr) {
    cv::Point3f normalized_point3d = intr.BackProject(target_uv);

    cv::Mat mat_obj_ray = (cv::Mat_<float>(3, 1) << normalized_point3d.x, normalized_point3d.y, normalized_point3d.z);
    cv::Mat mat_rot = p_eye2base.getRotationMatrix();
    cv::Mat mat_trans = p_eye2base.toCVMat().col(3).rowRange(0, 3);

    cv::Mat mat_rot_obj_ray = mat_rot * mat_obj_ray;

    float scale = -mat_trans.at<float>(2, 0) / mat_rot_obj_ray.at<float>(2, 0);

    cv::Mat mat_position = mat_trans + scale * mat_rot_obj_ray;
    return cv::Point3f(mat_position.at<float>(0, 0), mat_position.at<float>(1, 0), mat_position.at<float>(2, 0));
}

Pose PoseEstimator::EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) {
    // TODO(GW): add modification for cross class
    auto bbox = detection.bbox;
    cv::Point2f target_uv = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    cv::Point3f target_xyz = CalculatePositionByIntersection(p_eye2base, target_uv, intr_);
    return Pose(target_xyz.x, target_xyz.y, target_xyz.z, 0, 0, 0);
}

Pose PoseEstimator::EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) {
    return Pose();
}

void BallPoseEstimator::Init(const YAML::Node &node) {
    use_depth_ = as_or<bool>(node["use_depth"], false);
    radius_ = as_or<float>(node["radius"], 0.109);
    downsample_leaf_size_ = as_or<float>(node["down_sample_leaf_size"], 0.01);
    cluster_distance_threshold_ = as_or<float>(node["cluster_distance_threshold"], 0.01);
    fitting_distance_threshold_ = as_or<float>(node["fitting_distance_threshold"], 0.01);
    minimum_cluster_size_ = as_or<int>(node["minimum_cluster_size"], 150);
    filter_distance_ = as_or<float>(node["filter_distance"], 1.0);
    check_ball_height_ = as_or<bool>(node["check_ball_height"], false);
    std::cout << "filter_distance: " << filter_distance_ << std::endl;
}

Pose BallPoseEstimator::EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) {
    // TODO(GW): add modification for cross class
    auto bbox = detection.bbox;
    cv::Point2f target_uv = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height);
    cv::Point3f target_xyz = CalculatePositionByIntersection(p_eye2base, target_uv, intr_);
    return Pose(target_xyz.x, target_xyz.y, target_xyz.z, 0, 0, 0);
}

Pose BallPoseEstimator::EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) {
    if (!use_depth_ || depth.empty()) return Pose();

    auto pose = EstimateByColor(p_eye2base, detection, cv::Mat());
    if (cv::norm(pose.getTranslationVec()) > filter_distance_) return pose;
    std::cout << "ball distance by color: " << cv::norm(pose.getTranslationVec()) << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    CreatePointCloud(cloud, depth, rgb, detection.bbox, intr_);
    if (cloud->points.size() < 100) return Pose();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    DownSamplePointCloud(downsampled_cloud, downsample_leaf_size_, cloud);
    if (downsampled_cloud->points.size() < 100) return Pose();

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clustered_clouds;
    ClusterPointCloud(clustered_clouds, downsampled_cloud, cluster_distance_threshold_);

    if (clustered_clouds.empty()) return Pose();

    for (const auto &cluster : clustered_clouds) {
        if (cluster->size() < minimum_cluster_size_) continue;

        std::vector<float> sphere;
        float confidence;
        SphereFitting(sphere, confidence, cluster, fitting_distance_threshold_, radius_);

        if (confidence > 0.5) {
            std::cout << "ball z in robot frame by color: " << pose.getTranslationVec()[2] << std::endl;
            pose = p_eye2base * Pose(sphere[0], sphere[1], sphere[2], 0, 0, 0);
            std::cout << "ball z in robot frame: " << pose.getTranslationVec()[2] << std::endl;
            if (check_ball_height_ && pose.getTranslationVec()[2] < -0.03) {
                continue;
            }
            return pose;
        }
    }
    return Pose();
}

void HumanLikePoseEstimator::Init(const YAML::Node &node) {
    use_depth_ = as_or<bool>(node["use_depth"], false);
    downsample_leaf_size_ = as_or<float>(node["downsample_leaf_size"], 0.01);
    statistic_outlier_multiplier_ = as_or<float>(node["statistic_outlier_multiplier"], 0.01);
    fitting_distance_threshold_ = as_or<float>(node["fitting_distance_threshold"], 0.01);
}

Pose HumanLikePoseEstimator::EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) {
    auto bbox = detection.bbox;
    cv::Point2f target_uv = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height);
    cv::Point3f target_xyz = CalculatePositionByIntersection(p_eye2base, target_uv, intr_);
    return Pose(target_xyz.x, target_xyz.y, target_xyz.z, 0, 0, 0);
}

Pose HumanLikePoseEstimator::EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) {
    if (!use_depth_ || depth.empty()) return Pose();

    auto pose = EstimateByColor(p_eye2base, detection, cv::Mat());
    if (pose.getTranslationVec()[0] > 3) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    CreatePointCloud(cloud, depth, rgb, detection.bbox, intr_);
    if (cloud->points.size() < 100) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    DownSamplePointCloud(downsampled_cloud, downsample_leaf_size_, cloud);
    if (downsampled_cloud->points.size() < 100) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudNoiseRemoval(processed_cloud, 50, statistic_outlier_multiplier_, downsampled_cloud);
    if (processed_cloud->points.size() < 100) return pose;

    std::vector<float> plane_coeffs;
    float confidence;
    PlaneFitting(plane_coeffs, confidence, processed_cloud, fitting_distance_threshold_);

    // TODO(GW): add plane fitting res check

    // compute plane ray intersection
    auto bbox = detection.bbox;
    cv::Point2f target_uv = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    cv::Point3f normalized_point3d = intr_.BackProject(target_uv);

    float a = plane_coeffs[0];
    float b = plane_coeffs[1];
    float c = plane_coeffs[2];
    float d = plane_coeffs[3];
    float scale = -d / (normalized_point3d.x * a + normalized_point3d.y * b + c);

    pose = p_eye2base * Pose(normalized_point3d.x * scale, normalized_point3d.y * scale, normalized_point3d.z * scale, 0, 0, 0);
    return pose;
}

void FieldMarkerPoseEstimator::Init(const YAML::Node &node) {
    refine_ = as_or<bool>(node["refine"], false);
}

std::vector<float> getLineCoefficients(const cv::Vec4f &seg) {
    cv::Point2f p1(seg[0], seg[1]);
    cv::Point2f p2(seg[2], seg[3]);

    float A = p2.y - p1.y; // y2 - y1
    float B = p1.x - p2.x; // x1 - x2
    float C = p2.x * p1.y - p1.x * p2.y;
    return {A, B, C};
}

bool CanBeMerged(const cv::Vec4f &l1, const cv::Vec4f &l2, float threshold) {
    float tan1 = atan2f(l1[3] - l1[1], l1[2] - l1[0]);
    float tan2 = atan2f(l2[3] - l2[1], l2[2] - l2[0]);

    if (abs(tan2 - tan1) > CV_PI / 36) {
        return false;
    }

    auto coeffs1 = getLineCoefficients(l1);
    auto coeffs2 = getLineCoefficients(l2);

    auto predict_y = [](const std::vector<float> &coeffs, float x) {
        return -(coeffs[0] * x + coeffs[2]) / coeffs[1];
    };

    float diff1 = abs(l1[1] - predict_y(coeffs2, l1[0])) + abs(l1[3] - predict_y(coeffs2, l1[2]));
    float diff2 = abs(l2[1] - predict_y(coeffs1, l2[0])) + abs(l2[3] - predict_y(coeffs1, l2[2]));

    return (diff1 + diff2) < threshold;
}

void FitLineRANSAC(const std::vector<cv::Vec4i> &line_segments, const float &distance_threshold,
                   const int &max_trails, cv::Vec4f &merged_seg, std::vector<float> &coeffs) {
    std::vector<cv::Point2f> point_vec;
    for (const auto &seg : line_segments) {
        point_vec.emplace_back(seg[0], seg[1]);
        point_vec.emplace_back(seg[2], seg[3]);
    }

    cv::RNG rng;

    unsigned int n = point_vec.size();
    unsigned int best_inlier_count = 0;
    cv::Vec4f best_line_seg = {0, 0, 0, 0};

    for (int k = 0; k < max_trails; k++) {
        int i1 = 0;
        int i2 = 0;
        while (i1 == i2) {
            i1 = rng(n);
            i2 = rng(n);
        }

        const cv::Point2f &p1 = point_vec[i1];
        const cv::Point2f &p2 = point_vec[i2];

        cv::Point2f dp = p2 - p1;
        dp *= 1. / cv::norm(dp);

        unsigned int inlier_count = 0;
        for (unsigned int i = 0; i < n; i++) {
            cv::Point2f v = point_vec[i] - p1;
            float d = v.y * dp.x - v.x * dp.y;
            if (fabs(d) < distance_threshold) {
                inlier_count++;
            }
        }
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_line_seg = {p1.x, p1.y, p2.x, p2.y};
        }
    }
    // sort point_vec by x
    std::sort(point_vec.begin(), point_vec.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.x < b.x;
    });

    coeffs = getLineCoefficients(best_line_seg);
    float x1 = point_vec[0].x;
    float x2 = point_vec[n - 1].x;

    float y1 = -(1 / coeffs[1]) * (coeffs[0] * x1 + coeffs[2]);
    float y2 = -(1 / coeffs[1]) * (coeffs[0] * x2 + coeffs[2]);
    merged_seg = {x1, y1, x2, y2};
}

void get_colliner_idx(const int &idx, std::map<int, std::vector<int>> &idx_map, std::vector<int> &idx_vec) {
    if (idx_map.find(idx) == idx_map.end()) {
        return;
    } else {
        idx_vec.insert(idx_vec.end(), idx_map[idx].begin(), idx_map[idx].end());
        for (const auto &sub_idx : idx_map[idx]) {
            get_colliner_idx(sub_idx, idx_map, idx_vec);
        }
    }
}

cv::Point2f RefineFieldMarkerCenterDetection(const cv::Mat &img, const cv::Rect &bbox, float scale_factor) {
    cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);

    // crop image
    int extended_len = std::min(bbox.width, bbox.height) / 2;
    int img_w = img.cols;
    int img_h = img.rows;

    int new_x = std::max(0, bbox.x - extended_len);
    int new_y = std::max(0, bbox.y - extended_len);
    int new_x_max = std::min(img_w - 1, bbox.x + bbox.width + extended_len);
    int new_y_max = std::min(img_h - 1, bbox.y + bbox.height + extended_len);
    int new_width = new_x_max - new_x;
    int new_height = new_y_max - new_y;

    cv::Rect extended_area_bbox(new_x, new_y, new_width, new_height);
    cv::Mat extended_area_img = img(extended_area_bbox).clone();

    cv::Point2f center_in_extended_area(bbox.x - new_x + bbox.width / 2.0f, bbox.y - new_y + bbox.height / 2.0f);

    // compute edge
    cv::Mat gray, edges;
    cv::cvtColor(extended_area_img, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 40, 150, 3);

    // cv::imshow("Canny Edges", edges);
    // cv::waitKey(0);

    // detect line segments using HoughLinesP
    std::vector<cv::Vec4i> line_segments;
    int min_len = std::min(extended_area_img.cols, extended_area_img.rows) * 1 / 4;
    int line_gap = std::max(5, min_len / 4);
    cv::HoughLinesP(edges, line_segments, 1, CV_PI / 360, 15, min_len, line_gap);

    auto draw_segments = [](cv::Mat &img, std::vector<cv::Vec4f> segments) {
        cv::Mat crop_bk = img.clone();
        for (auto seg : segments) {
            cv::line(crop_bk, cv::Point(int(seg[0]), int(seg[1])), cv::Point(int(seg[2]), int(seg[3])), (255, 0, 255), 1);
        }
        return crop_bk;
    };

    // filter line segment by distance
    std::vector<cv::Vec4f> filtered_line_segments;
    float point_2_line_distance_threshold = std::max(20 * scale_factor, 10.0f);
    for (auto &seg : line_segments) {
        std::vector coeffs = getLineCoefficients(seg);
        float center_2_line_distance = abs(coeffs[0] * center_in_extended_area.x + coeffs[1] * center_in_extended_area.y + coeffs[2])
                                       / std::sqrt(coeffs[0] * coeffs[0] + coeffs[1] * coeffs[1]);

        // std::cout << "seg: " << seg << std::endl;
        // std::cout << "center2line_distance: " << center_2_line_distance << std::endl;
        // auto show_img = draw_segments(extended_area_img, {seg});
        // cv::circle(show_img, center_in_extended_area, 3, (255, 0 , 0));
        // cv::imshow("line seg", show_img);
        // cv::waitKey(0);

        if (center_2_line_distance > point_2_line_distance_threshold) {
            continue;
        }
        filtered_line_segments.push_back(seg);
    }

    // auto crop_bk = draw_segments(extended_area_img, filtered_line_segments);
    // std::cout << "center in croppped image: " << center_in_extended_area << std::endl;
    // cv::circle(crop_bk, center_in_extended_area, 3, (255, 0 , 0));
    // cv::imshow("filtered line segments", crop_bk);
    // cv::waitKey(0);

    // get colliner segment indexes

    using MergedIdxMap = std::map<int, std::vector<int>>;

    MergedIdxMap colliner_idx_map;
    std::vector<bool> colliner_status(filtered_line_segments.size(), false);
    std::vector<bool> processed_status(filtered_line_segments.size(), false);
    std::vector<std::vector<int>> colliner_idx_vec;

    float merge_distance_thshold = 15 * scale_factor;
    for (size_t i = 0; i < filtered_line_segments.size(); i++) {
        for (size_t j = i + 1; j < filtered_line_segments.size(); j++) {
            if (CanBeMerged(filtered_line_segments[i], filtered_line_segments[j], merge_distance_thshold)) {
                colliner_status[i] = true;
                colliner_status[j] = true;

                colliner_idx_map[i].push_back(j);
            }
        }
    }

    for (size_t i = 0; i < filtered_line_segments.size(); i++) {
        std::vector<int> idx_vec = {static_cast<int>(i)};
        if (!colliner_status[i]) {
            colliner_idx_vec.emplace_back(idx_vec);
            processed_status[i] = true;
        }
        if (colliner_status[i] & !processed_status[i]) {
            get_colliner_idx(i, colliner_idx_map, idx_vec);
            for (const auto &idx : idx_vec) {
                processed_status[idx] = true;
            }
            std::set<int> idx_set(idx_vec.begin(), idx_vec.end());
            idx_vec.clear();
            for (const auto &idx : idx_set) {
                idx_vec.push_back(idx);
            }

            colliner_idx_vec.push_back(idx_vec);
        }
    }

    if (colliner_idx_vec.size() < 4) {
        std::cout << "found marker center failed: no enough lines extracted " << std::endl;
        return center;
    }

    // merge colliner line segments
    std::vector<cv::Vec4f> merged_segments;
    std::vector<std::vector<float>> merged_line_coeffs;
    for (const auto &idx_vec : colliner_idx_vec) {
        if (idx_vec.size() == 1) {
            auto seg = filtered_line_segments[idx_vec[0]];
            merged_segments.emplace_back(seg[0], seg[1], seg[2], seg[3]);
            merged_line_coeffs.push_back(getLineCoefficients(seg));
        } else {
            std::vector<cv::Vec4i> line_segments;
            cv::Vec4f seg;
            std::vector<float> coeffs;
            for (const auto &idx : idx_vec) {
                line_segments.push_back(filtered_line_segments[idx]);
            }
            FitLineRANSAC(line_segments, 5, 40, seg, coeffs);
            merged_segments.push_back(seg);
            merged_line_coeffs.push_back(coeffs);
        }
    }

    // for (auto seg : merged_segments) {
    //     cv::line(extended_area_img, cv::Point(int(seg[0]), int(seg[1])), cv::Point(int(seg[2]), int(seg[3])), (255, 0, 255), 1);
    // }
    // cv::imshow("final line segments", extended_area_img);
    // cv::waitKey(0);

    // compute line intersections
    std::vector<cv::Point2f> intersections;
    for (size_t i = 0; i < merged_line_coeffs.size(); i++) {
        for (size_t j = i + 1; j < merged_line_coeffs.size(); j++) {
            double A1 = merged_line_coeffs[i][0];
            double B1 = merged_line_coeffs[i][1];
            double C1 = merged_line_coeffs[i][2];

            double A2 = merged_line_coeffs[j][0];
            double B2 = merged_line_coeffs[j][1];
            double C2 = merged_line_coeffs[j][2];
            double det = A1 * B2 - A2 * B1;

            if (det == 0) continue; // Lines are parallel

            double x = (-B2 * C1 + B1 * C2) / det;
            double y = (-A1 * C2 + A2 * C1) / det;

            if (x < 0 || y < 0 || x > extended_area_img.cols || y > extended_area_img.rows) continue;

            double dist = cv::norm(center_in_extended_area - cv::Point2f(x, y));

            if (dist <= 45) {
                intersections.push_back(cv::Point2f(x, y));
            }
        }
    }

    if (intersections.empty()) {
        std::cout << "found marker center failed: no intersections found " << std::endl;
        return center;
    }

    // compute center
    float center_x = 0;
    float center_y = 0;

    for (const auto &intersection : intersections) {
        center_x += intersection.x;
        center_y += intersection.y;
    }
    center_x /= intersections.size();
    center_y /= intersections.size();
    center_x += extended_area_bbox.x;
    center_y += extended_area_bbox.y;

    return {center_x, center_y};
}

Pose FieldMarkerPoseEstimator::EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &color) {
    auto pose = PoseEstimator::EstimateByColor(p_eye2base, detection, color);
    if (!refine_) return pose;

    float scale_factor = 2.0 / pose.getTranslationVec()[0];
    cv::Point2f target_uv = RefineFieldMarkerCenterDetection(color, detection.bbox, scale_factor);
    cv::Point3f target_xyz = CalculatePositionByIntersection(p_eye2base, target_uv, intr_);

    return Pose(target_xyz.x, target_xyz.y, target_xyz.z, 0, 0, 0);
}

void FitLineRANSAC(std::vector<cv::Point3f> &best_line, unsigned int &best_inlier_count, float &best_accu_distance,
                   const std::vector<cv::Point3f> &points, const float &dist_threshold, const int &max_trails) {
    cv::RNG rng;
    unsigned int n = points.size();

    best_inlier_count = 0;
    best_accu_distance = 0;
    best_line = {cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0)};

    for (int k = 0; k < max_trails; k++) {
        int i1 = 0;
        int i2 = 0;
        while (i1 == i2) {
            i1 = rng(n);
            i2 = rng(n);
        }

        const cv::Point3f &p1 = points[i1];
        const cv::Point3f &p2 = points[i2];

        cv::Point3f dp = p2 - p1;
        dp *= 1. / cv::norm(dp);

        unsigned int inlier_count = 0;
        float accu_distance = 0;
        for (unsigned int i = 0; i < n; i++) {
            cv::Point3f v = points[i] - p1;
            float d = std::fabs(v.y * dp.x - v.x * dp.y);
            accu_distance += d;
            if (fabs(d) < dist_threshold) {
                inlier_count++;
            }
        }
        if (inlier_count > best_inlier_count) {
            best_accu_distance = accu_distance;
            best_inlier_count = inlier_count;
            best_line[0] = p1;
            best_line[1] = p2;
        }
    }
    best_accu_distance /= n;
}

std::vector<FieldLineSegment> FitFieldLineSegments(const Pose &p_eye2base,
                                                   const Intrinsics &intr,
                                                   const std::vector<std::vector<cv::Point>> &contours,
                                                   const int& area_threshold) {
    std::vector<FieldLineSegment> field_line_segments;
    for (const auto &contour : contours) {
        FieldLineSegment seg;
        seg.contour_2d_points = contour;
        seg.area = cv::contourArea(contour);
        for (const auto &point : contour) {
            cv::Point3f point_3d = CalculatePositionByIntersection(p_eye2base, point, intr);
            seg.contour_3d_points.push_back(point_3d);
        }
        field_line_segments.push_back(seg);
    }

    cv::Mat t_base2eye = p_eye2base.toCVMat().inv();
    Pose p_base2eye = p_eye2base.inverse();

    std::vector<FieldLineSegment> ret;
    for (auto &seg : field_line_segments) {
        if (seg.area < area_threshold)
            continue;
        FitLineRANSAC(seg.line_model, seg.inlier_count, seg.accu_distance, seg.contour_3d_points, 0.07, 100);
        // std::cout << "===================================" << std::endl;
        cv::Point3f average_point = {0, 0, 0};
        for (int i = 0; i < seg.contour_3d_points.size(); i++) {
            // std::cout << "uv point: " << seg.contour_2d_points[i] << ", 3d point in robot frame: " << seg.contour_3d_points[i] << std::endl;
            average_point.x += seg.contour_3d_points[i].x;
            average_point.y += seg.contour_3d_points[i].y;
            average_point.z += seg.contour_3d_points[i].z;
        }
        average_point.x /= seg.contour_3d_points.size();
        average_point.y /= seg.contour_3d_points.size();
        average_point.z /= seg.contour_3d_points.size();

        auto p1 = seg.line_model[0];
        auto p2 = seg.line_model[1];
        auto dp = p2 - p1;

        std::vector<cv::Point3f> contour_3d_points = seg.contour_3d_points;
        std::sort(contour_3d_points.begin(), contour_3d_points.end(), [&](const cv::Point3f &a, const cv::Point3f &b) {
            return dp.dot(a - average_point) < dp.dot(b - average_point);
        });

        seg.end_points_3d.clear();
        seg.end_points_3d.push_back(contour_3d_points[0]);
        seg.end_points_3d.push_back(contour_3d_points[contour_3d_points.size() - 1]);

        seg.end_points_2d.clear();
        seg.end_points_2d.push_back(intr.Project(p_base2eye * seg.end_points_3d[0]));
        seg.end_points_2d.push_back(intr.Project(p_base2eye * seg.end_points_3d[1]));

        // std::cout << "start point in world: " << seg.end_points_3d[0] << std::endl;
        // std::cout << "end   point in world: " << seg.end_points_3d[1] << std::endl;
        // std::cout << "start point in uv: " << seg.end_points_2d[0] << std::endl;
        // std::cout << "end   point in uv: " << seg.end_points_2d[1] << std::endl;
        ret.push_back(seg);
    }
    return ret;
}

cv::Mat DrawFieldLineSegments(cv::Mat &img, const std::vector<FieldLineSegment> &segs) {
    cv::Mat display = img.clone();
    for (const auto &seg : segs) {
        std::vector<std::vector<cv::Point>> contour = {seg.contour_2d_points};
        cv::drawContours(display, contour, -1, cv::Scalar(255, 255, 255), 2);
        cv::circle(display, seg.end_points_2d[0], 5, (255, 0, 255), -1);
        cv::circle(display, seg.end_points_2d[1], 5, (255, 0, 255), -1);
        cv::line(display, seg.end_points_2d[0], seg.end_points_2d[1], (0, 0, 0), 2);
    }
    return display;
}

} // namespace booster_vision
