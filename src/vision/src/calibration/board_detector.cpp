#include "booster_vision/calibration/board_detector.h"

#include <opencv2/opencv.hpp>

namespace booster_vision {

Pose BoardDetector::getBoardPose() {
    cv::Mat rvec;
    cv::Mat tvec;
    bool res = cv::solvePnP(board_points_, board_uvs_, intr_.get_intrinsics_matrix(), intr_.distortion_coeffs, rvec, tvec);
    if (!res) {
        return Pose();
    }
    return Pose(rvec, tvec);
}

std::vector<cv::Point2f> BoardDetector::getBoardUVsSubpixel() {
    if (board_uvs_.empty()) {
        return {};
    }
    cv::cornerSubPix(gray_, board_uvs_, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    return board_uvs_;
}

cv::Mat BoardDetector::getBoardMask(const cv::Mat &img) {
    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());
    if (board_uvs_.empty() || board_uvs_.size() != board_size_.width * board_size_.height) {
        return mask;
    }
    std::vector<cv::Point> board_corners;
    board_corners.push_back(board_uvs_[0]);
    board_corners.push_back(board_uvs_[board_size_.width - 1]);
    board_corners.push_back(board_uvs_[board_size_.width * board_size_.height - 1]);
    board_corners.push_back(board_uvs_[board_size_.width * (board_size_.height - 1)]);
    std::vector<std::vector<cv::Point>> contour;
    contour.push_back(board_corners);

    cv::fillPoly(mask, contour, cv::Scalar(0, 255, 0));
    return mask;
}

bool BoardDetector::DetectBoard(const cv::Mat &img) {
    board_uvs_.clear();
    cv::cvtColor(img, gray_, cv::COLOR_BGR2GRAY);
    // bool found = cv::findChessboardCorners(gray_, board_size_, board_uvs_, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    bool found = cv::findChessboardCorners(gray_, board_size_, board_uvs_, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    return found;
}

} // namespace booster_vision