#include "booster_vision/calibration/calibration.h"

#include <ceres/ceres.h>

#include "booster_vision/calibration/optimizor.hpp"

namespace booster_vision {

double Compute2dError(
    const std::vector<std::vector<cv::Point3f>> &all_corners_3d,
    const std::vector<std::vector<cv::Point2f>> &all_corners_2d,
    const std::vector<Pose> p_head2bases,
    const Pose p_board2base,
    const Pose p_eye2head,
    const Intrinsics &intr) {
    double avg_error = 0;
    for (size_t i = 0; i < all_corners_2d.size(); i++) {
        double error = 0;
        Pose p_head2base = p_head2bases[i];
        Pose p_base2eye = (p_head2base * p_eye2head).inverse();
        for (size_t j = 0; j < all_corners_2d[i].size(); j++) {
            cv::Point3f corner_3d_wrt_eye = p_base2eye * p_board2base * all_corners_3d[i][j];
            cv::Point2f corner_2d_projected = intr.Project(corner_3d_wrt_eye);

            cv::Point2f diff = corner_2d_projected - all_corners_2d[i][j];
            error += cv::norm(diff);
        }
        error /= all_corners_2d[i].size();
        std::cout << "frame " << i << " error: " << error << std::endl;
        avg_error += error;
    }
    // cv::destroyWindow("reprojection");
    avg_error /= all_corners_2d.size();
    return avg_error;
}

void EyeInHandCalibration(double *reprojection_error,
                          Pose *p_cam2head_best,
                          Pose *p_board2base_best,
                          const std::vector<Pose> &p_board2cameras,
                          const std::vector<Pose> &p_head2bases,
                          const std::vector<std::vector<cv::Point3f>> &all_corners_3d,
                          const std::vector<std::vector<cv::Point2f>> &all_corners_2d,
                          const Intrinsics &intr,
                          bool optimization) {
    // prepare data
    std::vector<cv::Mat> r_head2bases;
    std::vector<cv::Mat> t_head2bases;
    std::vector<cv::Mat> r_board2cameras;
    std::vector<cv::Mat> t_board2cameras;

    for (int i = 0; i < p_head2bases.size(); i++) {
        Pose pose = p_head2bases[i];

        r_head2bases.push_back(pose.getRotationMatrix());
        t_head2bases.push_back(pose.getTranslationVecMatrix());

        r_board2cameras.push_back(p_board2cameras[i].getRotationMatrix());
        t_board2cameras.push_back(p_board2cameras[i].getTranslationVecMatrix());
    }

    *reprojection_error = std::numeric_limits<double>::max();
    for (int method = 0; method < 5; method++) {
        std::cout << "calibrate round: " << method << std::endl;
        cv::Mat r_eye2head;
        cv::Mat t_eye2head;
        try {
            cv::calibrateHandEye(r_head2bases, t_head2bases, r_board2cameras, t_board2cameras,
                                 r_eye2head, t_eye2head, static_cast<cv::HandEyeCalibrationMethod>(method));
        } catch (const cv::Exception &e) {
            std::cerr << "calibrateHandEye failed: " << e.what() << std::endl;
            continue;
        }

        Pose p_eye2head(r_eye2head, t_eye2head);
        Pose p_board2eye(r_board2cameras[0], t_board2cameras[0]);
        Pose p_board2base = p_head2bases[0] * p_eye2head * p_board2eye;

        double error = Compute2dError(all_corners_3d, all_corners_2d, p_head2bases, p_board2base, p_eye2head, intr);
        std::cout << "reprojection error: " << error << std::endl;

        if (error > 250) {
            std::cerr << "reprojection error too large, skip optimization" << std::endl;
            continue;
        }

        if (!optimization) {
            if (*reprojection_error > error) {
                *reprojection_error = error;
                *p_cam2head_best = p_eye2head;
                *p_board2base_best = p_board2base;
            }
        } else {
            std::cout << "optimizaiting ..." << std::endl;

            auto p_head2cam = p_eye2head.inverse();
            Eigen::Quaterniond q_head2cam = Eigen::Map<Eigen::Quaterniond>(p_head2cam.getQuaternionVec().data());
            Eigen::Vector3d t_head2cam = Eigen::Map<Eigen::Vector3f>(p_head2cam.getTranslationVec().data()).cast<double>();

            // board2base
            Eigen::Quaterniond q_board2base = Eigen::Map<Eigen::Quaterniond>(p_board2base.getQuaternionVec().data());
            Eigen::Vector3d t_board2base = Eigen::Map<Eigen::Vector3f>(p_board2base.getTranslationVec().data()).cast<double>();

            ceres::Problem problem;
            for (size_t i = 0; i < all_corners_2d.size(); i++) {
                for (size_t j = 0; j < all_corners_2d[i].size(); j++) {
                    cv::Point3f corner_3d = all_corners_3d[i][j];
                    cv::Point2f corner_2d = all_corners_2d[i][j];

                    // base2head
                    auto p_base2head = p_head2bases[i].inverse();
                    Eigen::Quaterniond q_base2head = Eigen::Map<Eigen::Quaterniond>(p_base2head.getQuaternionVec().data());
                    Eigen::Vector3d t_base2head = Eigen::Map<Eigen::Vector3f>(p_base2head.getTranslationVec().data()).cast<double>();

                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<ExtrinsicsOptimizer, 2, 4, 3, 4, 3>(
                        new ExtrinsicsOptimizer(corner_3d, corner_2d, q_base2head, t_base2head, intr));
                    problem.AddResidualBlock(cost_function, nullptr,
                                             q_head2cam.coeffs().data(), t_head2cam.data(),
                                             q_board2base.coeffs().data(), t_board2base.data());
                }
            }

            // set quaternion manifold for variables
            problem.SetManifold(q_head2cam.coeffs().data(), new ceres::EigenQuaternionManifold());
            problem.SetManifold(q_board2base.coeffs().data(), new ceres::EigenQuaternionManifold());

            ceres::Solver::Options options;
            // options.linear_solver_type = ceres::DENSE_QR;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.max_num_consecutive_invalid_steps = 30;
            options.max_num_iterations = 500;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            float point_count = all_corners_2d.size() * all_corners_2d[0].size();
            float initial_error = std::sqrt(2 * summary.initial_cost / point_count);
            float optimized_error = std::sqrt(2 * summary.final_cost / point_count);

            // convert optimized quaternion and translation to pose
            Eigen::Quaternionf q_board2base_opt = q_board2base.cast<float>();
            Eigen::Vector3f t_board2base_opt = t_board2base.cast<float>();
            Eigen::Quaternionf q_head2cam_opt = q_head2cam.cast<float>();
            Eigen::Vector3f t_head2cam_opt = t_head2cam.cast<float>();

            Pose p_board2base_opt(t_board2base_opt(0), t_board2base_opt(1), t_board2base_opt(2),
                                  q_board2base_opt.x(), q_board2base_opt.y(), q_board2base_opt.z(), q_board2base_opt.w());
            Pose p_head2cam_opt(t_head2cam_opt(0), t_head2cam_opt(1), t_head2cam_opt(2),
                                q_head2cam_opt.x(), q_head2cam_opt.y(), q_head2cam_opt.z(), q_head2cam_opt.w());

            std::cout << "extrinsics before optimization: \n"
                      << p_eye2head << std::endl
                      << "extrinsics after optimization: \n"
                      << p_head2cam_opt.inverse() << std::endl;

            std::cout << "board2base before optimization: \n"
                      << p_board2base << std::endl
                      << "board2base after optimization: \n"
                      << p_board2base_opt << std::endl;

            if (optimized_error < initial_error) {
                p_eye2head = p_head2cam_opt.inverse();
                p_board2base = p_board2base_opt;

                double error = Compute2dError(all_corners_3d, all_corners_2d, p_head2bases, p_board2base, p_eye2head, intr);
                std::cout << "reprojection error after optimizaiton: " << error << std::endl;

                if (*reprojection_error > optimized_error) {
                    *reprojection_error = error;
                    *p_cam2head_best = p_eye2head;
                    *p_board2base_best = p_board2base;
                }
            }
        }
    }
}

cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point3f &ray) {
    cv::Mat mat_obj_ray = (cv::Mat_<float>(3, 1) << ray.x, ray.y, ray.z);
    cv::Mat mat_rot = p_eye2base.getRotationMatrix();
    cv::Mat mat_trans = p_eye2base.toCVMat().col(3).rowRange(0, 3);

    cv::Mat mat_rot_obj_ray = mat_rot * mat_obj_ray;

    float scale = -mat_trans.at<float>(2, 0) / mat_rot_obj_ray.at<float>(2, 0);

    cv::Mat mat_position = mat_trans + scale * mat_rot_obj_ray;
    return cv::Point3f(mat_position.at<float>(0, 0), mat_position.at<float>(1, 0), mat_position.at<float>(2, 0));
}

double Compute3dError(
    const std::vector<cv::Point3f> &rays,
    const std::vector<cv::Point3f> &gts,
    const std::vector<Pose> &p_head2bases,
    const Pose &p_eye2head,
    const Pose &p_head2head_prime) {
    double avg_error = 0;
    for (size_t i = 0; i < rays.size(); i++) {
        auto p_eye2base = p_head2bases[i] * p_head2head_prime * p_eye2head;
        auto ray = rays[i];
        auto gt = gts[i];

        auto computed_val = CalculatePositionByIntersection(p_eye2base, ray);

        avg_error += cv::norm(computed_val - gt);
    }
    avg_error /= rays.size();
    return avg_error;
}

void EyeInHandOffsetCalibration(double *euclidian_error,
                                Pose *p_head2head_prime,
                                const Pose &p_eye2head,
                                const std::vector<Pose> &p_head2bases,
                                const std::vector<cv::Point3f> &computed_3d_rays,
                                const std::vector<cv::Point3f> &gt_3ds,
                                const bool without_translation) {
    // check input size

    Eigen::Quaterniond q_eye2head = Eigen::Map<Eigen::Quaterniond>(p_eye2head.getQuaternionVec().data());
    Eigen::Vector3d t_eye2head = Eigen::Map<Eigen::Vector3f>(p_eye2head.getTranslationVec().data()).cast<double>();

    Eigen::Quaterniond q_head2head_prime = Eigen::Map<Eigen::Quaterniond>(p_head2head_prime->getQuaternionVec().data());
    Eigen::Vector3d t_head2head_prime = Eigen::Map<Eigen::Vector3f>(p_head2head_prime->getTranslationVec().data()).cast<double>();

    ceres::Problem problem;
    int problem_cnt = 0;

    for (size_t i = 0; i < computed_3d_rays.size(); i++) {
        auto gt_3d = gt_3ds[i];
        auto computed_3d_ray = computed_3d_rays[i];
        auto p_head2base = p_head2bases[i];

        Eigen::Quaterniond q_head2base = Eigen::Map<Eigen::Quaterniond>(p_head2base.getQuaternionVec().data());
        Eigen::Vector3d t_head2base = Eigen::Map<Eigen::Vector3f>(p_head2base.getTranslationVec().data()).cast<double>();

        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<booster_vision::ExtrinsicsOffsetOptimizer, 2, 4, 3>(
            new booster_vision::ExtrinsicsOffsetOptimizer(gt_3d, computed_3d_ray,
                                                          q_head2base, t_head2base,
                                                          q_eye2head, t_eye2head));
        // problem.AddResidualBlock(cost_function, nullptr, q_head2head_prime.coeffs().data(), t_head2head_prime.data());
        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), q_head2head_prime.coeffs().data(), t_head2head_prime.data());
        problem_cnt++;
    }

    problem.SetManifold(q_head2head_prime.coeffs().data(), new ceres::EigenQuaternionManifold());
    problem.SetParameterLowerBound(t_head2head_prime.data(), 0, -0.1);
    problem.SetParameterUpperBound(t_head2head_prime.data(), 0, 0.1);
    problem.SetParameterLowerBound(t_head2head_prime.data(), 1, -0.1);
    problem.SetParameterUpperBound(t_head2head_prime.data(), 1, 0.1);
    problem.SetParameterLowerBound(t_head2head_prime.data(), 2, -0.1);
    problem.SetParameterUpperBound(t_head2head_prime.data(), 2, 0.1);
    if (without_translation) {
        problem.SetParameterBlockConstant(t_head2head_prime.data());
        std::cout << "optmize offset with constant translation constrain" << std::endl;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_consecutive_invalid_steps = 30;
    options.max_num_iterations = 500;
    options.function_tolerance = 1e-8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    float initial_error = std::sqrt(2 * summary.initial_cost / problem_cnt);
    std::cout << "3d error before optimization: " << initial_error << std::endl;
    float optimized_error = std::sqrt(2 * summary.final_cost / problem_cnt);
    std::cout << "3d error after optimization: " << optimized_error << std::endl;

    Eigen::Quaternionf q_head2head_prime_opt = q_head2head_prime.cast<float>();
    Eigen::Vector3f t_head2head_prime_opt = t_head2head_prime.cast<float>();

    booster_vision::Pose p_head2head_prime_opt(t_head2head_prime_opt(0), t_head2head_prime_opt(1), t_head2head_prime_opt(2),
                                               q_head2head_prime_opt.x(), q_head2head_prime_opt.y(), q_head2head_prime_opt.z(), q_head2head_prime_opt.w());

    std::cout << "optimized p_head2head_prime\n"
              << p_head2head_prime_opt << std::endl;
    auto rpy_opt = p_head2head_prime_opt.getEulerAnglesVec();
    std::cout << "optimized p_head2head_prime in rpy: " << rpy_opt[0] * 180 / M_PI << ", " << rpy_opt[1] * 180 / M_PI << ", " << rpy_opt[2] * 180 / M_PI << std::endl;
    *p_head2head_prime = p_head2head_prime_opt;
}

} // namespace booster_vision