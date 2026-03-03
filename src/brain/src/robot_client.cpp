#include <cmath>
#include "brain.h"
#include <booster/robot/b1/b1_loco_api.hpp>
#include "robot_client.h"
#include "booster_interface/message_utils.hpp"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

void RobotClient::init(string robot_name)
{
    string suffix = robot_name.empty() ? "" : ("/" + robot_name);
    publisher = brain->create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopic" + suffix + "Req", 10);
}

int RobotClient::call(booster_interface::msg::BoosterApiReqMsg msg)
{
    std::string uuid = gen_uuid();
    auto message = booster_msgs::msg::RpcReqMsg();
    message.uuid = uuid;

    nlohmann::json req_header;
    req_header["api_id"] = msg.api_id;
    message.header = req_header.dump();
    message.body = msg.body;
    publisher->publish(message);
    return 0;
}

int RobotClient::moveHead(double pitch, double yaw)
{
    yaw = cap(yaw, brain->config->get_head_yaw_limit_left(), brain->config->get_head_yaw_limit_right());
    pitch = max(pitch, brain->config->get_head_pitch_limit_up());

    brain->log->debug("move_head", format("pitch: %.1f, yaw: %.1f", pitch, yaw));

    return call(booster_interface::CreateRotateHeadMsg(pitch, yaw));
}

int RobotClient::standUp()
{
    booster_interface::msg::BoosterApiReqMsg msg;
    msg.api_id = static_cast<int64_t>(booster::robot::b1::LocoApiId::kGetUp);
    nlohmann::json body;
    msg.body = body.dump();

    return call(msg);
}

int RobotClient::RLVisionKick(bool start)
{
    booster_interface::msg::BoosterApiReqMsg msg;
    msg.api_id = static_cast<int64_t>(booster::robot::b1::LocoApiId::kVisualKick);
    nlohmann::json body;
    body["start"] = start;  // Add start parameter to indicate whether to enter VisualKick mode
    msg.body = body.dump();
    std::cout << "RobotClient::RLVisionKick called with start=" << (start ? "true" : "false") << std::endl;
    return call(msg);
}

int RobotClient::robocupWalk()
{
    std::cout << "RobotClient::robocupWalk CreateChangeModeMsg called" << std::endl;
    return call(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kWalking));
}

int RobotClient::enterDamping()
{
    return call(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kDamping));
}

int RobotClient::waveHand(bool doWaveHand)
{
    return call(booster_interface::CreateWaveHandMsg(booster::robot::b1::HandIndex::kRightHand, doWaveHand ? booster::robot::b1::HandAction::kHandOpen : booster::robot::b1::HandAction::kHandClose));
}

int RobotClient::setVelocity(double x, double y, double theta)
{
    brain->log->log("RobotClient/setVelocity_in",
                    format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta));

    double minx = brain->config->get_min_vx();
    double miny = brain->config->get_min_vy();
    double mintheta =  brain->config->get_min_vtheta();
    if (fabs(x) < minx && fabs(x) > 1e-5)
        x = x > 0 ? minx : -minx;
    if (fabs(y) < miny && fabs(y) > 1e-5)
        y = y > 0 ? miny : -miny;
    if (fabs(theta) < mintheta && fabs(theta) > 1e-5)
        theta = theta > 0 ? mintheta : -mintheta;
    x = cap(x, brain->config->get_vx_limit(), -brain->config->get_vx_limit());
    y = cap(y, brain->config->get_vy_limit(), -brain->config->get_vy_limit());
    theta = cap(theta, brain->config->get_vtheta_limit(), -brain->config->get_vtheta_limit());
    
    // log simulated path based on velocity
    vector<Pose2D> path = {{0, 0, 0}}; // Coordinate system is based on the robot's position at 0,0, with the linear velocity direction as theta = 0
    double v = norm(x, y);
    double simStep = 1e-1; double simLength = 5.;
    for (int i = 0; i < simLength/simStep; i++) {
        double dt = simStep * (i + 1);
        if (fabs(theta) < 1e-3) path.push_back({v * dt, 0, 0});
        else path.push_back({v/theta * sin(theta * dt), v/theta * (1 - cos(theta * dt)), theta * dt});
    }
    vector<Pose2D> path_f = {};
    for (int i = 0; i < path.size(); i++) {
        auto p = trans(
            path[i].x, path[i].y, path[i].theta, 
            brain->data->robotPoseToField.x, 
            brain->data->robotPoseToField.y, 
            brain->data->robotPoseToField.theta + atan2(y, x), 
            "back"
        );
        path_f.push_back({p[0], -p[1], -p[2]});
    }
    
    _vx = x; _vy = y; _vtheta = theta; // remember last command. Can be approximately used as the current robot's velocity.
    _lastCmdTime = brain->get_clock()->now();
    if (fabs(_vx) > 1e-3 || fabs(_vy) > 1e-3 || fabs(_vtheta) > 1e-3) _lastNonZeroCmdTime = brain->get_clock()->now();
    brain->log->log("RobotClient/setVelocity_out",
        format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta));
    return call(booster_interface::CreateMoveMsg(x, y, theta));
}

int RobotClient::crabWalk(double angle, double speed) {
    double vxFactor = brain->config->get_vx_factor();   // Used to adjust vx, vx *= vxFactor, to compensate for the deviation between the speed parameters in the x and y directions and the actual speed ratio, making the movement direction accurate
    double yawOffset = brain->config->get_yaw_offset(); // Used to compensate for the deviation in the localization angle
    double vxLimit = brain->config->get_vx_limit();
    double vyLimit = brain->config->get_vy_limit();

    // Calculate velocity command
    double cmdAngle = angle + yawOffset;
    double vx = cos(cmdAngle) * speed * vxFactor;
    double vy = sin(cmdAngle) * speed;

    if (fabs(vy) > vyLimit) {
        vx *= vyLimit / fabs(vy);
        vy = vyLimit * fabs(vy) / vy;
    }

    return setVelocity(vx, vy, 0);
}



bool RobotClient::isStandingStill(double timeBuffer) {
    return fabs(_vx) < 1e-3 && fabs(_vy) < 1e-3 && fabs(_vtheta) < 1e-3 && brain->msecsSince(_lastNonZeroCmdTime) > timeBuffer;
}

int RobotClient::moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    Pose2D target_f, target_r; 
    static Pose2D target_temp_r; // for obstacle avoidance
    static rclcpp::Time timeEndTempTarget = brain->get_clock()->now(); // Before this time, execute temp_target instead of the actual target

    if (brain->get_clock()->now() < timeEndTempTarget) { // Execute temporary target for obstacle avoidance
        target_r = target_temp_r;
        vxLimit = 0.4;
        vyLimit = 0.4;
    } else { // Obstacle avoidance time has passed, execute the actual target
        target_f.x = tx;
        target_f.y = ty;
        target_f.theta = ttheta;
        target_r = brain->data->field2robot(target_f);
    }

    double targetAngle = atan2(target_r.y, target_r.x);
    double targetDist = norm(target_r.x, target_r.y);

    double vx = 0.0, vy = 0.0, vtheta = 0.0;

    // Already reached the target?
    if (
        (fabs(brain->data->robotPoseToField.x - target_f.x) < xTolerance) && (fabs(brain->data->robotPoseToField.y - target_f.y) < yTolerance) && (fabs(toPInPI(brain->data->robotPoseToField.theta - target_f.theta)) < thetaTolerance))
    {
        return setVelocity(0, 0, 0);
    }

    // Far away
    static double breakOscillate = 0.0;
    if (targetDist > longRangeThreshold - breakOscillate)
    {
        breakOscillate = 0.5;

        // Large angle, turn towards the target first
        if (fabs(targetAngle) > turnThreshold)
        {
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        } else {
            vx = cap(target_r.x, vxLimit, -vxLimit);
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);

        }
    } else { // Close range
        breakOscillate = 0.0;
        vx = cap(target_r.x, vxLimit, -vxLimit);
        vy = cap(target_r.y, vyLimit, -vyLimit);
        vtheta = cap(target_r.theta, vthetaLimit, -vthetaLimit);
    }

    // Obstacle avoidance
    if (avoidObstacle) {
        double etc = msecsToCollide(vx, vy, vtheta); // estimated time to collide
        double eta = norm(target_r.x, target_r.y) / max(1e-5, norm(vx, vy)) * 1000;// estimated time to arrive

        auto color = 0x00FF00FF;
        if (etc < 3000) color = 0xFF0000FF;
        else if (etc < 5000) color = 0x00FF00FF;
        else color = 0x00FF00FF;

        brain->log->log(
            "tick/time_to_collide",
            format("etc: %.0fms, color: 0x%08X", etc, color)
        );
        if (etc < min(eta, 3000.)) {
            std::srand(std::time(0));
            vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01; // step on spot, don't full stop
            vy = 0.0;
            vtheta = 0;

            auto rbtPose = brain->data->robotPoseToField;
            double theta0 = toPInPI(atan2(ty - rbtPose.y, tx - rbtPose.x) - rbtPose.theta);
            double theta1;
            double vx_temp, vy_temp;
            for (int i = 0; i < 9; i++) {
                theta1 = theta0 + M_PI / 9 * i;
                vx_temp = 0.4 * cos(theta1);
                vy_temp = 0.4 * sin(theta1);
                if (msecsToCollide(vx_temp, vy_temp, 0) > 3000) break;

                theta1 = theta0 - M_PI / 9 * i;
                vx_temp = 0.4 * cos(theta1);
                vy_temp = 0.4 * sin(theta1);
                if (msecsToCollide(vx_temp, vy_temp, 0) > 3000) break;
            }
            brain->log->debug("avoidObstacle", format(
                "theta1 = %.2f",
                theta1
            ));
            target_temp_r.x = 1 * cos(theta1);
            target_temp_r.y = 1 * sin(theta1);
            timeEndTempTarget = brain->get_clock()->now() + rclcpp::Duration(3.5, 0.0);
        }
        else if (etc < min(eta, 6000.)) {
            vtheta += 0.2;
        }
    }

    return setVelocity(vx, vy, vtheta);
}

int RobotClient::moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    static string mode = "longRange";
    static bool isBacking = false;
    static rclcpp::Time timeEndAvoid = brain->get_clock()->now();
    static double avoidDir = 1.0; // 1.0 for turn left, -1.0 for turn right
    const double SAFE_DIST = brain->config->get_safe_distance();
    const double AVOID_SECS = brain->config->get_avoid_secs();

    

    double range = norm(tx - brain->data->robotPoseToField.x, ty - brain->data->robotPoseToField.y);
    if (range > longRangeThreshold * (mode == "longRange" ? 0.9 : 1.0)) {
        mode = "longRange";
    } else {
        mode = "shortRange";
    }

    double vx, vy, vtheta;
    double tarDir = atan2(ty - brain->data->robotPoseToField.y, tx - brain->data->robotPoseToField.x);
    double faceDir = brain->data->robotPoseToField.theta;
    double tarDir_r = toPInPI(tarDir - faceDir); 
    auto now = brain->get_clock()->now();
    if (mode == "longRange") { 

        if (fabs(tarDir_r) > turnThreshold) { 
            vx = 0.0;
            vy = 0.0;
            vtheta = tarDir_r;
        } else { 
            vx = cap(range, vxLimit, -vxLimit);
            vy = 0.0;
            vtheta = tarDir_r;
        }
       
        if (avoidObstacle) {
            if (now < timeEndAvoid) { 
                double distToObstacle = brain->distToObstacle(0);
                if (distToObstacle < SAFE_DIST / 2.0) { 
                    isBacking = true;
                    timeEndAvoid = now + rclcpp::Duration(AVOID_SECS, 0.0);
                    avoidDir = brain->calcAvoidDir(tarDir_r, SAFE_DIST) > 0 ? 1.0 : -1.0;
                    vx = -0.2;
                    vy = avoidDir * 0.2;
                    vtheta = 0.0; 
                }else if (brain->distToObstacle(0) < SAFE_DIST + (isBacking ? 0.5 : 0.0)) { 
                    isBacking = false;
                    timeEndAvoid = now + rclcpp::Duration(AVOID_SECS, 0.0);
                    avoidDir = brain->calcAvoidDir(tarDir_r, SAFE_DIST) > 0 ? 1.0 : -1.0;
                    vx = 0.0;
                    vy = 0.0;
                    vtheta = avoidDir * brain->config->get_vtheta_limit();
                } else {
                    vx = vxLimit;
                    if (brain->distToObstacle(tarDir_r) < SAFE_DIST * 2) vxLimit *= 0.5;
                    vy = 0.0;
                    vtheta = 0.0;
                }
            } else {
                double distToObstacle = brain->distToObstacle(tarDir_r);
                if (distToObstacle < SAFE_DIST * 2) vxLimit = 0.5;
                if (distToObstacle < SAFE_DIST) {
                    timeEndAvoid = now + rclcpp::Duration(AVOID_SECS, 0.0);
                    avoidDir = brain->calcAvoidDir(tarDir_r, SAFE_DIST) > 0 ? 1.0 : -1.0;

                    vx = 0.0;
                    vy = 0.0;
                    vtheta = 0.0;
                }
            }

        }
    } else if (mode == "shortRange") { 
        

        vx = range * cos(tarDir_r);
        vy = range * sin(tarDir_r);
        vtheta = toPInPI(ttheta - faceDir); 
        if (fabs(vx) < xTolerance && fabs(vy) < yTolerance && fabs(vtheta) < thetaTolerance) {
            vx = 0.0;
            vy = 0.0;
            vtheta = 0.0;
        }


        if (avoidObstacle) { 
            double distToObstacle = brain->distToObstacle(tarDir_r);

            if (distToObstacle < SAFE_DIST) {
                vx = 0.0;
                vy = 0.0;
                vtheta = tarDir_r;
            }
        }
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit); 
    return setVelocity(vx, vy, vtheta);
}

int RobotClient::moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    static string mode = "longRange";
    static bool should_avoid_in_move = false;
    bool isFreekickStartPlacing = brain->isFreekickStartPlacing();

    const double SAFE_DIST = isFreekickStartPlacing ? brain->config->get_freekick_start_placing_safe_distance() : brain->config->get_safe_distance();
    const double AVOID_SECS = isFreekickStartPlacing ? brain->config->get_freekick_start_placing_avoid_secs() : brain->config->get_avoid_secs();

    double range = norm(tx - brain->data->robotPoseToField.x, ty - brain->data->robotPoseToField.y);
    if (range > longRangeThreshold * (mode == "longRange" ? 0.9 : 1.0)) { 
        mode = "longRange";
    } else {
        mode = "shortRange";
    }

    double vx, vy, vtheta;
    double tarDir = atan2(ty - brain->data->robotPoseToField.y, tx - brain->data->robotPoseToField.x);
    double faceDir = brain->data->robotPoseToField.theta;
    double tarDir_r = toPInPI(tarDir - faceDir); 
    if (mode == "longRange") { 
        if (fabs(tarDir_r) > turnThreshold) { 
            vx = 0.0;
            vy = 0.0;
            vtheta = tarDir_r;
        } else { 
            vx = cap(range, vxLimit, -vxLimit);
            vy = 0.0;
            vtheta = tarDir_r;
        }
       
        if (avoidObstacle) {
            double mostViableDir = 0; 
            double distToObstacleMostViable = brain->distToObstacle(0); 
            for (size_t i = 1; i <= 10; i++)
            {
                double tarDir_r_temp = toPInPI(0 + M_PI / 10 * i);
                double distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }

                tarDir_r_temp = toPInPI(0 - M_PI / 10 * i);
                distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }
            }
            if (should_avoid_in_move) {
                double distToObstacle = brain->distToObstacle(0);
                vx = 0;
                vy = (mostViableDir - tarDir_r > 0 ? 0.3 : -0.3); 
                vtheta = 0.0;
                if (distToObstacle < SAFE_DIST) {
                    should_avoid_in_move = true;
                } else {
                    vx = vxLimit;
                    if (brain->distToObstacle(tarDir_r) < SAFE_DIST * 2) vxLimit *= 0.5;
                    vy = 0.0;
                    vtheta = 0.0;
                    should_avoid_in_move = false; 
                }
            } else {
                double distToObstacle = brain->distToObstacle(tarDir_r);
                if (distToObstacle < SAFE_DIST * 3) vxLimit = 0.5;
                if (distToObstacle < SAFE_DIST) {
                    should_avoid_in_move = true;

                    vx = 0.0;
                    vy = 0.0;
                    vtheta = 0.0;
                }
            }

        }
    } else if (mode == "shortRange") { 

        vx = range * cos(tarDir_r);
        vy = range * sin(tarDir_r);
        vtheta = toPInPI(ttheta - faceDir); 
        if (fabs(vx) < xTolerance && fabs(vy) < yTolerance && fabs(vtheta) < thetaTolerance) {
            vx = 0.0;
            vy = 0.0;
            vtheta = 0.0;
        }


        if (avoidObstacle) { 
            double distToObstacle = brain->distToObstacle(tarDir_r);

            double mostViableDir = tarDir_r; 
            double distToObstacleMostViable = distToObstacle;
            for (size_t i = 1; i <= 10; i++)
            {
                double tarDir_r_temp = toPInPI(tarDir_r + M_PI / 10 * i);
                double distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }

                tarDir_r_temp = toPInPI(tarDir_r - M_PI / 10 * i);
                distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }
            }

            if (distToObstacle < SAFE_DIST) {
                vx = 0.0;
                vy = mostViableDir == tarDir_r ? 0.0 : (mostViableDir - tarDir_r > 0 ? 0.2 : -0.2); 
                vtheta = tarDir_r; 
            }
        }
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit); 
    return setVelocity(vx, vy, vtheta);
}

double RobotClient::msecsToCollide(double vx, double vy, double vtheta, double maxTime) {
    auto robots = brain->data->getRobots();

    if (robots.size() == 0) return maxTime; 
    if (vx < 1e-3 && vy < 1e-3) return maxTime; 

    // simplify the calculation by not considering vtheta for now. This is a conservative estimation.
    double x0 = brain->data->robotPoseToField.x;
    double y0 = brain->data->robotPoseToField.y;
    double robotTheta = brain->data->robotPoseToField.theta;

    Line path = {
        x0, y0,
        x0 + vx * cos(robotTheta) * maxTime / 1000, y0 + vy * sin(robotTheta) * maxTime / 1000
    };

    double minDist = 1e6;
    const double SAFE_DIST = 0.6;
    for (int i = 0; i < robots.size(); i++) {
        auto pose = robots[i].posToField;
        Point2D p = {pose.x, pose.y};
        double distToPath = pointMinDistToLine(p, path);
        brain->log->debug("msecsToCollide", format(
            "i = %d, rbt at (%.2f, %.2f), obstacle at (%.2f, %.2f), distToPath = %.2f",
            i, x0, y0, p.x, p.y, distToPath
        ));
        if (distToPath < SAFE_DIST) {
            Line line = {x0, y0, pose.x, pose.y};
            double angle = angleBetweenLines(line, path);
            double l = norm(pose.x - x0, pose.y - y0);
            double d = l*cos(angle) - sqrt(SAFE_DIST*SAFE_DIST - l*l*sin(angle)*sin(angle));
            brain->log->debug("msecsToCollide", format(
                "angle: %.2f, l: %.2f, d: %.2f",
                angle, l, d
            ));
            if (d < minDist) minDist = d;
        }
    }

    return min(maxTime, minDist / norm(vx, vy) * 1000);
}
