#include <iostream>
#include <array>
#include <chrono>
#include <string>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "assist_strategy_policy.h"
#include "brain.h"
#include "fallen_robot_avoidance_policy.h"
#include "fall_recovery_policy.h"
#include "robot_obstacle_policy.h"
#include "utils/print.h"
#include "utils/math.h"
#include "utils/misc.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using std::placeholders::_1;

#define SUB_STATE_QUEUE_SIZE 1

#ifndef BRAIN_GIT_DESCRIBE
#define BRAIN_GIT_DESCRIBE "unknown"
#endif

namespace {

OdomDiagnosticPose toDiagnosticPose(const Pose2D &pose)
{
    return {pose.x, pose.y, pose.theta};
}

int64_t steadyTimeNs(std::chrono::steady_clock::time_point timePoint)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        timePoint.time_since_epoch()).count();
}

int64_t systemTimeNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

bool hostIsBigEndian()
{
    const uint16_t value = 0x0102;
    return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
}

uint16_t readDepthU16(const uint8_t *ptr, bool swapBytes)
{
    uint16_t value;
    std::memcpy(&value, ptr, sizeof(value));
    if (swapBytes) value = static_cast<uint16_t>((value >> 8) | (value << 8));
    return value;
}

float readDepthF32(const uint8_t *ptr, bool swapBytes)
{
    uint32_t bits;
    std::memcpy(&bits, ptr, sizeof(bits));
    if (swapBytes) {
        bits = ((bits & 0x000000FFu) << 24) |
               ((bits & 0x0000FF00u) << 8) |
               ((bits & 0x00FF0000u) >> 8) |
               ((bits & 0xFF000000u) >> 24);
    }
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool decodeDepthImage(const sensor_msgs::msg::Image &msg,
                      double scale16U, double scale32F,
                      cv::Mat &depthMeters, std::string &error)
{
    const bool is16U = msg.encoding == "16UC1" || msg.encoding == "mono16";
    const bool is32F = msg.encoding == "32FC1";
    if (!is16U && !is32F) {
        error = "unsupported encoding " + msg.encoding;
        return false;
    }

    const size_t bytesPerPixel = is16U ? sizeof(uint16_t) : sizeof(float);
    const size_t rowBytes = static_cast<size_t>(msg.width) * bytesPerPixel;
    const size_t step = msg.step == 0 ? rowBytes : static_cast<size_t>(msg.step);
    if (step < rowBytes || msg.data.size() < step * static_cast<size_t>(msg.height)) {
        error = "invalid step/data size";
        return false;
    }
    if (!std::isfinite(scale16U) || scale16U <= 0.0 ||
        !std::isfinite(scale32F) || scale32F <= 0.0) {
        error = "invalid depth scale";
        return false;
    }

    depthMeters = cv::Mat(static_cast<int>(msg.height), static_cast<int>(msg.width), CV_32FC1);
    const bool swapBytes = msg.is_bigendian != hostIsBigEndian();
    const double scale = is16U ? scale16U : scale32F;
    for (size_t row = 0; row < msg.height; ++row) {
        const uint8_t *src = msg.data.data() + row * step;
        float *dst = depthMeters.ptr<float>(static_cast<int>(row));
        for (size_t col = 0; col < msg.width; ++col) {
            const uint8_t *pixel = src + col * bytesPerPixel;
            const double raw = is16U
                ? static_cast<double>(readDepthU16(pixel, swapBytes))
                : static_cast<double>(readDepthF32(pixel, swapBytes));
            dst[col] = static_cast<float>(raw * scale);
        }
    }
    return true;
}

bool extractJsonIntField(const std::string &json, const std::string &field, int &value)
{
    const std::string key = "\"" + field + "\"";
    size_t pos = json.find(key);
    if (pos == std::string::npos) return false;
    pos = json.find(':', pos + key.size());
    if (pos == std::string::npos) return false;

    ++pos;
    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) ++pos;
    if (pos < json.size() && json[pos] == '"') ++pos;

    const size_t start = pos;
    if (pos < json.size() && (json[pos] == '-' || json[pos] == '+')) ++pos;
    while (pos < json.size() && std::isdigit(static_cast<unsigned char>(json[pos]))) ++pos;
    if (pos == start ||
        (pos == start + 1 && (json[start] == '-' || json[start] == '+'))) {
        return false;
    }

    try {
        value = std::stoi(json.substr(start, pos - start));
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

using SteadyClock = std::chrono::steady_clock;

double steadyMsecsSince(const SteadyClock::time_point &timePoint)
{
    if (timePoint == SteadyClock::time_point{}) {
        return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double, std::milli>(
        SteadyClock::now() - timePoint).count();
}

struct StrategyPoint {
    double x = 0.0;
    double y = 0.0;
};

StrategyPoint operator+(const StrategyPoint &lhs, const StrategyPoint &rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

StrategyPoint operator-(const StrategyPoint &lhs, const StrategyPoint &rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

StrategyPoint operator*(const StrategyPoint &point, double scale)
{
    return {point.x * scale, point.y * scale};
}

double strategyNorm(const StrategyPoint &point)
{
    return std::hypot(point.x, point.y);
}

StrategyPoint normalizedOr(const StrategyPoint &point,
                           const StrategyPoint &fallback = {1.0, 0.0})
{
    const double length = strategyNorm(point);
    if (length < 1e-6 || !std::isfinite(length)) return fallback;
    return point * (1.0 / length);
}

double strategyCross(const StrategyPoint &a, const StrategyPoint &b)
{
    return a.x * b.y - a.y * b.x;
}

double pointSegmentDistance(const StrategyPoint &point,
                            const StrategyPoint &start,
                            const StrategyPoint &end)
{
    const StrategyPoint segment = end - start;
    const double lengthSquared = segment.x * segment.x + segment.y * segment.y;
    if (lengthSquared < 1e-9) return strategyNorm(point - start);
    const StrategyPoint relative = point - start;
    const double t = std::clamp(
        (relative.x * segment.x + relative.y * segment.y) / lengthSquared,
        0.0,
        1.0);
    return strategyNorm(point - (start + segment * t));
}

int orientation(const StrategyPoint &a, const StrategyPoint &b,
                const StrategyPoint &c)
{
    const double value = strategyCross(b - a, c - a);
    if (std::fabs(value) < 1e-8) return 0;
    return value > 0.0 ? 1 : -1;
}

bool segmentsIntersect(const StrategyPoint &a0, const StrategyPoint &a1,
                       const StrategyPoint &b0, const StrategyPoint &b1)
{
    const int o1 = orientation(a0, a1, b0);
    const int o2 = orientation(a0, a1, b1);
    const int o3 = orientation(b0, b1, a0);
    const int o4 = orientation(b0, b1, a1);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && pointSegmentDistance(b0, a0, a1) < 1e-6) return true;
    if (o2 == 0 && pointSegmentDistance(b1, a0, a1) < 1e-6) return true;
    if (o3 == 0 && pointSegmentDistance(a0, b0, b1) < 1e-6) return true;
    if (o4 == 0 && pointSegmentDistance(a1, b0, b1) < 1e-6) return true;
    return false;
}

double segmentDistance(const StrategyPoint &a0, const StrategyPoint &a1,
                       const StrategyPoint &b0, const StrategyPoint &b1)
{
    if (segmentsIntersect(a0, a1, b0, b1)) return 0.0;
    return std::min({
        pointSegmentDistance(a0, b0, b1),
        pointSegmentDistance(a1, b0, b1),
        pointSegmentDistance(b0, a0, a1),
        pointSegmentDistance(b1, a0, a1),
    });
}

bool isUsableAssistSlot(AssistSlot slot)
{
    return slot >= AssistSlot::COVER_MID &&
           slot <= AssistSlot::WIDE_OUTLET;
}

vector<AssistSlot> slotsForAssistCount(size_t count)
{
    if (count == 0) return {};
    if (count == 1) return {AssistSlot::COVER_MID};
    if (count == 2) {
        return {AssistSlot::COVER_MID, AssistSlot::SHADOW_SUPPORT};
    }
    if (count == 3) {
        return {
            AssistSlot::COVER_MID,
            AssistSlot::SHADOW_SUPPORT,
            AssistSlot::ANCHOR_COVER,
        };
    }
    return {
        AssistSlot::COVER_MID,
        AssistSlot::SHADOW_SUPPORT,
        AssistSlot::ANCHOR_COVER,
        AssistSlot::WIDE_OUTLET,
    };
}

StrategyPoint calculateShadowTarget(const Point &ball, int side,
                                    const FieldDimensions &field)
{
    const auto target = assist_strategy_policy::rawShadowTarget(
        {ball.x, ball.y},
        side,
        {
            field.length,
            field.width,
            field.penaltyAreaLength,
            field.goalAreaLength,
        });
    return {target.x, target.y};
}

int preferredShadowSide(const Point &ball, double leaderKickDir, int leaderId,
                        const FieldDimensions &field)
{
    return assist_strategy_policy::preferredShadowSide(
        {ball.x, ball.y},
        leaderKickDir,
        leaderId,
        {
            field.length,
            field.width,
            field.penaltyAreaLength,
            field.goalAreaLength,
        });
}

Pose2D calculateAssistSlotTarget(AssistSlot slot, const Point &ball,
                                 double leaderKickDir, int leaderId,
                                 int shadowSideOverride,
                                 size_t assistCount,
                                 const FieldDimensions &field)
{
    const int shadowSide = std::abs(shadowSideOverride) == 1
        ? shadowSideOverride
        : preferredShadowSide(ball, leaderKickDir, leaderId, field);
    const auto targets = assist_strategy_policy::calculateTargets(
        assistCount,
        {ball.x, ball.y},
        shadowSide,
        {
            field.length,
            field.width,
            field.penaltyAreaLength,
            field.goalAreaLength,
        });
    const auto target = targets[static_cast<size_t>(slot)];

    return {
        target.x,
        target.y,
        std::atan2(ball.y - target.y, ball.x - target.x),
    };
}

double estimateAssistWalkingTime(const Pose2D &robot, const Pose2D &target)
{
    const double dxField = target.x - robot.x;
    const double dyField = target.y - robot.y;
    const double dx = std::cos(robot.theta) * dxField +
                      std::sin(robot.theta) * dyField;
    const double dy = -std::sin(robot.theta) * dxField +
                       std::cos(robot.theta) * dyField;
    const double deltaAngle = toPInPI(target.theta - robot.theta);

    constexpr double FORWARD_SPEED = 1.0;
    constexpr double BACKWARD_SPEED = 0.25;
    constexpr double STRAFE_SPEED = 0.6;
    constexpr double TURN_SPEED = 1.5;
    const double longitudinalTime = dx >= 0.0
        ? dx / FORWARD_SPEED
        : -dx / BACKWARD_SPEED;
    const double omniTime = std::sqrt(
        longitudinalTime * longitudinalTime +
        std::pow(dy / STRAFE_SPEED, 2.0) +
        std::pow(deltaAngle / TURN_SPEED, 2.0));

    const double travelDirection = std::atan2(dy, dx);
    const double distance = std::hypot(dx, dy);
    const double turnWalkTurnTime =
        std::fabs(travelDirection) / TURN_SPEED +
        distance / FORWARD_SPEED +
        std::fabs(toPInPI(deltaAngle - travelDirection)) / TURN_SPEED;
    const double blend = std::clamp((distance - 0.4) / 0.4, 0.0, 1.0);
    return omniTime * (1.0 - blend) + turnWalkTurnTime * blend;
}

std::array<AssistSlot, MAX_NUM_PLAYERS> calculateAssistAssignments(
    const vector<int> &assistantIds,
    const std::array<AssistSlot, MAX_NUM_PLAYERS> &previousAssignments,
    int previousOwnerId,
    int newOwnerId,
    const std::array<Pose2D, MAX_NUM_PLAYERS> &poses,
    const Point &ball,
    const Pose2D &leaderPose,
    double leaderKickDir,
    int formationShadowSide,
    const FieldDimensions &field,
    double normalSwitchPenalty,
    double anchorSwitchPenalty,
    double laneCrossPenalty,
    double pathCrossPenalty)
{
    std::array<AssistSlot, MAX_NUM_PLAYERS> result{};
    if (assistantIds.empty()) return result;

    const vector<AssistSlot> desiredSlots = slotsForAssistCount(assistantIds.size());
    vector<int> assignedIds = assistantIds;
    if (assignedIds.size() > desiredSlots.size()) {
        assignedIds.resize(desiredSlots.size());
    }

    vector<AssistSlot> permutation = desiredSlots;
    std::sort(permutation.begin(), permutation.end(), [](AssistSlot lhs, AssistSlot rhs) {
        return static_cast<int>(lhs) < static_cast<int>(rhs);
    });
    const AssistSlot inheritedSlot =
        newOwnerId > 0 && newOwnerId <= MAX_NUM_PLAYERS
        ? previousAssignments[newOwnerId - 1]
        : AssistSlot::NONE;
    const StrategyPoint ballPoint{ball.x, ball.y};
    const StrategyPoint leaderPoint{leaderPose.x, leaderPose.y};
    const StrategyPoint kickEnd{
        ball.x + 4.5 * std::cos(leaderKickDir),
        ball.y + 4.5 * std::sin(leaderKickDir),
    };

    double bestCost = std::numeric_limits<double>::infinity();
    vector<AssistSlot> bestPermutation = permutation;
    do {
        double totalCost = 0.0;
        vector<Pose2D> targets;
        targets.reserve(assignedIds.size());
        for (size_t i = 0; i < assignedIds.size(); ++i) {
            const int playerId = assignedIds[i];
            const AssistSlot slot = permutation[i];
            const Pose2D target = calculateAssistSlotTarget(
                slot, ball, leaderKickDir, newOwnerId,
                formationShadowSide, assistantIds.size(), field);
            targets.push_back(target);
            const double walkingTime = estimateAssistWalkingTime(
                poses[playerId - 1], target);
            totalCost += walkingTime * walkingTime;

            const AssistSlot previous = previousAssignments[playerId - 1];
            if (isUsableAssistSlot(previous) && previous != slot) {
                const bool previousStillExists = std::find(
                    desiredSlots.begin(), desiredSlots.end(), previous) !=
                    desiredSlots.end();
                if (previousStillExists) {
                    totalCost += 1000.0;
                } else {
                    totalCost += normalSwitchPenalty;
                }
                if (previous == AssistSlot::ANCHOR_COVER ||
                    slot == AssistSlot::ANCHOR_COVER) {
                    totalCost += anchorSwitchPenalty;
                }
            }
            if (playerId == previousOwnerId && isUsableAssistSlot(inheritedSlot) &&
                slot != inheritedSlot) {
                totalCost += 2000.0;
            }

            const StrategyPoint start{
                poses[playerId - 1].x,
                poses[playerId - 1].y,
            };
            const StrategyPoint end{target.x, target.y};
            if (segmentDistance(start, end, leaderPoint, ballPoint) < 0.9 ||
                segmentDistance(start, end, ballPoint, kickEnd) < 0.85 ||
                pointSegmentDistance(ballPoint, start, end) < 1.3) {
                totalCost += laneCrossPenalty;
            }
        }

        for (size_t i = 0; i < assignedIds.size(); ++i) {
            const StrategyPoint startI{
                poses[assignedIds[i] - 1].x,
                poses[assignedIds[i] - 1].y,
            };
            const StrategyPoint endI{targets[i].x, targets[i].y};
            for (size_t j = i + 1; j < assignedIds.size(); ++j) {
                const StrategyPoint startJ{
                    poses[assignedIds[j] - 1].x,
                    poses[assignedIds[j] - 1].y,
                };
                const StrategyPoint endJ{targets[j].x, targets[j].y};
                if (segmentDistance(startI, endI, startJ, endJ) < 0.7) {
                    totalCost += pathCrossPenalty;
                }
            }
        }

        if (totalCost + 1e-6 < bestCost) {
            bestCost = totalCost;
            bestPermutation = permutation;
        }
    } while (std::next_permutation(
        permutation.begin(), permutation.end(), [](AssistSlot lhs, AssistSlot rhs) {
            return static_cast<int>(lhs) < static_cast<int>(rhs);
        }));

    for (size_t i = 0; i < assignedIds.size(); ++i) {
        result[assignedIds[i] - 1] = bestPermutation[i];
    }
    return result;
}

bool assistAssignmentsValid(
    const std::array<AssistSlot, MAX_NUM_PLAYERS> &assignments,
    const vector<int> &assistantIds)
{
    const vector<AssistSlot> desiredSlots = slotsForAssistCount(assistantIds.size());
    if (assistantIds.size() > desiredSlots.size()) return false;
    vector<AssistSlot> actualSlots;
    for (int id : assistantIds) {
        const AssistSlot slot = assignments[id - 1];
        if (!isUsableAssistSlot(slot)) return false;
        actualSlots.push_back(slot);
    }
    std::sort(actualSlots.begin(), actualSlots.end(), [](AssistSlot lhs, AssistSlot rhs) {
        return static_cast<int>(lhs) < static_cast<int>(rhs);
    });
    vector<AssistSlot> sortedDesired = desiredSlots;
    std::sort(sortedDesired.begin(), sortedDesired.end(), [](AssistSlot lhs, AssistSlot rhs) {
        return static_cast<int>(lhs) < static_cast<int>(rhs);
    });
    return actualSlots == sortedDesired;
}

} // namespace

Brain::Brain() : rclcpp::Node("brain_node")
{
    // Initialize the TF broadcaster.
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Parameters must be declared here before they can be read.
    // Use dotted names for hierarchical YAML parameters.

    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<int>("game.initial_goalkeeper_id", 1);
    declare_parameter<string>("game.player_start_pos", "left");
    declare_parameter<bool>("game.treat_person_as_robot", false);
    declare_parameter<int>("game.number_of_players", 2);

    declare_parameter<double>("robot.robot_height", 1.28146);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.odom_theta_offset", 0.0);
    declare_parameter<bool>("robot.odom_theta_auto_align", true);
    declare_parameter<double>("robot.odom_theta_alignment_distance", 1.0);
    declare_parameter<double>(
        "robot.odom_theta_alignment_min_concentration", 0.75);
    declare_parameter<double>("robot.vx_factor", 0.90);
    declare_parameter<double>("robot.yaw_offset", 0.0);
    declare_parameter<double>("robot.vx_limit", 1.7);
    declare_parameter<double>("robot.vy_limit", 1.2);
    declare_parameter<double>("robot.vtheta_limit", 1.5);
    declare_parameter<double>("robot.min_vx", 0.4);
    declare_parameter<double>("robot.min_vy", 0.4);
    declare_parameter<double>("robot.min_vtheta", 0.4);

    declare_parameter<double>("strategy.ball_confidence_threshold", 50.0);
    declare_parameter<double>("strategy.ball_memory_timeout", 3.0);
    declare_parameter<double>("strategy.tm_ball_dist_threshold", 3.0);
    declare_parameter<bool>("strategy.limit_near_ball_speed", true);
    declare_parameter<double>("strategy.near_ball_speed_limit", 0.3);
    declare_parameter<double>("strategy.near_ball_range", 4.0);
    declare_parameter<bool>("strategy.soft_kickoff", false);
    declare_parameter<double>("strategy.soft_kickoff_speed", 0.3);
    declare_parameter<double>("strategy.kick_range", 1.0);
    declare_parameter<double>("strategy.kick_theta_range", 0.2);
    declare_parameter<double>("strategy.search.initial_spin_secs", 1.5);
    declare_parameter<double>("strategy.search.waypoint_scan_secs", 3.0);
    declare_parameter<double>("strategy.search.waypoint_timeout_secs", 25.0);
    declare_parameter<double>("strategy.search.last_observation_max_age_secs", 12.0);
    declare_parameter<double>("strategy.search.vx_limit", 0.7);
    declare_parameter<double>("strategy.search.vy_limit", 0.35);
    declare_parameter<double>("strategy.search.vtheta_limit", 0.8);
    declare_parameter<double>("strategy.search.arrival_tolerance", 0.65);
    declare_parameter<bool>("strategy.abort_kick_when_ball_moved", false);
    declare_parameter<double>("strategy.abort_kick_ball_move_threshold", 0.3);
    declare_parameter<double>("strategy.freekick.execution_timeout_ms", 10000.0);
    declare_parameter<double>("strategy.freekick.plan_select_ms", 250.0);
    declare_parameter<bool>("strategy.enable_bypass", false);
    declare_parameter<bool>("strategy.enable_shoot", false);
    declare_parameter<bool>("strategy.enable_directional_kick", false);

    declare_parameter<bool>("strategy.use_move_block", true);
    declare_parameter<double>("strategy.move_block_msecs", 2000.0);
    declare_parameter<bool>("strategy.enable_auto_visual_kick", false);
    declare_parameter<double>("strategy.auto_visual_kick_enable_dist_min", 0.2);
    declare_parameter<double>("strategy.auto_visual_kick_enable_dist_max", 4.0);
    declare_parameter<double>("strategy.auto_visual_kick_enable_angle", 1.2217304763960306);
    declare_parameter<bool>("strategy.power_shoot.enable", false);
    declare_parameter<bool>("strategy.power_shoot.use_for_kickoff", false);
    declare_parameter<double>("strategy.power_shoot.xmin", 0.5);
    declare_parameter<double>("strategy.power_shoot.xmax", 1.0);
    declare_parameter<double>("strategy.power_shoot.ymin", -0.5);
    declare_parameter<double>("strategy.power_shoot.ymax", 0.5);
    declare_parameter<double>("strategy.shoot.threat_threshold", 0.0);
    declare_parameter<double>("strategy.shoot.xmin", 0.5);
    declare_parameter<double>("strategy.shoot.xmax", 1.0);
    declare_parameter<double>("strategy.shoot.ymin", -0.5);
    declare_parameter<double>("strategy.shoot.ymax", 0.5);
    declare_parameter<bool>("strategy.cooperation.enable_role_switch", true);
    declare_parameter<double>("strategy.cooperation.ball_control_cost_threshold", 10.0);
    declare_parameter<double>("strategy.cooperation.goalie_claim_max_ball_range", 1.5);
    declare_parameter<double>("strategy.cooperation.goalie_claim_extra_depth", 1.0);
    declare_parameter<double>("strategy.cooperation.goalie_claim_lateral_margin", 0.5);
    declare_parameter<double>("strategy.cooperation.goalie_restore_stable_ms", 500.0);
    declare_parameter<double>("strategy.cooperation.tactical_packet_timeout_ms", 600.0);
    declare_parameter<double>("strategy.cooperation.leader_switch_cost_margin", 0.75);
    declare_parameter<double>("strategy.cooperation.leader_switch_cost_ratio", 0.9);
    declare_parameter<double>("strategy.cooperation.leader_switch_confirm_ms", 350.0);
    declare_parameter<double>("strategy.cooperation.leader_min_hold_ms", 1200.0);
    declare_parameter<double>("strategy.cooperation.leader_ownerless_fallback_ms", 600.0);
    declare_parameter<double>("strategy.cooperation.kickoff_owner_select_ms", 700.0);
    declare_parameter<double>("strategy.cooperation.kickoff_ball_move_threshold", 0.3);
    declare_parameter<double>("strategy.cooperation.kickoff_timeout_ms", 10000.0);
    declare_parameter<double>("strategy.cooperation.assist_slot_switch_penalty", 4.0);
    declare_parameter<double>("strategy.cooperation.assist_anchor_switch_penalty", 9.0);
    declare_parameter<double>("strategy.cooperation.assist_lane_cross_penalty", 16.0);
    declare_parameter<double>("strategy.cooperation.assist_path_cross_penalty", 9.0);
    declare_parameter<double>("strategy.cooperation.assist_target_deadband", 0.2);
    declare_parameter<double>("strategy.cooperation.assist_anchor_deadband", 0.5);
    declare_parameter<double>("strategy.cooperation.assist_anchor_hold_ms", 4000.0);
    declare_parameter<double>("strategy.cooperation.assist_shadow_side_hold_ms", 1500.0);
    declare_parameter<bool>("strategy.cooperation.assist_avoid_obstacles", true);
    declare_parameter<double>("strategy.cooperation.assist_yield_timeout_ms", 4800.0);
    declare_parameter<double>("strategy.cooperation.assist_teammate_position_clearance", 0.75);
    declare_parameter<double>("strategy.cooperation.assist_teammate_path_clearance", 0.60);
    declare_parameter<double>("strategy.cooperation.assist_waypoint_lock_ms", 1500.0);
    declare_parameter<double>("strategy.cooperation.assist_waypoint_ttl_ms", 3000.0);
    declare_parameter<double>("strategy.cooperation.assist_waypoint_no_progress_ms", 1000.0);

    declare_parameter<int>("obstacle_avoidance.depth_sample_step", 8);
    declare_parameter<int>("obstacle_avoidance.depth_confirm_frames", 2);
    declare_parameter<int>("obstacle_avoidance.depth_clear_frames", 2);
    declare_parameter<double>("obstacle_avoidance.ground_height", 0.0);
    declare_parameter<double>("obstacle_avoidance.obstacle_min_height", 0.3);
    declare_parameter<double>("obstacle_avoidance.grid_size", 0.2);
    declare_parameter<double>("obstacle_avoidance.max_x", 3.0);
    declare_parameter<double>("obstacle_avoidance.max_y", 5.0);
    declare_parameter<double>("obstacle_avoidance.exclusion_x", 0.4);
    declare_parameter<double>("obstacle_avoidance.exclusion_y", 0.5);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_radius", 0.0);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_height", 0.3);
    declare_parameter<int>("obstacle_avoidance.occupancy_threshold", 20);
    declare_parameter<double>("obstacle_avoidance.collision_threshold", 0.3);
    declare_parameter<double>("obstacle_avoidance.self_robot_radius", 0.5);
    declare_parameter<double>("obstacle_avoidance.safe_distance", 1.0);
    declare_parameter<double>("obstacle_avoidance.avoid_secs", 2.0);
    declare_parameter<bool>("obstacle_avoidance.enable_freekick_avoid", true);
    declare_parameter<bool>("obstacle_avoidance.enable_ball_obstacle", true);
    declare_parameter<double>("obstacle_avoidance.ball_obstacle_radius", 0.3);
    declare_parameter<double>(
        "obstacle_avoidance.ball_obstacle_max_age_msecs", 500.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_person", true);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_safe_distance", 0.5);
    declare_parameter<double>("obstacle_avoidance.obstacle_memory_msecs", 500.0);
    declare_parameter<double>("obstacle_avoidance.robot_obstacle_memory_msecs", 1000.0);
    declare_parameter<int>("obstacle_avoidance.robot_confirm_frames", 2);
    declare_parameter<double>("obstacle_avoidance.robot_match_distance", 0.7);
    declare_parameter<double>("obstacle_avoidance.robot_merge_distance", 0.30);
    declare_parameter<double>("obstacle_avoidance.robot_min_confidence", 40.0);
    declare_parameter<double>("obstacle_avoidance.robot_tracking_alpha", 0.65);
    declare_parameter<double>("obstacle_avoidance.robot_tracking_beta", 0.35);
    declare_parameter<double>("obstacle_avoidance.robot_obstacle_distance", 4.0);
    declare_parameter<double>(
        "obstacle_avoidance.robot_obstacle_min_distance", 0.20);
    declare_parameter<double>("obstacle_avoidance.robot_depth_fusion_distance", 0.5);
    declare_parameter<double>("obstacle_avoidance.robot_depth_fusion_max_age_msecs", 250.0);
    declare_parameter<double>("obstacle_avoidance.upright_robot_radius", 0.6);
    declare_parameter<double>("obstacle_avoidance.fallen_robot_radius", 0.85);
    declare_parameter<double>("obstacle_avoidance.max_robot_footprint_width", 1.8);
    declare_parameter<double>("obstacle_avoidance.max_robot_footprint_radius_increase", 0.15);
    declare_parameter<double>("obstacle_avoidance.robot_path_clearance", 0.12);
    declare_parameter<double>("obstacle_avoidance.robot_avoidance_speed", 0.70);
    declare_parameter<double>("obstacle_avoidance.robot_avoidance_min_speed", 0.32);
    declare_parameter<double>("obstacle_avoidance.robot_avoidance_min_vy", 0.40);
    declare_parameter<double>(
        "obstacle_avoidance.robot_avoidance_reverse_speed", 0.35);
    declare_parameter<double>("obstacle_avoidance.robot_emergency_distance", 1.25);
    declare_parameter<double>("obstacle_avoidance.robot_emergency_range", 2.50);
    declare_parameter<double>("obstacle_avoidance.robot_emergency_confidence", 70.0);
    declare_parameter<double>("obstacle_avoidance.robot_ttc_horizon_msecs", 900.0);
    declare_parameter<double>("obstacle_avoidance.robot_prediction_horizon_msecs", 500.0);
    declare_parameter<double>("obstacle_avoidance.robot_avoidance_lateral_gain", 0.80);
    declare_parameter<double>("obstacle_avoidance.robot_avoidance_switch_penalty", 0.8);
    declare_parameter<double>("obstacle_avoidance.avoidance_direction_lock_msecs", 1500.0);
    declare_parameter<double>("obstacle_avoidance.avoidance_direction_switch_margin", 0.2);
    declare_parameter<double>("obstacle_avoidance.avoidance_progress_timeout_msecs", 450.0);
    declare_parameter<double>("obstacle_avoidance.avoidance_progress_distance", 0.05);
    declare_parameter<double>("obstacle_avoidance.avoidance_progress_clearance", 0.05);
    declare_parameter<double>("obstacle_avoidance.fallen_robot_distance", 3.0);
    declare_parameter<double>("obstacle_avoidance.fallen_robot_angle", 0.7853981633974483);
    declare_parameter<bool>(
        "obstacle_avoidance.enable_fallen_robot_visual_kick_exit", true);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_chase", false);
    declare_parameter<double>("obstacle_avoidance.chase_ao_safe_dist", 2.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_kick", false);
    declare_parameter<double>("obstacle_avoidance.kick_ao_safe_dist", 1.0);
    declare_parameter<bool>("obstacle_avoidance.kick_ao_use_shoot", false);
    declare_parameter<bool>("obstacle_avoidance.log_enable", true);
    declare_parameter<string>(
        "obstacle_avoidance.log_path", "obstacle_avoidance.log");
    declare_parameter<double>("obstacle_avoidance.log_hz", 5.0);
    declare_parameter<double>("obstacle_avoidance.log_max_size_mb", 10.0);
    declare_parameter<int>("obstacle_avoidance.log_max_files", 3);
    declare_parameter<double>(
        "obstacle_avoidance.blocked_recovery_turn_rate", 0.5);

    declare_parameter<int>("locator.min_marker_count", 5);
    declare_parameter<double>("locator.max_residual", 0.3);

    declare_parameter<bool>("enable_com", false);
    declare_parameter<double>("communication.team_broadcast_rate_hz", 20.0);
    declare_parameter<string>(
        "communication.discovery_address", "auto");
    declare_parameter<string>(
        "communication.team_secret_hex", "");

    declare_parameter<bool>("rerunLog.enable_tcp", false);
    declare_parameter<string>("rerunLog.server_ip", "");
    declare_parameter<bool>("rerunLog.enable_file", false);
    declare_parameter<string>("rerunLog.log_dir", "");
    declare_parameter<double>("rerunLog.max_log_file_mins", 5.0);
    declare_parameter<int>("rerunLog.img_interval", 10);
    declare_parameter<double>("rerunLog.visual_hz", 20.0);
    declare_parameter<double>("rerunLog.timeseries_hz", 10.0);
    declare_parameter<double>("rerunLog.debug_hz", 5.0);

    declare_parameter<bool>("odomLog.enable", true);
    declare_parameter<string>("odomLog.log_dir", "");
    declare_parameter<double>("odomLog.hz", 20.0);
    declare_parameter<double>("odomLog.flush_interval_ms", 1000.0);

    declare_parameter<bool>("sound.enable", false);
    declare_parameter<string>("sound.sound_pack", "espeak");

    declare_parameter<string>("vision.image_topic", "/boostercamera/head/rgb");
    declare_parameter<string>("vision.depth_image_topic", "/boostercamera/head/depth");
    declare_parameter<string>(
        "vision.camera_info_topic", "/boostercamera/head/rgb/camera_info");
    declare_parameter<string>("vision.head_pose_topic", "/head_pose_stamped");
    declare_parameter<double>("vision.cam_pixel_width", 544);
    declare_parameter<double>("vision.cam_pixel_height", 448);
    declare_parameter<double>("vision.cam_fov_x", 105);
    declare_parameter<double>("vision.cam_fov_y", 94);
    declare_parameter<double>("vision.depth_scale_16u", 0.001);
    declare_parameter<double>("vision.depth_scale_32f", 0.001);
    declare_parameter<bool>("vision.depth_aligned_to_color", true);
    declare_parameter<bool>("vision.depth_is_z", true);
    declare_parameter<bool>("vision.depth_project_to_ground", true);
    declare_parameter<double>("vision.depth_pose_max_sync_diff_ms", 50.0);
    declare_parameter<double>("vision.depth_pose_buffer_msecs", 1000.0);

    declare_parameter<string>("game_control_ip", "0.0.0.0");

    declare_parameter<string>("tree_file_path", "");
    declare_parameter<string>("vision_config_path", "");
    declare_parameter<string>("vision_config_local_path", "");

    declare_parameter<int>("recovery.retry_max_count", 2);
    declare_parameter<int>("recovery.prepare_wait_ms", 1500);
    declare_parameter<int>("recovery.prepare_confirm_timeout_ms", 5000);
    declare_parameter<int>("recovery.mode_query_interval_ms", 200);
    declare_parameter<int>("recovery.mode_query_timeout_ms", 1000);
    declare_parameter<int>("recovery.posture_confirm_timeout_ms", 3000);
    declare_parameter<int>("recovery.walking_confirm_timeout_ms", 5000);
    declare_parameter<int>("recovery.soccer_switch_retry_ms", 1000);
    declare_parameter<int>("recovery.soccer_switch_timeout_ms", 15000);
    declare_parameter<string>("recovery.getup_version", "kV2");
    declare_parameter<double>("recovery.getup_no_movement_grace_sec", 3.0);
    declare_parameter<int>("recovery.localization_resume_delay_ms", 3500);
    recoveryPostGetupOdomSettleMinimumMs_ = declare_parameter<double>(
        "recovery.post_getup_odom_settle_minimum_ms", 3500.0);
    recoveryPostGetupOdomSettleWindowMs_ = declare_parameter<double>(
        "recovery.post_getup_odom_settle_window_ms", 750.0);
    recoveryPostGetupOdomSettleMaximumMs_ = declare_parameter<double>(
        "recovery.post_getup_odom_settle_maximum_ms", 7500.0);
    recoveryPostGetupOdomSettleMaxXyRangeM_ = declare_parameter<double>(
        "recovery.post_getup_odom_settle_max_xy_range_m", 0.015);
    recoveryPostGetupOdomSettleMaxThetaRangeDeg_ = declare_parameter<double>(
        "recovery.post_getup_odom_settle_max_theta_range_deg", 0.75);
    recoveryPostGetupOdomSettleMaxTiltRad_ = declare_parameter<double>(
        "recovery.post_getup_odom_settle_max_tilt_rad", 0.30);
    recoveryPostGetupOdomSettleMaxGyroRadPerSec_ = declare_parameter<double>(
        "recovery.post_getup_odom_settle_max_gyro_rad_per_sec", 0.35);
    declare_parameter<int>("recovery.heading_realign_window_ms", 9000);
    declare_parameter<int>("recovery.heading_realign_attempts", 18);
    declare_parameter<int>("recovery.heading_realign_interval_ms", 300);
    declare_parameter<int>("recovery.heading_realign_confirmations", 2);
    declare_parameter<double>("recovery.heading_realign_max_delta_rad", 1.85);
    declare_parameter<double>("recovery.heading_realign_min_delta_rad", 0.20);
    declare_parameter<double>("recovery.heading_realign_consistency_rad", 0.25);
    declare_parameter<double>("recovery.heading_realign_max_candidate_pos_m", 2.5);

    declare_parameter<string>("RLVisionKick.visual_kick_version", "kV2");
}

Brain::~Brain()
{
    // Communication threads dereference Brain state. Stop them before member
    // destruction begins so they cannot observe partially destroyed objects.
    communication.reset();
    obstacleAvoidanceLog_.reset();
    odomDiagnosticLog_.reset();
}

void Brain::init()
{

    config = std::make_shared<BrainConfig>();
    loadConfig();

    odomThetaAlignmentOffset_ = toPInPI(config->robotOdomThetaOffset);
    odomThetaAlignmentLocked_ = !config->robotOdomThetaAutoAlign;

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();
    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);
    communication = std::make_shared<BrainCommunication>(this);


    locator->init(config->fieldDimensions, config->pfMinMarkerCnt, config->pfMaxResidual);


    tree->init();


    client->init();
    setupRecoveryModeMonitor();

    log->prepare();
    initOdomDiagnosticLog();
    initObstacleAvoidanceLog();



    data->lastSuccessfulLocalizeTime = get_clock()->now();
    data->timeLastDet = get_clock()->now();
    data->timeLastLineDet = get_clock()->now();
    data->timeLastGamecontrolMsg = get_clock()->now();
    data->ball.timePoint = get_clock()->now();


    for (int i = 0; i < MAX_NUM_PLAYERS; i++) {
        data->tmStatus[i].isAlive = false;
    }

    data->gcGoalkeeperId = std::clamp(
        static_cast<int>(get_parameter("game.initial_goalkeeper_id").as_int()),
        1,
        std::max(1, std::min(config->numOfPlayers, MAX_NUM_PLAYERS)));
    data->gcPlayersPerTeam = std::min(config->numOfPlayers, MAX_NUM_PLAYERS);
    data->tmOutboundSnapshot.playerRole =
        config->playerRole == "goal_keeper" ? 2 : 1;

    // Start UDP threads only after all outbound snapshots are initialized.
    communication->initCommunication();


    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", SUB_STATE_QUEUE_SIZE, bind(&Brain::detectionsCallback, this, _1));
    subFieldLine = create_subscription<vision_interface::msg::LineSegments>("/booster_vision/line_segments", SUB_STATE_QUEUE_SIZE, bind(&Brain::fieldLineCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::lowStateCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", SUB_STATE_QUEUE_SIZE, bind(&Brain::headPoseCallback, this, _1));
    // JetPack 7.2/T2 publishes the timestamped pose used by vision.  Keep the
    // legacy Pose subscription above for deployments that have not migrated.
    const string headPoseTopic = get_parameter("vision.head_pose_topic").as_string();
    headPoseStampedSubscription = create_subscription<geometry_msgs::msg::PoseStamped>(
        headPoseTopic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        bind(&Brain::headPoseStampedCallback, this, _1));
    // Prefer the pose published by VisionNode. It contains the URDF camera-link
    // pose when automatic calibration is enabled, while legacy mode publishes
    // the raw head pose; camera.extrin is applied in the callback for both.
    headPoseTfSubscription = create_subscription<geometry_msgs::msg::TransformStamped>(
        "/booster_vision/t_head2base", rclcpp::QoS(10),
        bind(&Brain::headPoseTfCallback, this, _1));
    recoveryStateSubscription = create_subscription<booster_interface::msg::RawBytesMsg>("fall_down_recovery_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::recoveryStateCallback, this, _1));

    if (config->rerunLogEnableFile || config->rerunLogEnableTCP) {
        string imageTopic = get_parameter("vision.image_topic").as_string();
        imageSubscription = create_subscription<sensor_msgs::msg::Image>(
            imageTopic, rclcpp::SensorDataQoS(), bind(&Brain::imageCallback, this, _1));
    }
    string depthTopic = get_parameter("vision.depth_image_topic").as_string();
    // T2 camera streams use the ROS sensor-data (best-effort) QoS profile.
    const auto depthQos = rclcpp::SensorDataQoS();
    depthImageSubscription = create_subscription<sensor_msgs::msg::Image>(
        depthTopic, depthQos, bind(&Brain::depthImageCallback, this, _1));
    const string cameraInfoTopic =
        get_parameter("vision.camera_info_topic").as_string();
    cameraInfoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>(
        cameraInfoTopic,
        rclcpp::SensorDataQoS().keep_last(5),
        bind(&Brain::cameraInfoCallback, this, _1));

    pubSoundPlay = create_publisher<std_msgs::msg::String>("/play_sound", 10);
    pubSpeak = create_publisher<std_msgs::msg::String>("/speak", 10);
    pubKickBall = create_publisher<brain::msg::Kick>("/kick_ball", 10);
}

void Brain::initOdomDiagnosticLog()
{
    if (!config->odomLogEnable) return;

    namespace fs = std::filesystem;
    fs::path outputDirectory;
    const bool reuseRerunSession =
        config->rerunLogEnableFile && config->odomLogDir.empty();
    if (reuseRerunSession) {
        outputDirectory = config->rerunLogLogDir;
    } else {
        const std::string baseDirectory = config->odomLogDir.empty()
            ? config->rerunLogLogDir
            : config->odomLogDir;
        const fs::path basePath = baseDirectory.empty()
            ? fs::path(".")
            : fs::path(baseDirectory);
        outputDirectory = gen_timestamped_filename(
            basePath.string(),
            format("_P%d_T%d", config->playerId, config->teamId));
    }

    std::error_code directoryError;
    fs::create_directories(outputDirectory, directoryError);
    if (directoryError) {
        RCLCPP_ERROR(
            get_logger(),
            "Cannot create odom log directory '%s': %s",
            outputDirectory.c_str(),
            directoryError.message().c_str());
        return;
    }

    std::error_code executableError;
    const fs::path executable = fs::read_symlink("/proc/self/exe", executableError);
    OdomDiagnosticMetadata metadata;
    metadata.gitDescribe = BRAIN_GIT_DESCRIBE;
    metadata.buildDate = __DATE__;
    metadata.buildTime = __TIME__;
    metadata.executablePath = executableError ? "unknown" : executable.string();
    metadata.playerId = config->playerId;
    metadata.teamId = config->teamId;
    metadata.sampleHz = config->odomLogHz;
    metadata.flushIntervalMs = config->odomLogFlushIntervalMs;
    metadata.odomFactor = std::fabs(config->robotOdomFactor);
    metadata.odomThetaOffset = config->robotOdomThetaOffset;
    metadata.odomThetaAutoAlign = config->robotOdomThetaAutoAlign;
    metadata.odomThetaAlignmentDistance =
        config->robotOdomThetaAlignmentDistance;
    metadata.odomThetaAlignmentMinConcentration =
        config->robotOdomThetaAlignmentMinConcentration;

    const fs::path filePath = outputDirectory / "odom.log";
    auto diagnosticLog = std::make_unique<OdomDiagnosticLogger>(
        filePath.string(), metadata);
    if (!diagnosticLog->enabled()) {
        RCLCPP_ERROR(
            get_logger(),
            "Cannot start odom diagnostic log '%s': %s",
            filePath.c_str(),
            diagnosticLog->error().c_str());
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Odom diagnostic log: %s (%.1f Hz)",
        filePath.c_str(),
        config->odomLogHz);
    odomDiagnosticLog_ = std::move(diagnosticLog);
}

void Brain::initObstacleAvoidanceLog()
{
    if (!get_parameter("obstacle_avoidance.log_enable").as_bool()) return;

    const std::string configuredPath =
        get_parameter("obstacle_avoidance.log_path").as_string();
    const std::filesystem::path filePath = configuredPath.empty()
        ? std::filesystem::path("obstacle_avoidance.log")
        : std::filesystem::path(configuredPath);
    const double logHz = std::max(
        0.0, get_parameter("obstacle_avoidance.log_hz").as_double());
    const double maximumSizeMb = std::max(
        0.0,
        get_parameter("obstacle_avoidance.log_max_size_mb").as_double());
    const auto maximumBytes = static_cast<std::uintmax_t>(
        maximumSizeMb * 1024.0 * 1024.0);
    const auto maximumFiles = static_cast<std::size_t>(std::clamp<int64_t>(
        get_parameter("obstacle_avoidance.log_max_files").as_int(),
        int64_t{0}, int64_t{20}));

    auto avoidanceLog = std::make_unique<ObstacleAvoidanceLogger>(
        filePath.string(), logHz, maximumBytes, maximumFiles);
    if (!avoidanceLog->enabled()) {
        RCLCPP_ERROR(
            get_logger(),
            "Cannot start obstacle avoidance log '%s': %s",
            filePath.c_str(),
            avoidanceLog->error().c_str());
        return;
    }
    std::error_code absolutePathError;
    const auto absolutePath = std::filesystem::absolute(
        filePath, absolutePathError);
    RCLCPP_INFO(
        get_logger(),
        "Obstacle avoidance log: %s (%.1f Hz, %.1f MB x %zu backups)",
        (absolutePathError ? filePath : absolutePath).c_str(),
        logHz,
        maximumSizeMb,
        maximumFiles);
    obstacleAvoidanceLog_ = std::move(avoidanceLog);
}

void Brain::loadConfig()
{
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);
    get_parameter("game.treat_person_as_robot", config->treatPersonAsRobot);
    get_parameter("game.number_of_players", config->numOfPlayers);

    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.odom_theta_offset", config->robotOdomThetaOffset);
    get_parameter(
        "robot.odom_theta_auto_align", config->robotOdomThetaAutoAlign);
    get_parameter(
        "robot.odom_theta_alignment_distance",
        config->robotOdomThetaAlignmentDistance);
    get_parameter(
        "robot.odom_theta_alignment_min_concentration",
        config->robotOdomThetaAlignmentMinConcentration);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);
    get_parameter("robot.vx_limit", config->vxLimit);
    get_parameter("robot.vy_limit", config->vyLimit);
    get_parameter("robot.vtheta_limit", config->vthetaLimit);
    get_parameter("robot.min_vx", config->minVx);
    get_parameter("robot.min_vy", config->minVy);
    get_parameter("robot.min_vtheta", config->minVtheta);

    get_parameter("strategy.ball_confidence_threshold", config->ballConfidenceThreshold);
    get_parameter("strategy.tm_ball_dist_threshold", config->tmBallDistThreshold);
    get_parameter("strategy.limit_near_ball_speed", config->limitNearBallSpeed);
    get_parameter("strategy.near_ball_speed_limit", config->nearBallSpeedLimit);
    get_parameter("strategy.near_ball_range", config->nearBallRange);

    get_parameter("obstacle_avoidance.collision_threshold", config->collisionThreshold);
    get_parameter("obstacle_avoidance.safe_distance", config->safeDistance);
    get_parameter("obstacle_avoidance.avoid_secs", config->avoidSecs);

    get_parameter("locator.min_marker_count", config->pfMinMarkerCnt);
    get_parameter("locator.max_residual", config->pfMaxResidual);

    get_parameter("enable_com", config->enableCom);

    // get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.enable_tcp", config->rerunLogEnableTCP);
    get_parameter("rerunLog.server_ip", config->rerunLogServerIP);
    get_parameter("rerunLog.enable_file", config->rerunLogEnableFile);
    get_parameter("rerunLog.log_dir", config->rerunLogLogDir);
    get_parameter("rerunLog.max_log_file_mins", config->rerunLogMaxFileMins);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);
    get_parameter("rerunLog.visual_hz", config->rerunLogVisualHz);
    get_parameter("rerunLog.timeseries_hz", config->rerunLogTimeseriesHz);
    get_parameter("rerunLog.debug_hz", config->rerunLogDebugHz);

    get_parameter("odomLog.enable", config->odomLogEnable);
    get_parameter("odomLog.log_dir", config->odomLogDir);
    get_parameter("odomLog.hz", config->odomLogHz);
    get_parameter("odomLog.flush_interval_ms", config->odomLogFlushIntervalMs);

    get_parameter("sound.enable", config->soundEnable);
    get_parameter("sound.sound_pack", config->soundPack);

    get_parameter("RLVisionKick.visual_kick_version", config->RLVisionKickVisualKickVersion);

    get_parameter("tree_file_path", config->treeFilePath);

    get_parameter("vision.cam_pixel_width", config->camPixX);
    get_parameter("vision.cam_pixel_height", config->camPixY);
    double camDegX, camDegY;
    get_parameter("vision.cam_fov_x", camDegX);
    get_parameter("vision.cam_fov_y", camDegY);
    config->camAngleX = deg2rad(camDegX);
    config->camAngleY = deg2rad(camDegY);

    // Load parameters from the vision configuration.
    string visionConfigPath, visionConfigLocalPath;
    get_parameter("vision_config_path", visionConfigPath);
    get_parameter("vision_config_local_path", visionConfigLocalPath);
    if (!filesystem::exists(visionConfigPath)) {
        // Report the error and exit.
        RCLCPP_ERROR(get_logger(), "vision_config_path %s not exists", visionConfigPath.c_str());
        exit(1);
    }
    // else
    YAML::Node vConfig = YAML::LoadFile(visionConfigPath);
    if (filesystem::exists(visionConfigLocalPath)) {
        YAML::Node vConfigLocal = YAML::LoadFile(visionConfigLocalPath);
        MergeYAML(vConfig, vConfigLocal);
    }
    config->camfx = vConfig["camera"]["intrin"]["fx"].as<double>();
    config->camfy = vConfig["camera"]["intrin"]["fy"].as<double>();
    config->camcx = vConfig["camera"]["intrin"]["cx"].as<double>();
    config->camcy = vConfig["camera"]["intrin"]["cy"].as<double>();
    config->depthfx = config->camfx;
    config->depthfy = config->camfy;
    config->depthcx = config->camcx;
    config->depthcy = config->camcy;
    const auto depthIntrin = vConfig["camera"]["depth_intrin"];
    if (depthIntrin && depthIntrin.IsMap()) {
        config->depthfx = depthIntrin["fx"].as<double>(config->depthfx);
        config->depthfy = depthIntrin["fy"].as<double>(config->depthfy);
        config->depthcx = depthIntrin["cx"].as<double>(config->depthcx);
        config->depthcy = depthIntrin["cy"].as<double>(config->depthcy);
    }
    if (!std::isfinite(config->depthfx) ||
        !std::isfinite(config->depthfy) ||
        !std::isfinite(config->depthcx) ||
        !std::isfinite(config->depthcy) ||
        config->depthfx <= 0.0 || config->depthfy <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Invalid T2 depth intrinsics in vision config");
        exit(1);
    }
    get_parameter("vision.depth_scale_16u", config->depthScale16U);
    get_parameter("vision.depth_scale_32f", config->depthScale32F);
    get_parameter("vision.depth_aligned_to_color", config->depthAlignedToColor);
    get_parameter("vision.depth_is_z", config->depthIsZ);
    get_parameter("vision.depth_project_to_ground", config->depthProjectToGround);
    if (!std::isfinite(config->depthScale16U) || config->depthScale16U <= 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid vision.depth_scale_16u; using 0.001");
        config->depthScale16U = 0.001;
    }
    if (!std::isfinite(config->depthScale32F) || config->depthScale32F <= 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid vision.depth_scale_32f; using 0.001");
        config->depthScale32F = 0.001;
    }

    auto extrin = vConfig["camera"]["extrin"];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            config->camToHead(i, j) = extrin[i][j].as<double>();
        }
    }
    const std::string extrinDirection = vConfig["camera"]["extrin_direction"]
        ? vConfig["camera"]["extrin_direction"].as<std::string>()
        : "camera_to_head";
    if (extrinDirection == "head_to_camera") {
        config->camToHead = config->camToHead.inverse();
    } else if (extrinDirection != "camera_to_head") {
        RCLCPP_ERROR(
            get_logger(),
            "Unsupported camera extrin_direction '%s' (expected camera_to_head or head_to_camera)",
            extrinDirection.c_str());
        exit(1);
    }
    const Eigen::Matrix3d cameraRotation = config->camToHead.block<3, 3>(0, 0);
    const double rotationError =
        (cameraRotation * cameraRotation.transpose() - Eigen::Matrix3d::Identity()).norm();
    if (!config->camToHead.allFinite() ||
        std::fabs(config->camToHead(3, 3) - 1.0) > 1e-6 ||
        rotationError > 0.05 || std::fabs(cameraRotation.determinant() - 1.0) > 0.05) {
        RCLCPP_ERROR(get_logger(), "Invalid T2 camera extrinsic matrix");
        exit(1);
    }
    prtDebug(format("camfx: %f, camfy: %f, camcx: %f, camcy: %f; depthfx: %f, depthfy: %f, depthcx: %f, depthcy: %f",
                    config->camfx, config->camfy, config->camcx, config->camcy,
                    config->depthfx, config->depthfy, config->depthcx, config->depthcy));
    string str_cam2head = "camToHead: \n";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            str_cam2head += format("%.3f ", config->camToHead(i, j));
        }
        str_cam2head += "\n";
    }
    prtDebug(str_cam2head);


    config->handle();


    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}


void Brain::tick()
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    // Write diagnostic and log information.
    logDebugInfo();
    // logObstacleDistance(); // Expensive; enable only when needed.
    logLags();
    statusReport();
    logStatusToConsole();
    playSoundForFun();
    updateLogFile();

    updateMemory();
    handleSpecialStates();
    handleCooperation();
    handleKickoffPlan();
    handleFreeKickPlan();

    pubKickMsg();

    advanceRecoveryModeTransition(get_clock()->now());
    updateRecoveryLocalizationHold();
    tree->tick();
    refreshTeamCommunicationSnapshot();
}

void Brain::refreshTeamCommunicationSnapshot()
{
    TeamOutboundSnapshot snapshot;
    snapshot.playerRole =
        tree->getEntry<string>("player_role") == "goal_keeper" ? 2 : 1;
    snapshot.isAlive = data->tmImAlive;
    snapshot.isLead = data->tmImLead;
    snapshot.ballDetected = data->ballDetected;
    snapshot.ballLocationKnown = tree->getEntry<bool>("ball_location_known");
    snapshot.ballConfidence = data->ball.confidence;
    snapshot.ballRange = data->ball.range;
    snapshot.cost = data->tmMyCost;
    snapshot.ballPosToField = data->ball.posToField;
    snapshot.robotPoseToField = data->robotPoseToField;
    snapshot.kickDir = data->kickDir;
    snapshot.thetaRb = data->robotBallAngleToField;
    snapshot.ballOwnerId = data->tmBallOwnerId;
    snapshot.leaderTerm = data->tmLeaderTerm;
    snapshot.formationOwnerId = data->tmFormationOwnerId;
    snapshot.formationRevision = data->tmFormationRevision;
    snapshot.formationShadowSide = data->tmFormationShadowSide;
    snapshot.assistSlots = data->tmAssistSlots;
    snapshot.assistSlot = data->tmMyAssistSlot;
    snapshot.assistPhase = data->tmAssistPhase;
    snapshot.assistTarget = {
        data->tmAssistTarget.x,
        data->tmAssistTarget.y,
    };
    snapshot.kickoffActive =
        data->kickoffOwnerLocked && data->isKickingOff;
    snapshot.kickoffOwnerId = data->kickoffOwnerId;
    snapshot.kickoffOwnerTerm = data->kickoffOwnerTerm;
    {
        std::lock_guard<std::mutex> cooperationLock(data->cooperationMutex);
        const FreeKickPlanState &plan = data->freeKickPlan;
        snapshot.freeKickProposerBootId = plan.proposerBootId;
        snapshot.freeKickRefereePacketNumber = plan.refereePacketNumber;
        snapshot.freeKickEventId = plan.eventId;
        snapshot.freeKickLeaderTerm = plan.leaderTerm;
        snapshot.freeKickType = plan.type;
        snapshot.freeKickPhase = plan.phase;
        snapshot.freeKickOurRestart = plan.ourRestart;
        snapshot.freeKickKickerId = plan.kickerId;
        snapshot.freeKickReceiverId = plan.receiverId;
        snapshot.freeKickBall = plan.ballSnapshot;
        snapshot.freeKickPassTarget = plan.passTarget;
        snapshot.freeKickAssistSlots = plan.assistSlots;
        if (plan.executionStartTime.nanoseconds() != 0 &&
            freekick_policy::isExecutionActive(plan.phase)) {
            const double elapsedMs = std::max(
                0.0, msecsSince(plan.executionStartTime));
            snapshot.freeKickExecutionElapsedMs = static_cast<uint32_t>(
                std::min(
                    elapsedMs,
                    static_cast<double>(
                        std::numeric_limits<uint32_t>::max())));
        }
    }
    std::lock_guard<std::mutex> lock(data->teamOutboundMutex);
    data->tmOutboundSnapshot = snapshot;
}

RoboCupGameControlReturnData Brain::gameControllerReturnSnapshot()
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    RoboCupGameControlReturnData snapshot;
    snapshot.playerNum = static_cast<uint8_t>(config->playerId);
    snapshot.teamNum = static_cast<uint8_t>(config->teamId);
    const bool recoveryTransitionActive = isRecoveryModeTransitionActive();
    const bool localizationHoldActive =
        recoveryLocalizationPreHoldActive_ || recoveryLocalizationHoldActive_;
    snapshot.fallen =
        data->recoveryState == RobotRecoveryState::IS_READY &&
        !recoveryTransitionActive && !localizationHoldActive
        ? 0
        : 1;

    const auto finiteOrZero = [](double value) {
        return std::isfinite(value) ? static_cast<float>(value) : 0.0F;
    };
    snapshot.pose[0] = finiteOrZero(data->robotPoseToField.x * 1000.0);
    snapshot.pose[1] = finiteOrZero(data->robotPoseToField.y * 1000.0);
    snapshot.pose[2] = finiteOrZero(data->robotPoseToField.theta);

    if (data->ballEverDetected.load()) {
        const double ballAgeSeconds = std::max(
            0.0, msecsSince(data->ball.timePoint) / 1000.0);
        snapshot.ballAge = finiteOrZero(ballAgeSeconds);
        snapshot.ball[0] = finiteOrZero(data->ball.posToRobot.x * 1000.0);
        snapshot.ball[1] = finiteOrZero(data->ball.posToRobot.y * 1000.0);
    }
    return snapshot;
}

void Brain::setupRecoveryModeMonitor()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    modeQueryPublisher_ =
        create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", qos);
    locoApiResponseSubscription_ = create_subscription<booster_msgs::msg::RpcRespMsg>(
        "LocoApiTopicResp", qos, bind(&Brain::locoApiResponseCallback, this, _1));
}

void Brain::sendModeQueryIfNeeded(const rclcpp::Time &now, bool force)
{
    if (modeQueryPublisher_ == nullptr) return;

    const int queryIntervalMs = std::max(
        50, static_cast<int>(get_parameter("recovery.mode_query_interval_ms").as_int()));
    const int queryTimeoutMs = std::max(
        100, static_cast<int>(get_parameter("recovery.mode_query_timeout_ms").as_int()));

    std::string uuid;
    {
        std::lock_guard<std::mutex> lock(recoveryModeMutex_);
        if (force) {
            // A phase boundary must be acknowledged by a query issued for that
            // phase. Invalidate both an in-flight UUID and an already-cached
            // response while holding the same mutex as the response callback.
            pendingModeQueryUuid_.clear();
            pendingModeQueryTime_ = rclcpp::Time(0, 0, now.get_clock_type());
            sdkRobotMode_ = booster::robot::RobotMode::kUnknown;
            sdkRobotModeStatus_ = -1;
            sdkRobotModeTime_ = rclcpp::Time(0, 0, now.get_clock_type());
        }
        if (!pendingModeQueryUuid_.empty()) {
            const bool timedOut =
                pendingModeQueryTime_.nanoseconds() > 0 &&
                pendingModeQueryTime_.get_clock_type() == now.get_clock_type() &&
                (now - pendingModeQueryTime_).nanoseconds() / 1000000 >= queryTimeoutMs;
            if (!timedOut) return;
            pendingModeQueryUuid_.clear();
            pendingModeQueryTime_ = rclcpp::Time(0, 0, now.get_clock_type());
            sdkRobotMode_ = booster::robot::RobotMode::kUnknown;
            sdkRobotModeStatus_ = -1;
            sdkRobotModeTime_ = rclcpp::Time(0, 0, now.get_clock_type());
        }

        if (!force && lastModeQueryTime_.nanoseconds() > 0 &&
            lastModeQueryTime_.get_clock_type() == now.get_clock_type() &&
            (now - lastModeQueryTime_).nanoseconds() / 1000000 < queryIntervalMs) {
            return;
        }

        uuid = gen_uuid();
        pendingModeQueryUuid_ = uuid;
        pendingModeQueryTime_ = now;
        lastModeQueryTime_ = now;
    }

    booster_msgs::msg::RpcReqMsg message;
    message.uuid = uuid;
    nlohmann::json header;
    header["api_id"] = static_cast<int64_t>(booster::robot::b1::LocoApiId::kGetMode);
    header["expect_response"] = true;
    message.header = header.dump();
    message.body = "";
    modeQueryPublisher_->publish(message);
}

void Brain::locoApiResponseCallback(const booster_msgs::msg::RpcRespMsg &msg)
{
    std::lock_guard<std::mutex> lock(recoveryModeMutex_);
    if (pendingModeQueryUuid_.empty() || msg.uuid != pendingModeQueryUuid_) return;
    pendingModeQueryUuid_.clear();

    int status = 0;
    if (!msg.header.empty() && !extractJsonIntField(msg.header, "status", status)) {
        sdkRobotModeStatus_ = -1;
        return;
    }
    sdkRobotModeStatus_ = status;
    if (status != 0) return;

    int mode = -1;
    if (!extractJsonIntField(msg.body, "mode", mode) ||
        mode < static_cast<int>(booster::robot::RobotMode::kDamping) ||
        mode > static_cast<int>(booster::robot::RobotMode::kSoccer)) {
        sdkRobotModeStatus_ = -1;
        return;
    }
    sdkRobotMode_ = static_cast<booster::robot::RobotMode>(mode);
    sdkRobotModeTime_ = get_clock()->now();
}

bool Brain::hasRecoveryModeSince(
    booster::robot::RobotMode mode, const rclcpp::Time &since) const
{
    std::lock_guard<std::mutex> lock(recoveryModeMutex_);
    return sdkRobotModeStatus_ == 0 &&
           sdkRobotMode_ == mode &&
           sdkRobotModeTime_.nanoseconds() > 0 &&
           sdkRobotModeTime_.get_clock_type() == since.get_clock_type() &&
           sdkRobotModeTime_.nanoseconds() >= since.nanoseconds();
}

void Brain::beginRecoveryPrepareSequence(booster::robot::b1::GetUpVersion version)
{
    std::lock_guard<std::mutex> lock(recoveryTransitionMutex_);
    const auto now = get_clock()->now();
    recoveryGetUpVersion_ = version;
    recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::WaitingForPrepare;
    recoveryModeTransitionStarted_ = now;
    recoveryLastModeQueryTime_ = rclcpp::Time(0, 0, now.get_clock_type());
    recoveryLastSoccerRequestTime_ = rclcpp::Time(0, 0, now.get_clock_type());
    recoverySoccerTimeoutLogged_ = false;
    recoveryGetUpPostureStableSinceUs_ = 0;
    recoveryPostureConfirmed_ = false;
    sendModeQueryIfNeeded(now, true);
    log->log("recovery", rerun::TextLog("Requested kPrepare; waiting for SDK confirmation"));
}

void Brain::beginRecoverySoccerTransition(bool waitForWalking)
{
    std::lock_guard<std::mutex> lock(recoveryTransitionMutex_);
    const auto now = get_clock()->now();
    recoveryModeTransitionPhase_ = waitForWalking
        ? RecoveryModeTransitionPhase::WaitingForWalking
        : RecoveryModeTransitionPhase::WaitingForSoccer;
    recoveryModeTransitionStarted_ = now;
    recoveryLastModeQueryTime_ = rclcpp::Time(0, 0, now.get_clock_type());
    recoveryLastSoccerRequestTime_ = waitForWalking
        ? rclcpp::Time(0, 0, now.get_clock_type())
        : now;
    recoverySoccerTimeoutLogged_ = false;
    if (!waitForWalking) {
        client->changeRobocupMode();
    }
    sendModeQueryIfNeeded(now, true);
}

void Brain::cancelRecoveryModeTransition()
{
    std::lock_guard<std::mutex> lock(recoveryTransitionMutex_);
    recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::Idle;
    recoveryGetUpPostureStableSinceUs_ = 0;
    recoveryPostureConfirmed_ = false;
}

void Brain::advanceRecoveryModeTransition(const rclcpp::Time &now)
{
    std::lock_guard<std::mutex> lock(recoveryTransitionMutex_);
    const int queryIntervalMs = std::max(
        50, static_cast<int>(get_parameter("recovery.mode_query_interval_ms").as_int()));
    if (recoveryLastModeQueryTime_.nanoseconds() == 0 ||
        recoveryLastModeQueryTime_.get_clock_type() != now.get_clock_type() ||
        (now - recoveryLastModeQueryTime_).nanoseconds() / 1000000 >= queryIntervalMs) {
        sendModeQueryIfNeeded(now);
        recoveryLastModeQueryTime_ = now;
    }

    // Keep the SDK mode cache fresh in normal play as well. Otherwise startup
    // and post-recovery liveness depend indefinitely on the legacy planner
    // index, whose values are not stable across SDK releases.
    if (recoveryModeTransitionPhase_ == RecoveryModeTransitionPhase::Idle) return;

    if (recoveryModeTransitionPhase_ == RecoveryModeTransitionPhase::WaitingForPrepare) {
        if (hasRecoveryModeSince(booster::robot::RobotMode::kPrepare,
                                 recoveryModeTransitionStarted_)) {
            recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::PrepareDwell;
            recoveryModeTransitionStarted_ = now;
            log->log("recovery", rerun::TextLog("SDK confirmed kPrepare; starting dwell"));
            return;
        }

        const int timeoutMs = std::max(
            500, static_cast<int>(get_parameter("recovery.prepare_confirm_timeout_ms").as_int()));
        if ((now - recoveryModeTransitionStarted_).nanoseconds() / 1000000 >= timeoutMs) {
            RCLCPP_ERROR(get_logger(), "Recovery aborted: kPrepare not confirmed in %d ms", timeoutMs);
            recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::Idle;
            data->recoveryState = RobotRecoveryState::HAS_FALLEN;
            ++data->recoveryPerformedRetryCount;
            data->recoveryPerformed = false;
            client->enterDamping();
        }
        return;
    }

    if (recoveryModeTransitionPhase_ == RecoveryModeTransitionPhase::PrepareDwell) {
        const int dwellMs = std::max(
            0, static_cast<int>(get_parameter("recovery.prepare_wait_ms").as_int()));
        if ((now - recoveryModeTransitionStarted_).nanoseconds() / 1000000 < dwellMs) return;

        recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::Idle;
        const int ret = client->requestGetUp(recoveryGetUpVersion_);
        if (ret != 0) {
            data->recoveryState = RobotRecoveryState::HAS_FALLEN;
            ++data->recoveryPerformedRetryCount;
            data->recoveryPerformed = false;
        }
        return;
    }

    if (recoveryModeTransitionPhase_ == RecoveryModeTransitionPhase::WaitingForWalking) {
        if (recoveryPostureConfirmed_ &&
            hasRecoveryModeSince(booster::robot::RobotMode::kSoccer,
                                 recoveryModeTransitionStarted_)) {
            beginPostGetupOdomSettling(now);
            recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::Idle;
            recoveryPostureConfirmed_ = false;
            log->log(
                "recovery",
                rerun::TextLog(
                    "SDK already confirmed kSoccer after GetUp; recovery complete"));
            return;
        }

        const bool walkingConfirmed = hasRecoveryModeSince(
            booster::robot::RobotMode::kWalking,
            recoveryModeTransitionStarted_);
        const int walkingTimeoutMs = std::max(
            500,
            static_cast<int>(get_parameter(
                "recovery.walking_confirm_timeout_ms").as_int()));
        const bool walkingTimedOut =
            (now - recoveryModeTransitionStarted_).nanoseconds() / 1000000 >=
            walkingTimeoutMs;
        if (!walkingConfirmed && !walkingTimedOut) {
            return;
        }
        if (!recoveryPostureConfirmed_ && !walkingTimedOut) {
            // kWalking can be observed before the body has settled. Keep the
            // localization hold and wait for a fresh one-second confirmation.
            return;
        }
        if (!recoveryPostureConfirmed_) {
            RCLCPP_ERROR(
                get_logger(),
                "Recovery aborted: strict posture was not confirmed before the kWalking deadline");
            recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::Idle;
            data->recoveryState = RobotRecoveryState::HAS_FALLEN;
            ++data->recoveryPerformedRetryCount;
            data->recoveryPerformed = false;
            recoveryGetUpPostureStableSinceUs_ = 0;
            recoveryPostureConfirmed_ = false;
            client->enterDamping();
            return;
        }

        const char *reason = walkingConfirmed
            ? "SDK confirmed kWalking; requesting kSoccer"
            : "kWalking confirmation timed out after strict posture confirmation; requesting kSoccer";
        log->log("recovery", rerun::TextLog(reason));
        if (!walkingConfirmed) {
            RCLCPP_WARN(
                get_logger(),
                "kWalking not confirmed in %d ms; strict posture is confirmed, requesting kSoccer",
                walkingTimeoutMs);
        }
        client->changeRobocupMode();
        recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::WaitingForSoccer;
        recoveryModeTransitionStarted_ = now;
        recoveryLastSoccerRequestTime_ = now;
        recoverySoccerTimeoutLogged_ = false;
        sendModeQueryIfNeeded(now, true);
        recoveryLastModeQueryTime_ = now;
        return;
    }

    if (recoveryModeTransitionPhase_ == RecoveryModeTransitionPhase::WaitingForSoccer) {
        if (hasRecoveryModeSince(booster::robot::RobotMode::kSoccer,
                                 recoveryLastSoccerRequestTime_)) {
            beginPostGetupOdomSettling(now);
            recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::Idle;
            recoveryPostureConfirmed_ = false;
            log->log("recovery", rerun::TextLog("SDK confirmed kSoccer; recovery complete"));
            return;
        }

        const int retryMs = std::max(
            200, static_cast<int>(get_parameter("recovery.soccer_switch_retry_ms").as_int()));
        const int timeoutMs = std::max(
            1000, static_cast<int>(get_parameter("recovery.soccer_switch_timeout_ms").as_int()));
        if (!recoverySoccerTimeoutLogged_ &&
            (now - recoveryModeTransitionStarted_).nanoseconds() / 1000000 >= timeoutMs) {
            recoverySoccerTimeoutLogged_ = true;
            RCLCPP_WARN(
                get_logger(), "kSoccer not confirmed after %d ms; continuing retries",
                timeoutMs);
        }
        if ((now - recoveryLastSoccerRequestTime_).nanoseconds() / 1000000 >= retryMs) {
            client->changeRobocupMode();
            recoveryLastSoccerRequestTime_ = now;
        }
    }
}

bool Brain::isRecoveryDampingMode() const
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const auto now = get_clock()->now();
    const int freshnessMs =
        std::max(50, static_cast<int>(get_parameter("recovery.mode_query_interval_ms").as_int())) +
        std::max(100, static_cast<int>(get_parameter("recovery.mode_query_timeout_ms").as_int())) + 500;
    {
        std::lock_guard<std::mutex> lock(recoveryModeMutex_);
        const bool freshSdkMode =
            sdkRobotModeStatus_ == 0 &&
            sdkRobotModeTime_.nanoseconds() > 0 &&
            sdkRobotModeTime_.get_clock_type() == now.get_clock_type() &&
            (now - sdkRobotModeTime_).nanoseconds() / 1000000 <= freshnessMs;
        if (freshSdkMode) {
            return sdkRobotMode_ == booster::robot::RobotMode::kDamping;
        }
    }
    return data->currentRobotModeIndex == 1;
}

bool Brain::isRecoveryOperationalMode() const
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const auto now = get_clock()->now();
    const int freshnessMs =
        std::max(50, static_cast<int>(get_parameter("recovery.mode_query_interval_ms").as_int())) +
        std::max(100, static_cast<int>(get_parameter("recovery.mode_query_timeout_ms").as_int())) + 500;
    {
        std::lock_guard<std::mutex> lock(recoveryModeMutex_);
        const bool freshSdkMode =
            sdkRobotModeStatus_ == 0 &&
            sdkRobotModeTime_.nanoseconds() > 0 &&
            sdkRobotModeTime_.get_clock_type() == now.get_clock_type() &&
            (now - sdkRobotModeTime_).nanoseconds() / 1000000 <= freshnessMs;
        if (freshSdkMode) {
            return sdkRobotMode_ == booster::robot::RobotMode::kWalking ||
                   sdkRobotMode_ == booster::robot::RobotMode::kSoccer;
        }
    }
    return data->currentRobotModeIndex == 8 || data->currentRobotModeIndex == 20;
}

bool Brain::isRecoveryModeTransitionActive()
{
    std::lock_guard<std::mutex> lock(recoveryTransitionMutex_);
    return recoveryModeTransitionPhase_ != RecoveryModeTransitionPhase::Idle;
}

bool Brain::isRecoveryActive()
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    return data->recoveryState == RobotRecoveryState::IS_FALLING ||
           data->recoveryState == RobotRecoveryState::HAS_FALLEN ||
           data->recoveryState == RobotRecoveryState::IS_GETTING_UP ||
           isRecoveryModeTransitionActive();
}

bool Brain::isRecoveryLocalizationHoldRequired()
{
    return isRecoveryActive() || recoveryLocalizationPreHoldActive_ ||
           recoveryPostGetupOdomSettleActive_;
}

void Brain::updateRecoveryLocalizationPreHold(double roll, double pitch)
{
    const auto now = get_clock()->now();
    if (!recoveryLocalizationPreHoldActive_ &&
        data->recoveryState == RobotRecoveryState::IS_READY &&
        !isRecoveryModeTransitionActive() &&
        fall_recovery_policy::shouldEnterLocalizationPreHold(roll, pitch)) {
        recoveryLocalizationPreHoldActive_ = true;
        recoveryLocalizationPreHoldReleaseCandidateAt_ =
            rclcpp::Time(0, 0, now.get_clock_type());
    }

    if (!recoveryLocalizationPreHoldActive_) return;

    if (isRecoveryActive() ||
        !fall_recovery_policy::isLocalizationPreHoldReleaseTilt(
            roll, pitch)) {
        recoveryLocalizationPreHoldReleaseCandidateAt_ =
            rclcpp::Time(0, 0, now.get_clock_type());
        return;
    }

    if (recoveryLocalizationPreHoldReleaseCandidateAt_.nanoseconds() <= 0 ||
        recoveryLocalizationPreHoldReleaseCandidateAt_.get_clock_type() !=
            now.get_clock_type()) {
        recoveryLocalizationPreHoldReleaseCandidateAt_ = now;
        return;
    }

    const int64_t stableUs =
        (now - recoveryLocalizationPreHoldReleaseCandidateAt_).nanoseconds() /
        1000;
    if (stableUs >=
        fall_recovery_policy::kLocalizationPreHoldReleaseConfirmUs) {
        recoveryLocalizationPreHoldActive_ = false;
        recoveryLocalizationPreHoldReleaseCandidateAt_ =
            rclcpp::Time(0, 0, now.get_clock_type());
    }
}

void Brain::beginPostGetupOdomSettling(const rclcpp::Time &now)
{
    if (now.nanoseconds() <= 0) return;

    recoveryPostGetupOdomSettleActive_ = true;
    recoveryPostGetupOdomSettleTimedOut_ = false;
    recoveryPostGetupOdomSettleStartedAt_ = now;
    recoveryLocalizationGuardStartedAt_ = now;
    recoveryPostGetupOdomSettleReason_ = "minimum_dwell";
    recoveryPostGetupOdomSamples_.clear();
    log->setTimeNow();
    log->log("recovery", rerun::TextLog(format(
        "Post-getup odometry settling started: minimum=%.0fms window=%.0fms maximum=%.0fms max_xy=%.3fm max_theta=%.2fdeg",
        recoveryPostGetupOdomSettleMinimumMs_,
        recoveryPostGetupOdomSettleWindowMs_,
        recoveryPostGetupOdomSettleMaximumMs_,
        recoveryPostGetupOdomSettleMaxXyRangeM_,
        recoveryPostGetupOdomSettleMaxThetaRangeDeg_)));
}

void Brain::updatePostGetupOdomSettling(const rclcpp::Time &now)
{
    if (!recoveryPostGetupOdomSettleActive_ ||
        recoveryPostGetupOdomSettleStartedAt_.nanoseconds() <= 0 ||
        recoveryPostGetupOdomSettleStartedAt_.get_clock_type() !=
            now.get_clock_type()) {
        return;
    }

    const double elapsedMs =
        (now - recoveryPostGetupOdomSettleStartedAt_).nanoseconds() / 1e6;
    const double windowMs = std::max(
        1.0, recoveryPostGetupOdomSettleWindowMs_);
    const int64_t firstEligibleNs = std::max(
        recoveryPostGetupOdomSettleStartedAt_.nanoseconds(),
        now.nanoseconds() - static_cast<int64_t>(windowMs * 1e6));

    bool odomFinite = false;
    bool allSamplesFinite = true;
    size_t sampleCount = 0;
    rclcpp::Time firstSample(0, 0, now.get_clock_type());
    rclcpp::Time lastSample(0, 0, now.get_clock_type());
    double xMin = std::numeric_limits<double>::infinity();
    double xMax = -std::numeric_limits<double>::infinity();
    double yMin = std::numeric_limits<double>::infinity();
    double yMax = -std::numeric_limits<double>::infinity();
    double thetaMin = std::numeric_limits<double>::infinity();
    double thetaMax = -std::numeric_limits<double>::infinity();
    double unwrappedTheta = 0.0;
    double previousTheta = 0.0;
    bool havePreviousTheta = false;

    for (const auto &sample : recoveryPostGetupOdomSamples_) {
        if (sample.stamp.nanoseconds() < firstEligibleNs ||
            sample.stamp.nanoseconds() > now.nanoseconds()) {
            continue;
        }
        if (sampleCount == 0) firstSample = sample.stamp;
        lastSample = sample.stamp;
        ++sampleCount;
        const auto &pose = sample.pose;
        if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
            !std::isfinite(pose.theta)) {
            allSamplesFinite = false;
            continue;
        }
        xMin = std::min(xMin, pose.x);
        xMax = std::max(xMax, pose.x);
        yMin = std::min(yMin, pose.y);
        yMax = std::max(yMax, pose.y);
        if (!havePreviousTheta) {
            unwrappedTheta = pose.theta;
            havePreviousTheta = true;
        } else {
            unwrappedTheta += fall_recovery_policy::normalizeAngle(
                pose.theta - previousTheta);
        }
        previousTheta = pose.theta;
        thetaMin = std::min(thetaMin, unwrappedTheta);
        thetaMax = std::max(thetaMax, unwrappedTheta);
    }

    odomFinite = sampleCount > 0 && havePreviousTheta && allSamplesFinite;
    const double coverageMs = sampleCount >= 2
        ? (lastSample - firstSample).nanoseconds() / 1e6
        : 0.0;
    const bool windowReady = sampleCount >= 10 &&
        coverageMs >= windowMs * 0.95;
    const double xRange = odomFinite ? std::max(0.0, xMax - xMin) : -1.0;
    const double yRange = odomFinite ? std::max(0.0, yMax - yMin) : -1.0;
    const double thetaRange = odomFinite
        ? std::max(0.0, thetaMax - thetaMin)
        : -1.0;
    const bool imuUpright = recoveryLatestImuValid_ &&
        fall_recovery_policy::bodyTilt(
            recoveryLatestImuRoll_, recoveryLatestImuPitch_) <=
            std::max(0.0, recoveryPostGetupOdomSettleMaxTiltRad_);
    const bool gyroQuiet = recoveryLatestImuValid_ &&
        recoveryLatestGyroMagnitude_ <= std::max(
            0.0, recoveryPostGetupOdomSettleMaxGyroRadPerSec_);

    const auto decision =
        fall_recovery_policy::evaluatePostGetupOdomSettling({
            elapsedMs,
            recoveryPostGetupOdomSettleMinimumMs_,
            recoveryPostGetupOdomSettleMaximumMs_,
            windowReady,
            odomFinite,
            imuUpright,
            gyroQuiet,
            xRange,
            yRange,
            thetaRange,
            recoveryPostGetupOdomSettleMaxXyRangeM_,
            recoveryPostGetupOdomSettleMaxThetaRangeDeg_ * M_PI / 180.0});
    recoveryPostGetupOdomSettleReason_ = decision.reason;
    if (decision.active) return;

    recoveryPostGetupOdomSettleActive_ = false;
    recoveryPostGetupOdomSettleTimedOut_ = decision.timedOut;
    const std::string details = format(
        "Post-getup odometry settling released: reason=%s timeout=%d elapsed=%.0fms coverage=%.0fms samples=%zu x_range=%.4fm y_range=%.4fm theta_range=%.3fdeg tilt=%.3f gyro=%.3f",
        decision.reason,
        decision.timedOut ? 1 : 0,
        elapsedMs,
        coverageMs,
        sampleCount,
        xRange,
        yRange,
        thetaRange >= 0.0 ? thetaRange * 180.0 / M_PI : -1.0,
        fall_recovery_policy::bodyTilt(
            recoveryLatestImuRoll_, recoveryLatestImuPitch_),
        recoveryLatestGyroMagnitude_);
    log->setTimeNow();
    log->log("recovery", rerun::TextLog(details));
    if (decision.timedOut) {
        RCLCPP_WARN(get_logger(), "%s", details.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "%s", details.c_str());
    }
}

void Brain::reanchorPoseToCurrentOdom(const Pose2D &pose)
{
    const double xOdomToRobot =
        -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x -
        sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    const double yOdomToRobot =
        sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x -
        cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;

    transCoord(
        xOdomToRobot, yOdomToRobot, -data->robotPoseToOdom.theta,
        pose.x, pose.y, pose.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta);
    data->robotPoseToField = pose;
    ++odomTransformRevision_;
    rememberOdomThetaAlignmentAnchor(pose, "RecoveryHoldReanchor");
}

void Brain::rememberOdomThetaAlignmentAnchor(
    const Pose2D &fieldPose,
    const std::string &source)
{
    if (!config->robotOdomThetaAutoAlign || odomThetaAlignmentLocked_) return;

    odomThetaAlignmentAnchorRawPose_ = data->robotPoseToOdom;
    odomThetaAlignmentAnchorRawPose_.theta = toPInPI(
        odomThetaAlignmentAnchorRawPose_.theta -
        odomThetaAlignmentOffset_);
    odomThetaAlignmentAnchorFieldPose_ = fieldPose;
    odomThetaAlignmentAnchorSource_ = source;
    hasOdomThetaAlignmentAnchor_ = true;
}

bool Brain::updateOdomThetaAlignment(
    const Pose2D &rawOdomPose,
    const VelocityCommandSnapshot &velocity,
    const rclcpp::Time &callbackTime,
    std::chrono::steady_clock::time_point callbackSteadyTime)
{
    if (odomThetaAlignmentLocked_ ||
        !config->robotOdomThetaAutoAlign) {
        return false;
    }

    constexpr auto sampleInterval = std::chrono::milliseconds(50);
    if (!hasOdomThetaAlignmentSample_) {
        odomThetaAlignmentPreviousRawPose_ = rawOdomPose;
        odomThetaAlignmentLastSampleTime_ = callbackSteadyTime;
        hasOdomThetaAlignmentSample_ = true;
        return false;
    }
    if (callbackSteadyTime - odomThetaAlignmentLastSampleTime_ <
        sampleInterval) {
        return false;
    }

    const Pose2D previousRawPose = odomThetaAlignmentPreviousRawPose_;
    odomThetaAlignmentPreviousRawPose_ = rawOdomPose;
    odomThetaAlignmentLastSampleTime_ = callbackSteadyTime;

    if (recoveryLocalizationHoldActive_ ||
        recoveryPostGetupOdomSettleActive_) {
        return false;
    }

    const double commandSpeed = std::hypot(velocity.sentX, velocity.sentY);
    const int64_t commandTimeNs = velocity.time.nanoseconds();
    const double commandAgeMs = commandTimeNs > 0
        ? static_cast<double>(callbackTime.nanoseconds() - commandTimeNs) / 1e6
        : std::numeric_limits<double>::infinity();
    const double deltaX = rawOdomPose.x - previousRawPose.x;
    const double deltaY = rawOdomPose.y - previousRawPose.y;
    const double deltaDistance = std::hypot(deltaX, deltaY);
    if (!std::isfinite(commandSpeed) || commandSpeed < 0.2 ||
        !std::isfinite(commandAgeMs) || commandAgeMs < 0.0 ||
        commandAgeMs > 250.0 || !std::isfinite(deltaDistance) ||
        deltaDistance < 0.001 || deltaDistance > 0.15) {
        return false;
    }

    const double deltaTheta = toPInPI(
        rawOdomPose.theta - previousRawPose.theta);
    const double rawHeadingMid = toPInPI(
        previousRawPose.theta + 0.5 * deltaTheta);
    const double commandDirection = std::atan2(
        velocity.sentY, velocity.sentX);
    const double rawMotionDirection = std::atan2(deltaY, deltaX);
    const double alignmentError = toPInPI(
        rawMotionDirection - rawHeadingMid - commandDirection);

    odomThetaAlignmentCosSum_ +=
        deltaDistance * std::cos(alignmentError);
    odomThetaAlignmentSinSum_ +=
        deltaDistance * std::sin(alignmentError);
    odomThetaAlignmentDistance_ += deltaDistance;
    odomThetaAlignmentConcentration_ =
        std::hypot(
            odomThetaAlignmentCosSum_,
            odomThetaAlignmentSinSum_) /
        odomThetaAlignmentDistance_;

    const double minimumDistance = std::max(
        0.05, config->robotOdomThetaAlignmentDistance);
    const double minimumConcentration = std::clamp(
        config->robotOdomThetaAlignmentMinConcentration, 0.0, 1.0);
    if (odomThetaAlignmentDistance_ < minimumDistance ||
        odomThetaAlignmentConcentration_ < minimumConcentration) {
        return false;
    }

    odomThetaAlignmentOffset_ = std::atan2(
        odomThetaAlignmentSinSum_, odomThetaAlignmentCosSum_);
    odomThetaAlignmentLocked_ = true;
    return true;
}

void Brain::applyOdomThetaAlignment(
    const Pose2D &fieldPoseBeforeAlignment,
    const rclcpp::Time &callbackTime,
    std::chrono::steady_clock::time_point callbackSteadyTime)
{
    const Pose2D transformBefore = data->odomToField;
    const uint64_t revisionBefore = odomTransformRevision_;
    const bool anchorUsed = hasOdomThetaAlignmentAnchor_;
    const Pose2D requestedFieldPose = anchorUsed
        ? odomThetaAlignmentAnchorFieldPose_
        : fieldPoseBeforeAlignment;

    if (anchorUsed) {
        Pose2D correctedAnchorOdom = odomThetaAlignmentAnchorRawPose_;
        correctedAnchorOdom.theta = toPInPI(
            correctedAnchorOdom.theta + odomThetaAlignmentOffset_);
        data->odomToField.theta = toPInPI(
            requestedFieldPose.theta - correctedAnchorOdom.theta);
        const double cosine = std::cos(data->odomToField.theta);
        const double sine = std::sin(data->odomToField.theta);
        data->odomToField.x = requestedFieldPose.x -
            cosine * correctedAnchorOdom.x +
            sine * correctedAnchorOdom.y;
        data->odomToField.y = requestedFieldPose.y -
            sine * correctedAnchorOdom.x -
            cosine * correctedAnchorOdom.y;
        transCoord(
            data->robotPoseToOdom.x,
            data->robotPoseToOdom.y,
            data->robotPoseToOdom.theta,
            data->odomToField.x,
            data->odomToField.y,
            data->odomToField.theta,
            data->robotPoseToField.x,
            data->robotPoseToField.y,
            data->robotPoseToField.theta);
        ++odomTransformRevision_;
    } else {
        reanchorPoseToCurrentOdom(fieldPoseBeforeAlignment);
    }

    RCLCPP_WARN(
        get_logger(),
        "Odom theta auto-aligned: offset=%.2f deg, concentration=%.3f, "
        "distance=%.3f m, anchor=%s",
        rad2deg(odomThetaAlignmentOffset_),
        odomThetaAlignmentConcentration_,
        odomThetaAlignmentDistance_,
        anchorUsed ? odomThetaAlignmentAnchorSource_.c_str() : "current_pose");
    log->setTimeNow();
    log->log(
        "events/odom_theta_auto_align",
        rerun::TextLog(format(
            "offset=%.2fdeg concentration=%.3f distance=%.3fm anchor=%s",
            rad2deg(odomThetaAlignmentOffset_),
            odomThetaAlignmentConcentration_,
            odomThetaAlignmentDistance_,
            anchorUsed ? odomThetaAlignmentAnchorSource_.c_str()
                       : "current_pose")));

    if (odomDiagnosticLog_) {
        OdomDiagnosticTransformEvent event;
        event.rosTimeNs = callbackTime.nanoseconds();
        event.systemTimeNs = systemTimeNs();
        event.steadyTimeNs = steadyTimeNs(callbackSteadyTime);
        event.source = "OdomThetaAutoAlign";
        event.applied = true;
        event.revisionBefore = revisionBefore;
        event.revisionAfter = odomTransformRevision_;
        event.requestedFieldPose = toDiagnosticPose(requestedFieldPose);
        event.odom = toDiagnosticPose(data->robotPoseToOdom);
        event.transformBefore = toDiagnosticPose(transformBefore);
        event.transformAfter = toDiagnosticPose(data->odomToField);
        event.fieldBefore = toDiagnosticPose(fieldPoseBeforeAlignment);
        event.fieldAfter = toDiagnosticPose(data->robotPoseToField);
        event.odomThetaAlignmentOffset = odomThetaAlignmentOffset_;
        event.odomThetaAlignmentDistance = odomThetaAlignmentDistance_;
        event.odomThetaAlignmentConcentration =
            odomThetaAlignmentConcentration_;
        event.odomThetaAlignmentLocked = odomThetaAlignmentLocked_;
        event.odomThetaAlignmentAnchorUsed = anchorUsed;
        event.odomThetaAlignmentAnchorSource = anchorUsed
            ? odomThetaAlignmentAnchorSource_
            : "current_pose";
        event.recoveryHoldActive = recoveryLocalizationHoldActive_;
        event.postGetupSettleActive = recoveryPostGetupOdomSettleActive_;
        odomDiagnosticLog_->enqueueTransformEvent(event);
    }

}

void Brain::updateRecoveryLocalizationHold()
{
    const bool holdRequired = isRecoveryLocalizationHoldRequired();
    if (holdRequired && !recoveryLocalizationHoldActive_) {
        recoveryLocalizationHoldActive_ = true;
        recoveryLocalizationHoldPose_ = data->robotPoseToField;
        recoveryLocalizationHoldStartOdomPose_ = data->robotPoseToOdom;
        recoveryLocalizationHoldStartFieldTheta_ =
            data->robotPoseToField.theta;
        recoveryLocalizationHoldSawRecovery_ = isRecoveryActive();
        recoveryLocalizationHoldPoseTrusted_ =
            tree->getEntry<bool>("odom_calibrated");
        if (!recoveryPostGetupOdomSettleActive_) {
            recoveryLocalizationGuardStartedAt_ = rclcpp::Time(
                0, 0, get_clock()->get_clock_type());
            recoveryPostGetupOdomSamples_.clear();
        }
        recoveryHeadingRealignAttemptsRemaining_ = 0;
        recoveryHeadingRealignCandidateConfirmations_ = 0;
        recoveryHeadingRealignAttemptInFlight_ = false;
        recoveryLocalizationReleasedAt_ =
            rclcpp::Time(0, 0, get_clock()->get_clock_type());
        log->setTimeNow();
        log->log("recovery", rerun::TextLog(format(
            "Localization hold started: pose=(%.2f, %.2f, %.2f)",
            recoveryLocalizationHoldPose_.x,
            recoveryLocalizationHoldPose_.y,
            recoveryLocalizationHoldPose_.theta)));
    }

    if (holdRequired) {
        recoveryLocalizationHoldSawRecovery_ =
            recoveryLocalizationHoldSawRecovery_ || isRecoveryActive();
        reanchorPoseToCurrentOdom(recoveryLocalizationHoldPose_);
        return;
    }

    if (recoveryLocalizationHoldActive_) {
        // Discard support-frame translation accumulated during get-up and
        // settling. Keep the net heading and rebase all axes to the final
        // stable raw odometry sample.
        reanchorPoseToCurrentOdom(recoveryLocalizationHoldPose_);
        recoveryLocalizationHoldActive_ = false;
        recoveryLocalizationReleasedAt_ = get_clock()->now();
        if (recoveryLocalizationGuardStartedAt_.nanoseconds() <= 0) {
            recoveryLocalizationGuardStartedAt_ =
                recoveryLocalizationReleasedAt_;
        }
        data->lastSuccessfulLocalizeTime = recoveryLocalizationReleasedAt_;
        recoveryHeadingRealignAttemptsRemaining_ =
            recoveryLocalizationHoldSawRecovery_ &&
                    recoveryLocalizationHoldPoseTrusted_
                ? std::max(
                      0,
                      static_cast<int>(get_parameter(
                          "recovery.heading_realign_attempts").as_int()))
                : 0;
        recoveryHeadingRealignCandidateConfirmations_ = 0;
        recoveryHeadingRealignCandidateDelta_ = 0.0;
        recoveryHeadingRealignAttemptInFlight_ = false;
        recoveryHeadingRealignLastAttemptAt_ = rclcpp::Time(
            0, 0, recoveryLocalizationReleasedAt_.get_clock_type());

        // Observations captured while the camera was moving through get-up can
        // otherwise trigger a large visual correction immediately after release.
        data->setMarkings({});
        data->setFieldLines({});
        data->setGoalposts({});
        log->setTimeNow();
        log->log("recovery", rerun::TextLog(format(
            "Localization hold released after recovery: pose=(%.2f, %.2f, %.2f)",
            recoveryLocalizationHoldPose_.x,
            recoveryLocalizationHoldPose_.y,
            recoveryLocalizationHoldPose_.theta)));
        recoveryLocalizationHoldSawRecovery_ = false;
        recoveryLocalizationHoldPoseTrusted_ = false;
    }
}

bool Brain::isRecoveryLocalizationBlocked()
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    updateRecoveryLocalizationHold();
    const int resumeDelayMs = std::max(
        0, static_cast<int>(
               get_parameter("recovery.localization_resume_delay_ms").as_int()));
    const rclcpp::Time &resumeDelayStartedAt =
        recoveryLocalizationGuardStartedAt_.nanoseconds() > 0
        ? recoveryLocalizationGuardStartedAt_
        : recoveryLocalizationReleasedAt_;
    const bool resumeDelayActive =
        resumeDelayStartedAt.nanoseconds() > 0 &&
        msecsSince(resumeDelayStartedAt) < resumeDelayMs;
    const bool blocked = recoveryLocalizationHoldActive_ || resumeDelayActive;
    if (blocked) {
        data->setMarkings({});
        data->setFieldLines({});
        data->setGoalposts({});
    }
    return blocked;
}

bool Brain::isPostRecoveryHeadingRealignActive()
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    if (recoveryHeadingRealignAttemptsRemaining_ <= 0 ||
        recoveryLocalizationReleasedAt_.nanoseconds() <= 0) {
        return false;
    }
    const int windowMs = std::max(
        0,
        static_cast<int>(get_parameter(
            "recovery.heading_realign_window_ms").as_int()));
    if (msecsSince(recoveryLocalizationReleasedAt_) >= windowMs) {
        recoveryHeadingRealignAttemptsRemaining_ = 0;
        recoveryHeadingRealignCandidateConfirmations_ = 0;
        recoveryHeadingRealignAttemptInFlight_ = false;
        return false;
    }
    return true;
}

bool Brain::beginPostRecoveryHeadingRealignAttempt()
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    if (!isPostRecoveryHeadingRealignActive()) return false;

    const int intervalMs = std::max(
        50,
        static_cast<int>(get_parameter(
            "recovery.heading_realign_interval_ms").as_int()));
    if (recoveryHeadingRealignLastAttemptAt_.nanoseconds() > 0 &&
        msecsSince(recoveryHeadingRealignLastAttemptAt_) < intervalMs) {
        return false;
    }
    recoveryHeadingRealignLastAttemptAt_ = get_clock()->now();
    --recoveryHeadingRealignAttemptsRemaining_;
    recoveryHeadingRealignAttemptInFlight_ = true;
    return true;
}

bool Brain::applyPostRecoveryHeadingRealignCandidate(
    const Pose2D &candidatePose, double residual)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    if (!recoveryHeadingRealignAttemptInFlight_) {
        return false;
    }
    recoveryHeadingRealignAttemptInFlight_ = false;

    const int windowMs = std::max(
        0,
        static_cast<int>(get_parameter(
            "recovery.heading_realign_window_ms").as_int()));
    if (recoveryLocalizationHoldActive_ ||
        isRecoveryLocalizationHoldRequired() ||
        recoveryLocalizationReleasedAt_.nanoseconds() <= 0 ||
        msecsSince(recoveryLocalizationReleasedAt_) >= windowMs) {
        recoveryHeadingRealignAttemptsRemaining_ = 0;
        recoveryHeadingRealignCandidateConfirmations_ = 0;
        return false;
    }

    const Pose2D currentPose = data->robotPoseToField;
    const double maxPositionDelta = std::max(
        0.0,
        get_parameter(
            "recovery.heading_realign_max_candidate_pos_m").as_double());
    if (!fall_recovery_policy::isSafeHeadingRealignCandidate(
            recoveryLocalizationHoldPose_.x,
            currentPose.x,
            currentPose.y,
            candidatePose.x,
            candidatePose.y,
            maxPositionDelta)) {
        recoveryHeadingRealignCandidateConfirmations_ = 0;
        return false;
    }

    const double maxHeadingDelta = std::max(
        0.0,
        get_parameter("recovery.heading_realign_max_delta_rad").as_double());
    if (!std::isfinite(currentPose.theta) ||
        !std::isfinite(candidatePose.theta)) {
        recoveryHeadingRealignCandidateConfirmations_ = 0;
        return false;
    }
    const double candidateDelta = toPInPI(
        candidatePose.theta - currentPose.theta);
    if (!std::isfinite(candidateDelta) ||
        std::fabs(candidateDelta) > maxHeadingDelta + 1e-6) {
        recoveryHeadingRealignCandidateConfirmations_ = 0;
        return false;
    }

    const int requiredConfirmations = std::max(
        1,
        static_cast<int>(get_parameter(
            "recovery.heading_realign_confirmations").as_int()));
    const double consistency = std::max(
        0.0,
        get_parameter(
            "recovery.heading_realign_consistency_rad").as_double());
    if (recoveryHeadingRealignCandidateConfirmations_ == 0 ||
        !fall_recovery_policy::areHeadingCandidatesConsistent(
            recoveryHeadingRealignCandidateDelta_,
            candidateDelta,
            consistency)) {
        recoveryHeadingRealignCandidateDelta_ = candidateDelta;
        recoveryHeadingRealignCandidateConfirmations_ = 1;
    } else {
        recoveryHeadingRealignCandidateDelta_ = toPInPI(
            recoveryHeadingRealignCandidateDelta_ +
            0.5 * toPInPI(
                candidateDelta - recoveryHeadingRealignCandidateDelta_));
        ++recoveryHeadingRealignCandidateConfirmations_;
    }
    if (recoveryHeadingRealignCandidateConfirmations_ <
        requiredConfirmations) {
        return false;
    }

    const double headingDelta = recoveryHeadingRealignCandidateDelta_;
    const double minHeadingDelta = std::max(
        0.0,
        get_parameter("recovery.heading_realign_min_delta_rad").as_double());
    recoveryHeadingRealignAttemptsRemaining_ = 0;
    recoveryHeadingRealignCandidateConfirmations_ = 0;
    if (std::fabs(headingDelta) < minHeadingDelta) {
        log->setTimeNow();
        log->log(
            "recovery",
            rerun::TextLog(format(
                "Post-recovery heading verified without correction: delta=%.3f residual=%.3f",
                headingDelta,
                residual)));
        return false;
    }

    const double correctedTheta = toPInPI(
        currentPose.theta +
        cap(headingDelta, maxHeadingDelta, -maxHeadingDelta));
    calibrateOdom(
        currentPose.x,
        currentPose.y,
        correctedTheta,
        "PostRecoveryHeadingRealign");
    data->lastSuccessfulLocalizeTime = get_clock()->now();
    log->setTimeNow();
    log->log(
        "recovery",
        rerun::TextLog(format(
            "Post-recovery heading realigned: old=%.3f candidate=%.3f new=%.3f delta=%.3f residual=%.3f",
            currentPose.theta,
            toPInPI(currentPose.theta + headingDelta),
            correctedTheta,
            headingDelta,
            residual)));
    return true;
}

void Brain::pubKickMsg() {
    if (!pubKickBall) return;
    if (!data->ballDetected) return;
    brain::msg::Kick kickMsg;
    kickMsg.header.stamp = get_clock()->now();
    kickMsg.x = data->ball.posToRobot.x;
    kickMsg.y = data->ball.posToRobot.y;
    kickMsg.dir = toPInPI(data->kickDir - data->robotPoseToField.theta);

    double goal_x = config->fieldDimensions.length / 2;
    double goal_y = 0.0;
    Pose2D goalPose;
    goalPose.x = goal_x;
    goalPose.y = goal_y;
    double ball_x = data->ball.posToField.x;
    double ball_y = data->ball.posToField.y;
    double dist = std::sqrt((goal_x - ball_x) * (goal_x - ball_x) + (goal_y - ball_y) * (goal_y - ball_y));
    dist = std::abs(dist);
    double power = 0.0;

    if (dist > 6.0) {
        power = 2.0;
    } else {
        power = 6.0;
    }
    kickMsg.power = power;

    auto goalPose_r = data->field2robot(goalPose);
    kickMsg.goal_x = goalPose_r.x;
    kickMsg.goal_y = goalPose_r.y;

    kickMsg.robot_theta_to_field = data->robotPoseToField.theta;

    pubKickBall->publish(kickMsg);
}

void Brain::handleSpecialStates() {
    string gameState = tree->getEntry<string>("gc_game_state");
    bool isKickoffSide = tree->getEntry<bool>("gc_is_kickoff_side");
    auto now = get_clock()->now();

    const bool ownCenterKickoffSet =
        data->realGameSubState == "NONE" &&
        gameState == "SET" && isKickoffSide;
    if (ownCenterKickoffSet && !data->kickoffSetWasActive) {
        data->isKickingOff = true;
        data->kickoffStartTime = now;
        data->kickoffOwnerLocked = false;
        data->kickoffOwnerId = 0;
        data->kickoffOwnerTerm = 0;
        data->kickoffBallSnapshotValid = false;
    }
    data->kickoffSetWasActive = ownCenterKickoffSet;

    const double kickoffTimeoutMs = std::max(
        1000.0,
        get_parameter("strategy.cooperation.kickoff_timeout_ms").as_double());
    if (data->isKickingOff &&
        (!isKickoffSide || data->realGameSubState != "NONE" ||
         msecsSince(data->kickoffStartTime) > kickoffTimeoutMs)) {
        data->isKickingOff = false;
        data->kickoffOwnerLocked = false;
        data->kickoffOwnerId = 0;
        data->kickoffOwnerTerm = 0;
        data->kickoffBallSnapshotValid = false;
    }

    static int lastScore = 0;
    if (data->score > lastScore) {
        tree->setEntry<bool>("we_just_scored", true);
        lastScore = data->score;
    }
    if (gameState == "SET") {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

void Brain::handleCooperation() {
    const bool logCooperationDebug =
        log->shouldLog("cooperation_debug", config->rerunLogDebugHz);
    const bool logCooperationVisual =
        log->shouldLog("cooperation_visual", config->rerunLogVisualHz);
    const bool logCooperationTimeseries =
        log->shouldLog("cooperation_timeseries", config->rerunLogTimeseriesHz);
    auto log_ = [=](string msg) {
        if (logCooperationDebug) {
            log->setTimeNow();
            log->log("debug/handleCooperation", rerun::TextLog(msg));
        }
    };
    log_("handle cooperation");

    std::lock_guard<std::mutex> cooperationLock(data->cooperationMutex);

    const double COM_TIMEOUT = std::max(
        100.0,
        get_parameter("strategy.cooperation.tactical_packet_timeout_ms").as_double());

    const int selfId = config->playerId;
    const int selfIdx = selfId - 1;

    vector<int> aliveTmIdxs = {};
    data->tmImAlive =
        data->penalty[selfIdx] == PENALTY_NONE &&
        tree->getEntry<bool>("odom_calibrated") &&
        data->recoveryState == RobotRecoveryState::IS_READY &&
        !isRecoveryLocalizationBlocked() &&
        isRecoveryOperationalMode() &&
        std::isfinite(data->robotPoseToField.x) &&
        std::isfinite(data->robotPoseToField.y) &&
        std::isfinite(data->robotPoseToField.theta);

    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    {
        std::lock_guard<std::mutex> teamStatusLock(data->teamStatusMutex);
        std::copy(
            std::begin(data->tmStatus),
            std::end(data->tmStatus),
            teamStatuses.begin());
    }
    for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
        if (i == selfIdx) continue;
        if (data->penalty[i] != PENALTY_NONE ||
            teamStatuses[i].ageMs() > COM_TIMEOUT) {
            teamStatuses[i].isAlive = false;
            teamStatuses[i].isLead = false;
        }
    }
    updateCostToKick(teamStatuses);
    log_(format("ImAlive: %d, myCost: %.1f", data->tmImAlive, data->tmMyCost));


    int gcAliveCount = 0;
    for (int i = 0; i < MAX_NUM_PLAYERS; i++)
    {
        if (data->penalty[i] == PENALTY_NONE) {
            gcAliveCount++;

            int tmId = i + 1;
            if (tmId == config->playerId) continue;

            auto tmStatus = teamStatuses[i];
            if (logCooperationVisual) {
                log->setTimeNow();
                auto color = 0x00FFFFFF;
                if (!tmStatus.isAlive) color = 0x006666FF;
                else if (!tmStatus.isLead) color = 0x00CCCCFF;
                string label = format("ID: %d, Cost: %.1f", tmId, tmStatus.cost);
                log->logRobot(format("field/teammate-%d", tmId).c_str(), tmStatus.robotPoseToField, color, label);
                log->logBall(
                    format("tm_ball-%d", tmId).c_str(),
                    tmStatus.ballPosToField,
                    tmStatus.ballDetected ? 0x00FFFFFF : (tmStatus.isAlive ? 0x006666FF : 0x003333FF),
                    tmStatus.ballConfidence,
                    tmStatus.ballLocationKnown
                );
            }
        }
    }
    log_(format("gcAliveCnt: %d", gcAliveCount));

    for (int i = 0; i < MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        if (teamStatuses[i].isAlive) {
            aliveTmIdxs.push_back(i);
            if (logCooperationTimeseries) {
                log->setTimeNow();
                log->log(format("debug/tm_cost_scalar_%d", i + 1), rerun::Scalar(teamStatuses[i].cost));
                log->log(format("debug/tm_lead_scalar_%d", i + 1), rerun::Scalar(teamStatuses[i].isLead));
            }
        }
    }
    log_(format("alive TM Count: %d", aliveTmIdxs.size()));

    // Log active teammates.
    log_(format("Self: cost: %.1f, isLead: %d", data->tmMyCost, data->tmImLead));


    static rclcpp::Time lastTmBallPosTime = get_clock()->now();
    const double TM_BALL_TIMEOUT = 1000.;
    const double RANGE_THRESHOLD = config->tmBallDistThreshold;
    int trustedTMIdx = -1;
    double minRange = 1e6;
    log_(format("Find ball info among %d alive TMs", aliveTmIdxs.size()));
    for (int i = 0; i < aliveTmIdxs.size(); i++) {
        auto status = teamStatuses[aliveTmIdxs[i]];
        log_(format("TM %d, ballDetected: %d, ballRange: %.1f", i + 1, status.ballDetected, status.ballRange));
        if (status.ballDetected && status.ballRange < minRange) {
            log_(format("tm ball range(%.1f) < minRange(%.1f)", status.ballRange, minRange));
            double dist = norm(status.ballPosToField.x - data->robotPoseToField.x, status.ballPosToField.y - data->robotPoseToField.y);
            if (dist > RANGE_THRESHOLD) {
                log_(format("tm ball dist to me(%.1f) > threshold(%.1f), TM %d can be trusted", dist, RANGE_THRESHOLD, i+ 1));
                minRange = status.ballRange;
                trustedTMIdx = aliveTmIdxs[i];
            }  else {
                log_(format("tm ball dist to me(%.1f) < threshold(%.1f), TM %d can NOT be trusted", dist, RANGE_THRESHOLD, i+ 1));
            }
        }
    }
    if (trustedTMIdx >= 0) {
        log_(format("Reliable tm ball found. PlayerID = %d", trustedTMIdx + 1));
        data->tmBall.posToField = teamStatuses[trustedTMIdx].ballPosToField;
        updateRelativePos(data->tmBall);

        tree->setEntry<bool>("tm_ball_pos_reliable", true);
        lastTmBallPosTime = get_clock()->now();
        if (!tree->getEntry<bool>("ball_location_known")) {
            log_("update ball.posToField");
            data->ball.posToField = data->tmBall.posToField;
            updateRelativePos(data->ball);
        }
    } else {
        log_("TM reported NO BALL or can not be trusted");
        if (msecsSince(lastTmBallPosTime) > TM_BALL_TIMEOUT) {
            log_("TM ball timeout reached");
            tree->setEntry<bool>("tm_ball_pos_reliable", false);
        }
    }

    updatePlayerRoleForTeamAvailability(teamStatuses);

    constexpr double COST_RANK_RESOLUTION = 0.05;
    const string selfRole = tree->getEntry<string>("player_role");
    const double selfCost = std::isfinite(data->tmMyCost) ? data->tmMyCost : 1e9;
    struct BallOwnerCandidate {
        int id;
        double cost;
    };
    const auto candidateBetter = [=](const BallOwnerCandidate &lhs,
                                     const BallOwnerCandidate &rhs) {
        const long long lhsCostRank = std::llround(lhs.cost / COST_RANK_RESOLUTION);
        const long long rhsCostRank = std::llround(rhs.cost / COST_RANK_RESOLUTION);
        return lhsCostRank < rhsCostRank ||
               (lhsCostRank == rhsCostRank && lhs.id < rhs.id);
    };

    vector<BallOwnerCandidate> strikerCandidates;
    int myStrikerIDRank = 0;
    int assignedGoalkeeperId = data->tmAssignedGoalkeeperId;
    if (assignedGoalkeeperId < 1 || assignedGoalkeeperId > config->numOfPlayers) {
        assignedGoalkeeperId = data->gcGoalkeeperId;
    }
    if (data->tmImAlive && selfRole == "striker") {
        strikerCandidates.push_back({selfId, selfCost});
    }
    for (int tmIdx : aliveTmIdxs) {
        const auto &tmStatus = teamStatuses[tmIdx];
        if (tmStatus.role != "striker") continue;
        const double tmCost = std::isfinite(tmStatus.cost) ? tmStatus.cost : 1e9;
        strikerCandidates.push_back({tmIdx + 1, tmCost});
    }
    // READY placement must remain deterministic even while team packets are
    // delayed. Count every non-penalized player before us except the currently
    // assigned goalkeeper; an explicit remote goalkeeper role is also excluded
    // when role reassignment has not yet propagated locally.
    if (data->tmImAlive && selfRole == "striker") {
        for (int playerId = 1; playerId < selfId &&
             playerId <= std::min(config->numOfPlayers, MAX_NUM_PLAYERS);
             ++playerId) {
            if (data->penalty[playerId - 1] != PENALTY_NONE ||
                playerId == assignedGoalkeeperId) {
                continue;
            }
            if (teamStatuses[playerId - 1].role == "goal_keeper") continue;
            ++myStrikerIDRank;
        }
    }
    std::sort(strikerCandidates.begin(), strikerCandidates.end(), candidateBetter);
    data->tmMyCostRank = -1;
    for (size_t rank = 0; rank < strikerCandidates.size(); rank++) {
        if (strikerCandidates[rank].id == selfId) {
            data->tmMyCostRank = static_cast<int>(rank);
            break;
        }
    }
    data->myStrikerIDRank = myStrikerIDRank;

    vector<BallOwnerCandidate> ownerCandidates;
    const bool goalieCanContestNow =
        tree->getEntry<string>("gc_game_state") == "PLAY" &&
        tree->getEntry<string>("gc_game_sub_state_type") == "NONE";
    const bool selfGoalieClaimsBall =
        goalieCanContestNow &&
        selfRole == "goal_keeper" &&
        canGoalkeeperClaimBall(
            data->ballDetected,
            data->ball.range,
            data->ball.posToField.x,
            data->ball.posToField.y,
            selfCost);
    if (data->tmImAlive && (selfRole == "striker" || selfGoalieClaimsBall)) {
        ownerCandidates.push_back({selfId, selfCost});
    }
    for (int tmIdx : aliveTmIdxs) {
        const auto &tmStatus = teamStatuses[tmIdx];
        const double tmCost = std::isfinite(tmStatus.cost) ? tmStatus.cost : 1e9;
        const bool goalieClaimsBall =
            goalieCanContestNow &&
            tmStatus.role == "goal_keeper" &&
            canGoalkeeperClaimBall(
                tmStatus.ballDetected,
                tmStatus.ballRange,
                tmStatus.ballPosToField.x,
                tmStatus.ballPosToField.y,
                tmCost);
        if (tmStatus.role == "striker" || goalieClaimsBall) {
            ownerCandidates.push_back({tmIdx + 1, tmCost});
        }
    }
    std::sort(ownerCandidates.begin(), ownerCandidates.end(), candidateBetter);

    const auto candidateForId = [&](int id) -> const BallOwnerCandidate * {
        auto found = std::find_if(
            ownerCandidates.begin(), ownerCandidates.end(),
            [id](const BallOwnerCandidate &candidate) { return candidate.id == id; });
        return found == ownerCandidates.end() ? nullptr : &(*found);
    };

    const int oldOwnerId = data->tmBallOwnerId;
    const uint32_t oldLeaderTerm = data->tmLeaderTerm;
    if (oldOwnerId == 0 &&
        data->tmOwnerlessSince == SteadyClock::time_point{}) {
        data->tmOwnerlessSince = SteadyClock::now();
    } else if (oldOwnerId != 0) {
        data->tmOwnerlessSince = {};
    }
    int selectedOwnerId = oldOwnerId;
    uint32_t selectedLeaderTerm = data->tmLeaderTerm;
    uint32_t maxSeenLeaderTerm = data->tmLeaderTerm;
    for (int tmIdx : aliveTmIdxs) {
        maxSeenLeaderTerm = std::max(
            maxSeenLeaderTerm, teamStatuses[tmIdx].leaderTerm);
    }

    // Prefer a valid self-claim with a higher term to converge after UDP delay or a transient partition.
    int claimedOwnerId = 0;
    uint32_t claimedLeaderTerm = selectedLeaderTerm;
    if (selectedOwnerId == selfId && candidateForId(selfId) != nullptr) {
        claimedOwnerId = selfId;
    }
    for (int tmIdx : aliveTmIdxs) {
        const int tmId = tmIdx + 1;
        const auto &status = teamStatuses[tmIdx];
        if (status.ballOwnerId != tmId || candidateForId(tmId) == nullptr) continue;
        if (status.leaderTerm > claimedLeaderTerm ||
            (status.leaderTerm == claimedLeaderTerm &&
             (claimedOwnerId == 0 || tmId < claimedOwnerId))) {
            claimedLeaderTerm = status.leaderTerm;
            claimedOwnerId = tmId;
        }
    }

    bool selectedOwnerClaimValid = false;
    if (selectedOwnerId == selfId) {
        selectedOwnerClaimValid = candidateForId(selfId) != nullptr;
    } else if (selectedOwnerId > 0 && selectedOwnerId <= MAX_NUM_PLAYERS) {
        const auto &selectedStatus = teamStatuses[selectedOwnerId - 1];
        selectedOwnerClaimValid =
            candidateForId(selectedOwnerId) != nullptr &&
            selectedStatus.ballOwnerId == selectedOwnerId &&
            selectedStatus.leaderTerm == selectedLeaderTerm;
    }

    bool adoptedNetworkClaim = false;
    if (claimedOwnerId != 0 &&
        (claimedLeaderTerm > selectedLeaderTerm ||
         (claimedLeaderTerm == selectedLeaderTerm &&
          (!selectedOwnerClaimValid || claimedOwnerId < selectedOwnerId)))) {
        selectedOwnerId = claimedOwnerId;
        selectedLeaderTerm = claimedLeaderTerm;
        adoptedNetworkClaim = true;
    } else {
        const BallOwnerCandidate *currentOwner =
            selectedOwnerClaimValid ? candidateForId(selectedOwnerId) : nullptr;
        const BallOwnerCandidate *bestCandidate = ownerCandidates.empty()
            ? nullptr
            : &ownerCandidates.front();
        if (currentOwner == nullptr) {
            // Release a stale leader immediately. Only the best local candidate may publish a new term;
            // followers do not claim leadership for remote robots from asynchronous costs. If cost
            // snapshots remain inconsistent, the lowest-ID candidate prevents an all-assist deadlock.
            int fallbackOwnerId = 0;
            for (const auto &candidate : ownerCandidates) {
                if (fallbackOwnerId == 0 || candidate.id < fallbackOwnerId) {
                    fallbackOwnerId = candidate.id;
                }
            }
            const double ownerlessFallbackMs = std::max(
                0.0,
                get_parameter(
                    "strategy.cooperation.leader_ownerless_fallback_ms")
                    .as_double());
            const bool fallbackReady =
                oldOwnerId == 0 &&
                steadyMsecsSince(data->tmOwnerlessSince) >=
                    ownerlessFallbackMs;
            const bool selfMayClaim =
                bestCandidate != nullptr &&
                (bestCandidate->id == selfId ||
                 (fallbackReady && fallbackOwnerId == selfId));
            selectedOwnerId = selfMayClaim ? selfId : 0;
            data->tmPendingBallOwnerId = 0;
            data->tmPendingOwnerSince = {};
        } else if (bestCandidate == nullptr || bestCandidate->id == currentOwner->id) {
            data->tmPendingBallOwnerId = 0;
            data->tmPendingOwnerSince = {};
        } else {
            const double costMargin = std::max(
                0.0,
                get_parameter("strategy.cooperation.leader_switch_cost_margin").as_double());
            const double costRatio = std::clamp(
                get_parameter("strategy.cooperation.leader_switch_cost_ratio").as_double(),
                0.1,
                1.0);
            const double minimumHoldMs = std::max(
                0.0,
                get_parameter("strategy.cooperation.leader_min_hold_ms").as_double());
            const double confirmationMs = std::max(
                0.0,
                get_parameter("strategy.cooperation.leader_switch_confirm_ms").as_double());
            const bool leaseExpired =
                steadyMsecsSince(data->tmOwnerAdoptedAt) >= minimumHoldMs;
            const bool challengerDecisivelyBetter =
                bestCandidate->cost + costMargin < currentOwner->cost &&
                bestCandidate->cost <= currentOwner->cost * costRatio;

            if (bestCandidate->id != selfId ||
                !leaseExpired || !challengerDecisivelyBetter) {
                data->tmPendingBallOwnerId = 0;
                data->tmPendingOwnerSince = {};
            } else if (data->tmPendingBallOwnerId != bestCandidate->id) {
                data->tmPendingBallOwnerId = bestCandidate->id;
                data->tmPendingOwnerSince = SteadyClock::now();
            } else if (steadyMsecsSince(data->tmPendingOwnerSince) >= confirmationMs) {
                selectedOwnerId = bestCandidate->id;
                data->tmPendingBallOwnerId = 0;
                data->tmPendingOwnerSince = {};
            }
        }
    }

    const bool ownerChanged = selectedOwnerId != oldOwnerId;
    if (ownerChanged) {
        data->tmPreviousBallOwnerId = oldOwnerId;
        data->tmBallOwnerId = selectedOwnerId;
        data->tmOwnerAdoptedAt = SteadyClock::now();
        data->tmPendingBallOwnerId = 0;
        data->tmPendingOwnerSince = {};
        if (selectedOwnerId == selfId && !adoptedNetworkClaim) {
            selectedLeaderTerm = maxSeenLeaderTerm + 1;
        }
        data->tmLeaderTerm = selectedLeaderTerm;
    } else if (adoptedNetworkClaim) {
        data->tmLeaderTerm = selectedLeaderTerm;
        data->tmOwnerAdoptedAt = SteadyClock::now();
        data->tmPendingBallOwnerId = 0;
        data->tmPendingOwnerSince = {};
    }
    // After restart, the previous leader may self-elect with a low term before receiving the retained
    // higher term. Even with the same owner ID, it must cross this watermark or followers reject it forever.
    if (data->tmBallOwnerId == selfId &&
        candidateForId(selfId) != nullptr &&
        maxSeenLeaderTerm > data->tmLeaderTerm) {
        data->tmLeaderTerm = maxSeenLeaderTerm ==
                std::numeric_limits<uint32_t>::max()
            ? maxSeenLeaderTerm
            : maxSeenLeaderTerm + 1;
        data->tmOwnerAdoptedAt = SteadyClock::now();
        data->tmPendingBallOwnerId = 0;
        data->tmPendingOwnerSince = {};
    }
    if (data->tmBallOwnerId == 0) {
        if (data->tmOwnerlessSince == SteadyClock::time_point{}) {
            data->tmOwnerlessSince = SteadyClock::now();
        }
    } else {
        data->tmOwnerlessSince = {};
    }
    data->tmImLead =
        data->tmImAlive && data->tmBallOwnerId == selfId;
    const bool leaderTermChanged = data->tmLeaderTerm != oldLeaderTerm;
    tree->setEntry<bool>("is_lead", data->tmImLead);

    // The leader is the sole formation coordinator. Recompute only when ownership, membership, or mapping changes.
    vector<int> assistantIds;
    for (const auto &candidate : strikerCandidates) {
        if (candidate.id != data->tmBallOwnerId) assistantIds.push_back(candidate.id);
    }
    std::sort(assistantIds.begin(), assistantIds.end());
    if (assistantIds.size() > 4) assistantIds.resize(4);

    uint32_t rosterMask = 0;
    for (int id : assistantIds) rosterMask |= (1u << (id - 1));

    std::array<Pose2D, MAX_NUM_PLAYERS> playerPoses{};
    playerPoses[selfIdx] = data->robotPoseToField;
    for (int tmIdx : aliveTmIdxs) {
        playerPoses[tmIdx] = teamStatuses[tmIdx].robotPoseToField;
    }
    Pose2D leaderPose = data->robotPoseToField;
    double leaderKickDir = data->kickDir;
    Point formationBall = data->ball.posToField;
    if (data->tmBallOwnerId > 0 && data->tmBallOwnerId != selfId) {
        const int ownerIdx = data->tmBallOwnerId - 1;
        leaderPose = teamStatuses[ownerIdx].robotPoseToField;
        leaderKickDir = teamStatuses[ownerIdx].kickDir;
        formationBall = teamStatuses[ownerIdx].ballPosToField;
    }
    if (!std::isfinite(leaderKickDir)) leaderKickDir = 0.0;

    const int preferredFormationShadowSide = preferredShadowSide(
        formationBall,
        leaderKickDir,
        data->tmBallOwnerId,
        config->fieldDimensions);
    const int oldFormationShadowSide = data->tmFormationShadowSide;
    if (data->tmBallOwnerId == selfId && data->tmImAlive) {
        const double shadowSideHoldMs = std::max(
            0.0,
            get_parameter(
                "strategy.cooperation.assist_shadow_side_hold_ms").as_double());
        const StrategyPoint currentShadow = calculateShadowTarget(
            formationBall,
            data->tmFormationShadowSide,
            config->fieldDimensions);
        const StrategyPoint preferredShadow = calculateShadowTarget(
            formationBall,
            preferredFormationShadowSide,
            config->fieldDimensions);
        const bool holdExpired =
            steadyMsecsSince(data->tmFormationShadowSideChangedAt) >=
            shadowSideHoldMs;
        const double currentKickClearance =
            assist_strategy_policy::kickLaneClearance(
                {currentShadow.x, currentShadow.y},
                {formationBall.x, formationBall.y},
                leaderKickDir);
        const double preferredKickClearance =
            assist_strategy_policy::kickLaneClearance(
                {preferredShadow.x, preferredShadow.y},
                {formationBall.x, formationBall.y},
                leaderKickDir);
        const bool currentBlocksKickLane = currentKickClearance < 0.97;
        const bool meaningfulImprovement =
            preferredKickClearance - currentKickClearance > 0.35 ||
            std::fabs(currentShadow.y) - std::fabs(preferredShadow.y) > 0.6;
        const int selectedShadowSide =
            assist_strategy_policy::selectShadowSide(
                data->tmFormationShadowSide,
                preferredFormationShadowSide,
                ownerChanged,
                holdExpired,
                currentBlocksKickLane,
                meaningfulImprovement);
        if (selectedShadowSide != data->tmFormationShadowSide) {
            data->tmFormationShadowSide = selectedShadowSide;
            data->tmFormationShadowSideChangedAt = SteadyClock::now();
        } else if (ownerChanged) {
            // A new leader inherits the previous legal side and starts a new hold.
            data->tmFormationShadowSideChangedAt = SteadyClock::now();
        }
    } else if (data->tmBallOwnerId > 0) {
        const int remoteShadowSide =
            teamStatuses[data->tmBallOwnerId - 1].formationShadowSide;
        const int selectedShadowSide = std::abs(remoteShadowSide) == 1
            ? remoteShadowSide
            : preferredFormationShadowSide;
        if (selectedShadowSide != data->tmFormationShadowSide) {
            data->tmFormationShadowSideChangedAt = SteadyClock::now();
        }
        data->tmFormationShadowSide = selectedShadowSide;
    } else {
        if (preferredFormationShadowSide != data->tmFormationShadowSide) {
            data->tmFormationShadowSideChangedAt = SteadyClock::now();
        }
        data->tmFormationShadowSide = preferredFormationShadowSide;
    }
    const bool formationShadowSideChanged =
        oldFormationShadowSide != data->tmFormationShadowSide;

    data->tmFormationBall = formationBall;
    data->tmFormationLeaderPose = leaderPose;
    data->tmFormationKickDir = leaderKickDir;

    const double normalSwitchPenalty = std::max(
        0.0,
        get_parameter("strategy.cooperation.assist_slot_switch_penalty").as_double());
    const double anchorSwitchPenalty = std::max(
        normalSwitchPenalty,
        get_parameter("strategy.cooperation.assist_anchor_switch_penalty").as_double());
    const double laneCrossPenalty = std::max(
        0.0,
        get_parameter("strategy.cooperation.assist_lane_cross_penalty").as_double());
    const double pathCrossPenalty = std::max(
        0.0,
        get_parameter("strategy.cooperation.assist_path_cross_penalty").as_double());
    const bool localFormationInvalid =
        !assistAssignmentsValid(data->tmAssistSlots, assistantIds);
    const bool formationContextChanged =
        ownerChanged || leaderTermChanged ||
        data->tmFormationRosterMask != rosterMask;

    bool authoritativeFormationAccepted = false;
    bool leaderMappingRecomputed = false;
    if (data->tmBallOwnerId == selfId && data->tmImAlive) {
        if (formationContextChanged || localFormationInvalid ||
            data->tmFormationOwnerId != selfId) {
            data->tmAssistSlots = calculateAssistAssignments(
                assistantIds,
                data->tmAssistSlots,
                oldOwnerId,
                data->tmBallOwnerId,
                playerPoses,
                formationBall,
                leaderPose,
                leaderKickDir,
                data->tmFormationShadowSide,
                config->fieldDimensions,
                normalSwitchPenalty,
                anchorSwitchPenalty,
                laneCrossPenalty,
                pathCrossPenalty);
            data->tmFormationRevision++;
            leaderMappingRecomputed = true;
        }
        if (formationShadowSideChanged && !leaderMappingRecomputed) {
            data->tmFormationRevision++;
        }
        data->tmFormationOwnerId = selfId;
        authoritativeFormationAccepted = true;
    } else if (data->tmBallOwnerId > 0) {
        const int ownerIdx = data->tmBallOwnerId - 1;
        const auto &ownerStatus = teamStatuses[ownerIdx];
        std::array<AssistSlot, MAX_NUM_PLAYERS> remoteAssignments{};
        for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
            remoteAssignments[i] = static_cast<AssistSlot>(ownerStatus.assistSlots[i]);
        }
        if (ownerStatus.ballOwnerId == data->tmBallOwnerId &&
            ownerStatus.formationOwnerId == data->tmBallOwnerId &&
            ownerStatus.leaderTerm >= data->tmLeaderTerm &&
            assistAssignmentsValid(remoteAssignments, assistantIds)) {
            data->tmAssistSlots = remoteAssignments;
            data->tmLeaderTerm = ownerStatus.leaderTerm;
            data->tmFormationOwnerId = ownerStatus.formationOwnerId;
            data->tmFormationRevision = ownerStatus.formationRevision;
            if (std::abs(ownerStatus.formationShadowSide) == 1) {
                data->tmFormationShadowSide = ownerStatus.formationShadowSide;
            }
            authoritativeFormationAccepted = true;
        }
    }

    if (!authoritativeFormationAccepted &&
        (formationContextChanged || localFormationInvalid)) {
        data->tmAssistSlots = calculateAssistAssignments(
            assistantIds,
            data->tmAssistSlots,
            oldOwnerId,
            data->tmBallOwnerId,
            playerPoses,
            formationBall,
            leaderPose,
            leaderKickDir,
            data->tmFormationShadowSide,
            config->fieldDimensions,
            normalSwitchPenalty,
            anchorSwitchPenalty,
            laneCrossPenalty,
            pathCrossPenalty);
        data->tmFormationOwnerId = 0;
        data->tmFormationRevision = 0;
    }
    data->tmFormationRosterMask = rosterMask;

    const AssistSlot oldAssistSlot = data->tmMyAssistSlot;
    data->tmMyAssistSlot =
        data->tmImAlive && selfRole == "striker" && !data->tmImLead &&
                data->tmBallOwnerId > 0
        ? data->tmAssistSlots[selfIdx]
        : AssistSlot::NONE;
    const bool assistSlotChanged = oldAssistSlot != data->tmMyAssistSlot;
    const bool lostBallOwnership =
        ownerChanged && oldOwnerId == selfId && data->tmBallOwnerId != selfId;
    if (assistSlotChanged) {
        data->tmAssistTargetValid = false;
        data->tmAssistWaypointValid = false;
        data->tmAssistYieldSide = 0;
        data->tmAssistYieldTimeoutMs = 0.0;
        data->tmAssistYieldRouteExtended = false;
        data->tmAssistWaypointFailureCount = 0;
        data->tmAssistPhase = isUsableAssistSlot(data->tmMyAssistSlot)
            ? AssistPhase::TRANSIT_SLOT
            : AssistPhase::HOLD;
        data->tmAssistPhaseChangedAt = SteadyClock::now();
    }
    if (lostBallOwnership && isUsableAssistSlot(data->tmMyAssistSlot)) {
        const StrategyPoint approach = normalizedOr({
            formationBall.x - leaderPose.x,
            formationBall.y - leaderPose.y,
        });
        const StrategyPoint robotFromLeader{
            data->robotPoseToField.x - leaderPose.x,
            data->robotPoseToField.y - leaderPose.y,
        };
        double sideValue = strategyCross(approach, robotFromLeader);
        if (std::fabs(sideValue) < 0.15) {
            const Pose2D slotTarget = calculateAssistSlotTarget(
                data->tmMyAssistSlot,
                formationBall,
                leaderKickDir,
                data->tmBallOwnerId,
                data->tmFormationShadowSide,
                assistantIds.size(),
                config->fieldDimensions);
            sideValue = strategyCross(approach, {
                slotTarget.x - leaderPose.x,
                slotTarget.y - leaderPose.y,
            });
        }
        data->tmAssistYieldSide = std::fabs(sideValue) < 1e-6
            ? (selfId % 2 == 0 ? 1 : -1)
            : (sideValue > 0.0 ? 1 : -1);
        data->tmAssistPhase = AssistPhase::YIELD_LANE;
        data->tmAssistPhaseChangedAt = SteadyClock::now();
        data->tmAssistYieldTimeoutMs = 0.0;
        data->tmAssistYieldRouteExtended = false;
        data->tmAssistWaypointValid = false;
    }

    if (isUsableAssistSlot(data->tmMyAssistSlot)) {
        Pose2D candidateTarget = calculateAssistSlotTarget(
            data->tmMyAssistSlot,
            formationBall,
            leaderKickDir,
            data->tmBallOwnerId,
            data->tmFormationShadowSide,
            assistantIds.size(),
            config->fieldDimensions);
        const double targetShift = std::hypot(
            candidateTarget.x - data->tmAssistTarget.x,
            candidateTarget.y - data->tmAssistTarget.y);
        const bool anchorSlot =
            data->tmMyAssistSlot == AssistSlot::ANCHOR_COVER;
        const bool defensiveDanger = formationBall.x <
            assist_strategy_policy::defensiveDangerBoundary({
                config->fieldDimensions.length,
                config->fieldDimensions.width,
                config->fieldDimensions.penaltyAreaLength,
                config->fieldDimensions.goalAreaLength,
            });
        const double targetDeadband = std::max(
            0.0,
            get_parameter(anchorSlot
                ? "strategy.cooperation.assist_anchor_deadband"
                : "strategy.cooperation.assist_target_deadband").as_double());
        const double anchorHoldMs = std::max(
            0.0,
            get_parameter("strategy.cooperation.assist_anchor_hold_ms").as_double());
        const bool holdExpired =
            steadyMsecsSince(data->tmAssistTargetUpdatedAt) >= anchorHoldMs;
        const bool shouldUpdateTarget =
            !data->tmAssistTargetValid ||
            targetShift > targetDeadband &&
                (!anchorSlot || defensiveDanger || holdExpired);
        if (shouldUpdateTarget) {
            if (anchorSlot && data->tmAssistTargetValid && !defensiveDanger) {
                candidateTarget.x = 0.65 * data->tmAssistTarget.x +
                                    0.35 * candidateTarget.x;
                candidateTarget.y = 0.65 * data->tmAssistTarget.y +
                                    0.35 * candidateTarget.y;
            }
            data->tmAssistTarget = candidateTarget;
            data->tmAssistTargetValid = true;
            data->tmAssistTargetUpdatedAt = SteadyClock::now();
        }
        data->tmAssistTarget.theta = std::atan2(
            formationBall.y - data->tmAssistTarget.y,
            formationBall.x - data->tmAssistTarget.x);
    } else {
        data->tmAssistTargetValid = false;
        data->tmAssistWaypointValid = false;
        data->tmAssistYieldTimeoutMs = 0.0;
        data->tmAssistYieldRouteExtended = false;
    }

    if (ownerChanged || assistSlotChanged) {
        log->setTimeNow();
        log->log(
            "event/cooperation_transition",
            rerun::TextLog(format(
                "owner %d->%d term=%u slot %s->%s formation_owner=%d revision=%u",
                oldOwnerId,
                data->tmBallOwnerId,
                static_cast<unsigned>(data->tmLeaderTerm),
                assistSlotName(oldAssistSlot),
                assistSlotName(data->tmMyAssistSlot),
                data->tmFormationOwnerId,
                static_cast<unsigned>(data->tmFormationRevision))));
    }
    if (logCooperationVisual && data->tmAssistTargetValid) {
        log->setTimeNow();
        log->logRobot(
            "field/assist_target",
            data->tmAssistTarget,
            0x00FFCCFF,
            format(
                "%s / %s",
                assistSlotName(data->tmMyAssistSlot),
                assistPhaseName(data->tmAssistPhase)));
    }
    if (logCooperationTimeseries) {
        log->setTimeNow();
        log->log("debug/ball_owner_id", rerun::Scalar(data->tmBallOwnerId));
        log->log(
            "debug/my_assist_slot",
            rerun::Scalar(static_cast<int>(data->tmMyAssistSlot)));
    }
    log_(format(
        "owner=%d pending=%d term=%u rank=%d slot=%s phase=%s formation=%d/%u goalieClaim=%d",
        data->tmBallOwnerId,
        data->tmPendingBallOwnerId,
        static_cast<unsigned>(data->tmLeaderTerm),
        data->tmMyCostRank,
        assistSlotName(data->tmMyAssistSlot),
        assistPhaseName(data->tmAssistPhase),
        data->tmFormationOwnerId,
        static_cast<unsigned>(data->tmFormationRevision),
        selfGoalieClaimsBall));

    tree->setEntry<bool>("is_lead", data->tmImLead);

    return;
}

void Brain::handleKickoffPlan()
{
    const int selfId = config->playerId;
    const string gameState = tree->getEntry<string>("gc_game_state");
    const bool ownCenterKickoffWindow =
        data->realGameSubState == "NONE" &&
        tree->getEntry<bool>("gc_is_kickoff_side") &&
        (gameState == "READY" || gameState == "SET" || gameState == "PLAY");
    const bool active = data->isKickingOff && ownCenterKickoffWindow &&
        (gameState == "SET" || gameState == "PLAY");

    std::lock_guard<std::mutex> cooperationLock(data->cooperationMutex);
    const auto restoreNormalOwner = [&]() {
        data->tmImLead =
            data->tmImAlive && data->tmBallOwnerId == selfId;
        tree->setEntry<bool>("is_lead", data->tmImLead);
    };
    const auto clearLock = [&]() {
        data->kickoffOwnerLocked = false;
        data->kickoffOwnerId = 0;
        data->kickoffOwnerTerm = 0;
        data->kickoffBallSnapshotValid = false;
    };
    if (!ownCenterKickoffWindow) {
        clearLock();
        data->kickoffReadyRank = -1;
        return;
    }

    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    {
        std::lock_guard<std::mutex> teamStatusLock(data->teamStatusMutex);
        std::copy(
            std::begin(data->tmStatus),
            std::end(data->tmStatus),
            teamStatuses.begin());
    }
    const double packetTimeoutMs = std::max(
        100.0,
        get_parameter(
            "strategy.cooperation.tactical_packet_timeout_ms").as_double());
    const auto playerUsable = [&](int playerId) {
        if (playerId < 1 || playerId > config->numOfPlayers ||
            playerId > MAX_NUM_PLAYERS ||
            data->penalty[playerId - 1] != PENALTY_NONE) {
            return false;
        }
        if (playerId == selfId) {
            return data->tmImAlive &&
                tree->getEntry<string>("player_role") == "striker";
        }
        const TMStatus &status = teamStatuses[playerId - 1];
        return status.isAlive && status.role == "striker" &&
            status.ageMs() <= packetTimeoutMs;
    };

    // The normal owner election already applies cost hysteresis. Reuse that
    // owner for READY placement and only use ID order to break ties among the
    // remaining strikers, so a transient cost update cannot swap positions.
    const auto updateReadyRank = [&](int ownerId) {
        int rank = -1;
        if (playerUsable(selfId) && playerUsable(ownerId)) {
            if (ownerId == selfId) {
                rank = 0;
            } else {
                rank = 1;
                for (int playerId = 1;
                     playerId <= std::min(config->numOfPlayers, MAX_NUM_PLAYERS);
                     ++playerId) {
                    if (playerId == selfId || playerId == ownerId ||
                        playerId > selfId) {
                        continue;
                    }
                    if (playerUsable(playerId)) ++rank;
                }
            }
        }
        data->kickoffReadyRank = rank >= 0 ? rank : data->myStrikerIDRank;
    };

    if (!active) {
        clearLock();
        updateReadyRank(data->tmBallOwnerId);
        return;
    }

    if (data->kickoffOwnerLocked &&
        !playerUsable(data->kickoffOwnerId)) {
        clearLock();
    }

    // The authoritative owner explicitly ending its latch releases followers
    // that cannot currently see the ball move.
    if (data->kickoffOwnerLocked &&
        data->kickoffOwnerId != selfId) {
        const TMStatus &ownerStatus =
            teamStatuses[data->kickoffOwnerId - 1];
        if (ownerStatus.ageMs() <= packetTimeoutMs &&
            !ownerStatus.kickoffActive) {
            data->isKickingOff = false;
            clearLock();
            restoreNormalOwner();
            return;
        }
    }

    const double ballMoveThreshold = std::max(
        0.05,
        get_parameter(
            "strategy.cooperation.kickoff_ball_move_threshold").as_double());
    if (data->kickoffOwnerLocked &&
        data->kickoffBallSnapshotValid && data->ballDetected &&
        std::hypot(
            data->ball.posToField.x - data->kickoffBallSnapshot.x,
            data->ball.posToField.y - data->kickoffBallSnapshot.y) >
            ballMoveThreshold) {
        data->isKickingOff = false;
        clearLock();
        restoreNormalOwner();
        return;
    }

    int claimedOwnerId = data->kickoffOwnerLocked
        ? data->kickoffOwnerId : 0;
    uint32_t claimedOwnerTerm = data->kickoffOwnerLocked
        ? data->kickoffOwnerTerm : 0;
    // Before the first latch, accept the newest valid remote declaration. Once
    // latched, never replace it with a delayed or higher-term declaration;
    // only the explicit owner-release/invalid-owner paths above can unlock it.
    if (!data->kickoffOwnerLocked) {
        for (int senderId = 1;
             senderId <= std::min(config->numOfPlayers, MAX_NUM_PLAYERS);
             ++senderId) {
            if (senderId == selfId) continue;
            const TMStatus &status = teamStatuses[senderId - 1];
            if (status.ageMs() > packetTimeoutMs ||
                !status.kickoffActive ||
                status.kickoffOwnerId != senderId ||
                !playerUsable(senderId)) {
                continue;
            }
            if (claimedOwnerId == 0 ||
                status.kickoffOwnerTerm > claimedOwnerTerm ||
                (status.kickoffOwnerTerm == claimedOwnerTerm &&
                 senderId < claimedOwnerId)) {
                claimedOwnerId = senderId;
                claimedOwnerTerm = status.kickoffOwnerTerm;
            }
        }
    }

    if (claimedOwnerId == 0) {
        const double selectionDelayMs = std::max(
            0.0,
            get_parameter(
                "strategy.cooperation.kickoff_owner_select_ms").as_double());
        const bool selectionReady =
            msecsSince(data->kickoffStartTime) >= selectionDelayMs;
        if (selectionReady && data->tmBallOwnerId == selfId &&
            playerUsable(selfId)) {
            claimedOwnerId = selfId;
            claimedOwnerTerm = data->tmLeaderTerm;
        }
    }

    if (claimedOwnerId != 0 &&
        (!data->kickoffOwnerLocked ||
         claimedOwnerId != data->kickoffOwnerId ||
         claimedOwnerTerm != data->kickoffOwnerTerm)) {
        data->kickoffOwnerLocked = true;
        data->kickoffOwnerId = claimedOwnerId;
        data->kickoffOwnerTerm = claimedOwnerTerm;
        data->kickoffBallSnapshot = data->ball.posToField;
        data->kickoffBallSnapshotValid =
            tree->getEntry<bool>("ball_location_known") ||
            tree->getEntry<bool>("tm_ball_pos_reliable");
    }

    if (data->kickoffOwnerLocked) {
        data->tmImLead =
            data->tmImAlive && data->kickoffOwnerId == selfId;
        tree->setEntry<bool>("is_lead", data->tmImLead);
    }
    updateReadyRank(
        data->kickoffOwnerLocked ? data->kickoffOwnerId : data->tmBallOwnerId);
}

void Brain::updatePlayerRoleForTeamAvailability(
    const std::array<TMStatus, MAX_NUM_PLAYERS> &teamStatuses) {
    bool switchRole = false;
    get_parameter("strategy.cooperation.enable_role_switch", switchRole);
    if (!switchRole) return;

    const int selfIdx = config->playerId - 1;
    const int configuredPlayerCount = std::clamp(
        config->numOfPlayers, 1, MAX_NUM_PLAYERS);
    const int playerCount = std::clamp(
        data->gcPlayersPerTeam,
        1,
        configuredPlayerCount);
    const string oldRole = tree->getEntry<string>("player_role");
    const string gameState = tree->getEntry<string>("gc_game_state");
    const bool gameIsInitial = gameState.empty() || gameState == "INITIAL";

    // During a penalty shoot-out the GameController temporarily removes all
    // regular players and may report goalkeeper=0 while a shot is being set
    // up. Do not apply the normal initial-goalkeeper fallback here. If a
    // shot-specific goalkeeper is selected, use that explicit assignment;
    // otherwise the team has no assigned goalkeeper for this shot.
    if (data->gcGamePhase == GAME_PHASE_PENALTY_SHOOT_OUT) {
        const int shootoutGoalkeeperId = data->gcGoalkeeperId;
        const int targetGoalkeeperIdx =
            shootoutGoalkeeperId >= 1 &&
            shootoutGoalkeeperId <= playerCount &&
            data->penalty[shootoutGoalkeeperId - 1] == PENALTY_NONE
                ? shootoutGoalkeeperId - 1
                : -1;
        data->tmInitialGoalkeeperId = shootoutGoalkeeperId;
        data->tmInitialGoalkeeperUnavailable = false;
        data->tmGoalkeeperRestoreReadySince = {};
        data->tmAssignedGoalkeeperId = targetGoalkeeperIdx + 1;
        const string newRole = selfIdx == targetGoalkeeperIdx
            ? "goal_keeper"
            : "striker";
        if (newRole != oldRole) {
            tree->setEntry<string>("player_role", newRole);
            log->setTimeNow();
            log->log(
                "event/role_switch",
                rerun::TextLog(format(
                    "player %d: %s -> %s (penalty shoot-out)",
                    config->playerId,
                    oldRole.c_str(),
                    newRole.c_str())));
        }
        return;
    }

    int initialGoalkeeperId = data->gcGoalkeeperId;
    if (initialGoalkeeperId < 1 || initialGoalkeeperId > playerCount) {
        initialGoalkeeperId = std::clamp(
            static_cast<int>(
                get_parameter("game.initial_goalkeeper_id").as_int()),
            1,
            playerCount);
    }
    if (data->tmInitialGoalkeeperId != initialGoalkeeperId) {
        data->tmInitialGoalkeeperId = initialGoalkeeperId;
        // A process can restart after the original goalkeeper's penalty has
        // cleared but before it is operational. Outside INITIAL, require the
        // normal readiness handoff instead of assuming the goalkeeper is back.
        data->tmInitialGoalkeeperUnavailable = !gameIsInitial;
        data->tmGoalkeeperRestoreReadySince = {};
        data->tmAssignedGoalkeeperId = 0;
    }

    const int initialGoalkeeperIdx = initialGoalkeeperId - 1;
    const bool initialGoalkeeperPenalized =
        data->penalty[initialGoalkeeperIdx] != PENALTY_NONE;
    const double packetTimeoutMs = std::max(
        100.0,
        get_parameter(
            "strategy.cooperation.tactical_packet_timeout_ms").as_double());
    const auto poseIsFinite = [](const Pose2D &pose) {
        return std::isfinite(pose.x) &&
               std::isfinite(pose.y) &&
               std::isfinite(pose.theta);
    };
    bool initialGoalkeeperReady = false;
    if (initialGoalkeeperIdx == selfIdx) {
        initialGoalkeeperReady =
            data->penalty[selfIdx] == PENALTY_NONE &&
            data->tmImAlive &&
            tree->getEntry<bool>("odom_calibrated") &&
            data->recoveryState == RobotRecoveryState::IS_READY &&
            !isRecoveryModeTransitionActive() &&
            !isRecoveryLocalizationBlocked() &&
            isRecoveryOperationalMode() &&
            poseIsFinite(data->robotPoseToField);
    } else {
        const auto &goalkeeperStatus = teamStatuses[initialGoalkeeperIdx];
        initialGoalkeeperReady =
            data->penalty[initialGoalkeeperIdx] == PENALTY_NONE &&
            goalkeeperStatus.isAlive &&
            goalkeeperStatus.ageMs() <= packetTimeoutMs &&
            poseIsFinite(goalkeeperStatus.robotPoseToField);
    }

    if (gameIsInitial) {
        data->tmInitialGoalkeeperUnavailable = false;
        data->tmGoalkeeperRestoreReadySince = {};
    } else if (initialGoalkeeperPenalized) {
        // Only GameController penalties trigger substitution; communication timeouts could create two goalkeepers.
        data->tmInitialGoalkeeperUnavailable = true;
        data->tmGoalkeeperRestoreReadySince = {};
    }

    int availableCount = 0;
    int highestTemporaryGoalkeeperIdx = -1;
    for (int i = 0; i < playerCount; i++) {
        if (data->penalty[i] == PENALTY_NONE) {
            availableCount++;
            if (i != initialGoalkeeperIdx) {
                highestTemporaryGoalkeeperIdx = i;
            }
        }
    }

    int targetGoalkeeperIdx = initialGoalkeeperIdx;
    if (!gameIsInitial && availableCount <= 1) {
        // Rules do not require retaining a goalkeeper when only one unpenalized player remains.
        targetGoalkeeperIdx = -1;
        data->tmGoalkeeperRestoreReadySince = {};
    } else if (!gameIsInitial && data->tmInitialGoalkeeperUnavailable) {
        bool restoreInitialGoalkeeper = false;
        if (!initialGoalkeeperPenalized) {
            if (initialGoalkeeperIdx == selfIdx) {
                if (initialGoalkeeperReady) {
                    if (data->tmGoalkeeperRestoreReadySince ==
                        SteadyClock::time_point{}) {
                        data->tmGoalkeeperRestoreReadySince = SteadyClock::now();
                    }
                    const double stableMs = std::max(
                        0.0,
                        get_parameter(
                            "strategy.cooperation.goalie_restore_stable_ms")
                            .as_double());
                    restoreInitialGoalkeeper =
                        steadyMsecsSince(
                            data->tmGoalkeeperRestoreReadySince) >= stableMs;
                } else {
                    data->tmGoalkeeperRestoreReadySince = {};
                }
            } else if (config->enableCom) {
                // The original goalkeeper completes its stability window before the temporary goalkeeper yields.
                restoreInitialGoalkeeper =
                    initialGoalkeeperReady &&
                    teamStatuses[initialGoalkeeperIdx].role == "goal_keeper";
            } else {
                // Without teammate communication the temporary goalkeeper
                // cannot prove that the original one is operational. Keep the
                // fail-safe assignment rather than creating an empty goal.
                restoreInitialGoalkeeper = false;
            }
        }

        if (restoreInitialGoalkeeper) {
            data->tmInitialGoalkeeperUnavailable = false;
            data->tmGoalkeeperRestoreReadySince = {};
        }
        if (data->tmInitialGoalkeeperUnavailable) {
            targetGoalkeeperIdx = highestTemporaryGoalkeeperIdx;
        }
    }

    data->tmAssignedGoalkeeperId = targetGoalkeeperIdx + 1;
    const string newRole = selfIdx == targetGoalkeeperIdx
        ? "goal_keeper"
        : "striker";

    if (newRole != oldRole) {
        tree->setEntry<string>("player_role", newRole);
        log->setTimeNow();
        log->log(
            "event/role_switch",
            rerun::TextLog(format(
                "player %d: %s -> %s",
                config->playerId,
                oldRole.c_str(),
                newRole.c_str())));
    }
}

void Brain::updateMemory()
{
    updateBallMemory();
    updateRobotMemory();
    updateObstacleMemory();
    updateKickoffMemory();
}

void Brain::updateObstacleMemory() {

    auto obstacles = data->getObstacles();
    vector<GameObject> obs_new = {};

    const double OBS_EXPIRE_TIME = std::max(
        0.0,
        get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double());
    const bool depthSamplingInvalid = depthSamplingInvalid_.load();
    for (int i = 0; i < obstacles.size(); i++) {
        auto obs = obstacles[i];
        if (obs.label == "Ball") continue;
        if (depthSamplingInvalid && obs.label == "Obstacle") continue;
        if (msecsSince(obs.timePoint) > OBS_EXPIRE_TIME) continue;


        updateRelativePos(obs);
        obs_new.push_back(obs);
    }


    const bool ballObstacleEnabled = get_parameter(
        "obstacle_avoidance.enable_ball_obstacle").as_bool();
    const double ballObstacleMaxAgeMsecs = std::max(
        0.0,
        get_parameter(
            "obstacle_avoidance.ball_obstacle_max_age_msecs").as_double());
    const bool ballFresh = data->ball.timePoint.nanoseconds() > 0 &&
        msecsSince(data->ball.timePoint) <= ballObstacleMaxAgeMsecs;
    const bool ballReliable = data->ballDetected ||
        tree->getEntry<bool>("ball_location_known");
    const bool ballAvoidanceState =
        (get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() &&
         isFreekickStartPlacing()) ||
        tree->getEntry<string>("gc_game_state") == "READY";
    if (ballObstacleEnabled && ballFresh && ballReliable &&
        ballAvoidanceState) {
        obs_new.push_back(data->ball);
    }

    data->setObstacles(obs_new);
    logObstacles();
}

void Brain::updateBallMemory()
{

    double secs = msecsSince(data->ball.timePoint) / 1000;

    double ballMemTimeout;
    get_parameter("strategy.ball_memory_timeout", ballMemTimeout);

    if (secs > ballMemTimeout)
    {
        tree->setEntry<bool>("ball_location_known", false);
        tree->setEntry<bool>("ball_out", false);
    }


    updateRelativePos(data->ball);
    updateRelativePos(data->tmBall);
    tree->setEntry<double>("ball_range", data->ball.range);



    if (log->shouldLog("ball_visual", config->rerunLogVisualHz)) {
        log->setTimeNow();
        log->logBall(
            "field/ball",
            data->ball.posToField,
            data->ballDetected ? 0x00FF00FF : 0x006600FF,
            data->ballDetected,
            tree->getEntry<bool>("ball_location_known")
            );
        log->logBall(
            "field/tmBall",
            data->tmBall.posToField,
            0xFFFF00FF,
            tree->getEntry<bool>("tm_ball_pos_reliable"),
            tree->getEntry<bool>("tm_ball_pos_reliable")
            );
    }
}

void Brain::updateRobotMemory() {
    auto robots = data->getRobots();
    vector<GameObject> newRobots = {};
    const double robotMemoryMsecs = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_obstacle_memory_msecs").as_double());

    for (int i = 0; i < robots.size(); i++) {
        auto r = robots[i];


        if (msecsSince(r.timePoint) > robotMemoryMsecs) continue;


        updateRelativePos(r);
        newRobots.push_back(r);
    }

    data->setRobots(newRobots);

    logMemRobots();
}

void Brain::updateKickoffMemory() {

    static Point ballPos;
    const double BALL_MOVE_THRESHOLD_FACTOR = 0.15;
    const double BALL_MOVE_THRESHOLD_MIN = 0.3;
    auto ballMoved = [=]() {
        if (!data->ballDetected) return false;
        double range = data->ball.range;
        double threshold = max(range * BALL_MOVE_THRESHOLD_FACTOR, BALL_MOVE_THRESHOLD_MIN);
        double posChange = norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y);
        return posChange > threshold;
    };
    static rclcpp::Time kickOffTime;
    const double TIMEOUT = 1000 * 10;
    auto timeReached = [=]() {
        return msecsSince(kickOffTime) > TIMEOUT;
    };
    const bool setPlayActive =
        data->realGameSubState != "NONE" && data->realGameSubState != "TIMEOUT";
    bool isWaitingForKickoff = (
        !setPlayActive &&
        (tree->getEntry<string>("gc_game_state") == "SET"  || tree->getEntry<string>("gc_game_state") == "READY")
        && !tree->getEntry<bool>("gc_is_kickoff_side")
    );
    bool isWaitingForFreekickKickoff = (
        setPlayActive
        && (
            tree->getEntry<string>("gc_game_state") == "SET"
            || tree->getEntry<string>("gc_game_state") == "READY"
            || tree->getEntry<string>("gc_game_sub_state") == "STOP"
            || tree->getEntry<string>("gc_game_sub_state") == "SET"
            || tree->getEntry<string>("gc_game_sub_state") == "GET_READY"
        )
        && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
    );
    const bool isOurRestart = setPlayActive
        ? tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
        : tree->getEntry<bool>("gc_is_kickoff_side");
    if (isOurRestart) {
        tree->setEntry<bool>("wait_for_opponent_kickoff", false);
    } else if (isWaitingForFreekickKickoff || isWaitingForKickoff) {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    } else if (tree->getEntry<bool>("wait_for_opponent_kickoff")) {
        if (ballMoved() || timeReached()) {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY;

    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;


    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++)
    {
        auto post = goalposts[i];
        if (post.name == "OL")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.name == "OR")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

double Brain::calcKickDir(double goalPostMargin) {
    double dir_rb_f = data->robotBallAngleToField;
    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];

    if (isAngleGood(goalPostMargin)) return dir_rb_f;

    double delta_l = fabs(toPInPI(theta_l - dir_rb_f));
    double delta_r = fabs(toPInPI(theta_r - dir_rb_f));
    if (delta_l < delta_r) return theta_l;
    // else
    return theta_r;
}

void Brain::updateCostToKick(
    const std::array<TMStatus, MAX_NUM_PLAYERS> &teamStatuses) {
    const bool logCostDetails =
        log->shouldLog("cost_to_kick_debug", config->rerunLogDebugHz);
    auto log_ = [=](string msg) {
        if (logCostDetails) {
            log->setTimeNow();
            log->log("debug/updateCostToKick", rerun::TextLog(msg));
        }
    };
    double cost = 0.;


    // if (!data->ballDetected) cost += 2.0;
    double secsSinceBallDet = msecsSince(data->ball.timePoint) / 1000;
    cost += secsSinceBallDet;
    log_(format("ball not dectect cost: %.1f", secsSinceBallDet));


    if (!tree->getEntry<bool>("ball_location_known")) {
        cost += 5.0;
        log_(format("ball lost cost: %.1f", 5.0));
    }


    cost += data->ball.range;
    log_(format("ball range cost: %.1f", data->ball.range));



    if (distToObstacle(data->ball.yawToRobot) < 1.5) {
        log_(format("obstacle cost: %.1f", 2.0));
        cost += 0.5;
    }


    cost += fabs(data->ball.yawToRobot) / 1.0;
    log_(format("ball yaw cost: %.1f", fabs(data->ball.yawToRobot) / 1.0));



    int selfIdx = config->playerId - 1;
    for (int i = 0; i < MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        auto status = teamStatuses[i];
        if (!status.isAlive) continue;

        double theta_tm2ball = atan2(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double range_tm2ball = norm(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double theta_me2ball = data->robotBallAngleToField;
        double range_me2ball = data->ball.range;
        double deltaTheta = fabs(toPInPI(theta_tm2ball - theta_me2ball));

        const double BUMP_DIST = 1.0;
        if (range_tm2ball < range_me2ball && sin(deltaTheta) * range_tm2ball < BUMP_DIST) {
            cost += 2.0;
            log_(format("bump cost: %.1f", 2.0));
        }
    }

    cost += fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3;
    log_(format("ajust cost: %.1f", fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3));


    if (isRecoveryActive()) {
        cost += 15.0;
        log_(format("recovery cost: %.1f", 15.0));
    }


    if (!tree->getEntry<bool>("odom_calibrated")) {
        cost += 100;
        log_(format("localization cost: %.1f", 100.0));

    }

    if (tree->getEntry<bool>("ball_out")) {
        cost += 20.0;
        log_(format("ball_out cost: %.1f", 20.0));
    }

    if (!std::isfinite(cost)) {
        cost = 1e6;
        log_("non-finite cost input; using safe fallback");
    }
    const double lastCost = data->tmMyCost;
    if (!data->tmMyCostInitialized || !std::isfinite(lastCost)) {
        data->tmMyCost = cost;
        data->tmMyCostInitialized = true;
    } else {
        data->tmMyCost = lastCost * 0.8 + cost * 0.2;
    }
    log_(format(
        "lastCost: %.1f, newCost: %.1f, smoothCost: %.1f",
        lastCost, cost, data->tmMyCost));

    return;
}

bool Brain::isAngleGood(double goalPostMargin, string type) {
    double angle = 0;
    if (type == "kick") angle = data->robotBallAngleToField; // Robot-to-ball direction in the field frame
    if (type == "shoot") angle = data->robotPoseToField.theta; // Robot heading


    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];

    if (theta_l - theta_r < M_PI / 3 * 2) {
        goalPostAngles = getGoalPostAngles(0.5);
        theta_l = goalPostAngles[0];
        theta_r = goalPostAngles[1];
    }

    return (theta_l > angle && theta_r < angle);
}

bool Brain::isPrimaryStriker() {
    string myRole = tree->getEntry<string>("player_role");
    if (myRole != "striker") return false;

    if (!config->enableCom) return true;

    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    {
        std::lock_guard<std::mutex> teamStatusLock(data->teamStatusMutex);
        std::copy(
            std::begin(data->tmStatus),
            std::end(data->tmStatus),
            teamStatuses.begin());
    }

    const double timeoutMs = std::max(
        100.0,
        get_parameter(
            "strategy.cooperation.tactical_packet_timeout_ms").as_double());
    const int myIdx = config->playerId - 1;
    int firstAliveStrikerIdx = -1;
    for (int i = 0; i < MAX_NUM_PLAYERS; i++) {
        if (i == myIdx) continue;
        const auto &status = teamStatuses[i];
        if (data->penalty[i] == PENALTY_NONE &&
            status.isAlive &&
            status.ageMs() <= timeoutMs &&
            status.role == "striker") {
            firstAliveStrikerIdx = i;
            break;
        }
    }

    if (firstAliveStrikerIdx >= 0 && firstAliveStrikerIdx < myIdx) return false;

    return true;
}

void Brain::handleFreeKickPlan()
{
    using freekick_policy::Phase;
    using freekick_policy::Type;

    const auto now = get_clock()->now();
    const int selfId = config->playerId;
    const int playerCount = std::clamp(
        config->numOfPlayers, 1, MAX_NUM_PLAYERS);
    const double packetTimeoutMs = std::max(
        100.0,
        get_parameter(
            "strategy.cooperation.tactical_packet_timeout_ms").as_double());
    const double planSelectMs = std::max(
        0.0,
        get_parameter("strategy.freekick.plan_select_ms").as_double());

    std::lock_guard<std::mutex> cooperationLock(data->cooperationMutex);
    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    {
        std::lock_guard<std::mutex> teamStatusLock(data->teamStatusMutex);
        std::copy(
            std::begin(data->tmStatus),
            std::end(data->tmStatus),
            teamStatuses.begin());
    }

    const auto refereeType = [&]() {
        if (data->realGameSubState == "DIRECT_FREEKICK") {
            return Type::Direct;
        }
        if (data->realGameSubState == "INDIRECT_FREEKICK") {
            return Type::Indirect;
        }
        return Type::None;
    }();
    const bool freeKickContext =
        refereeType != Type::None &&
        tree->getEntry<string>("gc_game_state") == "PLAY" &&
        tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK";
    const bool ourFreeKick =
        freeKickContext &&
        tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    const bool stoppedFreeKick =
        freeKickContext &&
        tree->getEntry<string>("gc_game_sub_state") == "STOP";
    const bool ourStoppedFreeKick =
        stoppedFreeKick &&
        tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    const bool executingFreeKick =
        freeKickContext &&
        tree->getEntry<string>("gc_game_sub_state") == "EXECUTE";
    const bool ourExecutingFreeKick =
        executingFreeKick &&
        tree->getEntry<bool>("gc_is_sub_state_kickoff_side");

    const auto resetPlan = [&]() {
        data->freeKickPlan = {};
        data->freeKickPlanBlocked = false;
    };
    const auto clearFreeKickState = [&](const char *reason) {
        if (data->freeKickPlan.phase != Phase::Inactive) {
            logFreeKickPlanEvent(reason, data->freeKickPlan);
        }
        resetPlan();
        data->freeKickStopActive = false;
        data->pendingFreeKickType = Type::None;
        data->freeKickStopPacketNumberValid = false;
        tree->setEntry<bool>("freekick_plan_unavailable", false);
    };

    const auto playerUsable = [&](int playerId) {
        if (playerId < 1 || playerId > playerCount) return false;
        const int index = playerId - 1;
        if (data->penalty[index] != PENALTY_NONE) return false;
        if (playerId == selfId) {
            return data->tmImAlive &&
                tree->getEntry<string>("player_role") == "striker";
        }
        const TMStatus &status = teamStatuses[index];
        return status.isAlive && status.role == "striker" &&
            status.ageMs() <= packetTimeoutMs;
    };

    const auto remotePacketCurrent = [&](const TMStatus &status) {
        if (!data->gameControllerPacketNumberValid) return true;
        const uint8_t planPacket = static_cast<uint8_t>(
            status.freeKickRefereePacketNumber);
        const uint8_t currentPacket = static_cast<uint8_t>(
            data->gameControllerPacketNumber);
        if (ourStoppedFreeKick && data->freeKickStopPacketNumberValid) {
            return freekick_policy::packetInRestartWindow(
                planPacket,
                static_cast<uint8_t>(data->freeKickStopPacketNumber),
                currentPacket);
        }
        return freekick_policy::packetRecent(planPacket, currentPacket);
    };

    const auto remotePlanValid = [&](const TMStatus &status, int senderId) {
        if (senderId < 1 || senderId > playerCount ||
            data->penalty[senderId - 1] != PENALTY_NONE ||
            !status.isAlive ||
            status.freeKickProposerBootId == 0 ||
            !status.freeKickOurRestart || status.freeKickEventId == 0 ||
            status.freeKickType < static_cast<int>(Type::Direct) ||
            status.freeKickType > static_cast<int>(Type::Indirect) ||
            status.freeKickPhase < static_cast<int>(Phase::Preparing) ||
            status.freeKickPhase > static_cast<int>(Phase::Complete) ||
            status.freeKickKickerId < 1 ||
            status.freeKickKickerId > playerCount ||
            !std::isfinite(status.freeKickBall.x) ||
            !std::isfinite(status.freeKickBall.y) ||
            !std::isfinite(status.freeKickPassTarget.x) ||
            !std::isfinite(status.freeKickPassTarget.y) ||
            !remotePacketCurrent(status)) {
            return false;
        }
        const Type type = static_cast<Type>(status.freeKickType);
        const Phase phase = static_cast<Phase>(status.freeKickPhase);
        if (type == Type::Direct &&
            (status.freeKickReceiverId != 0 ||
             phase == Phase::AwaitingSecondTouch ||
             phase == Phase::SecondTouchKicking)) {
            return false;
        }
        if (!freekick_policy::hasValidReceiver(
                type,
                status.freeKickKickerId,
                status.freeKickReceiverId,
                playerCount)) {
            return false;
        }
        if (type == Type::Direct || phase <= Phase::AwaitingSecondTouch) {
            return senderId == status.freeKickKickerId;
        }
        if (phase == Phase::Complete) {
            return senderId == status.freeKickKickerId ||
                (status.freeKickReceiverId > 0 &&
                 senderId == status.freeKickReceiverId);
        }
        return status.freeKickReceiverId > 0 &&
            senderId == status.freeKickReceiverId;
    };

    const auto copyRemotePlan = [&](const TMStatus &status) {
        FreeKickPlanState plan;
        plan.proposerBootId = status.freeKickProposerBootId;
        plan.refereePacketNumber = status.freeKickRefereePacketNumber;
        plan.eventId = status.freeKickEventId;
        plan.leaderTerm = status.freeKickLeaderTerm;
        plan.type = static_cast<Type>(status.freeKickType);
        plan.phase = static_cast<Phase>(status.freeKickPhase);
        plan.ourRestart = status.freeKickOurRestart;
        plan.kickerId = status.freeKickKickerId;
        plan.receiverId = status.freeKickReceiverId;
        plan.ballSnapshot = status.freeKickBall;
        plan.passTarget = status.freeKickPassTarget;
        for (int index = 0; index < MAX_NUM_PLAYERS; ++index) {
            const int slot = status.freeKickAssistSlots[index];
            plan.assistSlots[index] =
                slot >= static_cast<int>(AssistSlot::NONE) &&
                slot <= static_cast<int>(AssistSlot::WIDE_OUTLET)
                ? static_cast<AssistSlot>(slot)
                : AssistSlot::NONE;
        }
        plan.phaseChangedAt = now;
        if (plan.phase >= Phase::Released &&
            plan.phase < Phase::Complete) {
            const double elapsedMs = std::min(
                static_cast<double>(status.freeKickExecutionElapsedMs),
                600000.0);
            plan.executionStartTime = now -
                rclcpp::Duration::from_seconds(elapsedMs / 1000.0);
            data->freekickKickoffStartTime = plan.executionStartTime;
        }
        data->pendingFreeKickType = plan.type;
        data->freeKickPlan = plan;
    };

    const auto samePlan = [](const FreeKickPlanState &plan,
                             const TMStatus &status) {
        return plan.proposerBootId == status.freeKickProposerBootId &&
            plan.refereePacketNumber == status.freeKickRefereePacketNumber &&
            plan.eventId == status.freeKickEventId &&
            plan.leaderTerm == status.freeKickLeaderTerm &&
            static_cast<int>(plan.type) == status.freeKickType &&
            plan.kickerId == status.freeKickKickerId;
    };

    const auto statusAuthority = [](const TMStatus &status) {
        return freekick_policy::PlanAuthority{
            status.freeKickLeaderTerm,
            status.freeKickKickerId,
            status.freeKickEventId,
            status.freeKickProposerBootId,
        };
    };
    const auto planAuthority = [](const FreeKickPlanState &plan) {
        return freekick_policy::PlanAuthority{
            plan.leaderTerm,
            plan.kickerId,
            plan.eventId,
            plan.proposerBootId,
        };
    };
    const auto mergeRemotePayload = [&](FreeKickPlanState &plan,
                                        const TMStatus &status) {
        plan.receiverId = status.freeKickReceiverId;
        plan.ballSnapshot = status.freeKickBall;
        plan.passTarget = status.freeKickPassTarget;
        for (int index = 0; index < MAX_NUM_PLAYERS; ++index) {
            const int slot = status.freeKickAssistSlots[index];
            plan.assistSlots[index] =
                slot >= static_cast<int>(AssistSlot::NONE) &&
                slot <= static_cast<int>(AssistSlot::WIDE_OUTLET)
                ? static_cast<AssistSlot>(slot)
                : AssistSlot::NONE;
        }
    };
    const auto remoteMayReplace = [&](const TMStatus &status) {
        const FreeKickPlanState &plan = data->freeKickPlan;
        if (plan.phase == Phase::Inactive) return true;
        if (samePlan(plan, status)) return true;
        if (plan.phase >= Phase::FirstTouchKicking) return false;
        return freekick_policy::authorityBetter(
            statusAuthority(status), planAuthority(plan));
    };
    const auto findBestRemote = [&](Phase minimumPhase,
                                    Phase maximumPhase) {
        const TMStatus *best = nullptr;
        for (int senderId = 1; senderId <= playerCount; ++senderId) {
            if (senderId == selfId) continue;
            const TMStatus &status = teamStatuses[senderId - 1];
            if (status.ageMs() > packetTimeoutMs ||
                !remotePlanValid(status, senderId) ||
                status.freeKickType != static_cast<int>(refereeType)) {
                continue;
            }
            const Phase phase = static_cast<Phase>(status.freeKickPhase);
            if (phase < minimumPhase || phase > maximumPhase) continue;
            if (best == nullptr ||
                freekick_policy::authorityBetter(
                    statusAuthority(status), statusAuthority(*best)) ||
                (statusAuthority(status).leaderTerm ==
                     statusAuthority(*best).leaderTerm &&
                 statusAuthority(status).kickerId ==
                     statusAuthority(*best).kickerId &&
                 statusAuthority(status).eventId ==
                     statusAuthority(*best).eventId &&
                 phase > static_cast<Phase>(best->freeKickPhase))) {
                best = &status;
            }
        }
        return best;
    };
    const auto adoptRemote = [&](const TMStatus &status,
                                 const char *reason) {
        copyRemotePlan(status);
        data->freeKickPlanBlocked = false;
        logFreeKickPlanEvent(reason, data->freeKickPlan);
    };

    const auto tryAdoptInitialPlan = [&]() {
        const TMStatus *best = nullptr;
        for (int senderId = 1; senderId <= playerCount; ++senderId) {
            if (senderId == selfId) continue;
            const TMStatus &status = teamStatuses[senderId - 1];
            if (status.ageMs() > packetTimeoutMs ||
                !remotePlanValid(status, senderId) ||
                (data->pendingFreeKickType != Type::None &&
                 status.freeKickType !=
                    static_cast<int>(data->pendingFreeKickType)) ||
                (ourStoppedFreeKick &&
                 status.freeKickPhase !=
                    static_cast<int>(Phase::Preparing))) {
                continue;
            }
            if (best == nullptr ||
                freekick_policy::authorityBetter(
                    statusAuthority(status), statusAuthority(*best))) {
                best = &status;
            }
        }
        if (best == nullptr) return false;
        copyRemotePlan(*best);
        logFreeKickPlanEvent("adopt_initial", data->freeKickPlan);
        return true;
    };

    if (!freeKickContext || !ourFreeKick) {
        clearFreeKickState(
            !freeKickContext ? "referee_context_expired"
                             : "restart_ownership_expired");
    } else if (ourStoppedFreeKick) {
        const bool newStop =
            !data->freeKickStopActive ||
            data->pendingFreeKickType != refereeType;
        if (newStop) {
            resetPlan();
            data->pendingFreeKickType = refereeType;
            data->freeKickStopDetectedAt = now;
        }
        data->freeKickStopActive = true;

        // Keep one deterministic plan during the whole stationary window.
        // A remote proposal can replace a local one until the first touch,
        // using the same authority order on every robot.
        const TMStatus *bestPreparing = findBestRemote(
            Phase::Preparing, Phase::Preparing);
        if (bestPreparing != nullptr && remoteMayReplace(*bestPreparing)) {
            if (data->freeKickPlan.phase == Phase::Inactive ||
                !samePlan(data->freeKickPlan, *bestPreparing)) {
                adoptRemote(*bestPreparing, "adopt_preparing");
            } else {
                mergeRemotePayload(data->freeKickPlan, *bestPreparing);
            }
        }

        if (data->freeKickPlan.phase == Phase::Preparing &&
            data->freeKickPlan.kickerId != selfId &&
            data->freeKickPlan.kickerId >= 1 &&
            data->freeKickPlan.kickerId <= playerCount) {
            const TMStatus &kickerStatus =
                teamStatuses[data->freeKickPlan.kickerId - 1];
            const bool kickerTimedOut =
                kickerStatus.ageMs() > packetTimeoutMs;
            if (kickerTimedOut || !remotePlanValid(
                    kickerStatus, data->freeKickPlan.kickerId)) {
                logFreeKickPlanEvent(
                    kickerTimedOut
                        ? "preparing_kicker_timeout"
                        : "preparing_kicker_invalid",
                    data->freeKickPlan);
                resetPlan();
            }
        }

        if (data->freeKickPlan.phase == Phase::Preparing &&
            data->freeKickPlan.kickerId == selfId &&
            refereeType == Type::Indirect &&
            !freekick_policy::hasValidReceiver(
                refereeType,
                data->freeKickPlan.kickerId,
                data->freeKickPlan.receiverId,
                playerCount)) {
            resetPlan();
        }

        if (msecsSince(data->freeKickStopDetectedAt) >= planSelectMs &&
            data->freeKickPlan.phase == Phase::Inactive) {
            if (tryAdoptInitialPlan()) {
                // The first authoritative plan remains frozen even if normal
                // ball-owner costs fluctuate while the referee is stopped.
            } else if (data->tmBallOwnerId == selfId && data->tmImAlive) {
                const bool ballPositionKnown =
                    (tree->getEntry<bool>("ball_location_known") ||
                     tree->getEntry<bool>("tm_ball_pos_reliable")) &&
                    std::isfinite(data->ball.posToField.x) &&
                    std::isfinite(data->ball.posToField.y);
                if (!ballPositionKnown) {
                    data->freeKickPlanBlocked = true;
                } else {
                    FreeKickPlanState plan;
                    plan.proposerBootId = communication != nullptr
                        ? communication->bootId() : 1;
                    plan.refereePacketNumber =
                        data->freeKickStopPacketNumberValid
                        ? data->freeKickStopPacketNumber
                        : data->gameControllerPacketNumber;
                    plan.eventId = data->nextFreeKickEventId++;
                    if (data->nextFreeKickEventId == 0) {
                        data->nextFreeKickEventId = 1;
                    }
                    plan.leaderTerm = data->tmLeaderTerm;
                    plan.type = refereeType;
                    plan.phase = Phase::Preparing;
                    plan.ourRestart = true;
                    plan.kickerId = selfId;
                    plan.assistSlots = data->tmAssistSlots;
                    if (selfId >= 1 && selfId <= MAX_NUM_PLAYERS) {
                        plan.assistSlots[selfId - 1] = AssistSlot::NONE;
                    }
                    plan.ballSnapshot = data->ball.posToField;

                    if (refereeType == Type::Indirect) {
                        int bestReceiver = 0;
                        freekick_policy::Point bestTarget{};
                        double bestScore =
                            -std::numeric_limits<double>::infinity();
                        std::vector<freekick_policy::Point> opponents;
                        for (const GameObject &robot : data->getRobots()) {
                            if (robot.label == "Opponent" &&
                                std::isfinite(robot.posToField.x) &&
                                std::isfinite(robot.posToField.y)) {
                                opponents.push_back({
                                    robot.posToField.x,
                                    robot.posToField.y,
                                });
                            }
                        }
                        for (int id = 1; id <= playerCount; ++id) {
                            if (id == selfId || !playerUsable(id)) continue;
                            const Pose2D &receiverPose =
                                teamStatuses[id - 1].robotPoseToField;
                            if (!std::isfinite(receiverPose.x) ||
                                !std::isfinite(receiverPose.y)) continue;
                            const auto target =
                                freekick_policy::chooseIndirectPassTarget(
                                    {plan.ballSnapshot.x,
                                     plan.ballSnapshot.y},
                                    {receiverPose.x, receiverPose.y},
                                    opponents,
                                    {
                                        config->fieldDimensions.length,
                                        config->fieldDimensions.width,
                                    });
                            const double score =
                                freekick_policy::indirectPassScore(
                                    {plan.ballSnapshot.x,
                                     plan.ballSnapshot.y},
                                    target,
                                    {receiverPose.x, receiverPose.y},
                                    opponents);
                            if (score > bestScore + 1e-9 ||
                                (std::fabs(score - bestScore) <= 1e-9 &&
                                 (bestReceiver == 0 || id < bestReceiver))) {
                                bestReceiver = id;
                                bestTarget = target;
                                bestScore = score;
                            }
                        }
                        if (bestReceiver != 0) {
                            plan.receiverId = bestReceiver;
                            plan.passTarget = {bestTarget.x, bestTarget.y};
                        }
                    } else {
                        plan.passTarget = {
                            config->fieldDimensions.length / 2.0,
                            0.0,
                        };
                    }
                    if (refereeType == Type::Direct || plan.receiverId != 0) {
                        plan.phaseChangedAt = now;
                        data->freeKickPlan = plan;
                        logFreeKickPlanEvent("create_preparing", plan);
                    }
                }
            }
        }
    } else if (executingFreeKick && data->freeKickStopActive) {
        const TMStatus *best = findBestRemote(
            Phase::Preparing, Phase::Complete);
        if (best != nullptr && remoteMayReplace(*best) &&
            (data->freeKickPlan.phase == Phase::Inactive ||
             !samePlan(data->freeKickPlan, *best) ||
             static_cast<Phase>(best->freeKickPhase) >
                 data->freeKickPlan.phase)) {
            adoptRemote(*best, "adopt_release");
        }
        data->freeKickStopActive = false;
        if (data->freeKickPlan.phase == Phase::Preparing &&
            data->freeKickPlan.kickerId == selfId &&
            (refereeType == Type::Direct ||
             freekick_policy::hasValidReceiver(
                 refereeType,
                 data->freeKickPlan.kickerId,
                 data->freeKickPlan.receiverId,
                 playerCount))) {
            data->freeKickPlan.phase = Phase::Released;
            data->freeKickPlan.phaseChangedAt = now;
            data->freeKickPlan.executionStartTime = now;
            data->freekickKickoffStartTime = now;
            logFreeKickPlanEvent("release", data->freeKickPlan);
        }
    }

    // A robot may start/reconnect during EXECUTE and miss the STOP edge.
    // Accept a fresh authoritative proposal directly from the current restart.
    if (ourExecutingFreeKick &&
        (data->freeKickPlan.phase == Phase::Inactive ||
         data->freeKickPlan.phase == Phase::Preparing ||
         data->freeKickPlan.phase == Phase::Released)) {
        const TMStatus *best = findBestRemote(
            Phase::Preparing, Phase::Complete);
        if (best != nullptr && remoteMayReplace(*best) &&
            (data->freeKickPlan.phase == Phase::Inactive ||
             !samePlan(data->freeKickPlan, *best) ||
             static_cast<Phase>(best->freeKickPhase) >
                 data->freeKickPlan.phase)) {
            adoptRemote(*best, "adopt_execute");
        }
        if (data->freeKickPlan.phase == Phase::Preparing &&
            data->freeKickPlan.kickerId != selfId &&
            data->freeKickPlan.kickerId >= 1 &&
            data->freeKickPlan.kickerId <= playerCount) {
            const TMStatus &kickerStatus =
                teamStatuses[data->freeKickPlan.kickerId - 1];
            const bool kickerTimedOut =
                kickerStatus.ageMs() > packetTimeoutMs;
            if (kickerTimedOut || !remotePlanValid(
                    kickerStatus, data->freeKickPlan.kickerId)) {
                logFreeKickPlanEvent(
                    kickerTimedOut
                        ? "preparing_kicker_timeout_execute"
                        : "preparing_kicker_invalid_execute",
                    data->freeKickPlan);
                resetPlan();
            }
        }
    }

    FreeKickPlanState &plan = data->freeKickPlan;
    if (plan.phase != Phase::Inactive && plan.phase != Phase::Complete) {
        Phase newestPhase = plan.phase;
        const TMStatus *newestStatus = nullptr;
        for (int senderId = 1; senderId <= playerCount; ++senderId) {
            if (senderId == selfId) continue;
            const TMStatus &status = teamStatuses[senderId - 1];
            if (status.ageMs() > packetTimeoutMs ||
                !remotePlanValid(status, senderId) ||
                !samePlan(plan, status)) {
                continue;
            }
            const Phase remotePhase =
                static_cast<Phase>(status.freeKickPhase);
            if (remotePhase > newestPhase) {
                newestPhase = remotePhase;
                newestStatus = &status;
            } else if (remotePhase == plan.phase &&
                       senderId == plan.kickerId) {
                mergeRemotePayload(plan, status);
                const double remoteElapsed =
                    static_cast<double>(status.freeKickExecutionElapsedMs);
                const double localElapsed =
                    plan.executionStartTime.nanoseconds() == 0
                    ? 0.0 : msecsSince(plan.executionStartTime);
                if (remoteElapsed > localElapsed + 50.0 &&
                    remoteElapsed < 600000.0) {
                    plan.executionStartTime = now -
                        rclcpp::Duration::from_seconds(remoteElapsed / 1000.0);
                    data->freekickKickoffStartTime = plan.executionStartTime;
                }
            }
        }
        if (newestStatus != nullptr) {
            copyRemotePlan(*newestStatus);
            logFreeKickPlanEvent("phase_sync", plan);
        }
    }

    bool receiverUnavailable = false;
    if (plan.type == Type::Indirect &&
        freekick_policy::isExecutionActive(plan.phase) &&
        !playerUsable(plan.receiverId)) {
        // A receiver can be penalized or disappear after the plan was frozen.
        // The kicker is the only authority allowed to publish a replacement.
        receiverUnavailable = true;
    }
    if (receiverUnavailable && plan.kickerId == selfId &&
        (plan.phase == Phase::Released ||
         plan.phase == Phase::AwaitingSecondTouch)) {
        const freekick_policy::Point ball{
            plan.ballSnapshot.x, plan.ballSnapshot.y};
        int replacementId = 0;
        freekick_policy::Point replacementTarget{};
        double replacementScore = -std::numeric_limits<double>::infinity();
        std::vector<freekick_policy::Point> opponents;
        for (const GameObject &robot : data->getRobots()) {
            if (robot.label == "Opponent" &&
                std::isfinite(robot.posToField.x) &&
                std::isfinite(robot.posToField.y)) {
                opponents.push_back({
                    robot.posToField.x,
                    robot.posToField.y,
                });
            }
        }
        for (int id = 1; id <= playerCount; ++id) {
            if (id == selfId || id == plan.receiverId ||
                !playerUsable(id)) continue;
            const Pose2D &pose = teamStatuses[id - 1].robotPoseToField;
            if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) continue;
            const auto target = freekick_policy::chooseIndirectPassTarget(
                ball,
                {pose.x, pose.y},
                opponents,
                {
                    config->fieldDimensions.length,
                    config->fieldDimensions.width,
                });
            const double score = freekick_policy::indirectPassScore(
                ball, target, {pose.x, pose.y}, opponents);
            if (score > replacementScore + 1e-9 ||
                (std::fabs(score - replacementScore) <= 1e-9 &&
                 (replacementId == 0 || id < replacementId))) {
                replacementId = id;
                replacementTarget = target;
                replacementScore = score;
            }
        }
        if (replacementId != 0) {
            plan.receiverId = replacementId;
            plan.passTarget = {
                replacementTarget.x,
                replacementTarget.y,
            };
            plan.phaseChangedAt = now;
            receiverUnavailable = false;
            logFreeKickPlanEvent("receiver_reassigned", plan);
        }
    }

    const double executionTimeoutMs = std::max(
        1000.0,
        get_parameter("strategy.freekick.execution_timeout_ms").as_double());
    const bool executionTimerValid =
        plan.executionStartTime.nanoseconds() != 0;
    const double executionElapsedMs = executionTimerValid
        ? msecsSince(plan.executionStartTime) : 0.0;
    if (executionTimerValid && freekick_policy::executionExpired(
            plan.phase, executionElapsedMs, executionTimeoutMs)) {
        plan.phase = Phase::Complete;
        plan.phaseChangedAt = now;
        logFreeKickPlanEvent("execution_timeout", plan);
    }

    const bool executionActive =
        plan.ourRestart && freekick_policy::isExecutionActive(plan.phase);
    data->isFreekickKickingOff = executionActive;
    data->isDirectShoot = executionActive && plan.type == Type::Direct;
    const int actorId = executionActive
        ? freekick_policy::actorForPhase(
            plan.type, plan.phase, plan.kickerId, plan.receiverId)
        : 0;
    const bool activePlanInvalid = executionActive &&
        (actorId <= 0 || receiverUnavailable);
    data->freeKickPlanBlocked = ourExecutingFreeKick &&
        (plan.phase == Phase::Inactive || plan.phase == Phase::Preparing ||
         activePlanInvalid);
    tree->setEntry<bool>(
        "freekick_plan_unavailable", data->freeKickPlanBlocked);
    if (executionActive) {
        data->tmImLead =
            data->tmImAlive && actorId > 0 && actorId == selfId;
        tree->setEntry<bool>("is_lead", data->tmImLead);
    }
}

void Brain::logFreeKickPlanEvent(
    const char *reason, const FreeKickPlanState &plan) const
{
    if (log == nullptr) return;
    log->setTimeNow();
    log->log(
        "event/freekick_plan",
        rerun::TextLog(format(
            "%s boot=%llu ref=%u event=%u term=%u type=%d phase=%d kicker=%d receiver=%d ball=(%.2f,%.2f) target=(%.2f,%.2f)",
            reason,
            static_cast<unsigned long long>(plan.proposerBootId),
            static_cast<unsigned>(plan.refereePacketNumber),
            static_cast<unsigned>(plan.eventId),
            static_cast<unsigned>(plan.leaderTerm),
            static_cast<int>(plan.type),
            static_cast<int>(plan.phase),
            plan.kickerId,
            plan.receiverId,
            plan.ballSnapshot.x,
            plan.ballSnapshot.y,
            plan.passTarget.x,
            plan.passTarget.y)));
}

bool Brain::isOwnFreeKickExecutionActive() const
{
    std::lock_guard<std::mutex> lock(data->cooperationMutex);
    return data->freeKickPlan.ourRestart &&
        freekick_policy::isExecutionActive(data->freeKickPlan.phase);
}

bool Brain::isPlannedFreeKickActor() const
{
    std::lock_guard<std::mutex> lock(data->cooperationMutex);
    const FreeKickPlanState &plan = data->freeKickPlan;
    return plan.ourRestart &&
        !data->freeKickPlanBlocked &&
        freekick_policy::isExecutionActive(plan.phase) &&
        freekick_policy::actorForPhase(
            plan.type, plan.phase, plan.kickerId, plan.receiverId) ==
            config->playerId;
}

void Brain::markFreeKickKickStarted()
{
    std::lock_guard<std::mutex> lock(data->cooperationMutex);
    FreeKickPlanState &plan = data->freeKickPlan;
    const int actorId = freekick_policy::actorForPhase(
        plan.type, plan.phase, plan.kickerId, plan.receiverId);
    if (!plan.ourRestart || actorId != config->playerId) return;
    const auto next = freekick_policy::kickStarted(plan.type, plan.phase);
    if (next != plan.phase) {
        plan.phase = next;
        plan.phaseChangedAt = get_clock()->now();
        logFreeKickPlanEvent("kick_started", plan);
    }
}

void Brain::markFreeKickKickCompleted()
{
    std::lock_guard<std::mutex> lock(data->cooperationMutex);
    FreeKickPlanState &plan = data->freeKickPlan;
    const int actorId = freekick_policy::actorForPhase(
        plan.type, plan.phase, plan.kickerId, plan.receiverId);
    if (!plan.ourRestart || actorId != config->playerId) return;
    const auto next = freekick_policy::kickCompleted(plan.type, plan.phase);
    if (next != plan.phase) {
        plan.phase = next;
        plan.phaseChangedAt = get_clock()->now();
        logFreeKickPlanEvent("kick_completed", plan);
        data->isFreekickKickingOff =
            freekick_policy::isExecutionActive(plan.phase);
        data->isDirectShoot =
            data->isFreekickKickingOff &&
            plan.type == freekick_policy::Type::Direct;
    }
}

void Brain::cancelFreeKickKickAttempt()
{
    std::lock_guard<std::mutex> lock(data->cooperationMutex);
    // Do not regress a synchronized UDP phase. A later Kick retry can finish
    // the same in-progress touch, and the execution timeout remains the guard.
    if (data->freeKickPlan.ourRestart &&
        freekick_policy::isExecutionActive(data->freeKickPlan.phase)) {
        logFreeKickPlanEvent("kick_unconfirmed_retry", data->freeKickPlan);
    }
}

bool Brain::canGoalkeeperClaimBall(
    bool ballDetected, double ballRange, double ballX, double ballY,
    double cost) const
{
    const double maxRange = std::max(
        0.0,
        get_parameter("strategy.cooperation.goalie_claim_max_ball_range").as_double());
    const double extraDepth = std::max(
        0.0,
        get_parameter("strategy.cooperation.goalie_claim_extra_depth").as_double());
    const double lateralMargin = std::max(
        0.0,
        get_parameter(
            "strategy.cooperation.goalie_claim_lateral_margin").as_double());
    const double maxCost = std::max(
        0.0,
        get_parameter("strategy.cooperation.ball_control_cost_threshold").as_double());
    const double maxBallX =
        -config->fieldDimensions.length / 2.0 +
        config->fieldDimensions.penaltyAreaLength + extraDepth;
    const double ownGoalX = -config->fieldDimensions.length / 2.0;
    const double maxBallY = config->fieldDimensions.penaltyAreaWidth / 2.0 +
        lateralMargin;

    return ballDetected &&
           std::isfinite(ballRange) && ballRange >= 0.0 && ballRange <= maxRange &&
           std::isfinite(ballX) && ballX >= ownGoalX - 0.25 &&
           ballX <= maxBallX &&
           std::isfinite(ballY) && std::fabs(ballY) <= maxBallY &&
           std::isfinite(cost) && cost <= maxCost;
}

bool Brain::isBallOut(double locCompareDist, double lineCompareDist)
{
    auto ball = data->ball;
    auto fd = config->fieldDimensions;

    if (fabs(ball.posToField.x) > fd.length / 2 + locCompareDist)
        return true;
    if (fabs(ball.posToField.y) > fd.width / 2 + locCompareDist)
        return true;

    auto fieldLines = data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (
            (line.type == LineType::TouchLine || line.type == LineType::GoalLine)
            && line.confidence > 1.0
         ) {
            Point2D p = {ball.posToField.x, ball.posToField.y};
            // prtWarn(format("Ball: %.2f, %.2f PerpDist: %.2f", ball.posToField.x, ball.posToField.y, pointPerpDistToLine(p, line.posToField)));
            if (pointPerpDistToLine(p, line.posToField) > lineCompareDist) return true;
        }
    }
    return false;
}

void Brain::updateBallOut() {
    bool lastBallOut = tree->getEntry<bool>("ball_out");
    double range = lastBallOut ? 4.0 : 2.5;
    double threshold = config->ballOutThreshold;
    threshold += (data->isFreekickKickingOff ? 1.0 : 0.0); // Relax the boundary check during our free kick.
    threshold *= (lastBallOut ? 1.0 : 1.5); // Add hysteresis to prevent oscillation.
    tree->setEntry<bool>("ball_out", isBallOut(threshold, 10.0) && data->ball.range < range); // Require localization to confirm out-of-bounds.
}

double Brain::distToBorder() {
    vector<Line> borders;
    auto fd = config->fieldDimensions;
    borders.push_back({fd.length / 2, fd.width / 2, -fd.length / 2, fd.width / 2});
    borders.push_back({fd.length / 2, -fd.width / 2, -fd.length / 2, -fd.width / 2});
    borders.push_back({fd.length / 2, fd.width / 2, fd.length / 2, -fd.width / 2});
    borders.push_back({-fd.length / 2, fd.width / 2, -fd.length / 2, -fd.width / 2});
    double maxDist = -100;
    Point2D robot = {data->robotPoseToField.x, data->robotPoseToField.y};
    for (int i = 0; i < borders.size(); i++) {
        auto line = borders[i];
        double dist = pointPerpDistToLine(robot, line);
        if (dist > maxDist) maxDist = dist;
    }
    return maxDist;
}

bool Brain::isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio, double yRatio) {
    double x = (boundingBox.xmin + boundingBox.xmax) / 2.0;
    double y = (boundingBox.ymin + boundingBox.ymax) / 2.0;

    return (x  > config->camPixX * (1 - xRatio) / 2)
        && (x < config->camPixX * (1 + xRatio) / 2)
        && (y > config->camPixY * (1 - yRatio) / 2)
        && (y < config->camPixY * (1 + yRatio) / 2);
}

bool Brain::isDefensing() {
    bool isFreeKick = tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK";
    bool isKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");

    return isFreeKick && (!isKickoffSide);
}

void Brain::calibrateOdom(
    double x,
    double y,
    double theta,
    const std::string &source)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const auto eventSteadyTime = std::chrono::steady_clock::now();
    const Pose2D transformBefore = data->odomToField;
    const Pose2D fieldBefore = data->robotPoseToField;
    const uint64_t revisionBefore = odomTransformRevision_;
    const auto logTransformEvent = [&](bool applied) {
        if (!odomDiagnosticLog_) return;
        OdomDiagnosticTransformEvent event;
        event.rosTimeNs = get_clock()->now().nanoseconds();
        event.systemTimeNs = systemTimeNs();
        event.steadyTimeNs = steadyTimeNs(eventSteadyTime);
        event.source = source;
        event.applied = applied;
        event.revisionBefore = revisionBefore;
        event.revisionAfter = odomTransformRevision_;
        event.requestedFieldPose = {x, y, theta};
        event.odom = toDiagnosticPose(data->robotPoseToOdom);
        event.transformBefore = toDiagnosticPose(transformBefore);
        event.transformAfter = toDiagnosticPose(data->odomToField);
        event.fieldBefore = toDiagnosticPose(fieldBefore);
        event.fieldAfter = toDiagnosticPose(data->robotPoseToField);
        event.odomThetaAlignmentOffset = odomThetaAlignmentOffset_;
        event.odomThetaAlignmentDistance = odomThetaAlignmentDistance_;
        event.odomThetaAlignmentConcentration =
            odomThetaAlignmentConcentration_;
        event.odomThetaAlignmentLocked = odomThetaAlignmentLocked_;
        event.recoveryHoldActive = recoveryLocalizationHoldActive_;
        event.postGetupSettleActive = recoveryPostGetupOdomSettleActive_;
        odomDiagnosticLog_->enqueueTransformEvent(event);
    };

    if (isRecoveryLocalizationBlocked()) {
        logTransformEvent(false);
        return;
    }

    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;


    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);


    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
    ++odomTransformRevision_;
    rememberOdomThetaAlignmentAnchor(data->robotPoseToField, source);
    logTransformEvent(true);


    double placeHolder;
    // ball
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder
    );

    // robots
    auto robots = data->getRobots();
    for (int i = 0; i < robots.size(); i++) {
        updateFieldPos(robots[i]);
    }
    data->setRobots(robots);

    // goalposts
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        updateFieldPos(goalposts[i]);
    }

    // markers
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        updateFieldPos(markings[i]);
    }

    // relog
    log->setTimeNow();
    // logVisionBox(get_clock()->now());
    vector<GameObject> gameObjects = {};
    if(data->ballDetected) gameObjects.push_back(data->ball);
    for (int i = 0; i < markings.size(); i++) gameObjects.push_back(markings[i]);
    for (int i = 0; i < robots.size(); i++) gameObjects.push_back(robots[i]);
    for (int i = 0; i < goalposts.size(); i++) gameObjects.push_back(goalposts[i]);
    logDetection(gameObjects);
}

void Brain::playSound(string soundName, double blockMsecs, bool allowRepeat)
{
    if (!pubSoundPlay) return;

    static string _lastSound;
    static rclcpp::Time _lastTime;
    static double _lastBlockMsecs = 0;

    auto now = get_clock()->now();
    if (msecsSince(_lastTime) < _lastBlockMsecs) return;

    if (_lastSound == soundName && (!allowRepeat)) return;

    // else

    std_msgs::msg::String msg;
    msg.data = soundName;
    pubSoundPlay->publish(msg);

    _lastBlockMsecs = blockMsecs;
    _lastTime = now;
    _lastSound = soundName;
}

void Brain::speak(string text, bool allowRepeat)
{
    auto log_ = [=](string msg) {
        // log->setTimeNow();
        // log->log("debug/speak", rerun::TextLog(msg));
    };

    const double COOLDOWN_MSECS = 2000.;
    if (!pubSpeak) {
        log_("publisher not found");
        return;
    }
    if (!config->soundEnable || config->soundPack != "espeak") {
        log_("config not compatible");
        return;
    }

    static string _lastText;
    static rclcpp::Time _lastTime;

    if (msecsSince(_lastTime) < COOLDOWN_MSECS) {
        log_("cooldown in process");
        return;
    }

    if (_lastText == text && (!allowRepeat)) {
        log_("repeat not allowed");
        return;
    }

    // else
    _lastTime = get_clock()->now();
    std_msgs::msg::String msg;
    msg.data = text;
    pubSpeak->publish(msg);

    _lastText = text;
}

double Brain::msecsSince(rclcpp::Time time)
{
    auto now = this->get_clock()->now();
    if (time.get_clock_type() != now.get_clock_type()) return 1e18;
    // A delayed message can carry a timestamp slightly in the future.  Treat
    // it as fresh instead of allowing a negative age to bypass expiry checks.
    return std::max(0.0, (now - time).nanoseconds() / 1e6);
}

rclcpp::Time Brain::timePointFromHeader(std_msgs::msg::Header header) {
    auto stamp = header.stamp;
    // ROS timestamps with nanosec == 0 are valid.  Only an all-zero (or
    // negative) stamp is invalid; use the node clock as a safe fallback.
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
        (stamp.sec == 0 && stamp.nanosec == 0)) {
        return get_clock()->now();
    }
    return rclcpp::Time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
}


void Brain::joystickCallback(const booster_interface::msg::RemoteControllerState &joy)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    auto log_ = [=](string msg) {
        if (!log->shouldLog("joystick_debug", config->rerunLogDebugHz))
            return;
        log->setTimeNow();
        log->log("debug/joystick", rerun::TextLog(msg));
    };
    // prtDebug("joy!!", RED_CODE);
    string soundPack = config->soundPack;

    // Control the robot from the gamepad without blocking button input.
    if (
        fabs(joy.lx) > 0.1
        || fabs(joy.ly) > 0.1
        || fabs(joy.rx) > 0.1
        || fabs(joy.ry) > 0.1
    ) {
        tree->setEntry<bool>("go_manual", true);
        // prtWarn("GO Manual");
    } else {
        tree->setEntry<bool>("go_manual", false);
        // prtWarn("Axe manual take over end");
    }

    // Button priority: LT combinations, RT combinations, then individual buttons.
    if (joy.lt && !joy.rt) { // LT combination
        // Live parameter tuning.
        if (joy.hat_u || joy.hat_d)
        {
            config->vxFactor += 0.01 * (joy.hat_u ? 1.0 : -1.0);
            speak(format("vx factor: %.2f", config->vxFactor));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        if (joy.hat_l || joy.hat_r)
        {
            config->yawOffset += 0.01 * (joy.hat_r ? 1.0 : -1.0);
            speak(format("yaw offset: %.2f", config->yawOffset));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        // Switch control states.
        if (joy.x)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
            // playSound("sad");
        }
        if (joy.a)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
            // playSound("search");
        }
        if (joy.b)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
            // playSound("exited");
        }
        else if (joy.y)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
            log_("SWITCH ROLE");
            // playSound("talk");
        }
    }

    if (joy.rt) { // RT combination
        // Nothing for now
    }

    // Otherwise process individual buttons.
    if (!joy.lt && !joy.rt) {
        if (joy.lb) {
            tree->setEntry<bool>("assist_chase", true);
            prtDebug("Assit Chase");
            playSound(soundPack + "-chase", 5000);
        } else {
            tree->setEntry<bool>("assist_chase", false);
            // prtWarn("Exit Assit Chase");
        }
        if (joy.rb) {
            tree->setEntry<bool>("assist_kick", true);
            prtDebug("Assit Kick");
            playSound(soundPack + "-kick", 5000);
        } else {
            tree->setEntry<bool>("assist_kick", false);
            // prtWarn("Exit Assit Kick");
        }

        if (joy.hat_u) {
            playSound(soundPack + "-celebrate", 2000, true);
        } else if (joy.hat_d) {
            playSound(soundPack + "-regret", 2000, true);
        } else if (joy.hat_r) {
            playSound(soundPack + "-ready", 2000, true);
        } else if (joy.hat_l) {
            playSound(soundPack + "-taunt", 2000, true);
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    static const vector<string> gameStateMap = {
        "INITIAL", "READY", "SET", "PLAY", "END"
    };
    if (msg.state >= gameStateMap.size())
    {
        prtErr(format("received invalid game controller state %d", msg.state));
        return;
    }
    if (msg.game_phase > GAME_PHASE_TIMEOUT ||
        msg.players_per_team == 0 ||
        msg.players_per_team > MAX_NUM_PLAYERS) {
        prtErr(format(
            "received invalid game controller phase=%d players_per_team=%d",
            msg.game_phase,
            msg.players_per_team));
        return;
    }
    if (msg.players_per_team != config->numOfPlayers) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Configuration mismatch: GameController reports %u players per team, "
            "but game.number_of_players is %d; teammate packets above P%d are rejected",
            static_cast<unsigned>(msg.players_per_team),
            config->numOfPlayers, config->numOfPlayers);
    }

    game_controller_interface::msg::TeamInfo myTeamInfo;
    game_controller_interface::msg::TeamInfo oppoTeamInfo;
    if (msg.teams[0].team_number == config->teamId) {
        myTeamInfo = msg.teams[0];
        oppoTeamInfo = msg.teams[1];
    } else if (msg.teams[1].team_number == config->teamId) {
        myTeamInfo = msg.teams[1];
        oppoTeamInfo = msg.teams[0];
    } else {
        prtErr(format(
            "received invalid game controller message team0 %d, team1 %d, teamId %d",
            msg.teams[0].team_number,
            msg.teams[1].team_number,
            config->teamId));
        return;
    }

    switch (msg.set_play) {
    case SET_PLAY_NONE:
    case SET_PLAY_DIRECT_FREE_KICK:
    case SET_PLAY_INDIRECT_FREE_KICK:
    case SET_PLAY_PENALTY_KICK:
    case SET_PLAY_THROW_IN:
    case SET_PLAY_GOAL_KICK:
    case SET_PLAY_CORNER_KICK:
        break;
    default:
        prtErr(format(
            "received invalid GameController set play %d", msg.set_play));
        return;
    }

    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const auto now = get_clock()->now();
    // UDP delivery can duplicate or reorder packets. Accept a new sequence
    // number in the forward half of the uint8 sequence space. If the source
    // has been silent for a while, accept the next packet as a controller
    // restart/recovery point even if its counter was reset.
    const bool controllerWasSilent =
        data->timeLastGamecontrolMsg.nanoseconds() <= 0 ||
        msecsSince(data->timeLastGamecontrolMsg) > 2000.0;
    const bool controllerSequenceReset =
        data->gameControllerPacketNumberValid && controllerWasSilent;
    if (data->gameControllerPacketNumberValid &&
        !controllerWasSilent &&
        !freekick_policy::packetNewer(
            msg.packet_number,
            static_cast<uint8_t>(data->gameControllerPacketNumber))) {
        return;
    }
    if (controllerSequenceReset) {
        // A restarted sender can reuse packet numbers. Re-arm edge-triggered
        // restart detection so a STOP packet after the restart creates a new
        // free-kick event instead of reusing the old window.
        data->gcOwnFreeKickStopWasActive = false;
        data->freeKickStopPacketNumberValid = false;
    }
    data->timeLastGamecontrolMsg = now;
    const int playersPerTeam = std::clamp(
        static_cast<int>(msg.players_per_team),
        1,
        MAX_NUM_PLAYERS);
    data->gcPlayersPerTeam = playersPerTeam;
    const int reportedGoalkeeperId = static_cast<int>(myTeamInfo.goalkeeper);
    data->gcGamePhase = msg.game_phase;
    const bool goalkeeperIsExplicit =
        reportedGoalkeeperId >= 1 && reportedGoalkeeperId <= playersPerTeam;
    if (msg.game_phase == GAME_PHASE_PENALTY_SHOOT_OUT) {
        // 0 is the protocol's valid "no goalkeeper selected" value during a
        // shoot-out. It must not be confused with malformed normal-play data.
        data->gcGoalkeeperId = goalkeeperIsExplicit ? reportedGoalkeeperId : 0;
    } else {
        data->gcGoalkeeperId = goalkeeperIsExplicit
            ? reportedGoalkeeperId
            : std::clamp(
                static_cast<int>(
                    get_parameter("game.initial_goalkeeper_id").as_int()),
                1,
                playersPerTeam);
    }

    const bool hasSetPlay = msg.set_play != SET_PLAY_NONE;
    const bool isOurSetPlay =
        hasSetPlay && msg.kicking_team == config->teamId;
    string gameState = gameStateMap[msg.state];
    // A global stop has no separate legacy blackboard state. Reuse SET so every
    // normal-play behavior stops immediately. Set-play stops are handled below.
    if (msg.stopped && !hasSetPlay && msg.game_phase != GAME_PHASE_TIMEOUT)
    {
        gameState = "SET";
    }
    tree->setEntry<string>("gc_game_state", gameState);

    const bool isKickOffSide =
        !hasSetPlay && msg.kicking_team == config->teamId;
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    switch (msg.set_play)
    {
        case SET_PLAY_NONE:
            data->realGameSubState = "NONE";
            data->isDirectShoot = false;
            break;
        case SET_PLAY_DIRECT_FREE_KICK:
            data->realGameSubState = "DIRECT_FREEKICK";
            data->isDirectShoot = isOurSetPlay;
            break;
        case SET_PLAY_INDIRECT_FREE_KICK:
            data->realGameSubState = "INDIRECT_FREEKICK";
            data->isDirectShoot = false;
            break;
        case SET_PLAY_PENALTY_KICK:
            data->realGameSubState = "PENALTY_KICK";
            data->isDirectShoot = isOurSetPlay;
            break;
        case SET_PLAY_THROW_IN:
            data->realGameSubState = "THROW_IN";
            data->isDirectShoot = false;
            break;
        case SET_PLAY_GOAL_KICK:
            data->realGameSubState = "GOAL_KICK";
            data->isDirectShoot = isOurSetPlay;
            break;
        case SET_PLAY_CORNER_KICK:
            data->realGameSubState = "CORNER_KICK";
            data->isDirectShoot = false;
            break;
        default:
            prtErr(format("received invalid GameController set play %d", msg.set_play));
            return;
    }

    string gameSubStateType = "NONE";
    string gameSubState;
    if (msg.game_phase == GAME_PHASE_TIMEOUT)
    {
        gameSubStateType = "TIMEOUT";
        data->realGameSubState = "TIMEOUT";
    }
    else if (hasSetPlay && msg.state == STATE_PLAYING)
    {
        // Keep the set-play identity after the referee resumes. STOP is a
        // mandatory stationary phase; EXECUTE is routed through the frozen
        // FreeKickPlan instead of falling through as ordinary play.
        gameSubStateType = "FREE_KICK";
        gameSubState = msg.stopped ? "STOP" : "EXECUTE";
    }

    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    const bool isSubStateKickOffSide = isOurSetPlay;
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    data->gameControllerPacketNumber = msg.packet_number;
    data->gameControllerPacketNumberValid = true;
    const bool ownDirectOrIndirectStop =
        msg.state == STATE_PLAYING && msg.stopped && isOurSetPlay &&
        (msg.set_play == SET_PLAY_DIRECT_FREE_KICK ||
         msg.set_play == SET_PLAY_INDIRECT_FREE_KICK);
    if (ownDirectOrIndirectStop && !data->gcOwnFreeKickStopWasActive) {
        data->freeKickStopPacketNumber = msg.packet_number;
        data->freeKickStopPacketNumberValid = true;
    }
    data->gcOwnFreeKickStopWasActive = ownDirectOrIndirectStop;

    // cout << "game state: " << gameState << " game sub state type: " << gameSubStateType << endl;
    int liveCount = 0;
    int oppoLiveCount = 0;
    // Apply the local player's penalty state; a penalized robot must not move.
    for (int i = 0; i < MAX_NUM_PLAYERS; i++) {
        if (i < playersPerTeam)
        {
            data->penalty[i] = static_cast<int>(myTeamInfo.players[i].penalty);
            data->oppoPenalty[i] = static_cast<int>(oppoTeamInfo.players[i].penalty);
        }
        else
        {
            data->penalty[i] = PENALTY_SUBSTITUTE;
            data->oppoPenalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->penalty[i] == PENALTY_NONE) liveCount++;
        if (data->oppoPenalty[i] == PENALTY_NONE) oppoLiveCount++;
    }
    data->liveCount = liveCount;
    data->oppoLiveCount = oppoLiveCount;

    // cout << "penalty: " << data->penalty[0] << " " << data->penalty[1] << " " << data->penalty[2] << " " << data->penalty[3] << endl;
    // cout << "oppo penalty: " << data->oppoPenalty[0] << " " << data->oppoPenalty[1] << " " << data->oppoPenalty[2] << " " << data->oppoPenalty[3] << endl;
    bool lastIsUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");
    bool isUnderPenalty = (data->penalty[config->playerId - 1] != PENALTY_NONE); // Whether this robot is penalized
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);
    if (isUnderPenalty && !lastIsUnderPenalty) tree->setEntry<bool>("odom_calibrated", false); // A newly penalized robot must relocalize on re-entry.

    // log game state
    log->setTimeNow();
    log->logToScreen(
        "tick/gamecontrol",
        format("Player: %d  Role: %s PrimaryStriker: %s GameState: %s  SubStateType: %s  SubState: %s UnderPenalty: %d isKickoff: %d isSubStateKickoff: %d stopped: %d setPlay: %d",
            config->playerId, tree->getEntry<string>("player_role").c_str(), isPrimaryStriker() ? "Yes" : "No", gameState.c_str(), gameSubStateType.c_str(), gameSubState.c_str(), isUnderPenalty, isKickOffSide, isSubStateKickOffSide, msg.stopped, msg.set_play
            ),
        0xFFFFFFFF,
        30.0
    );

    // Optional goal-celebration wave.
    data->score = static_cast<int>(myTeamInfo.score);
    data->oppoScore = static_cast<int>(oppoTeamInfo.score);
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const bool recoveryActive = isRecoveryActive();

    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    data->camConnected = true;
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastDet = timePoint; // Used to report detection latency.

    auto gameObjects = getGameObjects(msg);

    // Group detected objects by type.
    vector<GameObject> balls, goalposts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalposts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            // Person is always a collision hazard when safety avoidance is
            // enabled. treat_person_as_robot remains a game/debug semantic.
            if (config->treatPersonAsRobot ||
                get_parameter("obstacle_avoidance.avoid_person").as_bool())
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    // Process each object group.
    detectProcessBalls(balls);
    detectProcessGoalposts(goalposts);
    detectProcessMarkings(markings);
    if (recoveryActive) {
        // Camera projections are invalid while falling/getting up and often
        // contain the robot's own limbs. Rebuild robot tracks after recovery.
        data->setRobots({});
    } else {
        detectProcessRobots(robots);
    }

    // Process and record field-of-view information.
    detectProcessVisionBox(msg);

    // logVisionBox(timePoint);
    logDetection(gameObjects);
}

void Brain::updateLinePosToField(FieldLine& line) {
    double __; // __ is a placeholder for transformations
    transCoord(
        line.posToRobot.x0, line.posToRobot.y0, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x0, line.posToField.y0, __
    );
    transCoord(
        line.posToRobot.x1, line.posToRobot.y1, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x1, line.posToField.y1, __
    );
}

void Brain::fieldLineCallback(const vision_interface::msg::LineSegments &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    // auto timestamp = msg.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastLineDet = timePoint; // Used to report field-line detection latency.

    vector<FieldLine> lines = {};
    FieldLine line;

    double x0, y0, x1, y1, __; // __ is a placeholder for transformations
    if (msg.coordinates.size() != msg.coordinates_uv.size() ||
        msg.coordinates.size() % 4 != 0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring incomplete field-line coordinates: world=%zu image=%zu",
            msg.coordinates.size(), msg.coordinates_uv.size());
    }
    const size_t lineCount =
        std::min(msg.coordinates.size(), msg.coordinates_uv.size()) / 4;
    for (size_t i = 0; i < lineCount; ++i) {
        const size_t index = i * 4;
        bool finiteLine = true;
        for (size_t offset = 0; offset < 4; ++offset) {
            finiteLine = finiteLine &&
                std::isfinite(msg.coordinates[index + offset]) &&
                std::isfinite(msg.coordinates_uv[index + offset]);
        }
        if (!finiteLine) continue;
        line.posToRobot.x0 = msg.coordinates[index]; line.posOnCam.x0 = msg.coordinates_uv[index];
        line.posToRobot.y0 = msg.coordinates[index + 1]; line.posOnCam.y0 = msg.coordinates_uv[index + 1];
        line.posToRobot.x1 = msg.coordinates[index + 2]; line.posOnCam.x1 = msg.coordinates_uv[index + 2];
        line.posToRobot.y1 = msg.coordinates[index + 3]; line.posOnCam.y1 = msg.coordinates_uv[index + 3];
        updateLinePosToField(line);
        line.timePoint = timePoint;
        // TODO infer line dir and id

        lines.push_back(line);
    }
    lines = processFieldLines(lines);
    data->setFieldLines(lines);

    if (log->shouldLog("field_lines_visual", config->rerunLogVisualHz)) { // log processed lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};
        vector<string> logLabels = {};
        vector<unsigned int> logColors = {};

        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0},
                {line.posToRobot.x1, -line.posToRobot.y1},
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0},
                {line.posToField.x1, -line.posToField.y1},
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
            string label;
            unsigned int color = 0xFFFF00FF;
            if (line.type == LineType::GoalLine) {
                label = "GoalLine";
                color = 0xFF0000FF;
            }
            else if (line.type == LineType::TouchLine) {
                label = "TouchLine";
                color = 0xFF0000FF;
            }
            else if (line.type == LineType::MiddleLine) label = "MiddleLine";
            else if (line.type == LineType::PenaltyArea) label = "PenaltyArea";
            else if (line.type == LineType::GoalArea) label = "GoalArea";
            else if (line.type == LineType::NA) label = "NA";
            else label = "Other"; // should not see this label logged

            label += format(" c = %.1f", line.confidence);

            // if (line.dir == LineDir::Horizontal) label += " Horizontal";
            // else if (line.dir == LineDir::Vertical) label += " Vertical";
            // else label += " NA";

            logLabels.push_back(label);
            logColors.push_back(color);
        };
        log->log(
            "field/det_lines",
            rerun::LineStrips2D(logLinesOnField)
                .with_colors(logColors)
                .with_radii(0.04)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
        // log->log(
        //     "robotframe/det_lines",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(logColors)
        //         .with_radii(0.04)
        //         .with_draw_order(20)
        //         .with_labels(logLabels)
        // );
        log->log(
            "image/det_lines",
            rerun::LineStrips2D(logLinesOnCam)
                .with_colors(logColors)
                .with_radii(1.0)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
    }

    { // log original lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};

        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0},
                {line.posToRobot.x1, -line.posToRobot.y1},
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0},
                {line.posToField.x1, -line.posToField.y1},
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
        };
        // log->log(
        //     "field/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnField)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "robotframe/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "image/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnCam)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(3.0)
        //         .with_draw_order(10)
        // );
    }
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const auto callbackTime = get_clock()->now();
    const auto callbackSteadyTime = std::chrono::steady_clock::now();
    ++odomCallbackSequence_;
    updateRecoveryLocalizationHold();

    const auto velocity = client->lastVelocityCommand();
    const double odomDistanceScale = std::fabs(config->robotOdomFactor);
    // Hardware may define odometry x/y axes and zero theta from different startup poses.
    // Keep x/y in the fixed odometry frame and apply the session-estimated angular-axis offset to theta.
    const Pose2D rawOdomPose{
        msg.x * odomDistanceScale,
        msg.y * odomDistanceScale,
        msg.theta};
    const Pose2D fieldPoseBeforeAlignment = data->robotPoseToField;
    const bool alignmentLockedNow = updateOdomThetaAlignment(
        rawOdomPose, velocity, callbackTime, callbackSteadyTime);

    data->robotPoseToOdom.x = rawOdomPose.x;
    data->robotPoseToOdom.y = rawOdomPose.y;
    data->robotPoseToOdom.theta = toPInPI(
        rawOdomPose.theta + odomThetaAlignmentOffset_);
    if (alignmentLockedNow) {
        applyOdomThetaAlignment(
            fieldPoseBeforeAlignment,
            callbackTime,
            callbackSteadyTime);
    }

    if (recoveryPostGetupOdomSettleActive_) {
        recoveryPostGetupOdomSamples_.push_back(
            {callbackTime, data->robotPoseToOdom});
        const int64_t keepNs = static_cast<int64_t>(
            std::max(2000.0, recoveryPostGetupOdomSettleWindowMs_ * 2.0) *
            1e6);
        while (!recoveryPostGetupOdomSamples_.empty() &&
               callbackTime.nanoseconds() -
                       recoveryPostGetupOdomSamples_.front().stamp.nanoseconds() >
                   keepNs) {
            recoveryPostGetupOdomSamples_.pop_front();
        }
    }

    if (recoveryLocalizationHoldActive_) {
        recoveryLocalizationHoldPose_.theta =
            fall_recovery_policy::fieldHeadingWithHeldTranslation(
                recoveryLocalizationHoldStartFieldTheta_,
                recoveryLocalizationHoldStartOdomPose_.theta,
                data->robotPoseToOdom.theta);
    }

    // Release only after this exact raw sample completes the stable window,
    // then reanchor against it before allowing subsequent odometry through.
    updatePostGetupOdomSettling(callbackTime);
    updateRecoveryLocalizationHold();

    if (recoveryLocalizationHoldActive_) {
        reanchorPoseToCurrentOdom(recoveryLocalizationHoldPose_);
    } else {
        // Update the field-frame robot pose from odometry.
        transCoord(
            data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
            data->odomToField.x, data->odomToField.y, data->odomToField.theta,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
    }

    if (odomDiagnosticLog_ &&
        odomDiagnosticLog_->shouldSample(callbackSteadyTime)) {
        OdomDiagnosticSample sample;
        sample.callbackSequence = odomCallbackSequence_;
        sample.rosTimeNs = callbackTime.nanoseconds();
        sample.systemTimeNs = systemTimeNs();
        sample.steadyTimeNs = steadyTimeNs(callbackSteadyTime);
        sample.rawOdom = {msg.x, msg.y, msg.theta};
        sample.odom = toDiagnosticPose(data->robotPoseToOdom);
        sample.odomToField = toDiagnosticPose(data->odomToField);
        sample.field = toDiagnosticPose(data->robotPoseToField);
        sample.command = {
            velocity.requestedX,
            velocity.requestedY,
            velocity.requestedTheta,
            velocity.sentX,
            velocity.sentY,
            velocity.sentTheta,
            velocity.time.nanoseconds()};
        sample.transformRevision = odomTransformRevision_;
        sample.odomFactor = odomDistanceScale;
        sample.odomThetaAlignmentOffset = odomThetaAlignmentOffset_;
        sample.odomThetaAlignmentDistance = odomThetaAlignmentDistance_;
        sample.odomThetaAlignmentConcentration =
            odomThetaAlignmentConcentration_;
        sample.odomThetaAlignmentLocked = odomThetaAlignmentLocked_;
        sample.recoveryHoldActive = recoveryLocalizationHoldActive_;
        sample.postGetupSettleActive = recoveryPostGetupOdomSettleActive_;
        sample.odomCalibrated = tree->getEntry<bool>("odom_calibrated");
        sample.controlState = tree->getEntry<int>("control_state");
        sample.gameState = tree->getEntry<string>("gc_game_state");
        sample.gameSubState = tree->getEntry<string>("gc_game_sub_state");
        sample.gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
        sample.transformMode = recoveryLocalizationHoldActive_
            ? "RECOVERY_HOLD_REANCHOR"
            : "FIXED_ODOM_TO_FIELD";
        odomDiagnosticLog_->enqueueSample(sample);
    }

    // Publish the TF transform.
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    // Set the translation.
    transform.transform.translation.x = data->robotPoseToOdom.x;
    transform.transform.translation.y = data->robotPoseToOdom.y;
    transform.transform.translation.z = 0.0;

    // Convert Euler rotation to a quaternion.
    tf2::Quaternion q;
    q.setRPY(0, 0, data->robotPoseToOdom.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    const bool logOdometry =
        log->shouldLog("odometry_visual", config->rerunLogVisualHz);
    if (logOdometry) {
        log->setTimeNow();
        log->log("debug/odom_callback", rerun::TextLog(format("x: %.1f, y: %.1f, z: %.1f", data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta)));
    }

    // Broadcast the TF transform.
    tf_broadcaster_->sendTransform(transform);

    // Log odometry.

    if (logOdometry) {
        log->setTimeNow();
        bool imAlive = false;
        bool imLead = false;
        double myCost = 0.0;
        {
            std::lock_guard<std::mutex> cooperationLock(
                data->cooperationMutex);
            imAlive = data->tmImAlive;
            imLead = data->tmImLead;
            myCost = data->tmMyCost;
        }
        auto color = 0x00FF00FF;
        if (!imAlive) color = 0x006600FF;
        else if (!imLead) color = 0x00CC00FF;
        string label = format("Cost: %.1f", myCost);
        log->logRobot("field/robot", data->robotPoseToField, color, label, true);
    }
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    if (msg.motor_state_serial.size() >= 2) {
        data->headYaw = msg.motor_state_serial[0].q;
        data->headPitch = msg.motor_state_serial[1].q;
        if (log->shouldLog("head_angles_debug", config->rerunLogDebugHz)) {
            log->setTimeNow();
            log->log("debug/head_angles", rerun::TextLog(format(
                "pitch: %.1f, yaw: %.1f", data->headYaw, data->headPitch)));
        }
    }
    const double roll = msg.imu_state.rpy[0];
    const double pitch = msg.imu_state.rpy[1];
    const double gyroX = msg.imu_state.gyro[0];
    const double gyroY = msg.imu_state.gyro[1];
    const double gyroZ = msg.imu_state.gyro[2];
    recoveryLatestImuValid_ =
        std::isfinite(roll) && std::isfinite(pitch) &&
        fall_recovery_policy::isFiniteGyro(gyroX, gyroY, gyroZ);
    recoveryLatestImuRoll_ = roll;
    recoveryLatestImuPitch_ = pitch;
    recoveryLatestGyroMagnitude_ = recoveryLatestImuValid_
        ? std::sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ)
        : std::numeric_limits<double>::infinity();
    updateRecoveryLocalizationPreHold(
        roll, pitch);
    updateFallDownState(msg);
    updateRecoveryLocalizationHold();
}

bool Brain::isFallen(double roll, double pitch) const
{
    return std::fabs(roll) > data->fallenThresholdRad ||
           std::fabs(pitch) > data->fallenThresholdRad;
}

BrainData::FallDownSide Brain::getFallDownSide(double pitch) const
{
    return pitch > 0.0 ? BrainData::FallDownSide::FRONT
                       : BrainData::FallDownSide::BACK;
}

void Brain::updateFallDownState(const booster_interface::msg::LowState &msg)
{
    const double roll = msg.imu_state.rpy[0];
    const double pitch = msg.imu_state.rpy[1];
    const double gyroX = msg.imu_state.gyro[0];
    const double gyroY = msg.imu_state.gyro[1];
    const double gyroZ = msg.imu_state.gyro[2];
    const bool gyroFinite =
        fall_recovery_policy::isFiniteGyro(gyroX, gyroY, gyroZ);
    const double gyroMagnitude = gyroFinite
        ? std::sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ)
        : std::numeric_limits<double>::infinity();
    // Invalid gyro data must never satisfy a stationary recovery check.
    const bool moving =
        !gyroFinite || gyroMagnitude > data->movementThresholdRadPerSec;
    const bool fallen =
        std::isfinite(roll) && std::isfinite(pitch) && isFallen(roll, pitch);
    const bool stationaryUprightSample =
        fall_recovery_policy::isStationaryUprightSample(
            roll,
            pitch,
            gyroX,
            gyroY,
            gyroZ,
            data->movementThresholdRadPerSec);
    const int64_t nowUs = get_clock()->now().nanoseconds() / 1000;

    if (fallen) recoveryUprightStableSinceUs_ = 0;

    if (data->recoveryState == RobotRecoveryState::IS_GETTING_UP) {
        const bool postureConfirmed =
            fall_recovery_policy::updateUprightRecoveryConfirmation(
                !stationaryUprightSample,
                moving,
                nowUs,
                recoveryGetUpPostureStableSinceUs_);
        if (moving || data->lastGetupMovementDetectedUs == 0) {
            data->lastGetupMovementDetectedUs = nowUs;
        }

        const int64_t graceUs = static_cast<int64_t>(
            std::max(0.0, data->getupNoMovementGraceSec) * 1e6);
        const int64_t noMovementTimeoutUs = static_cast<int64_t>(
            std::max(0.1, data->noMovementTimeoutSec) * 1e6);
        const double expectedSec = data->fallDownSide == BrainData::FallDownSide::FRONT
            ? data->frontGetUpTimeSec : data->backGetUpTimeSec;
        const int64_t expectedUs = static_cast<int64_t>(
            std::max(0.0, expectedSec) * 1e6);
        const int64_t postureConfirmTimeoutUs = static_cast<int64_t>(
            std::max(
                0,
                static_cast<int>(get_parameter(
                    "recovery.posture_confirm_timeout_ms").as_int())) *
            1000LL);
        const int64_t elapsedUs = nowUs - data->lastGetupStartedUs;

        const bool noMovementTimeout =
            fallen && elapsedUs > graceUs &&
            nowUs - data->lastGetupMovementDetectedUs > noMovementTimeoutUs;
        const bool actionTimeout = elapsedUs > expectedUs;
        const bool postureConfirmTimeout =
            elapsedUs > expectedUs + postureConfirmTimeoutUs;
        if (noMovementTimeout || (postureConfirmTimeout && !postureConfirmed)) {
            RCLCPP_WARN(
                get_logger(), "GetUp failed (%s), roll=%.3f pitch=%.3f gyro=%.3f",
                noMovementTimeout ? "no movement" : "posture not confirmed",
                roll,
                pitch,
                gyroMagnitude);
            data->recoveryState = RobotRecoveryState::HAS_FALLEN;
            data->lastGetupStartedUs = 0;
            data->lastGetupMovementDetectedUs = 0;
            recoveryGetUpPostureStableSinceUs_ = 0;
            recoveryPostureConfirmed_ = false;
            ++data->recoveryPerformedRetryCount;
            data->recoveryPerformed = false;
            cancelRecoveryModeTransition();
            client->enterDamping();
            return;
        }

        if (actionTimeout && postureConfirmed) {
            RCLCPP_INFO(
                get_logger(),
                "GetUp posture confirmed upright and stationary for %.1f ms",
                fall_recovery_policy::kUprightRecoveryConfirmUs / 1000.0);
            data->recoveryState = RobotRecoveryState::IS_READY;
            data->lastGetupStartedUs = 0;
            data->lastGetupMovementDetectedUs = 0;
            recoveryPostureConfirmed_ = true;
            beginRecoverySoccerTransition();
        }
        return;
    }

    if (fallen) {
        if (data->recoveryState == RobotRecoveryState::IS_READY) {
            data->fallDownSide = getFallDownSide(pitch);
            data->recoveryState = moving
                ? RobotRecoveryState::IS_FALLING
                : RobotRecoveryState::HAS_FALLEN;
            cancelRecoveryModeTransition();
            client->enterDamping();
            RCLCPP_WARN(
                get_logger(), "Fall detected: state=%d roll=%.3f pitch=%.3f gyro=%.3f",
                static_cast<int>(data->recoveryState), roll, pitch, gyroMagnitude);
        } else if (data->recoveryState == RobotRecoveryState::IS_FALLING && !moving) {
            data->fallDownSide = getFallDownSide(pitch);
            data->recoveryState = RobotRecoveryState::HAS_FALLEN;
            RCLCPP_INFO(get_logger(), "Robot stopped falling; recovery is ready to start");
        }
    } else if (data->recoveryState == RobotRecoveryState::IS_FALLING ||
               data->recoveryState == RobotRecoveryState::HAS_FALLEN) {
        const bool notStablyUpright =
            !fall_recovery_policy::isLocalizationPreHoldReleaseTilt(
                roll, pitch);
        if (fall_recovery_policy::updateUprightRecoveryConfirmation(
                notStablyUpright,
                moving,
                nowUs,
                recoveryUprightStableSinceUs_)) {
            data->recoveryState = RobotRecoveryState::IS_READY;
            data->recoveryPerformed = false;
            data->recoveryPerformedRetryCount = 0;
            recoveryUprightStableSinceUs_ = 0;
            recoveryGetUpPostureStableSinceUs_ = 0;
            recoveryPostureConfirmed_ = true;
            // Fall detection already requested kDamping. Keep localization
            // frozen until the SDK confirms the return to kSoccer.
            beginRecoverySoccerTransition(false);
            RCLCPP_INFO(
                get_logger(),
                "Robot remained upright and stationary; requesting kSoccer");
        }
    } else if (data->recoveryState == RobotRecoveryState::IS_READY) {
        std::lock_guard<std::mutex> transitionLock(recoveryTransitionMutex_);
        if (recoveryModeTransitionPhase_ ==
            RecoveryModeTransitionPhase::WaitingForWalking) {
            // The first confirmation is not a permanent latch. A partial fall,
            // invalid IMU sample, or renewed get-up motion withdraws it until
            // the body is again strictly upright and stationary for one second.
            recoveryPostureConfirmed_ =
                fall_recovery_policy::updateUprightRecoveryConfirmation(
                    !stationaryUprightSample,
                    moving,
                    nowUs,
                    recoveryGetUpPostureStableSinceUs_);
        }
    }
}

void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{

    static int counter = 0;
    counter++;
    const int imgInterval = std::max(1, config->rerunLogImgInterval);
    if (counter % imgInterval == 0)
    {
        // Track runtime resolution because a weak camera connection may reduce it and affect CamTrackBall.
        {
            std::lock_guard<std::recursive_mutex> stateLock(
                recoveryStateMutex_);
            config->camPixX = msg.width;
            config->camPixY = msg.height;
        }
        log->log("debug/imageCallback", rerun::TextLog(format("img width: %.d, img height: %.d", msg.width, msg.height)));

        cv::Mat image;
        // Decode according to the image encoding.
        if (msg.encoding == "nv12" || msg.encoding == "NV12") {
            // NV12: Y plane (H x W) + interleaved UV (H/2 x W)
            size_t expected = (size_t)(msg.width * msg.height * 3 / 2);
            if (msg.data.size() < expected) {
                prtErr(format("NV12 buffer too small. got %zu expect >= %zu", msg.data.size(), expected));
                return;
            }
            cv::Mat yuv(msg.height + msg.height / 2, msg.width, CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
            cv::cvtColor(yuv, image, cv::COLOR_YUV2BGR_NV12);
        } else if (msg.encoding == "bgra8") {
            // Wrap the BGRA image in an OpenCV matrix.
            image = cv::Mat(msg.height, msg.width, CV_8UC4, const_cast<uint8_t *>(msg.data.data()));
            cv::Mat imageBGR;
            // Convert BGRA to BGR and discard alpha.
            cv::cvtColor(image, imageBGR, cv::COLOR_BGRA2BGR);
            image = imageBGR;
        } else if (msg.encoding == "bgr8") {
            // BGR8 input.
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        } else if (msg.encoding == "rgb8") {
            // RGB8 input.
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        } else {
            // Reject unsupported encodings.
            prtErr(format("Unsupported image encoding: %s", msg.encoding.c_str()));
            return;
        }

        // Compress the image.
        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10}; // JPEG quality; adjust as needed.
        cv::imencode(".jpg", image, compressed_image, compression_params);

        // Send compressed image data to Rerun.
        // double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        // log->setTimeSeconds(time);
        log->setTimeSeconds(timePointFromHeader(msg.header).seconds());
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose& msg)
{
    // Once the timestamped T2 stream is available, do not let a legacy
    // callback with a different frame convention overwrite its transform.
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    if (timestampedHeadPoseSeen_.load(std::memory_order_relaxed) ||
        visionCameraTransformSeen_.load(std::memory_order_relaxed)) {
        return;
    }
    updateCameraToRobot(msg);
}

void Brain::cameraInfoCallback(const sensor_msgs::msg::CameraInfo &msg)
{
    const double fx = msg.k[0];
    const double fy = msg.k[4];
    const double cx = msg.k[2];
    const double cy = msg.k[5];
    if (msg.width == 0 || msg.height == 0 ||
        !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy) ||
        fx <= 0.0 || fy <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring invalid T2 CameraInfo for depth projection");
        return;
    }

    bool changed = false;
    {
        std::lock_guard<std::recursive_mutex> stateLock(
            recoveryStateMutex_);
        changed = std::fabs(config->depthfx - fx) > 1e-6 ||
            std::fabs(config->depthfy - fy) > 1e-6 ||
            std::fabs(config->depthcx - cx) > 1e-6 ||
            std::fabs(config->depthcy - cy) > 1e-6 ||
            std::fabs(config->camPixX - msg.width) > 0.5 ||
            std::fabs(config->camPixY - msg.height) > 0.5;
        config->camfx = fx;
        config->camfy = fy;
        config->camcx = cx;
        config->camcy = cy;
        config->depthfx = fx;
        config->depthfy = fy;
        config->depthcx = cx;
        config->depthcy = cy;
        config->camPixX = msg.width;
        config->camPixY = msg.height;
    }

    if (changed) {
        RCLCPP_INFO(
            get_logger(),
            "T2 depth intrinsics updated from CameraInfo: "
            "%ux%u fx=%.6f fy=%.6f cx=%.6f cy=%.6f",
            msg.width, msg.height, fx, fy, cx, cy);
    }
}

void Brain::updateCameraToRobot(const geometry_msgs::msg::Pose& msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const double quaternionNorm = std::sqrt(
        msg.orientation.x * msg.orientation.x +
        msg.orientation.y * msg.orientation.y +
        msg.orientation.z * msg.orientation.z +
        msg.orientation.w * msg.orientation.w);
    if (!std::isfinite(quaternionNorm) || quaternionNorm < 1e-8 ||
        !std::isfinite(msg.position.x) || !std::isfinite(msg.position.y) ||
        !std::isfinite(msg.position.z)) {
        cameraTransformValid_ = false;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring invalid head pose while updating depth transform");
        return;
    }

    // Calculate the head-to-base transform.
    Eigen::Matrix4d headToBase = Eigen::Matrix4d::Identity();

    // Convert the quaternion to a rotation matrix.
    Eigen::Quaterniond q(
        msg.orientation.w / quaternionNorm,
        msg.orientation.x / quaternionNorm,
        msg.orientation.y / quaternionNorm,
        msg.orientation.z / quaternionNorm
    );
    headToBase.block<3,3>(0,0) = q.toRotationMatrix();

    // Set the translation vector.
    headToBase.block<3,1>(0,3) = Eigen::Vector3d(
        msg.position.x,
        msg.position.y,
        msg.position.z
    );

    // Define and calculate the camera-to-head transform.
    // Eigen::Matrix4d camToHead;
    // camToHead << 0,  0,  1,  0,
    //             -1,  0,  0,  0,
    //              0, -1,  0,  0,
    //              0,  0,  0,  1;

    // Calculate and store the camera-to-base transform.
    data->camToRobot = headToBase * config->camToHead;
    cameraTransformValid_ = data->camToRobot.allFinite();
    if (!cameraTransformValid_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Computed non-finite T2 camera-to-robot transform");
    }
}

void Brain::headPoseStampedCallback(const geometry_msgs::msg::PoseStamped &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    if (visionCameraTransformSeen_.load(std::memory_order_relaxed)) {
        return;
    }
    updateCameraToRobot(msg.pose);
    if (!cameraTransformValid_) return;

    const rclcpp::Time poseStamp(msg.header.stamp, RCL_ROS_TIME);
    if (poseStamp.nanoseconds() <= 0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring timestamped head pose with an invalid stamp");
        return;
    }

    timestampedHeadPoseSeen_.store(true);
    if (!cameraTransformBuffer_.empty() &&
        poseStamp < cameraTransformBuffer_.back().stamp) {
        // A restarted publisher can jump backwards. Old samples must not be
        // matched against the new clock epoch.
        cameraTransformBuffer_.clear();
    }
    cameraTransformBuffer_.push_back({poseStamp, data->camToRobot});

    const double bufferMsecs = std::max(
        100.0,
        get_parameter("vision.depth_pose_buffer_msecs").as_double());
    const int64_t bufferNsecs = static_cast<int64_t>(bufferMsecs * 1e6);
    while (!cameraTransformBuffer_.empty() &&
           poseStamp.nanoseconds() -
                   cameraTransformBuffer_.front().stamp.nanoseconds() >
               bufferNsecs) {
        cameraTransformBuffer_.pop_front();
    }
}

void Brain::headPoseTfCallback(const geometry_msgs::msg::TransformStamped &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const rclcpp::Time poseStamp(msg.header.stamp, RCL_ROS_TIME);
    if (poseStamp.nanoseconds() <= 0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring VisionNode camera transform with an invalid stamp");
        return;
    }

    const auto &translation = msg.transform.translation;
    const auto &rotation = msg.transform.rotation;
    const double quaternionNorm = std::sqrt(
        rotation.x * rotation.x + rotation.y * rotation.y +
        rotation.z * rotation.z + rotation.w * rotation.w);
    if (!std::isfinite(quaternionNorm) || quaternionNorm < 1e-8 ||
        !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
        !std::isfinite(translation.z)) {
        cameraTransformValid_ = false;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring invalid VisionNode camera transform");
        return;
    }

    Eigen::Quaterniond q(
        rotation.w / quaternionNorm,
        rotation.x / quaternionNorm,
        rotation.y / quaternionNorm,
        rotation.z / quaternionNorm);
    Eigen::Matrix4d nominalHeadToBase = Eigen::Matrix4d::Identity();
    nominalHeadToBase.block<3, 3>(0, 0) = q.toRotationMatrix();
    nominalHeadToBase.block<3, 1>(0, 3) = Eigen::Vector3d(
        translation.x, translation.y, translation.z);
    // The VisionNode transform is the raw head pose in legacy mode and the
    // nominal camera-link pose in automatic-calibration mode. In both cases
    // camera.extrin supplies the remaining camera-to-head/link transform.
    const Eigen::Matrix4d cameraToRobot = nominalHeadToBase * config->camToHead;
    if (!cameraToRobot.allFinite()) {
        cameraTransformValid_ = false;
        return;
    }

    // Remove raw samples collected before the adapted stream became
    // available. Mixing frame conventions would make nearest-timestamp
    // lookup nondeterministic.
    if (!visionCameraTransformSeen_.exchange(true, std::memory_order_relaxed)) {
        cameraTransformBuffer_.clear();
    }
    timestampedHeadPoseSeen_.store(true, std::memory_order_relaxed);
    if (!cameraTransformBuffer_.empty() &&
        poseStamp < cameraTransformBuffer_.back().stamp) {
        cameraTransformBuffer_.clear();
    }
    cameraTransformBuffer_.push_back({poseStamp, cameraToRobot});
    data->camToRobot = cameraToRobot;
    cameraTransformValid_ = true;

    const double bufferMsecs = std::max(
        100.0,
        get_parameter("vision.depth_pose_buffer_msecs").as_double());
    const int64_t bufferNsecs = static_cast<int64_t>(bufferMsecs * 1e6);
    while (!cameraTransformBuffer_.empty() &&
           poseStamp.nanoseconds() -
                   cameraTransformBuffer_.front().stamp.nanoseconds() >
               bufferNsecs) {
        cameraTransformBuffer_.pop_front();
    }
}

void Brain::recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(recoveryStateMutex_);
    const auto &buffer = msg.msg;
    if (buffer.size() < sizeof(RobotRecoveryStateData)) {
        RCLCPP_WARN(
            get_logger(), "Invalid recovery message size: %zu < %zu",
            buffer.size(), sizeof(RobotRecoveryStateData));
        return;
    }

    RobotRecoveryStateData recoveryState{};
    std::memcpy(&recoveryState, buffer.data(), sizeof(recoveryState));
    if (recoveryState.state > static_cast<uint8_t>(RobotRecoveryState::IS_GETTING_UP) ||
        recoveryState.is_recovery_available > 1) {
        RCLCPP_WARN(
            get_logger(), "Invalid recovery payload: state=%u available=%u",
            static_cast<unsigned>(recoveryState.state),
            static_cast<unsigned>(recoveryState.is_recovery_available));
        return;
    }

    // The legacy topic remains useful for diagnostics, but its planner indexes
    // are not stable across SDK releases. Local IMU logic owns recoveryState.
    data->isRecoveryAvailable = recoveryState.is_recovery_available != 0;
    data->currentRobotModeIndex = static_cast<int>(recoveryState.current_planner_index);
}


int Brain::markCntOnFieldLine(const string markType, const FieldLine line, const double margin) {
    int cnt = 0;
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto marking = markings[i];
        if (marking.label == markType) {
            Point2D point = {marking.posToField.x, marking.posToField.y};
            if (fabs(pointPerpDistToLine(point, line.posToField)) < margin) {
                cnt += 1;
            }
        }
    }
    return cnt;
}

int Brain::goalpostCntOnFieldLine(const FieldLine line, const double margin) {
    int cnt = 0;
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        auto post = goalposts[i];
        Point2D point = {post.posToField.x, post.posToField.y};
        if (pointMinDistToLine(point, line.posToField) < margin) {
            cnt += 1;
        }
    }
    return cnt;
}

bool Brain::isBallOnFieldLine(const FieldLine line, const double margin) {
    auto ballPos = data->ball.posToField;
    Point2D point = {ballPos.x, ballPos.y};
    return fabs(pointPerpDistToLine(point, line.posToField)) < margin;
}

void Brain::identifyFieldLine(FieldLine& line) {
    auto mapLines = config->mapLines;
    FieldLine mapLine;
    double confidence;
    line.type = LineType::NA;

    double bestConfidence = 0;
    double secondBestConfidence = 0;
    int bestIndex = -1;
    for (int i = 0; i < mapLines.size(); i++) {
        mapLine = mapLines[i];
        confidence = line.dir == mapLine.dir ?
            probPartOfLine(line.posToField, mapLine.posToField)
            : 0.0;

        // Boost confidence with other features
        if (mapLine.type == LineType::GoalLine) {
            confidence += 0.3 * markCntOnFieldLine("TCross", line, 0.2);
            confidence += 0.5 * goalpostCntOnFieldLine(line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "CORNER_KICK")
            ) confidence += 0.3; // The ball is on the goal line for a corner kick.
        }
        if (mapLine.type == LineType::MiddleLine) {
            confidence += 0.3 * markCntOnFieldLine("XCross", line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK")
            ) confidence += 0.3; // The ball is on the halfway line for a goal kick.
        }
        if (mapLine.type == LineType::TouchLine) {
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK" || data->realGameSubState == "CORNER_KICK" || data->realGameSubState == "THROW_IN")
            ) confidence += 0.3; // The ball is on a boundary line for corner, goal, and kick-in restarts.
        }

        // Avoid confusing a goal-area line with a goal line.
        auto fd = config->fieldDimensions;
        if (
            mapLine.type == LineType::GoalLine
            && fabs(line.posToField.y0) < fd.goalAreaWidth / 2 + 0.5
            && fabs(line.posToField.y1) < fd.goalAreaWidth / 2 + 0.5
        ) confidence -= 0.3;

        // Avoid confusing a penalty-area line with a touchline.
        if (
            mapLine.type == LineType::TouchLine
            && min(fabs(line.posToField.x0), fabs(line.posToField.x1)) > fd.length / 2.0 -  fd.penaltyAreaLength - 0.5
            && line.posToField.x0 * line.posToField.x1 > 0
        ) confidence -= 0.3;

        double length = norm(line.posToField.x0 - line.posToField.x1, line.posToField.y0 - line.posToField.y1);
        if (length < 0.5) confidence -= 0.5;
        else if (length < 1.0) confidence -= 0.1;

        if (confidence > bestConfidence) {
            secondBestConfidence = bestConfidence;
            bestConfidence = confidence;
            bestIndex = i;
        }
    }

    if (bestConfidence - secondBestConfidence < 0.5) bestConfidence -= 0.5;



    if (bestIndex >= 0 && bestIndex < mapLines.size()) {
        line.type = mapLines[bestIndex].type;
        line.half = mapLines[bestIndex].half;
        line.side = mapLines[bestIndex].side;
        line.confidence = bestConfidence;
        return;
    }

    // else
    line.type = LineType::NA;
    line.half = LineHalf::NA;
    line.side = LineSide::NA;
    line.confidence = 0.0;
    return;
}

void Brain::identifyMarking(GameObject& marking) {
    double minDist = 100;
    double secMinDist = 100;
    int mmIndex = -1;
    for (int i = 0; i < config->mapMarkings.size(); i++) {
       auto mm = config->mapMarkings[i];

       if (mm.type != marking.label) continue;

       double dist = norm(marking.posToField.x - mm.x, marking.posToField.y - mm.y);

       if (dist < minDist) {
           secMinDist = minDist;
           minDist = dist;
           mmIndex = i;
       } else if (dist < secMinDist) {
           secMinDist = dist;
       }
    }

    auto fd = config->fieldDimensions;
    if (
        mmIndex >=0 && mmIndex < config->mapMarkings.size()
        && minDist < 1.5 * 14 / fd.length // 1.0 for adultsize
        && secMinDist - minDist > 1.5 * 14 / fd.length // 2.0 for adultsize
        // && marking.confidence > 70
    ) {
        marking.id = mmIndex;
        marking.name = config->mapMarkings[mmIndex].name;
        marking.idConfidence = 1.0;
    } else {
        marking.id = -1;
        marking.name = "NA";
        marking.idConfidence = 0.0;
    }
}


void Brain::identifyGoalpost(GameObject& goalpost) {
    string side = "NA";
    string half = "NA";
    if (goalpost.posToField.x > 0) half = "O";
    else half = "S";

    if (goalpost.posToField.y > 0) side = "L";
    else side = "R";

    goalpost.id = 0;
    goalpost.name = half + side;
    goalpost.idConfidence = 1.0;
    // TODO: Use markings to improve goalpost identification.
}

vector<FieldLine> Brain::processFieldLines(vector<FieldLine>& fieldLines) {
    vector<FieldLine> original = fieldLines;
    vector<FieldLine> res;


    int sizeBefore = original.size();
    // merge lines that are actually the same line
    for (int i = 0; i < original.size(); i++) {
        for (int j = i + 1; j < original.size(); j++) {
            auto line1 = original[i].posToField;
            auto line2 = original[j].posToField;
            if (isSameLine(line1, line2, 0.1, 1.0)) {
                FieldLine mergedLine{};
                mergedLine.posToField = mergeLines(line1, line2);
                mergedLine.posToRobot = mergeLines(original[i].posToRobot, original[j].posToRobot);
                mergedLine.posOnCam = mergeLines(original[i].posOnCam, original[j].posOnCam);
                mergedLine.timePoint = original[i].timePoint;

                // replace first line in original with merged line and remove second line
                original[i] = mergedLine;
                original.erase(original.begin() + j);
                j--;
            }
        }
    }
    int sizeAfter = original.size();
    // prtWarn(format("Merged %d lines into %d lines", sizeBefore, sizeAfter));
    // return original;
    // filter out lines that are too short and infer direction while ditch lines whose dir cannot be inferred
    double valve = 0.2;
    for (int i = 0; i < original.size(); i++) {
        auto line = original[i];
        auto lineDir = atan2(line.posToField.y1 - line.posToField.y0, line.posToField.x1 - line.posToField.x0);

        if (fabs(toPInPI(lineDir - M_PI)) < 0.1 || fabs(lineDir) < 0.1) line.dir = LineDir::Vertical;
        else if (fabs(toPInPI(lineDir - M_PI/2)) < 0.1 || fabs(toPInPI(lineDir + M_PI/2)) < 0.1) line.dir = LineDir::Horizontal;
        else continue;

        // if line is direction can be verified, check if it is long enough
        if (lineLength(line.posToField) > valve) {
            res.push_back(line);
        }
    }

    // identify each line
    for (int i = 0; i < res.size(); i++) {
        identifyFieldLine(res[i]);
    }
    return res;
}


vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    // auto timestamp = detections.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(detections.header);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj{};

        gObj.timePoint = timePoint;
        gObj.label = obj.label;
        gObj.color = obj.color;

        if (obj.target_uv.size() == 2)
        { // Precise pixel position of a field marking.
            gObj.precisePixelPoint.x = static_cast<double>(obj.target_uv[0]);
            gObj.precisePixelPoint.y = static_cast<double>(obj.target_uv[1]);
        }

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;
        gObj.fallen = obj.fallen;
        gObj.fallenConfidence = obj.fallen_confidence;

        // Use projected distance directly instead of depth ranging.
        if (obj.position_projection.size() < 2) {
            continue;
        }
        gObj.posToRobot.x = obj.position_projection[0];
        gObj.posToRobot.y = obj.position_projection[1];
        if (!std::isfinite(gObj.posToRobot.x) ||
            !std::isfinite(gObj.posToRobot.y)) {
            continue;
        }

        if (obj.position_projection_left.size() >= 2 &&
            obj.position_projection_right.size() >= 2) {
            gObj.obstacleLeftToRobot.x = obj.position_projection_left[0];
            gObj.obstacleLeftToRobot.y = obj.position_projection_left[1];
            gObj.obstacleRightToRobot.x = obj.position_projection_right[0];
            gObj.obstacleRightToRobot.y = obj.position_projection_right[1];
            gObj.obstacleFootprintValid =
                std::isfinite(gObj.obstacleLeftToRobot.x) &&
                std::isfinite(gObj.obstacleLeftToRobot.y) &&
                std::isfinite(gObj.obstacleRightToRobot.x) &&
                std::isfinite(gObj.obstacleRightToRobot.y) &&
                gObj.obstacleLeftToRobot.x > 0.0 &&
                gObj.obstacleRightToRobot.x > 0.0;
            if (gObj.obstacleFootprintValid) {
                double unusedTheta = 0.0;
                transCoord(
                    gObj.obstacleLeftToRobot.x,
                    gObj.obstacleLeftToRobot.y,
                    0.0,
                    data->robotPoseToField.x,
                    data->robotPoseToField.y,
                    data->robotPoseToField.theta,
                    gObj.obstacleLeftToField.x,
                    gObj.obstacleLeftToField.y,
                    unusedTheta);
                transCoord(
                    gObj.obstacleRightToRobot.x,
                    gObj.obstacleRightToRobot.y,
                    0.0,
                    data->robotPoseToField.x,
                    data->robotPoseToField.y,
                    data->robotPoseToField.theta,
                    gObj.obstacleRightToField.x,
                    gObj.obstacleRightToField.y,
                    unusedTheta);
            }
        }

        // Calculate angles.
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range); // Approximation

        // Calculate the object's field-frame position.
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z // z is only a placeholder and is unused elsewhere.
        );

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    static rclcpp::Time lastSeenRealBallTime;
    double bestConfidence = 0;
    int indexRealBall = -1;  // Index of the accepted ball; -1 means no ball.

    // Select the most likely real ball.
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];
        auto oldBall = data->ball;

        // Reject overhead lights misclassified as balls.
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 15.0)
            continue;

        // Reject balls far outside the field. Disabled because it affects behaviors such as chase.
        // if (isBallOut(2.0, 2.0))
        //     continue;

        // Increase confidence when a detection is close in position and time to the previous ball.
        // double c = ballObj.confidence;
        // double oldC = oldBall.confidence;
        // double msecs = msecsSince(oldBall.timePoint);
        // double dist = norm(ballObj.posToField.x - oldBall.posToField.x, ballObj.posToField.y - oldBall.posToField.y);
        // ballObj.confidence += 0.5 * max(oldC, 0.0) * max(1 - msecs/1000, 0.0) * max(1 - dist / 2, 0.0);
        // ballObj.confidence = min(100.0, ballObj.confidence);
        // log->setTimeNow();
        // log->log("/debug/oldc_newc", rerun::TextLog(format(
        //     "oldc: %.2f  newc: %.2f",
        //     oldC,
        //     ballObj.confidence
        // )));

        // Reject low-confidence detections.
        if (ballObj.confidence < config->ballConfidenceThreshold)
            continue;

        // TODO: Reject detections on bodies, clearly outside the field, or with implausible position jumps.
        // Account for occlusion so an occluded ball remains trusted longer than one that disappears in open view.

        // Select the highest-confidence remaining detection.
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    auto now = this->get_clock()->now();

    if (indexRealBall >= 0)
    { // Ball detected.
        data->ballDetected = true;
        data->ballEverDetected = true;

        data->ball = ballObjs[indexRealBall];
        data->ball.confidence = bestConfidence;

        tree->setEntry<bool>("ball_location_known", true);
        updateBallOut();

        lastSeenRealBallTime = now;
        data->lose_ball = false;
    }
    else
    { // No ball detected.
        log->setTimeNow();
        // log->log("image/detection_boxes_realball", rerun::Clear::FLAT);
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;

        if (lastSeenRealBallTime.seconds() > 0.0)
        {
            double msecs = (now - lastSeenRealBallTime).nanoseconds() / 1e6;
            data->lose_ball = (msecs > 2000.0);
        }
        else
        {
            data->lose_ball = false;
        }

        // data->ball.confidence = 0; // DO NOT set confidence to 0, confidence decay depends on this.
    }

    // Calculate the field-frame robot-to-ball direction.
    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    // Distance-stability test.
    // for (int i = 0; i < markingObjs.size(); i++) {
    //    auto m = markingObjs[i];
    //    if (m.label != "PenaltyPoint" || m.posToField.x < 0.0) continue;

    //    double range = norm(m.posToField.x - data->robotPoseToField.x, m.posToField.y - data->robotPoseToField.y);

    //    log->setTimeNow();
    //    log->log("debug/penalty_point/range", rerun::Scalar(range));
    //    log->log("debug/penalty_point/x", rerun::Scalar(m.posToField.x));
    //    log->log("debug/penalty_point/y", rerun::Scalar(m.posToField.y));
    // }
    // // end testing

    const double confidenceValve = 50; // Reject confidence below this threshold.
    vector<GameObject> markings = {};
    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        // Reject low-confidence detections.
        if (marking.confidence < confidenceValve)
            continue;

        // Reject overhead objects misclassified as markings.
        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 15.0)
            continue;

        // Store detections that pass all filters.
        identifyMarking(marking);
        markings.push_back(marking);
    }
    data->setMarkings(markings);

    // log identified markings
    if (!log->shouldLog("identified_markings_visual", config->rerunLogVisualHz))
        return;

    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];
        if (markings[i].id >= 0) {
            circles.push_back(log->circle(m.posToField.x, -m.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", m.name.c_str(), m.idConfidence));
        }
    }

    log->log("field/identified_markings",
        rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
       .with_radii(0.01)
       .with_labels(labels)
       .with_colors(0xFFFFFFFF));
}

void Brain::detectProcessGoalposts(const vector<GameObject> &goalpostObjs)
{
    const double confidenceValve = 50; // Reject confidence below this threshold.
    vector<GameObject> goalposts = {};

    for (int i = 0; i < goalpostObjs.size(); i++) {
        auto goalpost = goalpostObjs[i];

        // Reject low-confidence detections.
        if (goalpost.confidence < confidenceValve)
            continue;

        identifyGoalpost(goalpost);
        goalposts.push_back(goalpost);
    }
    data->setGoalposts(goalposts);

    // log identified goalposts
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < goalposts.size(); i++) {
        auto p = goalposts[i];
        if (goalposts[i].id >= 0) {
            circles.push_back(log->circle(p.posToField.x, -p.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", p.name.c_str(), p.idConfidence));
        }
    }

    // log->log("field/identified_goalposts",
    //     rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
    //     .with_radii(0.01)
    //     .with_labels(labels)
    //     .with_colors(0xFFFFFFFF));
}


void Brain::detectProcessRobots(const vector<GameObject> &robotObjs) {
    rawRobotDetectionCount_.store(robotObjs.size());
    auto robots = data->getRobots();
    vector<bool> matched(robots.size(), false);
    vector<GameObject> robotsNew;

    const double kMatchDistance = std::max(
        0.1, get_parameter("obstacle_avoidance.robot_match_distance").as_double());
    const double kMergeDistance = std::max(
        0.0, get_parameter("obstacle_avoidance.robot_merge_distance").as_double());
    const double kMemoryMsecs = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_obstacle_memory_msecs").as_double());
    const double kRobotMinConfidence = std::clamp(
        get_parameter("obstacle_avoidance.robot_min_confidence").as_double(),
        0.0,
        100.0);
    const double trackingAlpha = std::clamp(
        get_parameter("obstacle_avoidance.robot_tracking_alpha").as_double(),
        0.0,
        1.0);
    const double trackingBeta = std::clamp(
        get_parameter("obstacle_avoidance.robot_tracking_beta").as_double(),
        0.0,
        1.0);

    const double uprightDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_obstacle_distance").as_double());
    const double fallenDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.fallen_robot_distance").as_double());
    const double uprightRadius = std::max(
        0.0,
        get_parameter("obstacle_avoidance.upright_robot_radius").as_double());
    const double fallenRadius = std::max(
        0.0,
        get_parameter("obstacle_avoidance.fallen_robot_radius").as_double());
    const double maxRobotRange = std::max(
        uprightDistance + uprightRadius,
        fallenDistance + fallenRadius);
    const double maxFootprintWidth = std::max(
        0.0,
        get_parameter("obstacle_avoidance.max_robot_footprint_width").as_double());

    // Reject malformed projections before they enter the tracker.  Without
    // this guard a single ground-plane failure can be remembered as a real
    // obstacle for the complete memory window.
    vector<GameObject> validDetections;
    validDetections.reserve(robotObjs.size());
    for (auto robot : robotObjs) {
        const double bboxWidth = robot.boundingBox.xmax - robot.boundingBox.xmin;
        const double bboxHeight = robot.boundingBox.ymax - robot.boundingBox.ymin;
        const bool validBox =
            std::isfinite(robot.boundingBox.xmin) &&
            std::isfinite(robot.boundingBox.xmax) &&
            std::isfinite(robot.boundingBox.ymin) &&
            std::isfinite(robot.boundingBox.ymax) &&
            bboxWidth >= 2.0 && bboxHeight >= 2.0;
        const double range = std::hypot(robot.posToField.x - data->robotPoseToField.x,
                                        robot.posToField.y - data->robotPoseToField.y);
        if (!std::isfinite(robot.confidence) ||
            robot.confidence < kRobotMinConfidence ||
            !std::isfinite(robot.posToRobot.x) ||
            !std::isfinite(robot.posToRobot.y) ||
            robot.posToRobot.x <= 0.0 ||
            !std::isfinite(range) || range > maxRobotRange ||
            !validBox) {
            continue;
        }

        if (robot.obstacleFootprintValid) {
            const double width = std::hypot(
                robot.obstacleLeftToField.x - robot.obstacleRightToField.x,
                robot.obstacleLeftToField.y - robot.obstacleRightToField.y);
            const double midpointError = std::hypot(
                0.5 * (robot.obstacleLeftToField.x + robot.obstacleRightToField.x) -
                    robot.posToField.x,
                0.5 * (robot.obstacleLeftToField.y + robot.obstacleRightToField.y) -
                    robot.posToField.y);
            if (!std::isfinite(width) || width < 0.05 ||
                (maxFootprintWidth > 0.0 && width > maxFootprintWidth) ||
                !std::isfinite(midpointError) || midpointError > 1.5) {
                robot.obstacleFootprintValid = false;
            }
        }
        validDetections.push_back(robot);
    }

    const auto mergeNearbyRobots = [&](const vector<GameObject> &input) {
        vector<robot_obstacle_policy::Position2D> positions;
        positions.reserve(input.size());
        for (const auto &robot : input) {
            positions.push_back({robot.posToField.x, robot.posToField.y});
        }
        const auto clusters = robot_obstacle_policy::clusterNearbyPositions(
            positions, kMergeDistance);
        vector<GameObject> mergedRobots;
        mergedRobots.reserve(clusters.size());
        for (const auto &cluster : clusters) {
            if (cluster.empty()) continue;
            std::size_t representative = cluster.front();
            for (const std::size_t index : cluster) {
                const auto &candidate = input[index];
                const auto &current = input[representative];
                if (candidate.timePoint.nanoseconds() >
                        current.timePoint.nanoseconds() ||
                    (candidate.timePoint.nanoseconds() ==
                         current.timePoint.nanoseconds() &&
                     candidate.confidence > current.confidence)) {
                    representative = index;
                }
            }

            GameObject merged = input[representative];
            for (const std::size_t index : cluster) {
                const auto &candidate = input[index];
                merged.obstacleSeenCount = std::max(
                    merged.obstacleSeenCount, candidate.obstacleSeenCount);
                merged.obstacleMissedCount = std::min(
                    merged.obstacleMissedCount, candidate.obstacleMissedCount);
                if (candidate.fallenConfidence > merged.fallenConfidence) {
                    merged.fallenConfidence = candidate.fallenConfidence;
                    merged.fallen = candidate.fallen;
                }
                if (std::fabs(candidate.uprightScore) >
                    std::fabs(merged.uprightScore)) {
                    merged.uprightScore = candidate.uprightScore;
                    merged.fallen = candidate.fallen;
                }
            }
            mergedRobots.push_back(std::move(merged));
        }
        return mergedRobots;
    };

    validDetections = mergeNearbyRobots(validDetections);
    mergedRobotDetectionCount_.store(validDetections.size());

    // Build all viable detection/track pairs, then assign the shortest pairs
    // first.  This removes dependence on detector output order and reduces
    // identity swaps when nearby robots cross.
    vector<robot_obstacle_policy::Position2D> detectionPositions;
    detectionPositions.reserve(validDetections.size());
    for (const auto &robot : validDetections) {
        detectionPositions.push_back({robot.posToField.x, robot.posToField.y});
    }
    vector<robot_obstacle_policy::Position2D> trackPositions;
    trackPositions.reserve(robots.size());
    const double invalidPosition = std::numeric_limits<double>::quiet_NaN();
    for (const auto &robot : robots) {
        if (msecsSince(robot.timePoint) <= kMemoryMsecs) {
            trackPositions.push_back({robot.posToField.x, robot.posToField.y});
        } else {
            trackPositions.push_back({invalidPosition, invalidPosition});
        }
    }
    const vector<int> detectionMatches =
        robot_obstacle_policy::matchNearestTracks(
            detectionPositions, trackPositions, kMatchDistance);

    for (size_t detectionIndex = 0; detectionIndex < validDetections.size(); ++detectionIndex) {
        auto robot = validDetections[detectionIndex];
        const int minIndex = detectionMatches[detectionIndex];
        const bool hasMatch = minIndex >= 0;
        if (hasMatch) {
            matched[minIndex] = true;
            const GameObject &previous = robots[minIndex];
            const double measuredX = robot.posToField.x;
            const double measuredY = robot.posToField.y;
            double dt = std::numeric_limits<double>::quiet_NaN();
            if (robot.timePoint.nanoseconds() > 0 &&
                previous.timePoint.nanoseconds() > 0 &&
                robot.timePoint.get_clock_type() ==
                    previous.timePoint.get_clock_type()) {
                dt = (robot.timePoint - previous.timePoint).seconds();
            }
            const bool validDt = std::isfinite(dt) && dt >= 0.01 && dt <= 1.0;
            const double previousVx = previous.obstacleVelocityValid
                ? previous.obstacleVelocityToField.x : 0.0;
            const double previousVy = previous.obstacleVelocityValid
                ? previous.obstacleVelocityToField.y : 0.0;
            double filteredX = trackingAlpha * measuredX +
                (1.0 - trackingAlpha) * previous.posToField.x;
            double filteredY = trackingAlpha * measuredY +
                (1.0 - trackingAlpha) * previous.posToField.y;
            double velocityX = 0.0;
            double velocityY = 0.0;
            if (validDt) {
                // Alpha-beta update: predict with the previous velocity, then
                // correct both position and velocity from the measurement.
                const double predictedX = previous.posToField.x + previousVx * dt;
                const double predictedY = previous.posToField.y + previousVy * dt;
                const double residualX = measuredX - predictedX;
                const double residualY = measuredY - predictedY;
                filteredX = predictedX + trackingAlpha * residualX;
                filteredY = predictedY + trackingAlpha * residualY;
                velocityX = previousVx + trackingBeta * residualX / dt;
                velocityY = previousVy + trackingBeta * residualY / dt;
                const double velocityNorm = std::hypot(velocityX, velocityY);
                // A bad projection should not turn into an arbitrarily large
                // prediction that blocks the whole field.
                if (!std::isfinite(velocityNorm) || velocityNorm > 4.0) {
                    velocityX = 0.0;
                    velocityY = 0.0;
                }
            } else {
                velocityX = previousVx;
                velocityY = previousVy;
            }
            robot.posToField.x = filteredX;
            robot.posToField.y = filteredY;
            robot.obstacleVelocityToField = {velocityX, velocityY, 0.0};
            robot.obstacleVelocityValid = validDt || previous.obstacleVelocityValid;
            // Keep the footprint width estimate, then align its midpoint with
            // the alpha-beta corrected centre so planning uses one coherent
            // track instead of a filtered centre and stale endpoints.
            const auto alignFootprintMidpoint = [&]() {
                if (!robot.obstacleFootprintValid) return;
                const double midpointX = 0.5 * (
                    robot.obstacleLeftToField.x +
                    robot.obstacleRightToField.x);
                const double midpointY = 0.5 * (
                    robot.obstacleLeftToField.y +
                    robot.obstacleRightToField.y);
                if (!std::isfinite(midpointX) ||
                    !std::isfinite(midpointY)) {
                    robot.obstacleFootprintValid = false;
                    return;
                }
                const double shiftX = filteredX - midpointX;
                const double shiftY = filteredY - midpointY;
                robot.obstacleLeftToField.x += shiftX;
                robot.obstacleLeftToField.y += shiftY;
                robot.obstacleRightToField.x += shiftX;
                robot.obstacleRightToField.y += shiftY;
            };

            if (robot.obstacleFootprintValid &&
                previous.obstacleFootprintValid) {
                robot.obstacleLeftToField.x =
                    trackingAlpha * robot.obstacleLeftToField.x +
                    (1.0 - trackingAlpha) * previous.obstacleLeftToField.x;
                robot.obstacleLeftToField.y =
                    trackingAlpha * robot.obstacleLeftToField.y +
                    (1.0 - trackingAlpha) * previous.obstacleLeftToField.y;
                robot.obstacleRightToField.x =
                    trackingAlpha * robot.obstacleRightToField.x +
                    (1.0 - trackingAlpha) * previous.obstacleRightToField.x;
                robot.obstacleRightToField.y =
                    trackingAlpha * robot.obstacleRightToField.y +
                    (1.0 - trackingAlpha) * previous.obstacleRightToField.y;
                alignFootprintMidpoint();
            } else if (!robot.obstacleFootprintValid &&
                       previous.obstacleFootprintValid) {
                robot.obstacleLeftToField = previous.obstacleLeftToField;
                robot.obstacleRightToField = previous.obstacleRightToField;
                robot.obstacleFootprintValid = true;
                alignFootprintMidpoint();
            } else if (robot.obstacleFootprintValid) {
                // There is no previous footprint to filter against, but the
                // current endpoints still need to follow the filtered centre.
                alignFootprintMidpoint();
            }
            robot.obstacleSeenCount = previous.obstacleSeenCount ==
                    std::numeric_limits<uint32_t>::max()
                ? previous.obstacleSeenCount
                : previous.obstacleSeenCount + 1;
            updateRelativePos(robot);
        } else {
            robot.obstacleVelocityToField = {};
            robot.obstacleVelocityValid = false;
        }
        const double oldScore = hasMatch ? robots[minIndex].uprightScore : 0.0;
        const bool oldFallen = hasMatch ? robots[minIndex].fallen : false;
        const auto temporalState =
            fallen_robot_avoidance_policy::updateTemporalFallenState(
                oldScore,
                oldFallen,
                robot.fallen,
                robot.fallenConfidence);
        robot.uprightScore = temporalState.uprightScore;
        robot.fallen = temporalState.fallen;

        robotsNew.push_back(robot);
    }

    for (size_t i = 0; i < robots.size(); ++i) {
        if (!matched[i] && msecsSince(robots[i].timePoint) < kMemoryMsecs) {
            robotsNew.push_back(robots[i]);
        }
    }

    data->setRobots(mergeNearbyRobots(robotsNew));
}


void Brain::detectProcessVisionBox(const vision_interface::msg::Detections &msg) {
    constexpr size_t kVisionBoxCoordinateCount = 10;
    if (msg.corner_pos.size() < kVisionBoxCoordinateCount ||
        !std::all_of(
            msg.corner_pos.begin(),
            msg.corner_pos.begin() + kVisionBoxCoordinateCount,
            [](float value) { return std::isfinite(value); })) {
        // Vision intentionally omits ground intersections when the image has
        // no time-aligned head pose. Keep the previous valid view polygon.
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring detection view polygon without 5 finite ground points");
        return;
    }
    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    // Process and record field-of-view information.
    VisionBox vbox;
    vbox.timePoint = timePoint;
    vbox.posToRobot.assign(
        msg.corner_pos.begin(),
        msg.corner_pos.begin() + kVisionBoxCoordinateCount);

    // Handle top corners with negative x, which represent points at infinity.
    const double VISION_LIMIT = 20.0;
    vector<vector<double>> v = {};
    for (int i = 0; i < 4; i++) {
        int start = i; int end = (i + 1) % 4;
        v.push_back({vbox.posToRobot[end * 2] - vbox.posToRobot[start * 2], vbox.posToRobot[end * 2 + 1] - vbox.posToRobot[start * 2 + 1]});
        v.push_back({-vbox.posToRobot[end * 2] + vbox.posToRobot[start * 2], -vbox.posToRobot[end * 2 + 1] + vbox.posToRobot[start * 2 + 1]});
    }

    const auto extendToVisionLimit = [VISION_LIMIT](double value) {
        return std::fabs(value) > 1e-9
            ? -std::copysign(VISION_LIMIT, value)
            : 0.0;
    };
    for (int i = 0; i < 2; i++) {
        double ox = vbox.posToRobot[2* i]; double oy = vbox.posToRobot[2 * i + 1];
        if (
            (i == 0 && crossProduct(v[5], v[6]) < 0)
            || (i == 1 && crossProduct(v[3], v[4]) < 0)
        ){
            vbox.posToRobot[2 * i] = extendToVisionLimit(ox);
            vbox.posToRobot[2 * i + 1] = extendToVisionLimit(oy);
        }
    }

    // Transform to the field frame.
    for (int i = 0; i < 5; i++) {
        double xr, yr, xf, yf, __;
        xr = vbox.posToRobot[2 * i];
        yr = vbox.posToRobot[2 * i + 1];
        transCoord(
            xr, yr, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            xf, yf, __
        );
        vbox.posToField.push_back(xf);
        vbox.posToField.push_back(yf);
    }

    // Commit the results to BrainData atomically.
    data->visionBox = vbox;
}

void Brain::logVisionBox(const rclcpp::Time &timePoint) {
    if (!log->shouldLog("vision_box_visual", config->rerunLogVisualHz))
        return;

    if (data->visionBox.posToField.size() >= 10) {
        auto v = data->visionBox.posToField;

        rerun::LineStrip2D logLines({{v[0], -v[1]}, {v[2], -v[3]}, {v[4], -v[5]}, {v[6], -v[7]}, {v[0], -v[1]}});
        log->setTimeSeconds(timePoint.seconds());
        log->log(
            "field/visionBox",
            rerun::LineStrips2D(logLines)
            .with_colors(0x0000FFCC)
            .with_radii(0.01)
        );
    }
}

void Brain::logDetection(const vector<GameObject> &gameObjects, bool logBoundingBox) {
    if (!log->shouldLog("detection_visual", config->rerunLogVisualHz))
        return;

    if (gameObjects.size() == 0) {
        if (logBoundingBox) log->log("image/detection_boxes", rerun::Clear::FLAT);
        log->log("field/detection_points", rerun::Clear::FLAT);
        // log->log("robotframe/detection_points", rerun::Clear::FLAT);
        return;
    }

    // else
    rclcpp::Time timePoint = gameObjects[0].timePoint;
    log->setTimeSeconds(timePoint.seconds());

    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
        {"PenaltyPoint", rerun::Color(0xFF9900FF)},
    };

    // for logging boundingBoxes
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame
    vector<double> radiis;

    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        labels.push_back(rerun::Text(
            format("%s x:%.2f y:%.2f c:%.1f",
                label == "Opponent" || label == "Person" ? (label + "[" + obj.color + "]").c_str() : label.c_str(),
                obj.posToRobot.x,
                obj.posToRobot.y,
                obj.confidence)
            )
        );
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y}); // Flip y for Rerun Viewer's left-handed coordinates.
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        // if (obj.label == "Opponent") radiis.push_back(0.5);
        radiis.push_back(0.1);

        auto color = rerun::Color(0xFFFFFFFF);

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            color = detectColorMap[label];
        }
        else
        {
            // do nothing, use default
            // colors.push_back(rerun::Color(0xFFFFFFFF));
        }
        if (label == "Ball" && isBallOut(0.2, 10.0))
            color = rerun::Color(0x000000FF);
        if (label == "Ball" && obj.confidence < config->ballConfidenceThreshold)
            color = rerun::Color(0xAAAAAAFF);
        colors.push_back(color);
    }


    if (logBoundingBox) log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));

    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
                 .with_radii(radiis)
             // .with_labels(labels)
    );
    // log->log("robotframe/detection_points",
    //          rerun::Points2D(points_r)
    //              .with_colors(colors)
    //              .with_radii(radiis)
    //          // .with_labels(labels)
    // );
}


void Brain::logMemRobots() {
    if (!log->shouldLog("memory_robots_visual", config->rerunLogVisualHz))
        return;

    auto rbts = data->getRobots();
    // prtDebug(format("logMemRobots called, robotsize = %d", rbts.size()), RED_CODE);

    if (rbts.size() == 0) {
        log->log("field/mem_robots", rerun::Clear::FLAT);
        log->log("field/robots", rerun::Clear::FLAT);
        log->log("field/robot_obstacle_radius", rerun::Clear::FLAT);
        log->log("field/robot_footprints", rerun::Clear::FLAT);
        // log->log("robotframe/mem_robots", rerun::Clear::FLAT);
        return;
    }

    // else
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles;
    vector<rerun::LineStrip2D> footprints;
    vector<string> labels;
    for (int i = 0; i < rbts.size(); i++)
    {
        auto rbt = rbts[i];
        const double obstacleRadius = robotObstacleRadius(rbt);
        log->logRobot("field/robots", Pose2D({rbt.posToField.x, rbt.posToField.y, -M_PI}), 0xFF0000FF);
        circles.push_back(log->circle(
            rbt.posToField.x,
            -rbt.posToField.y,
            obstacleRadius));
        labels.push_back(format(
            "%s seen=%u radius=%.2f",
            rbt.fallen ? "fallen" : "upright",
            rbt.obstacleSeenCount,
            obstacleRadius));
        if (rbt.obstacleFootprintValid) {
            footprints.emplace_back(rerun::LineStrip2D({
                {rbt.obstacleLeftToField.x, -rbt.obstacleLeftToField.y},
                {rbt.obstacleRightToField.x, -rbt.obstacleRightToField.y},
            }));
        }
    }

    log->log(
        "field/robot_obstacle_radius",
        rerun::LineStrips2D(circles)
            .with_colors(0xFF0000AA)
            .with_radii(0.01)
            .with_labels(labels));
    if (!footprints.empty()) {
        log->log(
            "field/robot_footprints",
            rerun::LineStrips2D(footprints)
                .with_colors(0xFFFF00FF)
                .with_radii(0.03));
    } else {
        log->log("field/robot_footprints", rerun::Clear::FLAT);
    }
}

void Brain::logObstacles() {
    if (!log->shouldLog("obstacles_visual", config->rerunLogVisualHz))
        return;

    // log->setTimeNow();
    // time is set on the outside

    // Record obstacles represented by occupied grid cells.
    auto obs = data->getObstacles();
    vector<rerun::Vec2D> points;
    vector<rerun::Color> colors;
    vector<rerun::Text> labels;
    const int occThreshold = static_cast<int>(std::clamp<int64_t>(
        get_parameter("obstacle_avoidance.occupancy_threshold").as_int(),
        int64_t{0}, static_cast<int64_t>(std::numeric_limits<int>::max())));
    const double configuredMemoryMsecs =
        get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double();
    const double memoryMsecs = std::isfinite(configuredMemoryMsecs)
        ? std::max(1.0, configuredMemoryMsecs)
        : 1.0;
    for (int i = 0; i < obs.size(); i++) {
        auto o = obs[i];

        if (o.confidence < occThreshold) continue; // Removing this filter logs confidence levels in different colors.

        points.push_back(rerun::Vec2D{o.posToField.x, -o.posToField.y});
        auto age = msecsSince(o.timePoint);
        const double fade = std::clamp(1.0 - age / memoryMsecs, 0.0, 1.0);
        uint8_t alpha = static_cast<uint8_t>(std::lround(255.0 * fade));
        uint32_t color = (o.confidence > occThreshold) ? (0xFF000000 | alpha) : (0xFFFF0000 | alpha);
        colors.push_back(rerun::Color(color));

        labels.push_back(rerun::Text(format("count: %.0f age: %.0fms", o.confidence, age)));
    }
    if (points.empty()) {
        log->log("field/obstacles", rerun::Clear::FLAT);
        return;
    }
    log->log(
        "field/obstacles",
        rerun::Points2D(points)
        .with_colors(colors)
        .with_labels(labels)
        .with_radii(0.1)
    );
}

void Brain::logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid_occupied, vector<rerun::Vec3D> &points_robot) {
    if (!log->shouldLog("depth_visual", config->rerunLogVisualHz))
        return;

    // time is set on the outside
    const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // Grid size
    const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
    const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
    const double y_max = -y_min;
    const double groundHeight = get_parameter("obstacle_avoidance.ground_height").as_double();

    // Record the raw point cloud and grid.
    vector<rerun::Position3D> vertices;
    vector<rerun::Color> vertex_colors;
    vector<array<uint32_t, 3>> triangle_indices;
    const int OCCUPANCY_THRESHOLD = get_parameter("obstacle_avoidance.occupancy_threshold").as_int(); // Visualization threshold

    for (int i = 0; i < grid_x_count; i++) {
        for (int j = 0; j < grid_y_count; j++) {
            if (grid_occupied[i][j] > 0) {
                // Calculate the four corners of an occupied cell.
                double x0 = x_min + i * grid_size;
                double y0 = y_min + j * grid_size;
                double x1 = x0 + grid_size;
                double y1 = y0 + grid_size;

                // Add the four vertices.
                uint32_t base_index = vertices.size();
                vertices.push_back({x0, y0, groundHeight});
                vertices.push_back({x1, y0, groundHeight});
                vertices.push_back({x1, y1, groundHeight});
                vertices.push_back({x0, y1, groundHeight});

                // Select color from occupancy level.
                rerun::Color color;
                if (grid_occupied[i][j] >= OCCUPANCY_THRESHOLD &&
                    OCCUPANCY_THRESHOLD > 0) {
                    color = rerun::Color(255, 0, 0, 255);  // RGBA red
                } else {
                    color = rerun::Color(255, 255, 0, 255);  // RGBA yellow
                }
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);

                // Add two triangular faces.
                triangle_indices.push_back({base_index, base_index + 1, base_index + 2});
                triangle_indices.push_back({base_index, base_index + 2, base_index + 3});
            }
        }
    }

    vector<uint32_t> point_colors;
    const double obstacleMinHeight = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
    for (auto &point : points_robot) {
        float z_val = std::clamp(
            static_cast<float>(point.z() - groundHeight), 0.0f, 1.0f);
        if (z_val < obstacleMinHeight) {
            point_colors.push_back(0x0000FFFF);
        } else {
            uint8_t r = static_cast<uint8_t>(z_val * 255);
            uint8_t g = static_cast<uint8_t>((1 - z_val) * 255);
            point_colors.push_back((r << 24) | (g << 16) | 0xFF);
        }
    }

    log->log("depth/depth_points",
            rerun::Points3D(points_robot)
                .with_radii(0.01)
                .with_colors(point_colors)
    );

    log->log("depth/grid_mesh",
            rerun::Mesh3D(vertices)
                .with_vertex_colors(vertex_colors)
                .with_triangle_indices(triangle_indices)
    );

    // Record the ball-exclusion box.
    double r = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
    double h = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
    log->log(
        "depth/ball_exclusion_box",
        rerun::Boxes3D::from_centers_and_half_sizes(
            {{ data->ball.posToRobot.x, data->ball.posToRobot.y, groundHeight + h/2}},
            {{ r, r, h/2}})
        .with_colors(0x00FF0044)     // Translucent green
    );
}

void Brain::logDebugInfo() {
    const bool logTickDebug =
        log->shouldLog("brain_tick_debug", config->rerunLogDebugHz);
    const bool logTickTimeseries =
        log->shouldLog("brain_state_timeseries", config->rerunLogTimeseriesHz);
    if (!logTickDebug && !logTickTimeseries)
        return;

    auto log_ = [=](string msg) {
        if (logTickDebug) {
            log->setTimeNow();
            log->log("debug/brain_tick", rerun::TextLog(msg));
        }
    };
    string gameState = tree->getEntry<string>("gc_game_state");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string isLead = data->tmImLead ? "ON" : "OFF";
    string ballOut = tree->getEntry<bool>("ball_out") ? "YES" : "NO";
    string ballDetected = data->ballDetected ? "YES" : "NO";
    string decision = tree->getEntry<string>("decision");
    string freeKickKickingOff = data->isFreekickKickingOff ? "YES" : "NO";
    string directShoot = data->isDirectShoot ? "YES" : "NO";
    string primaryStriker = isPrimaryStriker() ? "YES" : "NO";
    log_(format("Game State: %s, SubState: %s, SubStateType: %s, Lead: %s, Decision: %s, FreeKickKickingOff: %s, DirectShoot: %s, PrimaryStriker: %s",
        gameState.c_str(), gameSubState.c_str(), gameSubStateType.c_str(), isLead.c_str(), decision.c_str(), freeKickKickingOff.c_str(), directShoot.c_str(), primaryStriker.c_str()));

    if (logTickTimeseries) {
        log->setTimeNow();
        log->log("debug/my_cost_scalar", rerun::Scalar(data->tmMyCost));
        log->log("debug/my_lead_scalar", rerun::Scalar(data->tmImLead));
    }
}

void Brain::updateRelativePos(GameObject &obj) {
    Pose2D pf;
    pf.x = obj.posToField.x;
    pf.y = obj.posToField.y;
    pf.theta = 0;
    Pose2D pr = data->field2robot(pf);
    obj.posToRobot.x = pr.x;
    obj.posToRobot.y = pr.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = atan2(config->robotHeight, obj.range);

    if (obj.obstacleFootprintValid) {
        const Pose2D leftField{obj.obstacleLeftToField.x,
                               obj.obstacleLeftToField.y, 0.0};
        const Pose2D rightField{obj.obstacleRightToField.x,
                                obj.obstacleRightToField.y, 0.0};
        const Pose2D leftRobot = data->field2robot(leftField);
        const Pose2D rightRobot = data->field2robot(rightField);
        obj.obstacleLeftToRobot.x = leftRobot.x;
        obj.obstacleLeftToRobot.y = leftRobot.y;
        obj.obstacleRightToRobot.x = rightRobot.x;
        obj.obstacleRightToRobot.y = rightRobot.y;
    }
}

void Brain::updateFieldPos(GameObject &obj) {
    Pose2D pr;
    pr.x = obj.posToRobot.x;
    pr.y = obj.posToRobot.y;
    pr.theta = 0;
    Pose2D pf = data->robot2field(pr);
    obj.posToField.x = pf.x;
    obj.posToField.y = pf.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = atan2(config->robotHeight, obj.range);

    if (obj.obstacleFootprintValid) {
        double unusedTheta = 0.0;
        transCoord(
            obj.obstacleLeftToRobot.x,
            obj.obstacleLeftToRobot.y,
            0.0,
            data->robotPoseToField.x,
            data->robotPoseToField.y,
            data->robotPoseToField.theta,
            obj.obstacleLeftToField.x,
            obj.obstacleLeftToField.y,
            unusedTheta);
        transCoord(
            obj.obstacleRightToRobot.x,
            obj.obstacleRightToRobot.y,
            0.0,
            data->robotPoseToField.x,
            data->robotPoseToField.y,
            data->robotPoseToField.theta,
            obj.obstacleRightToField.x,
            obj.obstacleRightToField.y,
            unusedTheta);
    }
}

void Brain::depthImageCallback(const sensor_msgs::msg::Image &msg)
{
    try {
        if (isRecoveryActive()) {
            // Match B-Human's obstacle-model lifecycle: self-body depth during
            // a fall/get-up must never survive into the walking state.
            data->setObstacles({});
            return;
        }
        // Validate image data.
        if (msg.data.empty() || msg.height == 0 || msg.width == 0) {
            RCLCPP_WARN(get_logger(), "Received empty depth image");
            return;
        }

        // Convert every supported T2 encoding to metres.  In particular, do
        // not assume that Image::data is tightly packed: the SDK may pad step.
        cv::Mat depthFloat;
        std::string decodeError;
        if (!decodeDepthImage(
                msg, config->depthScale16U, config->depthScale32F,
                depthFloat, decodeError)) {
            RCLCPP_ERROR(
                get_logger(), "Unable to decode depth image (%s): %s",
                msg.encoding.c_str(), decodeError.c_str());
            return;
        }
        const rclcpp::Time depthTimePoint = timePointFromHeader(msg.header);

        vector<rerun::Vec3D> points_robot;  // for log

        Eigen::Matrix4d cameraToRobot;
        Point ballToRobot;
        Pose2D robotPoseToField;
        double viewYaw = 0.0;
        double fx = 0.0;
        double fy = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        double intrinsicsWidth = 0.0;
        double intrinsicsHeight = 0.0;
        bool cameraTransformValid = false;
        double cameraPoseDiffMsecs = std::numeric_limits<double>::infinity();
        {
            std::lock_guard<std::recursive_mutex> stateLock(
                recoveryStateMutex_);
            cameraToRobot = data->camToRobot;
            cameraTransformValid = cameraTransformValid_;

            if (timestampedHeadPoseSeen_.load()) {
                cameraTransformValid = false;
                const rclcpp::Time depthStamp = depthTimePoint;
                if (depthStamp.nanoseconds() > 0 &&
                    !cameraTransformBuffer_.empty()) {
                    const TimedCameraTransform *nearest = nullptr;
                    int64_t nearestDiffNsecs = std::numeric_limits<int64_t>::max();
                    for (const auto &sample : cameraTransformBuffer_) {
                        int64_t diffNsecs =
                            sample.stamp.nanoseconds() - depthStamp.nanoseconds();
                        if (diffNsecs < 0) diffNsecs = -diffNsecs;
                        if (diffNsecs < nearestDiffNsecs) {
                            nearest = &sample;
                            nearestDiffNsecs = diffNsecs;
                        }
                    }

                    cameraPoseDiffMsecs = nearestDiffNsecs / 1e6;
                    const double maxPoseDiffMsecs = std::max(
                        0.0,
                        get_parameter("vision.depth_pose_max_sync_diff_ms")
                            .as_double());
                    if (nearest != nullptr &&
                        cameraPoseDiffMsecs <= maxPoseDiffMsecs) {
                        cameraToRobot = nearest->cameraToRobot;
                        cameraTransformValid = true;
                    }
                }
            }
            ballToRobot = data->ball.posToRobot;
            robotPoseToField = data->robotPoseToField;
            viewYaw = data->headYaw;
            fx = config->depthfx;
            fy = config->depthfy;
            cx = config->depthcx;
            cy = config->depthcy;
            intrinsicsWidth = config->camPixX;
            intrinsicsHeight = config->camPixY;
        }
        if (!cameraTransformValid || !cameraToRobot.allFinite()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Ignoring depth image: no synchronized T2 head pose "
                "(nearest difference %.1f ms)",
                cameraPoseDiffMsecs);
            return;
        }

        // Depth pixels are projected with depth intrinsics.  For an aligned
        // T2 stream these are the RGB intrinsics; a separate depth_intrin map
        // in vision.yaml can override them for an unaligned stream.
        if (config->depthAlignedToColor && intrinsicsWidth > 0.0 &&
            intrinsicsHeight > 0.0 &&
            (std::fabs(static_cast<double>(msg.width) - intrinsicsWidth) > 0.5 ||
             std::fabs(static_cast<double>(msg.height) - intrinsicsHeight) > 0.5)) {
            // The configured T2 intrinsics are for the RGB/depth native
            // resolution.  Scale them when the driver emits a resized stream.
            const double sx = static_cast<double>(msg.width) / intrinsicsWidth;
            const double sy = static_cast<double>(msg.height) / intrinsicsHeight;
            fx *= sx;
            cx *= sx;
            fy *= sy;
            cy *= sy;
        }
        if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0 ||
            !std::isfinite(cx) || !std::isfinite(cy)) {
            RCLCPP_ERROR(get_logger(), "Invalid T2 depth intrinsics");
            return;
        }
        const Eigen::Matrix3d cameraRotation = cameraToRobot.block<3, 3>(0, 0);
        // Use the pose paired with this depth frame for visibility bookkeeping.
        // The legacy headYaw value can belong to a newer frame while the head
        // is moving, which would otherwise preserve/erase the wrong cells.
        const Eigen::Vector3d cameraForward =
            cameraRotation * Eigen::Vector3d::UnitZ();
        if (std::isfinite(cameraForward.x()) &&
            std::isfinite(cameraForward.y()) &&
            std::hypot(cameraForward.x(), cameraForward.y()) > 1e-6) {
            viewYaw = std::atan2(cameraForward.y(), cameraForward.x());
        }
        // cout << "fx = " << fx << " fy = " << fy << " cx = " << cx << " cy = " << cy << endl;

        // Define grid parameters.
        const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // Grid size
        const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
        const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
        const double y_max = -y_min;
        if (!std::isfinite(grid_size) || grid_size <= 0.0 ||
            !std::isfinite(x_max) || x_max <= x_min ||
            !std::isfinite(y_min) || !std::isfinite(y_max) || y_max <= y_min) {
            RCLCPP_ERROR(get_logger(), "Invalid T2 obstacle grid parameters");
            return;
        }
        const int grid_x_count = static_cast<int>((x_max - x_min) / grid_size);
        const int grid_y_count = static_cast<int>((y_max - y_min) / grid_size);
        if (grid_x_count <= 0 || grid_y_count <= 0) {
            RCLCPP_ERROR(get_logger(), "T2 obstacle grid has no cells");
            return;
        }

        // Create the occupancy grid.
        vector<vector<int>> grid_occupied(grid_x_count, vector<int>(grid_y_count, 0));

        // Process depth-image points.
        const int64_t sampleStepParam =
            get_parameter("obstacle_avoidance.depth_sample_step").as_int();
        const int sampleStep = static_cast<int>(std::clamp<int64_t>(
            sampleStepParam,
            int64_t{1},
            static_cast<int64_t>(std::numeric_limits<int>::max())));
        const int clearFrames = static_cast<int>(std::clamp<int64_t>(
            get_parameter("obstacle_avoidance.depth_clear_frames").as_int(),
            int64_t{1}, int64_t{100}));
        const double groundHeight = get_parameter("obstacle_avoidance.ground_height").as_double();
        const double obstacleMinHeight = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
        const double excludeMaxX = get_parameter("obstacle_avoidance.exclusion_x").as_double();
        const double excludeMaxY = get_parameter("obstacle_avoidance.exclusion_y").as_double();
        const double excludeMinX = -excludeMaxX;
        const double excludeMinY = -excludeMaxY;
        const double ballExclusionRadius = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
        const double ballExclusionHeight = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
        const int occupancyThreshold = static_cast<int>(std::clamp<int64_t>(
            get_parameter("obstacle_avoidance.occupancy_threshold").as_int(),
            int64_t{0},
            static_cast<int64_t>(std::numeric_limits<int>::max())));
        if (!std::isfinite(groundHeight) || !std::isfinite(obstacleMinHeight) ||
            obstacleMinHeight < 0.0 ||
            !std::isfinite(excludeMaxX) || excludeMaxX < 0.0 ||
            !std::isfinite(excludeMaxY) || excludeMaxY < 0.0 ||
            !std::isfinite(ballExclusionRadius) || ballExclusionRadius < 0.0 ||
            !std::isfinite(ballExclusionHeight) || ballExclusionHeight < 0.0) {
            RCLCPP_ERROR(get_logger(), "Invalid T2 obstacle height/exclusion parameters");
            return;
        }
        const auto logDepthSnapshot = [&](const char *reason) {
            ObstacleAvoidanceLogRecord record;
            record.source = "depth";
            record.reason = reason;
            record.depthImageWidth = static_cast<int>(msg.width);
            record.depthImageHeight = static_cast<int>(msg.height);
            record.depthGridWidth = grid_x_count;
            record.depthGridHeight = grid_y_count;
            for (const auto &column : grid_occupied) {
                for (const int occupancy : column) {
                    if (occupancy > 0) ++record.depthGridOccupiedCells;
                    record.depthGridMaxOccupancy = std::max(
                        record.depthGridMaxOccupancy, occupancy);
                    if (occupancyThreshold > 0 &&
                        occupancy >= occupancyThreshold) {
                        ++record.depthGridConfirmedCells;
                    }
                }
            }
            depthGridOccupiedCells_.store(record.depthGridOccupiedCells);
            depthGridMaxOccupancy_.store(record.depthGridMaxOccupancy);
            depthGridConfirmedCells_.store(record.depthGridConfirmedCells);
            logObstacleAvoidance(std::move(record));
        };
        const int64_t sampleColumns =
            (static_cast<int64_t>(msg.width) + sampleStep - 1) / sampleStep;
        const int64_t sampleRows =
            (static_cast<int64_t>(msg.height) + sampleStep - 1) / sampleStep;
        const int64_t sampleCount = sampleColumns * sampleRows;
        if (occupancyThreshold > 0 && sampleCount < occupancyThreshold) {
            // With too few pixels in the entire frame no grid cell can reach
            // the confirmation threshold. Retaining the previous cells here
            // would turn an invalid sampling setting into false obstacles.
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "depth_sample_step=%d yields only %ld samples for %ux%u image; "
                "occupancy_threshold=%d cannot be reached, clearing depth obstacles",
                sampleStep,
                static_cast<long>(sampleCount),
                msg.width,
                msg.height,
                occupancyThreshold);
            depthSamplingInvalid_.store(true);
            data->setObstacles({});
            log->setTimeSeconds(depthTimePoint.seconds());
            logDepth(grid_x_count, grid_y_count, grid_occupied, points_robot);
            logObstacles();
            logDepthSnapshot("depth_invalid");
            return;
        }
        depthSamplingInvalid_.store(false);
        for (int y = 0; y < msg.height; y += sampleStep)
        {
            for (int x = 0; x < msg.width; x += sampleStep)
            {
                float depth = depthFloat.at<float>(y, x);
                if (std::isfinite(depth) && depth > 0.0f)
                {
                    // Transform to the camera frame.
                    const double rayX = (static_cast<double>(x) - cx) / fx;
                    const double rayY = (static_cast<double>(y) - cy) / fy;
                    // Standard ROS depth is optical-axis Z.  Keep a switch
                    // for SDKs that publish Euclidean ray range instead.
                    const double z_cam = config->depthIsZ
                        ? static_cast<double>(depth)
                        : static_cast<double>(depth) /
                            std::sqrt(rayX * rayX + rayY * rayY + 1.0);
                    const double x_cam = rayX * z_cam;
                    const double y_cam = rayY * z_cam;

                    // Transform to the robot frame.
                    Eigen::Vector4d point_cam(x_cam, y_cam, z_cam, 1.0);
                    Eigen::Vector4d point_robot = cameraToRobot * point_cam;
                    if (!point_robot.allFinite() ||
                        point_robot(3) <= 0.0 ||
                        std::fabs(point_robot(3) - 1.0) > 1e-3) {
                        continue;
                    }

                    // The avoidance map is a horizontal ground-plane map.
                    // A measured 3-D point is therefore projected vertically
                    // by retaining its transformed x/y and dropping z. Using
                    // the camera ray/ground intersection here would move
                    // pixels from a robot's head behind its actual footprint.
                    const double gridX = point_robot(0);
                    const double gridY = point_robot(1);
                    if (!std::isfinite(gridX) || !std::isfinite(gridY)) {
                        continue;
                    }

                    // Store the point for visualization.
                    points_robot.push_back(rerun::Vec3D{point_robot(0), point_robot(1), point_robot(2)});

                    // Update grid occupancy.

                    auto isInRange = [&]() {
                        return gridX >= x_min && gridX < x_max
                            && gridY >= y_min && gridY < y_max;
                    };
                    auto isSelfBody = [&]() {
                        return gridX >= excludeMinX && gridX <= excludeMaxX
                            && gridY >= excludeMinY && gridY <= excludeMaxY;
                    };
                    auto isBall = [&]() {
                        return fabs(gridX - ballToRobot.x) < ballExclusionRadius
                            && fabs(gridY - ballToRobot.y) < ballExclusionRadius
                            && point_robot(2) - groundHeight < ballExclusionHeight;
                    };

                    if (
                        std::isfinite(point_robot(0)) &&
                        std::isfinite(point_robot(1)) &&
                        std::isfinite(point_robot(2)) &&
                        point_robot(0) > 0.0 &&
                        point_robot(2) - groundHeight > obstacleMinHeight
                        && isInRange()
                        && !isSelfBody()
                        && !isBall()
                    )
                    {
                        int grid_x = static_cast<int>((gridX - x_min) / grid_size);
                        int grid_y = static_cast<int>((gridY - y_min) / grid_size);
                        if (grid_x >= 0 && grid_x < grid_x_count &&
                            grid_y >= 0 && grid_y < grid_y_count) {
                            grid_occupied[grid_x][grid_y] += 1;
                        }
                    }
                }
            }
        }

        const double configuredMemoryMsecs = get_parameter(
            "obstacle_avoidance.obstacle_memory_msecs").as_double();
        const double obstacleMemoryMsecs = std::isfinite(configuredMemoryMsecs)
            ? std::max(0.0, configuredMemoryMsecs)
            : 0.0;
        vector<GameObject> obs_old;
        for (const auto &oldObstacle : data->getObstacles()) {
            // updateObstacleMemory() runs on the main tick while this depth
            // callback can run concurrently. Do not let a callback snapshot
            // reinsert an obstacle that the tick has already expired.
            if (oldObstacle.label == "Obstacle" &&
                msecsSince(oldObstacle.timePoint) > obstacleMemoryMsecs) {
                continue;
            }
            obs_old.push_back(oldObstacle);
        }
        // Obstacles are stored in both robot and field frames. Reproject the
        // previous field positions before merging with this depth frame so a
        // head turn or body motion cannot compare points from two robot-frame
        // origins as if they were current measurements.
        for (auto &oldObstacle : obs_old) {
            updateRelativePos(oldObstacle);
        }
        vector<bool> oldObstacleMatched(obs_old.size(), false);
        vector<GameObject> obs_new = {};

        // Add obstacles observed in this frame.
        for (int i = 0; i < grid_x_count; i++) {
            for (int j = 0; j < grid_y_count; j++) {
                // Keep only cells that can actually participate in collision
                // queries.  The complete grid remains available to logDepth.
                if (grid_occupied[i][j] >= occupancyThreshold &&
                    grid_occupied[i][j] > 0) {
                    GameObject obj;
                    obj.label = "Obstacle";
                    obj.timePoint = depthTimePoint;
                    obj.posToRobot.x = x_min + (i + 0.5) * grid_size;
                    obj.posToRobot.y = y_min + (j + 0.5) * grid_size;
                    obj.confidence = grid_occupied[i][j];
                    obj.obstacleSeenCount = 1;
                    obj.obstacleMissedCount = 0;
                    // A depth cell can move by a fraction of a grid cell as
                    // the head or robot moves. Carry its temporal evidence
                    // across that small displacement instead of requiring an
                    // exact cell hit on every frame.
                    double nearestOldDistance = std::numeric_limits<double>::infinity();
                    const GameObject *nearestOld = nullptr;
                    size_t nearestOldIndex = obs_old.size();
                    for (size_t oldIndex = 0;
                         oldIndex < obs_old.size(); ++oldIndex) {
                        if (oldObstacleMatched[oldIndex]) continue;
                        const auto &old = obs_old[oldIndex];
                        if (old.label != "Obstacle") continue;
                        const double oldDistance = std::hypot(
                            old.posToField.x -
                                (robotPoseToField.x +
                                 std::cos(robotPoseToField.theta) * obj.posToRobot.x -
                                 std::sin(robotPoseToField.theta) * obj.posToRobot.y),
                            old.posToField.y -
                                (robotPoseToField.y +
                                 std::sin(robotPoseToField.theta) * obj.posToRobot.x +
                                 std::cos(robotPoseToField.theta) * obj.posToRobot.y));
                        if (std::isfinite(oldDistance) &&
                            oldDistance < 0.75 * grid_size &&
                            oldDistance < nearestOldDistance) {
                            nearestOldDistance = oldDistance;
                            nearestOld = &old;
                            nearestOldIndex = oldIndex;
                        }
                    }
                    if (nearestOld != nullptr) {
                        oldObstacleMatched[nearestOldIndex] = true;
                        obj.obstacleSeenCount =
                            robot_obstacle_policy::nextDepthSeenCount(
                                nearestOld->obstacleSeenCount);
                    }
                    double unusedTheta = 0.0;
                    transCoord(
                        obj.posToRobot.x,
                        obj.posToRobot.y,
                        0.0,
                        robotPoseToField.x,
                        robotPoseToField.y,
                        robotPoseToField.theta,
                        obj.posToField.x,
                        obj.posToField.y,
                        unusedTheta);
                    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
                    obj.yawToRobot = atan2(
                        obj.posToRobot.y, obj.posToRobot.x);
                    obj.pitchToRobot = atan2(config->robotHeight, obj.range);
                    obs_new.push_back(obj);
                }
            }
        }

        // Remove stale obstacles while retaining cells for a small number of missed frames.
        // while it remains inside the current camera view. This is separate
        // from obstacle_memory_msecs, which handles head-scan blind spots.
        for (int i = 0; i < obs_old.size(); i++) {
            auto obs = obs_old[i];
            const double offset = 0.20;
            // Compare wrapped angular distance instead of raw left/right
            // bounds; otherwise a camera view crossing +/-pi leaves stale
            // obstacles behind indefinitely.
            const double obsYaw = std::atan2(
                obs.posToRobot.y, obs.posToRobot.x);
            const double obsHalfAngle = std::atan2(
                offset,
                std::max(1e-3, obs.posToRobot.x - offset));
            const double viewHalfAngle = std::max(
                0.0, config->camAngleX * 0.5) + obsHalfAngle;
            // Replace nearby old obstacles to prevent duplicates accumulating at cell boundaries.
            bool found = false;
            for (int j = 0; j < obs_new.size(); j++) {
                auto obs_n = obs_new[j];
                double dist = norm(obs.posToRobot.x - obs_n.posToRobot.x, obs.posToRobot.y - obs_n.posToRobot.y);
                if (dist < 0.75 * grid_size) {
                    found = true;
                    break;
                }
            }
            if (found) continue;

            if (std::fabs(toPInPI(obsYaw - viewYaw)) < viewHalfAngle) {
                if (obs.label == "Obstacle") {
                    obs.obstacleMissedCount =
                        robot_obstacle_policy::nextDepthMissedCount(
                            obs.obstacleMissedCount);
                    if (!robot_obstacle_policy::retainMissedDepthCell(
                            obs.obstacleMissedCount - 1, clearFrames)) {
                        continue;
                    }
                } else {
                    continue;
                }
            }

            // else
            obs_new.push_back(obs);
        }


        data->setObstacles(obs_new); // Stale obstacles are cleared in tick, not here.
        log->setTimeSeconds(depthTimePoint.seconds());
        logDepth(grid_x_count, grid_y_count, grid_occupied, points_robot);
        logObstacles();
        logDepthSnapshot("depth_update");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in depth image callback: %s", e.what());
    }
}

double Brain::robotObstacleRadius(const GameObject &robot) const
{
    const char *parameterName = robot.fallen
        ? "obstacle_avoidance.fallen_robot_radius"
        : "obstacle_avoidance.upright_robot_radius";
    const double configuredRadius = std::max(
        0.0, get_parameter(parameterName).as_double());
    if (!robot.obstacleFootprintValid) return configuredRadius;

    const double footprintWidth = std::hypot(
        robot.obstacleLeftToRobot.x - robot.obstacleRightToRobot.x,
        robot.obstacleLeftToRobot.y - robot.obstacleRightToRobot.y);
    if (!std::isfinite(footprintWidth)) return configuredRadius;
    const double maxFootprintWidth = std::max(
        0.0,
        get_parameter("obstacle_avoidance.max_robot_footprint_width").as_double());
    const double maxIncrease = std::max(
        0.0,
        get_parameter(
            "obstacle_avoidance.max_robot_footprint_radius_increase")
            .as_double());
    // The posture radius is already the final centre-line safety radius.
    // Only a bounded footprint correction is added; converting a noisy wide
    // projection into a full isotropic circle made every robot unnecessarily
    // large (and caused early, slow avoidance).
    return std::min(
        configuredRadius + maxIncrease,
        std::max(configuredRadius,
                 0.5 * std::min(footprintWidth, maxFootprintWidth)));
}

double Brain::selfRobotCollisionRadius() const
{
    const double configuredRadius = get_parameter(
        "obstacle_avoidance.self_robot_radius").as_double();
    return std::isfinite(configuredRadius)
        ? std::max(0.0, configuredRadius)
        : 0.0;
}

double Brain::robotObstacleMinimumDistance() const
{
    const double configuredDistance = get_parameter(
        "obstacle_avoidance.robot_obstacle_min_distance").as_double();
    return std::isfinite(configuredDistance)
        ? std::max(0.0, configuredDistance)
        : 0.0;
}

Point Brain::robotObstacleCenterToRobot(const GameObject &robot) const
{
    double centerFieldX = robot.posToField.x;
    double centerFieldY = robot.posToField.y;
    if (robot.obstacleFootprintValid) {
        centerFieldX = 0.5 * (
            robot.obstacleLeftToField.x + robot.obstacleRightToField.x);
        centerFieldY = 0.5 * (
            robot.obstacleLeftToField.y + robot.obstacleRightToField.y);
    }
    if (!std::isfinite(centerFieldX) || !std::isfinite(centerFieldY)) {
        return robot.posToRobot;
    }
    // Predict a remembered track to the current instant.  This compensates
    // for detector and callback latency without moving the stored track itself.
    if (robot.obstacleVelocityValid) {
        const double configuredHorizon = std::max(
            0.0,
            get_parameter(
                "obstacle_avoidance.robot_prediction_horizon_msecs")
                .as_double());
        // A clock-domain mismatch means the age is unknown; do not apply a
        // full-horizon extrapolation in that case.
        const auto now = get_clock()->now();
        const double ageSeconds = robot.timePoint.get_clock_type() ==
                now.get_clock_type()
            ? std::clamp(
                  std::max(0.0, (now - robot.timePoint).seconds()),
                  0.0,
                  configuredHorizon / 1000.0)
            : 0.0;
        centerFieldX += robot.obstacleVelocityToField.x * ageSeconds;
        centerFieldY += robot.obstacleVelocityToField.y * ageSeconds;
    }
    const Pose2D centerRobot = data->field2robot(
        Pose2D{centerFieldX, centerFieldY, 0.0});
    if (!std::isfinite(centerRobot.x) || !std::isfinite(centerRobot.y)) {
        return robot.posToRobot;
    }
    return Point{centerRobot.x, centerRobot.y, 0.0};
}

Point Brain::robotObstacleVelocityToRobot(const GameObject &robot) const
{
    if (!robot.obstacleVelocityValid ||
        !std::isfinite(robot.obstacleVelocityToField.x) ||
        !std::isfinite(robot.obstacleVelocityToField.y) ||
        !std::isfinite(data->robotPoseToField.theta)) {
        return {};
    }
    const double c = std::cos(data->robotPoseToField.theta);
    const double s = std::sin(data->robotPoseToField.theta);
    return {
        c * robot.obstacleVelocityToField.x +
            s * robot.obstacleVelocityToField.y,
        -s * robot.obstacleVelocityToField.x +
            c * robot.obstacleVelocityToField.y,
        0.0};
}

bool Brain::robotObstacleEmergency(
    const GameObject &robot,
    double pathAngle) const
{
    const double emergencyConfidence = std::clamp(
        get_parameter("obstacle_avoidance.robot_emergency_confidence")
            .as_double(),
        0.0,
        100.0);
    const double emergencyRange = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_emergency_range").as_double());
    const double emergencyDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_emergency_distance")
            .as_double());
    if (robot.obstacleSeenCount == 0 ||
        !std::isfinite(robot.confidence) ||
        robot.confidence < emergencyConfidence) {
        return false;
    }
    const Point center = robotObstacleCenterToRobot(robot);
    const double range = std::hypot(center.x, center.y);
    if (!std::isfinite(range) || range <= robotObstacleMinimumDistance() ||
        range > emergencyRange) {
        return false;
    }
    if (!std::isfinite(pathAngle)) return true;
    const auto intersection = fallen_robot_avoidance_policy::intersectsPath(
        center.x,
        center.y,
        pathAngle,
        emergencyRange,
        0.0,
        robotObstacleRadius(robot) + selfRobotCollisionRadius());
    return intersection.intersects &&
        intersection.entryDistance <= emergencyDistance;
}

robot_path_planner_policy::PathPlan Brain::planRobotPath(
    double goalX,
    double goalY,
    double extraClearance,
    double previousSide,
    bool preferPreviousSide,
    bool hardLockPreviousSide) const
{
    const uint32_t confirmFrames = static_cast<uint32_t>(std::clamp<int64_t>(
        get_parameter("obstacle_avoidance.robot_confirm_frames").as_int(),
        int64_t{1}, int64_t{100}));
    const double uprightDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_obstacle_distance").as_double());
    const double fallenDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.fallen_robot_distance").as_double());
    const double configuredClearance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_path_clearance").as_double());
    if (!std::isfinite(extraClearance) || extraClearance < 0.0) {
        extraClearance = configuredClearance;
    }

    const double selfRadius = selfRobotCollisionRadius();
    const double minimumRobotDistance = robotObstacleMinimumDistance();
    std::vector<robot_path_planner_policy::CircleObstacle> obstacles;
    for (const auto &robot : data->getRobots()) {
        if (robot.obstacleSeenCount < confirmFrames &&
            !robotObstacleEmergency(robot, std::atan2(goalY, goalX))) {
            continue;
        }
        const Point center = robotObstacleCenterToRobot(robot);
        const double robotDistance = std::hypot(center.x, center.y);
        if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
            !std::isfinite(robotDistance) ||
            robotDistance <= minimumRobotDistance ||
            robotDistance >
                std::max(uprightDistance, fallenDistance) + 1.0) {
            continue;
        }
        const double radius = robotObstacleRadius(robot) + selfRadius;
        const double rangeLimit = robot.fallen ? fallenDistance : uprightDistance;
        if (!std::isfinite(radius) || radius <= 0.0 ||
            std::hypot(center.x, center.y) - radius > rangeLimit + 1.0) {
            continue;
        }
        obstacles.push_back({{center.x, center.y}, radius});
    }

    const double switchPenalty = std::max(
        0.0,
        get_parameter(
            "obstacle_avoidance.robot_avoidance_switch_penalty")
            .as_double());
    return robot_path_planner_policy::planPath(
        {goalX, goalY},
        obstacles,
        extraClearance,
        previousSide,
        preferPreviousSide,
        switchPenalty,
        0.25,
        hardLockPreviousSide);
}

bool Brain::findFallenRobotOnPath(
    double pathAngle,
    GameObject &nearestRobot,
    double *entryDistance,
    const GameObject *ignoredRobot)
{
    const double maxDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.fallen_robot_distance").as_double());
    // Inflate the external robot region by this robot's own footprint. The
    // generic collision threshold remains reserved for depth-grid points.
    constexpr double robotClearance = 0.0;
    const double selfRadius = selfRobotCollisionRadius();
    const double minimumRobotDistance = robotObstacleMinimumDistance();
    const uint32_t robotConfirmFrames = static_cast<uint32_t>(
        std::clamp<int64_t>(
            get_parameter("obstacle_avoidance.robot_confirm_frames").as_int(),
            int64_t{1}, int64_t{100}));
    double nearestEntry = std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto &robot : data->getRobots()) {
        if (!robot.fallen ||
            (robot.obstacleSeenCount < robotConfirmFrames &&
             !robotObstacleEmergency(robot, pathAngle))) {
            continue;
        }
        if (ignoredRobot != nullptr &&
            std::hypot(
                robot.posToField.x - ignoredRobot->posToField.x,
                robot.posToField.y - ignoredRobot->posToField.y) < 0.2) {
            continue;
        }
        const Point center = robotObstacleCenterToRobot(robot);
        if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
            std::hypot(center.x, center.y) <= minimumRobotDistance) {
            continue;
        }
        const auto intersection = fallen_robot_avoidance_policy::intersectsPath(
            center.x,
            center.y,
            pathAngle,
            maxDistance,
            robotClearance,
            robotObstacleRadius(robot) + selfRadius);
        if (intersection.intersects && intersection.entryDistance < nearestEntry) {
            nearestEntry = intersection.entryDistance;
            nearestRobot = robot;
            found = true;
        }
    }

    if (entryDistance != nullptr) {
        *entryDistance = found
            ? nearestEntry
            : std::numeric_limits<double>::infinity();
    }
    return found;
}

bool Brain::isFallenRobotVisualKickExitEnabled() const
{
    return get_parameter(
        "obstacle_avoidance.enable_fallen_robot_visual_kick_exit").as_bool();
}

bool Brain::hasFallenRobotInVisualKickZone(GameObject *nearestRobot)
{
    constexpr double kPi = 3.14159265358979323846;
    const double maxDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.fallen_robot_distance").as_double());
    const double halfAngle = std::clamp(
        get_parameter("obstacle_avoidance.fallen_robot_angle").as_double(),
        0.0,
        kPi);
    const uint32_t robotConfirmFrames = static_cast<uint32_t>(
        std::clamp<int64_t>(
            get_parameter("obstacle_avoidance.robot_confirm_frames").as_int(),
            int64_t{1}, int64_t{100}));
    const double selfRadius = selfRobotCollisionRadius();
    const double minimumRobotDistance = robotObstacleMinimumDistance();
    double nearestDistance = std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto &robot : data->getRobots()) {
        const Point center = robotObstacleCenterToRobot(robot);
        if (!robot.fallen ||
            (robot.obstacleSeenCount < robotConfirmFrames &&
             !robotObstacleEmergency(robot, 0.0)) ||
            !std::isfinite(center.x) || !std::isfinite(center.y) ||
            std::hypot(center.x, center.y) <= minimumRobotDistance ||
            !fallen_robot_avoidance_policy::intersectsForwardSector(
                center.x,
                center.y,
                maxDistance,
                halfAngle,
                robotObstacleRadius(robot) + selfRadius)) {
            continue;
        }

        const double distance = std::max(
            0.0,
            std::hypot(center.x, center.y) -
                robotObstacleRadius(robot) - selfRadius);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            if (nearestRobot != nullptr) *nearestRobot = robot;
            found = true;
        }
    }
    return found;
}

double Brain::distToObstacle(
    double angle,
    const GameObject *ignoredFallenRobot)
{
    return distToObstacleImpl(angle, ignoredFallenRobot, true);
}

double Brain::distToDepthObstacle(double angle)
{
    return distToObstacleImpl(angle, nullptr, false);
}

double Brain::distToObstacleImpl(
    double angle,
    const GameObject *ignoredFallenRobot,
    bool includeRecognizedRobots)
{
    if (!std::isfinite(angle)) return 0.0;
    angle = toPInPI(angle);
    auto obs = data->getObstacles();
    const auto robots = data->getRobots();
    double minDist = 1e9;
    // double obstacleThreshold = static_cast<double>(config->obstacleThreshold);
    const double obstacleThreshold = std::max(
        0.0,
        static_cast<double>(get_parameter(
            "obstacle_avoidance.occupancy_threshold").as_int()));
    const double obstacleMemoryMsecs = std::max(
        0.0,
        get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double());
    const bool ballObstacleEnabled = get_parameter(
        "obstacle_avoidance.enable_ball_obstacle").as_bool();
    const double ballObstacleRadius = std::max(
        0.0,
        get_parameter("obstacle_avoidance.ball_obstacle_radius").as_double());
    const double ballObstacleMaxAgeMsecs = std::max(
        0.0,
        get_parameter(
            "obstacle_avoidance.ball_obstacle_max_age_msecs").as_double());

    const double collisionThreshold = std::isfinite(config->collisionThreshold)
        ? std::max(0.0, config->collisionThreshold)
        : 0.0;
    const double robotDepthFusionDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_depth_fusion_distance").as_double());
    const double robotDepthFusionMaxAgeMsecs = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_depth_fusion_max_age_msecs")
            .as_double());
    const uint32_t depthConfirmFrames = static_cast<uint32_t>(std::clamp<int64_t>(
        get_parameter("obstacle_avoidance.depth_confirm_frames").as_int(),
        int64_t{1}, int64_t{100}));
    const uint32_t robotConfirmFrames = static_cast<uint32_t>(std::clamp<int64_t>(
        get_parameter("obstacle_avoidance.robot_confirm_frames").as_int(),
        int64_t{1}, int64_t{100}));
    const double uprightRobotDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.robot_obstacle_distance").as_double());
    const double fallenRobotDistance = std::max(
        0.0,
        get_parameter("obstacle_avoidance.fallen_robot_distance").as_double());
    const double selfRadius = selfRobotCollisionRadius();
    const double minimumRobotDistance = robotObstacleMinimumDistance();
    const double effectiveCollisionThreshold = collisionThreshold + selfRadius;

    const auto robotCoversDepthObstacle = [&](const GameObject &obstacle) {
        for (const auto &robot : robots) {
            if ((robot.obstacleSeenCount < robotConfirmFrames &&
                 !robotObstacleEmergency(robot)) ||
                msecsSince(robot.timePoint) > robotDepthFusionMaxAgeMsecs) {
                continue;
            }
            const Point center = robotObstacleCenterToRobot(robot);
            const double robotDistance = std::hypot(center.x, center.y);
            const double robotRangeLimit = robot.fallen
                ? fallenRobotDistance
                : uprightRobotDistance;
            if (!std::isfinite(robotDistance) ||
                robotDistance <= minimumRobotDistance ||
                robotDistance - robotObstacleRadius(robot) > robotRangeLimit) {
                continue;
            }
            const double footprintWidth = std::hypot(
                robot.obstacleLeftToRobot.x - robot.obstacleRightToRobot.x,
                robot.obstacleLeftToRobot.y - robot.obstacleRightToRobot.y);
            if (robot_obstacle_policy::coversDepthPoint(
                    {center.x, center.y},
                    {obstacle.posToRobot.x, obstacle.posToRobot.y},
                    std::atan2(center.y, center.x),
                    robotObstacleRadius(robot),
                    robotDepthFusionDistance,
                    robot.obstacleFootprintValid,
                    footprintWidth)) {
                return true;
            }
        }
        return false;
    };

    for (int i = 0; i < obs.size(); i++) {
        const auto &storedObstacle = obs[i];
        if (storedObstacle.label == "Ball") {
            if (!ballObstacleEnabled ||
                msecsSince(storedObstacle.timePoint) >
                    ballObstacleMaxAgeMsecs) {
                continue;
            }
            const auto intersection =
                fallen_robot_avoidance_policy::intersectsPath(
                    storedObstacle.posToRobot.x,
                    storedObstacle.posToRobot.y,
                    angle,
                    1e6,
                    0.0,
                    ballObstacleRadius + selfRadius);
            if (intersection.intersects) {
                minDist = std::min(minDist, intersection.entryDistance);
            }
            continue;
        }
        if (storedObstacle.label != "Obstacle" ||
            msecsSince(storedObstacle.timePoint) > obstacleMemoryMsecs ||
            storedObstacle.confidence < obstacleThreshold ||
            storedObstacle.obstacleSeenCount < depthConfirmFrames) {
            continue;
        }

        auto o = storedObstacle;
        if (robotCoversDepthObstacle(o)) continue;
        Line line = {
            0, 0,
            cos(angle) * 100, sin(angle) * 100
        };
        double perpDist = fabs(pointPerpDistToLine(Point2D{o.posToRobot.x, o.posToRobot.y}, line));
        if (perpDist < effectiveCollisionThreshold) {
            double dist = innerProduct(vector<double>{o.posToRobot.x, o.posToRobot.y}, vector<double>{cos(angle), sin(angle)});
            const double collisionDistance = std::max(0.0, dist - selfRadius);
            if (dist > 0 && collisionDistance < minDist) {
                // Depth points represent occupied space, so return the
                // distance to the point minus this robot's footprint radius.
                minDist = collisionDistance;
            }
        }
    }

    if (!includeRecognizedRobots) return minDist;

    for (const auto &robot : robots) {
        if (robot.obstacleSeenCount < robotConfirmFrames &&
            !robotObstacleEmergency(robot, angle)) continue;
        if (ignoredFallenRobot != nullptr && robot.fallen &&
            std::hypot(
                robot.posToField.x - ignoredFallenRobot->posToField.x,
                robot.posToField.y - ignoredFallenRobot->posToField.y) < 0.2) {
            continue;
        }

        const Point center = robotObstacleCenterToRobot(robot);
        if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
            std::hypot(center.x, center.y) <= minimumRobotDistance) {
            continue;
        }
        const auto intersection = fallen_robot_avoidance_policy::intersectsPath(
            center.x,
            center.y,
            angle,
            robot.fallen ? fallenRobotDistance : uprightRobotDistance,
            0.0,
            robotObstacleRadius(robot) + selfRadius);
        if (intersection.intersects) {
            minDist = std::min(minDist, intersection.entryDistance);
        }
    }
    return minDist;
}

vector<double> Brain::findSafeDirections(double startAngle, double safeDist, double step) {
    if (!std::isfinite(startAngle) || !std::isfinite(safeDist) ||
        !std::isfinite(step) || step <= 1e-4) {
        return vector<double>{0.0, 0.0, 0.0, 0.0};
    }
    step = std::min(step, M_PI / 2.0);
    double safeAngleLeft = startAngle;
    double safeAngleRight = startAngle;
    double leftFound = 0;
    double rightFound = 0;
    for (double angle = startAngle; angle < startAngle + M_PI; angle += step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleLeft = angle;
            leftFound = 1;
            break;
        }
    }
    for (double angle = startAngle; angle > startAngle - M_PI; angle -= step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleRight = angle;
            rightFound = 1;
            break;
        }
    }

    return vector<double>{leftFound, toPInPI(safeAngleLeft), rightFound, toPInPI(safeAngleRight)};
}

double Brain::calcAvoidDir(double startAngle, double safeDist) {
    if (!std::isfinite(startAngle) || !std::isfinite(safeDist)) return 0.0;
    if (distToObstacle(startAngle) > safeDist) {
        return toPInPI(startAngle);
    }
    auto res = findSafeDirections(startAngle, safeDist);
    bool leftFound = res[0] > 0.5;
    bool rightFound = res[2] > 0.5;
    double angleLeft = res[1];
    double angleRight = res[3];
    double determinedAngle = 0.0;
    if (leftFound && rightFound) {
        const double leftOffset = std::fabs(toPInPI(angleLeft - startAngle));
        const double rightOffset = std::fabs(toPInPI(angleRight - startAngle));
        const double switchMargin = std::max(
            0.0,
            get_parameter("obstacle_avoidance.avoidance_direction_switch_margin")
                .as_double());
        const double lockMsecs = std::max(
            0.0,
            get_parameter("obstacle_avoidance.avoidance_direction_lock_msecs")
                .as_double());
        const double lastDirectionAge =
            msecsSince(lastAvoidanceDirectionAt_);
        const double lockedClearance =
            distToObstacle(lastAvoidanceDirection_);
        const bool lockActive = std::fabs(lastAvoidanceDirection_) > 1e-4 &&
            lastDirectionAge < lockMsecs && lockedClearance > safeDist;
        if (lockActive) {
            determinedAngle = lastAvoidanceDirection_;
        } else {
            determinedAngle = leftOffset <= rightOffset + switchMargin
                ? angleLeft
                : angleRight;
        }
    } else if (leftFound) {
        determinedAngle = angleLeft;
    } else if (rightFound) {
        determinedAngle = angleRight;
    } else {
        // A direction can be useful before it reaches safeDist. This is common
        // when the robot is close to several obstacles: requiring full safety
        // in one scan caused a permanent zero-speed state. Choose the ray with
        // the greatest immediate clearance and let the next control cycle
        // replan as clearance grows.
        double bestAngle = toPInPI(startAngle);
        double bestClearance = distToObstacle(bestAngle);
        const double scanStep = deg2rad(5);
        for (double offset = scanStep; offset < M_PI; offset += scanStep) {
            const double leftAngle = toPInPI(startAngle + offset);
            const double rightAngle = toPInPI(startAngle - offset);
            const double leftClearance = distToObstacle(leftAngle);
            const double rightClearance = distToObstacle(rightAngle);
            if (leftClearance > bestClearance + 1e-6) {
                bestClearance = leftClearance;
                bestAngle = leftAngle;
            }
            if (rightClearance > bestClearance + 1e-6) {
                bestClearance = rightClearance;
                bestAngle = rightAngle;
            }
        }

        const double lockMsecs = std::max(
            0.0,
            get_parameter("obstacle_avoidance.avoidance_direction_lock_msecs")
                .as_double());
        const double lockedClearance =
            distToObstacle(lastAvoidanceDirection_);
        const bool lockActive = std::fabs(lastAvoidanceDirection_) > 1e-4 &&
            msecsSince(lastAvoidanceDirectionAt_) < lockMsecs &&
            lockedClearance >= std::max(0.20, bestClearance - 0.05);
        determinedAngle = lockActive
            ? lastAvoidanceDirection_
            : bestAngle;
    }
    determinedAngle = toPInPI(determinedAngle);
    if (std::fabs(determinedAngle) > 1e-4) {
        lastAvoidanceDirection_ = determinedAngle;
        lastAvoidanceDirectionAt_ = get_clock()->now();
    }
    return determinedAngle;
}

void Brain::logObstacleAvoidance(ObstacleAvoidanceLogRecord record)
{
    if (!obstacleAvoidanceLog_ ||
        !obstacleAvoidanceLog_->shouldLog(record.source, record.reason)) {
        return;
    }

    const auto now = get_clock()->now();
    record.rosTimeNs = now.nanoseconds();
    record.robotX = data->robotPoseToField.x;
    record.robotY = data->robotPoseToField.y;
    record.robotTheta = data->robotPoseToField.theta;
    record.selfRadius = selfRobotCollisionRadius();
    record.depthSampleStep = get_parameter(
        "obstacle_avoidance.depth_sample_step").as_int();
    record.cameraWidth = static_cast<int>(std::lround(
        get_parameter("vision.cam_pixel_width").as_double()));
    record.cameraHeight = static_cast<int>(std::lround(
        get_parameter("vision.cam_pixel_height").as_double()));
    record.depthSamplingInvalid = depthSamplingInvalid_.load();
    record.depthGridOccupiedCells = depthGridOccupiedCells_.load();
    record.depthGridMaxOccupancy = depthGridMaxOccupancy_.load();
    record.depthGridConfirmedCells = depthGridConfirmedCells_.load();
    record.rawRobotDetectionCount = rawRobotDetectionCount_.load();
    record.mergedRobotDetectionCount = mergedRobotDetectionCount_.load();
    record.occupancyThreshold = static_cast<int>(get_parameter(
        "obstacle_avoidance.occupancy_threshold").as_int());
    record.obstacleMemoryMsecs = get_parameter(
        "obstacle_avoidance.obstacle_memory_msecs").as_double();
    record.robotMergeDistance = get_parameter(
        "obstacle_avoidance.robot_merge_distance").as_double();
    record.robotMinimumDistance = robotObstacleMinimumDistance();
    record.avoidanceMaximumSpeed = get_parameter(
        "obstacle_avoidance.robot_avoidance_speed").as_double();
    record.avoidanceMinimumVy = get_parameter(
        "obstacle_avoidance.robot_avoidance_min_vy").as_double();
    record.avoidanceReverseSpeed = get_parameter(
        "obstacle_avoidance.robot_avoidance_reverse_speed").as_double();
    record.buildVersion = BRAIN_GIT_DESCRIBE;

    const auto depthObstacles = data->getObstacles();
    const auto robots = data->getRobots();
    record.depthObstacleCount = static_cast<std::size_t>(std::count_if(
        depthObstacles.begin(), depthObstacles.end(),
        [](const auto &obstacle) { return obstacle.label == "Obstacle"; }));
    record.ballObstacleCount = static_cast<std::size_t>(std::count_if(
        depthObstacles.begin(), depthObstacles.end(),
        [](const auto &obstacle) { return obstacle.label == "Ball"; }));
    record.robotObstacleCount = robots.size();
    record.obstacles.reserve(std::min<std::size_t>(
        20, depthObstacles.size() + robots.size()));

    std::vector<ObstacleAvoidanceLogObstacle> snapshots;
    snapshots.reserve(depthObstacles.size() + robots.size());
    const double depthRadius = std::max(
        0.0, config->collisionThreshold) + record.selfRadius;
    const double ballRadius = std::max(
        0.0,
        get_parameter("obstacle_avoidance.ball_obstacle_radius").as_double()) +
        record.selfRadius;
    for (const auto &obstacle : depthObstacles) {
        ObstacleAvoidanceLogObstacle snapshot;
        snapshot.type = obstacle.label == "Ball" ? "ball" : "depth";
        snapshot.x = obstacle.posToRobot.x;
        snapshot.y = obstacle.posToRobot.y;
        snapshot.radius = obstacle.label == "Ball" ? ballRadius : depthRadius;
        snapshot.confidence = obstacle.confidence;
        snapshot.ageMs = msecsSince(obstacle.timePoint);
        snapshot.seenCount = obstacle.obstacleSeenCount;
        snapshot.missedCount = obstacle.obstacleMissedCount;
        snapshots.push_back(std::move(snapshot));
    }
    for (const auto &robot : robots) {
        const Point center = robotObstacleCenterToRobot(robot);
        ObstacleAvoidanceLogObstacle snapshot;
        snapshot.type = robot.label == "Person" ? "person" : "robot";
        snapshot.x = center.x;
        snapshot.y = center.y;
        snapshot.radius = robotObstacleRadius(robot) + record.selfRadius;
        snapshot.confidence = robot.confidence;
        snapshot.ageMs = msecsSince(robot.timePoint);
        snapshot.seenCount = robot.obstacleSeenCount;
        snapshot.missedCount = robot.obstacleMissedCount;
        snapshot.fallen = robot.fallen;
        snapshots.push_back(std::move(snapshot));
    }
    std::sort(
        snapshots.begin(), snapshots.end(),
        [](const auto &left, const auto &right) {
            return std::hypot(left.x, left.y) < std::hypot(right.x, right.y);
        });
    if (snapshots.size() > 20) snapshots.resize(20);
    record.obstacles = std::move(snapshots);
    obstacleAvoidanceLog_->log(record);
}

void Brain::updateLogFile() {
    if (config->rerunLogEnableFile && msecsSince(data->timeLastLogSave) > config->rerunLogMaxFileMins * 60000)
        log->updateLogFilePath();
}

// ------------------------------------------------------ Diagnostic logging ------------------------------------------------------
void Brain::logObstacleDistance() {
    if (!log->shouldLog("obstacle_distance_visual", config->rerunLogVisualHz))
        return;

    log->setTimeNow();

    // log obstacle distance for test
    vector<rerun::LineStrip2D> lines = {};
    for (int i = 0; i < 180; i++) {
        double angle = i * M_PI / 90;
        double dist = min(5.0, distToObstacle(angle));
        double angle_f = toPInPI(data->robotPoseToField.theta + angle);
        lines.push_back(
            rerun::LineStrip2D({
                {data->robotPoseToField.x, -data->robotPoseToField.y},
                {data->robotPoseToField.x + cos(-angle_f) * dist, -data->robotPoseToField.y + sin(-angle_f) * dist}
            })
        );
    }
    log->log(
        "field/obstacle_distance",
        rerun::LineStrips2D(lines)
            .with_colors(0x666666FF)
            .with_radii(0.01)
            .with_draw_order(-10)
    );
}

void Brain::logLags() {
    const bool logLagVisual =
        log->shouldLog("lags_visual", config->rerunLogVisualHz);
    const bool logLagTimeseries =
        log->shouldLog("lags_timeseries", config->rerunLogTimeseriesHz);
    if (!logLagVisual && !logLagTimeseries)
        return;

    log->setTimeNow();

    double detLag = msecsSince(data->timeLastDet);
    double lineLag = msecsSince(data->timeLastLineDet);
    double gcLag = msecsSince(data->timeLastGamecontrolMsg);

    if (logLagVisual) {
        const double maxLagLength = config->camPixX;
        auto color = detLag > 500 ? 0xFF0000FF
            : (detLag > 100 ? 0xFFFF00FF : 0x00FF00FF);
        log->log(
            "image/detection_lag",
            rerun::LineStrips2D(rerun::LineStrip2D({{10., -150.}, {10. + min(detLag, maxLagLength), -150.}}))
                .with_colors(color)
                .with_radii(2.0)
                .with_draw_order(10)
                .with_labels({format("Det Lag %.0fms", detLag)})
        );

        color = lineLag > 500 ? 0xFF0000FF
            : (lineLag > 100 ? 0xFFFF00FF : 0x00FF00FF);
        log->log(
            "image/fieldline_detection_lag",
            rerun::LineStrips2D(rerun::LineStrip2D({{10., -100.}, {10. + min(lineLag, maxLagLength), -100.}}))
                .with_colors(color)
                .with_radii(2.0)
                .with_draw_order(10)
                .with_labels({format("Line Det Lag %.0fms", lineLag)})
        );

        color = gcLag > 5000 ? 0xFF0000FF
            : (gcLag > 1000 ? 0xFFFF00FF : 0x00FF00FF);
        log->log(
            "image/gamecontrol_lag",
            rerun::LineStrips2D(rerun::LineStrip2D({{10., -50.}, {10. + min(gcLag, maxLagLength), -50.}}))
                .with_colors(color)
                .with_radii(2.0)
                .with_draw_order(10)
                .with_labels({format("GC Lag %.0fms", gcLag)})
        );
    }

    if (logLagTimeseries) {
        log->log("performance/detection_lag_timeseries", rerun::Scalar(detLag));
        log->log("performance/fieldline_detection_lag_timeseries", rerun::Scalar(lineLag));
        log->log("performance/gamecontrol_lag_timeseries", rerun::Scalar(gcLag));
    }
}

void Brain::statusReport() {
    if (!config->soundEnable || config->soundPack != "espeak") return;

    log->setTimeNow();
    static int reportInterval = 100;
    static string lastReport = "";
    string report;
    bool camOK = msecsSince(data->timeLastDet) < 1000;
    bool gcOK = msecsSince(data->timeLastGamecontrolMsg) < 1000;

    if (camOK && gcOK) {
        report = "Team" + to_string(config->teamId) + " Player " + to_string(config->playerId) + " " + tree->getEntry<string>("player_role") + " " + " OK";
    } else {
        report = "";
        if (!camOK) report += "camera lost";
        if (!gcOK) report += "gamecontrol lost";
    }
    if (lastReport != report) {
        speak(report);
        lastReport = report;
    }
}

void Brain::logStatusToConsole() {
    static int cnt = 0;
    const int LOG_INTERVAL = 30;
    cnt++;
    if (cnt % LOG_INTERVAL == 0) {
        string msg = "";
        string gameState = tree->getEntry<string>("gc_game_state");
        gameState = gameState == "" ? "-----" : gameState;
        string gameSubType = tree->getEntry<string>("gc_game_sub_state_type");
        gameSubType = gameSubType == "" ? "-----" : gameSubType;
        string gameSubState = tree->getEntry<string>("gc_game_sub_state");
        gameSubState = gameSubState == "" ? "-----" : gameSubState;

        msg += format(
            "ROBOT:\n\tTeamID: %d\tPlayerID: %d\tNumberOfPlayers: %d\tRole: %s\tStartRole: %s\n\n",
            config->teamId,
            config->playerId,
            config->numOfPlayers,
            tree->getEntry<string>("player_role").c_str(),
            config->playerRole.c_str()
        );
        msg += format(
            "FIELD_POSE:\n\tx: %.3f m\ty: %.3f m\ttheta: %.3f rad (%.1f deg)\tCalibrated: %s\n\n",
            data->robotPoseToField.x,
            data->robotPoseToField.y,
            data->robotPoseToField.theta,
            data->robotPoseToField.theta * 180.0 / M_PI,
            tree->getEntry<bool>("odom_calibrated") ? "YES" : "NO"
        );
        msg += format(
            "GAME:\n\tState: %s\tKickOffSide: %s\tisKickingOff: %s(%s)\n\tSubType: %s\tSubState: %s\tSubKickOffSide: %s\tisKickingOff: %s(%s)\n\tScore: %s\tJustScored: %s\n\tLiveCount: %d\tOppoLiveCount: %d\tPrimary: %s\n\n",
            gameState.c_str(),
            tree->getEntry<bool>("gc_is_kickoff_side") ? "YES" : "NO",
            data->isKickingOff ? "YES" : "NO",
            msecsSince(data->kickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->kickoffStartTime)/1000).c_str(),
            gameSubType.c_str(),
            gameSubState.c_str(),
            tree->getEntry<bool>("gc_is_sub_state_kickoff_side") ? "YES" : "NO",
            data->isFreekickKickingOff ? "YES" : "NO",
            msecsSince(data->freekickKickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->freekickKickoffStartTime)/1000).c_str(),
            format("%d:%d", data->score, data->oppoScore).c_str(),
            tree->getEntry<bool>("we_just_scored") ? "YES" : "NO",
            data->liveCount,
            data->oppoLiveCount,
            isPrimaryStriker() ? "YES" : "NO"
        );

        msg += getComLogString();

        msg += format(
            "DEBUG:\n\tcom: %s\tlogFile: %s\tlogTCP: %s\n\tvxFactor: %.2f\tyawOffset: %.2f\n\tControlState: %d\n\tTickTime: %.0fms",
            config->enableCom ? "YES" : "NO",
            config->rerunLogEnableFile ? "YES" : "NO",
            config->rerunLogEnableTCP ? "YES" : "NO",
            config->vxFactor,
            config->yawOffset,
            tree->getEntry<int>("control_state"),
            msecsSince(data->lastTick)
        );
        prtDebug(msg);
    }
    data->lastTick = get_clock()->now();
}

string Brain::getComLogString() {
    stringstream ss;
    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    {
        std::lock_guard<std::mutex> teamStatusLock(data->teamStatusMutex);
        std::copy(
            std::begin(data->tmStatus),
            std::end(data->tmStatus),
            teamStatuses.begin());
    }
    const double communicationTimeoutMs = std::max(
        100.0,
        get_parameter(
            "strategy.cooperation.tactical_packet_timeout_ms").as_double());
    const int selfIdx = config->playerId - 1;
    for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
        if (i == selfIdx) continue;
        if (data->penalty[i] != PENALTY_NONE ||
            teamStatuses[i].ageMs() > communicationTimeoutMs) {
            teamStatuses[i].isAlive = false;
            teamStatuses[i].isLead = false;
        }
    }
    int ballOwnerId = 0;
    uint32_t leaderTerm = 0;
    int formationOwnerId = 0;
    uint32_t formationRevision = 0;
    AssistSlot myAssistSlot;
    AssistPhase myAssistPhase;
    int assignedGoalkeeperId = 0;
    {
        std::lock_guard<std::mutex> cooperationLock(data->cooperationMutex);
        ballOwnerId = data->tmBallOwnerId;
        leaderTerm = data->tmLeaderTerm;
        formationOwnerId = data->tmFormationOwnerId;
        formationRevision = data->tmFormationRevision;
        myAssistSlot = data->tmMyAssistSlot;
        myAssistPhase = data->tmAssistPhase;
        assignedGoalkeeperId = data->tmAssignedGoalkeeperId;
    }
    int onFieldCnt = 0;
    int aliveCnt = 0;
    vector<int> onFieldIdxs = {};
    for (int i = 0; i < MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        if (data->penalty[i] == PENALTY_NONE) {
            onFieldCnt += 1;
            onFieldIdxs.push_back(i);
        }

        if (teamStatuses[i].isAlive) aliveCnt += 1;
    }
    ss << CYAN_CODE << "COM: " << "\n";
    ss << "Teammates: OnField: " << onFieldCnt << "[";
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        ss << " P" << idx + 1 << " ";
    }
    ss << "]";
    ss << "  Alive: " << aliveCnt << "\n";

    // Self info
    ss << "Self\tCost: " << format("%.1f", data->tmMyCost) << "\tLead: ";
    if (data->tmImLead)
        ss << GREEN_CODE << "YES" << CYAN_CODE;
    else
        ss << RED_CODE << "NO" << CYAN_CODE;
    ss << format(
        "\n\tOwner: %d/%u\tFormation: %d/%u\tSlot: %s/%s\tGoalie: P%d\n",
        ballOwnerId,
        static_cast<unsigned>(leaderTerm),
        formationOwnerId,
        static_cast<unsigned>(formationRevision),
        assistSlotName(myAssistSlot),
        assistPhaseName(myAssistPhase),
        assignedGoalkeeperId);

    // Teammates info
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        const auto &status = teamStatuses[idx];
        ss << "P" << idx + 1 << "[";
        if (status.isAlive)
            ss << GREEN_CODE << "★" << CYAN_CODE;
        else
            ss << RED_CODE << "☆" << CYAN_CODE;
        ss << "]\tCost: " << format("%.1f", status.cost);
        ss << "\tLead: ";
        if (status.isLead)
            ss << GREEN_CODE << "YES" << CYAN_CODE;
        else
            ss << RED_CODE << "NO" << CYAN_CODE;
        ss << format(
            "\tOwner: %d/%u\tFormation: %d/%u\tSlot: %s/%s",
            status.ballOwnerId,
            static_cast<unsigned>(status.leaderTerm),
            status.formationOwnerId,
            static_cast<unsigned>(status.formationRevision),
            assistSlotName(status.assistSlot),
            assistPhaseName(status.assistPhase));
        ss << "\tLag: " << format("%.0f", status.ageMs()) << "ms" << "\n";
    }
    ss << "\n";

    return ss.str();
}

bool Brain::isFreekickStartPlacing() {
    return tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK" &&
        tree->getEntry<string>("gc_game_state") == "PLAY" &&
        tree->getEntry<string>("gc_game_sub_state") == "STOP";
}

void Brain::playSoundForFun() {
    string soundPack = config->soundPack;
    if (config->soundEnable && soundPack != "espeak") {
        static string gcGameState_last;
        string gcGameState = tree->getEntry<string>("gc_game_state");

        static bool gameStarted = false;
        if (gcGameState == "PLAY") gameStarted = true;
        if (gcGameState == "READY")
        {
            if (!gameStarted) playSound(soundPack + "-ready", 2000);
            else if (tree->getEntry<bool>("we_just_scored")) playSound(soundPack + "-celebrate", 2000);
            else playSound(soundPack + "-regret", 2000);
        }

        if (gcGameState == "PLAY") {
            auto decision = tree->getEntry<string>("decision");
            if (decision == "chase") playSound(soundPack + "-chase", 5000);
            else if (decision == "adjust") playSound(soundPack + "-adjust", 2000);
            else if (decision == "kick") playSound(soundPack + "-kick", 2000);
        }
    }
}

/**
 * Evaluate the utility of kicking in direction dir.
 */
double Brain::kickValue(double dir)
{
    // Prefer directions toward the opponent goal.
    auto fd = config->fieldDimensions;
    double goalDir = atan2(0.0 - data->robotPoseToField.y, fd.length/2.0 - data->robotPoseToField.x);
    double diff = fabs(toPInPI(dir - goalDir));

    // Smaller angular error produces a higher value in [0, 1].
    return max(0.0, 1.0 - diff / M_PI);
}

/**
 * Calculate threat level: 0 safe, 1 threatened, 2 dangerous.
 */
double Brain::threatLevel()
{
    // Nearby opponents increase the threat level.
    double minOpponentDist = 100.0;
    auto robots = data->getRobots();
    for(const auto& r : robots) {
        if(r.label == "Opponent") {
            double dist = r.range;
            if(dist < minOpponentDist) minOpponentDist = dist;
        }
    }

    if (minOpponentDist < 1.0) return 2.0;
    if (minOpponentDist < 2.5) return 1.0;
    return 0.0;
}

/**
 * Return whether the current geometry permits a directional kick.
 */
bool Brain::isAngleGoodForDirectionalKick(double goalPostMargin)
{
    // Reuse the common kick-angle validation.
    return isAngleGood(goalPostMargin, "kick");
}

/**
 * Check whether a forward sector is clear of obstacles.
 */
bool Brain::isFrontRangeClear(double startAngle, double endAngle, double safeDist, double step)
{
    // Sample the nearest obstacle distance across the angular range.
    for (double ang = startAngle; ang <= endAngle; ang += step) {
        if (distToObstacle(ang) < safeDist) {
            return false; // Obstacle found.
        }
    }
    return true; // Sector is clear.
}

/**
 * Publish vision calibration parameters.
 */
void Brain::pubCalParamMsg(double pitch, double yaw, double z)
{
    if (pubCalParam) {
        vision_interface::msg::CalParam msg;
        msg.pitch_compensation = pitch;
        msg.yaw_compensation = yaw;
        msg.z_compensation = z;

        pubCalParam->publish(msg);
    }
}
