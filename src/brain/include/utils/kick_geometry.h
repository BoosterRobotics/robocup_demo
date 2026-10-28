#pragma once

#include <algorithm>
#include <cmath>

namespace kick_geometry
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double normalizeAngle(double angle)
{
    return std::remainder(angle, kTwoPi);
}

inline double angleDistance(double lhs, double rhs)
{
    return std::abs(normalizeAngle(lhs - rhs));
}

inline double minorArcWidth(double edgeA, double edgeB)
{
    return angleDistance(edgeA, edgeB);
}

inline bool isAngleWithinMinorArc(double angle, double edgeA, double edgeB,
                                  double tolerance = 1e-9)
{
    const double signedSpan = normalizeAngle(edgeA - edgeB);
    const double halfWidth = std::abs(signedSpan) / 2.0;
    const double midpoint = normalizeAngle(edgeB + signedSpan / 2.0);
    return angleDistance(angle, midpoint) <= halfWidth + tolerance;
}

inline double directionToTarget(double fromX, double fromY,
                                double targetX, double targetY,
                                double fallback = 0.0)
{
    const double dx = targetX - fromX;
    const double dy = targetY - fromY;
    if (std::hypot(dx, dy) <= 1e-9) {
        return normalizeAngle(fallback);
    }
    return std::atan2(dy, dx);
}

inline double directionToOpponentGoal(double ballX, double ballY,
                                      double opponentGoalLineX,
                                      double aimDepth = 1.0)
{
    const double depth = std::max(0.0, aimDepth);
    const double distanceToGoalLine = opponentGoalLineX - ballX;
    const double targetX = opponentGoalLineX + depth -
        std::min(distanceToGoalLine, depth);
    return directionToTarget(ballX, ballY, targetX, 0.0, 0.0);
}

inline double directionForIndirectSetPlay(double ballX, double ballY,
                                          double opponentGoalLineX,
                                          double fieldWidth,
                                          double sidelineRatio = 0.8)
{
    const double sidelineThreshold = std::abs(fieldWidth) / 2.0 *
        std::clamp(sidelineRatio, 0.0, 1.0);
    if (ballY > sidelineThreshold) return -kPi / 2.0;
    if (ballY < -sidelineThreshold) return kPi / 2.0;
    return directionToOpponentGoal(ballX, ballY, opponentGoalLineX);
}
} // namespace kick_geometry
