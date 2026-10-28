#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace ball_state_estimator {

struct Vec2
{
    double x = 0.0;
    double y = 0.0;

    double norm() const;
};

// 与 robocup 主线一致的滚球物理模型。位置单位 m，速度 m/s，摩擦 m/s^2。
struct BallPhysics
{
    static constexpr double MAX_BALL_SPEED = 5.0;

    static void clampVelocity(double &vx, double &vy);
    static void propagateWithSpeedFriction(
        double &px, double &py, double &vx, double &vy,
        double dt, double baseFriction, double speedFactor);
    static Vec2 getEndPositionWithSpeedFriction(
        double px, double py, double vx, double vy,
        double baseFriction, double speedFactor);
};

// 静止球/滚动球多假设卡尔曼估计器。接口与 robocup 中 BallStateEstimator 对齐。
class BallStateEstimator
{
public:
    BallStateEstimator();
    ~BallStateEstimator();

    BallStateEstimator(const BallStateEstimator &) = delete;
    BallStateEstimator &operator=(const BallStateEstimator &) = delete;

    void update(double posX, double posY, const rclcpp::Time &timestamp,
                double measurementStdDev = 0.05);
    void reset();

    Vec2 getEstimatedPosition() const;
    Vec2 getEstimatedVelocity() const;
    Vec2 getEndPosition() const;
    bool isRolling() const;
    bool hasValidEstimate() const;

    void setFriction(double friction);
    void setMinSpeedThreshold(double speed);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ball_state_estimator
