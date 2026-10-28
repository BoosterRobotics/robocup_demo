#include "ball_state_estimator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace ball_state_estimator {

double Vec2::norm() const
{
    return std::hypot(x, y);
}

namespace {

struct Matrix2
{
    double a00 = 1.0;
    double a01 = 0.0;
    double a10 = 0.0;
    double a11 = 1.0;

    static Matrix2 zero() { return {0.0, 0.0, 0.0, 0.0}; }

    Matrix2 operator+(const Matrix2 &o) const
    {
        return {a00 + o.a00, a01 + o.a01, a10 + o.a10, a11 + o.a11};
    }
    Matrix2 operator-(const Matrix2 &o) const
    {
        return {a00 - o.a00, a01 - o.a01, a10 - o.a10, a11 - o.a11};
    }
    Matrix2 operator*(const Matrix2 &o) const
    {
        return {
            a00 * o.a00 + a01 * o.a10,
            a00 * o.a01 + a01 * o.a11,
            a10 * o.a00 + a11 * o.a10,
            a10 * o.a01 + a11 * o.a11};
    }
    Matrix2 transpose() const { return {a00, a10, a01, a11}; }
    Matrix2 inverse() const
    {
        double det = a00 * a11 - a01 * a10;
        if (std::abs(det) < 1e-12) det = std::copysign(1e-12, det == 0.0 ? 1.0 : det);
        return {a11 / det, -a01 / det, -a10 / det, a00 / det};
    }
    void addDiagonal(double value)
    {
        a00 += value;
        a11 += value;
    }
};

Vec2 operator+(const Vec2 &a, const Vec2 &b) { return {a.x + b.x, a.y + b.y}; }
Vec2 operator-(const Vec2 &a, const Vec2 &b) { return {a.x - b.x, a.y - b.y}; }
Vec2 operator*(const Vec2 &v, double scale) { return {v.x * scale, v.y * scale}; }
Vec2 operator*(const Matrix2 &m, const Vec2 &v)
{
    return {m.a00 * v.x + m.a01 * v.y, m.a10 * v.x + m.a11 * v.y};
}

double weighting(double nll, const Matrix2 &positionCovariance)
{
    const double det = std::max(1e-12,
        positionCovariance.a00 * positionCovariance.a11 -
        positionCovariance.a01 * positionCovariance.a10);
    return nll + 0.5 * std::log(det);
}

double measurementNll(const Vec2 &innovation, const Matrix2 &innovationCovariance)
{
    const Matrix2 inv = innovationCovariance.inverse();
    return 0.5 * (innovation.x * (inv.a00 * innovation.x + inv.a01 * innovation.y) +
                  innovation.y * (inv.a10 * innovation.x + inv.a11 * innovation.y));
}

struct StationaryHypothesis
{
    Vec2 position{};
    Matrix2 covariance{};
    int measurements = 0;
    double nll = 0.0;
    double score = std::numeric_limits<double>::infinity();

    void init(const Vec2 &measurement, const Matrix2 &measurementCovariance, double initialNll)
    {
        position = measurement;
        covariance = measurementCovariance;
        measurements = 1;
        nll = initialNll;
        score = weighting(nll, covariance);
    }

    void predict(double processNoise) { covariance.addDiagonal(processNoise); }

    void correct(const Vec2 &measurement, const Matrix2 &measurementCovariance)
    {
        const Vec2 innovation = measurement - position;
        const Matrix2 innovationCovariance = covariance + measurementCovariance;
        nll += measurementNll(innovation, innovationCovariance);
        const Matrix2 gain = covariance * innovationCovariance.inverse();
        position = position + gain * innovation;
        covariance = covariance - gain * covariance;
        ++measurements;
        score = weighting(nll, covariance);
    }
};

struct RollingHypothesis
{
    Vec2 position{};
    Vec2 velocity{};
    Matrix2 pxx{};
    Matrix2 pxv = Matrix2::zero();
    Matrix2 pvx = Matrix2::zero();
    Matrix2 pvv{};
    int measurements = 0;
    double nll = 0.0;
    double score = std::numeric_limits<double>::infinity();

    void init(const Vec2 &measurement, const Vec2 &initialVelocity,
              const Matrix2 &positionCovariance, const Matrix2 &velocityCovariance,
              double initialNll)
    {
        position = measurement;
        velocity = initialVelocity;
        pxx = positionCovariance;
        pvv = velocityCovariance;
        pxv = Matrix2::zero();
        pvx = Matrix2::zero();
        measurements = 2;
        nll = initialNll;
        score = weighting(nll, pxx);
    }

    void predict(double dt, double friction, double processNoisePosition,
                 double processNoiseVelocity)
    {
        position = position + velocity * dt;
        const double speed = velocity.norm();
        if (speed > 1e-6 && friction < 0.0) {
            const double nextSpeed = std::max(0.0, speed + friction * dt);
            velocity = velocity * (nextSpeed / speed);
        }

        const Matrix2 dtIdentity{dt, 0.0, 0.0, dt};
        const Matrix2 nextPxx = pxx + dtIdentity * pvx + pxv * dtIdentity.transpose() +
            dtIdentity * pvv * dtIdentity.transpose();
        const Matrix2 nextPxv = pxv + dtIdentity * pvv;
        const Matrix2 nextPvx = pvx + pvv * dtIdentity.transpose();
        pxx = nextPxx;
        pxv = nextPxv;
        pvx = nextPvx;
        pxx.addDiagonal(processNoisePosition);
        pvv.addDiagonal(processNoiseVelocity);
    }

    void correct(const Vec2 &measurement, const Matrix2 &measurementCovariance)
    {
        const Vec2 innovation = measurement - position;
        const Matrix2 innovationCovariance = pxx + measurementCovariance;
        nll += measurementNll(innovation, innovationCovariance);
        const Matrix2 inverse = innovationCovariance.inverse();
        const Matrix2 positionGain = pxx * inverse;
        const Matrix2 velocityGain = pvx * inverse;
        const Matrix2 oldPxx = pxx;
        const Matrix2 oldPxv = pxv;
        position = position + positionGain * innovation;
        velocity = velocity + velocityGain * innovation;
        pxx = pxx - positionGain * oldPxx;
        pxv = pxv - positionGain * oldPxv;
        pvx = pvx - velocityGain * oldPxx;
        pvv = pvv - velocityGain * oldPxv;
        ++measurements;
        score = weighting(nll, pxx);
    }

    StationaryHypothesis toStationary() const
    {
        StationaryHypothesis result;
        result.position = position;
        result.covariance = pxx;
        result.measurements = measurements;
        result.nll = nll;
        result.score = weighting(nll, pxx);
        return result;
    }
};

template<typename Hypothesis>
void prune(std::vector<Hypothesis> &hypotheses, std::size_t maxCount)
{
    if (hypotheses.size() <= maxCount) return;
    std::sort(hypotheses.begin(), hypotheses.end(),
        [](const Hypothesis &a, const Hypothesis &b) { return a.score < b.score; });
    hypotheses.resize(maxCount);
}

} // namespace

struct BallStateEstimator::Impl
{
    static constexpr std::size_t kMaxHypotheses = 12;
    static constexpr double kInitialNll = 2.302585092994046;

    std::vector<StationaryHypothesis> stationary;
    std::vector<RollingHypothesis> rolling;
    Vec2 bestPosition{};
    Vec2 bestVelocity{};
    Vec2 endPosition{};
    Vec2 lastMeasurement{};
    rclcpp::Time lastUpdate{0, 0, RCL_ROS_TIME};
    bool firstUpdate = true;
    bool valid = false;
    bool rollingBest = false;
    double friction = -0.35;
    double minSpeed = 0.1;
    double positionNoise = 0.001;
    double velocityNoise = 0.001;

    void selectBest()
    {
        valid = false;
        rollingBest = false;
        double bestScore = std::numeric_limits<double>::infinity();
        for (const auto &hypothesis : stationary) {
            if (hypothesis.score < bestScore) {
                bestScore = hypothesis.score;
                bestPosition = hypothesis.position;
                bestVelocity = {};
                valid = true;
                rollingBest = false;
            }
        }
        for (const auto &hypothesis : rolling) {
            if (hypothesis.score < bestScore) {
                bestScore = hypothesis.score;
                bestPosition = hypothesis.position;
                bestVelocity = hypothesis.measurements >= 2 ? hypothesis.velocity : Vec2{};
                valid = true;
                rollingBest = true;
            }
        }
    }

    void normalizeNll()
    {
        double minimum = std::numeric_limits<double>::infinity();
        for (const auto &h : stationary) minimum = std::min(minimum, h.nll);
        for (const auto &h : rolling) minimum = std::min(minimum, h.nll);
        if (!std::isfinite(minimum)) return;
        for (auto &h : stationary) {
            h.nll -= minimum;
            h.score = weighting(h.nll, h.covariance);
        }
        for (auto &h : rolling) {
            h.nll -= minimum;
            h.score = weighting(h.nll, h.pxx);
        }
    }
};

void BallPhysics::clampVelocity(double &vx, double &vy)
{
    const double speed = std::hypot(vx, vy);
    if (speed > MAX_BALL_SPEED) {
        const double scale = MAX_BALL_SPEED / speed;
        vx *= scale;
        vy *= scale;
    }
}

void BallPhysics::propagateWithSpeedFriction(
    double &px, double &py, double &vx, double &vy,
    double dt, double baseFriction, double speedFactor)
{
    if (dt <= 0.0 || !std::isfinite(dt)) return;
    double remaining = dt;
    while (remaining > 1e-9) {
        const double speed = std::hypot(vx, vy);
        if (speed <= 0.01 || !std::isfinite(speed)) {
            vx = vy = 0.0;
            return;
        }
        const double effectiveFriction = std::min(-1e-3,
            baseFriction * (1.0 + std::max(0.0, speedFactor) * speed));
        const double stepDt = std::min(remaining, 0.01);
        const double stopTime = speed / -effectiveFriction;
        const double usedDt = std::min(stepDt, stopTime);
        const double ax = vx / speed * effectiveFriction;
        const double ay = vy / speed * effectiveFriction;
        px += vx * usedDt + 0.5 * ax * usedDt * usedDt;
        py += vy * usedDt + 0.5 * ay * usedDt * usedDt;
        if (usedDt >= stopTime - 1e-9) {
            vx = vy = 0.0;
            return;
        }
        vx += ax * usedDt;
        vy += ay * usedDt;
        remaining -= usedDt;
    }
}

Vec2 BallPhysics::getEndPositionWithSpeedFriction(
    double px, double py, double vx, double vy,
    double baseFriction, double speedFactor)
{
    for (int i = 0; i < 1000 && std::hypot(vx, vy) > 0.01; ++i) {
        propagateWithSpeedFriction(px, py, vx, vy, 0.01, baseFriction, speedFactor);
    }
    return {px, py};
}

BallStateEstimator::BallStateEstimator() : impl_(std::make_unique<Impl>()) {}
BallStateEstimator::~BallStateEstimator() = default;

void BallStateEstimator::reset()
{
    impl_->stationary.clear();
    impl_->rolling.clear();
    impl_->valid = false;
    impl_->rollingBest = false;
    impl_->firstUpdate = true;
}

void BallStateEstimator::update(double posX, double posY, const rclcpp::Time &timestamp,
                                double measurementStdDev)
{
    const Vec2 measurement{posX, posY};
    const double variance = std::max(1e-6, measurementStdDev * measurementStdDev);
    const Matrix2 measurementCovariance{variance, 0.0, 0.0, variance};

    if (impl_->firstUpdate) {
        impl_->stationary.clear();
        impl_->rolling.clear();
        StationaryHypothesis initial;
        initial.init(measurement, measurementCovariance, 0.0);
        impl_->stationary.push_back(initial);
        impl_->lastMeasurement = measurement;
        impl_->lastUpdate = timestamp;
        impl_->firstUpdate = false;
        impl_->selectBest();
        impl_->endPosition = measurement;
        return;
    }

    double dt = (timestamp - impl_->lastUpdate).nanoseconds() / 1e9;
    if (dt > 4.0) {
        reset();
        update(posX, posY, timestamp, measurementStdDev);
        return;
    }
    if (dt <= 0.0) dt = 0.033;

    for (auto &hypothesis : impl_->stationary) hypothesis.predict(impl_->positionNoise);
    for (auto it = impl_->rolling.begin(); it != impl_->rolling.end();) {
        it->predict(dt, impl_->friction, impl_->positionNoise, impl_->velocityNoise);
        if (it->velocity.norm() < impl_->minSpeed) {
            impl_->stationary.push_back(it->toStationary());
            it = impl_->rolling.erase(it);
        } else {
            ++it;
        }
    }
    for (auto &hypothesis : impl_->stationary) hypothesis.correct(measurement, measurementCovariance);
    for (auto &hypothesis : impl_->rolling) hypothesis.correct(measurement, measurementCovariance);

    impl_->normalizeNll();
    prune(impl_->stationary, Impl::kMaxHypotheses - 1);
    prune(impl_->rolling, Impl::kMaxHypotheses - 1);
    impl_->selectBest();

    StationaryHypothesis newStationary;
    newStationary.init(measurement, measurementCovariance, Impl::kInitialNll);
    impl_->stationary.push_back(newStationary);
    if (dt < 0.2) {
        const Vec2 delta = measurement - impl_->lastMeasurement;
        const double distance = delta.norm();
        const double instantSpeed = distance / dt + 0.5 * std::abs(impl_->friction) * dt;
        if (distance > 1e-6 && instantSpeed > impl_->minSpeed) {
            const Vec2 initialVelocity = delta * (instantSpeed / distance);
            const double velocityVariance = 2.0 * variance / (dt * dt);
            RollingHypothesis newRolling;
            newRolling.init(measurement, initialVelocity, measurementCovariance,
                            {velocityVariance, 0.0, 0.0, velocityVariance},
                            Impl::kInitialNll);
            impl_->rolling.push_back(newRolling);
        }
    }
    if (!impl_->valid) impl_->selectBest();

    BallPhysics::clampVelocity(impl_->bestVelocity.x, impl_->bestVelocity.y);
    impl_->endPosition = BallPhysics::getEndPositionWithSpeedFriction(
        impl_->bestPosition.x, impl_->bestPosition.y,
        impl_->bestVelocity.x, impl_->bestVelocity.y,
        impl_->friction, 0.0);
    impl_->lastMeasurement = measurement;
    impl_->lastUpdate = timestamp;
}

Vec2 BallStateEstimator::getEstimatedPosition() const { return impl_->bestPosition; }
Vec2 BallStateEstimator::getEstimatedVelocity() const { return impl_->bestVelocity; }
Vec2 BallStateEstimator::getEndPosition() const { return impl_->endPosition; }
bool BallStateEstimator::isRolling() const { return impl_->rollingBest; }
bool BallStateEstimator::hasValidEstimate() const { return impl_->valid; }
void BallStateEstimator::setFriction(double friction) { impl_->friction = std::min(-1e-3, friction); }
void BallStateEstimator::setMinSpeedThreshold(double speed) { impl_->minSpeed = std::max(0.0, speed); }

} // namespace ball_state_estimator
