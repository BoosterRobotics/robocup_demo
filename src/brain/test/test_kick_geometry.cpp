#include <cmath>
#include <iostream>
#include <string>

#include "utils/kick_geometry.h"

namespace
{
bool expectNear(const std::string &name, double actual, double expected,
                double tolerance = 1e-9)
{
    if (std::abs(actual - expected) <= tolerance) return true;
    std::cerr << name << ": expected " << expected << ", got " << actual << '\n';
    return false;
}

bool expectTrue(const std::string &name, bool value)
{
    if (value) return true;
    std::cerr << name << ": expected true\n";
    return false;
}

bool expectFalse(const std::string &name, bool value)
{
    if (!value) return true;
    std::cerr << name << ": expected false\n";
    return false;
}
} // namespace

int main()
{
    using namespace kick_geometry;
    bool ok = true;

    const double deg = kPi / 180.0;
    ok &= expectNear("wrapped goal opening", minorArcWidth(170.0 * deg, -170.0 * deg),
                     20.0 * deg);
    ok &= expectTrue("angle inside wrapped opening",
                     isAngleWithinMinorArc(kPi, 170.0 * deg, -170.0 * deg));
    ok &= expectFalse("angle outside wrapped opening",
                      isAngleWithinMinorArc(0.0, 170.0 * deg, -170.0 * deg));

    const double expected = std::atan2(-2.0, 11.0);
    ok &= expectNear("opponent goal direction",
                     directionToOpponentGoal(0.0, 2.0, 11.0), expected);

    const double beforeLine = directionToOpponentGoal(10.99, 0.5, 11.0);
    const double afterLine = directionToOpponentGoal(11.01, 0.5, 11.0);
    ok &= expectTrue("goal-line direction remains continuous",
                     angleDistance(beforeLine, afterLine) < 0.02);
    ok &= expectTrue("beyond-line direction does not snap to zero",
                     std::abs(afterLine) > 0.1);
    const double farBeyondLine = directionToOpponentGoal(13.0, 0.5, 11.0);
    ok &= expectTrue("far beyond line still aims forward",
                     std::cos(farBeyondLine) > 0.0);

    ok &= expectNear("degenerate target fallback",
                     directionToTarget(1.0, 2.0, 1.0, 2.0, 0.7), 0.7);

    ok &= expectNear("central indirect set play has a fresh direction",
                     directionForIndirectSetPlay(0.0, 0.0, 11.0, 14.0), 0.0);
    ok &= expectNear("left sideline set play turns inward",
                     directionForIndirectSetPlay(2.0, 6.0, 11.0, 14.0), -kPi / 2.0);
    ok &= expectNear("right sideline set play turns inward",
                     directionForIndirectSetPlay(2.0, -6.0, 11.0, 14.0), kPi / 2.0);

    return ok ? 0 : 1;
}
