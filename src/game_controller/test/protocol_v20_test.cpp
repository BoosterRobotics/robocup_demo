#include <cassert>

#include "RoboCupGameControlData.h"
#include "game_controller_protocol_v20.h"

static_assert(sizeof(HlRoboCupGameControlData) == 688,
              "Legacy Humanoid GameController packet layout mismatch");
static_assert(sizeof(RoboCupGameControlReturnData) == 32,
              "GameController v20 return packet layout mismatch");

int main()
{
    using namespace game_controller_protocol_v20;

    assert(toLegacySecondaryState(kGamePhaseNormal, kSetPlayNone) == 0);
    assert(toLegacySecondaryState(kGamePhasePenaltyShootOut, kSetPlayNone) == 1);
    assert(toLegacySecondaryState(kGamePhaseExtraTime, kSetPlayNone) == 2);
    assert(toLegacySecondaryState(kGamePhaseTimeout, kSetPlayNone) == 3);
    assert(toLegacySecondaryState(kGamePhaseNormal, kSetPlayDirectFreeKick) == 4);
    assert(toLegacySecondaryState(kGamePhaseNormal, kSetPlayIndirectFreeKick) == 5);
    assert(toLegacySecondaryState(kGamePhaseNormal, kSetPlayPenaltyKick) == 6);
    assert(toLegacySecondaryState(kGamePhaseNormal, kSetPlayCornerKick) == 7);
    assert(toLegacySecondaryState(kGamePhaseNormal, kSetPlayGoalKick) == 8);
    assert(toLegacySecondaryState(kGamePhaseNormal, kSetPlayThrowIn) == 9);

    const char validHeader[4] = {'R', 'G', 'm', 'e'};
    const char invalidHeader[4] = {'R', 'G', 'T', 'D'};
    assert(hasValidHeader(validHeader));
    assert(!hasValidHeader(invalidHeader));
    return 0;
}
