#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace game_controller_protocol_v20
{

constexpr std::uint8_t kStructVersion = 20;
constexpr std::size_t kMaxNumPlayers = 20;

constexpr std::uint8_t kGamePhaseNormal = 0;
constexpr std::uint8_t kGamePhasePenaltyShootOut = 1;
constexpr std::uint8_t kGamePhaseExtraTime = 2;
constexpr std::uint8_t kGamePhaseTimeout = 3;

constexpr std::uint8_t kSetPlayNone = 0;
constexpr std::uint8_t kSetPlayDirectFreeKick = 1;
constexpr std::uint8_t kSetPlayIndirectFreeKick = 2;
constexpr std::uint8_t kSetPlayPenaltyKick = 3;
constexpr std::uint8_t kSetPlayThrowIn = 4;
constexpr std::uint8_t kSetPlayGoalKick = 5;
constexpr std::uint8_t kSetPlayCornerKick = 6;

constexpr std::uint8_t kPenaltySentOff = 12;
constexpr std::uint8_t kKickingTeamNone = 255;

struct RobotInfo
{
    std::uint8_t penalty;
    std::uint8_t secsTillUnpenalised;
    std::uint8_t cautions;
};

struct TeamInfo
{
    std::uint8_t teamNumber;
    std::uint8_t fieldPlayerColour;
    std::uint8_t goalkeeperColour;
    std::uint8_t goalkeeper;
    std::uint8_t score;
    std::uint8_t penaltyShot;
    std::uint16_t singleShots;
    std::uint16_t messageBudget;
    RobotInfo players[kMaxNumPlayers];
};

struct GameControlData
{
    char header[4];
    std::uint8_t version;
    std::uint8_t packetNumber;
    std::uint8_t playersPerTeam;
    std::uint8_t competitionType;
    std::uint8_t stopped;
    std::uint8_t gamePhase;
    std::uint8_t state;
    std::uint8_t setPlay;
    std::uint8_t firstHalf;
    std::uint8_t kickingTeam;
    std::int16_t secsRemaining;
    std::int16_t secondaryTime;
    TeamInfo teams[2];
};

static_assert(sizeof(RobotInfo) == 3, "GameController v20 RobotInfo layout mismatch");
static_assert(sizeof(TeamInfo) == 70, "GameController v20 TeamInfo layout mismatch");
static_assert(sizeof(GameControlData) == 158, "GameController v20 packet layout mismatch");

inline bool hasValidHeader(const char header[4])
{
    return std::memcmp(header, "RGme", 4) == 0;
}

// Preserve the legacy ROS/brain secondary-state numbering.
inline std::uint8_t toLegacySecondaryState(std::uint8_t gamePhase, std::uint8_t setPlay)
{
    if (gamePhase != kGamePhaseNormal)
    {
        return gamePhase;
    }

    switch (setPlay)
    {
        case kSetPlayNone: return 0;
        case kSetPlayDirectFreeKick: return 4;
        case kSetPlayIndirectFreeKick: return 5;
        case kSetPlayPenaltyKick: return 6;
        case kSetPlayCornerKick: return 7;
        case kSetPlayGoalKick: return 8;
        case kSetPlayThrowIn: return 9;
        default: return 0;
    }
}

} // namespace game_controller_protocol_v20
