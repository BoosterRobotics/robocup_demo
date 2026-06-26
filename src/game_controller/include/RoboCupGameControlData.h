#ifndef ROBOCUPGAMECONTROLDATA_H
#define ROBOCUPGAMECONTROLDATA_H

#include <stdint.h>

#define GAMECONTROLLER_DATA_PORT   3838
#define GAMECONTROLLER_RETURN_PORT 3939

#define GAMECONTROLLER_STRUCT_HEADER  "RGme"
#define GAMECONTROLLER_STRUCT_VERSION_V19 19
#define GAMECONTROLLER_STRUCT_VERSION_V20 20

// The strategy stack consumes v19-compatible penalty values. The receiver can
// parse both v19 and v20 wire formats and normalizes v20 penalties before
// publishing ROS messages.
#define GAMECONTROLLER_STRUCT_VERSION GAMECONTROLLER_STRUCT_VERSION_V19

#define MAX_NUM_PLAYERS 20

#define TEAM_BLUE   0
#define TEAM_RED    1
#define TEAM_YELLOW 2
#define TEAM_BLACK  3
#define TEAM_WHITE  4
#define TEAM_GREEN  5
#define TEAM_ORANGE 6
#define TEAM_PURPLE 7
#define TEAM_BROWN  8
#define TEAM_GRAY   9

#define COMPETITION_TYPE_SMALL  0
#define COMPETITION_TYPE_MIDDLE 1
#define COMPETITION_TYPE_LARGE  2

#define GAME_PHASE_NORMAL            0
#define GAME_PHASE_PENALTY_SHOOT_OUT 1
#define GAME_PHASE_EXTRA_TIME        2
#define GAME_PHASE_TIMEOUT           3

#define STATE_INITIAL  0
#define STATE_READY    1
#define STATE_SET      2
#define STATE_PLAYING  3
#define STATE_FINISHED 4

#define SET_PLAY_NONE               0
#define SET_PLAY_DIRECT_FREE_KICK   1
#define SET_PLAY_INDIRECT_FREE_KICK 2
#define SET_PLAY_PENALTY_KICK       3
#define SET_PLAY_THROW_IN           4
#define SET_PLAY_GOAL_KICK          5
#define SET_PLAY_CORNER_KICK        6

#define KICKING_TEAM_NONE 255

#define PENALTY_V19_NONE                    0
#define PENALTY_V19_ILLEGAL_POSITIONING     1
#define PENALTY_V19_MOTION_IN_SET           2
#define PENALTY_V19_LOCAL_GAME_STUCK        3
#define PENALTY_V19_INCAPABLE_ROBOT         4
#define PENALTY_V19_PICK_UP                 5
#define PENALTY_V19_BALL_HOLDING            6
#define PENALTY_V19_LEAVING_THE_FIELD       7
#define PENALTY_V19_PLAYING_WITH_ARMS_HANDS 8
#define PENALTY_V19_PUSHING                 9
#define PENALTY_V19_SENT_OFF                10
#define PENALTY_V19_SUBSTITUTE              11

#define PENALTY_NONE                    PENALTY_V19_NONE
#define PENALTY_ILLEGAL_POSITIONING     PENALTY_V19_ILLEGAL_POSITIONING
#define PENALTY_MOTION_IN_SET           PENALTY_V19_MOTION_IN_SET
#define PENALTY_LOCAL_GAME_STUCK        PENALTY_V19_LOCAL_GAME_STUCK
#define PENALTY_INCAPABLE_ROBOT         PENALTY_V19_INCAPABLE_ROBOT
#define PENALTY_PICK_UP                 PENALTY_V19_PICK_UP
#define PENALTY_BALL_HOLDING            PENALTY_V19_BALL_HOLDING
#define PENALTY_LEAVING_THE_FIELD       PENALTY_V19_LEAVING_THE_FIELD
#define PENALTY_PLAYING_WITH_ARMS_HANDS PENALTY_V19_PLAYING_WITH_ARMS_HANDS
#define PENALTY_PUSHING                 PENALTY_V19_PUSHING
#define PENALTY_SENT_OFF                PENALTY_V19_SENT_OFF
#define PENALTY_SUBSTITUTE              PENALTY_V19_SUBSTITUTE

#define PENALTY_V20_NONE                    0
#define PENALTY_V20_ILLEGAL_POSITIONING     1
#define PENALTY_V20_MOTION_IN_SET           2
#define PENALTY_V20_MOTION_IN_STOP          3
#define PENALTY_V20_LOCAL_GAME_STUCK        4
#define PENALTY_V20_INCAPABLE_ROBOT         5
#define PENALTY_V20_PICK_UP                 6
#define PENALTY_V20_BALL_HOLDING            7
#define PENALTY_V20_LEAVING_THE_FIELD       8
#define PENALTY_V20_PLAYING_WITH_ARMS_HANDS 9
#define PENALTY_V20_PUSHING                 10
#define PENALTY_V20_CAUTIONED               11
#define PENALTY_V20_SENT_OFF                12
#define PENALTY_V20_SUBSTITUTE              13

struct RobotInfo
{
  uint8_t penalty;
  uint8_t secsTillUnpenalised;
  uint8_t warnings;
  uint8_t cautions;
};

struct RobotInfoV20
{
  uint8_t penalty;
  uint8_t secsTillUnpenalised;
  uint8_t cautions;
};

struct TeamInfo
{
  uint8_t teamNumber;
  uint8_t fieldPlayerColour;
  uint8_t goalkeeperColour;
  uint8_t goalkeeper;
  uint8_t score;
  uint8_t penaltyShot;
  uint16_t singleShots;
  uint16_t messageBudget;
  struct RobotInfo players[MAX_NUM_PLAYERS];
};

struct TeamInfoV20
{
  uint8_t teamNumber;
  uint8_t fieldPlayerColour;
  uint8_t goalkeeperColour;
  uint8_t goalkeeper;
  uint8_t score;
  uint8_t penaltyShot;
  uint16_t singleShots;
  uint16_t messageBudget;
  struct RobotInfoV20 players[MAX_NUM_PLAYERS];
};

struct RoboCupGameControlData
{
  char header[4];
  uint8_t version;
  uint8_t packetNumber;
  uint8_t playersPerTeam;
  uint8_t competitionType;
  uint8_t stopped;
  uint8_t gamePhase;
  uint8_t state;
  uint8_t setPlay;
  uint8_t firstHalf;
  uint8_t kickingTeam;
  int16_t secsRemaining;
  int16_t secondaryTime;
  struct TeamInfo teams[2];
};

struct RoboCupGameControlDataV20
{
  char header[4];
  uint8_t version;
  uint8_t packetNumber;
  uint8_t playersPerTeam;
  uint8_t competitionType;
  uint8_t stopped;
  uint8_t gamePhase;
  uint8_t state;
  uint8_t setPlay;
  uint8_t firstHalf;
  uint8_t kickingTeam;
  int16_t secsRemaining;
  int16_t secondaryTime;
  struct TeamInfoV20 teams[2];
};

typedef struct RobotInfo RobotInfoV19;
typedef struct TeamInfo TeamInfoV19;
typedef struct RoboCupGameControlData RoboCupGameControlDataV19;

#define GAMECONTROLLER_RETURN_STRUCT_HEADER  "RGrt"
#define GAMECONTROLLER_RETURN_STRUCT_VERSION 4

struct RoboCupGameControlReturnData
{
  char header[4];
  uint8_t version;
  uint8_t playerNum;
  uint8_t teamNum;
  uint8_t fallen;
  float pose[3];
  float ballAge;
  float ball[2];

#ifdef __cplusplus
  RoboCupGameControlReturnData() :
    version(GAMECONTROLLER_RETURN_STRUCT_VERSION),
    playerNum(0),
    teamNum(0),
    fallen(255),
    ballAge(-1.f)
  {
    const char* init = GAMECONTROLLER_RETURN_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];
    pose[0] = 0.f;
    pose[1] = 0.f;
    pose[2] = 0.f;
    ball[0] = 0.f;
    ball[1] = 0.f;
  }
#endif
};

#endif
