#ifndef ROBOCUPGAMECONTROLDATA_H
#define ROBOCUPGAMECONTROLDATA_H

#include "SPLCoachMessage.h"

#define GAMECONTROLLER_DATA_PORT          3838
#define GAMECONTROLLER_RETURN_PORT        3939

#define GAMECONTROLLER_STRUCT_HEADER      "RGme"
#define HL_GAMECONTROLLER_STRUCT_VERSION  12
#define GAMECONTROLLER_STRUCT_VERSION     15

#define MAX_NUM_PLAYERS                   20
#define HL_MAX_NUM_PLAYERS                11

#define GAMECONTROLLER_RETURN_STRUCT_HEADER      "RGrt"

#pragma region Humanoid Constants

// SPL
#define TEAM_BLUE                   0 // cyan, blue, violet
#define TEAM_RED                    1 // magenta, pink (not red/orange)
#define TEAM_YELLOW                 2 // yellow
#define TEAM_BLACK                  3 // black, dark gray
#define TEAM_WHITE                  4 // white
#define TEAM_GREEN                  5 // green
#define TEAM_ORANGE                 6 // orange
#define TEAM_PURPLE                 7 // purple, violet
#define TEAM_BROWN                  8 // brown
#define TEAM_GRAY                   9 // lighter grey

// HL
#define TEAM_CYAN                   0
#define TEAM_MAGENTA                1
#define DROPBALL                    255

#define GAME_KID_SIZE               0
#define GAME_ADULT                  1
#define GAME_DROPIN                 2

#define STATE_INITIAL               0
#define STATE_READY                 1
#define STATE_SET                   2
#define STATE_PLAYING               3
#define STATE_FINISHED              4
#define STATE_DANIEL                15

#define STATE2_NORMAL               0
#define STATE2_PENALTYSHOOT         1
#define STATE2_OVERTIME             2
#define STATE2_TIMEOUT              3
#define STATE2_DIRECT_FREEKICK      4
#define STATE2_INDIRECT_FREEKICK    5
#define STATE2_PENALTYKICK          6
#define STATE2_CORNER_KICK          7
#define STATE2_GOAL_KICK            8
#define STATE2_THROW_IN             9

#define PENALTY_NONE                        0

#define UNKNOWN                             255
#define NONE                                0
#define SUBSTITUTE                          14 // normal substitute
#define MANUAL                              15

#define SPL_ILLEGAL_BALL_CONTACT            1
#define SPL_PLAYER_PUSHING                  2
#define SPL_ILLEGAL_MOTION_IN_SET           3
#define SPL_INACTIVE_PLAYER                 4
#define SPL_ILLEGAL_DEFENDER                5
#define SPL_LEAVING_THE_FIELD               6
#define SPL_KICK_OFF_GOAL                   7
#define SPL_REQUEST_FOR_PICKUP              8
#define SPL_COACH_MOTION                    9


#define HL_BALL_MANIPULATION                30 // ball man
#define HL_PHYSICAL_CONTACT                 31 // pushing
#define HL_ILLEGAL_ATTACK                   32
#define HL_ILLEGAL_DEFENSE                  33
#define HL_PICKUP_OR_INCAPABLE              34 // read the variable daniel ... (comment by daniel)
#define HL_SERVICE                          35

#define HL_GAMECONTROLLER_RETURN_STRUCT_VERSION     2

#define GAMECONTROLLER_RETURN_MSG_MAN_PENALISE   0
#define GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE 1
#define GAMECONTROLLER_RETURN_MSG_ALIVE          2

#pragma endregion // Humanoid stuff

#pragma region SPL Constants

#define COMPETITION_PHASE_ROUNDROBIN 0
#define COMPETITION_PHASE_PLAYOFF    1

#define COMPETITION_TYPE_NORMAL                0
#define COMPETITION_TYPE_DYNAMIC_BALL_HANDLING 1

#define GAME_PHASE_NORMAL       0
#define GAME_PHASE_PENALTYSHOOT 1
#define GAME_PHASE_OVERTIME     2
#define GAME_PHASE_TIMEOUT      3

#define SET_PLAY_NONE              0
#define SET_PLAY_GOAL_KICK         1
#define SET_PLAY_PUSHING_FREE_KICK 2
#define SET_PLAY_CORNER_KICK       3
#define SET_PLAY_KICK_IN           4
#define SET_PLAY_PENALTY_KICK      5

#define PENALTY_SPL_ILLEGAL_BALL_CONTACT      1 // ball holding / playing with hands
#define PENALTY_SPL_PLAYER_PUSHING            2
#define PENALTY_SPL_ILLEGAL_MOTION_IN_SET     3 // heard whistle too early?
#define PENALTY_SPL_INACTIVE_PLAYER           4 // fallen, inactive
#define PENALTY_SPL_ILLEGAL_POSITION          5
#define PENALTY_SPL_LEAVING_THE_FIELD         6
#define PENALTY_SPL_REQUEST_FOR_PICKUP        7
#define PENALTY_SPL_LOCAL_GAME_STUCK          8
#define PENALTY_SPL_ILLEGAL_POSITION_IN_SET   9
#define PENALTY_SPL_PLAYER_STANCE             10

#define PENALTY_SUBSTITUTE                    14
#define PENALTY_MANUAL                        15

#define GAMECONTROLLER_RETURN_STRUCT_VERSION     4

#pragma endregion // SPL Stuff

#pragma region Humanoid Structures
struct HlRobotInfo
{
  uint8_t penalty;              // penalty state of the player
  uint8_t secsTillUnpenalised;  // estimate of time till unpenalised
  uint8_t numberOfWarnings;     // number of warnings
  uint8_t yellowCardCount;      // number of yellow cards
  uint8_t redCardCount;         // number of red cards
  uint8_t goalKeeper;           // flags if robot is goal keeper
};

struct HlTeamInfo
{
  uint8_t teamNumber;           // unique team number
  uint8_t fieldPlayerColour;           // colour of the team
  uint8_t score;                // team's score
  uint8_t penaltyShot;          // penalty shot counter
  uint16_t singleShots;         // bits represent penalty shot success
  uint8_t coachSequence;        // sequence number of the coach's message
  uint8_t coachMessage[SPL_COACH_MESSAGE_SIZE]; // the coach's message to the team
  struct HlRobotInfo coach;
  struct HlRobotInfo players[HL_MAX_NUM_PLAYERS]; // the team's players
};

struct HlRoboCupGameControlData
{
  char header[4];               // header to identify the structure
  uint16_t version;             // version of the data structure
  uint8_t packetNumber;         // number incremented with each packet sent (with wraparound)
  uint8_t playersPerTeam;       // the number of players on a team
  uint8_t gameType;             // type of the game (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
  uint8_t state;                // state of the game (STATE_READY, STATE_PLAYING, etc)
  uint8_t firstHalf;            // 1 = game in first half, 0 otherwise
  uint8_t kickOffTeam;          // the team number of the next team to kick off or DROPBALL
  uint8_t secondaryState;       // extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
  char secondaryStateInfo[4];   // Extra info on the secondary state
  uint8_t dropInTeam;           // number of team that caused last drop in
  uint16_t dropInTime;          // number of seconds passed since the last drop in. -1 (0xffff) before first dropin
  uint16_t secsRemaining;       // estimate of number of seconds remaining in the half
  uint16_t secondaryTime;       // number of seconds shown as secondary time (remaining ready, until free ball, etc)
  struct HlTeamInfo teams[2];
};

struct HlRoboCupGameControlReturnData
{
  char header[4];
  uint8_t version;
  uint8_t team;    // team number
  uint8_t player;  // player number starts with 1
  uint8_t message; // one of the three messages defined above

#ifdef __cplusplus
  // constructor
  HlRoboCupGameControlReturnData() : version(HL_GAMECONTROLLER_RETURN_STRUCT_VERSION)
  {
    const char* init = GAMECONTROLLER_RETURN_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];
  }
#endif
};

#pragma endregion

#pragma region SPL Structures
struct RobotInfo
{
  uint8_t penalty;             // penalty state of the player
  uint8_t secsTillUnpenalised; // estimate of time till unpenalised
};

struct TeamInfo
{
  uint8_t teamNumber;                        // unique team number
  uint8_t fieldPlayerColour;                 // colour of the field players
  uint8_t goalkeeperColour;                  // colour of the goalkeeper
  uint8_t goalkeeper;                        // player number of the goalkeeper (1-MAX_NUM_PLAYERS)
  uint8_t score;                             // team's score
  uint8_t penaltyShot;                       // penalty shot counter
  uint16_t singleShots;                      // bits represent penalty shot success
  uint16_t messageBudget;                    // number of team messages the team is allowed to send for the remainder of the game
  struct RobotInfo players[MAX_NUM_PLAYERS]; // the team's players
};

struct RoboCupGameControlData
{
  char header[4];           // header to identify the structure
  uint8_t version;          // version of the data structure
  uint8_t packetNumber;     // number incremented with each packet sent (with wraparound)
  uint8_t playersPerTeam;   // the number of players on a team
  uint8_t competitionPhase; // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  uint8_t competitionType;  // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_DYNAMIC_BALL_HANDLING)
  uint8_t gamePhase;        // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  uint8_t state;            // state of the game (STATE_READY, STATE_PLAYING, etc)
  uint8_t setPlay;          // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)
  uint8_t firstHalf;        // 1 = game in first half, 0 otherwise
  uint8_t kickingTeam;      // the team number of the next team to kick off, free kick etc
  int16_t secsRemaining;    // estimate of number of seconds remaining in the half
  int16_t secondaryTime;    // number of seconds shown as secondary time (remaining ready, until free ball, etc)
  struct TeamInfo teams[2];
};

struct RoboCupGameControlReturnData
{
  char header[4];     // "RGrt"
  uint8_t version;    // has to be set to GAMECONTROLLER_RETURN_STRUCT_VERSION
  uint8_t playerNum;  // player number starts with 1
  uint8_t teamNum;    // team number
  uint8_t fallen;     // 1 means that the robot is fallen, 0 means that the robot can play

  // position and orientation of robot
  // coordinates in millimeters
  // 0,0 is in center of field
  // +ve x-axis points towards the goal we are attempting to score on
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  // angle in radians, 0 along the +x axis, increasing counter clockwise
  float pose[3];         // x,y,theta

  // ball information
  float ballAge;         // seconds since this robot last saw the ball. -1.f if we haven't seen it

  // position of ball relative to the robot
  // coordinates in millimeters
  // 0,0 is in center of the robot
  // +ve x-axis points forward from the robot
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  float ball[2];

#ifdef __cplusplus
  // constructor
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

#pragma endregion

#endif // ROBOCUPGAMECONTROLDATA_H