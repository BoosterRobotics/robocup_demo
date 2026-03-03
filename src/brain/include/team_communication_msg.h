#pragma once

#include "types.h"

#define VALIDATION_COMMUNICATION 31202
#define VALIDATION_DISCOVERY 41203
struct TeamCommunicationMsg
{
    int validation = VALIDATION_COMMUNICATION; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
    int playerRole; // 1: striker, 2: goal_keeper, 3: unknown
    bool isAlive; // Whether on the field and not currently penalized
    bool isLead; // Whether in ball-control state
    bool ballDetected;
    bool ballLocationKnown;
    double ballConfidence;
    double ballRange;
    double cost; // Estimated cost to reach/kick the ball from current state
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir;
    double thetaRb;
    int cmdId; // Each player increments cmdId when publishing; used to indicate message order.
    int cmd; // Encoded command: hundreds digit=1 means self requests ball control; tens digit=1 means goalkeeper requests substitution, units digit stores substitute playerId. e.g. 100 = self requests ball control; 011 = goalkeeper goes out and requests player 1 to substitute.
};

struct TeamDiscoveryMsg
{
    int validation = VALIDATION_DISCOVERY; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
};
