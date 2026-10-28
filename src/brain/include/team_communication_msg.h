#pragma once

#include <cstddef>
#include "types.h"

#define VALIDATION_COMMUNICATION_LEGACY 31202
#define VALIDATION_COMMUNICATION 31203
struct TeamCommunicationMsg
{
    int validation = VALIDATION_COMMUNICATION; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
    int playerRole; // 1: striker, 2: goal_keeper, 3: unknown
    bool isAlive; // 是否在场上, 且没有在罚时中
    bool isLead; // 是否在控球状态
    bool ballDetected;
    bool ballLocationKnown;
    double ballConfidence;
    double ballRange;
    double cost; // 计算从当前状态到能踢到球的成本
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir;
    double thetaRb;
    int cmdId; // 每个 player 发布时, 需要将 cmdId + 1. 用来代表发布的顺序. 
    int cmd; // 百位为 1 时, 代表自己要球控球. 十位为 1 时, 代表守门员要求另一个球员接替守门员角色, 此时个位数字代表接替球员的 playerId. 例如: 100, 代表自己要球控球, 另一个 striker 进入辅助角色; 011, 代表守门员要出击, 要求 1 号球员接替守门.  
    // 新字段只能追加在旧包末尾，不能插到中间破坏 120 字节旧协议的字段偏移。
    bool isInVisualKick; // 是否正在执行视觉踢球
};

constexpr std::size_t TEAM_COMMUNICATION_LEGACY_SIZE =
    offsetof(TeamCommunicationMsg, isInVisualKick);
static_assert(TEAM_COMMUNICATION_LEGACY_SIZE == 120,
              "legacy team packet prefix must remain byte-compatible");

// Original 120-byte packet. Keep an explicit decoder type instead of copying
// only a prefix into TeamCommunicationMsg.
struct TeamCommunicationMsgLegacy
{
    int validation = VALIDATION_COMMUNICATION_LEGACY;
    int communicationId;
    int teamId;
    int playerId;
    int playerRole;
    bool isAlive;
    bool isLead;
    bool ballDetected;
    bool ballLocationKnown;
    double ballConfidence;
    double ballRange;
    double cost;
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir;
    double thetaRb;
    int cmdId;
    int cmd;
};
static_assert(sizeof(TeamCommunicationMsgLegacy) == TEAM_COMMUNICATION_LEGACY_SIZE,
              "legacy team packet layout changed");

// 兼容本次迁移早期生成过的 128 字节过渡包：isInVisualKick 曾被插在 bool 字段中间。
// 仅用于网络解码/滚动升级，新的发送端使用上面的可追加布局。
struct TeamCommunicationMsgInsertedV1
{
    int validation = VALIDATION_COMMUNICATION_LEGACY;
    int communicationId;
    int teamId;
    int playerId;
    int playerRole;
    bool isAlive;
    bool isLead;
    bool isInVisualKick;
    bool ballDetected;
    bool ballLocationKnown;
    double ballConfidence;
    double ballRange;
    double cost;
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir;
    double thetaRb;
    int cmdId;
    int cmd;
};
static_assert(sizeof(TeamCommunicationMsgInsertedV1) == 128,
              "transitional team packet layout changed");
