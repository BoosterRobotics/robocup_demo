#pragma once

#include <string>
#include <random>
#include <sstream>

#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_api.hpp>

#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <booster_msgs/third_party/nlohmann_json/json.hpp>

namespace booster_msgs
{
    std::string GenUUID()
    {
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<int> uni(0, 15);
        std::uniform_int_distribution<int> uni8(8, 11);

        std::stringstream ss;
        ss << std::hex;
        for (int i = 0; i < 8; i++)
            ss << uni(rng);
        ss << "-";
        for (int i = 0; i < 4; i++)
            ss << uni(rng);
        ss << "-4";
        for (int i = 0; i < 3; i++)
            ss << uni(rng);
        ss << "-";
        ss << uni8(rng);
        for (int i = 0; i < 3; i++)
            ss << uni(rng);
        ss << "-";
        for (int i = 0; i < 12; i++)
            ss << uni(rng);

        return ss.str();
    }

    msg::RpcReqMsg ConstructMsg(booster::robot::b1::LocoApiId api_id, nlohmann::json json_body)
    {
        msg::RpcReqMsg msg;

        nlohmann::json json_header;
        json_header["api_id"] = static_cast<int64_t>(api_id);
        msg.uuid = GenUUID();
        msg.header = json_header.dump();
        msg.body = json_body.dump();
        return msg;
    }

    msg::RpcReqMsg ConstructMsgDir(int64_t api_id, nlohmann::json json_body)
    {
        msg::RpcReqMsg msg;

        nlohmann::json json_header;
        json_header["api_id"] = static_cast<int64_t>(api_id);
        msg.uuid = GenUUID();
        msg.header = json_header.dump();
        msg.body = json_body.dump();
        return msg;
    }

    msg::RpcReqMsg CreateChangeModeMsg(booster::robot::RobotMode mode)
    {
        nlohmann::json body;
        body["mode"] = static_cast<int>(mode);
        return ConstructMsg(booster::robot::b1::LocoApiId::kChangeMode, body);
    }

    msg::RpcReqMsg CreateMoveMsg(float vx, float vy, float vyaw)
    {
        nlohmann::json body;
        body["vx"] = vx;
        body["vy"] = vy;
        body["vyaw"] = vyaw;
        return ConstructMsg(booster::robot::b1::LocoApiId::kMove, body);
    }

    msg::RpcReqMsg CreateRotateHeadMsg(float pitch, float yaw)
    {
        nlohmann::json body;
        body["pitch"] = pitch;
        body["yaw"] = yaw;
        return ConstructMsg(booster::robot::b1::LocoApiId::kRotateHead, body);
    }

    msg::RpcReqMsg CreateRotateHeadWithDirectionMsg(int pitch_direction, int yaw_direction)
    {
        nlohmann::json body;
        body["pitch_direction"] = pitch_direction;
        body["yaw_direction"] = yaw_direction;
        return ConstructMsg(booster::robot::b1::LocoApiId::kRotateHeadWithDirection, body);
    }

    msg::RpcReqMsg CreateWaveHandMsg(booster::robot::b1::HandIndex hand_index, booster::robot::b1::HandAction hand_action)
    {
        nlohmann::json body;
        body["hand_index"] = static_cast<int>(hand_index);
        body["hand_action"] = static_cast<int>(hand_action);
        return ConstructMsg(booster::robot::b1::LocoApiId::kWaveHand, body);
    }

    msg::RpcReqMsg CreateLieDownMsg()
    {
        return ConstructMsg(booster::robot::b1::LocoApiId::kLieDown, "");
    }

    msg::RpcReqMsg CreateGetUpMsg()
    {
        return ConstructMsg(booster::robot::b1::LocoApiId::kGetUp, "");
    }

}