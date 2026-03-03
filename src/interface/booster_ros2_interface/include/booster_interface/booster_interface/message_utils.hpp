#pragma once

#include "booster_interface/msg/booster_api_req_msg.hpp"
#include <booster/robot/common/robot_shared.hpp>
#include <booster/robot/common/entities.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_api.hpp>

namespace booster_interface {

using namespace booster::robot::b1;
using namespace booster::robot;

msg::BoosterApiReqMsg ConstructMsg(LocoApiId api_id, std::string json_body) {
    msg::BoosterApiReqMsg msg;
    msg.api_id = static_cast<int64_t>(api_id);
    msg.body = json_body;
    return msg;
}

template <LocoApiId api_id, typename ParameterType = void, typename... Args>
msg::BoosterApiReqMsg CreateMsg(Args &&...args) {
    if constexpr (std::is_same_v<ParameterType, void>) {
        return ConstructMsg(api_id, "");
    } else {
        ParameterType param(std::forward<Args>(args)...);
        return ConstructMsg(api_id, param.ToJson().dump());
    }
}

/**
 * @deprecated Legacy API – Use the newer template interface above instead.
 * This version is no longer actively maintained.
 */
msg::BoosterApiReqMsg CreateChangeModeMsg(booster::robot::RobotMode mode) {
    ChangeModeParameter change_mode(mode);
    std::string param = change_mode.ToJson().dump();
    return ConstructMsg(LocoApiId::kChangeMode, param);
}
msg::BoosterApiReqMsg CreateMoveMsg(float vx, float vy, float vyaw) {
    MoveParameter move(vx, vy, vyaw);
    std::string param = move.ToJson().dump();
    return ConstructMsg(LocoApiId::kMove, param);
}
msg::BoosterApiReqMsg CreateRotateHeadMsg(float pitch, float yaw) {
    RotateHeadParameter head_ctrl(pitch, yaw);
    std::string param = head_ctrl.ToJson().dump();
    return ConstructMsg(LocoApiId::kRotateHead, param);
}
msg::BoosterApiReqMsg CreateRotateHeadWithDirectionMsg(int pitch_direction, int yaw_direction) {
    RotateHeadWithDirectionParameter head_ctrl(pitch_direction, yaw_direction);
    std::string param = head_ctrl.ToJson().dump();
    return ConstructMsg(LocoApiId::kRotateHeadWithDirection, param);
}
msg::BoosterApiReqMsg CreateWaveHandMsg(HandIndex hand_index, HandAction hand_action) {
    WaveHandParameter wave_hand(hand_index, hand_action);
    std::string body = wave_hand.ToJson().dump();
    return ConstructMsg(LocoApiId::kWaveHand, body);
}
msg::BoosterApiReqMsg CreateLieDownMsg() {
    return ConstructMsg(LocoApiId::kLieDown, "");
}
msg::BoosterApiReqMsg CreateGetUpMsg() {
    return ConstructMsg(LocoApiId::kGetUp, "");
}
msg::BoosterApiReqMsg CreateMoveHandEndEffectorWithAuxMsg(
    const Posture &target_posture,
    const Posture &aux_posture,
    int time_millis,
    HandIndex hand_index) {
    MoveHandEndEffectorParameter move_hand(target_posture, aux_posture, time_millis, hand_index);
    std::string param = move_hand.ToJson().dump();
    return ConstructMsg(LocoApiId::kMoveHandEndEffector, param);
}
msg::BoosterApiReqMsg CreateMoveHandEndEffectorMsg(
    const Posture &target_posture,
    int time_millis,
    HandIndex hand_index) {
    MoveHandEndEffectorParameter move_hand(target_posture, time_millis, hand_index);
    std::string param = move_hand.ToJson().dump();
    return ConstructMsg(LocoApiId::kMoveHandEndEffector, param);
}
msg::BoosterApiReqMsg CreateControlGripperMsg(
    const GripperMotionParameter &motion_param,
    GripperControlMode mode,
    HandIndex hand_index) {
    ControlGripperParameter control_gripper(motion_param, mode, hand_index);
    std::string param = control_gripper.ToJson().dump();
    return ConstructMsg(LocoApiId::kControlGripper, param);
}
msg::BoosterApiReqMsg CreateSwitchHandEndEffectorControlModeMsg(bool switch_on) {
    SwitchHandEndEffectorControlModeParameter switch_param(switch_on);
    std::string param = switch_param.ToJson().dump();
    return ConstructMsg(LocoApiId::kSwitchHandEndEffectorControlMode, param);
}

} // namespace booster_interface