#!/usr/bin/env python3
"""
最小化测试: 直接向 /LocoApiTopicReq 发 VisionKick (api_id=2038) 命令
用法: 在机器人上运行 (先 source /opt/ros/humble/setup.bash):
  python3 test_visual_kick.py

机器人必须已经处于 walking 模式 (站立状态)
"""
import rclpy
from rclpy.node import Node
from booster_msgs.msg import RpcReqMsg
import json, uuid, time, sys

class VisionKickTest(Node):
    def __init__(self):
        super().__init__('vision_kick_test')
        self.pub = self.create_publisher(RpcReqMsg, 'LocoApiTopicReq', 10)
        time.sleep(1)  # 等待 publisher 连接

    def send(self, api_id, body_dict=None):
        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())
        msg.header = json.dumps({"api_id": api_id})
        msg.body = json.dumps(body_dict) if body_dict else ""
        self.pub.publish(msg)
        self.get_logger().info(f"Sent api_id={api_id}, body={msg.body}")

    def visual_kick_start(self):
        self.send(2038, {"start": True})

    def visual_kick_stop(self):
        self.send(2038, {"start": False})

    def change_mode_walking(self):
        self.send(2000, {"mode": "walking"})

def main():
    rclpy.init()
    node = VisionKickTest()

    print("\n=== VisionKick 最小化测试 ===")
    print("确保机器人已站立 (walking 模式)")
    print("把球放在机器人前方 0.5-1m 处")
    print()
    print("命令:")
    print("  1 - 发送 VisionKick start (api_id=2038)")
    print("  2 - 发送 VisionKick stop  (api_id=2038)")
    print("  3 - 发送 ChangeMode walking (api_id=2000)")
    print("  q - 退出")
    print()

    try:
        while True:
            cmd = input("> ").strip()
            if cmd == '1':
                node.visual_kick_start()
            elif cmd == '2':
                node.visual_kick_stop()
            elif cmd == '3':
                node.change_mode_walking()
            elif cmd == 'q':
                break
            else:
                print("无效命令")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
