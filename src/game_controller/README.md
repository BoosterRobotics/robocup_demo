# robocup game_controller
## 功能说明
接收 Robocup 裁判机 GameController 发到局域网的 UDP 包，转成 Ros2 Topic 消息，供 robocup brain 使用

## 启动方式
```
# 进入 robocup_demo 目录
> cd robocup_demo
# 编译
> ./script/build.sh
# 运行
> ./start_game_controller.sh
# 查看 topic 信息
> ros2 topic info -v /robocup/game_controller
```
