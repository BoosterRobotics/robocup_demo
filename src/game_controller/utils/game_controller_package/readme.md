# 打包用于发布的 gamecontroller 和 joystick 包

## 编译产物并放入包中
```
colcon build
cp -r install ./src/game_controller/utils/game_controller_package

```

## 打包方式

```
cd ./src/game_controller/utils/game_controller_package

makeself ../game_controller_package ../game_controller_0.0.2.run "booster_game_controller" ./install.sh
```

## gamecontroller 修改配置方式
```
vim /opt/robocup/install/game_controller/share/game_controller/launch/launch.py
```