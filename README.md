# fixedwing-airtrack

## 简介
本项目为无人机航迹跟踪控制软件，包含KCF追踪算法、PID控制器、无人机控制算法等。
## 测试环境
- Ubuntu 24.04
- ROS2 jazzy

## Setup
### 进入工作空间
```
cd ./fixedwing-airtrack
```
### 编译
```
colcon build
```
### source
```
source install/setup.bash
```
### 启动KCF追踪GUI节点
```
ros2 launch trackergui trackergui
```
### 启动飞行控制节点
```
ros2 launch planner planner
```

## ROS2 基本操作
### 创建软件包
```
ros2 pkg create --build-type ament_python <package_name>
```
### 清理编译中间文件
```
rm -rf build install log
```