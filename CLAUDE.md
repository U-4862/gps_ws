# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 构建与运行

```bash
# 构建 (从工作空间根目录)
colcon build --packages-select gps

# 运行节点
ros2 run gps gps_node --ros-args \
  -p motion_port:=/dev/ttyUSB0 \
  -p imu_topic:=/livox/imu \
  -p tick_period_ms:=50

# 只做编译检查不完整构建
colcon build --packages-select gps --cmake-args -DCMAKE_CXX_COMPILE_COMMANDS=ON
```

## 项目架构

这是一个 **ROS 2 Humble** 工作空间，通过串口向机器人底盘发送运动指令。机器人的运动逻辑由 **BehaviorTree.CPP** 行为树驱动。

### 核心组件

- **`gps_node.cpp`** — 主程序入口。初始化 ROS 节点、串口、`SensorNode`（TF+IMU），注册行为树节点，进入 `tickOnce()` 主循环
- **`usart.hpp/usart.cpp`** — 串口封装库 (`SerialPort`)，提供 `writeExact()` 和 `readSome()` 两个核心接口，自带互斥锁线程安全
- **`tree.xml`** — 行为树定义文件，运行时从包的 share 目录加载

### 通信协议

运动指令通过 `Pose2D` 结构体经串口发送（4 字节打包）：
```
uint8_t header  // 0x0F
int8_t  x       // 前进/后退线速度
int8_t  y       // (未使用)
int8_t  z       // 左转/右转角速度(正=左转,负=右转)
```

### 行为树节点

所有运动节点继承自 `TimedVelocityAction`（BT::StatefulActionNode），内置 "超时即停止" 逻辑：
- `MoveForward` — x=1, z=0
- `TurnLeft` — x=0, z=1
- `TurnRight` — x=0, z=-1

每个节点通过 `duration_ms` 端口控制执行时长（默认 260ms）。

### AppContext

`AppContext` 结构体聚合全局共享资源（串口指针、SensorNode、logger），通过 `BT::Blackboard` 注入行为树上下文。

### SensorNode（TF + IMU）

`SensorNode` 同时订阅 `/tf` 和 `/livox/imu`（可通过 `imu_topic` 参数配置）两个话题：

- **TF 数据** — 实时缓存当前 x/y/z 坐标，供行为树条件节点通过 `currentX/Y/Z()` 查询
- **IMU 数据** — `ImuData` 结构体包含完整的姿态四元数 `(ori_x, ori_y, ori_z, ori_w)`、角速度 `(ang_vel_x/y/z)` 和线加速度 `(lin_acc_x/y/z)`，通过 `imuData()` 获取副本
- TF 与 IMU 各自持有独立的 mutex，互不阻塞
