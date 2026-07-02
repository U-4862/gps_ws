# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# explain
使用中文解释

user memory means 

## 相关工作空间

- **slam_ws** (`/home/w/slam_ws`) — SLAM 建图和回环检测工作空间
  - 包含 `slam_bridge` 包，用于运行 slam_toolbox 和点云到激光扫描的转换
  - 地图保存目录：`/home/w/slam_ws/maps/`（目前为空）
  - 配置文件：`src/slam_bridge/config/slam_toolbox_params.yaml`



## 构建与运行

```bash
# 构建 (从工作空间根目录)
colcon build --packages-select gps

# 运行 SLAM 回环检测 (需先启动 livox 驱动和 FAST_LIO)
ros2 launch slam_bridge slam_bringup.launch.py

# 运行 gps 运动节点 (默认使用 slam 修正后的 /odom_corrected)
ros2 run gps gps_node --ros-args \
  -p motion_port:=/dev/ttyUSB0 \
  -p imu_topic:=/livox/imu \
  -p tick_period_ms:=50

# 如需降级使用原始 FAST_LIO /Odometry (无回环修正)
ros2 run gps gps_node --ros-args \
  -p odom_topic:=/Odometry \
  -p motion_port:=/dev/ttyUSB0

# 只做编译检查不完整构建
colcon build --packages-select gps --cmake-args -DCMAKE_CXX_COMPILE_COMMANDS=ON
```

## 项目架构

这是一个 **ROS 2 Humble** 工作空间，通过串口向机器人底盘发送运动指令。机器人的运动逻辑由 **BehaviorTree.CPP** 行为树驱动。

### 核心组件

- **`gps_node.cpp`** — 主程序入口。初始化 ROS 节点、串口、`SensorNode`（Odometry+IMU），注册行为树节点，进入 `tickOnce()` 主循环
- **`usart.hpp/usart.cpp`** — 串口封装库 (`SerialPort`)，提供 `writeExact()` 和 `readSome()` 两个核心接口，自带互斥锁线程安全
- **`map/map.h`** — 地图数据结构，包含通信协议枚举、`Pose2D` 结构体、`Location` 结构体和 `point_map` 点位地图
- **`tree.xml`** — 行为树定义文件，运行时从包的 share 目录加载

### 通信协议

运动指令通过 `Pose2D` 结构体经串口发送（8 字节打包）：
```
uint8_t header        // 0x0F
uint8_t x             // 前进/后退线速度
uint8_t y             // (未使用)
uint8_t z             // 左转/右转角速度(正=左转,负=右转)
uint8_t pn            // 正负号信号 (POS=0, NEG=1)
uint8_t grip_signal   // 抓取信号 (LOOSE=0, GRIPPED=1)
uint8_t up_signal     // 上升信号
uint8_t down_signal   // 下降信号
```

预定义的运动指令常量（定义在 `gps_node.cpp` 顶部）：
- `kStop` / `kForward` / `kTurnLeft` / `kTurnRight`
- `kGrip` — 抓取指令
- `kTurn90Left` / `kTurn90Right` — 90度旋转指令

### 行为树节点

所有运动节点继承自 `TimedVelocityAction`（BT::StatefulActionNode），内置 "超时即停止" 逻辑：

- **`MoveForward`** — 继承 `TimedVelocityAction`，发送 x=1 的前进指令
- **`MoveToLocation`** — 继承 `TimedVelocityAction`，通过 `location` 端口接收目标点位名称，从 `point_map` 查找坐标，使用五阶段导航（TURN_X → DRIVE_X → TURN_Y → DRIVE_Y → DONE），包含 25 秒全局超时
- **`TurnLeft`** — 继承 `MoveToLocation`，重写 `onRunning()` 使用 `turnToFace()` 转向 M_PI_2（90° 左转）
- **`TurnRight`** — 继承 `MoveToLocation`，重写 `onRunning()` 使用 `turnToFace()` 转向 -M_PI_2（90° 右转）

每个节点通过 `duration_ms` 端口控制执行时长（默认 260ms）。

### MoveToLocation 导航算法

五阶段顺序执行：
1. **TURN_X** — 使用 `turnToFace()` 转向目标 x 方向（0° 或 180°）
2. **DRIVE_X** — 使用 `driveToTarget()` 沿 x 轴移动直到距离 < 0.15m
3. **TURN_Y** — 使用 `turnToFace()` 转向目标 y 方向（±90°）
4. **DRIVE_Y** — 使用 `driveToTarget()` 沿 y 轴移动直到距离 < 0.15m
5. **DONE** — 停止，返回 SUCCESS

辅助方法：
- `quatToYaw()` — 四元数转偏航角
- `turnToFace(target_yaw, current_yaw, next_phase)` — 控制转向，容差 0.1 rad
- `driveToTarget(distance, next_phase)` — 比例控制驱动（P=10），速度钳位 ±20，容差 0.15m

### 地图 (map/map.h)

`point_map` 是一个 `std::unordered_map<std::string, Location>`，存储命名点位及其 (x, y) 坐标：
- `restart_point` (0.0, 0.0)
- `weapon_chair` (1.0, 0.5)
- `home` (0.4, 0.0)
- `test1` (1.0, 0.8)
- `test2` (2.4, -1.5)

### AppContext

`AppContext` 结构体聚合全局共享资源（串口指针、SensorNode、logger），通过 `BT::Blackboard` 注入行为树上下文。

### SensorNode（Odometry + IMU）

`SensorNode` 订阅 Odometry（可通过 `odom_topic` 参数配置，默认 `/odom_corrected`）和雷达 IMU（可通过 `imu_topic` 参数配置，默认 `/livox/imu`）：

- **Odometry 数据** — 实时缓存当前 x/y/z 坐标和姿态四元数，通过 `currentX/Y/Z()`、`currentYaw()` 和 `poseData()` 查询
- **PoseData 结构体** — 包含位置 (x,y,z)、姿态四元数 (ori_x, ori_y, ori_z, ori_w)、线速度 (lin_vel_x/y/z) 和角速度 (ang_vel_x/y/z)
- **雷达 IMU 数据** — `ImuData` 结构体包含完整的姿态四元数 `(ori_x, ori_y, ori_z, ori_w)`、角速度 `(ang_vel_x/y/z)` 和线加速度 `(lin_acc_x/y/z)`，通过 `imuData()` 获取副本
- Odometry 与 IMU 各自持有独立的 mutex，互不阻塞

