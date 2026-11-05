# AGENTS.md

This file provides guidance to Codex (claude.ai/code) when working with code in this repository.


***不要修改我的Gps_node.cpp!!!!!!!!! ***


# explain
使用中文解释

任何我提出的想法先按第一性原理给我判断，再查找best practice
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

### Xbox 手柄监控（示例）

- 低延迟启动：`ros2 launch examples/xbox_joy_demo/joy_demo.launch.py`；GUI：`python3 examples/xbox_joy_demo/example.py`，按 `q` 退出。
- 需要 teleop 时用 `examples/xbox_joy_demo/teleop_xbox.launch.py`；不要和系统 `teleop_twist_joy` launch 同时跑，避免 `/joy` 双发布者。
- 硬件：Xbox Series X Controller（蓝牙，USB ID `045E:0B13`，`/dev/input/js0`，input name `bluez-hog-device`，uniq `ac:8e:bd:3c:ef:fc`）。
- `/joy`：8 轴（`jstest` 名称 `X,Y,Z,Rz,Gas,Brake,Hat0X,Hat0Y`）、15 键（`BtnA,B,C,X,Y,Z,TL,TR,TL2,TR2,Select,Start,Mode,ThumbL,ThumbR`）。
- `joy_node`：`device_id=0`、`deadzone=0.05`、`autorepeat_rate=100.0`、`coalesce_interval_ms=1`；实测约 54 Hz，间隔 10–21 ms。
- `teleop_xbox.launch.py` 用 `teleop_twist_joy/config/xbox.config.yaml`：`axis_linear.x=1`、`axis_angular.yaw=0`，scale 0.7/0.4，turbo 1.5，`enable_button=2`，`enable_turbo_button=5`，`publish_stamped_twist=False`。
- GUI 方向：`STICK_X_SIGN=-1.0`、`STICK_Y_SIGN=-1.0`；60 FPS，按 `q` 退出。

### 先验地图保存（FAST_LIO → PCD）

```bash
# 用本仓库的配置覆盖 map_file_path（默认写 /home/w/slam_ws/maps/prior_map.pcd）
ros2 launch fast_lio mapping.launch.py \
  config_path:=$HOME/projects/projects/gps_ws/config \
  config_file:=fastlio_mid360.yaml

# 建完图后保存（会校验文件并打印点数）
scripts/save-map.sh
scripts/save-map.sh --dry-run   # 只看会存到哪
```

关键事实（踩过的坑）：

- **Ctrl-C 不会保存地图**：`FAST_LIO_ROS2/src/laserMapping.cpp` 里累积
  `pcl_wait_save` 的整段代码被注释掉了，`main()` 退出时
  `if (pcl_wait_save->size() > 0 && pcd_save_en)` 永远不成立。
  上游 README 说会生成 `PCD/scans.pcd`，这份代码不是。
- 唯一落盘入口是 **`/map_save` 服务**（`std_srvs/srv/Trigger`）→
  `save_to_pcd()` → 写参数 `map_file_path`。
- `map_file_path` **必须绝对路径**。默认 `"./test.pcd"` 相对的是
  节点进程的当前目录（`ros2 launch` 时 = 你敲命令的目录）。
- 服务**不检查** `writeBinary` 返回值，写失败照样回 `success=True,
  message='Map saved.'`（静默失败），所以要用 `scripts/save-map.sh` 复核文件。
- 存的是 `pcl_wait_pub`，仅在 `map_en: true` 时按 1Hz 累积；且
  `dense_publish_en: false` 时存下来的是 0.5m 体素降采样图（更稀但更小）。

## 项目架构

这是一个 **ROS 2 Humble** 工作空间，通过串口向机器人底盘发送运动指令。机器人的运动逻辑由 **BehaviorTree.CPP** 行为树驱动。

### 核心组件

- **`gps_node.cpp`** — 主程序入口。初始化 ROS 节点、串口、`SensorNode`（Odometry+IMU），注册行为树节点，进入 `tickOnce()` 主循环
- **`usart.hpp/usart.cpp`** — 串口封装库 (`SerialPort`)，提供 `writeExact()` 和 `readSome()` 两个核心接口，自带互斥锁线程安全
- **`Map/map.h`** — 地图数据结构，包含通信协议枚举、`Location` 结构体和 `point_map` 点位地图（`Pose2D` 仅存历史，协议已弃用）
- **`tree.xml`** — 行为树定义文件，运行时从包的 share 目录加载

### 通信协议

> **Pose2D 协议已完全弃用。** 8 字节 `Pose2D`（header `0x0F` + x/y/z/pn/grip/up/down）
> 是旧下位机的运动帧，**不再是目标协议**：不要按它新增任何代码、行为树节点或
> 指令常量。`gps_node.cpp` 里存量的 `writeExact(&Pose2D)` 调用点属待迁移债务，
> 迁移方向见下面的新协议。

当前目标协议：**30 字节定长帧 `radar_msg`**（`Chassis/ChassisMsg.hpp` 定义结构体，
`Chassis/SerialChassis.hpp` 的 `chassis_protocol` 负责编解码）：

```
[0]      0xFF                帧头
[1..4]   x   (float 小端)     摇杆量，满量程 ±150 ↔ ±3.75 m/s
[5..8]   y   (float 小端)
[9..12]  z   (float 小端)
[13..16] translation_distance (float 小端, mm)
[17..20] high_err             (float 小端, mm)
[21..24] front_err            (float 小端, mm)
[25]     route_flag
[26]     transmit_block_flag
[27]     error_flag
[28]     校验和 = (Σ[1..27]) & 0xFF
[29]     0xFE                帧尾
```

坑：`radar_msg` 结构体 32 字节 ≠ 帧 30 字节；三个标志位在结构体里的顺序
（error / route / transmit）和协议顺序（route / transmit / error）**不同**，
禁止 `memcpy(&msg)` 直发；校验是**累加和**，不是 CRC-16。量纲换算用
`world_vel_to_stick()` / `stick_to_world_vel()`。

**下行只有「发」这一条通路，下位机串口回传（上行帧）尚未实现**，见「待办」。

`kStop` / `kForward` / `kTurnLeft` / `kTurnRight` / `kGrip` / `kTurn90Left` /
`kTurn90Right` 这些 `Pose2D` 常量（`gps_node.cpp` 顶部）属于已弃用协议，
只在存量代码里存在，新代码不要引用。

### 行为树节点

所有运动节点继承自 `TimedVelocityAction`（BT::StatefulActionNode），内置 "超时即停止" 逻辑：

- **`MoveForward`** — 继承 `TimedVelocityAction`，发送 x=1 的前进指令
- **`MoveToLocation`** — 继承 `TimedVelocityAction`，通过 `location` 端口接收目标点位名称，从 `point_map` 查找坐标，使用五阶段导航（TURN_X → DRIVE_X → TURN_Y → DRIVE_Y → DONE），包含 25 秒全局超时
- **`TurnLeft`** — 继承 `MoveToLocation`，重写 `onRunning()` 使用 `turnToFace()` 转向 M_PI_2（90° 左转）
- **`TurnRight`** — 继承 `MoveToLocation`，重写 `onRunning()` 使用 `turnToFace()` 转向 -M_PI_2（90° 右转）
- **`AdjustPosition`** — 独立节点（非 `TimedVelocityAction` 子类），将机器人旋转至绝对偏航角目标。端口：`target_yaw`（弧度，默认0.0）、`timeout_ms`（默认5000）。使用 PD 控制器：P 项基于 Odometry 偏航角误差，D 项通过 IMU `ang_vel_z` 阻尼，速度钳位：z ∈ [1, 20]，角度容差 0.1 rad。
  - 当前注意事项：`target_yaw_` 是成员变量，读取端口失败时可能残留上次目标；到达判据只看角度、不看角速度，可能在高速穿过目标时提前 SUCCESS；最小转速固定为 1，可能在容差边界抖振；Odometry yaw 与 IMU `ang_vel_z` 的坐标系/符号必须确认一致，否则 D 项会变成正反馈。

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

### 地图 (Map/map.h)

`point_map` 是一个 `std::unordered_map<std::string, Location>`，存储命名点位及其 (x, y) 坐标：
- `restart_point` (0.0, 0.0)
- `weapon_chair` (1.0, 0.5)
- `home` (0.4, 0.0)
- `test1` (1.0, 0.8)
- `test2` (2.4, -1.5)

### AppContext

`AppContext` 结构体聚合全局共享资源（`motion_port` 串口指针、`sensor_node`、`link_monitor`、
`logger`、`line`/`kfs_loc` 配置）。实际注入方式是**构造函数注入**：`registerNodes()` 里每个
`registerBuilder` 用 lambda 捕获 context 再传给节点构造函数（BehaviorTree.CPP 官方推荐做法）。
`main()` 里的 `blackboard->set("app_context", context)` 是死代码，无人读取。

### SensorNode（Odometry + IMU）

`SensorNode` 订阅 Odometry（可通过 `odom_topic` 参数配置，默认 `/odom_corrected`）和雷达 IMU（可通过 `imu_topic` 参数配置，默认 `/livox/imu`）：

- **Odometry 数据** — 实时缓存当前 x/y/z 坐标和姿态四元数，通过 `currentX/Y/Z()`、`currentYaw()` 和 `poseData()` 查询
- **PoseData 结构体** — 包含位置 (x,y,z)、姿态四元数 (ori_x, ori_y, ori_z, ori_w)、线速度 (lin_vel_x/y/z) 和角速度 (ang_vel_x/y/z)
- **雷达 IMU 数据** — `ImuData` 结构体包含完整的姿态四元数 `(ori_x, ori_y, ori_z, ori_w)`、角速度 `(ang_vel_x/y/z)` 和线加速度 `(lin_acc_x/y/z)`，通过 `imuData()` 获取副本
- Odometry 与 IMU 各自持有独立的 mutex，互不阻塞

## 待办

- **新协议下位机串口回传待实现（当前阻塞项）**
  - 现状：新协议（30 字节 `radar_msg`）**只有下行**。`SerialChassis::get_wheel_speed()`
    恒返回 0；链路健康（`is_linked`）也没有新协议的上行帧承载。
  - 待定（必须先和下位机对齐）：上行帧 header、帧长、payload 字段、CRC 位置与字节序、
    CRC 覆盖范围、心跳/回传周期。
  - 影响：`is_linked` 判定、轮速反馈、急停确认都无法闭环。这几项定下来之前，
    不要按旧 `0x0E` / `Signal2D` 的形状硬编码新的解析逻辑。
  - 归属约定：串口读写由 `SerialChassis` 独占（一条 tty 只能有一个读者），对外以
    `ILinkStatus`（`isLinked()` / `lastRecvNs()` / `lastIsLinked()`）暴露；BT 节点
    不直接持有串口或具体底盘类。`motion_port` 与 `LinkMonitor` 随 Pose2D 一起退场。

- **`SerialPort` 读写锁要分离**：`readSome` 与 `writeExact` 共用同一把 `mutex_`，
  且 `VTIME=2` 时一次读最多持锁阻塞 200ms，而 tick 是 50ms → 读线程会拖慢运动指令。
  修法：读改非阻塞（`VMIN=0,VTIME=0` + `poll()`）或读写分锁。

- **下位机串口改为变长帧 + CRC 解析**（`LinkMonitor::run()` 现在是定长 `Signal2D`(2字节)
  按位读取，属旧协议）
  - 目标：同步 header → 按 header 查帧长 → 收满整帧 → 校验 CRC → 提取字段，校验失败丢 1 字节重同步
  - 已确认：按 header 区分帧长；旧协议只有 `0x0E` 心跳帧；新协议上行帧待定（见上一条）
  - 待商量：0x0E 帧除 `is_linked` 外的完整 payload 字段、CRC 在帧内的字节序（低字节前/高字节前）、CRC 覆盖范围、是否会新增其它 header
  - 实现时涉及：`Map/map.h` 的 `Signal2D`、`gps_node.cpp` 的 `LinkMonitor::run()`；
    CRC-16/MODBUS 已内联在 `Chassis/SerialChassis.hpp` 的 `chassis_protocol::crc16()`
    （`SerialPort/crc16.hpp` 已移除）

