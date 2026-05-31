# Build Summary — 2026-05-31

## 改动概览

### 1. `/tf` → `/Odometry` 切换 (SensorNode)
- 订阅从 `tf2_msgs::msg::TFMessage` 改为 `nav_msgs::msg::Odometry`
- 依赖从 `tf2_msgs` 改为 `nav_msgs` (CMakeLists.txt + package.xml)
- 新增 `PoseData` 结构体，存储位置、四元数朝向、线速度、角速度
- `currentX/Y/Z()` 保持不变，新增 `poseData()` 返回完整 PoseData
- 修复了之前从 `/tf` 遍历 transforms 数组只取最后一个的不确定性

### 2. `Pose2D` 精简
- 删除 `nextPose1`、`nextPose2` 两个指针成员
- `sizeof(Pose2D)` 从 32 字节降为 4 字节
- `writeExact` 只发送协议需要的 header + x + y + z
- 所有子类初始化从 `{0x0f, 1, 0, 0, nullptr, nullptr}` 简化为 `{0x0f, 1, 0, 0}`

### 3. `TimedVelocityAction` 访问控制
- `context_`、`command_`、`deadline_` 从 `private` 移入 `protected`
- 子类现在可以在 `onRunning()` 中访问传感器、修改指令、判断时间

### 4. `TurnLeft::onRunning()` 三阶段逻辑
- 修复 `sendCommand(commend_)` 拼写错误 → `command_`
- 修复 `return RUNNING` 导致后续 phase 不可达 → `else if` 链
- 修复孤儿代码（`stopRobot`/`RCLCPP_INFO`/`return SUCCESS` 在函数体外）
- 三阶段：>1500ms 高速启动(3) → 400-1500ms 正常转弯(1) → <400ms 停止(0)

### 5. 基类 `onRunning()` 补全
- 修复缺少的 `}` 导致类结构崩溃

### 6. `tree.xml` 简化
- 行为树测试序列简化为 MoveForward(2000ms) → TurnLeft(2000ms)

## 构建结果
- `colcon build --packages-select gps`: 通过
