# test_chassis：编译修复与实发使用说明

> 一套用于 **30 字节雷达/对接协议** 的测试底盘实现（`TestChassis`）和自测程序（`test_chassis`）。
> 默认纯离线比对，不碰硬件；加 `--live` 才真的发串口。
>
> 本文记录：它是什么、之前为什么编不过、现在怎么用、串口打不开怎么排查。

---

## 1. 背景：为什么会有这个东西

`gps` 包原来只有一条下发链路：`gps_node` 用 `Pose2D`（8 字节）跟老底盘通信。
现在要接的下位机换了一套 **30 字节协议**，报文样例放在 `Docs/xcom_测试报文.txt`。

新老协议差别不小，最容易踩的两个坑：

| 坑 | 说明 |
|---|---|
| **不能 memcpy 结构体** | `radar_msg` 是 32 字节，协议帧只有 30 字节；字段顺序也不一样 |
| **标志位顺序是错开的** | 结构体是 `error / route / transmit_block`，协议是 `route / transmit_block / error` |

所以新增了：

- `src/gps/src/Test/TestChassis.*` —— 实现 `IChassis`，负责把 `radar_msg` 编码成 30 字节帧并写串口
- `src/gps/src/Test/test.cpp` —— 自测程序，用 `Docs/xcom_测试报文.txt` 的现成报文做**逐字节比对**
- `scripts/build-test.sh` —— 只编 `test_chassis` 这一个 target 的构建脚本

**为什么需要单独的构建脚本**：
`colcon build --packages-select gps` 会连带编译 `gps_node.cpp`，而它目前因 `map/map.h` 里
`GraphEdge` 未定义而编译失败，连带 `test_chassis` 也出不来。脚本用独立 cmake 目录
（`.build-test/`）只编这一个 target，和 colcon 的 `build/` 完全隔离，互不干扰。

---

## 2. 这次修掉的问题

### 2.1 `#include` 写进了 `main()` 函数体 —— 一串极具误导性的报错

**现象**：编译 `test_chassis` 时，报错全部指向标准库头文件，看起来像编译器/标准库坏了：

```
In file included from /usr/include/c++/11/utility:68,
                 from .../TestChassis.hpp:28,
                 from .../test.cpp:14:
/usr/include/c++/11/bits/refwrap.h:43:15: error: expected '=' before '__attribute__'
   43 | namespace std _GLIBCXX_VISIBILITY(default)
      |               ^~~~~~~~~~~~~~~~~~~
```

**真正的原因**：`test.cpp` 里为了临时开一段调试代码，把 include 写在了函数体内：

```cpp
int main() {
    ...
    #include <chrono>     // ← 在函数里面！
    #include <thread>
```

标准库头文件里有 `namespace std { ... }` 这种定义，而 **C++ 不允许在函数体内定义命名空间**。
于是编译器解析到 `namespace std _GLIBCXX_VISIBILITY(default)` 时完全无法理解，报出一堆
`expected '='` / `expected ';'`，位置却都在 `/usr/include/c++/` 下，很容易误判成环境问题。

**修复**：把 `<chrono>`、`<thread>` 移到文件头部的 include 区。

> 经验：**报错位置在系统头文件里，不等于问题在系统头文件里**。看第一个 `In file included from`
> 的链条，找到"是你的哪个文件引起它被包含的"，基本就能定位。

### 2.2 实发串口的代码默认启用、设备名硬编码

原代码里"想真发就打开注释"的那段，**注释是打开状态的**，与文件头写的"不需要接硬件"自相矛盾；
而且设备写死 `/dev/ttyUSB0`，随便跑一下自测就会尝试打开串口。

**修复**：改成显式开关 —— 默认只跑离线比对，加 `--live` 才进实发分支，设备名可传参。

### 2.3 `SerialPort` 构造不会打开端口（实发模式根本打不开串口）

`SerialPort` 构造函数只保存参数，**不调用 `openPort()`**（`gps_node.cpp` 里也是显式调用的）。
而 `TestChassis` 构造时没调，导致：

- `is_open()` 永远返回 `false`
- `last_error()` 永远是空字符串

也就是说：实发模式**必然失败**，且失败信息还是空的，极难排查。

**修复**：`TestChassis` 构造函数里补上 `serial_.openPort()`，并把失败原因留在 `last_error()`。

### 2.4 构建脚本：参数进不去、`--help` 会错位

| 问题 | 修复 |
|---|---|
| 脚本只认白名单参数，`--live` 直接报"未知参数"，没法一条命令构建 + 实发 | 支持 `--` 之后**原样透传**给 `test_chassis`，并隐含 `--run` |
| 遇到未知参数只甩一句"未知参数"，用户不知道该怎么传 | 额外打印提示和正确示例 |
| `--help` 用 `sed -n '2,26p'` 写死行号，往头注释加几行就会把代码打出来 | 改用 `awk` 打印到第一行非注释为止，随便改注释都不会串 |
| 退出码 1/2 含义不同，脚本却笼统地说"自测有失败项" | 按 0/1/2 分别给出提示 |

---

## 3. 怎么用

### 3.1 构建 + 离线自测（不碰硬件）

```bash
# 方式一：脚本（推荐，自动构建后运行）
./scripts/build-test.sh --run

# 方式二：直接跑产物
./.build-test/test_chassis
```

通过的输出长这样：

```
====== 结果：15 通过，0 失败 ======
```

### 3.2 实发串口

```bash
# 脚本透传（-- 之后的参数原样交给程序）
./scripts/build-test.sh -- --live /dev/ttyUSB0

# 或者直接跑二进制
./.build-test/test_chassis --live /dev/ttyUSB0
./install/gps/lib/gps/test_chassis --live /dev/ttyUSB0   # --install 之后
```

参数：

```bash
test_chassis --live <设备> [摇杆量] [秒数]
```

| 参数 | 默认 | 说明 |
|---|---|---|
| 设备 | `/dev/ttyUSB0` | 串口路径，也支持 `/dev/serial/by-id/...` |
| 摇杆量 | 20（≈0.5 m/s） | 协议量纲 ±150，超出会先提示再按边界算 |
| 秒数 | 3 | 按 20 Hz 发送，即默认发 60 帧 |

实发会做的事：

1. 先做**设备预检**（在不在、有没有读写权限），不通过就直接给排查提示并退出
2. 以 115200 8N1 打开串口
3. 以 20 Hz 连发 `秒数 × 20` 帧，只朝 +X 方向（安全起见不转弯）
4. **末尾补一帧全零停车帧**（`emergency_stop()`）
5. 打印首帧十六进制，可直接和 `Docs/xcom_测试报文.txt` 对照

示例输出：

```
=== 实发模式 /dev/ttyUSB0（20 帧/秒，3 秒，摇杆量 20.0 ≈ 目标 0.50 m/s）===
  ✓ 已打开 /dev/ttyUSB0（115200 8N1）
  已发 60 帧：成功 60，失败 0；末尾已补一帧停车
  首帧（可直接和 Docs/xcom_测试报文.txt 对照）：FF 00 00 A0 41 ...
实发结束
```

> ⚠️ `--live` 会让底盘**真的动起来**。接真机时先确认周围没有障碍、有人能急停。

### 3.3 退出码约定

| 退出码 | 含义 |
|---|---|
| `0` | 自测通过，实发（如有）全部成功 |
| `1` | 自测有失败项（编码和文档对不上，别接硬件） |
| `2` | 实发阶段失败：设备不存在 / 无权限 / 打不开 / 写失败 |

脚本会原样透传这个退出码，方便 CI 或别的脚本区分"协议算错了"和"串口没通"。

---

## 4. 串口打不开怎么排查

程序会先做预检，输出里已经带了大部分答案。常见的三种：

**① 设备不存在**

```
✗ 串口设备不存在：/dev/ttyUSB0
  系统返回：No such file or directory
  常见原因：USB-TTL 没插好 / 线只供电不传数据 / 适配器掉线后节点消失
  但系统里有这些串口，可能只是编号变了：
    /dev/ttyUSB1
  换一个试试： --live /dev/ttyUSB1
  稳定设备名（插拔后编号会变，推荐用它）：
    /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

**② 设备在，但没有权限** —— 当前终端不在 `dialout` 组，或加组后没重新登录：

```bash
id -Gn | grep dialout              # 看当前终端有没有
sudo usermod -aG dialout "$USER"   # 加组（只需一次）
newgrp dialout                     # 或在当前终端临时生效
```

**③ 设备在、权限也有，还是打不开** —— 多半被别的进程占用（比如 `gps_node` 还在跑）：

```bash
fuser -v /dev/ttyUSB0
```

其他有用的命令：

```bash
journalctl -k | grep -iE 'ch34|ttyUSB' | tail    # 看 USB 串口驱动有没有报错
lsusb -t | grep -i -E 'ch34|cp210|ftdi'          # 看适配器识别情况
```

---

## 5. 离线自测覆盖了什么（15 项）

| 分组 | 项数 | 内容 |
|---|---|---|
| 文档现成报文比对 | 8 | 停车 / ±X / ±Y 满速 / 原地左右转 / 慢速 +X，逐字节比对整帧 |
| 标志位顺序 | 1 | 验证 `[25]=route`、`[26]=transmit_block`、`[27]=error` 的错位映射 |
| 校验与帧完整性 | 2 | `frame_valid()` 正常通过；篡改一字节后能被校验发现 |
| 量纲换算 | 4 | m/s ↔ 摇杆量（×40），0 / 0.5 / 1.25 / 3.75 m/s |

帧结构（30 字节）：

```
[0]      0xFF                     帧头
[1..4]   x            float 小端   摇杆量 ±150
[5..8]   y            float 小端
[9..12]  z            float 小端   角速度
[13..16] translation_distance      对接距离 mm
[17..20] high_err                  高度误差 mm
[21..24] front_err                 前后误差 mm
[25]     route_flag
[26]     transmit_block_flag
[27]     error_flag
[28]     校验和 = Σ[1..27] & 0xFF
[29]     0xFE                     帧尾
```

---

## 6. 验证记录

```bash
$ ./scripts/build-test.sh --run
[100%] Built target test_chassis
  ✓ /home/w/projects/projects/gps_ws/.build-test/test_chassis
====== 结果：15 通过，0 失败 ======
✓ 自测全部通过                      # exit 0

$ ./.build-test/test_chassis --live /dev/ttyUSB9
✗ 串口设备不存在：/dev/ttyUSB9
  ...（列出系统现有串口和排查命令）
                                   # exit 2

$ ./scripts/build-test.sh --live              # 忘了用 -- 分隔
未知参数: --live（用 --help 看用法）
提示：要传参数给 test_chassis，用 -- 分隔，例如：
      ./scripts/build-test.sh --run -- --live /dev/ttyUSB0
                                   # exit 2
```

---

## 7. 遗留问题与注意

1. **`coordinate_transform_world_to_robot()` 目前是空实现**
   （`ChassisMsg.hpp`，函数体只 `(void)` 掉参数）。当前没有任何调用者，不会出错；
   但以后要用世界系→机器人系变换时，得先把真正的实现补上，否则会静默返回全零。

2. **`gps_node` 仍然编不过**（`map/map.h` 的 `GraphEdge` 未定义），所以
   `colcon build --packages-select gps` 依旧会失败。`test_chassis` 靠 `scripts/build-test.sh`
   独立构建绕开。等 `GraphEdge` 修好后，建议回到 `colcon build` 的统一流程。

3. **`.vscode/settings.json`、`log/latest_build` 属于本地/构建产物**，没有纳入提交。

---

## 8. 文件清单

| 文件 | 作用 |
|---|---|
| `src/gps/src/Test/TestChassis.hpp` | 公开接口：帧长/帧头帧尾、量纲换算、`radar_msg` → 帧 的函数声明 |
| `src/gps/src/Test/TestChassis.cpp` | 字节偏移表（匿名 namespace，编码细节）、`encode()` / `checksum_of()` / `frame_valid()` / 串口发送 |
| `src/gps/src/Test/test.cpp` | 自测 + 实发模式入口 |
| `src/gps/src/Test/test.h` | Test 目录统一入口头文件 |
| `src/gps/src/Chassis/ChassisMsg.hpp` | 新增 `radar_msg` 结构体（32 字节，注意不是帧长） |
| `Docs/xcom_测试报文.txt` | 电控提供的报文样例，自测的期望值来源 |
| `scripts/build-test.sh` | 独立构建 + 运行脚本（支持 `--` 透传参数） |
