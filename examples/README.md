# Qt 例程：从零到 ROS2 面板

这套例程配合 `gps_ws` 讲解 Qt5。每个例子都能单独编译运行，按编号递进。

## 环境

```bash
sudo apt install qtbase5-dev cmake g++     # 本机已装好
```

## 编译（纯 Qt，5 个例程一起编）

```bash
cd ~/projects/projects/gps_ws/examples/qt_basics
   cmake -S . -B build
cmake --build build -j4
```

## 运行（必须在有桌面的终端里，SSH 里没有 DISPLAY 会报错）

```bash
./build/hello       # 例程 1：最小窗口
./build/button      # 例程 2：按钮 + 信号槽
./build/clock       # 例程 3：QTimer 每秒刷新
./build/worker      # 例程 4：后台线程 + 跨线程信号
./build/qprocess    # 例程 5：QProcess 起命令、读输出
```

## 例程 6：ROS2 位姿面板

```bash
source /opt/ros/humble/setup.bash
cd ~/projects/projects/gps_ws/examples/qt_ros_demo
cmake -S . -B build
cmake --build build -j4
./build/qt_ros_demo
```

有 `/Odometry` 发布时会实时显示 x/y；没有则显示“等待 /Odometry ...”。

## 三个核心概念

1. **事件循环**：`app.exec()` 之后程序不往下走，所有点击/定时器/网络/进程输出都变成回调。
   所以界面代码里不要写 `sleep()`、不要写死循环等数据。
2. **控件树 + 布局**：`new QLabel(parent)` 把控件挂到父对象上，父对象析构时自动 `delete`；
   `QVBoxLayout`/`QHBoxLayout` 负责位置和随窗口缩放重排。
3. **信号槽**：`connect(sender, &Sender::signal, receiver, &Receiver::slot)`。
   槽可以是成员函数，也可以是 lambda。跨线程时 Qt 自动改用队列连接，槽在接收者所在线程执行。

## 最容易踩的坑

- 类里写了 `Q_OBJECT`，CMake 必须开 `CMAKE_AUTOMOC ON`；类写在 `.cpp` 里时文件末尾要
  `#include "xxx.moc"`，否则链接报 `undefined reference to vtable`。
- 耗时逻辑（ROS 的 `spin`、串口阻塞读、大计算）不能放界面线程，否则窗口“未响应”。
- 跨线程只通过信号传数据，不要在子线程里直接 `label->setText()`。
- `QProcess` 的 `terminate()` 只发给直接子进程；像 `ros2 launch` 这种会再拉起节点的，
  要自己处理进程组，不然关掉界面后节点可能还在跑。
