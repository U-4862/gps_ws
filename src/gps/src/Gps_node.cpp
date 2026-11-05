#include "Chassis/SerialChassis.hpp"
#include "Nav/PurePursuit.hpp"
#include "Sensor/Sensor.hpp"
#include "Map/map.h"

#include <cerrno>
#include <chrono>
#include <thread>
#include <vector>


inline const std::vector<Location> tst_path_1 {
    {0.0f,0.1f},
    {0.1f , 0.1f},
    {0.3f , 0.1f},
    {2.0f, 1.0f}
};

using chassis_protocol::Frame;
using chassis_protocol::kRadarFrameLen;


SerialChassis serial_tst_chassis_1("/dev/ttyUSB0" , 115200);
PurePursuit pp(0.3f, 1.0f);

bool path_test_1(const PurePursuit::State & current_state)
{
    Command cmd{};
    cmd = pp.compute(current_state,tst_path_1,0.15);
    if(cmd.arrived){
        serial_tst_chassis_1.emergency_stop();
        return true;
    }
    serial_tst_chassis_1.set_velocity(cmd.msg);
    return false;  
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto sensor = std::make_shared<Sensor>("/odom_map", "/livox/imu");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(sensor);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);

    while (rclcpp::ok()) {
        exec.spin_some(std::chrono::milliseconds(2));      // ★ 让回调跑

        const auto snap = sensor->lastFrame();
        if (!snap) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }

        const int64_t age_ms = (sensor->now().nanoseconds() - snap->pose_stamp_ns) / 1'000'000;
        if (age_ms > 200) {                                 // 看门狗
            serial_tst_chassis_1.emergency_stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        if (std::chrono::steady_clock::now() > deadline) break;

        PurePursuit::State s;
        s.x     = static_cast<float>(snap->pose.x);
        s.y     = static_cast<float>(snap->pose.y);
        s.theta = static_cast<float>(sensor->currentYaw());

        if (path_test_1(s)) break;                          // 到达
        std::this_thread::sleep_for(std::chrono::milliseconds(50));   // 20 Hz
    }

    serial_tst_chassis_1.emergency_stop();
    rclcpp::shutdown();
    return 0;
}