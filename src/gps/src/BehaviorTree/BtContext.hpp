#pragma once

#include <memory>
#include <utility>
#include <rclcpp/logger.hpp>

/**
 * @brief 行为树的依赖注入载体（骨架，当前尚未接入 CMake / gps_node）
 * @purpose 把 BT 节点需要的共享依赖聚成一处，用构造函数注入；
 *          SerialPort / Sensor 的具体定义只在 .cpp 里 include，
 *          本头文件仅做前向声明，避免重复包含 usart.hpp 与传感器消息。
 */

class SerialPort;
class Sensor;

/// 部署期常量：构造后不再变化
struct RunConfig
{
    int line {1};
    int kfs_loc {1};
};

class BtContext
{
public:
    BtContext(rclcpp::Logger logger,
              std::shared_ptr<Sensor> sensor,
              RunConfig config)
        : logger_(std::move(logger)),
          sensor_(std::move(sensor)),
          cfg_(config)
    {}

    const rclcpp::Logger& logger() const { return logger_; }
    Sensor& sensor() const { return *sensor_; }
    const RunConfig& config() const { return cfg_; }

private:
    rclcpp::Logger logger_;
    std::shared_ptr<Sensor> sensor_;
    RunConfig cfg_;
};
