#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

/***
 *    语义尚未统一的雷达数据类型
 *
 *
 */
struct ImuData
{
    double ori_x {0.0}, ori_y {0.0}, ori_z {0.0}, ori_w {1.0};
    double ang_vel_x {0.0}, ang_vel_y {0.0}, ang_vel_z {0.0};
    double lin_acc_x {0.0}, lin_acc_y {0.0}, lin_acc_z {0.0};
};

struct PoseData
{
    double x {0.0}, y {0.0}, z {0.0};
    double ori_x {0.0}, ori_y {0.0}, ori_z {0.0}, ori_w {1.0};
    double lin_vel_x {0.0}, lin_vel_y {0.0}, lin_vel_z {0.0};
    double ang_vel_x {0.0}, ang_vel_y {0.0}, ang_vel_z {0.0};
};

struct SensorSnapshot {
    PoseData pose;
    ImuData imu;
    int64_t pose_stamp_ns{0};
    int64_t imu_stamp_ns{0};
};


/***
 * 将 ros 驱动的感知层独立封装出来
 */

class Sensor : public rclcpp::Node
{
public:
    explicit Sensor(
        const std::string& radar_odom = "/Odometry",
        const std::string& radar_imu = "/livox/imu");

    double currentX() const;
    double currentY() const;
    double currentZ() const;
    double currentYaw() const;

    PoseData poseData() const;
    ImuData imuData() const;

    // 这三个实现在 cpp 里
    std::optional<SensorSnapshot> lastFrame() const;

    bool hasData() const;

    bool isStale(std::chrono::milliseconds timeout) const;

private:
    void onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onRadarImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg);

    static PoseData toPoseData(const nav_msgs::msg::Odometry& src);
    static ImuData toImuData(const sensor_msgs::msg::Imu& src);

    // builtin_interfaces::msg::Time -> 纳秒
    static int64_t stampToNs(const builtin_interfaces::msg::Time& stamp);

    // Odometry
    mutable std::mutex odom_mutex_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr radar_odom_sub_;
    PoseData pose_;
    int64_t pose_stamp_ns_{0};

    // 雷达 IMU
    mutable std::mutex radar_imu_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr radar_imu_sub_;
    ImuData radar_imu_;
    int64_t imu_stamp_ns_{0};
};  
