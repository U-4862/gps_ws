#include "Sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

Sensor::Sensor(const std::string& radar_odom, const std::string& radar_imu)
: rclcpp::Node("sensor_node")
{
    const auto qos = rclcpp::SensorDataQoS();

    radar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        radar_odom, qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            onOdometryReceived(std::move(msg));
        });

    radar_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        radar_imu, qos,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            onRadarImuReceived(std::move(msg));
        });

    RCLCPP_INFO(get_logger(),
        "Listening on %s, radar IMU: %s",
        radar_odom.c_str(), radar_imu.c_str());
}

double Sensor::currentX() const
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return pose_.x;
}

double Sensor::currentY() const
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return pose_.y;
}

double Sensor::currentZ() const
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return pose_.z;
}

double Sensor::currentYaw() const
{
    std::lock_guard<std::mutex> lock(odom_mutex_);

    const double siny =
        2.0 * (pose_.ori_w * pose_.ori_z + pose_.ori_x * pose_.ori_y);
    const double cosy =
        1.0 - 2.0 * (pose_.ori_y * pose_.ori_y + pose_.ori_z * pose_.ori_z);
    return std::atan2(siny, cosy);
}

PoseData Sensor::poseData() const
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return pose_;
}

ImuData Sensor::imuData() const
{
    std::lock_guard<std::mutex> lock(radar_imu_mutex_);
    return radar_imu_;
}

std::optional<SensorSnapshot> Sensor::lastFrame() const
{
    // 同时锁住两路，保证拷贝期间回调不会改数据。
    // 两路回调各自只拿一把锁，不会和这里形成环。
    std::scoped_lock lock(odom_mutex_, radar_imu_mutex_);

    SensorSnapshot snapshot;
    snapshot.pose = pose_;
    snapshot.pose_stamp_ns = pose_stamp_ns_;
    snapshot.imu = radar_imu_;
    snapshot.imu_stamp_ns = imu_stamp_ns_;

    // 两路都还没收到过数据，没有可用快照。
    if (snapshot.pose_stamp_ns == 0 && snapshot.imu_stamp_ns == 0) {
        return std::nullopt;
    }

    return snapshot;
}

bool Sensor::hasData() const
{
    std::scoped_lock lock(odom_mutex_, radar_imu_mutex_);
    return pose_stamp_ns_ != 0 || imu_stamp_ns_ != 0;
}

bool Sensor::isStale(std::chrono::milliseconds timeout) const
{
    std::scoped_lock lock(odom_mutex_, radar_imu_mutex_);

    // 任意一路没数据，都视为不可用。
    if (pose_stamp_ns_ == 0 || imu_stamp_ns_ == 0) {
        return true;
    }

    const int64_t now_ns = now().nanoseconds();

    // 用较旧的一路判断，避免新 IMU 掩盖旧 odom。
    const int64_t oldest_ns = std::min(pose_stamp_ns_, imu_stamp_ns_);
    const int64_t timeout_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count();

    return (now_ns - oldest_ns) > timeout_ns;
}

void Sensor::onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const PoseData data = toPoseData(*msg);

    int64_t stamp_ns = stampToNs(msg->header.stamp);
    if (stamp_ns == 0) {
        // 驱动没填 header.stamp 时，用本地接收时间兜底。
        stamp_ns = now().nanoseconds();
    }

    std::lock_guard<std::mutex> lock(odom_mutex_);
    pose_ = data;
    pose_stamp_ns_ = stamp_ns;
}

void Sensor::onRadarImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const ImuData data = toImuData(*msg);

    int64_t stamp_ns = stampToNs(msg->header.stamp);
    if (stamp_ns == 0) {
        stamp_ns = now().nanoseconds();
    }

    std::lock_guard<std::mutex> lock(radar_imu_mutex_);
    radar_imu_ = data;
    imu_stamp_ns_ = stamp_ns;
}

PoseData Sensor::toPoseData(const nav_msgs::msg::Odometry& src)
{
    PoseData dst;
    dst.x = src.pose.pose.position.x;
    dst.y = src.pose.pose.position.y;
    dst.z = src.pose.pose.position.z;
    dst.ori_x = src.pose.pose.orientation.x;
    dst.ori_y = src.pose.pose.orientation.y;
    dst.ori_z = src.pose.pose.orientation.z;
    dst.ori_w = src.pose.pose.orientation.w;
    dst.lin_vel_x = src.twist.twist.linear.x;
    dst.lin_vel_y = src.twist.twist.linear.y;
    dst.lin_vel_z = src.twist.twist.linear.z;
    dst.ang_vel_x = src.twist.twist.angular.x;
    dst.ang_vel_y = src.twist.twist.angular.y;
    dst.ang_vel_z = src.twist.twist.angular.z;
    return dst;
}

ImuData Sensor::toImuData(const sensor_msgs::msg::Imu& src)
{
    ImuData dst;
    dst.ori_x = src.orientation.x;
    dst.ori_y = src.orientation.y;
    dst.ori_z = src.orientation.z;
    dst.ori_w = src.orientation.w;
    dst.ang_vel_x = src.angular_velocity.x;
    dst.ang_vel_y = src.angular_velocity.y;
    dst.ang_vel_z = src.angular_velocity.z;
    dst.lin_acc_x = src.linear_acceleration.x;
    dst.lin_acc_y = src.linear_acceleration.y;
    dst.lin_acc_z = src.linear_acceleration.z;
    return dst;
}

int64_t Sensor::stampToNs(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL
         + static_cast<int64_t>(stamp.nanosec);
}
