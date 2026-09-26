#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "map/map.h"
#include "SerialPort/usart.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using std::placeholders::_1;
namespace chr = std::chrono;

inline constexpr Pose2D kLeft{0x0f, 0, 1, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::NORMAL , up_signal::NORMAL};
inline constexpr Pose2D kStop{0x0f, 0, 0, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::NORMAL , up_signal::NORMAL};
inline constexpr Pose2D kForward{0x0f, 1, 0, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::NORMAL, up_signal::NORMAL};
inline constexpr Pose2D kBack{0x0f, 1, 0, 0, pn_signal::NEG, grip_signal::LOOSE, up_signal::NORMAL, up_signal::NORMAL};
inline constexpr Pose2D kTurnLeft{0x0f, 0, 0, 1, pn_signal::POS, grip_signal::LOOSE, up_signal::NORMAL ,up_signal::NORMAL};
inline constexpr Pose2D kTurnRight{0x0f, 0, 0, 1, pn_signal::NEG, grip_signal::LOOSE, up_signal::NORMAL ,up_signal::NORMAL};
inline constexpr Pose2D kGrip{0x0f, 0, 0, 0, pn_signal::POS, grip_signal::GRIP, up_signal::NORMAL,up_signal::NORMAL };
inline constexpr Pose2D kLoose{0x0f, 0,0,0 ,pn_signal::POS,grip_signal::LOOSE,up_signal::NORMAL , up_signal::NORMAL};
inline constexpr Pose2D kTurn90Left{0x0f, 0, 0, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::UP,up_signal::NORMAL};
inline constexpr Pose2D kTurn90Right{0x0f, 0, 0, 0, pn_signal::NEG, grip_signal::LOOSE, up_signal::NORMAL ,up_signal::NORMAL};
inline constexpr Pose2D kDown{0x0f, 0,0,0 ,pn_signal::POS ,grip_signal::LOOSE , up_signal::DOWN ,up_signal::NORMAL};




/**
 * @brief FastLIO IMU 完整数据结构
 */
struct ImuData
{
    double ori_x {0.0}, ori_y {0.0}, ori_z {0.0}, ori_w {1.0};
    double ang_vel_x {0.0}, ang_vel_y {0.0}, ang_vel_z {0.0};
    double lin_acc_x {0.0}, lin_acc_y {0.0}, lin_acc_z {0.0};
};

// struct ImuDiff
// {
//     double ang_vel_diff {0.0};   // 角速度矢量差的模
//     double lin_acc_diff {0.0};   // 线加速度矢量差的模
//     bool valid {false};
// };

/**
 * @brief 里程计完整数据结构
 */
struct PoseData
{
    double x {0.0}, y {0.0}, z {0.0};
    double ori_x {0.0}, ori_y {0.0}, ori_z {0.0}, ori_w {1.0};
    double lin_vel_x {0.0}, lin_vel_y {0.0}, lin_vel_z {0.0};
    double ang_vel_x {0.0}, ang_vel_y {0.0}, ang_vel_z {0.0};
};

/**
 * @brief 传感器监听器，订阅 /Odometry 和雷达 IMU
 */
class SensorNode : public rclcpp::Node
{
public:
    explicit SensorNode(
            const std::string& odom_topic = "/Odometry",
            const std::string& radar_imu_topic = "/livox/imu"/*,
            const std::string& chassis_imu_topic = "/chassis/imu"*/)
        : rclcpp::Node("sensor_node")
    {
        const auto qos = rclcpp::SensorDataQoS();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, qos,
            std::bind(&SensorNode::onOdometryReceived, this, _1));
        radar_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(radar_imu_topic, qos,
            std::bind(&SensorNode::onRadarImuReceived, this, _1));
        
        
        RCLCPP_INFO(get_logger(),
            "Listening on %s, radar IMU: %s",
            odom_topic.c_str(), radar_imu_topic.c_str());
    }

    // Odometry
    double currentX() const { std::lock_guard<std::mutex> lock(odom_mutex_); return pose_.x; }
    double currentY() const { std::lock_guard<std::mutex> lock(odom_mutex_); return pose_.y; }
    double currentZ() const { std::lock_guard<std::mutex> lock(odom_mutex_); return pose_.z; }
    
    double currentYaw() const
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        double siny = 2.0 * (pose_.ori_w * pose_.ori_z + pose_.ori_x * pose_.ori_y);
        double cosy = 1.0 - 2.0 * (pose_.ori_y * pose_.ori_y + pose_.ori_z * pose_.ori_z); 
        return std::atan2(siny , cosy);
    }

    PoseData poseData() const { std::lock_guard<std::mutex> lock(odom_mutex_); return pose_; }

    // 雷达 IMU
    ImuData imuData() const { std::lock_guard<std::mutex> lock(radar_imu_mutex_); return radar_imu_; }

    // 底盘 IMU
    // ImuData chassisImuData() const { std::lock_guard<std::mutex> lock(chassis_imu_mutex_); return chassis_imu_; }

private:
    void onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;
        pose_.z = msg->pose.pose.position.z;
        pose_.ori_x = msg->pose.pose.orientation.x;
        pose_.ori_y = msg->pose.pose.orientation.y;
        pose_.ori_z = msg->pose.pose.orientation.z;
        pose_.ori_w = msg->pose.pose.orientation.w;
        pose_.lin_vel_x = msg->twist.twist.linear.x;
        pose_.lin_vel_y = msg->twist.twist.linear.y;
        pose_.lin_vel_z = msg->twist.twist.linear.z;
        pose_.ang_vel_x = msg->twist.twist.angular.x;
        pose_.ang_vel_y = msg->twist.twist.angular.y;
        pose_.ang_vel_z = msg->twist.twist.angular.z;
    }

    void onRadarImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(radar_imu_mutex_);
        copyImu(*msg, radar_imu_);
    }

    static void copyImu(const sensor_msgs::msg::Imu& src, ImuData& dst)
    {
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
    }

    // Odometry
    mutable std::mutex odom_mutex_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    PoseData pose_;

    // 雷达 IMU
    mutable std::mutex radar_imu_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr radar_imu_sub_;
    ImuData radar_imu_;

    // // 底盘 IMU
    // mutable std::mutex chassis_imu_mutex_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr chassis_imu_sub_;
    // ImuData chassis_imu_;
};


class Map 
{
public:
    Map() = default;
    // 这里可以添加地图相关的方法和成员变量 



public:
    std::vector<std::pair<double, double>> obstacles;
     // 存储障碍物位置的示例
};

/**
 * @brief 后台读线程：持续解析 0x0E 心跳包，维护 is_linked 值和最后接收时间戳
 */
class LinkMonitor
{
public:
    LinkMonitor(std::shared_ptr<SerialPort> port,
                rclcpp::Logger logger,
                chr::milliseconds heartbeat_timeout = chr::milliseconds(500))
        : port_(std::move(port)),
          logger_(logger),
          heartbeat_timeout_(heartbeat_timeout)
    {}

    ~LinkMonitor() { stop(); }

    LinkMonitor(const LinkMonitor&) = delete;
    LinkMonitor& operator=(const LinkMonitor&) = delete;

    void start()
    {
        if (running_.exchange(true)) return;
        thread_ = std::thread(&LinkMonitor::run, this);
    }

    void stop()
    {
        if (!running_.exchange(false)) return;
        if (thread_.joinable()) thread_.join();
    }

    bool isLinked() const
    {
        if (last_is_linked_.load(std::memory_order_relaxed) != 1) return false;
        const int64_t last_ns = last_recv_ns_.load(std::memory_order_relaxed);
        if (last_ns == 0) return false;
        const auto now_ns = chr::duration_cast<chr::nanoseconds>(
            chr::steady_clock::now().time_since_epoch()).count();
        return chr::nanoseconds(now_ns - last_ns) < heartbeat_timeout_;
    }

    int lastIsLinked() const { return last_is_linked_.load(std::memory_order_relaxed); }
    int64_t lastRecvNs() const { return last_recv_ns_.load(std::memory_order_relaxed); }

private:
    void run()
    {
        constexpr std::size_t kLen = sizeof(Signal2D);
        uint8_t buf[kLen] {};
        std::size_t buf_pos = 0;

        while (running_.load(std::memory_order_relaxed))
        {
            ssize_t n = port_->readSome(buf + buf_pos, kLen - buf_pos);
            if (n < 0)
            {
                RCLCPP_ERROR(logger_, "[LinkMonitor] 读取错误: %s",
                    port_->lastError().c_str());
                std::this_thread::sleep_for(chr::milliseconds(100));
                continue;
            }
            if (n == 0)
            {
                std::this_thread::sleep_for(chr::milliseconds(20));
                continue;
            }

            buf_pos += static_cast<std::size_t>(n);
            if (buf_pos < kLen) continue;

            Signal2D pkt;
            std::memcpy(&pkt, buf, kLen);

            if (pkt.header != 0x0E)
            {
                std::memmove(buf, buf + 1, kLen - 1);
                buf_pos = kLen - 1;
                continue;
            }

            buf_pos = 0;
            last_is_linked_.store(static_cast<int>(pkt.is_linked),
                                  std::memory_order_relaxed);
            last_recv_ns_.store(
                chr::duration_cast<chr::nanoseconds>(
                    chr::steady_clock::now().time_since_epoch()).count(),
                std::memory_order_relaxed);
        }
    }

    std::shared_ptr<SerialPort> port_;
    rclcpp::Logger logger_;
    chr::milliseconds heartbeat_timeout_;

    std::atomic<bool> running_{false};
    std::thread thread_;
    std::atomic<int> last_is_linked_{-1};
    std::atomic<int64_t> last_recv_ns_{0};
};

/**
 * @brief 应用程序上下文，包含共享资源如串口和tf监听器
 *
 */
struct AppContext
{
    rclcpp::Logger logger {rclcpp::get_logger("gps_bt_app")};
    std::shared_ptr<SerialPort>   motion_port;
    std::shared_ptr<SensorNode> sensor_node;
    std::shared_ptr<LinkMonitor> link_monitor;

    int line;
    int kfs_loc;
};





/**
 * @brief A base class for timed velocity actions that send Pose2D commands to the robot's motion port.
 * Derived classes can specify different velocity commands by providing different Pose2D values.
 * The action will run for a specified duration, repeatedly sending the command until the duration expires.
 * 
 */

class TimedVelocityAction : public BT::StatefulActionNode
{
public:
    TimedVelocityAction(
        const std::string& name,const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context,
        Pose2D command)
        : BT::StatefulActionNode(name, config),
          context_(std::move(context)),
          command_(command)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("duration_ms", 260, "Action duration in milliseconds")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!context_ || !context_->motion_port)
        {
            throw BT::RuntimeError("motion serial port context is missing");
        }

        int duration_ms = 260;
        if (const auto value = getInput<int>("duration_ms"))
        {
            duration_ms = value.value();
        }

        if (duration_ms <= 0)
        {
            throw BT::RuntimeError("duration_ms must be > 0");
        }

        deadline_ = chr::steady_clock::now() + chr::milliseconds(duration_ms);

        if (!sendCommand(command_))
        {
            RCLCPP_ERROR(
                context_->logger,
                "[%s] failed to send start command: %s",
                name().c_str(),
                context_->motion_port->lastError().c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(
            context_->logger,
            "[%s] started: x=%.3f y=%.3f z=%.3f duration=%dms",
            name().c_str(),
            context_->sensor_node->currentX(),
            context_->sensor_node->currentY(),
            context_->sensor_node->currentZ(),
            duration_ms);
        

        RCLCPP_INFO(
            context_->logger,
            "[%s] started: x=%3d y=%3d z=%3d",
            name().c_str(),
            command_.x,
            command_.y,
            command_.z);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (chr::steady_clock::now() < deadline_)
        {
            if (chr::steady_clock::now() < (deadline_) - chr::milliseconds(1000))
            {
                RCLCPP_DEBUG(
                    context_->logger,
                    "[%s] running: time left %ldms",
                    name().c_str(),
                    std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - std::chrono::steady_clock::now()).count());
            }
            if (!sendCommand(command_))
                {
                    RCLCPP_ERROR(
                        context_->logger,
                        "[%s] failed while running: %s",
                    name().c_str(),
                    context_->motion_port->lastError().c_str());
                stopRobot();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }


        stopRobot();
        RCLCPP_INFO(context_->logger, "[%s] completed", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        stopRobot();
        if (context_)
        {
            RCLCPP_WARN(context_->logger, "[%s] halted", name().c_str());
        }
    }

protected:

    bool sendCommand(const Pose2D& msg)
    {
        return context_->motion_port->writeExact(&msg, sizeof(msg));
    }


    void stopRobot()
    {
        const Pose2D stop = kStop;
        if (!sendCommand(stop) && context_)
        {
            RCLCPP_ERROR(
                context_->logger,
                "[%s] failed to send stop command: %s",
                name().c_str(),
                context_->motion_port->lastError().c_str());
        }
    }

    std::shared_ptr<AppContext> context_;
    Pose2D command_;
    chr::steady_clock::time_point deadline_ {};
};

class MoveUP : public TimedVelocityAction
{
public:
    MoveUP(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context 
    ) : TimedVelocityAction(name, config, std::move(context), kTurn90Left)
    {}
};

class MoveDOWN  : public TimedVelocityAction
{
public:
    MoveDOWN(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context 
    ) : TimedVelocityAction(name, config, std::move(context), kDown)
    {}
};

class MoveToLocation : public TimedVelocityAction
{
public:
    MoveToLocation(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context )
        : TimedVelocityAction(name, config, std::move(context), kStop)

    {   
    }

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<int>("duration_ms", 260, "Action duration in milliseconds"),
            BT::InputPort<std::string>("location" ,"home" ," to where")
        };
    }

    BT::NodeStatus onStart() override
    {
        std::string loc_name; 
        if (!getInput<std::string>("location", loc_name))
            return BT::NodeStatus::FAILURE;

        auto it = point_map.find(loc_name);
        if (it == point_map.end())
        {
            RCLCPP_ERROR(context_->logger, "未知点位: %s", loc_name.c_str());
            return BT::NodeStatus::FAILURE;
        }
        dest_location_ = it->second;
        phase_ = Phase::TURN_X;
        
        overall_deadtime_ = chr::steady_clock::now() + chr::seconds(50);
        int duration_ms = 5000;
        getInput<int>("duration_ms", duration_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(duration_ms);

        RCLCPP_INFO(context_->logger, "前往 %s (%.1f, %.1f)", 
                    loc_name.c_str(), dest_location_.x, dest_location_.y);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = chr::steady_clock::now();
        if (now >= overall_deadtime_)
        {
            stopRobot();
            return BT::NodeStatus::FAILURE;
        }

        PoseData Pose = context_->sensor_node->poseData();

        //uint8_t k = 10;
        Location current_location;
        current_location.x = static_cast<float>(context_->sensor_node->currentX());
        current_location.y = static_cast<float>(context_->sensor_node->currentY());
        float distance_x = dest_location_.x - current_location.x;
        float distance_y = dest_location_.y - current_location.y;
        double yaw = quatToYaw(Pose.ori_x , Pose.ori_y , Pose.ori_z ,Pose.ori_w );
        RCLCPP_INFO(context_->logger , "current_node:{%s}dis_x:%3f,dis_y:%3f",name().c_str(),distance_x ,distance_y);

        
        // if(chr::steady_clock::now() < deadline_)
        // {
        //     if(chr::steady_clock::now()  < (deadline_) - chr::milliseconds(4500))
        //     {
        //         float speed_x = k * distance_x;
        //         if(speed_x >=0)
        //         {
                    
        //             Pose2D cmd{0x0f, (uint8_t)speed_x, 0, 0, 0, 0, 0};
        //             sendCommand(cmd);
        //         }
        //         else
        //         {

        //             Pose2D cmd{0x0f, (uint8_t)-speed_x, 0, 0, 1, 0, 0};
        //             sendCommand(cmd);
        //         }
                
        //     }
        //     else
        //     {
        //         float speed_y = k * distance_y;
        //         if(speed_y >=0)
        //         {
                    
        //             Pose2D cmd{0x0f, 0, (uint8_t)speed_y, 0, 0, 0, 0};
        //             sendCommand(cmd);
        //         }
        //         else
        //         {

        //             Pose2D cmd{0x0f,  0, (uint8_t)-speed_y, 0, 1, 0, 0};
        //             sendCommand(cmd);
        //         }
        //     }

            float k_tolerance = 0.2f;
            if((std::abs(distance_x) < k_tolerance) && (std::abs(distance_y) < k_tolerance))
            {
                stopRobot();
                return BT::NodeStatus::SUCCESS;
            }

            switch (phase_)
            {
            case Phase::TURN_X: return turnToFace((distance_x>=0) ? 0.0 : M_PI ,yaw,Phase::DRIVE_X );
            case Phase::DRIVE_X : return driveToTarget(distance_x  , Phase::TURN_Y);
            case Phase::TURN_Y : return turnToFace((distance_y>=0) ? M_PI_2 : -M_PI_2,yaw,Phase::DRIVE_Y);
            case Phase::DRIVE_Y : return driveToTarget(distance_y , Phase::DONE);
            case Phase::DONE : stopRobot(); return BT::NodeStatus::SUCCESS;
            }


        //     return BT::NodeStatus::RUNNING;
        // }
        return BT::NodeStatus::FAILURE;
    }


protected:
    enum class Phase{ TURN_X , DRIVE_X , TURN_Y , DRIVE_Y ,DONE};
    
    static double quatToYaw(double x,double y, double z,double w)
    {
        double siny = 2.0 * (w*z + x*y);
        double cosy = 1.0 - 2.0 * ( y*y  + z* z);
        return std::atan2(siny , cosy);
    }

    BT::NodeStatus turnToFace(double target_yaw , double current_yaw , Phase next)
    {
        double diff = target_yaw - current_yaw ;
        while (diff > M_PI) diff -= 2* M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;

        constexpr double kYawTol = 0.17;
        if(std::abs(diff) < kYawTol)
        {
            stopRobot();
            phase_ = next;
            RCLCPP_INFO(context_->logger,"MoveTo:转向完成，进入下一阶段");
            return BT::NodeStatus::RUNNING;
        }
        
        sendCommand((diff>0) ? kTurnLeft: kTurnRight);
        return BT::NodeStatus::RUNNING;
    } 


    virtual BT::NodeStatus driveToTarget(float distance , Phase next)
    {
        constexpr float kTol = 0.15f;
        if( std::abs(distance) < kTol)
        {
            stopRobot();
            phase_ = next;
            return BT::NodeStatus::RUNNING;
        }

        float speed = std::clamp( 10.0f * distance , -20.0f ,20.0f);
        if(speed >= 0)
        {
            Pose2D cmd{0x0f, (uint8_t)speed, 0, 0, pn_signal::POS,grip_signal::LOOSE, up_signal::NORMAL, up_signal::NORMAL};
            sendCommand(cmd);
        }
        else
        {
            Pose2D cmd{0x0f, (uint8_t)(-speed), 0, 0, pn_signal::POS, grip_signal::LOOSE , up_signal::NORMAL ,up_signal::NORMAL};
            sendCommand(cmd);
        }
        return BT::NodeStatus::RUNNING;
    }

    Location dest_location_ ;
    Phase phase_ {Phase::TURN_X};
    chr::steady_clock::time_point overall_deadtime_ ; 
};

// Rotate-only var#iants of MoveToLocation.
// A/B/C/D share the same logic: spin toward a fixed absolute yaw, return SUCCESS
// once within tolerance. Subclasses only override targetYaw().
class MoveToLocationA : public MoveToLocation
{
public:
    MoveToLocationA(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocation(name, config, std::move(context))
    {}

    BT::NodeStatus onRunning() override
    {
        if (chr::steady_clock::now() >= overall_deadtime_)
        {
            stopRobot();
            return BT::NodeStatus::FAILURE;
        }

        PoseData pose = context_->sensor_node->poseData();
        double yaw = quatToYaw(pose.ori_x, pose.ori_y, pose.ori_z, pose.ori_w);
        double diff = targetYaw() - yaw;
        while (diff >  M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;

        constexpr double kYawTol = 0.17;
        if (std::abs(diff) < kYawTol)
        {
            stopRobot();
            RCLCPP_INFO(context_->logger, "%s: 转向完成", name().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        sendCommand((diff > 0) ? kTurnLeft : kTurnRight);
        return BT::NodeStatus::RUNNING;
    }

protected:
    virtual double targetYaw() const { return M_PI; }
};

class MoveToLocationB : public MoveToLocationA
{
public:
    MoveToLocationB(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocationA(name, config, std::move(context))
    {}

protected:
    double targetYaw() const override { return M_PI_2; }
};

class MoveToLocationC : public MoveToLocationA
{
public:
    MoveToLocationC(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocationA(name, config, std::move(context))
    {}

protected:
    double targetYaw() const override { return -M_PI_2; }
};

class MoveToLocationD : public MoveToLocationA
{
public:
    MoveToLocationD(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocationA(name, config, std::move(context))
    {}

protected:
    double targetYaw() const override { return -M_PI; }
};
// class MoveToLocationUP : public TimedVelocityAction
// {
// public: 
//     MoveToLocationUP(
//         const std::string& name,
//         const BT::NodeConfig& config,
//         std::shared_pt<AppContext> context )
//         : TimedVelocityAction(name , config , std::move(context), kStop)
//         {
//         }

//         static BT::PortsList providedPorts()
//         {
//             return {
//                 BT::InputPort<int>("duration_ms" , 260 , "duration in milleseconds"),
//                 BT::InputPort<int>("location" ,"home" , "to where")
//             };
//         }
    
//     BT::NodeStatus onStart() override
//     {
//         std::string loc_name;
//         if (!getInput<std::string>("location" , loc_name))
//             return BT::NodeStatus::FAILURE;
        
//         auto it = point_map.find(loc_name);
//         if (it == point_map.end())
//         {
//             RCLCPP_ERROR(context_->logger/ )
//         }
//     }

        
// }

class MoveToLocationUP: public MoveToLocation
{
    public:
    MoveToLocationUP(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocation(name, config, std::move(context))
    {}

    BT::NodeStatus onStart() override
    {
        std::string loc_name; 
        if (!getInput<std::string>("location", loc_name))
            return BT::NodeStatus::FAILURE;

        auto it = point_map.find(loc_name);
        if (it == point_map.end())
        {
            RCLCPP_ERROR(context_->logger, "未知点位: %s", loc_name.c_str());
            return BT::NodeStatus::FAILURE;
        }
        dest_location_ = it->second;
    
        
        overall_deadtime_ = chr::steady_clock::now() + chr::seconds(10);
        int duration_ms = 5000;
        getInput<int>("duration_ms", duration_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(duration_ms);

        RCLCPP_INFO(context_->logger, "前往 %s (%.1f, %.1f)", 
                    loc_name.c_str(), dest_location_.x, dest_location_.y);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = chr::steady_clock::now();
        if (now >= overall_deadtime_)
        {
            stopRobot();
            return BT::NodeStatus::FAILURE;
        }

        PoseData Pose = context_->sensor_node->poseData();

        //uint8_t k = 10;
        Location current_location;
        current_location.x = static_cast<float>(context_->sensor_node->currentX());
        current_location.y = static_cast<float>(context_->sensor_node->currentY());
        float distance_x = dest_location_.x - current_location.x;
        float distance_y = dest_location_.y - current_location.y;
        double yaw = quatToYaw(Pose.ori_x , Pose.ori_y , Pose.ori_z ,Pose.ori_w );
        RCLCPP_INFO(context_->logger , "current_node:{%s}dis_x:%3f,dis_y:%3f",name().c_str(),distance_x ,distance_y);

            float k_tolerance = 0.3f;
            if((std::abs(distance_x) < k_tolerance) && (std::abs(distance_y) < k_tolerance))
            {
                stopRobot();
                return BT::NodeStatus::SUCCESS;
            }

            switch (phase_)
            {
            case Phase::TURN_X: return turnToFace((distance_x>=0) ? M_PI : 0.0 ,yaw,Phase::DRIVE_X );
            case Phase::DRIVE_X : return driveToTarget(distance_x  , Phase::TURN_Y);
            case Phase::TURN_Y : return turnToFace((distance_y>=0) ? M_PI_2 : -M_PI_2,yaw,Phase::DRIVE_Y);
            case Phase::DRIVE_Y : return driveToTarget(distance_y , Phase::DONE);
            case Phase::DONE : stopRobot(); return BT::NodeStatus::SUCCESS;
            }


        //     return BT::NodeStatus::RUNNING;
        // }
        return BT::NodeStatus::FAILURE;
    }


    protected:
    
    


    BT::NodeStatus driveToTarget(float distance ,Phase next) override
    {
        constexpr float kTol = 0.15f;
        if( std::abs(distance) < kTol)
        {
            stopRobot();
            phase_ = next;
            return BT::NodeStatus::RUNNING;
        }

        float speed = std::clamp( 10.0f * distance , -20.0f ,20.0f);
        if(speed >= 0)
        {
            Pose2D cmd{0x0f, (uint8_t)speed, 0, 0, pn_signal::NEG, grip_signal::LOOSE, up_signal::UP, up_signal::NORMAL};
            sendCommand(cmd);
        }
        else
        {
            Pose2D cmd{0x0f, (uint8_t)(-speed), 0, 0, pn_signal::NEG, grip_signal::LOOSE, up_signal::UP, up_signal::NORMAL};
            sendCommand(cmd);
        }
        return BT::NodeStatus::RUNNING;
    }

};

class MoveToLocationDOWN: public MoveToLocation
{
    public:
    MoveToLocationDOWN(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocation(name, config, std::move(context))
    {}

    protected:
    BT::NodeStatus driveToTarget(float distance ,Phase next) override
    {
        constexpr float kTol = 0.15f;
        if( std::abs(distance) < kTol)
        {
            stopRobot();
            phase_ = next;
            return BT::NodeStatus::RUNNING;
        }

        float speed = std::clamp( 10.0f * distance , -20.0f ,20.0f);
        if(speed >= 0)
        {
            Pose2D cmd{0x0f, (uint8_t)speed, 0, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::DOWN, up_signal::NORMAL};
            sendCommand(cmd);
        }
        else
        {
            Pose2D cmd{0x0f, (uint8_t)(-speed), 0, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::DOWN, up_signal::NORMAL};
            sendCommand(cmd);
        }
        return BT::NodeStatus::RUNNING;
    }

};

class MoveToLocationL : public MoveToLocation
{
public:
    MoveToLocationL(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocation(name, config, std::move(context))
    {}

    BT::NodeStatus onStart() override
    {
        auto status = MoveToLocation::onStart();
        phase_ = Phase::TURN_Y;
        return status;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = chr::steady_clock::now();
        if (now >= overall_deadtime_)
        {
            stopRobot();
            return BT::NodeStatus::FAILURE;
        }

        PoseData Pose = context_->sensor_node->poseData();
        Location current_location;
        current_location.x = static_cast<float>(context_->sensor_node->currentX());
        current_location.y = static_cast<float>(context_->sensor_node->currentY());
        float distance_x = dest_location_.x - current_location.x;
        float distance_y = dest_location_.y - current_location.y;
        double yaw = quatToYaw(Pose.ori_x, Pose.ori_y, Pose.ori_z, Pose.ori_w);
        RCLCPP_INFO(context_->logger, "current_node:{%s}dis_x:%3f,dis_y:%3f",
                    name().c_str(), distance_x, distance_y);

        float k_tolerance = 0.2f;
        if ((std::abs(distance_x) < k_tolerance) && (std::abs(distance_y) < k_tolerance))
        {
            stopRobot();
            return BT::NodeStatus::SUCCESS;
        }

        switch (phase_)
        {
        case Phase::TURN_Y:  return turnToFace((distance_y >= 0) ? M_PI_2 : -M_PI_2, yaw, Phase::DRIVE_Y);
        case Phase::DRIVE_Y: return driveToTarget(distance_y, Phase::TURN_X);
        case Phase::TURN_X:  return turnToFace((distance_x >= 0) ? 0.0 : M_PI, yaw, Phase::DRIVE_X);
        case Phase::DRIVE_X: return driveToTarget(distance_x, Phase::DONE);
        case Phase::DONE:    stopRobot(); return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class MoveToLocationUPL : public MoveToLocationL
{
public:
    MoveToLocationUPL(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocationL(name, config, std::move(context))
    {}

protected:
    BT::NodeStatus driveToTarget(float distance, Phase next) override
    {
        constexpr float kTol = 0.15f;
        if (std::abs(distance) < kTol)
        {
            stopRobot();
            phase_ = next;
            return BT::NodeStatus::RUNNING;
        }
        float speed = std::clamp(10.0f * distance, -20.0f, 20.0f);
        if (speed >= 0)
        {
            Pose2D cmd{0x0f, (uint8_t)speed, 0, 0, pn_signal::NEG, grip_signal::LOOSE, up_signal::UP, up_signal::NORMAL};
            sendCommand(cmd);
        }
        else
        {
            Pose2D cmd{0x0f, (uint8_t)(-speed), 0, 0, pn_signal::NEG, grip_signal::LOOSE, up_signal::UP, up_signal::NORMAL};
            sendCommand(cmd);
        }
        return BT::NodeStatus::RUNNING;
    }
};

class MoveToLocationDOWNL : public MoveToLocationL
{
public:
    MoveToLocationDOWNL(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocationL(name, config, std::move(context))
    {}

protected:
    BT::NodeStatus driveToTarget(float distance, Phase next) override
    {
        constexpr float kTol = 0.15f;
        if (std::abs(distance) < kTol)
        {
            stopRobot();
            phase_ = next;
            return BT::NodeStatus::RUNNING;
        }
        float speed = std::clamp(10.0f * distance, -20.0f, 20.0f);
        if (speed >= 0)
        {
            Pose2D cmd{0x0f, (uint8_t)speed, 0, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::DOWN, up_signal::NORMAL};
            sendCommand(cmd);
        }
        else
        {
            Pose2D cmd{0x0f, (uint8_t)(-speed), 0, 0, pn_signal::POS, grip_signal::LOOSE, up_signal::DOWN, up_signal::NORMAL};
            sendCommand(cmd);
        }
        return BT::NodeStatus::RUNNING;
    }
};


class MoveToLocationHorizontal : public MoveToLocation
{
public:
    MoveToLocationHorizontal(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : MoveToLocation(name, config, std::move(context))
    {}

    BT::NodeStatus onStart() override
    {
        auto status = MoveToLocation::onStart();
        phase_ = Phase::DRIVE_X;
        return status;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = chr::steady_clock::now();
        if (now >= overall_deadtime_)
        {
            stopRobot();
            return BT::NodeStatus::FAILURE;
        }

        Location current_location;
        current_location.x = static_cast<float>(context_->sensor_node->currentX());
        current_location.y = static_cast<float>(context_->sensor_node->currentY());
        float distance_x = dest_location_.x - current_location.x;
        float distance_y = dest_location_.y - current_location.y;
        RCLCPP_INFO(context_->logger, "current_node:{%s}dis_x:%3f,dis_y:%3f",
                    name().c_str(), distance_x, distance_y);

        constexpr float k_tolerance = 0.2f;
        if ((std::abs(distance_x) < k_tolerance) && (std::abs(distance_y) < k_tolerance))
        {
            stopRobot();
            return BT::NodeStatus::SUCCESS;
        }

        switch (phase_)
        {
        case Phase::DRIVE_X: return driveLongitudinal(distance_x, Phase::DRIVE_Y);
        case Phase::DRIVE_Y: return driveLateral(distance_y, Phase::DONE);
        case Phase::DONE:    stopRobot(); return BT::NodeStatus::SUCCESS;
        default:             return BT::NodeStatus::FAILURE;
        }
    }

protected:
    BT::NodeStatus driveLongitudinal(float distance, Phase next)
    {
        constexpr float kTol = 0.15f;
        if (std::abs(distance) < kTol)
        {
            stopRobot();
            phase_ = next;
            return BT::NodeStatus::RUNNING;
        }
        float speed = std::clamp(10.0f * distance, -20.0f, 20.0f);
        uint8_t pn = (speed >= 0) ? pn_signal::POS : pn_signal::NEG;
        uint8_t mag = static_cast<uint8_t>(std::abs(speed));
        Pose2D cmd{0x0f, mag, 0, 0, pn, grip_signal::LOOSE, up_signal::NORMAL, up_signal::NORMAL};
        sendCommand(cmd);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus driveLateral(float distance, Phase next)
    {
        constexpr float kTol = 0.15f;
        if (std::abs(distance) < kTol)
        {
            stopRobot();
            phase_ = next;
            return BT::NodeStatus::RUNNING;
        }
        float speed = std::clamp(10.0f * distance, -20.0f, 20.0f);
        uint8_t pn = (speed >= 0) ? pn_signal::POS : pn_signal::NEG;
        uint8_t mag = static_cast<uint8_t>(std::abs(speed));
        Pose2D cmd{0x0f, 0, mag, 0, pn, grip_signal::LOOSE, up_signal::NORMAL, up_signal::NORMAL};
        sendCommand(cmd);
        return BT::NodeStatus::RUNNING;
    }
};


class MoveForward final : public TimedVelocityAction
{
public:
    MoveForward(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), kForward)
    {}
};

class MoveBackward final : public TimedVelocityAction
{
public:
    MoveBackward(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), kBack)
    {}
};

class TurnLeft final : public TimedVelocityAction
{
public:
    TurnLeft(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), kGrip)
    {
    }

};

class Horizon final : public TimedVelocityAction
{
public:
    Horizon(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), kLeft)
    {
    }

};


class Sync final : public TimedVelocityAction
{
public:
    Sync(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), kStop)
    {
    }

};


class AvoidKFS final : public BT::StatefulActionNode
{
public:
    AvoidKFS(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::StatefulActionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("current_point", "", "可选override，留空则用 odometry 自动定位到最近的 L1..R4"),
            BT::InputPort<std::string>("prefer_side", "auto", "left/right/auto"),
            BT::InputPort<double>("tolerance", 0.3, "到达邻居点位的容差(米)"),
            BT::InputPort<double>("snap_radius", 1.5, "自动定位时，距离最近网格点必须小于此值(米)"),
            BT::InputPort<int>("timeout_ms", 15000, "整体超时(毫秒)")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!context_ || !context_->motion_port)
            throw BT::RuntimeError("motion serial port context is missing");

        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
            context_->sensor_node, "/vision_node");
        if (!param_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(context_->logger, "[AvoidKFS] 无法连接 vision_node，跳过");
            return BT::NodeStatus::SUCCESS;
        }
        bool avoid_flag = false;
        try
        {
            auto params = param_client_->get_parameters({"Avoid"});
            if (!params.empty()) avoid_flag = params[0].as_bool();
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(context_->logger, "[AvoidKFS] 读取 Avoid 异常: %s", e.what());
            return BT::NodeStatus::SUCCESS;
        }
        if (!avoid_flag)
        {
            RCLCPP_INFO(context_->logger, "[AvoidKFS] Avoid=false，无需避让");
            return BT::NodeStatus::SUCCESS;
        }

        double snap_radius = 1.5;
        getInput<double>("snap_radius", snap_radius);

        std::string cur;
        getInput<std::string>("current_point", cur);
        if (cur.empty())
        {
            cur = locateNearestGridPoint(snap_radius);
            if (cur.empty())
            {
                RCLCPP_ERROR(context_->logger,
                    "[AvoidKFS] 自动定位失败: (%.2f, %.2f) 距任何 L1..R4 都超过 %.2fm",
                    context_->sensor_node->currentX(),
                    context_->sensor_node->currentY(),
                    snap_radius);
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(context_->logger, "[AvoidKFS] 自动定位到 %s", cur.c_str());
        }

        auto cur_pm = point_map.find(cur);
        if (cur_pm == point_map.end())
        {
            RCLCPP_ERROR(context_->logger, "[AvoidKFS] %s 不在 point_map", cur.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto adj = point_graph.find(cur);
        if (adj == point_graph.end() || adj->second.empty())
        {
            RCLCPP_ERROR(context_->logger, "[AvoidKFS] %s 无邻接点", cur.c_str());
            return BT::NodeStatus::FAILURE;
        }

        std::string prefer = "auto";
        getInput<std::string>("prefer_side", prefer);

        double yaw = context_->sensor_node->currentYaw();
        double c = std::cos(yaw), s = std::sin(yaw);

        std::string best_left, best_right;
        double best_left_score  = -1.0;
        double best_right_score = -1.0;
        Location best_left_loc, best_right_loc;

        for (const auto& edge : adj->second)
        {
            auto it = point_map.find(edge.to);
            if (it == point_map.end()) continue;
            double dx = it->second.x - cur_pm->second.x;
            double dy = it->second.y - cur_pm->second.y;
            double body_fwd = c * dx + s * dy;
            double body_lat = -s * dx + c * dy;
            if (std::abs(body_lat) <= std::abs(body_fwd)) continue; // 主要是前后向,不算横向邻居
            double score = std::abs(body_lat);
            if (body_lat > 0.0)
            {
                if (score > best_left_score)
                {
                    best_left_score = score;
                    best_left = edge.to;
                    best_left_loc = it->second;
                }
            }
            else
            {
                if (score > best_right_score)
                {
                    best_right_score = score;
                    best_right = edge.to;
                    best_right_loc = it->second;
                }
            }
        }

        const bool has_left  = !best_left.empty();
        const bool has_right = !best_right.empty();
        if (!has_left && !has_right)
        {
            RCLCPP_ERROR(context_->logger,
                "[AvoidKFS] %s 无横向邻居 (yaw=%.2f)", cur.c_str(), yaw);
            return BT::NodeStatus::FAILURE;
        }

        std::string chosen_side;
        if (prefer == "left" && has_left)
        {
            target_point_ = best_left; dest_ = best_left_loc; chosen_side = "left";
        }
        else if (prefer == "right" && has_right)
        {
            target_point_ = best_right; dest_ = best_right_loc; chosen_side = "right";
        }
        else
        {
            // auto,或首选侧不存在时,选横向分量最大的一侧
            if (best_left_score >= best_right_score && has_left)
            {
                target_point_ = best_left; dest_ = best_left_loc; chosen_side = "left";
            }
            else
            {
                target_point_ = best_right; dest_ = best_right_loc; chosen_side = "right";
            }
        }

        getInput<double>("tolerance", tolerance_);
        int timeout_ms = 15000;
        getInput<int>("timeout_ms", timeout_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(timeout_ms);

        RCLCPP_INFO(context_->logger,
            "[AvoidKFS] %s -> %s (yaw=%.2f, side=%s)",
            cur.c_str(), target_point_.c_str(), yaw, chosen_side.c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (chr::steady_clock::now() >= deadline_)
        {
            stopRobot();
            RCLCPP_WARN(context_->logger, "[AvoidKFS] 超时");
            return BT::NodeStatus::FAILURE;
        }

        float cur_x = static_cast<float>(context_->sensor_node->currentX());
        float cur_y = static_cast<float>(context_->sensor_node->currentY());
        float dx = dest_.x - cur_x;
        float dy = dest_.y - cur_y;

        if (std::hypot(dx, dy) < static_cast<float>(tolerance_))
        {
            stopRobot();
            RCLCPP_INFO(context_->logger, "[AvoidKFS] 已到达 %s", target_point_.c_str());
            return BT::NodeStatus::SUCCESS;
        }

        double yaw = context_->sensor_node->currentYaw();
        double c = std::cos(yaw), s = std::sin(yaw);
        float body_fwd = static_cast<float>(c * dx + s * dy);
        float body_lat = static_cast<float>(-s * dx + c * dy);

        float fwd_speed = std::clamp(10.0f * body_fwd, -20.0f, 20.0f);
        float lat_speed = std::clamp(10.0f * body_lat, -20.0f, 20.0f);
        uint8_t pn = (fwd_speed >= 0) ? pn_signal::POS : pn_signal::NEG;
        uint8_t x_mag = static_cast<uint8_t>(std::abs(fwd_speed));
        uint8_t y_mag = static_cast<uint8_t>(std::abs(lat_speed));

        Pose2D cmd{0x0f, x_mag, y_mag, 0, pn,
                   grip_signal::LOOSE, up_signal::NORMAL, up_signal::NORMAL};
        context_->motion_port->writeExact(&cmd, sizeof(cmd));
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        stopRobot();
        RCLCPP_WARN(context_->logger, "[AvoidKFS] 被中断");
    }

private:
    std::string locateNearestGridPoint(double max_dist) const
    {
        static const std::vector<std::string> grid_names = {
            "L1","L2","L3","L4","M1","M2","M3","M4","R1","R2","R3","R4"
        };
        double cx = context_->sensor_node->currentX();
        double cy = context_->sensor_node->currentY();
        std::string best;
        double best_d2 = max_dist * max_dist;
        for (const auto& name : grid_names)
        {
            auto it = point_map.find(name);
            if (it == point_map.end()) continue;
            double dx = it->second.x - cx;
            double dy = it->second.y - cy;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2)
            {
                best_d2 = d2;
                best = name;
            }
        }
        return best;
    }
    void stopRobot()

    {
        context_->motion_port->writeExact(&kStop, sizeof(kStop));
    }

    std::shared_ptr<AppContext> context_;
    std::shared_ptr<rclcpp::SyncParametersClient> param_client_;
    std::string target_point_;
    Location dest_;
    double tolerance_ = 0.2;
    chr::steady_clock::time_point deadline_;
};


class TurnRight final : public TimedVelocityAction
{
public:
    TurnRight(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), kTurn90Left)
    {
    }

    BT::NodeStatus onStart() override
    {
        int duration_ms = 2000;
        getInput<int>("duration_ms", duration_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(duration_ms);

        start_yaw_ = context_->sensor_node->currentYaw();
        target_yaw_ = normalizeAngle(start_yaw_ - M_PI_2);

        RCLCPP_INFO(context_->logger,
            "[%s] turn right: start_yaw=%.2f target_yaw=%.2f timeout=%dms",
            name().c_str(), start_yaw_, target_yaw_, duration_ms);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (chr::steady_clock::now() >= deadline_)
        {
            stopRobot();
            RCLCPP_WARN(context_->logger, "[%s] turn right timeout", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        double yaw = context_->sensor_node->currentYaw();
        double diff = normalizeAngle(target_yaw_ - yaw);

        if (std::abs(diff) < kYawTolerance)
        {
            stopRobot();
            RCLCPP_INFO(context_->logger, "[%s] turn right done, yaw=%.2f", name().c_str(), yaw);
            return BT::NodeStatus::SUCCESS;
        }

        sendCommand(diff > 0 ? kTurnLeft : kTurnRight);
        return BT::NodeStatus::RUNNING;
    }

private:
    static double normalizeAngle(double a)
    {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    static constexpr double kYawTolerance = 0.2 ;
    double start_yaw_ = 0.0;
    double target_yaw_ = 0.0;
};





/**
 * @brief judge position (待完善
 * 
 */

//  class JudgePosition final : public TimedVelocityAction
// {
// public:
//     JudgePosition(
//         const std::string& name,
//         const BT::NodeConfig& config,
//         std::shared_ptr<AppContext> context)
//         : TimedVelocityAction(name, config, std::move(context), Pose2D{0x0f, 0, 0, 0})
//     {}
// };



/**
 * @brief 视觉代码（待完善
 * 
 */
// class DetectFront final : public BT::ConditionNode
// {
// public:
//     DetectFront(
//         const std::string& name,
//         const BT::NodeConfig& config,
//         std::shared_ptr<AppContext> context)
//         : BT::ConditionNode(name, config),
//           context_(std::move(context))
//     {
//     }

//     static BT::PortsList providedPorts()
//     {
//         return {
//             BT::InputPort<int>("expected_value", 1, "Expected front sensor byte"),
//             BT::InputPort<int>("read_len", 1, "How many bytes to read")
//         };
//     }

//     BT::NodeStatus tick() override
//     {
//         if (!context_ || !context_->sensor_port)
//         {
//             throw BT::RuntimeError("sensor serial port context is missing");
//         }

//         int expected_value = 1;
//         int read_len = 1;

//         if (const auto value = getInput<int>("expected_value"))
//         {
//             expected_value = value.value();
//         }

//         if (const auto value = getInput<int>("read_len"))
//         {
//             read_len = value.value();
//         }

//         if (read_len <= 0 || read_len > 256)
//         {
//             throw BT::RuntimeError("read_len must be in range [1, 256]");
//         }

//         std::vector<std::uint8_t> buffer(static_cast<std::size_t>(read_len), 0U);
//         const ssize_t n = context_->sensor_port->readSome(buffer.data(), buffer.size());

//         if (n < 0)
//         {
//             RCLCPP_ERROR(
//                 context_->logger,
//                 "[%s] sensor read failed: %s",
//                 name().c_str(),
//                 context_->sensor_port->lastError().c_str());
//             return BT::NodeStatus::FAILURE;
//         }

//         if (n == 0)
//         {
//             RCLCPP_DEBUG(context_->logger, "[%s] no sensor data yet", name().c_str());
//             return BT::NodeStatus::FAILURE;
//         }

//         const auto actual_value = static_cast<int>(buffer.front());
//         const auto matched = (actual_value == expected_value);

//         RCLCPP_INFO(
//             context_->logger,
//             "[%s] sensor=%d expected=%d result=%s",
//             name().c_str(),
//             actual_value,
//             expected_value,
//             matched ? "SUCCESS" : "FAILURE");

//         return matched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
//     }

// private:
//     std::shared_ptr<AppContext> context_;
// };

/**
 * @brief 节点注册函数
 * 
 * @param factory 
 * @param context 
 */
class AdjustPosition final : public BT::StatefulActionNode
{
public:
    AdjustPosition(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::StatefulActionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("target_yaw", 0.0, "Target heading in radians"),
            BT::InputPort<int>("timeout_ms", 5000, "Timeout in milliseconds")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!context_ || !context_->motion_port)
            throw BT::RuntimeError("motion serial port context is missing");
        context_->motion_port->writeExact(&kStop, sizeof(kStop));

        getInput<double>("target_yaw", target_yaw_);
        int timeout_ms = 5000;
        getInput<int>("timeout_ms", timeout_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(timeout_ms);

        RCLCPP_INFO(context_->logger,
            "[AdjustPosition] start yaw=%.3f target=%.3f timeout=%dms",
            context_->sensor_node->currentYaw(), target_yaw_, timeout_ms);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (chr::steady_clock::now() >= deadline_)
        {
            stopRobot();
            RCLCPP_WARN(context_->logger, "[AdjustPosition] timeout");
            return BT::NodeStatus::FAILURE;
        }

        double yaw = context_->sensor_node->currentYaw();
        double diff = target_yaw_ - yaw;
        while (diff >  M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;

        if (std::abs(diff) < kYawTolerance)
        {
            stopRobot();
            RCLCPP_INFO(context_->logger, "[AdjustPosition] done, yaw=%.3f", yaw);
            return BT::NodeStatus::SUCCESS;
        }

        // PD: P on angle error, D damps via IMU ang_vel_z to prevent overshoot
        double ang_vel = context_->sensor_node->imuData().ang_vel_z;
        double raw_speed = kP * diff - kD * ang_vel;
        auto z_val = static_cast<uint8_t>(std::clamp(std::abs(raw_speed), 1.0, 20.0));
        Pose2D cmd = (raw_speed > 0) ? kTurnLeft : kTurnRight;
        cmd.z = z_val;
        context_->motion_port->writeExact(&cmd, sizeof(cmd));
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        stopRobot();
        RCLCPP_WARN(context_->logger, "[AdjustPosition] halted");
    }

private:
    void stopRobot()
    {
        context_->motion_port->writeExact(&kStop, sizeof(kStop));
    }

    static constexpr double kYawTolerance = 0.1;
    static constexpr double kP = 10.0;
    static constexpr double kD = 3.0;
    std::shared_ptr<AppContext> context_;
    double target_yaw_ = 0.0;
    chr::steady_clock::time_point deadline_;
};

class CloseVision final : public BT::SyncActionNode
{
public:
    CloseVision(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::SyncActionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        context_->motion_port->writeExact(&kStop, sizeof(kStop));

        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
            context_->sensor_node, "/vision_node");
        if (!param_client->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(context_->logger, "[CloseVision] 无法连接到 vision_node");
            return BT::NodeStatus::FAILURE;
        }
        try
        {
            auto result = param_client->set_parameters({rclcpp::Parameter("START_VISION", false)});
            if (result[0].successful)
            {
                RCLCPP_INFO(context_->logger, "[CloseVision] 已关闭视觉系统");
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_ERROR(context_->logger, "[CloseVision] 设置参数失败: %s", result[0].reason.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(context_->logger, "[CloseVision] 异常: %s", e.what());
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<AppContext> context_;
};

class Observe final : public BT::StatefulActionNode
{
public:
    Observe(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::StatefulActionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("duration_ms", 3000, "观察持续时间（毫秒）") };
    }

    BT::NodeStatus onStart() override
    {   
        context_->motion_port->writeExact(&kStop, sizeof(kStop));
        
        
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
            context_->sensor_node, "/vision_node");
        if (!param_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(context_->logger, "[Observe] 无法连接到 vision_node");
            return BT::NodeStatus::FAILURE;
        }
        try
        {
            auto result = param_client_->set_parameters({rclcpp::Parameter("START_VISION", true)});
            if (!result[0].successful)
            {
                RCLCPP_ERROR(context_->logger, "[Observe] 启动视觉失败: %s", result[0].reason.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(context_->logger, "[Observe] 异常: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }
        int duration_ms = 3000;
        getInput<int>("duration_ms", duration_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(duration_ms);
        RCLCPP_INFO(context_->logger, "[Observe] 开始观察，持续 %d ms", duration_ms);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (chr::steady_clock::now() >= deadline_)
        {
            RCLCPP_INFO(context_->logger, "[Observe] 观察完成");
            return BT::NodeStatus::SUCCESS;
        }
        try
        {
            auto params = param_client_->get_parameters({"IS_GRIPPED"});
            if (!params.empty() && params[0].as_string() == "GRIPPED")
            {
                RCLCPP_INFO(context_->logger, "[Observe] 检测到目标已抓取");
                return BT::NodeStatus::SUCCESS;
            }
        }
        catch (const std::exception&) {}
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(context_->logger, "[Observe] 观察被中断");
    }

private:
    std::shared_ptr<AppContext> context_;
    std::shared_ptr<rclcpp::SyncParametersClient> param_client_;
    chr::steady_clock::time_point deadline_;
};

class GrabKFS final : public BT::StatefulActionNode
{
public:
    GrabKFS(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::StatefulActionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("duration_ms", 2000, "抓取动作持续时间（毫秒）") };
    }

    BT::NodeStatus onStart() override
    {
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
            context_->sensor_node, "/vision_node");
        if (!param_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(context_->logger, "[GrabKFS] 无法连接到 vision_node");
            return BT::NodeStatus::FAILURE;
        }
        int duration_ms = 2000;
        getInput<int>("duration_ms", duration_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(duration_ms);
        RCLCPP_INFO(context_->logger, "[GrabKFS] 开始抓取，持续 %d ms", duration_ms);
        if (context_->motion_port)
            context_->motion_port->writeExact(&kStop, sizeof(kStop));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        context_->motion_port->writeExact(&kStop, sizeof(kStop));
        if (chr::steady_clock::now() >= deadline_)
        {
            try
            {
                auto result = param_client_->set_parameters({rclcpp::Parameter("IS_GRIPPED", "GRIPPED")});
                if (result[0].successful)
                {
                    RCLCPP_INFO(context_->logger, "[GrabKFS] 抓取完成");
                    if (context_->motion_port)
                        context_->motion_port->writeExact(&kStop, sizeof(kStop));
                    return BT::NodeStatus::SUCCESS;
                }
                RCLCPP_ERROR(context_->logger, "[GrabKFS] 通知失败: %s", result[0].reason.c_str());
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(context_->logger, "[GrabKFS] 异常: %s", e.what());
            }
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(context_->logger, "[GrabKFS] 抓取被中断");
        if (context_->motion_port)
            context_->motion_port->writeExact(&kStop, sizeof(kStop));
    }

private:
    std::shared_ptr<AppContext> context_;
    std::shared_ptr<rclcpp::SyncParametersClient> param_client_;
    chr::steady_clock::time_point deadline_;
};

class CheckAvoid final : public BT::ConditionNode
{
public:
    CheckAvoid(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::ConditionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        context_->motion_port->writeExact(&kStop, sizeof(kStop));
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
            context_->sensor_node, "/vision_node");
        if (!param_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(context_->logger, "[CheckAvoid] 无法连接 vision_node");
            return BT::NodeStatus::FAILURE;
        }
        try
        {
            auto params = param_client->get_parameters({"Avoid"});
            if (!params.empty() && params[0].as_bool())
            {
                RCLCPP_INFO(context_->logger, "[CheckAvoid] 检测到 KFS");
                return BT::NodeStatus::SUCCESS;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(context_->logger, "[CheckAvoid] 异常: %s", e.what());
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<AppContext> context_;
};
 
class CheckIsGripped final : public BT::ConditionNode
{
public:
    CheckIsGripped(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::ConditionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("expected", 1, "期望值: 1=已抓取, 0=未抓取")
        };
    }

    BT::NodeStatus tick() override
    {
        context_->motion_port->writeExact(&kStop, sizeof(kStop));
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
            context_->sensor_node, "/vision_node");
        if (!param_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(context_->logger, "[CheckIsGripped] 无法连接 vision_node");
            return BT::NodeStatus::FAILURE;
        }
        int expected = 1;
        getInput<int>("expected", expected);
        try
        {
            auto params = param_client->get_parameters({"is_gripped"});
            if (!params.empty())
            {
                int val = static_cast<int>(params[0].as_int());
                RCLCPP_INFO(context_->logger,
                    "[CheckIsGripped] is_gripped=%d expected=%d", val, expected);
                return (val == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(context_->logger, "[CheckIsGripped] 异常: %s", e.what());
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<AppContext> context_;
};

class IsAtLine1 final : public BT::ConditionNode
{
public:
    IsAtLine1(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context) : BT::ConditionNode(name, config), context_(std::move(context))
        {}
        
        static BT::PortsList providedPorts()
        {
            return{};
        }

        BT::NodeStatus tick() override
        {
            int line;
            line = context_->line;
            if(line == 1)
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }

private:
    std::shared_ptr<AppContext> context_;
};


class IsAtLine2 final : public BT::ConditionNode
{
public:
    IsAtLine2(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::ConditionNode(name, config), context_(std::move(context))
        {}
        
        static BT::PortsList providedPorts()
        {
            return{};
        }

        BT::NodeStatus tick() override
        {
            int line;
            line = context_->line;
            if(line == 2)
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }

private:
    std::shared_ptr<AppContext> context_;
};

class IsAtLine3 final : public BT::ConditionNode
{
public:
    IsAtLine3(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::ConditionNode(name, config), context_(std::move(context))
        {}
        
        static BT::PortsList providedPorts()
        {
            return{};
        }

        BT::NodeStatus tick() override
        {
            int line;
            line = context_->line;
            if(line == 3)
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }

private:
    std::shared_ptr<AppContext> context_;
};


class IsAtTarget final : public BT::ConditionNode
{
public:
    IsAtTarget(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::ConditionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location", "home", "目标点位名称"),
            BT::InputPort<double>("tolerance", 0.2, "到达容差（米）")
        };
    }

    BT::NodeStatus tick() override
    {
        
        std::string loc_name;
        if (!getInput<std::string>("location", loc_name))
            return BT::NodeStatus::FAILURE;

        auto it = point_map.find(loc_name);
        if (it == point_map.end())
        {
            RCLCPP_ERROR(context_->logger, "[IsAtTarget] 未知点位: %s", loc_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        double tolerance = 0.2;
        getInput<double>("tolerance", tolerance);

        float dx = it->second.x - static_cast<float>(context_->sensor_node->currentX());
        float dy = it->second.y - static_cast<float>(context_->sensor_node->currentY());

        return std::hypot(dx, dy) < static_cast<float>(tolerance)
            ? BT::NodeStatus::SUCCESS
            : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<AppContext> context_;
};

class ReadAction final : public BT::StatefulActionNode
{
public:
    ReadAction(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::StatefulActionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("timeout_ms",      3000, "超时时间（毫秒）"),
            BT::OutputPort<int>("out_is_linked",        "收到的 is_linked 字段")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!context_ || !context_->link_monitor)
            throw BT::RuntimeError("link_monitor context is missing");

        int timeout_ms = 3000;
        getInput<int>("timeout_ms", timeout_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(timeout_ms);
        start_recv_ns_ = context_->link_monitor->lastRecvNs();

        RCLCPP_INFO(context_->logger,
            "[ReadAction] 等待新的 Signal2D 心跳，超时 %d ms", timeout_ms);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (context_->motion_port)
            context_->motion_port->writeExact(&kStop, sizeof(kStop));

        if (chr::steady_clock::now() >= deadline_)
        {
            RCLCPP_WARN(context_->logger, "[ReadAction] 超时");
            return BT::NodeStatus::FAILURE;
        }

        const int64_t now_recv = context_->link_monitor->lastRecvNs();
        if (now_recv == start_recv_ns_)
            return BT::NodeStatus::RUNNING;  // 还没收到新包

        const int is_linked = context_->link_monitor->lastIsLinked();
        RCLCPP_INFO(context_->logger,
            "[ReadAction] Signal2D: is_linked=%d", is_linked);
        setOutput("out_is_linked", is_linked);
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        RCLCPP_WARN(context_->logger, "[ReadAction] 被中断");
    }

private:
    std::shared_ptr<AppContext> context_;
    chr::steady_clock::time_point deadline_;
    int64_t start_recv_ns_ {0};
};

class WaitTillLinked final : public BT::StatefulActionNode
{
public:
    WaitTillLinked(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::StatefulActionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("timeout_ms", 10000, "超时时间（毫秒）")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!context_ || !context_->link_monitor)
            throw BT::RuntimeError("link_monitor context is missing");

        int timeout_ms = 10000;
        getInput<int>("timeout_ms", timeout_ms);
        deadline_ = chr::steady_clock::now() + chr::milliseconds(timeout_ms);

        RCLCPP_INFO(context_->logger,
            "[WaitTillLinked] 等待下位机连接信号，超时 %d ms", timeout_ms);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (context_->motion_port)
            context_->motion_port->writeExact(&kStop, sizeof(kStop));

        if (context_->link_monitor->isLinked())
        {
            RCLCPP_INFO(context_->logger, "[WaitTillLinked] 已连接");
            return BT::NodeStatus::SUCCESS;
        }

        if (chr::steady_clock::now() >= deadline_)
        {
            RCLCPP_WARN(context_->logger, "[WaitTillLinked] 超时，未收到连接信号");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(context_->logger, "[WaitTillLinked] 被中断");
    }

private:
    std::shared_ptr<AppContext> context_;
    chr::steady_clock::time_point deadline_;
};

class IsLinked final : public BT::ConditionNode
{
public:
    IsLinked(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::ConditionNode(name, config), context_(std::move(context))
    {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        if (!context_ || !context_->link_monitor)
            return BT::NodeStatus::FAILURE;
        return context_->link_monitor->isLinked()
            ? BT::NodeStatus::SUCCESS
            : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<AppContext> context_;
};

static void registerNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<AppContext>& context)
{
    factory.registerBuilder<MoveForward>(
        "MoveForward",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveForward>(name, config, context);
        });

    factory.registerBuilder<MoveBackward>(
        "MoveBackward",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveBackward>(name, config, context);
        });

    factory.registerBuilder<TurnLeft>(
        "TurnLeft",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<TurnLeft>(name, config, context);
        });

    factory.registerBuilder<TurnRight>(
        "TurnRight",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<TurnRight>(name, config, context);
        });

    factory.registerBuilder<Sync>(
        "Sync",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<Sync>(name, config, context);
        });

     factory.registerBuilder<Horizon>(
        "Horizon",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<Horizon>(name, config, context);
        });

    factory.registerBuilder<MoveToLocation>(
        "MoveToLocation",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocation>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationUP>(
        "MoveToLocationUP",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationUP>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationDOWN>(
        "MoveToLocationDOWN",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationDOWN>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationL>(
        "MoveToLocationL",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationL>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationUPL>(
        "MoveToLocationUPL",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationUPL>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationDOWNL>(
        "MoveToLocationDOWNL",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationDOWNL>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationHorizontal>(
        "MoveToLocationHorizontal",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationHorizontal>(name, config, context);
        });

    factory.registerBuilder<AdjustPosition>(
        "AdjustPosition",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<AdjustPosition>(name, config, context);
        });

    factory.registerBuilder<MoveUP>(
        "MoveUP",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveUP>(name, config, context);
        });

    factory.registerBuilder<MoveDOWN>(
        "MoveDOWN",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveDOWN>(name, config, context);
        });

    factory.registerBuilder<CloseVision>(
        "CloseVision",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<CloseVision>(name, config, context);
        });

    factory.registerBuilder<Observe>(
        "Observe",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<Observe>(name, config, context);
        });

    factory.registerBuilder<GrabKFS>(
        "GrabKFS",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GrabKFS>(name, config, context);
        });

    factory.registerBuilder<CheckAvoid>(
        "CheckAvoid",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<CheckAvoid>(name, config, context);
        });

    factory.registerBuilder<CheckIsGripped>(
        "CheckIsGripped",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<CheckIsGripped>(name, config, context);
        });

    factory.registerBuilder<AvoidKFS>(
        "AvoidKFS",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<AvoidKFS>(name, config, context);
        });

    factory.registerBuilder<IsAtTarget>(
        "IsAtTarget",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsAtTarget>(name, config, context);
        });

    factory.registerBuilder<IsAtLine1>(
        "IsAtLine1",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsAtLine1>(name, config, context);
        });

    factory.registerBuilder<IsAtLine2>(
        "IsAtLine2",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsAtLine2>(name, config, context);
        });

    factory.registerBuilder<IsAtLine3>(
        "IsAtLine3",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsAtLine3>(name, config, context);
        });

    factory.registerBuilder<ReadAction>(
        "ReadAction",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ReadAction>(name, config, context);
        });

    factory.registerBuilder<WaitTillLinked>(
        "WaitTillLinked",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<WaitTillLinked>(name, config, context);
        });

    factory.registerBuilder<IsLinked>(
        "IsLinked",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsLinked>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationA>(
        "MoveToLocationA",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationA>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationB>(
        "MoveToLocationB",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationB>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationC>(
        "MoveToLocationC",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationC>(name, config, context);
        });

    factory.registerBuilder<MoveToLocationD>(
        "MoveToLocationD",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveToLocationD>(name, config, context);
        });
}

int main(int argc, char** argv)
{
    /**
     * @brief 程序初始化逻辑
     * 
     */

    rclcpp::init(argc, argv);

    auto app_node = std::make_shared<rclcpp::Node>("gps_bt_app");
    app_node->declare_parameter<std::string>("tree_xml", "tree.xml");
    app_node->declare_parameter<std::string>("motion_port", "/dev/ttyUSB0");
    app_node->declare_parameter<std::string>("odom_topic", "/odom_corrected");
    app_node->declare_parameter<std::string>("imu_topic", "/livox/imu");
    app_node->declare_parameter<int>("tick_period_ms", 50);
    app_node->declare_parameter<int>("line" , 1);
    app_node->declare_parameter<int>("KFSloc",1);



    const auto tree_xml = app_node->get_parameter("tree_xml").as_string();
    const auto motion_port_path = app_node->get_parameter("motion_port").as_string();
    const auto odom_topic = app_node->get_parameter("odom_topic").as_string();
    const auto imu_topic = app_node->get_parameter("imu_topic").as_string();
    const auto tick_period_ms = app_node->get_parameter("tick_period_ms").as_int();
    const auto line = app_node->get_parameter("line").as_int();
    const auto kfs_loc = app_node->get_parameter("KFSloc").as_int();

    auto sensor_node = std::make_shared<SensorNode>(odom_topic, imu_topic);
    auto motion_port = std::make_shared<SerialPort>(motion_port_path, B115200, 0, 2);
    

    buildPointGraph();

    if (!motion_port->openPort())
    {
        RCLCPP_FATAL(
            app_node->get_logger(),
            "Failed to open motion port %s: %s",motion_port_path.c_str(),motion_port->lastError().c_str());
        rclcpp::shutdown();
        return 1;
    }

    auto link_monitor = std::make_shared<LinkMonitor>(
        motion_port, app_node->get_logger(), chr::milliseconds(500));
    link_monitor->start();

    auto context = std::make_shared<AppContext>();
    context->logger = app_node->get_logger();
    context->motion_port = motion_port;
    context->sensor_node = sensor_node;
    context->link_monitor = link_monitor;
    context->line = line;
    context->kfs_loc = kfs_loc;

    BT::BehaviorTreeFactory factory;
    registerNodes(factory, context);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("app_context", context);
    std::string tree_path = ament_index_cpp::get_package_share_directory("gps") + "/tree.xml";
    
    BT::Tree tree;
    try
    {
        tree = factory.createTreeFromFile(tree_path, blackboard);
    }
    catch (const std::exception& ex)
    {
        RCLCPP_FATAL(app_node->get_logger(), "Failed to create behavior tree: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }


    /**
     * @brief 程序执行器逻辑
     * 
     */

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(app_node);
    executor.add_node(sensor_node);

    RCLCPP_INFO(app_node->get_logger(), "Behavior tree started: %s", tree_xml.c_str());

    const auto sleep_duration = chr::milliseconds(std::max<long>(1, tick_period_ms));

    while (rclcpp::ok())
    {
        executor.spin_some();

        const BT::NodeStatus status = tree.tickOnce();
        
        if (status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(app_node->get_logger(), "Behavior tree completed with SUCCESS");
            break;
        }

        if (status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_WARN(app_node->get_logger(), "Behavior tree returned FAILURE");
        }

        std::this_thread::sleep_for(sleep_duration);
    }

    tree.haltTree();
    rclcpp::shutdown();
    return 0;
}