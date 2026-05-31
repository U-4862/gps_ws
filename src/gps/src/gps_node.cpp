#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "SerialPort/usart.hpp"
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

struct Pose2D
{
    uint8_t header {0x0F};
    int8_t x {0};
    int8_t y {0};
    int8_t z {0};
};



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
            const std::string& radar_imu_topic = "/livox/imu"/*,
            const std::string& chassis_imu_topic = "/chassis/imu"*/)
        : rclcpp::Node("sensor_node")
    {
        const auto qos = rclcpp::SensorDataQoS();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/Odometry", qos,
            std::bind(&SensorNode::onOdometryReceived, this, _1));
        radar_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(radar_imu_topic, qos,
            std::bind(&SensorNode::onRadarImuReceived, this, _1));
        // chassis_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(chassis_imu_topic, qos,
        //     std::bind(&SensorNode::onChassisImuReceived, this, _1));
        RCLCPP_INFO(get_logger(),
            "Listening on /Odometry, radar IMU: %s",
            radar_imu_topic.c_str());
    }

    // Odometry
    double currentX() const { std::lock_guard<std::mutex> lock(odom_mutex_); return pose_.x; }
    double currentY() const { std::lock_guard<std::mutex> lock(odom_mutex_); return pose_.y; }
    double currentZ() const { std::lock_guard<std::mutex> lock(odom_mutex_); return pose_.z; }
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
    vector<pair<double, double>> obstacles;
     // 存储障碍物位置的示例
}

/**
 * @brief 应用程序上下文，包含共享资源如串口和tf监听器
 * 
 */
struct AppContext
{
    rclcpp::Logger logger {rclcpp::get_logger("gps_bt_app")};
    std::shared_ptr<SerialPort> motion_port;
    std::shared_ptr<SensorNode> sensor_node;
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
            {
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
        const Pose2D stop {};
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

class MoveForward final : public TimedVelocityAction
{
public:
    MoveForward(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), Pose2D{0x0f, 1, 0, 0})
    {}
};

class TurnLeft final : public TimedVelocityAction
{
public:
    TurnLeft(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), Pose2D{0x0f, 0, 0, 1})
    {
    }

    BT::NodeStatus onRunning() override
    {
        if(chr::steady_clock::now() < deadline_)
        {
            if (chr::steady_clock::now() < (deadline_) - chr::milliseconds(1500))
            {
                Pose2D start_command{0x0f, 0, 0, 3};
                if (!sendCommand(start_command))
                    {
                    RCLCPP_ERROR(
                        context_->logger,
                        "[%s] failed to send ##Start To Turn command: %s",
                        name().c_str(),
                        context_->motion_port->lastError().c_str());
                        return BT::NodeStatus::FAILURE;
                    }
                RCLCPP_DEBUG(
                    context_->logger,
                    "[%s] running: time left %ldms",
                    name().c_str(),
                    std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - std::chrono::steady_clock::now()).count());
            }
            else if (chr::steady_clock::now() < deadline_ - chr::milliseconds(1000))
            {
                
                if (!sendCommand(command_))
                    {
                    RCLCPP_ERROR(
                        context_->logger,
                        "[%s] failed to send turning command: %s",
                        name().c_str(),
                        context_->motion_port->lastError().c_str());
                        return BT::NodeStatus::FAILURE;
                    }
                RCLCPP_DEBUG(
                    context_->logger,
                    "[%s] running: time left %ldms",
                    name().c_str(),
                    std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - std::chrono::steady_clock::now()).count());
            }
            else if (chr::steady_clock::now() < deadline_ - chr::milliseconds(400))
            {
                Pose2D stop_command{0x0f, 0, 0, 0};
                if (!sendCommand(stop_command))
                    {
                    RCLCPP_ERROR(
                        context_->logger,
                        "[%s] failed to send stop command: %s",
                        name().c_str(),
                        context_->motion_port->lastError().c_str());
                        return BT::NodeStatus::FAILURE;
                    }
                if (!sendCommand(command_))
                    {
                    RCLCPP_ERROR(
                        context_->logger,
                        "[%s] failed to send turning command: %s",
                        name().c_str(),
                        context_->motion_port->lastError().c_str());
                        return BT::NodeStatus::FAILURE;
                    }
                RCLCPP_DEBUG(
                    context_->logger,
                    "[%s] running: time left %ldms",
                    name().c_str(),
                    std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - std::chrono::steady_clock::now()).count());
            }
            return BT::NodeStatus::RUNNING;
        }

        stopRobot();
        RCLCPP_INFO(context_->logger, "[%s] completed", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
};


class TurnRight final : public TimedVelocityAction
{
public:
    TurnRight(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), Pose2D{0x0f, 0, 0, -1})
    {
    }
};

/**
 * @brief judge position (待完善
 * 
 */

 class JudgePosition final : public TimedVelocityAction
{
public:
    JudgePosition(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), Pose2D{0x0f, 0, 0, 0})
    {}
};



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
static void registerNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<AppContext>& context)
{
    factory.registerBuilder<MoveForward>(
        "MoveForward",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveForward>(name, config, context);
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
    app_node->declare_parameter<std::string>("imu_topic", "/livox/imu");
    // app_node->declare_parameter<std::string>("chassis_imu_topic", "/chassis/imu");
    app_node->declare_parameter<int>("tick_period_ms", 50);

    const auto tree_xml = app_node->get_parameter("tree_xml").as_string();
    const auto motion_port_path = app_node->get_parameter("motion_port").as_string();
    const auto imu_topic = app_node->get_parameter("imu_topic").as_string();
    // const auto chassis_imu_topic = app_node->get_parameter("chassis_imu_topic").as_string();
    const auto tick_period_ms = app_node->get_parameter("tick_period_ms").as_int();

    auto sensor_node = std::make_shared<SensorNode>(imu_topic /*, chassis_imu_topic*/);
    auto motion_port = std::make_shared<SerialPort>(motion_port_path, B115200, 0, 2);

    if (!motion_port->openPort())
    {
        RCLCPP_FATAL(
            app_node->get_logger(),
            "Failed to open motion port %s: %s",motion_port_path.c_str(),motion_port->lastError().c_str());
        rclcpp::shutdown();
        return 1;
    }

    auto context = std::make_shared<AppContext>();
    context->logger = app_node->get_logger();
    context->motion_port = motion_port;
    context->sensor_node = sensor_node;

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