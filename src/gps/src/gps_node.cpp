#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
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
    Pose2D * nextPose1;
    Pose2D * nextPose2;
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

/**
 * @brief MID360 FastLIO 传感器监听器，同时订阅 /tf 和 IMU 话题
 */
class SensorNode : public rclcpp::Node
{
public:
    explicit SensorNode(const std::string& imu_topic = "/livox/imu")
        : rclcpp::Node("sensor_node")
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
        tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", qos,
            std::bind(&SensorNode::onTfReceived, this, _1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos,
            std::bind(&SensorNode::onImuReceived, this, _1));
        RCLCPP_INFO(get_logger(), "Listening on /tf and %s", imu_topic.c_str());
    }

    // TF
    double currentX() const { std::lock_guard<std::mutex> lock(tf_mutex_); return current_x_; }
    double currentY() const { std::lock_guard<std::mutex> lock(tf_mutex_); return current_y_; }
    double currentZ() const { std::lock_guard<std::mutex> lock(tf_mutex_); return current_z_; }

    // IMU
    ImuData imuData() const { std::lock_guard<std::mutex> lock(imu_mutex_); return imu_; }

private:
    void onTfReceived(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        for (const auto& transform_stamped : msg->transforms)
        {
            const auto& trans = transform_stamped.transform.translation;
            current_x_ = trans.x;
            current_y_ = trans.y;
            current_z_ = trans.z;
        }
    }

    void onImuReceived(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_.ori_x = msg->orientation.x;
        imu_.ori_y = msg->orientation.y;
        imu_.ori_z = msg->orientation.z;
        imu_.ori_w = msg->orientation.w;
        imu_.ang_vel_x = msg->angular_velocity.x;
        imu_.ang_vel_y = msg->angular_velocity.y;
        imu_.ang_vel_z = msg->angular_velocity.z;
        imu_.lin_acc_x = msg->linear_acceleration.x;
        imu_.lin_acc_y = msg->linear_acceleration.y;
        imu_.lin_acc_z = msg->linear_acceleration.z;
    }

    // TF
    mutable std::mutex tf_mutex_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    double current_x_ {0.0}, current_y_ {0.0}, current_z_ {0.0};

    // IMU
    mutable std::mutex imu_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    ImuData imu_;
};

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

private:
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
        : TimedVelocityAction(name, config, std::move(context), Pose2D {0x0f,1, 0 ,0 })
    {}
};

class TurnLeft final : public TimedVelocityAction
{
public:
    TurnLeft(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), Pose2D {0x0f,0, 0 ,1 })
    {
    }
};

class TurnRight final : public TimedVelocityAction
{
public:
    TurnRight(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), Pose2D {0x0f,0, 0 ,-1})
    {
    }
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
    app_node->declare_parameter<int>("tick_period_ms", 50);

    const auto tree_xml = app_node->get_parameter("tree_xml").as_string();
    const auto motion_port_path = app_node->get_parameter("motion_port").as_string();
    const auto imu_topic = app_node->get_parameter("imu_topic").as_string();
    const auto tick_period_ms = app_node->get_parameter("tick_period_ms").as_int();

    auto sensor_node = std::make_shared<SensorNode>(imu_topic);
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