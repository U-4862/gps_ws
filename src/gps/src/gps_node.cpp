#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "SerialPort/usart.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <vector>

using std::placeholders::_1;
namespace chr = std::chrono;

struct Pose2D
{
    float x {0};
    float y {0};
    float z {0};
};

class SerialPort
{
public:
    SerialPort(
        std::string device,
        speed_t baud_rate = B115200,
        int vmin = 0,
        int vtime_ds = 2)
        : device_(std::move(device)),
          baud_rate_(baud_rate),
          vmin_(vmin),
          vtime_ds_(vtime_ds)
    {
    }

    ~SerialPort()
    {
        closePort();
    }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    SerialPort& operator=(SerialPort&&) = delete;

    bool openPort()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (fd_ >= 0)
        {
            return true;
        }

        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            last_error_ = "open(" + device_ + ") failed: " + std::string(std::strerror(errno));
            return false;
        }

        termios tty {};
        if (tcgetattr(fd_, &tty) != 0)
        {
            last_error_ = "tcgetattr(" + device_ + ") failed: " + std::string(std::strerror(errno));
            closeUnlocked();
            return false;
        }

        cfsetospeed(&tty, baud_rate_);
        cfsetispeed(&tty, baud_rate_);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        tty.c_iflag &= ~(INPCK | ISTRIP | PARMRK);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = static_cast<cc_t>(vmin_);
        tty.c_cc[VTIME] = static_cast<cc_t>(vtime_ds_);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            last_error_ = "tcsetattr(" + device_ + ") failed: " + std::string(std::strerror(errno));
            closeUnlocked();
            return false;
        }

        if (tcflush(fd_, TCIOFLUSH) != 0)
        {
            last_error_ = "tcflush(" + device_ + ") failed: " + std::string(std::strerror(errno));
            closeUnlocked();
            return false;
        }

        last_error_.clear();
        return true;
    }

    void closePort()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closeUnlocked();
    }

    bool isOpen() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return fd_ >= 0;
    }

    std::string lastError() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_error_;
    }

    bool writeExact(const void* data, std::size_t len)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (fd_ < 0)
        {
            last_error_ = "writeExact called on closed port: " + device_;
            return false;
        }

        const auto* bytes = static_cast<const std::uint8_t*>(data);
        std::size_t total_written = 0;

        while (total_written < len)
        {
            const ssize_t written = ::write(fd_, bytes + total_written, len - total_written);
            if (written < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                last_error_ = "write(" + device_ + ") failed: " + std::string(std::strerror(errno));
                return false;
            }

            total_written += static_cast<std::size_t>(written);
        }

        if (tcdrain(fd_) != 0)
        {
            last_error_ = "tcdrain(" + device_ + ") failed: " + std::string(std::strerror(errno));
            return false;
        }

        last_error_.clear();
        return true;
    }

    ssize_t readSome(void* buffer, std::size_t max_len)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (fd_ < 0)
        {
            last_error_ = "readSome called on closed port: " + device_;
            return -1;
        }

        const ssize_t n = ::read(fd_, buffer, max_len);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                return 0;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return 0;
            }

            last_error_ = "read(" + device_ + ") failed: " + std::string(std::strerror(errno));
            return -1;
        }

        last_error_.clear();
        return n;
    }

private:
    void closeUnlocked()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    std::string device_;
    speed_t baud_rate_ {B115200};
    int vmin_ {0};
    int vtime_ds_ {2};

    mutable std::mutex mutex_;
    int fd_ {-1};
    std::string last_error_;
};

class TFListenerNode : public rclcpp::Node
{
public:
    TFListenerNode()
        : rclcpp::Node("tf_listener")
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
        subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            qos,
            std::bind(&TFListenerNode::onTfReceived, this, _1));

        RCLCPP_INFO(get_logger(), "Listening on /tf");
    }

    double currentX() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_x_;
    }

    double currentY() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_y_;
    }

    double currentZ() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_z_;
    }

private:
    void onTfReceived(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        for (const auto& transform_stamped : msg->transforms)
        {
            const auto& trans = transform_stamped.transform.translation;
            current_x_ = trans.x;
            current_y_ = trans.y;
            current_z_ = trans.z;
        }
    }

    mutable std::mutex mutex_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    double current_x_ {0.0};
    double current_y_ {0.0};
    double current_z_ {0.0};
};

struct AppContext
{
    rclcpp::Logger logger {rclcpp::get_logger("gps_bt_app")};
    std::shared_ptr<SerialPort> motion_port;
    std::shared_ptr<SerialPort> sensor_port;
    std::shared_ptr<TFListenerNode> tf_listener;
};

class TimedVelocityAction : public BT::StatefulActionNode
{
public:
    TimedVelocityAction(
        const std::string& name,
        const BT::NodeConfig& config,
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
            BT::InputPort<int>("duration_ms", 220, "Action duration in milliseconds")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!context_ || !context_->motion_port)
        {
            throw BT::RuntimeError("motion serial port context is missing");
        }

        int duration_ms = 220;
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
            "[%s] started: x=%.3f y=%.3f z=%.3f",
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
        : TimedVelocityAction(name, config, std::move(context), Pose2D {0.1F, 0.0F, 0.0F})
    {
    }
};

class TurnLeft final : public TimedVelocityAction
{
public:
    TurnLeft(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : TimedVelocityAction(name, config, std::move(context), Pose2D {0.1F, 0.0F, 0.1F})
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
        : TimedVelocityAction(name, config, std::move(context), Pose2D {0.1F, 0.0F, -0.1F})
    {
    }
};

class DetectFront final : public BT::ConditionNode
{
public:
    DetectFront(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::ConditionNode(name, config),
          context_(std::move(context))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("expected_value", 1, "Expected front sensor byte"),
            BT::InputPort<int>("read_len", 1, "How many bytes to read")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!context_ || !context_->sensor_port)
        {
            throw BT::RuntimeError("sensor serial port context is missing");
        }

        int expected_value = 1;
        int read_len = 1;

        if (const auto value = getInput<int>("expected_value"))
        {
            expected_value = value.value();
        }

        if (const auto value = getInput<int>("read_len"))
        {
            read_len = value.value();
        }

        if (read_len <= 0 || read_len > 256)
        {
            throw BT::RuntimeError("read_len must be in range [1, 256]");
        }

        std::vector<std::uint8_t> buffer(static_cast<std::size_t>(read_len), 0U);
        const ssize_t n = context_->sensor_port->readSome(buffer.data(), buffer.size());

        if (n < 0)
        {
            RCLCPP_ERROR(
                context_->logger,
                "[%s] sensor read failed: %s",
                name().c_str(),
                context_->sensor_port->lastError().c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (n == 0)
        {
            RCLCPP_DEBUG(context_->logger, "[%s] no sensor data yet", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        const auto actual_value = static_cast<int>(buffer.front());
        const auto matched = (actual_value == expected_value);

        RCLCPP_INFO(
            context_->logger,
            "[%s] sensor=%d expected=%d result=%s",
            name().c_str(),
            actual_value,
            expected_value,
            matched ? "SUCCESS" : "FAILURE");

        return matched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<AppContext> context_;
};

class GrabDuantou final : public BT::SyncActionNode
{
public:
    GrabDuantou(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<AppContext> context)
        : BT::SyncActionNode(name, config),
          context_(std::move(context))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("message", "grab", "Diagnostic message")
        };
    }

    BT::NodeStatus tick() override
    {
        const auto msg = getInput<std::string>("message");
        if (!msg)
        {
            throw BT::RuntimeError("missing input [message]: ", msg.error());
        }

        RCLCPP_INFO(context_->logger, "[%s] %s", name().c_str(), msg->c_str());
        return BT::NodeStatus::SUCCESS;
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

    factory.registerBuilder<DetectFront>(
        "DetectFront",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<DetectFront>(name, config, context);
        });

    factory.registerBuilder<GrabDuantou>(
        "GrabDuantou",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GrabDuantou>(name, config, context);
        });
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto app_node = std::make_shared<rclcpp::Node>("gps_bt_app");
    app_node->declare_parameter<std::string>("tree_xml", "tree.xml");
    app_node->declare_parameter<std::string>("motion_port", "/dev/ttyUSB0");
    app_node->declare_parameter<std::string>("sensor_port", "/dev/ttyUSB1");
    app_node->declare_parameter<int>("tick_period_ms", 50);

    const auto tree_xml = app_node->get_parameter("tree_xml").as_string();
    const auto motion_port_path = app_node->get_parameter("motion_port").as_string();
    const auto sensor_port_path = app_node->get_parameter("sensor_port").as_string();
    const auto tick_period_ms = app_node->get_parameter("tick_period_ms").as_int();

    auto tf_listener = std::make_shared<TFListenerNode>();
    auto motion_port = std::make_shared<SerialPort>(motion_port_path, B115200, 0, 2);
    auto sensor_port = std::make_shared<SerialPort>(sensor_port_path, B115200, 0, 2);

    if (!motion_port->openPort())
    {
        RCLCPP_FATAL(
            app_node->get_logger(),
            "Failed to open motion port %s: %s",
            motion_port_path.c_str(),
            motion_port->lastError().c_str());
        rclcpp::shutdown();
        return 1;
    }

    if (!sensor_port->openPort())
    {
        RCLCPP_FATAL(
            app_node->get_logger(),
            "Failed to open sensor port %s: %s",
            sensor_port_path.c_str(),
            sensor_port->lastError().c_str());
        rclcpp::shutdown();
        return 1;
    }

    auto context = std::make_shared<AppContext>();
    context->logger = app_node->get_logger();
    context->motion_port = motion_port;
    context->sensor_port = sensor_port;
    context->tf_listener = tf_listener;

    BT::BehaviorTreeFactory factory;
    registerNodes(factory, context);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("app_context", context);

    BT::Tree tree;
    try
    {
        tree = factory.createTreeFromFile(tree_xml, blackboard);
    }
    catch (const std::exception& ex)
    {
        RCLCPP_FATAL(app_node->get_logger(), "Failed to create behavior tree: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(app_node);
    executor.add_node(tf_listener);

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