#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <behaviortree_cpp/bt_factory.h>
using std::placeholders::_1;

// 加载 打开串口并读取显示 用到的库
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>



struct pos{
    uint8_t start;
    float x;
    float y;
    float z;
    uint8_t end;
};

struct Pose2D
{
    float x;
    float y;
    float z;
};

enum FACING_DIRECTION{
    TOWARD =1,
    LEFT,
    RIGHT
};

unsigned turned_num;
unsigned kfs_gripped;
unsigned c1_gripped;


class Serial_1
{
    public:

    int fd;
    struct termios tty;
    char buffer[1024];
    ssize_t bytesRead;

    public:

    Serial_1()
    {
        fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);

        if (fd == -1)
        {
        std::cerr << "Error opening serial port." << std::endl;
        }

        if (tcgetattr(fd, &tty) != 0)
        {
        std::cerr << "Error getting serial port attributes." << std::endl;
        close(fd);
        }

        // 配置波特率
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // 5. 最重要的两个标志，几乎所有串口程序都要开
        tty.c_cflag |= (CLOCAL | CREAD);
        // 3. 关闭奇偶校验（Parity）
        tty.c_cflag &= ~PARENB;
        // 4. 设置 1 个停止位（默认就是 1 位，显式关闭 2 位模式）
        tty.c_cflag &= ~CSTOPB;
        // 1. 先清空数据位相关的旧设置（因为 CSIZE 占多个 bit，需要先清零再设置新值）
        tty.c_cflag &= ~CSIZE;
        // 2. 设置成 8 位数据
        tty.c_cflag |= CS8;

        //把输入变成非规范模式 + 无回显 + 无信号处理，这是“raw mode”的核心。
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        //确保输入是原始的 8 位二进制数据，不做任何校验或位剥离。
        tty.c_iflag &= ~(INPCK | ISTRIP);
        //让输出也变成完全原始的字节流，不被内核“帮忙”加工。
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2;

        // 行为解释：
        // 1. 调用 read 时，如果有数据，立即返回（不等待 200ms）。
        // 2. 如果没数据，内核会等待最多 200ms。
        //    - 期间来了数据：立即返回。
        //    - 200ms 到了还没数据：返回 0。
        // ==========================================

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            std::cerr << "Error setting serial port attributes: " << strerror(errno) << std::endl;
            close(fd);
            fd = -1;
        }

        // 清空旧数据，避免一上来就读到之前的残留
        tcflush(fd, TCIFLUSH);
    }

    ~Serial_1()
    {
        if (fd != -1)
        {
            close(fd);
            std::cout << "Serial port closed." << std::endl;
        }
    }

    ssize_t send(const void* data, size_t len)
    {
        if (fd == -1) return -1;

        ssize_t written = write(fd, data, len);
        if (written < 0)
        {
            std::cerr << "write failed: " << std::strerror(errno) << std::endl;
        }
        else if (static_cast<size_t>(written) != len)
        {
            std::cerr << "Partial write: " << written << " of " << len << " bytes" << std::endl;
        }

        tcdrain(fd);  // 等待所有数据真正发出（重要！）

        return written;
    }

    ssize_t receive_with_timeout(void* out_buffer, size_t max_len)
    {
        if (fd == -1) return -1;

        ssize_t n = read(fd, out_buffer, max_len);

        if (n < 0)
        {
            // 如果配置正确 (VMIN=0, VTIME=2)，通常不会遇到 EAGAIN，除非 open 用了 O_NONBLOCK
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 这种情况视为“超时/无数据”
                return 0;
            }
            std::cerr << "read error: " << strerror(errno) << std::endl;
            return -1;
        }

        // n == 0 表示超时 (VTIME 到期且无数据)
        // n > 0 表示读到了数据 (可能少于 max_len，因为只要有一个字节就会立即返回)
        return n;
    }

    

};


class Serial_2
{
    public:

    int fd;
    struct termios tty;
    char buffer[1024];
    ssize_t bytesRead;

    public:

    Serial_2()
    {
        fd = open("/dev/ttyUSB1", O_RDONLY | O_NOCTTY);

        if (fd == -1)
        {
        std::cerr << "Error opening serial port." << std::endl;
        }

        if (tcgetattr(fd, &tty) != 0)
        {
        std::cerr << "Error getting serial port attributes." << std::endl;
        close(fd);
        }

        // 配置波特率
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // 5. 最重要的两个标志，几乎所有串口程序都要开
        tty.c_cflag |= (CLOCAL | CREAD);
        // 3. 关闭奇偶校验（Parity）
        tty.c_cflag &= ~PARENB;
        // 4. 设置 1 个停止位（默认就是 1 位，显式关闭 2 位模式）
        tty.c_cflag &= ~CSTOPB;
        // 1. 先清空数据位相关的旧设置（因为 CSIZE 占多个 bit，需要先清零再设置新值）
        tty.c_cflag &= ~CSIZE;
        // 2. 设置成 8 位数据
        tty.c_cflag |= CS8;

        //把输入变成非规范模式 + 无回显 + 无信号处理，这是“raw mode”的核心。
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        //确保输入是原始的 8 位二进制数据，不做任何校验或位剥离。
        tty.c_iflag &= ~(INPCK | ISTRIP);
        //让输出也变成完全原始的字节流，不被内核“帮忙”加工。
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2;

        // 行为解释：
        // 1. 调用 read 时，如果有数据，立即返回（不等待 200ms）。
        // 2. 如果没数据，内核会等待最多 200ms。
        //    - 期间来了数据：立即返回。
        //    - 200ms 到了还没数据：返回 0。
        // ==========================================

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            std::cerr << "Error setting serial port attributes: " << strerror(errno) << std::endl;
            close(fd);
            fd = -1;
        }

        // 清空旧数据，避免一上来就读到之前的残留
        tcflush(fd, TCIFLUSH);
    }

    ~Serial_2()
    {
        if (fd != -1)
        {
            close(fd);
            std::cout << "Serial port closed." << std::endl;
        }
    }

    ssize_t send(const void* data, size_t len)
    {
        if (fd == -1) return -1;

        ssize_t written = write(fd, data, len);
        if (written < 0)
        {
            std::cerr << "write failed: " << std::strerror(errno) << std::endl;
        }
        else if (static_cast<size_t>(written) != len)
        {
            std::cerr << "Partial write: " << written << " of " << len << " bytes" << std::endl;
        }

        tcdrain(fd);  // 等待所有数据真正发出（重要！）

        return written;
    }

    ssize_t receive_with_timeout(void* out_buffer, size_t max_len)
    {
        if (fd == -1) return -1;

        ssize_t n = read(fd, out_buffer, max_len);

        if (n < 0)
        {
            // 如果配置正确 (VMIN=0, VTIME=2)，通常不会遇到 EAGAIN，除非 open 用了 O_NONBLOCK
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 这种情况视为“超时/无数据”
                return 0;
            }
            std::cerr << "read error: " << strerror(errno) << std::endl;
            return -1;
        }

        // n == 0 表示超时 (VTIME 到期且无数据)
        // n > 0 表示读到了数据 (可能少于 max_len，因为只要有一个字节就会立即返回)
        return n;
    }

    

};
Serial_1 serial_1;

Serial_2 serial_2;

namespace chr = std::chrono;

class DetectFront : public BT::StatefulActionNode
{
  public:
    
    DetectFront(const std::string& name, const BT::NodeConfig& config)
      : StatefulActionNode(name, config)
    {}

      
    static BT::PortsList providedPorts()
    {
        
        return{ };
    }

    // 节点开始时唤醒一次
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

  private:
    Pose2D _goal;
    chr::system_clock::time_point _completion_time;
};

//-------------------------

BT::NodeStatus DetectFront::onStart()
{
 
  _completion_time = chr::system_clock::now() + chr::milliseconds(220);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectFront::onRunning()
{
    
    return BT::NodeStatus::RUNNING;
}

void DetectFront::onHalted()
{
  printf("视觉信息传输失败");
}

class TFListener : public rclcpp::Node 
{
    public: 
    TFListener(): Node("tf_listener")
    {
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_z_ = 0.0;

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
        pos_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf",qos,std::bind(&TFListener::on_pose_received_, this, _1));
        RCLCPP_INFO(this->get_logger(), "开始监听 /tf 话题...");

    }

    double get_current_x() const { return current_x_; }
    double get_current_y() const { return current_y_; }
    double get_current_z() const { return current_z_; }

    public:
    void on_pose_received_(const tf2_msgs::msg::TFMessage::SharedPtr msg) 
    {   
        for (const auto& t : msg->transforms)
        {
            const auto& trans = t.transform.translation;

            current_x_ = trans.x;
            current_y_ = trans.y;
            current_z_ = trans.z;
            RCLCPP_INFO(this->get_logger(),
            "  translation      : x=%.6f  y=%.6f  z=%.6f",
            trans.x, trans.y, trans.z);

            const auto& rot = t.transform.rotation;
            RCLCPP_INFO(this->get_logger(),
            "  rotation (quat)  : x=%.6f  y=%.6f  z=%.6f  w=%.6f",
            rot.x, rot.y, rot.z, rot.w);
        }
        
    }


    private:
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pos_subscriber_;
        double current_x_{0.0};
        double current_y_{0.0};
        double current_z_{0.0};
        
};

    class MoveForward : public BT::StatefulActionNode
    {
        public:

        MoveForward(const std::string& name,const, BT::NodeConfig& config)
            :StatefulActionNode(name,config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

        private:
        Pose2D message;
        chr::system_clock::time_point _completion_time;

    }

    BT::NodeStatus MoveForward::onStart()
    {
        _completion_time = chr::system_clock::now() + chr::milliseconds(220);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus MoveForward::onRunning()
    {
        current_time = chr::system_clock.now();
        if(current_time < _completion_time)
        {
            Pose2D msg = {0.1, 0.0, 0.0};
            serial_1.send(&msg,sizeof(msg));
            return BT::NodeStatus::RUNNING;
        }
        else{
            return BT::NodeStatus::SUCCESS;
        }
    }



    class TurnLeft : public BT::StatefulActionNode
    {
        public:

        TurnLeft(const std::string& name,const, BT::NodeConfig& config)
            :StatefulActionNode(name,config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

        private:
        Pose2D message;
        chr::system_clock::time_point _completion_time;

    }

    BT::NodeStatus TurnLeft::onStart()
    {
        _completion_time = chr::system_clock::now() + chr::milliseconds(220);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus TurnLeft::onRunning()
    {
        current_time = chr::system_clock.now();
        if(current_time < _completion_time)
        {
            Pose2D msg = {0.1, 0.0, 0.1};
            serial_1.send(&msg,sizeof(msg));
            return BT::NodeStatus::RUNNING;
        }
        else{
            return BT::NodeStatus::SUCCESS;
        }
    }

    class TurnRight : public BT::StatefulActionNode
    {
        public:

        TurnRight(const std::string& name,const, BT::NodeConfig& config)
            :StatefulActionNode(name,config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

        private:
        Pose2D message;
        chr::system_clock::time_point _completion_time;

    }

    BT::NodeStatus TurnRight::onStart()
    {
        _completion_time = chr::system_clock::now() + chr::milliseconds(220);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus TurnRight::onRunning()
    {
        current_time = chr::system_clock.now();
        if(current_time < _completion_time)
        {
            Pose2D msg = {0.1, 0.0, -0.1};
            serial_1.send(&msg,sizeof(msg));
            return BT::NodeStatus::RUNNING;
        }
        else{
            return BT::NodeStatus::SUCCESS;
        }
    }

class Grab_duantou : public BT::SyncActionNode
{
    public:
    Grab_duantou(const std::string& name , const BT::NodeConfig& config )
    : BT::SyncActionNode(name, config){}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("message")}; 
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> msg = getInput<std::string>("message");
        if(!msg)
        {
            throw BT::RuntimeError("missing required input [message]: ", msg.error());
        }
        std::cout << "Robot says: " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class Detect_front : public BT::SyncActionNode
{
    public:
    Detect_front(const std::string& name , const BT::NodeConfig& config )
    : BT::SyncActionNode(name, config){}

    static BT::PortsList providedPorts()
    {
        uint8_t return_status = 0;

        serial_1.receive_with_timeout();
        return {BT::InputPort<std::string>("message")};
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> msg = getInput<std::string>("message");
        if(!msg)
        {
            throw BT::RuntimeError("missing required input [message]: ", msg.error());
        }
        std::cout << "Robot says: " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<DetectFront>("DetectFront"); 
    factory.registerNodeType<MoveForward>("MoveForward");
    factory.registerNodeType<TurnLeft>("TurnLeft");
    factory.registerNodeType<TurnRight>("TurnRight");

    
    auto tree = factory.createTreeFromFile("tree.xml");
    std::cout << "Starting to tick the tree..." << std::endl;

    // std::cout << "Starting to tick the tree..." << std::endl;
    // tree.tickWhileRunning();
    while (rclcpp::ok()) {
        rclcpp::spin(node);
        // BT::NodeStatus status = tree.tickExactlyOnce();
    }
    // std::cout << "Tree execution finished!" << std::endl;
    
    rclcpp::shutdown();
    return 0;
}