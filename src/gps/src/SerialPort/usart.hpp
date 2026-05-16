/**
 * @brief 串口模块的封装
 *  
 */


 struct Pose2D
{
    float x {0};
    float y {0};
    float z {0};
};

class SerialPort
{
public:
    SerialPort(std::string device, speed_t baud_rate = B115200,
               int vmin = 0, int vtime_ds = 2);
    ~SerialPort();

    //禁止拷贝
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    SerialPort& operator=(SerialPort&&) = delete;

    bool openPort();             // 只剩声明
    void closePort();
    bool isOpen() const;         // 简单函数，也可内联
    std::string lastError() const;

    /**
     * @brief 串口的读与写
     */
    bool writeExact(const void* data, std::size_t len);
    ssize_t readSome(void* buffer, std::size_t max_len);

private:
    void closeUnlocked();

    std::string device_;
    speed_t baud_rate_ {B115200};
    int vmin_ {0};
    int vtime_ds_ {2};

    mutable std::mutex mutex_;
    int fd_ {-1};
    std::string last_error_;
};


