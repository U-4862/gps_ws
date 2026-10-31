// 例程 6：Qt 界面显示 ROS2 里程计（/Odometry）
// 关键：ROS 的 spin 会阻塞，绝不能放在界面线程；用 QThread + 信号把数据送回界面
#include <QApplication>
#include <QLabel>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

// 住在子线程里的 ROS 节点：只收数据、发信号，不碰任何界面
class RosWorker : public QObject {
    Q_OBJECT
public:
    RosWorker() {
        node_ = std::make_shared<rclcpp::Node>("qt_pose_panel");
        sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // 这个回调在 spin 线程执行；emit 之后 Qt 自动排队到界面线程
                emit poseUpdated(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y);
            });
    }

    // 在子线程里阻塞运行，直到 rclcpp::shutdown()
    void spin() { rclcpp::spin(node_); }

    void stop() { rclcpp::shutdown(); }

signals:
    void poseUpdated(double x, double y);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

class Window : public QWidget {
    Q_OBJECT
public:
    Window() {
        auto* layout = new QVBoxLayout(this);
        label_ = new QLabel("等待 /Odometry ...", this);
        label_->setAlignment(Qt::AlignCenter);
        label_->setStyleSheet("font-size: 22px; font-family: Monospace;");
        layout->addWidget(label_);

        thread_ = new QThread(this);
        worker_ = new RosWorker;
        worker_->moveToThread(thread_);

        connect(thread_, &QThread::started, worker_, &RosWorker::spin);
        connect(worker_, &RosWorker::poseUpdated, this,
                [this](double x, double y) {
                    label_->setText(QString("x = %1 m\ny = %2 m")
                                        .arg(x, 0, 'f', 3)
                                        .arg(y, 0, 'f', 3));
                });

        thread_->start();
    }

    ~Window() override {
        worker_->stop();      // 让 rclcpp::spin 返回
        thread_->quit();
        thread_->wait();
        delete worker_;
    }

private:
    QLabel* label_ = nullptr;
    QThread* thread_ = nullptr;
    RosWorker* worker_ = nullptr;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    Window window;
    window.setWindowTitle("例程 6：ROS2 位姿面板");
    window.resize(360, 180);
    window.show();
    return app.exec();
}

#include "06_ros2_pose.moc"
