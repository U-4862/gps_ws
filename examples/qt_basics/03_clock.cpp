// 例程 3：自定义窗口类 + QTimer 定时刷新
// 这是“界面显示实时数据”的雏形：定时器周期性取数据 -> setText
#include <QApplication>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

// 继承 QMainWindow 做一个自己的窗口。
// Q_OBJECT 宏是信号槽/moc 机制需要的；写在 .cpp 里时文件末尾要 include 生成的 .moc
class ClockWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit ClockWindow(QWidget* parent = nullptr) : QMainWindow(parent) {
        auto* central = new QWidget(this);
        setCentralWidget(central);
        auto* layout = new QVBoxLayout(central);

        timeLabel_ = new QLabel("--:--:--", central);
        timeLabel_->setAlignment(Qt::AlignCenter);
        timeLabel_->setStyleSheet("font-size: 32px; font-family: Monospace;");
        auto* quitBtn = new QPushButton("退出", central);
        layout->addWidget(timeLabel_);
        layout->addWidget(quitBtn);

        connect(quitBtn, &QPushButton::clicked, this, &QWidget::close);

        timer_ = new QTimer(this);   // 父对象是 this -> 窗口析构时自动 delete
        connect(timer_, &QTimer::timeout, this, &ClockWindow::updateTime);
        timer_->start(1000);         // 每 1000 ms 触发一次 timeout
        updateTime();                // 先立即刷一次，避免启动时空白
    }

private slots:
    void updateTime() {
        timeLabel_->setText(QTime::currentTime().toString("hh:mm:ss"));
    }

private:
    QLabel* timeLabel_ = nullptr;
    QTimer* timer_ = nullptr;
};

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    ClockWindow window;
    window.setWindowTitle("例程 3：QTimer");
    window.resize(320, 180);
    window.show();
    return app.exec();
}

#include "03_clock.moc"   // AUTOMOC 生成，放在 .cpp 里的 Q_OBJECT 类必须加
