// 例程 4：后台线程干活，界面不卡
// 核心：耗时的活儿放进 Worker（住在子线程），结果用信号发回界面线程
#include <QApplication>
#include <QLabel>
#include <QObject>
#include <QPushButton>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>

// 干活的对象：只负责计算，不碰界面
class Worker : public QObject {
    Q_OBJECT
public slots:
    void run() {
        for (int i = 1; i <= 5; ++i) {
            QThread::msleep(500);   // 模拟耗时操作
            emit progress(i);       // emit 是给信号用的“关键字”
        }
        emit done();
    }

signals:
    void progress(int value);       // 信号只要声明，不用实现
    void done();
};

class Window : public QWidget {
    Q_OBJECT

public:
    Window() {
        auto* layout = new QVBoxLayout(this);
        label_ = new QLabel("等待开始", this);
        button_ = new QPushButton("开始后台任务", this);
        layout->addWidget(label_);
        layout->addWidget(button_);

        // 1. 新建线程和 Worker
        thread_ = new QThread(this);
        worker_ = new Worker;
        // 2. 把 Worker 挪进子线程：之后 Worker 的槽函数都在子线程执行
        worker_->moveToThread(thread_);
        connect(thread_, &QThread::finished, worker_, &QObject::deleteLater);

        // 3. 按钮点击 -> Worker::run。接收者在别的线程，Qt 自动用队列连接，
        //    即：点击事件在界面线程，run() 在子线程，不会阻塞界面
        connect(button_, &QPushButton::clicked, this, [this] {
            button_->setEnabled(false);
            label_->setText("计算中...");
        });
        connect(button_, &QPushButton::clicked, worker_, &Worker::run);

        // 4. Worker 的信号 -> 界面更新。跨线程同样自动排队，回到界面线程执行
        connect(worker_, &Worker::progress, this, [this](int v) {
            label_->setText(QString("进度：%1 / 5").arg(v));
        });
        connect(worker_, &Worker::done, this, [this] {
            label_->setText("完成");
            button_->setEnabled(true);
        });

        thread_->start();   // 启动事件循环，Worker 的槽才有地方执行
    }

    ~Window() override {
        thread_->quit();
        thread_->wait();
    }

private:
    QLabel* label_ = nullptr;
    QPushButton* button_ = nullptr;
    QThread* thread_ = nullptr;
    Worker* worker_ = nullptr;
};

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    Window window;
    window.setWindowTitle("例程 4：后台线程 + 跨线程信号");
    window.resize(320, 160);
    window.show();
    return app.exec();
}

#include "04_worker.moc"
