// 例程 5：QProcess 启动外部命令并显示输出
// 这就是现有 launcher 的核心：按钮 -> 起进程 -> 读 stdout -> 显示
#include <QApplication>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QScrollBar>
#include <QTextCursor>
#include <QVBoxLayout>
#include <QWidget>

class ProcessWindow : public QWidget {
    Q_OBJECT

public:
    ProcessWindow() {
        auto* layout = new QVBoxLayout(this);

        log_ = new QPlainTextEdit(this);
        log_->setReadOnly(true);
        log_->setMaximumBlockCount(1000);   // 只留最近 1000 行，防止内存一直涨

        auto* startBtn = new QPushButton("启动（每秒打印一行）", this);
        auto* stopBtn = new QPushButton("停止", this);
        layout->addWidget(log_);
        layout->addWidget(startBtn);
        layout->addWidget(stopBtn);

        process_ = new QProcess(this);
        process_->setProcessChannelMode(QProcess::MergedChannels);  // stderr 合并进 stdout

        connect(process_, &QProcess::readyRead, this, &ProcessWindow::readOutput);
        connect(process_,
                QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this](int code, QProcess::ExitStatus) {
                    log_->appendPlainText(QString("[进程退出] code=%1").arg(code));
                });

        connect(startBtn, &QPushButton::clicked, this, [this] {
            if (process_->state() != QProcess::NotRunning) return;
            log_->appendPlainText("[启动]");
            // 用 bash -c 执行一段脚本，模拟 ros2 launch 这类长时间运行的程序
            process_->start("/bin/bash", {"-c",
                "for i in $(seq 1 100); do echo \"tick $i\"; sleep 1; done"});
        });

        connect(stopBtn, &QPushButton::clicked, this, [this] {
            if (process_->state() == QProcess::NotRunning) return;
            process_->terminate();                      // 先礼貌地 SIGTERM
            if (!process_->waitForFinished(2000))       // 2 秒还不退
                process_->kill();                       // 再强杀
        });
    }

private slots:
    void readOutput() {
        const QByteArray data = process_->readAll();
        if (data.isEmpty()) return;
        log_->moveCursor(QTextCursor::End);
        log_->insertPlainText(QString::fromLocal8Bit(data));
        log_->verticalScrollBar()->setValue(log_->verticalScrollBar()->maximum());
    }

private:
    QProcess* process_ = nullptr;
    QPlainTextEdit* log_ = nullptr;
};

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    ProcessWindow window;
    window.setWindowTitle("例程 5：QProcess");
    window.resize(560, 360);
    window.show();
    return app.exec();
}

#include "05_qprocess.moc"
