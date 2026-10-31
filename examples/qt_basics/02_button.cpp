// 例程 2：按钮 + 信号槽 + 布局
// 点按钮 -> 触发 clicked 信号 -> 槽函数（这里是 lambda）执行
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    int count = 0;   // 放在 window 之前，保证它比窗口活得久
    QWidget window;
    window.setWindowTitle("例程 2：信号与槽");

    // 布局负责摆放子控件，窗口缩放时自动重排
    auto* layout = new QVBoxLayout(&window);
    auto* label = new QLabel("点击次数：0", &window);
    auto* button = new QPushButton("点我 +1", &window);
    layout->addWidget(label);
    layout->addWidget(button);

    // connect(谁, 什么信号, 谁接收, 干什么)
    QObject::connect(button, &QPushButton::clicked, &window, [&count, label] {
        ++count;
        label->setText(QString("点击次数：%1").arg(count));
    });

    window.resize(280, 140);
    window.show();
    return app.exec();
}
