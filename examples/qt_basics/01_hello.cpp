// 例程 1：最小的 Qt 窗口
// 三个关键点：QApplication、控件、app.exec() 事件循环
#include <QApplication>
#include <QLabel>

int main(int argc, char** argv) {
    QApplication app(argc, argv);   // 每个 Qt GUI 程序有且只有一个

    QLabel label("你好，Qt！");       // 控件（Widget）
    label.setWindowTitle("例程 1");
    label.setAlignment(Qt::AlignCenter);
    label.resize(320, 120);
    label.show();                    // 只是“请求显示”，并不会卡住

    return app.exec();               // 进入事件循环，直到最后一个窗口关闭
}
