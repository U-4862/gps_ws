#ifndef ___UART_H___
#define ___UART_H___

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <iostream>
#include <termios.h>
#include <errno.h>   
#include <time.h>
#include <pthread.h>
#include <string.h>

#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>

#include <unistd.h>
#include <fcntl.h>


#define DEFAULT_UART_COM            "/dev/ttyS3" //环境探测器
#define DEFAULT_VMIN                5
#define DEFAULT_VTIME               6

class SerialPort
{

private:
    SerialPort(); //构造方法

public:
    // 串口打开关闭与读写
     int openUart(const char* path);

    //设置串口波特率
    int setSpeed(int fd, int speed);

    //设置串口的数据位、停止位、校验位。
    int setParity(int fd, int databits, int stopbits, char parity);

    //串口写入数据
    int writeUart(int fd, const void* data, size_t size);

    //读取串口数据
    int readUart(int fd, void* data, size_t size);
    
    //关闭串口
    int closeUart(int fd);

private:
    static SerialPort* m_instance_ptr; //单例变量

public:
    static pthread_mutex_t mutex;
    static SerialPort* getInstance(); //单例实例化函数
   
};


#endif // !___UART_H___