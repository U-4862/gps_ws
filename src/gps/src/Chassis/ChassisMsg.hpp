#pragma once

#include <cstdint>

struct Speed_Command{
    uint8_t header;
    float linear;
    float angular;
    uint8_t crc;
};

struct Wheel_Speed_Command{
    uint8_t header;
    float vx;
    float vy;
    float vw;
    uint16_t crc;
}; #_Pragma()
struct Wheel_Speed{
    float vx;
    float vy;
    float vw;
};

/* 以下为河工结构体   */

struct Velocity_Data
{
    float Vx;
    float Vy;
    float Vw;
};

// Speed Mode Enum

enum Chassis_Mode_Enum
{
    stop,
    fine_adjustment,
    low_speed,
    mid_speed

};

struct Chassis_Data{
    Chassis_Mode_Enum chassis_mode;   

    Velocity_Data input_world_velocity; // 输入世界坐标系速度（手柄给的速度）
    Velocity_Data planning_velocity;  // 规划之后的速度  让加减速更平滑

    Velocity_Data target_world_velocity; //目标世界坐标系速度
    Velocity_Data target_robot_velocity; //目标机器人坐标系速度


    /* 关于解算 */
    float wheel_cal_speed[4];   //轮向电机
    float steer_cal_speed[4];   //舵向电机解算出的角度
    float last_steer_cal_angle[4];  // 上一次计算的舵向角度
    float angle_err[4]; //角度误差数组
    float steer_back_angle[4]; //舵向电机反馈角度

    float wheel_radius; //轮半径
    float chassis_radius; //底盘半径
  
    float target_yaw; // 目标yaw角
    float current_yaw; //当前yaw角 赋值的时候必须为弧度制
    float err_yaw;

    float wheel_speed[4]; //四轮速度       （最终发给can任务的数据）
    float steer_angle[4]; //舵轮角度

    float increase_speed_temp; //加速阶段的加速度
    float reduce_speed_temp; //减速阶段的加速度

    uint8_t stop_flag;
    uint8_t park_flag;  //泊车标志位

    uint8_t radar_err_flag;  //雷达错误标志位
}; 


struct Chassis_Motor{
    float wheel_speed[4];
    float wheel_angle[4];
};


void coordinate_transform_world_to_robot(Velocity_Data *world_data, Velocity_Data *robot_data, float yaw);
 

