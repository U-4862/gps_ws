#pragma once

#include <vector>

#include "Map/map.h"
#include "Chassis/SerialChassis.hpp"



struct Command {
    radar_msg msg {};            // 车体系摇杆量，可直接发串口
    float lateral_err {0.0f};    // 横向偏差 (m)，做偏离保护用
    bool  arrived {false};       // 到终点
};





/**
 * @brief 纯追踪 (Pure Pursuit) 路径跟踪器
 *
 * 输入当前位姿、路径点列表和当前线速度，输出跟踪路径所需的角速度 omega。
 * 但 串口全向轮底盘 接受的是 radar_msg 与 编码后的 
 * 【二、报文格式（30 字节，协议不能改，上位机照这个发）】

  偏移     长度   内容
  [0]       1     0xFF                     帧头
  [1..4]    4     x   (float, 小端)        → 底盘 Vx
  [5..8]    4     y   (float, 小端)        → 底盘 Vy
  [9..12]   4     z   (float, 小端)        → 底盘 Wz
  [13..16]  4     translation_distance (float, mm)  现在填 0
  [17..20]  4     high_err             (float, mm)  现在填 0
  [21..24]  4     front_err            (float, mm)  现在填 0
  [25]      1     route_flag                        填 0
  [26]      1     transmit_block_flag               填 0
  [27]      1     error_flag                        填 0（填 1 会点亮车上红灯/触发错误逻辑）
  [28]      1     校验和 = 第 [1]~[27] 字节逐个相加，取低 8 位
  [29]      1     0xFE                     帧尾
  
  因此需要在radar_msg 里赋值 

  现在是测试ver0.1 先写纯函数测试
 */



class PurePursuit
{
public:
    /// 当前位姿 (x, y, theta)，theta 为弧度
    struct State
    {
        float x {0.0f};
        float y {0.0f};
        float theta {0.0f};
    };

    /**
     * @param Ld_min 最小前瞻距离 (m)
     * @param k      速度对前瞻距离的增益 (Ld = max(Ld_min, k * v))
     */
    explicit PurePursuit(float Ld_min = 0.3f, float k = 1.0f)
        : Ld_min_(Ld_min), k_(k) {}



    Command compute(const State& state,
                const std::vector<Location>& path,
                float v,
                float arrive_tol = 0.15f) const;

    /**
     * @brief 计算跟踪路径所需的角速度
     *
     * @param state 当前位姿
     * @param path  路径点列表 (至少 1 个点)
     * @param v     当前线速度
     * @return 角速度 omega (正=左转)；路径为空或已到达时返回 0
     */
    float computeOmega(const State& state,
                       const std::vector<Location>& path,
                       float v) const;

private:
    float Ld_min_;
    float k_;

};
