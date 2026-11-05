#include "PurePursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


// PurePursuit.cpp
Command PurePursuit::compute(const State& s, const std::vector<Location>& path,
                             float v, float arrive_tol) const
{
    Command out;
    if (path.empty()) { 
        out.arrived = true;
        return out; 
        }

    // 1. 最近点 + 横向偏差
    float min_dist = std::numeric_limits<float>::infinity();
    std::size_t nearest = 0;
    for (std::size_t i = 0; i < path.size(); ++i) {
        const float d = std::hypot(path[i].x - s.x, path[i].y - s.y);
        if (d < min_dist) { min_dist = d; nearest = i; }
    }
    out.lateral_err = min_dist;

    // 2. 到达判断（终点 + 容差），放在算速度之前
    if (nearest == path.size() - 1 && min_dist < arrive_tol) {
        out.arrived = true;
        return out;                     // msg 全零 = 停车帧
    }

    // 3. 前瞻点
    const float Ld = std::max(Ld_min_, k_ * v);
    Location target = path.back();
    for (std::size_t i = nearest; i < path.size(); ++i) {
        if (std::hypot(path[i].x - s.x, path[i].y - s.y) >= Ld) { target = path[i]; break; }
    }

    // 4. 车体系
    const float dx = target.x - s.x, dy = target.y - s.y;
    const float lx =  dx * std::cos(s.theta) + dy * std::sin(s.theta);
    const float ly = -dx * std::sin(s.theta) + dy * std::cos(s.theta);

    const float Ld_act = std::hypot(lx, ly);
    if (Ld_act < 1e-3f) return out;     // 退化，保持零帧

    // 5. 全向轮：速度矢量指向前瞻点（不是 ω！）
    out.msg.x = chassis_protocol::world_vel_to_stick(v * lx / Ld_act);
    out.msg.y = chassis_protocol::world_vel_to_stick(v * ly / Ld_act);
    out.msg.z = 0.0f;
    return out;
}




float PurePursuit::computeOmega(const State& state,
                                const std::vector<Location>& path,
                                float v) const
{
    if (path.empty()) {
        return 0.0f;
    }

    const float x = state.x;
    const float y = state.y;
    const float theta = state.theta;

    // 1. 找到路径上离车最近的点
    float min_dist = std::numeric_limits<float>::infinity();
    std::size_t nearest_idx = 0;
    for (std::size_t i = 0; i < path.size(); ++i) {
        const float d = std::hypot(path[i].x - x, path[i].y - y);
        if (d < min_dist) {
            min_dist = d;
            nearest_idx = i;
        }
    }

    // 2. 从最近点往前搜索，找到距离 >= Ld 的前瞻点
    const float Ld = std::max(Ld_min_, k_ * v);
    Location target = path.back();  // 默认用终点
    for (std::size_t i = nearest_idx; i < path.size(); ++i) {
        const float d = std::hypot(path[i].x - x, path[i].y - y);
        if (d >= Ld) {
            target = path[i];
            break;
        }
    }

    // 3. 把前瞻点转换到车体坐标系
    const float dx = target.x - x;
    const float dy = target.y - y;
    const float local_x =  dx * std::cos(theta) + dy * std::sin(theta);
    const float local_y = -dx * std::sin(theta) + dy * std::cos(theta);


    // 4. 算曲率和角速度
    const float Ld_actual = std::hypot(local_x, local_y);
    if (Ld_actual < 0.01f) {
        return 0.0f;
    }
    const float curvature = 2.0f * local_y / (Ld_actual * Ld_actual);
    const float omega = v * curvature;

    return omega;

    
}



