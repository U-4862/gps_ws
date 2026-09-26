#include "PurePursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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



