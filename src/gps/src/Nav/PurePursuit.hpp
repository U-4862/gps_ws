#pragma once

#include <vector>

#include "../map/map.h"

/**
 * @brief 纯追踪 (Pure Pursuit) 路径跟踪器
 *
 * 输入当前位姿、路径点列表和当前线速度，输出跟踪路径所需的角速度 omega。
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
