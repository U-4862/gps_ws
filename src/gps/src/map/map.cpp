#include "map.h"
#include <cmath>

std::unordered_map<std::string, std::vector<GraphEdge>> point_graph;

namespace
{
static float edgeWeight(const std::string& a, const std::string& b)
{
    auto ia = point_map.find(a);
    auto ib = point_map.find(b);
    if (ia == point_map.end() || ib == point_map.end())
        return 0.0f;
    float dx = ia->second.x - ib->second.x;
    float dy = ia->second.y - ib->second.y;
    return std::sqrt(dx * dx + dy * dy);
}

static void addUndirected(const std::string& a, const std::string& b)
{
    float w = edgeWeight(a, b);
    point_graph[a].push_back({b, w});
    point_graph[b].push_back({a, w});
}
} // namespace

/**
 * 12 点 3×4 网格邻接图。
 *
 *   坐标语义（world frame）:
 *     列 1..4 沿 +x 递增: 1 = 后, 4 = 前
 *     行 L/M/R 沿 y 递减: L = +y (左), M = 0, R = -y (右)
 *
 *          back <---------- x ----------> front
 *      +y  L1 - L2 - L3 - L4
 *          |    |    |    |
 *       0  M1 - M2 - M3 - M4
 *          |    |    |    |
 *      -y  R1 - R2 - R3 - R4
 *
 *   只连接同行相邻列 (前后邻居) + 同列相邻行 (左右邻居),
 *   权重为欧氏距离。运行时的 "机体左右" 由 yaw 决定,
 *   与本图连接方式无关 (AvoidKFS 用投影判断)。
 */
void buildPointGraph()
{
    point_graph.clear();

    const std::vector<std::vector<std::string>> grid = {
        {"L1", "L2", "L3", "L4"},
        {"M1", "M2", "M3", "M4"},
        {"R1", "R2", "R3", "R4"}
    };

    for (std::size_t r = 0; r < grid.size(); ++r)
    {
        for (std::size_t c = 0; c < grid[r].size(); ++c)
        {
            if (c + 1 < grid[r].size())
                addUndirected(grid[r][c], grid[r][c + 1]);
            if (r + 1 < grid.size())
                addUndirected(grid[r][c], grid[r + 1][c]);
        }
    }
}
