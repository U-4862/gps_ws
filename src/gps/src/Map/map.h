#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Parameter enum
 * 
 */
enum pn_signal{
    POS =0,
    NEG =1
};

enum grip_signal{
    LOOSE = 0,
    GRIP = 1,
};

enum up_signal{
    NORMAL = 0,
    UP = 1,
    DOWN = 2
};



// enum RFS_param{
//     none = 0,
//     true = 1,
//     fake = 2
// };


/**
 * @brief The Basic Signal Params
 * 
 */

struct Signal2D
{
    uint8_t header {0x0E};
    uint8_t is_linked {0};
};



/**
 * @brief The Basic Data Structures
 * 
 */

struct Pose2D
{
    uint8_t header {0x0F};
    uint8_t x {0};
    uint8_t y {0};
    uint8_t z {0};
    uint8_t pn {0};
    uint8_t grip_signal{0};
    uint8_t up_signal{0};
    uint8_t soap_signal{0};
};

struct Location
{
    float x{0.0f};
    float y{0.0f};
};

/// 点位图的一条有向边：to 为邻接点名，cost 为欧氏距离（米）
struct GraphEdge
{
    std::string to;
    double cost{0.0};
};


/**
 * @brief The Location Index
 * 
 */

 inline std::unordered_map<std::string, Location> point_map =
{
    {"restart_point", {0.0f, 0.0f}},
    {"weapon_chair", {1.0f, 0.5f}},
    {"home" ,{0.4f , 0.0f}},
    {"test1" , {1.0f , 1.3f}},
    
    {"test2" , {2.4f , 1.3f}},
    

    {"R6" , {4.8f, -2.7f}},

    {"R9" , {6.0f, -2.7f}},

    {"R12" , {7.2f, -2.7f}},
                                                                                                                                                          
    {"tran1" , {0.72f, 1.22f}},
    {"tran12" , {0.61f, 1.22f}},
    
    {"TGRA" ,{0.69F,0.55F}},
    {"tran2", {0.69f , 0.0f }},
    {"grab_pos1" , {2.2f , 0.2f}},
    {"grab_pos2" , {2.2f , -1.4f}},
    {"grab_pos2-b" , {1.8f , -1.4f}},
    {"grab_pos3" , {2.2f , -2.4f}},
 
    {"L1" , {3.2f,  0.2f}},
    {"L2" , {4.4f,  0.2f}},
    {"L3" , {5.6f,  0.2f}},
    {"L4" , {6.8f,  0.2f}},
    {"L-final" , {8.0f ,0.2f}},
    
    {"M1" , {3.2f, -1.4f}},
    {"M2" , {4.4f, -1.4f}},
    {"M3" , {5.6f, -1.4f}},
    {"M4" , {6.8f, -1.4f}},

    {"R1" , {3.2f, -2.4f}},
    {"R2" , {4.4f, -2.4f}},
    {"R3" , {5.6f, -2.4f}},
    {"R4" , {6.8f, -2.4f}},

    {"tran3" , {1.0f , 0.0f}},
    {"tran4" , {2.2f,0.0f}},
    {"tran5" , {3.0f , 0.0f}}
};


/// 邻接表；改为头文件内联定义（C++17 inline variable），不再需要 map.cpp
inline std::unordered_map<std::string, std::vector<GraphEdge>> point_graph;

namespace map_detail {

inline void addEdge(const std::string& from, const std::string& to)
{
    const auto a = point_map.find(from);
    const auto b = point_map.find(to);
    if (a == point_map.end() || b == point_map.end()) {
        return;
    }

    const double dx = static_cast<double>(a->second.x) - b->second.x;
    const double dy = static_cast<double>(a->second.y) - b->second.y;
    const double cost = std::hypot(dx, dy);

    point_graph[from].push_back({to, cost});
    point_graph[to].push_back({from, cost});
}

}  // namespace map_detail

/// 由 point_map 构建网格邻接表（L/M/R 三行 + 横向/纵向连接）
inline void buildPointGraph()
{
    point_graph.clear();

    static const char* const rows[3][4] = {
        {"L1", "L2", "L3", "L4"},
        {"M1", "M2", "M3", "M4"},
        {"R1", "R2", "R3", "R4"},
    };

    for (const auto& row : rows) {
        for (int i = 0; i < 3; ++i) {
            map_detail::addEdge(row[i], row[i + 1]);
        }
    }

    for (int col = 0; col < 4; ++col) {
        map_detail::addEdge(rows[0][col], rows[1][col]);
        map_detail::addEdge(rows[1][col], rows[2][col]);
    }

    // 入口点接到网格左端，保证从 tran5 能进入避障图
    map_detail::addEdge("tran5", "L1");
}
