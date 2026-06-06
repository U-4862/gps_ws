#include <unordered_map>
#include <cstdint>
#include <string>

struct Pose2D
{
    uint8_t header {0x0F};
    int8_t x {0};
    int8_t y {0};
    int8_t z {0};
    int8_t pn {0};
    int8_t grip_signal{0};
    int8_t stop_signal{0};
};

struct Location
{
    float x{0.0f};
    float y{0.0f};
};

 inline std::unordered_map<std::string, Location> point_map =
{
    {"restart_point", {0.0f, 0.0f}},
    {"weapon_chair", {1.2f, 0.7f}}
    
};