#include <unordered_map>
#include <cstdint>
#include <string>
/**
 * @brief Parameter enum
 * 
 */
enum pn_signal{
    POS =0,
    NEG =1
};

enum grip_param{
    LOOSE = 0,
    GRIPPED = 1,
};

enum stop_signal{
    NORMAL = 0,
    FORCE_STOP = 1
};

enum turn90_signal{
    NONE = 0,
    TURN_90 = 1
};

// enum RFS_param{
//     none = 0,
//     true = 1,
//     fake = 2
// };




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
    uint8_t stop_signal{0};
    uint8_t turn90_signal{0};
};

struct Location
{
    float x{0.0f};
    float y{0.0f};
};


/**
 * @brief The Location Index
 * 
 */

 inline std::unordered_map<std::string, Location> point_map =
{
    {"restart_point", {0.0f, 0.0f}},
    {"weapon_chair", {1.2f, 0.7f}},
    {"home" ,{0.4f , 0.0f}},
    {"test1" , {1.0f , 1.5f}},
    {"test2" , {2.57f , -1.6f}}
    
};

