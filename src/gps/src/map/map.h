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
    
    {"M2" ,{4.6f , 1.3f}},
   

    {"R3" , {4.6f, -2.7f}},
    

    {"R6" , {4.8f, -2.7f}},

    {"R9" , {6.0f, -2.7f}},

    {"R12" , {7.2f -2.7f}},

    {"tran1" , {0.69f, 0.65f}},
    {"TGRA" ,{0.69F,0.65F}},
    {"tran2", {0.69f , 0.0f }},
    {"tran3" , {1.0f , 0.0f}},
    {"tran4" , {2.2f,0.0f}},
    {"tran5" , {3.0f , 0.0f}}
};

