#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <array>


#include "SerialPort/usart.hpp"
#include "ChassisMsg.hpp"
#include "IChassis.hpp"



namespace chassis_protocol{
    inline constexpr std::size_t kRadarFrameLen = 30;
    inline constexpr std::uint8_t  kRadarFrameHead = 0xFF;
    inline constexpr std::uint8_t kRadarFrameTail = 0xfe;

    /**
     * @brief 摇杆量与速度的转换
     * 
     */

    inline constexpr float kStickFullScale   = 150.0f;
    inline constexpr float kWorldVelFullScale = 3.75f;

    inline float world_vel_to_stick(float mps)
    {
    return mps * (kStickFullScale / kWorldVelFullScale);
    }


    inline float stick_to_world_vel(float stick)
    {
    return stick * (kWorldVelFullScale / kStickFullScale);
    }



    inline uint16_t crc16(const uint8_t *data, std::size_t len)
    {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
    }
    



    using Frame = std::array< std::uint8_t ,kRadarFrameLen >;
    
    Frame encode(const radar_msg& radar_msg);

    std::uint8_t check_sum_of(const Frame& frame);

}



class SerialChassis : public IChassis {

public:
    explicit SerialChassis(const std::string &port, unsigned int baudrate); //显式声明底盘

    bool set_velocity(const radar_msg& msg);

    bool set_velocity(float linear, float angular) override;  //

    void emergency_stop() override;

    Wheel_Speed get_wheel_speed() override;

    chassis_protocol::Frame lastFrame() { return last_frame_; }

    std::string lastError() {return serial_.lastError(); }

    bool isOpen() {return serial_.isOpen(); }


private:
    bool send(const radar_msg & msg);

    chassis_protocol::Frame last_frame_  {};
    SerialPort serial_;
    
};


