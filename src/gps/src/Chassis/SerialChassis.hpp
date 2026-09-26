#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

#include "SerialPort/usart.hpp"
#include "IChassis.hpp"

// CRC-16/Modbus。注意:多项式(0xA001)必须与下位机(电控)约定一致,否则校验永远失败。
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

class SerialChassis : public IChassis {
    SerialPort serial_;

public:
    SerialChassis(const std::string &port, unsigned int baudrate)
        : serial_(port, baudrate) {}

    bool set_velocity(float linear, float angular) override
    {
       
    }

    void emergency_stop() override
    {
     
    }

    Wheel_Speed get_wheel_speed() override
    {
   
    }
};
