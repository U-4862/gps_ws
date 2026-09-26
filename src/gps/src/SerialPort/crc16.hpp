/**
 * @brief CRC-16/MODBUS 校验
 *
 * 初值 0xFFFF，多项式 0xA001（0x8005 反射），右移实现。
 * 收发两侧复用同一份：接收时校验帧尾 CRC，发送时可为出帧计算 CRC。
 */

#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief 计算 CRC-16/MODBUS
 *
 * @param data   参与校验的字节起始地址
 * @param length 参与校验的字节数（通常为整帧去掉尾部 2 字节 CRC）
 * @return uint16_t 校验值
 */
inline uint16_t crc16(const uint8_t* data, std::size_t length)
{
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

