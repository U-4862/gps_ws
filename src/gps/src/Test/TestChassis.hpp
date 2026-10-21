#pragma once
/**
 * @file TestChassis.hpp
 * @brief 用 radar_msg 与下位机通信的测试底盘实现（30 字节协议）
 *
 * 协议来源：Docs/xcom_测试报文.txt
 *
 *   [0]      0xFF                        帧头
 *   [1..4]   x                    (float, 小端)  底盘 Vx（摇杆量，满量程 ±150）
 *   [5..8]   y                    (float, 小端)  底盘 Vy
 *   [9..12]  z                    (float, 小端)  底盘 Wz
 *   [13..16] translation_distance (float, 小端, mm)
 *   [17..20] high_err             (float, 小端, mm)
 *   [21..24] front_err            (float, 小端, mm)
 *   [25]     route_flag
 *   [26]     transmit_block_flag
 *   [27]     error_flag
 *   [28]     校验和 = (第 1~27 字节逐个相加) & 0xFF
 *   [29]     0xFE                        帧尾
 *
 * ⚠ radar_msg 不能直接 memcpy 发：
 *   1. 结构体 32 字节，协议帧 30 字节
 *   2. 三个标志位顺序不同（结构体是 error/route/transmit，协议是 route/transmit/error）
 *   3. data_flag / nuc_flag 不在下行协议里
 *   4. 校验是累加和，不是 CRC16
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "Chassis/IChassis.hpp"
#include "Chassis/ChassisMsg.hpp"
#include "SerialPort/usart.hpp"

namespace test_chassis {

// ---- 协议常量 --------------------------------------------------------------
inline constexpr std::size_t kRadarFrameLen  = 30;
inline constexpr std::uint8_t kRadarFrameHead = 0xFF;
inline constexpr std::uint8_t kRadarFrameTail = 0xFE;

// 协议标注的字节偏移，和 encode() 一一对应，改协议时只改这里
namespace off {
inline constexpr std::size_t kHead          = 0;
inline constexpr std::size_t kX             = 1;
inline constexpr std::size_t kY             = 5;
inline constexpr std::size_t kZ             = 9;
inline constexpr std::size_t kTransDist     = 13;
inline constexpr std::size_t kHighErr       = 17;
inline constexpr std::size_t kFrontErr      = 21;
inline constexpr std::size_t kRouteFlag     = 25;
inline constexpr std::size_t kTransmitBlock = 26;
inline constexpr std::size_t kErrorFlag     = 27;
inline constexpr std::size_t kChecksum      = 28;
inline constexpr std::size_t kTail          = 29;
}  // namespace off

// 校验覆盖范围：[1, 27]（不含帧头）
inline constexpr std::size_t kChecksumFirst = 1;
inline constexpr std::size_t kChecksumLast  = 27;

// ---- 量纲换算 --------------------------------------------------------------
// 协议：x/y/z 是"摇杆量"，满量程 ±150 对应目标世界速度 ±3.75
inline constexpr float kStickFullScale   = 150.0f;
inline constexpr float kWorldVelFullScale = 3.75f;

/// m/s -> 摇杆量（×40）
inline float world_vel_to_stick(float mps)
{
    return mps * (kStickFullScale / kWorldVelFullScale);
}

/// 摇杆量 -> m/s
inline float stick_to_world_vel(float stick)
{
    return stick * (kWorldVelFullScale / kStickFullScale);
}

using Frame = std::array<std::uint8_t, kRadarFrameLen>;

/// 把 radar_msg 编码成 30 字节协议帧（纯函数，可离线单测）
Frame encode(const radar_msg& msg);

/// 算累加和校验（对 [1..27] 求和取低 8 位）
std::uint8_t checksum_of(const Frame& frame);

/// 校验一帧是否合法（帧头/帧尾/校验和）
bool frame_valid(const Frame& frame);

}  // namespace test_chassis


// ============================================================================
// TestChassis —— 可直接吃 radar_msg 的底盘实现
//
// 用法：
//     TestChassis chassis("/dev/ttyUSB0", B115200);
//     radar_msg m{};
//     m.x = test_chassis::world_vel_to_stick(0.5f);   // 0.5 m/s -> 20
//     chassis.set_velocity(m);                        // 直接引用传，零拷贝
// ============================================================================
class TestChassis final : public IChassis {
public:
    /// @param port      串口设备，如 "/dev/ttyUSB0"
    /// @param baud_rate termios 波特率，默认 B115200（对应协议 115200 8N1）
    explicit TestChassis(const std::string& port,
                         speed_t baud_rate = B115200,
                         int vmin = 0, int vtime_ds = 2);

    // ---- 核心接口：直接接收 radar_msg（const 引用，零拷贝） ----------------
    /// 把 msg 编码成 30 字节帧并整帧写出。
    /// @note msg.x/y/z 请填**摇杆量**（±150），不是 m/s；用 world_vel_to_stick() 换算。
    /// @note msg.translation_distance/high_err/front_err 按协议是 **mm**。
    /// @note msg.data_flag / nuc_flag **不会**被发送（下行协议没这两个字段）。
    /// @return 写串口成功与否
    bool set_velocity(const radar_msg& msg);

    // ---- IChassis 通用接口 -------------------------------------------------
    /// @param linear  协议里的 x（摇杆量，±150）
    /// @param angular 协议里的 z（摇杆量，±150）
    bool set_velocity(float linear, float angular) override;

    /// 发一帧全零（停车），并把 error_flag 置 0
    void emergency_stop() override;

    /// 这个下行协议没有回传通道，恒返回全零。
    /// 要读底盘状态需要另外的接收链路（data_flag / nuc_flag 暗示存在，但文档未定义）。
    Wheel_Speed get_wheel_speed() override;

    // ---- 调试辅助 ----------------------------------------------------------
    /// 最近一次发出的帧（未发出成功时为空）
    const test_chassis::Frame& last_frame() const { return last_frame_; }
    bool is_open() const { return serial_.isOpen(); }
    std::string last_error() const { return serial_.lastError(); }

private:
    /// 编码 + 写串口 + 记下最后一帧
    bool send(const radar_msg& msg);

    SerialPort serial_;
    test_chassis::Frame last_frame_{};
};
