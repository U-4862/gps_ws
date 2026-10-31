/**
 * @file TestChassis.cpp
 * @brief 30 字节雷达/对接协议的编码与串口发送
 */

#include "TestChassis.hpp"

#include <cstring>

namespace test_chassis {

namespace {

// ---------------------------------------------------------------------------
// 30 字节帧的字节偏移表（协议来源：Docs/xcom_测试报文.txt）
//
// 纯编码细节，不往头文件放：外部只关心"给我一帧字节"，不关心字段落在第几位。
// 和下面的 encode() 一一对应，改协议只改这一张表。
// ---------------------------------------------------------------------------
namespace off {
constexpr std::size_t kHead          = 0;    // 0xFF
constexpr std::size_t kX             = 1;    // float 小端，摇杆量 ±150
constexpr std::size_t kY             = 5;
constexpr std::size_t kZ             = 9;
constexpr std::size_t kTransDist     = 13;   // mm
constexpr std::size_t kHighErr       = 17;   // mm
constexpr std::size_t kFrontErr      = 21;   // mm
constexpr std::size_t kRouteFlag     = 25;   // 注意：协议顺序与结构体不同
constexpr std::size_t kTransmitBlock = 26;
constexpr std::size_t kErrorFlag     = 27;
constexpr std::size_t kChecksum      = 28;   // Σ[1..27] & 0xFF
constexpr std::size_t kTail          = 29;   // 0xFE
}  // namespace off

// 校验覆盖范围：[1, 27]，不含帧头，也不含校验位自己
constexpr std::size_t kChecksumFirst = 1;
constexpr std::size_t kChecksumLast  = 27;

/// 写 4 字节小端 float —— 不依赖主机字节序，ARM/x86 都能对上电控
void put_le_f32(std::uint8_t* dst, float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));       // float -> 位模式
    dst[0] = static_cast<std::uint8_t>(bits & 0xFFu);
    dst[1] = static_cast<std::uint8_t>((bits >> 8) & 0xFFu);
    dst[2] = static_cast<std::uint8_t>((bits >> 16) & 0xFFu);
    dst[3] = static_cast<std::uint8_t>((bits >> 24) & 0xFFu);
}

}  // namespace


Frame encode(const radar_msg& msg)
{
    Frame f{};

    f[off::kHead] = kRadarFrameHead;

    // ---- 6 个 float：偏移与协议完全一致，逐字段写，不用 memcpy(&msg) ----
    put_le_f32(&f[off::kX],         msg.x);
    put_le_f32(&f[off::kY],         msg.y);
    put_le_f32(&f[off::kZ],         msg.z);
    put_le_f32(&f[off::kTransDist], msg.translation_distance);
    put_le_f32(&f[off::kHighErr],   msg.high_err);
    put_le_f32(&f[off::kFrontErr],  msg.front_err);

    // ---- 3 个标志位：注意协议顺序与结构体顺序不同！ ----
    // 结构体: error_flag, route_flag, transmit_block_flag
    // 协议  : route_flag, transmit_block_flag, error_flag
    f[off::kRouteFlag]     = msg.route_flag;
    f[off::kTransmitBlock] = msg.transmit_block_flag;
    f[off::kErrorFlag]     = msg.error_flag;

    // ---- 校验和：第 [1]~[27] 字节逐个相加（不含帧头、不含自己） ----
    std::uint8_t sum = 0;
    for (std::size_t i = kChecksumFirst; i <= kChecksumLast; ++i)
    {
        sum = static_cast<std::uint8_t>(sum + f[i]);
    }
    f[off::kChecksum] = sum;

    f[off::kTail] = kRadarFrameTail;
    return f;
}


std::uint8_t checksum_of(const Frame& frame)
{
    std::uint8_t sum = 0;
    for (std::size_t i = kChecksumFirst; i <= kChecksumLast; ++i)
    {
        sum = static_cast<std::uint8_t>(sum + frame[i]);
    }
    return sum;
}


bool frame_valid(const Frame& frame)
{
    if (frame[off::kHead] != kRadarFrameHead) return false;
    if (frame[off::kTail] != kRadarFrameTail) return false;
    return frame[off::kChecksum] == checksum_of(frame);
}

}  // namespace test_chassis


// ============================================================================
// TestChassis
// ============================================================================

TestChassis::TestChassis(const std::string& port,
                         speed_t baud_rate,
                         int vmin, int vtime_ds)
    : serial_(port, baud_rate, vmin, vtime_ds)
{
    // SerialPort 构造只记参数，不真正 open；这里补上，失败原因留在 last_error()
    serial_.openPort();
}


bool TestChassis::set_velocity(const radar_msg& msg)
{
    return send(msg);
}


bool TestChassis::set_velocity(float linear, float angular)
{
    // 通用接口：linear/angular 直接当协议里的 x / z（摇杆量）
    // 注意这里是"世界系速度"，需要机器人系时请自行做坐标变换后再调用
    radar_msg msg{};
    msg.x = linear;
    msg.z = angular;
    return send(msg);
}


void TestChassis::emergency_stop()
{
    radar_msg msg{};          // 全零：x=y=z=0，三个标志位都是 0
    (void)send(msg);          // 急停不在乎返回值，尽力发
}


Wheel_Speed TestChassis::get_wheel_speed()
{
    // 下行协议没有回传通道，这里只能返回零值。
    // 若将来电控加了上行帧（data_flag/nuc_flag 所在的协议），在这里补解析。
    return Wheel_Speed{0.0f, 0.0f, 0.0f};
}


bool TestChassis::send(const radar_msg& msg)
{
    const auto frame = test_chassis::encode(msg);
    last_frame_ = frame;

    if (!serial_.isOpen())
    {
        return false;
    }
    return serial_.writeExact(frame.data(), frame.size());
}
