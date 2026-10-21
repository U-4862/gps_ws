/**
 * @file test.cpp
 * @brief TestChassis 离线自测：用 Docs/xcom_测试报文.txt 里的现成报文做逐字节比对
 *
 * 这个测试**不需要接硬件、不需要开串口**，纯算编码结果。
 * 跑通就说明字段映射、标志位顺序、校验和全部和电控一致。
 *
 *   colcon build --packages-select gps
 *   ./install/gps/lib/gps/test_chassis                      # 纯离线自测
 *   ./install/gps/lib/gps/test_chassis --live /dev/ttyUSB0  # 真发 60 帧（20Hz×3s，摇杆量 20）
 *   ./install/gps/lib/gps/test_chassis --live /dev/ttyUSB0 50 5   # 摇杆量 50，发 5 秒
 *
 * 退出码：
 *   0  自测（及实发）全部成功
 *   1  自测有失败项
 *   2  实发阶段失败：设备不存在 / 无权限 / 打不开 / 写失败
 *   （实发失败优先于自测失败返回，方便脚本区分两类问题）
 */

#include "TestChassis.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <glob.h>
#include <unistd.h>

using test_chassis::Frame;
using test_chassis::kRadarFrameLen;

namespace {

int g_pass = 0;
int g_fail = 0;

/// 把 "FF 00 00 16 43 ..." 这种文本解析成字节数组
std::vector<std::uint8_t> parse_hex(const std::string& text)
{
    std::vector<std::uint8_t> bytes;
    unsigned int value = 0;
    bool half = false;
    for (char c : text)
    {
        int digit = -1;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        else continue;

        if (!half)
        {
            value = static_cast<unsigned int>(digit);
            half = true;
        }
        else
        {
            value = (value << 4) | static_cast<unsigned int>(digit);
            bytes.push_back(static_cast<std::uint8_t>(value & 0xFFu));
            half = false;
        }
    }
    return bytes;
}

std::string to_hex(const std::uint8_t* data, std::size_t len)
{
    static const char* kDigits = "0123456789ABCDEF";
    std::string out;
    out.reserve(len * 3);
    for (std::size_t i = 0; i < len; ++i)
    {
        if (i) out.push_back(' ');
        out.push_back(kDigits[(data[i] >> 4) & 0x0F]);
        out.push_back(kDigits[data[i] & 0x0F]);
    }
    return out;
}

/// 用 glob() 展开一个模式，返回匹配到的路径（不存在就是空）
std::vector<std::string> glob_paths(const char* pattern)
{
    std::vector<std::string> out;
    glob_t matches{};
    if (::glob(pattern, 0, nullptr, &matches) == 0)
    {
        for (std::size_t i = 0; i < matches.gl_pathc; ++i)
        {
            out.emplace_back(matches.gl_pathv[i]);
        }
    }
    ::globfree(&matches);          // globfree 对未初始化/无匹配也安全
    return out;
}
                        
/// 开串口前体检：设备在不在、有没有读写权限。
/// 不存在时把「系统里现在有哪些串口」列出来，避免只有一句 No such file or directory。
/// @return true = 可以尝试 open()
bool preflight_port(const std::string& port)
{
    if (::access(port.c_str(), F_OK) == 0)
    {
        if (::access(port.c_str(), R_OK | W_OK) == 0)
        {
            return true;
        }

        std::printf("\n✗ 串口存在但没有读写权限：%s\n", port.c_str());
        std::printf("  当前会话不在 dialout 组，或者加组后没重新登录：\n");
        std::printf("    id -Gn | grep dialout              # 看当前终端有没有\n");
        std::printf("    sudo usermod -aG dialout \"$USER\"   # 加组（只需一次）\n");
        std::printf("    newgrp dialout                     # 或在当前终端临时生效\n");
        return false;
    }

    std::printf("\n✗ 串口设备不存在：%s\n", port.c_str());
    std::printf("  系统返回：%s\n", std::strerror(errno));
    std::printf("  常见原因：USB-TTL 没插好 / 线只供电不传数据 / 适配器掉线后节点消失\n");

    std::vector<std::string> candidates;
    for (const char* pattern : {"/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyCH34*"})
    {
        const auto found = glob_paths(pattern);
        candidates.insert(candidates.end(), found.begin(), found.end());
    }

    if (candidates.empty())
    {
        std::printf("  系统里现在一个串口设备都没有 —— 先把 USB-TTL 插回去\n");
    }
    else
    {
        std::printf("  但系统里有这些串口，可能只是编号变了：\n");
        for (const auto& path : candidates)
        {
            std::printf("    %s\n", path.c_str());
        }
        std::printf("  换一个试试： --live %s\n", candidates.front().c_str());
    }

    const auto by_id = glob_paths("/dev/serial/by-id/*");
    if (!by_id.empty())
    {
        std::printf("  稳定设备名（插拔后编号会变，推荐用它）：\n");
        for (const auto& path : by_id)
        {
            std::printf("    %s\n", path.c_str());
        }
    }

    std::printf("  排查命令：\n");
    std::printf("    journalctl -k | grep -iE 'ch34|ttyUSB' | tail\n");
    std::printf("    lsusb -t | grep -i -E 'ch34|cp210|ftdi'\n");
    return false;
}

/// 一条用例：描述 + 载荷 + 期望的整帧十六进制
void check(const char* name, const radar_msg& msg, const char* expected_hex)
{
    const Frame got = test_chassis::encode(msg);
    const auto want = parse_hex(expected_hex);

    bool ok = (want.size() == got.size()) &&
              (std::memcmp(want.data(), got.data(), got.size()) == 0);

    if (ok)
    {
        ++g_pass;
        std::printf("  \033[32mPASS\033[0m  %s\n", name);
    }
    else
    {
        ++g_fail;
        std::printf("  \033[31mFAIL\033[0m  %s\n", name);
        std::printf("        期望: %s\n", expected_hex);
        std::printf("        实际: %s\n", to_hex(got.data(), got.size()).c_str());
    }
}

}  // namespace


int main(int argc, char** argv)
{
    std::printf("====== TestChassis 编码自测（对照 Docs/xcom_测试报文.txt）======\n\n");

    // 先说明「为什么不能 memcpy」—— 这是设计前提，不是 bug
    std::printf("[结构体 vs 协议]\n");
    std::printf("  sizeof(radar_msg) = %zu\n", sizeof(radar_msg));
    std::printf("  协议帧长          = %zu\n", kRadarFrameLen);
    if (sizeof(radar_msg) != kRadarFrameLen)
    {
        std::printf("  -> 长度不同，必须逐字段 encode()，不能 memcpy\n");
    }
    std::printf("\n");

    // ---- 文档第四节给出的 8 条现成报文，逐字节比对 ----
    std::printf("[文档现成报文比对]\n");

    {
        radar_msg m{};                       // 1. 停车
        check("1. 停车 vx=vy=wz=0", m,
              "FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 00 FE");
    }
    {
        radar_msg m{};                       // 2. 世界 +X 满速
        m.x = 150.0f;
        check("2. 世界 +X 满速 vx=150", m,
              "FF 00 00 16 43 00 00 00 00 00 00 00 00 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 59 FE");
    }
    {
        radar_msg m{};                       // 3. 世界 -X 满速
        m.x = -150.0f;
        check("3. 世界 -X 满速 vx=-150", m,
              "FF 00 00 16 C3 00 00 00 00 00 00 00 00 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 D9 FE");
    }
    {
        radar_msg m{};                       // 4. 世界 +Y 满速
        m.y = 150.0f;
        check("4. 世界 +Y 满速 vy=150", m,
              "FF 00 00 00 00 00 00 16 43 00 00 00 00 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 59 FE");
    }
    {
        radar_msg m{};                       // 5. 世界 -Y 满速
        m.y = -150.0f;
        check("5. 世界 -Y 满速 vy=-150", m,
              "FF 00 00 00 00 00 00 16 C3 00 00 00 00 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 D9 FE");
    }
    {
        radar_msg m{};                       // 6. 原地左转
        m.z = 100.0f;
        check("6. 原地左转 wz=100", m,
              "FF 00 00 00 00 00 00 00 00 00 00 C8 42 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 0A FE");
    }
    {
        radar_msg m{};                       // 7. 原地右转
        m.z = -100.0f;
        check("7. 原地右转 wz=-100", m,
              "FF 00 00 00 00 00 00 00 00 00 00 C8 C2 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 8A FE");
    }
    {
        radar_msg m{};                       // 8. 慢速 +X
        m.x = 50.0f;
        check("8. 慢速 +X 平移 vx=50", m,
              "FF 00 00 48 42 00 00 00 00 00 00 00 00 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 00 8A FE");
    }

    // ---- 标志位顺序：最容易抄错的地方，单独验一条 ----
    std::printf("\n[标志位顺序]\n");
    {
        radar_msg m{};
        m.route_flag          = 0x11;
        m.transmit_block_flag = 0x22;
        m.error_flag          = 0x33;

        const Frame f = test_chassis::encode(m);
        if (f[25] == 0x11 && f[26] == 0x22 && f[27] == 0x33)
        {
            ++g_pass;
            std::printf("  \033[32mPASS\033[0m  [25]=route [26]=transmit_block [27]=error\n");
        }
        else
        {
            ++g_fail;
            std::printf("  \033[31mFAIL\033[0m  错位：[25]=%02X [26]=%02X [27]=%02X\n",
                        f[25], f[26], f[27]);
        }
        std::printf("        结构体顺序 error/route/transmit，协议顺序 route/transmit/error\n");
    }

    // ---- 校验与帧完整性 ----
    std::printf("\n[校验与帧完整性]\n");
    {
        radar_msg m{};
        m.x = 50.0f;
        const Frame f = test_chassis::encode(m);

        if (test_chassis::frame_valid(f))
        {
            ++g_pass;
            std::printf("  \033[32mPASS\033[0m  frame_valid() 通过\n");
        }
        else
        {
            ++g_fail;
            std::printf("  \033[31mFAIL\033[0m  frame_valid() 失败\n");
        }

        Frame bad = f;                        // 篡改一字节，应被校验抓到
        bad[3] = static_cast<std::uint8_t>(bad[3] ^ 0xFF);
        if (!test_chassis::frame_valid(bad))
        {
            ++g_pass;
            std::printf("  \033[32mPASS\033[0m  篡改后能被校验发现\n");
        }
        else
        {
            ++g_fail;
            std::printf("  \033[31mFAIL\033[0m  篡改后竟然校验通过\n");
        }
    }

    // ---- 量纲换算 ----
    std::printf("\n[量纲换算]\n");
    {
        struct Case { float mps; float stick; };
        const Case cases[] = {
            {0.0f,    0.0f},
            {0.5f,   20.0f},
            {1.25f,  50.0f},
            {3.75f, 150.0f},
        };
        for (const auto& c : cases)
        {
            const float s = test_chassis::world_vel_to_stick(c.mps);
            const bool ok = (std::fabs(s - c.stick) < 0.01f);
            if (ok) ++g_pass; else ++g_fail;
            std::printf("  %s  %.2f m/s -> %.1f 摇杆量\n",
                        ok ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m",
                        c.mps, s);
        }
    }

    std::printf("\n====== 结果：%d 通过，%d 失败 ======\n", g_pass, g_fail);

    // ---- 实发模式：默认不跑，带 --live 才真的开串口 ----
    //   ./test_chassis                          # 纯离线自测，不碰硬件
    //   ./test_chassis --live /dev/ttyUSB0      # 发 3 秒（20Hz），摇杆量默认 20
    //   ./test_chassis --live /dev/ttyUSB0 50 5 # 摇杆量 50，发 5 秒
    if (argc > 1 && std::strcmp(argv[1], "--live") == 0)
    {
        const char* port = (argc > 2) ? argv[2] : "/dev/ttyUSB0";

        // 第三、四个参数是可选的：摇杆量（协议量纲 ±150）与秒数
        float stick = (argc > 3) ? std::strtof(argv[3], nullptr)
                                 : test_chassis::world_vel_to_stick(0.5f);
        int seconds = (argc > 4) ? std::atoi(argv[4]) : 3;

        if (!(stick == stick))                       // NaN 兜底
        {
            std::printf("\n⚠ 摇杆量不是合法数字，按 0 处理\n");
            stick = 0.0f;
        }
        if (stick > test_chassis::kStickFullScale || stick < -test_chassis::kStickFullScale)
        {
            std::printf("\n⚠ 摇杆量 %.1f 超出 ±%.0f，板子会自己限幅，这里先按边界发\n",
                        stick, test_chassis::kStickFullScale);
            stick = (stick > 0.0f) ? test_chassis::kStickFullScale
                                   : -test_chassis::kStickFullScale;
        }
        if (seconds <= 0)
        {
            std::printf("\n⚠ 秒数必须 > 0，按 3 秒处理\n");
            seconds = 3;
        }

        const float mps = test_chassis::stick_to_world_vel(stick);
        std::printf("\n=== 实发模式 %s（20 帧/秒，%d 秒，摇杆量 %.1f ≈ 目标 %.2f m/s）===\n",
                    port, seconds, stick, mps);

        // 先把「设备在不在 / 有没有权限」查清楚，再 open()，省得只看到一句 errno
        if (!preflight_port(port))
        {
            return 2;
        }

        TestChassis chassis(port, B115200);
        if (!chassis.is_open())
        {
            std::printf("✗ 串口打开失败：%s\n", chassis.last_error().c_str());
            return 2;
        }
        std::printf("  ✓ 已打开 %s（115200 8N1）\n", port);

        radar_msg cmd{};
        cmd.x = stick;                               // 世界系 +X

        const int total = 20 * seconds;
        int sent = 0;
        int failed = 0;
        Frame first_sent{};
        for (int i = 0; i < total; ++i)
        {
            if (chassis.set_velocity(cmd))
            {
                if (sent == 0)
                {
                    first_sent = chassis.last_frame();
                }
                ++sent;
            }
            else
            {
                ++failed;
                if (failed == 1)                     // 只报第一条，避免刷屏
                {
                    std::printf("  ✗ 第 %d 帧发送失败：%s（后续失败不再重复打印）\n",
                                i, chassis.last_error().c_str());
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));   // 50ms = 20Hz
        }
        chassis.emergency_stop();                    // 末尾补一帧全零停车

        std::printf("  已发 %d 帧：成功 %d，失败 %d；末尾已补一帧停车\n",
                    total, sent, failed);
        if (sent > 0)
        {
            std::printf("  首帧（可直接和 Docs/xcom_测试报文.txt 对照）：%s\n",
                        to_hex(first_sent.data(), first_sent.size()).c_str());
        }

        if (failed > 0)
        {
            return 2;
        }
        std::printf("实发结束\n");
    }

    // 实发失败已在上面提前 return 2；走到这里只可能是自测失败
    return (g_fail == 0) ? 0 : 1;
}
