#pragma once

/**
 * @file FrameAlign.hpp
 * @brief map 系 → 底盘世界系 的坐标对齐与量纲换算（header-only，纯函数）
 *
 * ============================================================================
 * 一、为什么要这一层
 * ============================================================================
 *
 * 项目里有三套坐标系，各自锚在不同的时刻，彼此不相干：
 *
 *   ① camera_init   FAST_LIO 启动那一刻的雷达位姿，锁死（就是我们的 map 系）
 *        └─ body     跟着雷达实时走
 *
 *   ② 底盘世界系     底盘上电那一刻的车头方向 + 180°，由 HI14 陀螺仪积分维持
 *        └─ 车体系    底盘内部用 -current_yaw 做顺时针旋转得到
 *
 *   ③ 车体系         跟着车头走
 *
 * 关键事实（读固件源码确认，见 bsp/chassis/chassis.h:45、chassis.c:107、312-320）：
 *
 *   · 30 字节帧里的 x/y/z **是底盘世界系速度，不是车体系**
 *     （`Velocity_Data input_world_velocity;  // 输入世界坐标系速度`）
 *   · 底盘自己会做「世界系 → 车体系」的旋转：
 *         Coordinate_Transform_World_To_Robot(&world, &robot, -current_yaw);
 *     `current_yaw` 来自 HI14，`gyro_HI14.c:15` 写死 `HI14_YAW_INIT_DEG = 180.0f`，
 *     所以底盘世界系的 0° = 上电时车头的**反方向**。
 *
 * 由此得到两条结论：
 *
 *   A. 上位机只需要做「map 系 → 底盘世界系」这**一层**旋转，
 *      不需要关心车头朝向（底盘自己会转）。
 *   B. 两层原点锚定的时刻不同，中间隔着"车从底盘上电到 FAST_LIO 启动之间转了
 *      多少"。所以这个夹角 **theta_cw 每次开机都不一样，必须现场标定**，
 *      它不是安装误差，不能跨次复用。
 *
 * ============================================================================
 * 二、theta_cw 的定义与标定
 * ============================================================================
 *
 *   theta_cw := 底盘世界系的 +x 轴，在 map 系里的角度（弧度，逆时针为正）
 *
 * 标定方法（一次即可，2 秒）：
 *
 *   1. 发一帧纯 +x 速度（例如 x_stick = +25，y = z = 0），持续 1.5 s
 *   2. 记下起止两点的 /Odometry 位移 (dx, dy)（先各静止 1 s 取均值去抖）
 *   3. theta_cw = atan2(dy, dx)
 *
 * 因为发的是底盘世界 +x，车实际就往那个方向走，而位移是在 map 系里量到的，
 * 于是这一步直接读出了两套系的夹角。
 *
 * 实测（2026-11-07，一次开机内的数据）：
 *
 *   x=+25  → 位移 0.296 m，方向 +83.0°   速度 0.197 m/s
 *   x=-25  → 位移 0.284 m，方向 -123.6°  速度 0.189 m/s
 *   y=+25  → 位移 0.177 m，方向 +11.4°   速度 0.118 m/s
 *   y=-25  → 位移 0.305 m，方向 -177.1°  速度 0.204 m/s
 *
 *   → theta_cw ≈ 83°，重复测量波动约 ±5°
 *   → 同一指令重复性很好，说明底盘世界系是稳的，不是随机漂移
 *
 * 注意：x 正反向差 203.7°（不是 180°）、x 与 y 差 71.6°（不是 90°），
 * 这部分残差来自全向轮横移打滑（y=+25 速度只有 x 的 60%）与减速惯性，
 * 不是坐标系问题。要更准就多测几次取平均，或者降低速度。
 *
 * ============================================================================
 * 三、量纲：stick ↔ m/s
 * ============================================================================
 *
 * `chassis_protocol::world_vel_to_stick()` 用的是 `kWorldVelFullScale = 3.75`，
 * 即 150 stick ↔ 3.75 m/s = 40 stick/(m/s)。
 *
 * 但实测慢档（GEAR_SLOW_SCALE = 6）：
 *
 *     x_stick = 25  →  0.197 m/s
 *     → 实际比例 25 / 0.197 ≈ 126.9 stick/(m/s)
 *
 * 比 3.75 那套慢 **3.17 倍**。原因是 3.75 是底盘内部 `speed_k` 和 `0.01f`
 * 折算出的"快档理论满速"，而轮速最后还要乘 `wheel_scale`（慢档 6 / 快档 12，
 * 见 fdcan_it.c:526-527、683）。
 *
 * 所以本文件**不用** world_vel_to_stick，改用实测的 stick_per_mps，
 * 默认值取慢档的 126.9。跑快档请把它改成 126.9 / 2 ≈ 63.5。
 *
 * ============================================================================
 * 四、埋着的坑：CHASSIS_YAW_HOLD
 * ============================================================================
 *
 * `fdcan_it.c:88`  `#define CHASSIS_YAW_HOLD 1`
 * `fdcan_it.c:670` `Chassis_Control(&sbus_chassis, mid_speed, s_yaw_ready);`
 * `chassis.c:89`   `if (pid_flag) { ... target_world_velocity.Vw = pid_calc(err_yaw); }`
 *
 * 只要陀螺仪给出有效数据（MINS500_data.first_flag != 0 → s_yaw_ready = 1），
 * 底盘就会用 PID 把航向锁死在 `s_yaw_base`（上电朝向），
 * **你发的 z 会被完全丢弃**，且不会有任何报错。
 *
 * 当前实测 z=+10 是能转的，说明陀螺仪此刻没有数据、pid_flag 走了 0 分支。
 * 陀螺仪一旦恢复，z 就静默失效。所以：
 *
 *   · 若程序依赖 z 转向 → 发完 z 之后必须用 IMU 或里程计**验证车真的转了**
 *   · 若程序不依赖 z（只做全向平移）→ 无影响，但仍建议把 z 填 0
 */

#include <cmath>

#include "Chassis/SerialChassis.hpp"

namespace nav
{

// ============================================================================
// 配置
// ============================================================================

struct FrameAlignConfig
{
    /// 底盘世界 +x 轴在 map 系里的角度（弧度，逆时针为正）。每次开机需重新标定。
    float theta_cw {0.0f};

    /// 摇杆量与线速度的换算比例（stick per m/s）。
    /// 默认 126.9 = 实测慢档（x_stick=25 → 0.197 m/s）。
    /// 快档请改成 63.5。
    float stick_per_mps {126.9f};

    /// 摇杆量上限（协议固定 ±150，不要改）
    float max_stick {chassis_protocol::kStickFullScale};

    /// 单轴速度上限（m/s），超过就按比例整体缩放，避免削顶变形
    float max_speed_mps {1.2f};
};

// ============================================================================
// 纯函数
// ============================================================================

/// 角度归一化到 (-pi, pi]
inline float wrap_pi(float a)
{
    while (a > static_cast<float>(M_PI))  { a -= 2.0f * static_cast<float>(M_PI); }
    while (a <= -static_cast<float>(M_PI)) { a += 2.0f * static_cast<float>(M_PI); }
    return a;
}

inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/**
 * @brief map 系速度 → 底盘世界系速度
 *
 * 推导：设 v_map 指向 theta_cw 方向，即 (cos θ, sin θ)。
 *       那么它在底盘世界系里应当是 (+1, 0)。
 *       代入下面的式子：
 *           vx_c =  cosθ·cosθ + sinθ·sinθ = 1
 *           vy_c = -sinθ·cosθ + cosθ·sinθ = 0
 *       ✓ 成立。
 *
 * @param vx_map, vy_map  地图系线速度 (m/s)
 * @param theta_cw        底盘世界 +x 在 map 系的角度 (rad)
 * @param vx_c, vy_c      输出：底盘世界系线速度 (m/s)
 */
inline void map_to_chassis_world(float vx_map, float vy_map, float theta_cw,
                                 float & vx_c, float & vy_c)
{
    const float c = std::cos(theta_cw);
    const float s = std::sin(theta_cw);
    vx_c =  c * vx_map + s * vy_map;
    vy_c = -s * vx_map + c * vy_map;
}

/// 反方向：底盘世界系 → map 系（调试、回放时用）
inline void chassis_world_to_map(float vx_c, float vy_c, float theta_cw,
                                 float & vx_map, float & vy_map)
{
    const float c = std::cos(theta_cw);
    const float s = std::sin(theta_cw);
    vx_map = c * vx_c - s * vy_c;
    vy_map = s * vx_c + c * vy_c;
}

/// 由一次标定测量的位移算 theta_cw
inline float theta_from_displacement(float dx, float dy)
{
    return wrap_pi(std::atan2(dy, dx));
}

/**
 * @brief 整体等比限速，保持方向不变
 *
 * 不做分轴削顶：分轴削顶会把斜向运动掰成 45°，方向就错了。
 */
inline void limit_speed(float & vx, float & vy, float max_speed)
{
    const float v = std::hypot(vx, vy);
    if (v > max_speed && v > 1e-6f) {
        const float k = max_speed / v;
        vx *= k;
        vy *= k;
    }
}

// ============================================================================
// 对齐器
// ============================================================================

class FrameAlign
{
public:
    FrameAlign() = default;
    explicit FrameAlign(const FrameAlignConfig & cfg) : cfg_(cfg) {}

    // ---- 配置 ----

    const FrameAlignConfig & config() const { return cfg_; }
    FrameAlignConfig &       config()       { return cfg_; }

    void set_theta_cw(float rad)          { cfg_.theta_cw = wrap_pi(rad); }
    void set_stick_per_mps(float k)       { cfg_.stick_per_mps = k; }
    void set_max_speed(float mps)         { cfg_.max_speed_mps = mps; }

    float theta_cw() const                { return cfg_.theta_cw; }

    /**
     * @brief 用一次实测位移完成标定
     *
     * 步骤：发一帧纯 +x 速度（y=z=0）持续约 1.5 s，
     *       取起止两点的 /Odometry 位移 (dx, dy) 传进来。
     */
    void calibrate_from_displacement(float dx, float dy)
    {
        // 位移太小说明车没动（或撞到什么），不采信
        if (std::hypot(dx, dy) < 0.05f) { return; }
        cfg_.theta_cw = theta_from_displacement(dx, dy);
    }

    // ---- 换算 ----

    /**
     * @brief map 系速度 → 底盘世界系摇杆量
     *
     * @param vx_map, vy_map  map 系线速度 (m/s)
     * @param wz_stick        角速度摇杆量（原样下发，见文件头"CHASSIS_YAW_HOLD"的坑）
     * @return radar_msg，x/y/z 已填好，其余字段清零
     */
    radar_msg to_msg(float vx_map, float vy_map, float wz_stick = 0.0f) const
    {
        // 1) 限速：整体等比缩放，保持方向不变
        float vx = vx_map;
        float vy = vy_map;
        limit_speed(vx, vy, cfg_.max_speed_mps);

        // 2) 坐标旋转
        float vx_c = 0.0f, vy_c = 0.0f;
        map_to_chassis_world(vx, vy, cfg_.theta_cw, vx_c, vy_c);

        // 3) 量纲：m/s → stick
        radar_msg msg {};
        msg.x = clampf(vx_c * cfg_.stick_per_mps, -cfg_.max_stick, cfg_.max_stick);
        msg.y = clampf(vy_c * cfg_.stick_per_mps, -cfg_.max_stick, cfg_.max_stick);
        msg.z = clampf(wz_stick,                      -cfg_.max_stick, cfg_.max_stick);

        // 其余字段保持协议要求的 0
        msg.translation_distance = 0.0f;
        msg.high_err             = 0.0f;
        msg.front_err            = 0.0f;
        msg.route_flag           = 0;
        msg.transmit_block_flag  = 0;
        msg.error_flag           = 0;
        return msg;
    }

    /// 只算底盘世界系速度（已限速），不转 stick（打日志、调试用）
    void to_chassis_vel(float vx_map, float vy_map, float & vx_c, float & vy_c) const
    {
        float vx = vx_map;
        float vy = vy_map;
        limit_speed(vx, vy, cfg_.max_speed_mps);
        map_to_chassis_world(vx, vy, cfg_.theta_cw, vx_c, vy_c);
    }

    /// 全零帧（停车）
    static radar_msg stop_msg()
    {
        radar_msg msg {};
        msg.x = 0.0f;
        msg.y = 0.0f;
        msg.z = 0.0f;
        return msg;
    }

    // ---- 与现有 PurePursuit 对接 ----

    /**
     * @brief 把 PurePursuit 输出的「车体系 stick 帧」重投影成「底盘世界系 stick 帧」
     *
     * 为什么需要这一步：
     *   `Nav/PurePursuit.cpp:49-50` 产出的是
     *       msg.x = world_vel_to_stick(v * lx / Ld_act)
     *   其中 lx/ly 是**前瞻点在车体系**下的坐标，所以它是车体系速度；
     *   而且用的是 `world_vel_to_stick()` 的 40 stick/(m/s)，
     *   与实测的 126.9 差了 3.17 倍。
     *
     * 两处都对不上。直接发出去会：方向偏 theta_cw（实测约 83°）+ 速度只有 1/3。
     *
     * 这个方法一次掰正：
     *   车体系 stick ──(÷ in_stick_per_mps)──> 车体系 m/s
     *                ──(旋转 yaw_map)──>       map m/s
     *                ──(to_msg)──>             底盘世界系 stick（实测比例）
     *
     * @param body_msg         PurePursuit::compute() 返回的 Command::msg
     * @param yaw_map          当前车头朝向（map 系，来自 /Odometry 的 yaw），弧度
     * @param in_stick_per_mps 输入帧用的换算比例，默认取 chassis_protocol 那套 40
     */
    radar_msg rebase_body_stick(const radar_msg & body_msg,
                               float yaw_map,
                               float in_stick_per_mps =
                                   chassis_protocol::kStickFullScale /
                                   chassis_protocol::kWorldVelFullScale) const
    {
        if (in_stick_per_mps < 1e-3f) { in_stick_per_mps = 40.0f; }

        // 1) 车体系 stick → 车体系 m/s
        const float vx_b = body_msg.x / in_stick_per_mps;
        const float vy_b = body_msg.y / in_stick_per_mps;

        // 2) 车体系 → map（绕 z 逆时针转 yaw）
        const float c = std::cos(yaw_map);
        const float s = std::sin(yaw_map);
        const float vx_m = c * vx_b - s * vy_b;
        const float vy_m = s * vx_b + c * vy_b;

        // 3) map → 底盘世界系 → stick（实测比例，带限速）
        return to_msg(vx_m, vy_m, body_msg.z);
    }

private:
    FrameAlignConfig cfg_ {};
};

}  // namespace nav
