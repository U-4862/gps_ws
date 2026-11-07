// 纯离线自测：不碰串口、不碰 ROS
#include <cstdio>
#include <cmath>
#include "Nav/FrameAlign.hpp"

using namespace nav;

static int fails = 0;
static void check(bool ok, const char * name, float got, float want, float tol)
{
    const bool pass = ok && std::fabs(got - want) <= tol;
    std::printf("%s %-38s got=%8.4f want=%8.4f\n", pass ? "PASS" : "FAIL", name, got, want);
    if (!pass) ++fails;
}

int main()
{
    constexpr float D2R = static_cast<float>(M_PI) / 180.0f;

    // ---- 1) theta = 0 时应当恒等 ----
    {
        FrameAlign fa;
        float vx = 0, vy = 0;
        fa.to_chassis_vel(1.0f, 0.0f, vx, vy);
        check(true, "theta=0  map+x -> cw+x", vx, 1.0f, 1e-5f);
        check(true, "theta=0  map+x -> cw+y", vy, 0.0f, 1e-5f);
    }

    // ---- 2) 实测夹角 83°：map 速度若指向 83°，底盘世界应收到 (+1, 0) ----
    {
        FrameAlign fa;
        fa.set_theta_cw(83.0f * D2R);
        float vx = 0, vy = 0;

        fa.to_chassis_vel(std::cos(83.0f * D2R), std::sin(83.0f * D2R), vx, vy);
        check(true, "theta=83  v along 83 -> cw x", vx, 1.0f, 1e-4f);
        check(true, "theta=83  v along 83 -> cw y", vy, 0.0f, 1e-4f);

        // map +x 应当落到 cw 的 -83°
        fa.to_chassis_vel(1.0f, 0.0f, vx, vy);
        check(true, "theta=83  map+x -> cw.x", vx,  std::cos(83.0f * D2R), 1e-4f);
        check(true, "theta=83  map+x -> cw.y", vy, -std::sin(83.0f * D2R), 1e-4f);
    }

    // ---- 3) 正逆变换互为逆 ----
    {
        FrameAlign fa;
        fa.set_theta_cw(-37.0f * D2R);
        float cx = 0, cy = 0, bx = 0, by = 0;
        map_to_chassis_world(0.3f, -0.7f, fa.theta_cw(), cx, cy);
        chassis_world_to_map(cx, cy, fa.theta_cw(), bx, by);
        check(true, "roundtrip x", bx, 0.3f, 1e-5f);
        check(true, "roundtrip y", by, -0.7f, 1e-5f);
    }

    // ---- 4) 限速：超速时等比缩放，方向不变 ----
    {
        FrameAlign fa;
        fa.config().max_speed_mps = 1.0f;
        fa.set_theta_cw(0.0f);
        float vx = 0, vy = 0;
        fa.to_chassis_vel(3.0f, 4.0f, vx, vy);          // 模长 5，限到 1
        check(true, "limitspeed |v|", std::hypot(vx, vy), 1.0f, 1e-5f);
        check(true, "limitspeed ratio", vx / vy, 3.0f / 4.0f, 1e-4f);
    }

    // ---- 5) stick 换算：25 stick 应对应实测的 0.197 m/s ----
    {
        FrameAlign fa;
        fa.set_theta_cw(0.0f);
        const radar_msg m = fa.to_msg(0.197f, 0.0f);
        check(true, "0.197 m/s -> stick", m.x, 25.0f, 0.2f);
    }

    // ---- 6) 标定函数：位移方向直接给出夹角 ----
    {
        FrameAlign fa;
        fa.calibrate_from_displacement(0.10f, 0.20f);
        check(true, "calib atan2(0.2,0.1) deg", fa.theta_cw() / D2R, 63.4349f, 0.01f);

        fa.calibrate_from_displacement(0.001f, 0.001f);   // 太短，应被忽略
        check(true, "calib ignores tiny move", fa.theta_cw() / D2R, 63.4349f, 0.01f);
    }

    // ---- 7) 摇杆量削顶 ----
    {
        FrameAlign fa;
        fa.config().max_speed_mps = 10.0f;   // 放开限速，专测削顶
        fa.set_theta_cw(0.0f);
        const radar_msg m = fa.to_msg(5.0f, 0.0f);
        check(true, "clamp at max stick", m.x, 150.0f, 1e-3f);
    }


    // ---- 8) 与 PurePursuit 对接：车体系 stick 重投影 ----
    {
        FrameAlign fa;
        fa.set_theta_cw(83.0f * D2R);

        // 车头朝 map +x（yaw=0），PurePursuit 给「前进 25 stick」
        radar_msg body {};
        body.x = 25.0f;
        const float v = 25.0f / 40.0f;              // = 0.625 m/s
        const radar_msg out = fa.rebase_body_stick(body, 0.0f);

        // 模长必须守恒：0.625 m/s × 126.9 stick/(m/s)
        check(true, "rebase |stick| kept", std::hypot(out.x, out.y), v * 126.9f, 0.5f);
        // 方向必须是底盘世界系的 -83°
        check(true, "rebase x", out.x,  v * 126.9f * std::cos(83.0f * D2R),  0.3f);
        check(true, "rebase y", out.y, -v * 126.9f * std::sin(83.0f * D2R),  0.3f);

        // 车头掉头 180°：同样 25 stick 前进，底盘世界方向应整体反相
        const radar_msg out2 = fa.rebase_body_stick(body, static_cast<float>(M_PI));
        check(true, "rebase yaw180 x", out2.x, -out.x, 0.3f);
        check(true, "rebase yaw180 y", out2.y, -out.y, 0.3f);
    }

    std::printf("\n%s  失败 %d 项\n", fails ? "✗ 有失败" : "✓ 全部通过", fails);
    return fails ? 1 : 0;
}
