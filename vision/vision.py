import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import serial
import struct
import logging
import time

# ==================== 用户配置区 ====================
DET_MODEL_PATH = "best.pt"  # YOLO 权重路径
CONF_THRESHOLD = 0.6  # 目标检测置信度阈值
DEPTH_SAMPLE_SIZE = 5  # 深度采样区域半边长（像素），用于抗噪滤波
KFS_HALF_DEPTH = 175.0  # 350mm 武术秘籍纸箱深度的一半 (单位: 毫米)

# --- 串口配置 ---
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
HEADER = bytes([0xAA, 0xAA, 0xBB, 0xBB])
TAIL = bytes([0xCC, 0xCC, 0xDD, 0xDD])
STATE_MOVE = 2

# --- 联调机制配置 ---
SEND_COOLDOWN = 0.5  # 发送冷却时间 (秒)：限制发送频率，防止 STM32 溢出

# --- 武术秘籍类别字典 ---
# 规则参考: R1(Logo), R2(甲骨文), 假(小篆)
CLASS_NAMES = {
    0: {"name": "R1_武术秘籍 (Logo)", "is_real": True, "color": (0, 255, 0)},  # 绿色框
    1: {"name": "R2_武术秘籍 (甲骨文)", "is_real": True, "color": (255, 255, 0)},  # 青色框
    2: {"name": "假_武术秘籍 (小篆)", "is_real": False, "color": (0, 0, 255)}  # 红色框
}
# ====================================================

# ==================== 初始化模块 ====================
# 1. 初始化双路日志系统
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s.%(msecs)03d] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.FileHandler("vision_target_log.txt", mode='w', encoding='utf-8'),
        logging.StreamHandler()
    ]
)

# 2. 初始化串口
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    logging.info(f"[系统] 串口已成功打开: {SERIAL_PORT} @ {BAUDRATE}")
except Exception as e:
    logging.warning(f"[警告] 串口打开失败，将仅在本地运行视觉逻辑: {e}")
    ser = None

# ==================== 核心功能函数 ====================
def get_robust_depth(depth_frame, cx, cy, sample_size):
    """提取中心区域的抗噪深度中位数 (剔除0值黑洞)"""
    x_min = max(0, cx - sample_size)
    x_max = min(depth_frame.get_width(), cx + sample_size)
    y_min = max(0, cy - sample_size)
    y_max = min(depth_frame.get_height(), cy + sample_size)

    depths = []
    for y in range(y_min, y_max):
        for x in range(x_min, x_max):
            dist = depth_frame.get_distance(x, y)  # 单位：米
            if dist > 0:
                depths.append(dist)

    if not depths:
        return 0.0
    return float(np.median(depths))

def send_to_stm32(x_mm, y_mm, z_mm):
    """将物理坐标打包发送给底盘/机械臂单片机"""
    if ser is None:
        return

    x_int = int(round(x_mm))
    y_int = int(round(y_mm))
    z_int = int(round(z_mm))

    # 按照 15 字节协议组包：大端短整型 (>h)
    x_bytes = struct.pack('>h', x_int)
    y_bytes = struct.pack('>h', y_int)
    z_bytes = struct.pack('>h', z_int)
    packet = HEADER + bytes([STATE_MOVE]) + x_bytes + y_bytes + z_bytes + TAIL

    ser.write(packet)
    # logging.info(f"[串口 TX] 已发送 {len(packet)} 字节: {packet.hex().upper()}")

# ==================== 主循环 ====================
def main():
    logging.info("[系统] 正在加载 YOLOv8 模型...")
    model = YOLO(DET_MODEL_PATH)

    logging.info("[系统] 正在唤醒 D435i 相机...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        profile = pipeline.start(config)
    except Exception as e:
        logging.error(f"[致命错误] D435i 启动失败，请检查连线: {e}")
        return

    # 获取相机内参并创建对齐对象
    color_profile = profile.get_stream(rs.stream.color)
    color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
    align = rs.align(rs.stream.color)

    # --- 联调状态机变量 ---
    last_send_time = 0
    waiting_for_pd15 = False

    logging.info("=" * 60)
    logging.info(" 视觉系统进入战备状态！开始异步联调模式。按 'q' 键退出。")
    logging.info("=" * 60)

    try:
        while True:
            # ---------------------------------------------------------
            # 1. 异步处理 STM32 的反馈 (非阻塞，不卡顿画面)
            # ---------------------------------------------------------
            if ser is not None and ser.in_waiting > 0:
                response = ser.read(1)
                pd15_val = response[0]
                logging.info(
                    f"======> [下位机反馈 RX] 收到 PD15 状态: {'高电平 (1)' if pd15_val == 1 else '低电平 (0)'}")
                # 收到反馈后，解除等待锁，允许发送下一个坐标
                waiting_for_pd15 = False

            # ---------------------------------------------------------
            # 2. 拉取画面与模型推理 (保持 30FPS)
            # ---------------------------------------------------------
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image, conf=CONF_THRESHOLD, verbose=False)[0]

            best_target_coords = None
            best_target_name = "1"

            # 遍历画面中的检测框
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                class_id = int(box.cls[0].item())

                target_info = CLASS_NAMES.get(class_id,
                                              {"name": "未知目标", "is_real": False, "color": (128, 128, 128)})
                target_name = target_info["name"]
                is_real = target_info["is_real"]
                box_color = target_info["color"]

                status_str = "【真 - 允许抓取】" if is_real else "【假 - 危险勿碰】"

                # 计算 2D 像素中心与深度
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                dist_m = get_robust_depth(depth_frame, cx, cy, DEPTH_SAMPLE_SIZE)

                # 有效测距范围
                if 0.1 < dist_m < 8.0:
                    # 反投影：计算物体表面中心点的 3D 相机坐标
                    point_3d = rs.rs2_deproject_pixel_to_point(color_intrinsics, [cx, cy], dist_m)

                    # 几何中心修正 (Z轴推进 175mm)
                    X_center = point_3d[0] * 1000
                    Y_center = point_3d[1] * 1000
                    Z_center = (point_3d[2] * 1000) + KFS_HALF_DEPTH

                    # 记录并渲染目标信息
                    log_msg = (f"发现: {target_name} | {status_str} | "
                               f"XYZ: {X_center:>5.0f}, {Y_center:>5.0f}, {Z_center:>5.0f} mm")
                    # 为了日志干净，只在没被“等待锁”锁住时频繁打印视觉发现
                    if not waiting_for_pd15:
                        logging.info(log_msg)

                    # UI 渲染
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2)
                    cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                    text = f"{target_name} {status_str}"
                    cv2.putText(color_image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    cv2.putText(color_image, f"XYZ: {X_center:.0f}, {Y_center:.0f}, {Z_center:.0f} mm",
                                (x1, y2 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    # ---------------------------------------------------------
                    # 3. 筛选真实目标进行发送
                    # ---------------------------------------------------------
                    if is_real and best_target_coords is None:
                        # 如果是真秘籍，且还没锁定目标，则将其设为本次循环的抓取目标
                        best_target_coords = (X_center, Y_center, Z_center)
                        best_target_name = target_name

            # ---------------------------------------------------------
            # 4. 安全发送逻辑 (防洪泄流 + 等待闭环)
            # ---------------------------------------------------------
            current_time = time.time()
            if best_target_coords is not None:
                # 判断：没有在等回复 && 过了冷却期
                if not waiting_for_pd15 and (current_time - last_send_time > SEND_COOLDOWN):
                    x_mm, y_mm, z_mm = best_target_coords

                    # ★★★ 这里需要根据你的实际情况添加手眼标定平移量 ★★★
                    # x_robot = x_mm + 150
                    # y_robot = y_mm
                    # z_robot = z_mm + 50

                    send_to_stm32(x_mm, y_mm, z_mm)

                    last_send_time = current_time
                    waiting_for_pd15 = True
                    logging.info(
                        f"====> [串口 TX] 已向单片机发送 {best_target_name} 的坐标。系统进入等待反馈状态 (冻结发送)...")

            # 刷新画面
            cv2.imshow('R2 Vision - Kung Fu Quest Pro', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("[系统] 接收到退出指令，视觉程序终止。")
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if ser:
            ser.close()
            logging.info("[系统] 串口已安全关闭。")

if __name__ == "__main__":
    main()
