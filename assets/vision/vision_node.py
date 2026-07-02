#!/usr/bin/env python3
"""
视觉检测 ROS 2 节点
功能：YOLO 目标检测 + RealSense 深度测距 + 武术秘籍真伪判定
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
# import serial         # ★ 串口发送已禁用
import struct
import logging
import time


# ==================== 用户配置区 ====================
DET_MODEL_PATH = "best.pt"          # YOLO 权重路径
CONF_THRESHOLD = 0.6                # 目标检测置信度阈值
DEPTH_SAMPLE_SIZE = 5               # 深度采样区域半边长（像素），用于抗噪滤波
KFS_HALF_DEPTH = 175.0              # 350mm 武术秘籍纸箱深度的一半 (单位: 毫米)

# --- 串口配置 (已禁用) ---
# SERIAL_PORT = "/dev/ttyUSB0"
# BAUDRATE = 115200
# HEADER = bytes(0xff)
# TAIL = bytes(0x0f)
# STATE_MOVE = 2

# --- 联调机制配置 ---
SEND_COOLDOWN = 0.5                 # 发送冷却时间 (秒)：限制发送频率，防止 STM32 溢出

# --- 武术秘籍类别字典 ---
# 规则参考: R1(Logo), R2(甲骨文), 假(小篆)
CLASS_NAMES = {
    0: {"name": "R1_武术秘籍 (Logo)",   "is_real": True,  "color": (0, 255, 0)},    # 绿色框
    1: {"name": "R2_武术秘籍 (甲骨文)", "is_real": True,  "color": (255, 255, 0)},  # 青色框
    2: {"name": "假_武术秘籍 (小篆)",   "is_real": False, "color": (0, 0, 255)}     # 红色框
}
# ====================================================


class VisionNode(Node):
    """视觉检测 ROS 2 节点

    订阅：无（独立运行，可选订阅 /Odometry 获取机器人位姿）
    发布：无（当前仅做本地推理与显示，后续可扩展检测结果发布）
    参数：
        START_VISION — 是否启用视觉推理 (bool, 默认 True)
                       True  → 正常检测，画面实时更新
                       False → 暂停推理与画面，节点保持存活等待唤醒     
        KFS_Status   — 武术秘籍系统状态字 (string, 默认 "ACTIVE")
        IS_GRIPPED   — 抓手是否已抓取完成 (string, 默认 "NOT_GRIPPED")
                       外部节点抓到目标后应设为 "GRIPPED"，
                       本节点检测到后解除等待并自动重置为 "NOT_GRIPPED"
    """

    def __init__(self):
        super().__init__('vision_node')

        # ---- 声明 ROS 2 参数 ----
        self.declare_parameter('START_VISION', True)
        self.declare_parameter('KFS_Status', 'ACTIVE')
        self.declare_parameter('IS_GRIPPED', 'NOT_GRIPPED')

        # ---- 初始化双路日志系统 ----
        logging.basicConfig(
            level=logging.INFO,
            format='[%(asctime)s.%(msecs)03d] %(message)s',
            datefmt='%H:%M:%S',
            handlers=[
                logging.FileHandler("vision_target_log.txt", mode='w', encoding='utf-8'),
                logging.StreamHandler()
            ]
        )

        # ---- 初始化串口 (已禁用) ----
        # try:
        #     self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        #     logging.info(f"[系统] 串口已成功打开: {SERIAL_PORT} @ {BAUDRATE}")
        # except Exception as e:
        #     logging.warning(f"[警告] 串口打开失败，将仅在本地运行视觉逻辑: {e}")
        #     self.ser = None
        self.ser = None  # 串口已禁用
        self.get_logger().info("串口发送已禁用，仅运行本地视觉逻辑")

        # ---- 加载 YOLO 模型 ----
        self.get_logger().info("正在加载 YOLOv8 模型...")
        self.model = YOLO(DET_MODEL_PATH)
        self.get_logger().info("YOLOv8 模型加载完成")

        # ---- 唤醒 D435i 相机 ----
        self.get_logger().info("正在唤醒 D435i 相机...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            profile = self.pipeline.start(self.config)
        except Exception as e:
            self.get_logger().fatal(f"D435i 启动失败，请检查连线: {e}")
            raise

        # 获取相机内参并创建对齐对象
        color_profile = profile.get_stream(rs.stream.color)
        self.color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)

        # ---- 联调状态机变量 ----
        self.last_send_time = 0.0
        self.grip_waiting = False          # 当前是否因已锁定目标而等待夹爪抓取
        self.locked_target_coords = None   # 锁定目标的 3D 坐标
        self.locked_target_name = ""       # 锁定目标的名称

        # ---- 创建定时器驱动主循环 (~30 Hz) ----
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

        self.get_logger().info("=" * 60)
        self.get_logger().info(" 视觉系统进入战备状态！按 'q' 键退出。")
        self.get_logger().info("=" * 60)

    # ==================== 核心功能函数 ====================

    @staticmethod
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

    # ---- 串口发送函数 (已禁用) ----
    # def send_to_stm32(self, x_mm, y_mm, z_mm):
    #     """将物理坐标打包发送给底盘/机械臂单片机"""
    #     if self.ser is None:
    #         return
    #
    #     x_int = int(round(x_mm))
    #     y_int = int(round(y_mm))
    #     z_int = int(round(z_mm))
    #
    #     # 按照 15 字节协议组包：大端短整型 (>h)
    #     x_bytes = struct.pack('>h', x_int)
    #     y_bytes = struct.pack('>h', y_int)
    #     z_bytes = struct.pack('>h', z_int)
    #     packet = HEADER + bytes([STATE_MOVE]) + x_bytes + y_bytes + z_bytes + TAIL
    #
    #     self.ser.write(packet)
    #     # logging.info(f"[串口 TX] 已发送 {len(packet)} 字节: {packet.hex().upper()}")

    # ==================== 主循环 (定时器回调) ====================

    def tick(self):
        """定时器回调：处理一帧视觉数据 (~30 Hz)"""
        # 读取 ROS 参数
        start_vision = self.get_parameter('START_VISION').get_parameter_value().bool_value
        kfs_status = self.get_parameter('KFS_Status').get_parameter_value().string_value
        is_gripped = self.get_parameter('IS_GRIPPED').get_parameter_value().string_value

        # ---------------------------------------------------------
        # 0. START_VISION 开关 — False 时跳过所有视觉处理
        # ---------------------------------------------------------
        if not start_vision:
            # 仅处理夹爪反馈（保持状态机连通）
            if self.grip_waiting and is_gripped == "GRIPPED":
                self.grip_waiting = False
                self.locked_target_coords = None
                self.locked_target_name = ""
                self.set_parameters(
                    [rclpy.parameter.Parameter('IS_GRIPPED',
                        rclpy.parameter.Parameter.Type.STRING, 'NOT_GRIPPED')]
                )
                self.get_logger().info(
                    "======> [夹爪反馈] IS_GRIPPED=GRIPPED，抓取完成（视觉已暂停）")
            return  # 跳过推理和渲染

        # ---------------------------------------------------------
        # 1. 检查夹爪反馈 (IS_GRIPPED 参数)
        #    外部节点设置 IS_GRIPPED="GRIPPED" 表示抓取完成
        #    本节点检测后解除等待锁并自动重置为 "NOT_GRIPPED"
        # ---------------------------------------------------------
        if self.grip_waiting and is_gripped == "GRIPPED":
            self.grip_waiting = False
            self.locked_target_coords = None
            self.locked_target_name = ""
            # 重置参数，等待下次外部写入
            self.set_parameters(
                [rclpy.parameter.Parameter('IS_GRIPPED',
                    rclpy.parameter.Parameter.Type.STRING, 'NOT_GRIPPED')]
            )
            self.get_logger().info(
                "======> [夹爪反馈] IS_GRIPPED=GRIPPED，抓取完成，解除等待锁，继续搜索下一个目标")

        # ---------------------------------------------------------
        # 2. 异步处理 STM32 的反馈 (已禁用)
        # ---------------------------------------------------------
        # if self.ser is not None and self.ser.in_waiting > 0:
        #     response = self.ser.read(1)
        #     pd15_val = response[0]
        #     logging.info(
        #         f"======> [下位机反馈 RX] 收到 PD15 状态: {'高电平 (1)' if pd15_val == 1 else '低电平 (0)'}")
        #     self.grip_waiting = False

        # ---------------------------------------------------------
        # 3. 拉取画面与模型推理
        # ---------------------------------------------------------
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        results = self.model(color_image, conf=CONF_THRESHOLD, verbose=False)[0]

        best_target_coords = None
        best_target_name = "1"

        # 遍历画面中的检测框
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            class_id = int(box.cls[0].item())

            target_info = CLASS_NAMES.get(
                class_id,
                {"name": "未知目标", "is_real": False, "color": (128, 128, 128)}
            )
            target_name = target_info["name"]
            is_real = target_info["is_real"]
            box_color = target_info["color"]

            status_str = "【真 - 允许抓取】" if is_real else "【假 - 危险勿碰】"

            # 计算 2D 像素中心与深度
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            dist_m = self.get_robust_depth(depth_frame, cx, cy, DEPTH_SAMPLE_SIZE)

            # 有效测距范围
            if 0.1 < dist_m < 8.0:
                # 反投影：计算物体表面中心点的 3D 相机坐标
                point_3d = rs.rs2_deproject_pixel_to_point(
                    self.color_intrinsics, [cx, cy], dist_m
                )

                # 几何中心修正 (Z轴推进 175mm)
                X_center = point_3d[0] * 1000
                Y_center = point_3d[1] * 1000
                Z_center = (point_3d[2] * 1000) + KFS_HALF_DEPTH

                # 记录并渲染目标信息 (等待抓取期间不频繁打印日志)
                log_msg = (f"发现: {target_name} | {status_str} | "
                           f"XYZ: {X_center:>5.0f}, {Y_center:>5.0f}, {Z_center:>5.0f} mm | "
                           f"KFS: {kfs_status} | IS_GRIPPED: {is_gripped}")
                if not self.grip_waiting:
                    self.get_logger().info(log_msg)

                # UI 渲染
                cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2)
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                text = f"{target_name} {status_str}"
                cv2.putText(color_image, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                cv2.putText(color_image, f"XYZ: {X_center:.0f}, {Y_center:.0f}, {Z_center:.0f} mm",
                            (x1, y2 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # ---------------------------------------------------------
                # 4. 筛选真实目标 — 锁定第一个真秘籍并进入等待夹爪状态
                # ---------------------------------------------------------
                if is_real and best_target_coords is None and not self.grip_waiting:
                    best_target_coords = (X_center, Y_center, Z_center)
                    best_target_name = target_name

        # ---------------------------------------------------------
        # 5. 目标锁定与等待逻辑
        #    发现真目标 → 锁定坐标 → 设置 grip_waiting → 等待外部设 IS_GRIPPED="GRIPPED"
        # ---------------------------------------------------------
        if best_target_coords is not None and not self.grip_waiting:
            x_mm, y_mm, z_mm = best_target_coords

            # ★★★ 这里需要根据你的实际情况添加手眼标定平移量 ★★★
            # x_robot = x_mm + 150
            # y_robot = y_mm
            # z_robot = z_mm + 50

            # 锁定目标，进入等待
            self.locked_target_coords = (x_mm, y_mm, z_mm)
            self.locked_target_name = best_target_name
            self.grip_waiting = True
            self.get_logger().info(
                f"====> [目标锁定] 已锁定 {best_target_name} "
                f"坐标: ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm | "
                f"等待 IS_GRIPPED=GRIPPED ...")

        # 在画面上叠加状态信息
        # 第一行: KFS_Status
        cv2.putText(color_image, f"KFS: {kfs_status}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        # 第二行: IS_GRIPPED 状态
        grip_color = (0, 255, 0) if not self.grip_waiting else (0, 165, 255)
        cv2.putText(color_image, f"GRIP: {is_gripped}{' (waiting...)' if self.grip_waiting else ''}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, grip_color, 2)
        # 第三行: 锁定目标信息
        if self.grip_waiting and self.locked_target_coords is not None:
            lx, ly, lz = self.locked_target_coords
            cv2.putText(color_image,
                        f"Locked: {self.locked_target_name} @ ({lx:.0f}, {ly:.0f}, {lz:.0f}) mm",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        # 刷新画面
        cv2.imshow('R2 Vision - Kung Fu Quest Pro', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("接收到退出指令，视觉程序终止。")
            rclpy.shutdown()

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.pipeline.stop()
        cv2.destroyAllWindows()
        # if self.ser:
        #     self.ser.close()
        #     self.get_logger().info("串口已安全关闭。")
        self.get_logger().info("视觉节点已安全关闭。")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
