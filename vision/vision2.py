import cv2
import numpy as np
import pyrealsense2 as rs

# HSV ranges for red (wraps around 0/180 in OpenCV)
# LOWER_RED1 = np.array([0, 120, 70])
# UPPER_RED1 = np.array([10, 255, 255])
# LOWER_RED2 = np.array([170, 120, 70])
# UPPER_RED2 = np.array([180, 255, 255])


# LOWER_RED1 = np.array([0, 120, 70])
# UPPER_RED1 = np.array([5, 255, 255])   # 上限从 10 缩小到 5

# LOWER_RED2 = np.array([175, 120, 70])  # 下限从 170 扩大到 175
# UPPER_RED2 = np.array([180, 255, 255])


LOWER_RED1 = np.array([0, 200, 200])   # 提高饱和度到 200，提高亮度到 200
UPPER_RED1 = np.array([10, 255, 255])

# 范围 2：针对边缘的深红色（如果 LED 中心过曝变白，可能需要这个范围来补全轮廓）
LOWER_RED2 = np.array([170, 200, 200]) # 同样提高饱和度和亮度
UPPER_RED2 = np.array([180, 255, 255])


MIN_CONTOUR_AREA = 20

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    profile = pipeline.start(config)
except Exception as e:
    print(f"Cannot start RealSense camera: {e}")
    exit(1)

color_profile = profile.get_stream(rs.stream.color)
color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
align = rs.align(rs.stream.color)

print("Press 'q' to quit")

while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()


    if not depth_frame or not color_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # ... (前面的颜色阈值设置和 cv2.inRange 代码保持不变) ...

# 1. 合并两个红色区间的 Mask
    mask = cv2.bitwise_or(mask1, mask2)

# --- 【新增部分】形态学处理 ---

# 定义卷积核 (Kernel)
# 大小建议为奇数，如 (5,5), (7,7), (9,9)。LED 光晕越大，数值设得越大。
    kernel_size = 15
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

# 执行闭运算：先膨胀后腐蚀
# 作用：填补前景物体（红色 LED）内部的小孔洞（过曝的白色区域），并连接邻近的断裂区域
    mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# (可选) 如果边缘太粗糙，可以加一步开运算去除噪点
# mask_clean = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))

# ---------------------------

# 2. 使用处理后的 mask_closed 进行轮廓查找
    contours, _ = cv2.findContours(mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ... (后续的绘图代码) ...

# 在左侧窗口显示原始 Mask 用于对比调试
    cv2.imshow("Original Mask", mask)
# 在右侧窗口显示处理后的 Mask
    cv2.imshow("Closed Mask", mask_closed)


    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        if cv2.contourArea(c) < MIN_CONTOUR_AREA:
            continue

        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 3D coordinates from depth frame
        center_x = x + w // 2
        center_y = y + h // 2
        depth_m = depth_frame.get_distance(center_x, center_y)
        if depth_m > 0:
            point_3d = rs.rs2_deproject_pixel_to_point(
                color_intrinsics, [center_x, center_y], depth_m
            )
            label = f"LED {point_3d[0]:.2f},{point_3d[1]:.2f},{point_3d[2]:.2f}m"
            cv2.putText(color_image, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('Red LED Detection', color_image)
    cv2.imshow('Red Mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()