#!/usr/bin/env python3
"""
临时脚本：使用 RealSense D435i 彩色帧检测灰色斑块 (grey blob)。

思路：
1) 在 HSV 空间中筛选低饱和度、中等明度区域视为"灰色"。
2) 形态学开/闭去噪。
3) 提取轮廓，过滤面积，绘制外接矩形与质心。
按 q 退出，按 s 保存当前帧到 grey_blob_snapshot.png。
"""

import cv2
import numpy as np
import pyrealsense2 as rs


# ==================== 可调参数 ====================
FRAME_W, FRAME_H, FPS = 640, 480, 30

# 灰色 HSV 门限：低饱和度 + 中等亮度
H_MIN, H_MAX = 0, 180        # 灰色不依赖色相
S_MIN, S_MAX = 0, 50         # 饱和度低 → 无明显颜色
V_MIN, V_MAX = 60, 200       # 亮度中段 → 排除纯黑 / 高光白

MORPH_KERNEL = 5             # 形态学核尺寸
MIN_AREA = 800               # 最小 blob 面积 (像素)
MAX_AREA = 200_000           # 最大 blob 面积 (像素)

WINDOW_MAIN = "Grey Blob Detect"
WINDOW_MASK = "Grey Mask"
# ==================================================


def build_grey_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([H_MIN, S_MIN, V_MIN], dtype=np.uint8)
    upper = np.array([H_MAX, S_MAX, V_MAX], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def find_blobs(mask: np.ndarray):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA or area > MAX_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        blobs.append({"bbox": (x, y, w, h), "centroid": (cx, cy), "area": area})
    return blobs


def annotate(frame: np.ndarray, blobs, depth_frame=None):
    for i, b in enumerate(blobs):
        x, y, w, h = b["bbox"]
        cx, cy = b["centroid"]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 200, 255), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

        label = f"#{i} area={int(b['area'])}"
        if depth_frame is not None:
            dist_m = depth_frame.get_distance(cx, cy)
            if dist_m > 0:
                label += f" d={dist_m:.2f}m"
        cv2.putText(frame, label, (x, max(0, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, FRAME_W, FRAME_H, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, FRAME_W, FRAME_H, rs.format.z16, FPS)

    pipeline.start(config)
    align = rs.align(rs.stream.color)

    print("[grey_blob_detect] 启动完成 — q 退出, s 保存快照")

    snapshot_idx = 0
    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            mask = build_grey_mask(frame)
            blobs = find_blobs(mask)
            annotate(frame, blobs, depth_frame)

            cv2.putText(frame, f"blobs: {len(blobs)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow(WINDOW_MAIN, frame)
            cv2.imshow(WINDOW_MASK, mask)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord('s'):
                fname = f"grey_blob_snapshot_{snapshot_idx:02d}.png"
                cv2.imwrite(fname, frame)
                print(f"[grey_blob_detect] 已保存 {fname}")
                snapshot_idx += 1
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
