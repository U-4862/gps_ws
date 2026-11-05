#!/usr/bin/env python3

import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# 根据实际 /joy 输出调整这些索引
AXIS = {
    "LX": 0,
    "LY": 1,
    "LT": 2,
    "RX": 3,
    "RY": 4,
    "RT": 5,
    "DPAD_X": 6,
    "DPAD_Y": 7,
}

# 以下是常见映射，具体按键编号需要实测
BUTTONS = [
    "A", "B", "X", "Y",
    "LB", "RB", "VIEW", "MENU",
    "L3", "R3", "GUIDE"
]

# 60 FPS。Tkinter 的 after 只是上限，实际帧率受绘制耗时影响。
FRAME_MS = 16

# 方向校准：不同驱动/映射下轴方向可能不同。
# 当前实机测试 X、Y 都需要取反；若某个方向不对，单独改对应的符号。
STICK_X_SIGN = -1.0
STICK_Y_SIGN = -1.0


class JoyMonitor(Node):
    def __init__(self):
        super().__init__("xbox_monitor")
        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self.received = False

        self.create_subscription(
            Joy, "/joy", self.callback, 10
        )

    def callback(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        self.received = True

    def axis(self, name):
        idx = AXIS[name]
        return self.axes[idx] if idx < len(self.axes) else 0.0


class XboxGUI:
    def __init__(self, ros_node):
        self.node = ros_node

        self.root = tk.Tk()
        self.root.title("ROS 2 Xbox Controller Monitor - Press Q to quit")
        self.root.geometry("760x680")
        self.root.configure(bg="#171b24")

        self.canvas = tk.Canvas(
            self.root,
            width=740,
            height=600,
            bg="#171b24",
            highlightthickness=0
        )
        self.canvas.pack(padx=10, pady=10)

        self.root.protocol(
            "WM_DELETE_WINDOW", self.close
        )
        self.root.bind("<KeyPress>", self._on_key)
        self.canvas.focus_set()
        self.running = True

        self._build_static()
        self.update()

    def _build_static(self):
        c = self.canvas

        c.create_text(
            370, 30,
            text="ROS 2 Xbox Controller Monitor",
            fill="#e5e7eb",
            font=("DejaVu Sans", 20),
            anchor="center"
        )
        self.status_id = c.create_text(
            370, 65,
            text="Waiting for /joy ...",
            fill="#f59e0b",
            font=("DejaVu Sans", 12),
            anchor="center"
        )

        self.left_stick = self._build_stick(190, 205, "LEFT STICK")
        self.right_stick = self._build_stick(550, 205, "RIGHT STICK")

        self.left_trigger = self._build_trigger(95, 360, "LT")
        self.right_trigger = self._build_trigger(465, 360, "RT")

        c.create_text(
            370, 435,
            text="BUTTON STATUS (GREEN = PRESSED)",
            fill="#e5e7eb",
            font=("DejaVu Sans", 13),
            anchor="center"
        )

        self.button_items = []
        for i, name in enumerate(BUTTONS):
            col = i % 6
            row = i // 6

            x = 32 + col * 119
            y = 470 + row * 60

            rect = c.create_rectangle(
                x, y, x + 104, y + 42,
                fill="#334155",
                outline="#64748b"
            )
            c.create_text(
                x + 52, y + 21,
                text=f"{i}: {name}",
                fill="#e5e7eb",
                font=("DejaVu Sans", 12),
                anchor="center"
            )
            self.button_items.append((rect, i))

        self.dpad_id = c.create_text(
            370, 595,
            text="D-PAD: X=+0.0  Y=+0.0",
            fill="#e5e7eb",
            font=("DejaVu Sans", 12),
            anchor="center"
        )

    def _build_stick(self, cx, cy, name):
        c = self.canvas
        r = 75

        c.create_oval(
            cx - r, cy - r, cx + r, cy + r,
            outline="#475569", width=2
        )
        c.create_line(
            cx - r, cy, cx + r, cy,
            fill="#334155", dash=(4, 4)
        )
        c.create_line(
            cx, cy - r, cx, cy + r,
            fill="#334155", dash=(4, 4)
        )
        c.create_text(
            cx, cy - 100,
            text=name,
            fill="#e5e7eb",
            font=("DejaVu Sans", 14),
            anchor="center"
        )

        line = c.create_line(
            cx, cy, cx, cy,
            fill="#22c55e", width=3
        )
        knob = c.create_oval(
            cx - 12, cy - 12, cx + 12, cy + 12,
            fill="#22c55e", outline="#86efac", width=2
        )
        value = c.create_text(
            cx, cy + 102,
            text="X: +0.000   Y: +0.000",
            fill="#94a3b8",
            font=("DejaVu Sans", 11),
            anchor="center"
        )

        return {
            "cx": cx, "cy": cy, "r": r,
            "line": line, "knob": knob, "value": value,
        }

    def _build_trigger(self, x, y, name):
        c = self.canvas

        c.create_rectangle(
            x, y, x + 180, y + 20,
            fill="#334155", outline=""
        )
        fill = c.create_rectangle(
            x, y, x, y + 20,
            fill="#22c55e", outline=""
        )
        text = c.create_text(
            x + 90, y - 16,
            text=f"{name}: 0%",
            fill="#e5e7eb",
            font=("DejaVu Sans", 12),
            anchor="center"
        )

        return {
            "x": x, "y": y, "name": name,
            "fill": fill, "text": text,
        }

    def _update_stick(self, stick, x, y):
        c = self.canvas
        x = max(-1.0, min(1.0, float(x)))
        y = max(-1.0, min(1.0, float(y)))

        px = stick["cx"] + STICK_X_SIGN * x * stick["r"]
        py = stick["cy"] + STICK_Y_SIGN * y * stick["r"]

        c.coords(
            stick["line"],
            stick["cx"], stick["cy"], px, py
        )
        c.coords(
            stick["knob"],
            px - 12, py - 12, px + 12, py + 12
        )
        c.itemconfigure(
            stick["value"],
            text=f"X: {x:+.3f}   Y: {y:+.3f}"
        )

    def _update_trigger(self, trigger, value):
        value = max(-1.0, min(1.0, float(value)))
        percent = max(0.0, min(1.0, (1.0 - value) / 2.0))

        self.canvas.coords(
            trigger["fill"],
            trigger["x"], trigger["y"],
            trigger["x"] + 180 * percent, trigger["y"] + 20
        )
        self.canvas.itemconfigure(
            trigger["text"],
            text=f"{trigger['name']}: {percent:.0%}"
        )

    def update(self):
        if not self.running:
            return

        # Ctrl-C / SIGTERM 时 rclpy 会先 shutdown，Tk 仍在跑，
        # 这里主动退出，避免 spin_once 反复抛 RCLError。
        if not rclpy.ok():
            self.close()
            return

        # 每帧最多处理 10 条，避免 GUI 卡顿时队列积压导致延迟
        try:
            for _ in range(10):
                rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            if rclpy.ok():
                raise
            self.close()
            return

        c = self.canvas
        status = (
            "Receiving /joy"
            if self.node.received
            else "Waiting for /joy ..."
        )
        c.itemconfigure(
            self.status_id,
            text=status,
            fill="#22c55e" if self.node.received else "#f59e0b"
        )

        self._update_stick(
            self.left_stick,
            self.node.axis("LX"),
            self.node.axis("LY")
        )
        self._update_stick(
            self.right_stick,
            self.node.axis("RX"),
            self.node.axis("RY")
        )

        self._update_trigger(
            self.left_trigger,
            self.node.axis("LT")
        )
        self._update_trigger(
            self.right_trigger,
            self.node.axis("RT")
        )

        for rect, i in self.button_items:
            pressed = (
                i < len(self.node.buttons)
                and self.node.buttons[i] == 1
            )
            c.itemconfigure(
                rect,
                fill="#16a34a" if pressed else "#334155"
            )

        c.itemconfigure(
            self.dpad_id,
            text=(
                f"D-PAD: X={self.node.axis('DPAD_X'):+.1f}"
                f"  Y={self.node.axis('DPAD_Y'):+.1f}"
            )
        )

        self.root.after(FRAME_MS, self.update)

    def _on_key(self, event):
        if getattr(event, "char", "").lower() == "q":
            self.close()

    def close(self):
        if not self.running:
            return
        self.running = False
        if self.root is not None:
            self.root.destroy()
            self.root = None

    def run(self):
        if self.root is not None:
            self.root.mainloop()


def main():
    rclpy.init()
    node = JoyMonitor()

    try:
        gui = XboxGUI(node)
        gui.run()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
