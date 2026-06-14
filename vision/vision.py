#!/usr/bin/env python3
"""
兼容入口：直接运行 python vision.py 启动视觉节点
（推荐使用 ros2 run vision vision_node 启动）
"""

from vision.vision import main

if __name__ == "__main__":
    main()
