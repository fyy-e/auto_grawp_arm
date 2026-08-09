# -*- coding: utf-8 -*-
"""UDP 链路自检：模拟 C++ VisionDetector 接收 20 字节抓取包
用法（在 dm_arm_end 根目录）：
    python3 vision/test_udp_link.py
另开一个终端运行：python3 vision/grcnn_server.py --offline
"""
import socket
import struct
import subprocess
import sys
import os

VISION_DIR = os.path.dirname(os.path.abspath(__file__))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005))
sock.settimeout(20)

proc = subprocess.Popen([sys.executable,
                         os.path.join(VISION_DIR, "grcnn_server.py"),
                         "--offline", "--offline-num", "2"])
try:
    data, addr = sock.recvfrom(64)
    print(f"收到 {len(data)} 字节，来自 {addr}")
    if len(data) == 20:
        x, y, z, angle, width = struct.unpack("<5f", data)
        print(f"解析: x={x:+.4f} y={y:+.4f} z={z:.4f} angle={angle:+.4f}rad "
              f"width={width*1000:.1f}mm")
        print("协议校验: PASS (20B GRCNN 协议)")
    else:
        print(f"协议校验: FAIL（期望 20 字节，收到 {len(data)} 字节）")
except socket.timeout:
    print("协议校验: FAIL（20s 未收到包）")
finally:
    proc.wait()
    sock.close()
