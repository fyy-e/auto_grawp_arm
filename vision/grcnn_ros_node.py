#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GRCNN 抓取检测 ROS2 节点（对标 detect_3d1.py 架构）
=====================================================
相机由独立的 realsense2_camera 驱动进程提供（与 YOLO 方案完全相同的
启动方式），本节点只做：订阅图像 → GRCNN 推理 → 显示 → UDP 发送。

架构：
  ROS 回调（主线程）—— 每帧刷新显示窗口（流畅）
  推理工作线程      —— 全速跑 GRCNN，更新抓取框 + 发 UDP（~15Hz）

启动（两个终端，先 1 后 2）：
  1) source /opt/ros/humble/setup.bash
     ros2 launch realsense2_camera rs_align_depth_launch.py \
         enable_gyro:=false enable_accel:=false unite_imu_method:=0 \
         depth_module.depth_profile:=640x360x30 rgb_camera.color_profile:=640x360x30
  2) source /opt/ros/humble/setup.bash
     cd ~/dm_arm_end && python3 vision/grcnn_ros_node.py

注意：不要同时运行 vision/grcnn_server.py（相机会被 ROS 驱动独占）。
UDP 协议与 C++ 端 VisionDetector 一致：20 字节 [x,y,z,angle,width]，端口 5005。
"""

import os
import sys
import json
import socket
import struct
import threading
import time

import cv2
import numpy as np
import torch

VISION_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, VISION_DIR)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import String
import message_filters

from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import detect_grasps
from grasp_tracker import GraspTracker

MIN_DEPTH, MAX_DEPTH = 0.10, 1.00   # 深度有效区间 (m)


class GraspRosNode(Node):
    def __init__(self):
        super().__init__('grcnn_grasp')

        # ---- GRCNN 模型 ----
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        torch.backends.cudnn.benchmark = True
        model_path = os.path.join(
            VISION_DIR, "models/cornell-randsplit-rgbd-grconvnet3-drop1-ch32/epoch_19_iou_0.98")
        try:
            self.model = torch.load(model_path, map_location=self.device,
                                    weights_only=False)
        except TypeError:
            self.model = torch.load(model_path, map_location=self.device)
        self.model.eval()
        self.get_logger().info(f"GRCNN 模型已加载 -> {self.device}")

        self.cam_data = None          # 首帧到来时按实际分辨率初始化
        self.bridge = CvBridge()

        # ---- UDP（与 C++ VisionDetector 约定）----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_target = ("127.0.0.1", 5005)

        # ---- 线程间共享 ----
        self.frame_lock = threading.Lock()
        self.latest_frame = None       # (color_bgr, depth_m)
        self.det_lock = threading.Lock()
        self.latest_det = None         # 最新抓取结果 dict 或 None
        self.infer_running = True

        self.color_camera_info = None
        self.sent_count = 0
        self.tracker = GraspTracker()   # 帧间跟踪+EMA 平滑，防对称峰横跳

        # ---- ROS 订阅（与 detect_3d1.py 相同的话题）----
        self.info_color_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.color_info_callback, 10)
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.result_pub = self.create_publisher(String, '/grasp_3d', 10)

        # ---- 推理工作线程 ----
        self.infer_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.infer_thread.start()

        self.get_logger().info("GRCNN + D435i 抓取检测节点已启动")

    def color_info_callback(self, msg):
        if self.color_camera_info is None:
            self.color_camera_info = msg
            K = msg.k
            self.get_logger().info(
                f"彩色内参: fx={K[0]:.2f} fy={K[4]:.2f} cx={K[2]:.2f} cy={K[5]:.2f}")
            self.destroy_subscription(self.info_color_sub)

    # ================= ROS 回调：收帧 + 显示（每帧都跑，保证流畅）=================
    def sync_callback(self, color_msg, depth_msg):
        if self.color_camera_info is None:
            self.get_logger().warn("等待相机内参...", throttle_duration_sec=5)
            return
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            if depth_msg.encoding == '16UC1':
                depth_m = self.bridge.imgmsg_to_cv2(
                    depth_msg, "16UC1").astype(np.float32) / 1000.0
            else:
                depth_m = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        if self.cam_data is None:
            h, w = depth_m.shape[:2]
            self.cam_data = CameraData(width=w, height=h, output_size=300,
                                       include_rgb=True, include_depth=True)
            self.get_logger().info(f"图像尺寸 {w}x{h}，网络视野 300x300 中心裁剪")

        with self.frame_lock:
            self.latest_frame = (color_image[:, :, ::-1].copy(), depth_m)

        # 每帧都刷新显示：视频流畅，抓取框叠加最新推理结果
        with self.det_lock:
            det = self.latest_det
        cv2.imshow("GRCNN grasp (q=quit)", self.draw_overlay(color_image, det))
        cv2.waitKey(1)

    # ================= 推理工作线程 =================
    def inference_loop(self):
        while self.infer_running:
            with self.frame_lock:
                frame = self.latest_frame
            if frame is None or self.cam_data is None:
                time.sleep(0.01)
                continue
            rgb, depth_m = frame

            depth_in = depth_m.copy()
            depth_in[(depth_in > MAX_DEPTH) | (depth_in < MIN_DEPTH)] = 0
            x, _, _ = self.cam_data.get_data(rgb=rgb,
                                             depth=np.expand_dims(depth_in, axis=2))
            with torch.no_grad():
                pred = self.model.predict(x.to(self.device))
            q_img, ang_img, width_img = post_process_output(
                pred['pos'], pred['cos'], pred['sin'], pred['width'])
            grasps = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=5)

            if len(grasps) == 0:
                with self.det_lock:
                    self.latest_det = None
                continue

            # 帧间跟踪 + EMA 平滑：防止对称峰之间逐帧横跳（绿框乱飞的根源）
            g = self.tracker.update(grasps, q_img)
            if g is None:
                with self.det_lock:
                    self.latest_det = None
                continue
            v = int(round(g["row"] + self.cam_data.top_left[0]))
            u = int(round(g["col"] + self.cam_data.top_left[1]))
            v = min(max(v, 0), depth_m.shape[0] - 1)
            u = min(max(u, 0), depth_m.shape[1] - 1)

            z = float(depth_m[v, u])
            if not (MIN_DEPTH <= z <= MAX_DEPTH):   # 越界深度拦截，防垃圾坐标
                patch = depth_m[max(0, v-2):v+3, max(0, u-2):u+3]
                valid = patch[(patch >= MIN_DEPTH) & (patch <= MAX_DEPTH)]
                if valid.size == 0:
                    with self.det_lock:
                        self.latest_det = None
                    continue
                z = float(np.median(valid))

            K = self.color_camera_info.k
            fx, fy, cx0, cy0 = K[0], K[4], K[2], K[5]
            X = (u - cx0) * z / fx
            Y = (v - cy0) * z / fy
            angle = float(g["angle"])
            width_m = max(float(g["length"]) * z / fx, 0.005)   # 下限 5mm

            # UDP 发送（20 字节 GRCNN 协议）
            self.sock.sendto(struct.pack('<5f', X, Y, z, angle, width_m),
                             self.udp_target)
            self.sent_count += 1

            with self.det_lock:
                self.latest_det = {
                    "u": u, "v": v, "angle": angle,
                    "width_px": float(g["length"]), "width_m": width_m, "z": z,
                }

            # ROS 话题发布（调试/上位机可用）
            msg = String()
            msg.data = json.dumps({
                "camera_xyz_mm": [round(X*1000, 1), round(Y*1000, 1), round(z*1000, 1)],
                "angle_rad": round(angle, 3), "width_mm": round(width_m*1000, 1),
            }, ensure_ascii=False)
            self.result_pub.publish(msg)

            if self.sent_count % 15 == 1:
                self.get_logger().info(
                    f"[#{self.sent_count}] cam=({X:+.3f},{Y:+.3f},{z:.3f})m "
                    f"angle={angle:+.2f}rad width={width_m*1000:.0f}mm")

    # ================= 显示叠加 =================
    def draw_overlay(self, bgr, det):
        img = bgr.copy()
        top, left = self.cam_data.top_left
        size = self.cam_data.output_size
        cv2.rectangle(img, (left, top), (left + size, top + size), (0, 255, 255), 1)
        if det:
            cx, cy = int(det["u"]), int(det["v"])
            angle, length = det["angle"], det["width_px"]
            half_w = length / 2
            dx, dy = np.cos(-angle), np.sin(-angle)
            px, py = -dy, dx
            pts = np.array([(cx - dx*length/2 - px*half_w/2, cy - dy*length/2 - py*half_w/2),
                            (cx + dx*length/2 - px*half_w/2, cy + dy*length/2 - py*half_w/2),
                            (cx + dx*length/2 + px*half_w/2, cy + dy*length/2 + py*half_w/2),
                            (cx - dx*length/2 + px*half_w/2, cy - dy*length/2 - py*half_w/2)],
                           dtype=np.int32)
            # 半透明填充 + 绿色轮廓 + 两条夹爪长边标黄（与官方可视化一致）
            overlay = img.copy()
            cv2.fillPoly(overlay, [pts], (0, 200, 0))
            cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)
            cv2.polylines(img, [pts], True, (0, 255, 0), 2)
            cv2.line(img, tuple(pts[0]), tuple(pts[1]), (0, 255, 255), 3)
            cv2.line(img, tuple(pts[2]), tuple(pts[3]), (0, 255, 255), 3)
            cv2.drawMarker(img, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
            cv2.putText(img, f"ang={angle:+.2f} w={det['width_m']*1000:.0f}mm "
                             f"z={det['z']:.2f}m",
                        (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(img, "no grasp", (left + 6, top + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return img

    def destroy_node(self):
        self.infer_running = False
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():          # timeout/SIGINT 场景下可能已被关闭
            rclpy.shutdown()


if __name__ == '__main__':
    main()
