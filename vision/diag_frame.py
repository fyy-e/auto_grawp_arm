# -*- coding: utf-8 -*-
"""
GRCNN 实时帧诊断脚本
=====================
从 D435i 抓 N 帧实时画面，用与 grcnn_server.py 完全相同的预处理+模型
离线推理，输出：
  1. 深度有效率（ROI 内 / 全图）——判断"距离太近深度烂"假设
  2. 质量图统计（峰值、分布范围）——判断网络是否"拿不准"导致框乱飞
  3. 每帧 top-3 抓取候选（中心/角度/宽度/质量分）——看帧间是否稳定
  4. 保存 PNG：RGB 叠加抓取框、质量图热力图、深度可视化

用法（dm_arm_end 工程根目录，先停掉占用相机的 grcnn_server / ros 驱动）：
    python3 vision/diag_frame.py                # 默认抓 5 帧，间隔 0.4s
    python3 vision/diag_frame.py --frames 8 --out vision/out/diag2
"""

import argparse
import os
import sys
import time

import numpy as np
import torch

GRCNN_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, GRCNN_ROOT)

from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import detect_grasps

MIN_DEPTH, MAX_DEPTH = 0.10, 1.00


def load_model(model_path, device):
    try:
        model = torch.load(model_path, map_location=device, weights_only=False)
    except TypeError:
        model = torch.load(model_path, map_location=device)
    model.eval()
    return model


def make_camera():
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipe.start(cfg)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    filters = [rs.spatial_filter(), rs.temporal_filter(0.4, 20, 3),
               rs.hole_filling_filter(1)]
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    return pipe, align, filters, depth_scale, intr


def grab(pipe, align, filters, depth_scale):
    frames = pipe.wait_for_frames(10000)
    while True:
        newer = pipe.poll_for_frames()
        if not newer:
            break
        frames = newer
    frames = align.process(frames)
    depth_frame = frames.get_depth_frame()
    for f in filters:
        depth_frame = f.process(depth_frame)
    color = np.asanyarray(frames.get_color_frame().get_data())[:, :, ::-1].copy()
    depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
    return color, depth_m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=os.path.join(
        GRCNN_ROOT, 'models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93'))
    ap.add_argument('--frames', type=int, default=5)
    ap.add_argument('--interval', type=float, default=0.4)
    ap.add_argument('--out', default=os.path.join(GRCNN_ROOT, 'out/diag'))
    args = ap.parse_args()

    import cv2
    os.makedirs(args.out, exist_ok=True)

    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    print(f"[diag] 加载模型 -> {device}")
    model = load_model(args.model, device)
    if device.type == 'cuda':
        torch.backends.cudnn.benchmark = True

    cam_data = CameraData(width=640, height=480, output_size=300,
                          include_rgb=True, include_depth=True)

    pipe, align, filters, depth_scale, intr = make_camera()
    print(f"[diag] 相机已开 depth_scale={depth_scale} fx={intr.fx:.1f}")

    try:
        for i in range(args.frames):
            rgb, depth_m = grab(pipe, align, filters, depth_scale)

            depth_in = depth_m.copy()
            depth_in[(depth_in > MAX_DEPTH) | (depth_in < MIN_DEPTH)] = 0

            x, _, _ = cam_data.get_data(rgb=rgb, depth=np.expand_dims(depth_in, axis=2))
            top, left = cam_data.top_left
            size = cam_data.output_size

            with torch.no_grad():
                pred = model.predict(x.to(device))
            q_img, ang_img, width_img = post_process_output(
                pred['pos'], pred['cos'], pred['sin'], pred['width'])
            grasps = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=3)

            # ---- 深度有效率统计 ----
            roi_depth = depth_m[top:top + size, left:left + size]
            roi_valid = ((roi_depth >= MIN_DEPTH) & (roi_depth <= MAX_DEPTH)).mean()
            all_valid = ((depth_m >= MIN_DEPTH) & (depth_m <= MAX_DEPTH)).mean()
            if roi_valid > 0:
                valid_vals = roi_depth[(roi_depth >= MIN_DEPTH) & (roi_depth <= MAX_DEPTH)]
                z_med = float(np.median(valid_vals))
            else:
                z_med = float('nan')

            # ---- 质量图统计（判断"乱飞"根源）----
            q_max = float(q_img.max())
            n_peaks = int((q_img > 0.7 * q_max).sum()) if q_max > 0 else 0

            print(f"\n===== 帧 {i} =====")
            print(f"  深度有效率: ROI内={roi_valid*100:.1f}%  全图={all_valid*100:.1f}%  "
                  f"ROI中位深度={z_med:.3f}m")
            print(f"  质量图: max={q_max:.3f}  高于0.7*max的像素数={n_peaks}"
                  f"  (越大说明候选峰越多、框越容易跳)")

            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.rectangle(img, (left, top), (left + size, top + size), (0, 255, 255), 1)
            for k, g in enumerate(grasps):
                row, col = g.center
                v, u = row + top, col + left
                z = float(depth_m[v, u])
                width_m = float(g.length) * z / intr.fx if z > 0 else 0
                q_here = float(q_img[row, col])
                print(f"  候选{k}: 像素=({u},{v}) angle={g.angle:+.2f}rad "
                      f"width={width_m*1000:.0f}mm z={z:.3f}m q={q_here:.3f}")
                color = (0, 255, 0) if k == 0 else (0, 200, 255)
                cv2.drawMarker(img, (u, v), color, cv2.MARKER_CROSS, 14, 2)
                if k == 0:
                    dx, dy = np.cos(-g.angle), np.sin(-g.angle)
                    px, py = -dy, dx
                    L, hw = float(g.length), float(g.length) / 2
                    pts = np.array([(u-dx*L/2-px*hw/2, v-dy*L/2-py*hw/2),
                                    (u+dx*L/2-px*hw/2, v+dy*L/2-py*hw/2),
                                    (u+dx*L/2+px*hw/2, v+dy*L/2+py*hw/2),
                                    (u-dx*L/2+px*hw/2, v-dy*L/2-py*hw/2)], dtype=np.int32)
                    cv2.polylines(img, [pts], True, (0, 255, 0), 2)

            cv2.imwrite(os.path.join(args.out, f"frame{i}_rgb.png"), img)

            q_vis = cv2.applyColorMap((q_img / max(q_max, 1e-6) * 255).astype(np.uint8),
                                      cv2.COLORMAP_JET)
            cv2.imwrite(os.path.join(args.out, f"frame{i}_qmap.png"), q_vis)

            d_vis = np.clip(depth_m / MAX_DEPTH * 255, 0, 255).astype(np.uint8)
            d_vis = cv2.applyColorMap(d_vis, cv2.COLORMAP_JET)
            d_vis[depth_m < MIN_DEPTH] = (0, 0, 0)   # 无效深度标黑
            cv2.rectangle(d_vis, (left, top), (left + size, top + size), (255, 255, 255), 1)
            cv2.imwrite(os.path.join(args.out, f"frame{i}_depth.png"), d_vis)

            if i < args.frames - 1:
                time.sleep(args.interval)
    finally:
        pipe.stop()
    print(f"\n[diag] 完成，图片保存在 {args.out}")


if __name__ == '__main__':
    main()
