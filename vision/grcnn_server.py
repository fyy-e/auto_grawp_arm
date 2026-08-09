# -*- coding: utf-8 -*-
"""
GRCNN 视觉抓取服务端（dm_arm_end 视觉子系统）
=================================================
替代原 YOLO 视觉节点。从 D435i 取 RGB-D 图像，用 GR-ConvNet
预测最优抓取（中心像素 + 抓取角 + 夹爪宽度），反投影为相机系
3D 坐标后通过 UDP 发送。

协议：5 个 float32 小端 = 20 字节，端口 5005
    [ x, y, z, angle_rad, width_m ]

架构（对标原 YOLO 方案的流畅度）：
    取图线程   —— 相机 30fps 取图 + 深度对齐，只保留最新帧
    推理线程   —— 全速跑 GRCNN + UDP 发送（~10-15Hz）
    显示主线程 —— 画面按相机帧率刷新，抓取框叠加最新推理结果
    三条线解耦：视频流畅度不受推理速度影响。

运行（在 dm_arm_end 工程根目录）：
    python3 vision/grcnn_server.py                      # 实时模式（无界面）
    python3 vision/grcnn_server.py --visualize          # 弹窗实时画面
    python3 vision/grcnn_server.py --offline            # 无相机自检
"""

import argparse
import os
import struct
import sys
import threading
import time

import numpy as np
import torch

# vision/ 目录即 GRCNN 包根（inference/ 与 utils/ 在其下）
GRCNN_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, GRCNN_ROOT)

from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import detect_grasps
from grasp_tracker import GraspTracker


# ---------------------------------------------------------------- 模型
def load_model(model_path, device, fp16=False):
    """兼容新版 PyTorch（>=2.6 默认 weights_only=True）"""
    try:
        model = torch.load(model_path, map_location=device, weights_only=False)
    except TypeError:
        model = torch.load(model_path, map_location=device)
    model.eval()
    if fp16 and device.type == "cuda":
        model = model.half()          # Orin Tensor Core 半精度加速
        print("[模型] 已启用 fp16 半精度推理")
    return model


# ---------------------------------------------------------------- 相机
class RealSenseSource:
    """Intel D435i：640x480@30，深度对齐到彩色，可选后处理滤波链"""

    def __init__(self, width=640, height=480, fps=30, use_filters=True):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipe.start(cfg)
        dev = profile.get_device()
        print(f"[相机] 设备: {dev.get_info(rs.camera_info.name)} "
              f"固件: {dev.get_info(rs.camera_info.firmware_version)}")
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)

        # D435i 深度后处理：改善物体边缘与空洞（抓取点深度更可靠）
        # 注意1：不能用 decimation_filter——它会降低深度图分辨率，
        #       破坏 align 后深度与彩色 1:1 的像素对应关系
        # 注意2：ARM CPU 上 spatial/temporal 滤波较耗时（~25ms/帧），
        #       取图线程独立后不影响显示流畅度，但会略降推理频率
        self.filters = []
        if use_filters:
            self.filters = [
                rs.spatial_filter(),              # 边缘保持平滑
                rs.temporal_filter(0.4, 20, 3),   # 时间域防抖（眼在手上会轻微晃动）
                rs.hole_filling_filter(1),        # 空洞填充
            ]

        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy, self.cx, self.cy = intr.fx, intr.fy, intr.ppx, intr.ppy
        print(f"[相机] 内参 fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f} "
              f"depth_scale={self.depth_scale}")

    def get(self):
        frames = self.pipe.wait_for_frames(10000)
        # 关键：排空驱动缓冲区里的积压旧帧，只保留最新一组，
        # 否则处理速度跟不上 30fps 时延迟会持续累积
        while True:
            newer = self.pipe.poll_for_frames()
            if not newer:
                break
            frames = newer
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        for f in self.filters:
            depth_frame = f.process(depth_frame)
        color = np.asanyarray(frames.get_color_frame().get_data())[:, :, ::-1].copy()  # BGR->RGB
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_m = depth_raw.astype(np.float32) * self.depth_scale
        return color, depth_m

    def stop(self):
        self.pipe.stop()


class OfflineSource:
    """无相机自检：用 test_data/ 下的 cmp{1..6}.png / hmp{1..6}.png"""

    def __init__(self, num=2):
        from PIL import Image
        # 测试图由 D415 拍摄，内参与 cam_pose/camera_depth_scale.txt 一致
        self.fx, self.fy, self.cx, self.cy = 615.284, 614.557, 309.623, 247.967
        scale_path = os.path.join(GRCNN_ROOT, 'cam_pose/camera_depth_scale.txt')
        self.depth_scale = float(np.loadtxt(scale_path))
        rgb_path = os.path.join(GRCNN_ROOT, f"test_data/cmp{num}.png")
        depth_path = os.path.join(GRCNN_ROOT, f"test_data/hmp{num}.png")
        self._rgb = np.array(Image.open(rgb_path))
        self._depth = np.array(Image.open(depth_path)).astype(np.float32)

    def get(self):
        return self._rgb.copy(), (self._depth * self.depth_scale).copy()

    def stop(self):
        pass


# ---------------------------------------------------------------- 可视化
def draw_overlay_cv(rgb, det, cam_data):
    """在 RGB 图上画检测视野框 + 最新抓取结果，返回 BGR 图
    det 为 None 时显示 no grasp"""
    import cv2
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    top, left = cam_data.top_left
    size = cam_data.output_size
    cv2.rectangle(img, (left, top), (left + size, top + size), (0, 255, 255), 1)
    if det:
        cx, cy = int(det["u"]), int(det["v"])
        angle, width_px = det["angle"], det["width_px"]
        length = width_px
        half_w = width_px / 2
        dx, dy = np.cos(-angle), np.sin(-angle)
        px, py = -dy, dx
        pts = np.array([(cx - dx*length/2 - px*half_w/2, cy - dy*length/2 - py*half_w/2),
                        (cx + dx*length/2 - px*half_w/2, cy + dy*length/2 - py*half_w/2),
                        (cx + dx*length/2 + px*half_w/2, cy + dy*length/2 + py*half_w/2),
                        (cx - dx*length/2 + px*half_w/2, cy - dy*length/2 - py*half_w/2)],
                       dtype=np.int32)
        # 半透明填充：一眼看出"框选住了物体"
        overlay = img.copy()
        cv2.fillPoly(overlay, [pts], (0, 200, 0))
        cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)
        cv2.polylines(img, [pts], True, (0, 255, 0), 2)
        # 两条长边 = 夹爪指面，加粗标黄（对应官方图里的夹爪方向）
        cv2.line(img, tuple(pts[0]), tuple(pts[1]), (0, 255, 255), 3)
        cv2.line(img, tuple(pts[2]), tuple(pts[3]), (0, 255, 255), 3)
        cv2.drawMarker(img, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(img, f"ang={angle:+.2f}rad w={det['width_m']*1000:.0f}mm "
                         f"z={det['z']:.2f}m",
                    (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        cv2.putText(img, "no grasp", (left + 6, top + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return img


def save_vis(fig_dir, frame_id, rgb, det, cam_data):
    """SSH 无界面时用：把检测画面保存成图片"""
    import cv2
    os.makedirs(fig_dir, exist_ok=True)
    out = os.path.join(fig_dir, f"grasp_{frame_id:04d}.png")
    cv2.imwrite(out, draw_overlay_cv(rgb, det, cam_data))
    return out


# ---------------------------------------------------------------- 共享状态（线程间）
class Shared:
    def __init__(self):
        self.frame_lock = threading.Lock()
        self.latest_frame = None        # (rgb, depth_m)，取图线程写
        self.det_lock = threading.Lock()
        self.latest_det = None          # 最近一次有效抓取结果 dict
        self.running = True


# ---------------------------------------------------------------- 取图线程
def capture_loop(source, shared):
    while shared.running:
        try:
            rgb, depth_m = source.get()
        except RuntimeError as e:
            print(f"[取图] 相机错误: {e}")
            time.sleep(0.5)
            continue
        with shared.frame_lock:
            shared.latest_frame = (rgb, depth_m)


# ---------------------------------------------------------------- 推理线程
def inference_loop(args, model, cam_data, source, shared, sock, target, device):
    import socket as _s  # noqa
    sent = 0
    prof_cam = prof_prep = prof_infer = 0.0
    n = 0
    tracker = GraspTracker(alpha=args.track_alpha, max_miss=args.track_max_miss)
    while shared.running:
        with shared.frame_lock:
            frame = shared.latest_frame
        if frame is None:
            time.sleep(0.01)
            continue
        rgb, depth_m = frame
        t0 = time.time()

        depth_in = depth_m.copy()
        depth_in[depth_in > args.max_depth] = 0
        depth_in[depth_in < args.min_depth] = 0
        x, _, _ = cam_data.get_data(rgb=rgb, depth=np.expand_dims(depth_in, axis=2))
        t1 = time.time()

        if device.type == "cuda" and args.fp16:
            x = x.half()
        with torch.no_grad():
            pred = model.predict(x.to(device))
        q_img, ang_img, width_img = post_process_output(
            pred['pos'], pred['cos'], pred['sin'], pred['width'])
        grasps = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=5)
        t2 = time.time()

        prof_prep = 0.9 * prof_prep + 0.1 * (t1 - t0)
        prof_infer = 0.9 * prof_infer + 0.1 * (t2 - t1)
        n += 1
        if args.profile and n % args.print_every == 0:
            print(f"[计时] 预处理={prof_prep*1000:.0f}ms 推理+后处理={prof_infer*1000:.0f}ms "
                  f"(推理频率 {1.0/max(prof_prep+prof_infer,1e-3):.1f}Hz)")

        if len(grasps) == 0:
            with shared.det_lock:
                shared.latest_det = None
            continue

        # 帧间跟踪 + EMA 平滑：防止对称峰之间逐帧横跳（绿框乱飞的根源）
        g = tracker.update(grasps, q_img)
        if g is None:
            with shared.det_lock:
                shared.latest_det = None
            continue
        v = int(round(g["row"] + cam_data.top_left[0]))  # 还原到原始 640x480
        u = int(round(g["col"] + cam_data.top_left[1]))
        v = min(max(v, 0), depth_m.shape[0] - 1)
        u = min(max(u, 0), depth_m.shape[1] - 1)

        z = float(depth_m[v, u])
        # 深度有效性检查：越界（0/超远/超近）同样无效，
        # 防止把垃圾坐标（如 z=14m）发给机械臂
        if not (args.min_depth <= z <= args.max_depth):
            patch = depth_m[max(0, v-2):v+3, max(0, u-2):u+3]
            valid = patch[(patch >= args.min_depth) & (patch <= args.max_depth)]
            if valid.size == 0:
                with shared.det_lock:
                    shared.latest_det = None
                continue
            z = float(np.median(valid))

        X = (u - source.cx) * z / source.fx
        Y = (v - source.cy) * z / source.fy
        Z = z
        angle = float(g["angle"])
        width_m = max(float(g["length"]) * z / source.fx, 0.005)  # 宽度钳制下限 5mm，防负值

        pkt = struct.pack('<5f', X, Y, Z, angle, width_m)
        sock.sendto(pkt, target)
        sent += 1

        with shared.det_lock:
            shared.latest_det = {
                "u": u, "v": v, "angle": angle,
                "width_px": float(g["length"]), "width_m": width_m, "z": z,
            }

        if sent % args.print_every == 1:
            print(f"[发送 #{sent}] cam=({X:+.3f},{Y:+.3f},{Z:.3f})m "
                  f"angle={angle:+.2f}rad width={width_m*1000:.0f}mm")

        if args.save_vis and (sent % args.vis_every == 1):
            out = save_vis(os.path.join(GRCNN_ROOT, "out"), sent, rgb,
                           shared.latest_det, cam_data)
            print(f"[可视化] {out}")

        if args.offline:   # 离线模式只发一帧便于联调
            print("[离线] 已发送 1 帧，退出")
            shared.running = False
            return


# ---------------------------------------------------------------- 显示主线程
def display_loop(shared, cam_data):
    import cv2
    print("[显示] 弹窗已启动，按 q 退出")
    while shared.running:
        with shared.frame_lock:
            frame = shared.latest_frame
        with shared.det_lock:
            det = shared.latest_det
        if frame is not None:
            rgb, _ = frame
            cv2.imshow("GRCNN live (q=quit)", draw_overlay_cv(rgb, det, cam_data))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[显示] 用户退出")
            shared.running = False
            break
        time.sleep(0.01)   # 显示刷新上限 ~100Hz，实际取决于取图速度
    cv2.destroyAllWindows()


# ---------------------------------------------------------------- 主流程
def run(args):
    device = torch.device("cuda:0" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"[模型] 加载 {args.model}  ->  {device}")
    model = load_model(args.model, device, fp16=args.fp16)
    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True   # 输入尺寸固定，加速卷积

    cam_data = CameraData(width=args.cam_width, height=args.cam_height,
                          output_size=args.input_size,
                          include_rgb=True, include_depth=True)

    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.target_ip, args.port)

    source = OfflineSource(args.offline_num) if args.offline else RealSenseSource(
        width=args.cam_width, height=args.cam_height, fps=args.cam_fps,
        use_filters=not args.no_filters)

    shared = Shared()
    t_cap = threading.Thread(target=capture_loop, args=(source, shared), daemon=True)
    t_inf = threading.Thread(target=inference_loop,
                             args=(args, model, cam_data, source, shared, sock, target, device),
                             daemon=True)
    t_cap.start()
    t_inf.start()
    print(f"[服务] 开始向 {target[0]}:{target[1]} 发送抓取包 (20B = x,y,z,angle,width)")

    try:
        if args.visualize:
            display_loop(shared, cam_data)   # 主线程跑显示（Qt 要求）
        else:
            while shared.running:
                time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[服务] 停止")
    finally:
        shared.running = False
        t_cap.join(timeout=2)
        t_inf.join(timeout=2)
        source.stop()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description="GRCNN 视觉抓取服务端 (UDP)")
    ap.add_argument('--model', default=os.path.join(
        GRCNN_ROOT, 'models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93'))
    ap.add_argument('--target-ip', default='127.0.0.1', help='机械臂控制端 IP（同机即本机）')
    ap.add_argument('--port', type=int, default=5005, help='与 VisionDetector 端口一致')
    ap.add_argument('--input-size', type=int, default=300, help='需与模型训练尺寸一致')
    ap.add_argument('--cam-width', type=int, default=640, help='相机分辨率宽（可改 640）')
    ap.add_argument('--cam-height', type=int, default=480, help='相机分辨率高（可改 360 更轻量，需 >= input-size）')
    ap.add_argument('--cam-fps', type=int, default=30, help='相机帧率')
    ap.add_argument('--cpu', action='store_true')
    ap.add_argument('--fp16', action='store_true', default=False,
                    help='CUDA 半精度推理（实测对本模型提速不明显且会降低宽度输出精度，默认关闭）')
    ap.add_argument('--min-depth', type=float, default=0.10)
    ap.add_argument('--max-depth', type=float, default=1.00)
    ap.add_argument('--visualize', action='store_true',
                    help='弹窗实时显示识别画面（需图形环境，按 q 退出）')
    ap.add_argument('--save-vis', action='store_true',
                    help='无界面时：保存检测画面到 vision/out/')
    ap.add_argument('--vis-every', type=int, default=30, help='--save-vis 时每多少帧保存一张')
    ap.add_argument('--print-every', type=int, default=15)
    ap.add_argument('--no-filters', action='store_true',
                    help='关闭 D435i 深度滤波链（推理频率更高，深度略糙）')
    ap.add_argument('--profile', action='store_true', help='打印分段计时（定位性能瓶颈）')
    ap.add_argument('--track-alpha', type=float, default=0.4,
                    help='帧间 EMA 平滑系数（越大越跟手，越小越稳）')
    ap.add_argument('--track-max-miss', type=int, default=8,
                    help='连续丢失多少帧后判定 no grasp')
    ap.add_argument('--offline', action='store_true', help='无相机，用 test_data 自检')
    ap.add_argument('--offline-num', type=int, default=2, choices=range(1, 7))
    run(ap.parse_args())
