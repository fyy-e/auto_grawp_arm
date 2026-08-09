#!/usr/bin/env python3
"""GRCNN 推理全链路分段基准测试：定位 Orin Nano 上的耗时分布"""
import os, sys, time
import numpy as np
import torch

GRCNN_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, GRCNN_ROOT)

from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import detect_grasps

device = torch.device("cuda:0")
torch.backends.cudnn.benchmark = True

model_path = os.path.join(GRCNN_ROOT, "models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93")
model = torch.load(model_path, map_location=device, weights_only=False)
model.eval()

x = torch.rand(1, 4, 300, 300).to(device)

def bench(fn, n=100, warmup=20):
    for _ in range(warmup):
        fn()
    torch.cuda.synchronize()
    t0 = time.time()
    for _ in range(n):
        fn()
    torch.cuda.synchronize()
    return (time.time() - t0) / n * 1000

# 1. 纯前向（model.forward）
def fwd():
    with torch.no_grad():
        model(x)
print(f"纯前向 forward:        {bench(fwd):6.1f} ms")

# 2. model.predict（含 sigmoid 等）
def pred():
    with torch.no_grad():
        model.predict(x)
print(f"model.predict:         {bench(pred):6.1f} ms")

with torch.no_grad():
    p = model.predict(x)

# 3. 后处理
def pp():
    post_process_output(p['pos'], p['cos'], p['sin'], p['width'])
print(f"post_process_output:   {bench(pp):6.1f} ms")

q, a, w = post_process_output(p['pos'], p['cos'], p['sin'], p['width'])

# 4. detect_grasps
def dg():
    detect_grasps(q, a, width_img=w, no_grasps=1)
print(f"detect_grasps:         {bench(dg):6.1f} ms")

# 5. 数据预处理（CameraData）
cam_data = CameraData(width=640, height=480, output_size=300, include_rgb=True, include_depth=True)
rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
depth = np.random.rand(480, 640, 1).astype(np.float32)
def prep():
    cam_data.get_data(rgb=rgb, depth=depth)
print(f"CameraData.get_data:   {bench(prep):6.1f} ms")

# 6. CPU 参考
model_cpu = torch.load(model_path, map_location="cpu", weights_only=False).eval()
xc = torch.rand(1, 4, 300, 300)
def fwd_cpu():
    with torch.no_grad():
        model_cpu(xc)
print(f"纯前向 (CPU 参考):     {bench(fwd_cpu, n=20, warmup=5):6.1f} ms")
