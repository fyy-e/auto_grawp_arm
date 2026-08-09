# -*- coding: utf-8 -*-
"""
GRCNN 帧间抓取跟踪器
=====================
解决"逐帧独立 argmax 导致抓取框在对称峰值间横跳"的问题。

问题来源（实测诊断，见 docs/GRCNN改造说明.md）：
  对称/多可抓位物体（如胶带卷左右两侧）在质量图上会产生多个
  高度几乎相等的峰（q 差 <0.02），detect_grasps(no_grasps=1)
  每帧取全局最大值，帧间就在这些峰之间跳变，表现为绿框乱飞。

策略：
  1. 取 top-k 候选（而不是只取第 1 名）；
  2. 已有锁定时，优先选"离上一帧位置近且质量合格"的候选
     ——框被吸在原目标上，不再横跳；
  3. 对中心/角度/宽度做 EMA 平滑（角度按 pi 周期处理跳变）；
  4. 连续 miss 超过 max_miss 帧才判定目标丢失（no grasp），
     避免单帧质量波动造成显示闪烁。

坐标约定：全部在"网络输出图"坐标系（300x300），
调用方负责加回 cam_data.top_left 还原到原图。
"""

import math

import numpy as np


class GraspTracker:
    def __init__(self,
                 max_jump_px=60,      # 跟踪吸附半径：候选距上一帧中心超过该值视为新目标
                 q_acquire=0.50,      # 新目标锁定所需最低质量分
                 q_keep=0.30,         # 跟踪中候选的最低质量分
                 alpha=0.4,           # EMA 平滑系数（越大越跟手，越小越稳）
                 max_miss=8):         # 连续丢失多少帧后判定 no grasp
        self.max_jump_px = max_jump_px
        self.q_acquire = q_acquire
        self.q_keep = q_keep
        self.alpha = alpha
        self.max_miss = max_miss
        self._smooth = None          # 平滑状态 dict 或 None
        self._miss = 0

    @staticmethod
    def _angle_ema(prev, new, alpha):
        """抓取角以 pi 为周期，直接 EMA 会在 ±pi/2 边界跳变，
        先把差值折回 (-pi/2, pi/2] 再平滑"""
        delta = (new - prev + math.pi / 2) % math.pi - math.pi / 2
        out = prev + alpha * delta
        # 归一化回 (-pi/2, pi/2]
        return (out + math.pi / 2) % math.pi - math.pi / 2

    def update(self, grasps, q_img):
        """
        grasps: detect_grasps(no_grasps=k) 返回的候选列表（质量降序）
        q_img : 质量图（网络输出坐标）
        返回: 平滑后的 dict {row, col, angle, length, q} 或 None（目标丢失）
        """
        cands = []
        for g in grasps:
            row, col = g.center
            cands.append({
                "row": float(row), "col": float(col),
                "angle": float(g.angle), "length": float(g.length),
                "q": float(q_img[row, col]),
            })

        chosen = None
        if self._smooth is not None:
            # 已锁定：在吸附半径内找质量最高的候选
            near = [c for c in cands
                    if c["q"] >= self.q_keep
                    and math.hypot(c["row"] - self._smooth["row"],
                                   c["col"] - self._smooth["col"]) <= self.max_jump_px]
            if near:
                chosen = max(near, key=lambda c: c["q"])
                self._miss = 0
            else:
                self._miss += 1
                if self._miss > self.max_miss:
                    self._smooth = None      # 目标确实丢了
                    self._miss = 0
                    return None
                return dict(self._smooth)    # 短暂丢失：保持上次结果，防闪烁
        else:
            # 未锁定：只接受高质量的全局最优，防噪声误锁
            good = [c for c in cands if c["q"] >= self.q_acquire]
            if good:
                chosen = good[0]
                self._miss = 0

        if chosen is None:
            return None

        if self._smooth is None:
            self._smooth = dict(chosen)      # 新锁定，直接初始化
        else:
            a = self.alpha
            s = self._smooth
            s["row"] = (1 - a) * s["row"] + a * chosen["row"]
            s["col"] = (1 - a) * s["col"] + a * chosen["col"]
            s["angle"] = self._angle_ema(s["angle"], chosen["angle"], a)
            s["length"] = (1 - a) * s["length"] + a * chosen["length"]
            s["q"] = chosen["q"]
        return dict(self._smooth)
