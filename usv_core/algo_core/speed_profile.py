from __future__ import annotations
from typing import List, Tuple
import math

def profile_speed(points: List[Tuple[float,float]],
                  v_max_mps: float,
                  a_lat_max: float,
                  stop_slowdown_m: float) -> List[float]:
    n = len(points)
    if n == 0: return []
    if n == 1: return [0.0]

    # 弧长
    seg = [0.0]
    for i in range(1,n):
        dx = points[i][0]-points[i-1][0]
        dy = points[i][1]-points[i-1][1]
        seg.append(math.hypot(dx,dy))
    s = [0.0]*n
    for i in range(1,n):
        s[i] = s[i-1] + seg[i]

    # 曲率
    def curvature(i):
        if i==0 or i==n-1: return 0.0
        ax,ay=points[i-1]; bx,by=points[i]; cx,cy=points[i+1]
        ab=math.hypot(bx-ax, by-ay); bc=math.hypot(cx-bx, cy-by)
        if ab<1e-6 or bc<1e-6: return 0.0
        area=abs((bx-ax)*(cy-ay)-(by-ay)*(cx-ax))/2.0
        if area<1e-6: return 0.0
        R=(ab*bc*math.hypot(cx-ax, cy-ay))/(4.0*area+1e-9)
        return 0.0 if R<1e-6 else 1.0/R

    # 曲率限速（已有）
    v_curve = [min(v_max_mps, math.sqrt(a_lat_max / kap) if kap > 1e-9 else v_max_mps) for kap in
               [curvature(i) for i in range(n)]]

    # 前瞻减速：看未来 Lf 米内的最大曲率，提前限速
    Lf = 20.0  # 15~25m 自行调
    for i in range(n):
        s_i = s[i]
        j = i
        kap_max = 0.0
        while j < n and (s[j] - s_i) <= Lf:
            # 重用 curvature 计算
            ax, ay = points[j - 1] if j > 0 else points[j]
            bx, by = points[j]
            cx, cy = points[j + 1] if j + 1 < n else points[j]
            ab = math.hypot(bx - ax, by - ay)
            bc = math.hypot(cx - bx, cy - by)
            if ab > 1e-6 and bc > 1e-6:
                area = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) / 2.0
                if area > 1e-6:
                    R = (ab * bc * math.hypot(cx - ax, cy - ay)) / (4.0 * area + 1e-9)
                    kap = 0.0 if R < 1e-6 else 1.0 / R
                    kap_max = max(kap_max, kap)
            j += 1
        if kap_max > 1e-9:
            v_curve[i] = min(v_curve[i], math.sqrt(a_lat_max / kap_max))

    # 终点线性减速到 0
    for i in range(n):
        dist_to_end = s[-1] - s[i]
        if dist_to_end <= stop_slowdown_m:
            v_curve[i] = min(v_curve[i], v_max_mps * max(0.0, dist_to_end / stop_slowdown_m))
    v_curve[-1] = 0.0

    # 轻量“jerk”限制：相邻点速度变化不要太陡（数值滤波）
    for i in range(1, n):
        dv = v_curve[i] - v_curve[i - 1]
        max_dv = 0.6  # m/s 每采样（按点间距≈2~3m估计，可调）
        if dv > max_dv: v_curve[i] = v_curve[i - 1] + max_dv
        if dv < -max_dv: v_curve[i] = v_curve[i - 1] - max_dv

    return v_curve

