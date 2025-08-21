from __future__ import annotations
from typing import List, Tuple
import math

def chaikin(points: List[Tuple[float,float]], iters: int = 1, ratio: float = 0.25) -> List[Tuple[float,float]]:
    if len(points) < 3 or iters <= 0: return points
    pts = points[:]
    for _ in range(iters):
        nq = [pts[0]]
        for i in range(len(pts)-1):
            p = pts[i]; q = pts[i+1]
            L = ((1-ratio)*p[0] + ratio*q[0], (1-ratio)*p[1] + ratio*q[1])
            R = (ratio*p[0] + (1-ratio)*q[0], ratio*p[1] + (1-ratio)*q[1])
            nq.extend([L,R])
        nq.append(pts[-1])
        pts = nq
    return pts

def moving_avg(points: List[Tuple[float,float]], win: int = 5) -> List[Tuple[float,float]]:
    if len(points) <= win or win < 2: return points
    out = []
    half = win//2
    for i in range(len(points)):
        xs=ys=0.0; cnt=0
        for j in range(max(0,i-half), min(len(points), i+half+1)):
            xs += points[j][0]; ys += points[j][1]; cnt += 1
        out.append((xs/cnt, ys/cnt))
    return out

def smooth_path(points: List[Tuple[float,float]], max_kappa: float) -> List[Tuple[float,float]]:
    if len(points) <= 2: return points
    pts = chaikin(points, iters=1, ratio=0.25)
    pts = moving_avg(pts, win=5)

    def curvature(a,b,c):
        ax,ay=a; bx,by=b; cx,cy=c
        ab = math.hypot(bx-ax, by-ay)
        bc = math.hypot(cx-bx, cy-by)
        if ab<1e-6 or bc<1e-6: return 0.0
        area = abs((bx-ax)*(cy-ay) - (by-ay)*(cx-ax)) / 2.0
        if area < 1e-6: return 0.0
        R = (ab*bc*math.hypot(cx-ax, cy-ay)) / (4.0*area + 1e-9)
        return 0.0 if R < 1e-6 else 1.0/R

    i = 1
    out = [pts[0]]
    while i < len(pts)-1:
        a=out[-1]; b=pts[i]; c=pts[i+1]
        kap = curvature(a,b,c)
        if kap > max_kappa:
            mid = ((a[0]+c[0])/2.0, (a[1]+c[1])/2.0)
            out.append(mid)  # 插点降曲率
        else:
            out.append(b); i += 1
    out.append(pts[-1])
    return out
