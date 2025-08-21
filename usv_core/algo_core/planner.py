from __future__ import annotations
from typing import List, Tuple, Dict, Optional
import math, heapq

def _line_circle_intersects(a,b,c,r)->bool:
    ax,ay=a; bx,by=b; cx,cy=c
    vx, vy = bx-ax, by-ay
    wx, wy = cx-ax, cy-ay
    c1 = vx*wx + vy*wy
    if c1 <= 0:
        d2 = (cx-ax)**2 + (cy-ay)**2
    else:
        c2 = vx*vx + vy*vy
        if c1 >= c2:
            d2 = (cx-bx)**2 + (cy-by)**2
        else:
            t = c1/c2
            px,py = ax + t*vx, ay + t*vy
            d2 = (cx-px)**2 + (cy-py)**2
    return d2 <= r*r

def _visible(a,b,obstacles)->bool:
    for (ox,oy,orad) in obstacles:
        if _line_circle_intersects(a,b,(ox,oy),orad):
            return False
    return True

# 栅格 A*（8邻接）+ 可见性剪枝
def astar(start_xy: Tuple[float,float],
          goal_xy: Tuple[float,float],
          obstacles: List[Tuple[float,float,float]],
          grid_res: float,
          diag_cost: float = 1.41421356237,
          bound_margin: float = 30.0) -> List[Tuple[float,float]]:
    if math.hypot(start_xy[0]-goal_xy[0], start_xy[1]-goal_xy[1]) < grid_res:
        return [start_xy, goal_xy]

    # 边界盒
    minx = min(start_xy[0], goal_xy[0]) - bound_margin
    miny = min(start_xy[1], goal_xy[1]) - bound_margin
    maxx = max(start_xy[0], goal_xy[0]) + bound_margin
    maxy = max(start_xy[1], goal_xy[1]) + bound_margin
    for (ox,oy,orad) in obstacles:
        minx = min(minx, ox-orad-bound_margin)
        miny = min(miny, oy-orad-bound_margin)
        maxx = max(maxx, ox+orad+bound_margin)
        maxy = max(maxy, oy+orad+bound_margin)

    def to_ij(xy):
        x,y = xy
        i = int((x - minx)/grid_res)
        j = int((y - miny)/grid_res)
        return i,j

    def to_xy(i,j):
        x = minx + (i+0.5)*grid_res
        y = miny + (j+0.5)*grid_res
        return (x,y)

    ni, nj = to_ij((maxx, maxy))
    ni = max(ni, 2); nj = max(nj, 2)

    # 占据栅格
    occ = [[False]*(nj+1) for _ in range(ni+1)]
    for i in range(ni+1):
        for j in range(nj+1):
            x,y = to_xy(i,j)
            for (ox,oy,orad) in obstacles:
                if (x-ox)**2 + (y-oy)**2 <= orad**2:
                    occ[i][j] = True
                    break

    si, sj = to_ij(start_xy); gi, gj = to_ij(goal_xy)
    si = max(0, min(si, ni)); sj = max(0, min(sj, nj))
    gi = max(0, min(gi, ni)); gj = max(0, min(gj, nj))

    # A*
    def h(i,j):
        dx = abs(i-gi); dy = abs(j-gj)
        return (dx+dy) + (diag_cost-2)*min(dx,dy)

    nbrs = [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),
            (-1,-1,diag_cost),(1,1,diag_cost),(-1,1,diag_cost),(1,-1,diag_cost)]
    openq = []
    heapq.heappush(openq, (h(si,sj), 0.0, (si,sj), None))
    came: Dict[Tuple[int,int], Optional[Tuple[int,int]]] = {}
    gval: Dict[Tuple[int,int], float] = {(si,sj): 0.0}
    closed = set()

    while openq:
        f,g,(i,j),parent = heapq.heappop(openq)
        if (i,j) in closed: continue
        came[(i,j)] = parent
        if (i,j) == (gi,gj): break
        closed.add((i,j))
        for di,dj,cost in nbrs:
            i2, j2 = i+di, j+dj
            if not (0 <= i2 <= ni and 0 <= j2 <= nj): continue
            if occ[i2][j2]: continue
            # 距离代价
            gg = g + cost
            # 风险代价：距最近障碍越近，越贵（λ 可在 0.2~1.0 之间调）
            lam = 0.5

            def nearest_clearance(i2, j2):
                x, y = to_xy(i2, j2)
                best = 1e9
                for (ox, oy, orad) in obstacles:
                    d = math.hypot(x - ox, y - oy) - orad
                    if d < best: best = d
                return max(0.0, best)

            clr = nearest_clearance(i2, j2) + 1e-6
            gg += lam * (1.0 / clr)  # 离障近→代价大
            if gg < gval.get((i2,j2), 1e18):
                gval[(i2,j2)] = gg
                ff = gg + h(i2,j2)
                heapq.heappush(openq, (ff, gg, (i2,j2), (i,j)))

    if (gi,gj) not in came:
        # 失败时退化为直线（上层可二次处理）
        return [start_xy, goal_xy]

    # 回溯
    path = []
    cur = (gi,gj)
    while cur:
        path.append(to_xy(*cur))
        cur = came[cur]
    path.reverse()

    # 可见性剪枝（一次扫描）
    if len(path) <= 2: return path
    kept = [path[0]]
    i = 0
    while True:
        best = len(path)-1
        for j in range(len(path)-1, i, -1):
            if _visible(path[i], path[j], obstacles):
                best = j
                break
        kept.append(path[best])
        if best == len(path)-1: break
        i = best
    return kept
