from __future__ import annotations
from typing import Dict, List, Tuple, Optional
import math, random

def _dist(a: Tuple[float,float], b: Tuple[float,float]) -> float:
    dx, dy = a[0]-b[0], a[1]-b[1]
    return math.hypot(dx, dy)

def choose_next_for_boat(vid: str,
                         boat_xy: Tuple[float,float],
                         boat_load: int,
                         boat_max: int,
                         pickups_xy: Dict[int, Tuple[Tuple[float,float], int]],
                         dropoffs_xy: Dict[int, Tuple[Tuple[float,float], int]]
                         ) -> Optional[Tuple[str,int,Tuple[float,float]]]:
    """
    兼容旧接口：单船的“下一目标”。当上游没有调用多船版本时备用。
    策略：仍然是“若有货→最近送货；否则→最近取货”。
    """
    # 送货优先
    if boat_load > 0:
        best = None
        for did, (xy, need) in dropoffs_xy.items():
            if need <= 0: continue
            d = _dist(boat_xy, xy)
            if (best is None) or (d < best[0]): best = (d, ('drop', did, xy))
        if best: return best[1]

    # 否则取货
    if boat_load < boat_max:
        best = None
        for pid, (xy, num) in pickups_xy.items():
            if num <= 0: continue
            d = _dist(boat_xy, xy)
            if (best is None) or (d < best[0]): best = (d, ('pick', pid, xy))
        if best: return best[1]

    return None

# ==== 新增：多船-多任务分配（轻量“拍卖”） ====
def auction_assign(boats_xy: Dict[str, Tuple[float,float]],
                   boats_load: Dict[str, int],
                   boats_max: Dict[str, int],
                   pickups_xy: Dict[int, Tuple[Tuple[float,float], int]],
                   dropoffs_xy: Dict[int, Tuple[Tuple[float,float], int]]
                   ) -> Dict[str, Tuple[str,int,Tuple[float,float]]]:
    """
    返回 {vid: (kind, id, target_xy)}，一轮只决定每船的“下一跳”。
    规则：
      1) 船上有货 → 参与“送货拍卖”，每艘船竞价自己最近的有效 drop。
      2) 船上没货 → 参与“取货拍卖”，竞价最近的有效 pickup（考虑容量）。
      3) 每个任务点本轮只允许一个赢家（避免撞车）；未分配到的船本轮返航或空闲。
    """
    # 送货分组 / 取货分组
    with_load = [vid for vid,L in boats_load.items() if L>0]
    no_load   = [vid for vid,L in boats_load.items() if L<=0]

    assign: Dict[str, Tuple[str,int,Tuple[float,float]]] = {}

    # --- 送货拍卖 ---
    bids = []  # (cost, vid, ('drop', did, xy))
    for vid in with_load:
        cur = boats_xy[vid]
        best = None
        for did, (xy, need) in dropoffs_xy.items():
            if need <= 0: continue
            d = _dist(cur, xy)
            if (best is None) or (d < best[0]): best = (d, ('drop', did, xy))
        if best: bids.append((best[0], vid, best[1]))
    # 中标：每个 drop 只给一条船
    taken_drop = set()
    for cost, vid, item in sorted(bids):
        _, did, xy = item
        if did in taken_drop: continue
        taken_drop.add(did)
        assign[vid] = item

    # --- 取货拍卖 ---
    bids = []  # (cost, vid, ('pick', pid, xy))
    for vid in no_load:
        cur = boats_xy[vid]; maxL = boats_max[vid]
        # 只竞价仍有货的点
        best = None
        for pid, (xy, num) in pickups_xy.items():
            if num <= 0: continue
            # 容量可行（至少能拿 1 单位）
            d = _dist(cur, xy)
            # 轻量启发：距离 + 拥堵惩罚（剩余货越少，价格越高，避免多船抢同一少量点）
            price = d * (1.0 + 0.3/(num+1e-6))
            if (best is None) or (price < best[0]): best = (price, ('pick', pid, xy))
        if best: bids.append((best[0], vid, best[1]))

    taken_pick = set()
    for cost, vid, item in sorted(bids):
        _, pid, xy = item
        if pid in taken_pick: continue
        taken_pick.add(pid)
        assign[vid] = item

    return assign
