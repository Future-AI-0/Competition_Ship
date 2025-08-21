from __future__ import annotations
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import argparse, time, yaml

from .contracts import *
from .world import WorldModel, GeoProjector, knots_to_mps, mps_to_knots
from .allocator import choose_next_for_boat
from .planner import astar
from .smoother import smooth_path
from .speed_profile import profile_speed

def densify(points, step=3.0):
    """按固定步长插值加密，用于减少下游跟踪抖动。"""
    if len(points) <= 1: return points
    out = [points[0]]
    for i in range(1, len(points)):
        ax,ay = out[-1]
        bx,by = points[i]
        dx,dy = bx-ax, by-ay
        dist = (dx*dx+dy*dy)**0.5
        if dist <= 1e-6:
            continue
        n = int(dist // step)
        for k in range(1, n+1):
            t = (k*step)/dist
            out.append((ax + t*dx, ay + t*dy))
        if out[-1] != (bx,by):
            out.append((bx,by))
    return out

@dataclass
class Cfg:
    lon0: float
    lat0: float
    arrive_radius_m: float
    safety_margin_m: float
    grid_res_m: float
    diag_cost: float
    replan_min_interval_s: float
    v_max_knots: float
    a_lat_max: float
    stop_slowdown_m: float

class USVCore:
    """你的算法内核：只管 on_* 输入 → tick() 产出 Plan"""
    def __init__(self, cfg: Cfg):
        self.cfg = cfg
        self.world = WorldModel(
            projector=GeoProjector(cfg.lon0, cfg.lat0),
            arrive_radius_m=cfg.arrive_radius_m,
            safety_margin_m=cfg.safety_margin_m
        )
        self._last_end_ll: Dict[str, Tuple[float,float]] = {}
        # === bandit 参数候选（可以继续扩展）===
        self._param_candidates = [
            {"vmax": 5.0, "safety": 3.0, "Lf": 15.0},
            {"vmax": 5.5, "safety": 4.0, "Lf": 20.0},
            {"vmax": 6.0, "safety": 5.0, "Lf": 25.0},
        ]
        self._param_score = [0.0]*len(self._param_candidates)
        self._param_count = [1e-6]*len(self._param_candidates)
        self._param_idx = 0
        self._freeze_until = 0.0
        self._eps = 0.2  # 探索率
        self.last_emit_ts = 0.0
        self.dirty = True

    # ===== 输入接口 =====
    def on_init(self,
                boats_init: List[Tuple[str, CoordLL, int]],
                pickups: List[Pickup],
                dropoffs: List[Dropoff],
                obstacles: List[ObstacleLL]) -> None:
        pickups_ll = [(p.pid, p.point, p.num) for p in pickups]
        dropoffs_ll = [(d.did, d.point, d.num) for d in dropoffs]
        obstacles_ll = [(o.oid, o.point, o.radius_m) for o in obstacles]
        self.world.reset(boats_init, pickups_ll, dropoffs_ll, obstacles_ll)
        self.dirty = True

    def on_state(self, states: List[BoatState]) -> None:
        self.world.update_states([(s.vid, s.lon, s.lat) for s in states])

    def on_obstacles(self, obstacles: List[ObstacleLL]) -> None:
        self.world.obstacles.clear()
        for o in obstacles:
            x,y = self.world.to_xy(o.point)
            self.world.obstacles.append((x,y, o.radius_m + self.cfg.safety_margin_m))
        self.dirty = True

    def on_new_tasks(self, pickups: List[Pickup], dropoffs: List[Dropoff]) -> None:
        for p in pickups:
            self.world.pickups[p.pid] = (self.world.to_xy(p.point), p.num)
        for d in dropoffs:
            self.world.dropoffs[d.did] = (self.world.to_xy(d.point), d.num)
        self.dirty = True

    # ===== 主循环 =====
    def tick(self, now: float) -> Optional[Plan]:
        if not self.dirty and (now - self.last_emit_ts < self.cfg.replan_min_interval_s):
            return None

        # bandit 选择参数（每 ~5s 可切换一次）
        if now >= self._freeze_until:
            import random
            if random.random() < self._eps:
                self._param_idx = random.randrange(len(self._param_candidates))
            else:
                # 选平均得分最高的
                avg = [s/c for s,c in zip(self._param_score, self._param_count)]
                self._param_idx = max(range(len(avg)), key=lambda i: avg[i])
            cand = self._param_candidates[self._param_idx]
            # 覆盖运行时参数
            self.cfg.v_max_knots = cand["vmax"]
            self.cfg.safety_margin_m = cand["safety"]
            # 速度前瞻 Lf 传给 speed_profile（你也可以放到 cfg）
            # 这里简化为：把 stop_slowdown_m 绑定到 Lf（也可单独传）
            self.cfg.stop_slowdown_m = cand["Lf"]
            # 冻结 5s 避免频繁切换
            self._freeze_until = now + 5.0

        plan: Plan = {}
        for vid, boat in self.world.boats.items():
            cur_xy = self.world.boat_xy(vid)

            segments = []
            seg_targets = []

            # 选择下一步
            choice = choose_next_for_boat(vid, cur_xy, boat.load, boat.max_load,
                                          self.world.pickups, self.world.dropoffs)
            if choice is None:
                # 无任务 → 返航
                seg_targets.append(self.world.to_xy(boat.home_ll))
            else:
                kind, the_id, tgt_xy = choice
                seg_targets.append(tgt_xy)

                # 如果是取货，自动补一个最近的送货点
                if kind == 'pick':
                    # 选一个仍有需求的最近 drop
                    best = None
                    for did, (dxy, need) in self.world.dropoffs.items():
                        if need <= 0: continue
                        d = ((dxy[0] - tgt_xy[0]) ** 2 + (dxy[1] - tgt_xy[1]) ** 2) ** 0.5
                        if (best is None) or (d < best[0]):
                            best = (d, dxy)
                    if best:
                        seg_targets.append(best[1])

                # 可选：回家
                seg_targets.append(self.world.to_xy(boat.home_ll))

            # 逐段规划并拼接
            path_xy = [cur_xy]
            for tgt in seg_targets:
                # ---- 新增：按 bandit 的 safety 现算障碍半径 ----
                base_obs = [(ox, oy, r - self.world.safety_margin_m) for (ox, oy, r) in self.world.obstacles]
                obs_now = [(ox, oy, base_r + self.cfg.safety_margin_m) for (ox, oy, base_r) in base_obs]

                raw = astar(path_xy[-1], tgt, obs_now,
                            grid_res=self.cfg.grid_res_m, diag_cost=self.cfg.diag_cost)
                if len(raw) > 1:
                    path_xy.extend(raw[1:])

            # 平滑 + 速度
            v_max_mps = knots_to_mps(self.cfg.v_max_knots)
            max_kappa = self.cfg.a_lat_max / max(v_max_mps * v_max_mps, 0.1)
            nice = smooth_path(path_xy, max_kappa=max_kappa)

            # 可选：加密采样，保证点间距不超过 3m（下游跟踪更稳定）
            nice = densify(nice, step=3.0)

            v_list_mps = profile_speed(nice, v_max_mps=v_max_mps,
                                       a_lat_max=self.cfg.a_lat_max,
                                       stop_slowdown_m=self.cfg.stop_slowdown_m)

            # 打包
            pts = []
            for (x, y), v in zip(nice, v_list_mps):
                lon, lat = self.world.to_ll((x, y))
                pts.append({"coord": [lon, lat], "spd": mps_to_knots(v), "acc": -1})

            if len(pts) < 2:
                if len(pts) == 1:
                    pts.append(pts[0])
                else:
                    continue

            # 防抖：终点变化小就不下发
            end_ll = tuple(pts[-1]["coord"])
            last_end = self._last_end_ll.get(vid)
            def ll_dist(a,b):
                # 近似：经纬差转米（仅用于阈值判断）
                import math
                R = 6378137.0
                lon0, lat0 = math.radians(a[0]), math.radians(a[1])
                lon1, lat1 = math.radians(b[0]), math.radians(b[1])
                clat = math.cos((lat0+lat1)/2.0)
                dx = (lon1-lon0)*clat*R
                dy = (lat1-lat0)*R
                return (dx*dx+dy*dy)**0.5
            if last_end is not None and ll_dist(end_ll, last_end) < 2.0:
                # 终点位移 < 2m 认为没必要更新本船计划
                continue
            self._last_end_ll[vid] = end_ll

            plan[vid] = [{"shape": "LineString", "points": pts}]

        if plan:
            # 估算“即时回报”：负路径长度 - 风险 - 末端残距
            def path_length_ll(pts):
                import math
                if len(pts) < 2: return 0.0
                R = 6378137.0
                s = 0.0
                for i in range(1, len(pts)):
                    lon0, lat0 = map(math.radians, pts[i - 1]["coord"])
                    lon1, lat1 = map(math.radians, pts[i]["coord"])
                    clat = math.cos((lat0 + lat1) / 2.0)
                    dx = (lon1 - lon0) * clat * R;
                    dy = (lat1 - lat0) * R
                    s += (dx * dx + dy * dy) ** 0.5
                return s

            total_len = 0.0
            for vlist in plan.values():
                pts = vlist[0]["points"]
                total_len += path_length_ll(pts)
            # 风险：最近障碍平均 1/clearance（用 A* 的最近距离函数可复用；这里简单用 0）
            risk = 0.0
            reward = - (0.001 * total_len + 0.5 * risk)  # 越大越好
            i = self._param_idx
            self._param_score[i] += reward
            self._param_count[i] += 1.0
            # 正常返回
            self.last_emit_ts = now
            self.dirty = False
            return plan

        return None

# ===== DEMO / CLI =====
def _load_cfg(path: str) -> Cfg:
    with open(path, "r", encoding="utf-8") as f:
        y = yaml.safe_load(f)
    return Cfg(
        lon0 = y["projection"]["lon0"],
        lat0 = y["projection"]["lat0"],
        arrive_radius_m = y["world"]["arrive_radius_m"],
        safety_margin_m = y["world"]["safety_margin_m"],
        grid_res_m = y["planner"]["grid_res_m"],
        diag_cost = y["planner"]["diag_cost"],
        replan_min_interval_s = y["planner"]["replan_min_interval_s"],
        v_max_knots = y["speed"]["v_max_knots"],
        a_lat_max = y["speed"]["a_lat_max"],
        stop_slowdown_m = y["speed"]["stop_slowdown_m"],
    )

def _demo_once():
    cfg = _load_cfg("usv_core/config.yaml")  # 注意路径
    core = USVCore(cfg)
    boats = [("10AF0201", (cfg.lon0, cfg.lat0), 2)]
    pickups = [Pickup(1, (cfg.lon0+0.01, cfg.lat0+0.005), 1)]
    dropoffs= [Dropoff(1, (cfg.lon0+0.02, cfg.lat0+0.007), 1)]
    obs = [ObstacleLL(1, (cfg.lon0+0.015, cfg.lat0+0.006), 80.0)]

    core.on_init(boats, pickups, dropoffs, obs)
    core.on_state([BoatState("10AF0201", cfg.lon0, cfg.lat0, 0, 0, 0, time.time())])
    plan = core.tick(time.time())
    print("\n=== DEMO plan ===")
    print(plan)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    args = ap.parse_args()
    if args.demo:
        _demo_once()
