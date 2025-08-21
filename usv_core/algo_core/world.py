from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Tuple, List
import math

R_EARTH = 6378137.0

def mps_to_knots(v: float) -> float:
    return v * 1.94384

def knots_to_mps(v: float) -> float:
    return v / 1.94384

class GeoProjector:
    """简易本地 ENU 投影（近似）"""
    def __init__(self, lon0: float, lat0: float):
        self.lon0 = math.radians(lon0)
        self.lat0 = math.radians(lat0)
        self.clat = math.cos(self.lat0)

    def ll2xy(self, lon: float, lat: float) -> Tuple[float, float]:
        lon, lat = math.radians(lon), math.radians(lat)
        x = (lon - self.lon0) * self.clat * R_EARTH
        y = (lat - self.lat0) * R_EARTH
        return x, y

    def xy2ll(self, x: float, y: float) -> Tuple[float, float]:
        lon = x/(self.clat*R_EARTH) + self.lon0
        lat = y/R_EARTH + self.lat0
        return math.degrees(lon), math.degrees(lat)

@dataclass
class BoatCtx:
    vid: str
    home_ll: Tuple[float, float]
    max_load: int
    load: int = 0
    target_seq: List[Tuple[str, int, Tuple[float,float]]] = field(default_factory=list)  # (kind, id, ll)
    last_plan_xy: List[Tuple[float,float]] = field(default_factory=list)

@dataclass
class WorldModel:
    projector: GeoProjector
    arrive_radius_m: float
    safety_margin_m: float

    boats: Dict[str, BoatCtx] = field(default_factory=dict)
    states_xy: Dict[str, Tuple[float,float]] = field(default_factory=dict)
    pickups: Dict[int, Tuple[Tuple[float,float], int]] = field(default_factory=dict)    # pid -> ((x,y), num)
    dropoffs: Dict[int, Tuple[Tuple[float,float], int]] = field(default_factory=dict)   # did -> ((x,y), need)
    obstacles: List[Tuple[float,float,float]] = field(default_factory=list)             # (x,y,r_inflated)

    def reset(self,
              boats_init: List[Tuple[str, Tuple[float,float], int]],
              pickups_ll: List[Tuple[int, Tuple[float,float], int]],
              dropoffs_ll: List[Tuple[int, Tuple[float,float], int]],
              obstacles_ll: List[Tuple[int, Tuple[float,float], float]]):
        self.boats.clear(); self.states_xy.clear()
        self.pickups.clear(); self.dropoffs.clear(); self.obstacles.clear()

        for vid, ll, maxL in boats_init:
            self.boats[vid] = BoatCtx(vid, ll, maxL)

        for pid, ll, num in pickups_ll:
            self.pickups[pid] = (self.projector.ll2xy(*ll), num)

        for did, ll, num in dropoffs_ll:
            self.dropoffs[did] = (self.projector.ll2xy(*ll), num)

        for oid, ll, r in obstacles_ll:
            x,y = self.projector.ll2xy(*ll)
            self.obstacles.append((x, y, r + self.safety_margin_m))

    def update_states(self, states_ll: List[Tuple[str, float, float]]):
        for vid, lon, lat in states_ll:
            self.states_xy[vid] = self.projector.ll2xy(lon, lat)

    def boat_xy(self, vid: str) -> Tuple[float,float]:
        return self.states_xy.get(vid, self.projector.ll2xy(*self.boats[vid].home_ll))

    def obstacles_xy(self) -> List[Tuple[float,float,float]]:
        return list(self.obstacles)

    def to_ll(self, xy: Tuple[float,float]) -> Tuple[float,float]:
        return self.projector.xy2ll(*xy)

    def to_xy(self, ll: Tuple[float,float]) -> Tuple[float,float]:
        return self.projector.ll2xy(*ll)
