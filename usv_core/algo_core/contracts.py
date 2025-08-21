from __future__ import annotations
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional, Any

CoordLL = Tuple[float, float]   # (lon, lat) degrees

@dataclass
class BoatState:
    vid: str
    lon: float
    lat: float
    sog: float        # speed over ground (knots)
    cog: float        # course over ground (deg)
    heading: float    # bow angle (deg)
    ts: float         # timestamp (s)

@dataclass
class Pickup:
    pid: int
    point: CoordLL
    num: int

@dataclass
class Dropoff:
    did: int
    point: CoordLL
    num: int

@dataclass
class ObstacleLL:
    oid: int
    point: CoordLL
    radius_m: float   # 米制半径（内部使用）

SegmentPoint = Dict[str, Any]   # {"coord":[lon,lat], "spd": float_knots, "acc": float}
Plan = Dict[str, List[Dict[str, List[SegmentPoint]]]]  # {vid: [{"shape":"LineString","points":[...]}]}
