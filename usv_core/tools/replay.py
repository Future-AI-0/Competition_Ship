import time, os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from usv_core.algo_core.core import USVCore, _load_cfg
from usv_core.algo_core.contracts import Pickup, Dropoff, ObstacleLL, BoatState

def main():
    cfg = _load_cfg(os.path.join(os.path.dirname(__file__), "..", "config.yaml"))
    core = USVCore(cfg)

    lon0, lat0 = cfg.lon0, cfg.lat0
    boats = [("10AF0201", (lon0, lat0), 2), ("10AF0202", (lon0+0.002, lat0+0.001), 1)]
    pickups = [Pickup(1, (lon0+0.010, lat0+0.006), 2)]
    dropoffs= [Dropoff(1,(lon0+0.020, lat0+0.008), 2)]
    obs = [ObstacleLL(1, (lon0+0.013, lat0+0.006), 60.0)]

    core.on_init(boats, pickups, dropoffs, obs)
    core.on_state([
        BoatState("10AF0201", lon0, lat0, 0, 0, 0, time.time()),
        BoatState("10AF0202", lon0+0.002, lat0+0.001, 0, 0, 0, time.time()),
    ])

    plan = core.tick(time.time())
    print("PLAN:", plan)

if __name__ == "__main__":
    main()
