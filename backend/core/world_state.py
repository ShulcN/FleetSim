from dataclasses import dataclass, field


@dataclass
class WorldConfig:
    width_m: float = 20.0
    height_m: float = 12.0


@dataclass
class WorldSnapshot:
    time: float
    width_m: float
    height_m: float
    status: str
    collision_mode: str
    max_sim_time: float
    robots: list[dict] = field(default_factory=list)
    map: dict | None = None
    graph: dict | None = None
    wms: dict | None = None
    metrics_summary: dict | None = None
    last_collisions: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "time": self.time,
            "status": self.status,
            "collision_mode": self.collision_mode,
            "max_sim_time": self.max_sim_time,
            "world": {
                "width_m": self.width_m,
                "height_m": self.height_m,
            },
            "map": self.map,
            "graph": self.graph,
            "robots": self.robots,
            "wms": self.wms,
            "metrics_summary": self.metrics_summary,
            "last_collisions": self.last_collisions,
        }
