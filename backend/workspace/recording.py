from bisect import bisect_right
from copy import deepcopy
from datetime import datetime, timezone
from uuid import uuid4


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class RunRecording:
    """History is read-only: seeking never restores or mutates the simulation."""

    interval_s = 0.5

    def __init__(self, inputs=None):
        self.id = str(uuid4())
        self.created_at = utc_now()
        self.inputs = deepcopy(inputs or {})
        self.frames = []
        self.times = []
        self.next_time = 0.0
        self.journal = []
        self.controls = []
        self.started = False
        self.enabled = True

    def capture(self, engine, force=False):
        if not self.enabled:
            return
        if not force and engine.sim_time + 1e-8 < self.next_time:
            return
        frame = dict(
            time=engine.sim_time,
            status=engine.status,
            robots=[
                {k: v for k, v in r.snapshot().items() if k not in ("parameters",)}
                for r in engine.robots.values()
            ],
            mapf=engine.traffic.snapshot() if engine.traffic else None,
            metrics_summary=dict(
                collision_count=len(engine.metrics.collisions),
                orders_delivered=sum(
                    o.status == "delivered" for o in engine.wms.orders
                ),
                fifo_violation_count=len(engine.metrics.fifo_violations),
            ),
            orders=[o.to_dict() for o in engine.wms.orders],
            last_collisions=engine.last_collisions,
        )
        frame = deepcopy(frame)
        if self.times and self.times[-1] == engine.sim_time:
            self.frames[-1] = frame
        else:
            self.times.append(engine.sim_time)
            self.frames.append(frame)
        self.next_time = engine.sim_time + self.interval_s

    def at(self, sim_time):
        if not self.frames:
            raise ValueError("History is empty")
        index = max(0, bisect_right(self.times, sim_time) - 1)
        return dict(
            run_id=self.id,
            frame=self.frames[index],
            first=self.times[0],
            last=self.times[-1],
            interval_s=self.interval_s,
        )

    def info(self):
        return dict(
            run_id=self.id,
            started=self.started,
            frames=len(self.frames),
            first=self.times[0] if self.times else 0,
            last=self.times[-1] if self.times else 0,
            interval_s=self.interval_s,
            journal_count=len(self.journal),
        )
