"""Versioned study objective; normalizers are frozen for an experiment series."""

from math import isfinite
from statistics import mean, pstdev

KEYS = (
    "mean_delivery_s",
    "distance_m",
    "conflict_episodes",
    "delivery_imbalance",
    "energy",
)
DEFAULTS = dict(
    weights=dict(zip(KEYS, [1.0, 1.0, 1.0, 1.0, 1.0])),
    scales=dict(zip(KEYS, [600.0, 1000.0, 10.0, 1.0, 1000.0])),
    collision_penalty=1000.0,
    incomplete_weight=10.0,
)


def measure(engine):
    traffic = engine.traffic
    orders = engine.wms.orders
    completed = [o for o in orders if o.delivered_at is not None]
    distances = engine.metrics.robot_distance_m
    agents = traffic.agents
    energy = sum(
        float(r.state.parameters.get("energy_alpha", 1)) * distances.get(rid, 0)
        + float(r.state.parameters.get("energy_beta", 0.1)) * agents[rid].idle_s
        for rid, r in engine.robots.items()
    )
    return dict(
        mean_delivery_s=(
            mean(o.delivered_at - o.release_time for o in completed)
            if completed
            else None
        ),
        distance_m=sum(distances.values()),
        conflict_episodes=sum(a.conflicts for a in agents.values()),
        vertex_conflicts=sum(a.vertex_conflicts for a in agents.values()),
        edge_conflicts=sum(a.edge_conflicts for a in agents.values()),
        forced_delay_episodes=sum(a.delay_episodes for a in agents.values()),
        delivery_imbalance=pstdev(a.delivered for a in agents.values()),
        energy=energy,
        released=len(orders),
        completed=len(completed),
        incomplete_fraction=1 - len(completed) / len(orders) if orders else 0.0,
        collision_episodes=len(engine.metrics.collisions),
        duration_s=engine.sim_time,
        autonomous_robot_s=sum(a.autonomous_s for a in agents.values()),
    )


def objective(metrics, config=None):
    cfg = {**DEFAULTS, **(config or {})}
    scales, weights = cfg["scales"], cfg["weights"]
    values = (
        list(scales.values())
        + list(weights.values())
        + [cfg["collision_penalty"], cfg["incomplete_weight"]]
    )
    if any(
        isinstance(v, bool)
        or not isinstance(v, (int, float))
        or not isfinite(v)
        or v < 0
        for v in values
    ):
        raise ValueError("Objective parameters must be finite and nonnegative")
    if (
        set(scales) != set(KEYS)
        or set(weights) != set(KEYS)
        or any(scales[k] <= 0 for k in KEYS)
    ):
        raise ValueError("Provide a positive scale and weight for each study metric")
    # No completion: use full experiment horizon as a documented censoring surrogate.
    values = {
        k: metrics[k] if metrics[k] is not None else metrics["duration_s"] for k in KEYS
    }
    terms = {k: weights[k] * values[k] / scales[k] for k in KEYS}
    terms["collisions"] = cfg["collision_penalty"] * metrics["collision_episodes"]
    terms["incomplete"] = cfg["incomplete_weight"] * metrics["incomplete_fraction"]
    return dict(
        version=1,
        value=sum(terms.values()),
        terms=terms,
        configuration=cfg,
        no_completions_surrogate="experiment_duration_s",
        direction="minimize",
    )


def baseline_scales(metrics, fallback):
    return {
        k: metrics[k] if metrics[k] is not None and metrics[k] > 0 else fallback[k]
        for k in KEYS
    }
