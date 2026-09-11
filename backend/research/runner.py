"""Headless API and CLI: run, seeded random search, or a finite Cartesian grid."""

import argparse
import asyncio
from copy import deepcopy
import csv
import hashlib
import itertools
import json
from pathlib import Path
import platform
import random
import sys
from time import perf_counter

from backend.collisions import CollisionMode
from backend.core.simulation_engine import SimulationEngine, SimulationSettings
from backend.maps.dxf_map import load_dxf_map
from backend.maps.geojson_graph import load_geojson_graph
from backend.mapf import validate_parameters, solver_catalog
from backend.robots.factory import create_robots_from_fleet_dict
from backend.wms.orders import WmsScenario
from backend.workspace.storage import WorkspaceStore, write_json
from backend.workspace.reports import build_report
from .metrics import DEFAULTS, measure, objective, baseline_scales

ROOT = Path(__file__).resolve().parents[2]


async def evaluate(
    document,
    solver="whca",
    parameters=None,
    duration_s=None,
    objective_config=None,
    history=False,
):
    """Isolated run returning a canonical report. Does not mutate the browser engine."""
    d = deepcopy(document)
    if d["wms"]["fixed_loops"].get("mode") != "mixed":
        raise ValueError(
            "Research evaluation requires a mixed-mode scenario with exogenous streams"
        )
    params = validate_parameters(solver, parameters)
    cfg = d["wms"]["fixed_loops"]
    cfg.update(solver=solver, solver_parameters=params)
    cfg.update(
        {k: params[k] for k in ("horizon_steps", "time_limit_s", "max_expansions")}
    )
    if duration_s is not None:
        d["simulation"]["duration_s"] = duration_s
    d["simulation"]["collision_mode"] = "count_only"
    store = WorkspaceStore(ROOT)
    d = store.validate(d)
    duration = d["simulation"]["duration_s"]
    if abs(duration / 0.05 - round(duration / 0.05)) > 1e-6:
        raise ValueError("Experiment duration must be a multiple of physics dt=0.05s")
    digest = hashlib.sha256()
    for path in sorted((ROOT / "backend").rglob("*.py")):
        digest.update(str(path.relative_to(ROOT)).encode())
        digest.update(path.read_bytes())
    inputs = dict(
        scenario=d,
        assets=store.fingerprint(d),
        software=dict(
            backend_sha256=digest.hexdigest(),
            python=sys.version,
            platform=platform.platform(),
        ),
        solver=dict(
            id=solver,
            parameters=params,
            descriptor=next(x for x in solver_catalog() if x["id"] == solver),
        ),
        simulation=dict(dt_s=0.05, duration_s=duration, collision_mode="count_only"),
    )
    e = SimulationEngine()
    e.use_comms = False
    await e.reset(
        robots=create_robots_from_fleet_dict(d["fleet"]),
        dxf_map=load_dxf_map(store.asset(d["map_asset"], ".dxf")),
        graph=load_geojson_graph(store.asset(d["graph_asset"], ".geojson")),
        amr_graph=(
            load_geojson_graph(store.asset(d["amr_graph_asset"], ".geojson"))
            if d.get("amr_graph_asset")
            else None
        ),
        wms=WmsScenario.from_dict(d["wms"]),
        settings=SimulationSettings(
            max_sim_time=duration, collision_mode=CollisionMode.COUNT_ONLY
        ),
        recording_inputs=inputs,
    )
    e.recording.enabled = history
    if not history:
        e.recording.frames.clear()
        e.recording.times.clear()
    await e.start()
    begun = perf_counter()
    for _ in range(round(duration / 0.05)):
        await e.step()
    e.status = "finished"
    e.sim_time = duration
    e.recording.capture(e, force=True)
    report = build_report(e, await e.export_metrics())
    metrics = measure(e)
    report["research"] = dict(
        schema_version=1,
        metrics=metrics,
        objective=objective(metrics, objective_config),
        wall_time_s=perf_counter() - begun,
    )
    report["summary"]["research"] = deepcopy(report["research"])
    return report


async def search(document, config, output):
    """Finite candidates use one baseline, frozen scales, and identical order streams."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("Choose a new output folder; existing series is preserved")
    duration = config.get("duration_s", 600)
    scoring = deepcopy({**DEFAULTS, **config.get("objective", {})})
    space = config.get(
        "space",
        dict(
            W=[8, 16, 32],
            T_replan=[2, 4],
            C_wait=[1, 2],
            k_wait=[0.5, 1],
            k_goal=[0.1, 0.5],
        ),
    )
    if set(space) != set(("W", "T_replan", "C_wait", "k_wait", "k_goal")) or any(
        not isinstance(v, list) or not v for v in space.values()
    ):
        raise ValueError(
            "space must contain nonempty candidate lists for all five WHCA parameters"
        )
    for key, values in space.items():
        for value in values:
            validate_parameters("whca", {key: value})
        space[key] = list(dict.fromkeys(values))
    budget = config.get("budget", 20)
    if (
        isinstance(budget, bool)
        or not isinstance(budget, int)
        or not 1 <= budget <= 100000
    ):
        raise ValueError("budget must be an integer in [1,100000]")
    keys = list(space)
    if config.get("method", "random") == "random":
        from math import prod

        pairs = [(w, t) for w in space["W"] for t in space["T_replan"] if t <= w]
        dimensions = [pairs, space["C_wait"], space["k_wait"], space["k_goal"]]
        total = prod(map(len, dimensions))
        candidates = []
        for index in random.Random(config.get("seed", 42)).sample(
            range(total), min(total, budget)
        ):
            chosen = []
            for values in reversed(dimensions):
                index, remainder = divmod(index, len(values))
                chosen.append(values[remainder])
            pair, cw, kw, kg = reversed(chosen)
            candidates.append(
                dict(W=pair[0], T_replan=pair[1], C_wait=cw, k_wait=kw, k_goal=kg)
            )
    elif config["method"] == "grid":
        candidates = list(
            itertools.islice(
                (
                    dict(zip(keys, v))
                    for v in itertools.product(*(space[k] for k in keys))
                    if dict(zip(keys, v))["T_replan"] <= dict(zip(keys, v))["W"]
                ),
                budget,
            )
        )
    else:
        raise ValueError("method must be grid or random")
    if not candidates:
        raise ValueError("No valid candidates with T_replan <= W")
    baseline = await evaluate(
        document, "whca", config.get("baseline_parameters", {}), duration, scoring
    )
    scoring["scales"] = baseline_scales(
        baseline["research"]["metrics"], scoring["scales"]
    )
    baseline["research"]["objective"] = objective(
        baseline["research"]["metrics"], scoring
    )
    baseline["summary"]["research"] = deepcopy(baseline["research"])
    write_json(output / "baseline.json", baseline)
    manifest = dict(
        kind="fleetsim.research-series",
        schema_version=1,
        config=config,
        objective=scoring,
        scenario_sha256=hashlib.sha256(
            json.dumps(document, sort_keys=True).encode()
        ).hexdigest(),
        candidates=candidates,
        completed=0,
    )
    write_json(output / "series.json", manifest)
    rows = []
    for i, params in enumerate(candidates):
        params = {**config.get("fixed_parameters", {}), **params}
        report = await evaluate(document, "whca", params, duration, scoring)
        name = f"run-{i+1:04}.json"
        write_json(output / name, report)
        rows.append(
            dict(
                file=name,
                **params,
                **report["research"]["metrics"],
                score=report["research"]["objective"]["value"],
            )
        )
        with (output / "summary.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        manifest["completed"] = i + 1
        manifest["best"] = min(rows, key=lambda r: r["score"])
        write_json(output / "series.json", manifest)
        print(f'{i+1}/{len(candidates)} score={rows[-1]["score"]:.6f}', flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["run", "search"])
    parser.add_argument(
        "--scenario", default="examples/scenarios/factory_mixed_20.json"
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--config", help="JSON search configuration")
    parser.add_argument(
        "--solver", default="whca", choices=["whca", "joint_astar", "cbs_astar"]
    )
    parser.add_argument("--parameters", default="{}", help="JSON solver parameters")
    parser.add_argument("--duration", type=float, default=600)
    parser.add_argument("--history", action="store_true")
    args = parser.parse_args()
    d = json.loads(Path(args.scenario).read_text())
    if args.action == "run":
        out = Path(args.output)
        if out.exists():
            parser.error("Output already exists")
        report = asyncio.run(
            evaluate(
                d,
                args.solver,
                json.loads(args.parameters),
                args.duration,
                history=args.history,
            )
        )
        write_json(out, report)
        print(json.dumps(report["research"], ensure_ascii=False, indent=2))
    else:
        cfg = (
            json.loads(Path(args.config).read_text())
            if args.config
            else dict(duration_s=args.duration)
        )
        asyncio.run(search(d, cfg, args.output))


if __name__ == "__main__":
    main()
