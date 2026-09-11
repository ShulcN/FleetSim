"""Explicit conversion of legacy scenario copies to forward-only AGV research mode."""

from copy import deepcopy
from math import hypot
from backend.mapf.loop_network import LoopNetwork
from backend.maps.geojson_graph import load_geojson_graph
from backend.maps.dxf_map import load_dxf_map
from backend.robots.factory import create_robots_from_fleet_dict
from .network import Navigation


def migrate(document, store, period_s=100):
    d = deepcopy(document)
    if d["wms"]["fixed_loops"].get("mode") == "mixed":
        return d
    cfg = d["wms"]["fixed_loops"]
    graph = load_geojson_graph(store.asset(d["graph_asset"], ".geojson"))
    old = LoopNetwork(graph, cfg.get("cell_size_m", 5))
    for r in d["fleet"]["robots"]:
        r["type"] = "agv"
        r.setdefault("parameters", {})["min_turn_radius"] = 0.8
    robots = {r.state.id: r for r in create_robots_from_fleet_dict(d["fleet"])}
    cfg.update(
        mode="mixed",
        solver="whca",
        solver_parameters={},
        planning_tick_s=0.5,
        stations=[],
        streams=[],
    )
    nav = Navigation(
        graph, None, robots, cfg, load_dxf_map(store.asset(d["map_asset"], ".dxf"))
    )
    for a in cfg["assignments"]:
        rid = a["robot_id"]
        r = next(r for r in d["fleet"]["robots"] if r["id"] == rid)
        points = [nav.nodes[n] for n in nav.loops[nav.robot_layers[rid]]]
        nearest = lambda x, y: min(
            range(len(points)), key=lambda i: hypot(points[i][0] - x, points[i][1] - y)
        )
        a["start_index"] = nearest(r["initial_pose"]["x"], r["initial_pose"]["y"])
        x, y, theta = points[a["start_index"]]
        r["initial_pose"] = dict(x=x, y=y, theta=theta)
        task = (a.get("orders") or [a])[0]
        indices = {}
        for key in ["pickup_index", "dropoff_index"]:
            node = old.graph.nodes[old.loops[a["route_id"]].nodes[task[key]]]
            indices[key] = nearest(node.x, node.y)
        cfg["streams"].append(
            dict(
                kind="agv",
                robot_id=rid,
                **indices,
                period_s=period_s,
                start_s=task.get("release_time", 0),
                service_time_s=task.get("service_time_s", 3)
            )
        )
        a.pop("orders", None)
        a.pop("repeat_orders", None)
    d.pop("id", None)
    d["name"] += " · кинематика AGV"
    d["description"] = (
        d.get("description", "")
        + " Миграция: первая пара доставки каждого AGV; независимый период "
        + str(period_s)
        + " с."
    )
    return d
