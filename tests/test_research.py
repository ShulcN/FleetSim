import asyncio
import json
from math import hypot, pi
from pathlib import Path
from types import SimpleNamespace

import pytest

from backend.research.motion import Piece, motion, rounded_loop
from backend.research.planning import TimedProblem, Schedule, Slot, Search, solve_timed
from backend.research.metrics import measure, objective, baseline_scales, DEFAULTS
from backend.research.graph_io import convert
from backend.maps.geojson_graph import load_geojson_graph
from backend.robots.factory import create_robot_from_config
from backend.robots.commands import ControlCommand


def robot(rid, x, y, kind="amr"):
    return create_robot_from_config(
        dict(
            id=rid,
            type=kind,
            initial_pose=dict(x=x, y=y, theta=0),
            footprint=dict(length=0.2, width=0.2),
            parameters=dict(max_linear=1, min_turn_radius=0.8),
            route_follower=dict(max_linear=1),
        )
    )


class TinyNav:
    tick_s = 0.5
    zones = []
    corridors = []

    def __init__(self):
        self.nodes = {
            "a": (-2, 0, 0),
            "b": (2, 0, 0),
            "c": (0, -2, pi / 2),
            "d": (0, 2, pi / 2),
        }
        self.robots = {"A": robot("A", -2, 0), "B": robot("B", 0, -2)}

    def distances(self, g):
        return {
            n: hypot(v[0] - self.nodes[g][0], v[1] - self.nodes[g][1])
            for n, v in self.nodes.items()
        }

    def neighbors(self, r, n, h):
        end = {"a": "b", "c": "d"}.get(n)
        if end:
            x, y, theta = self.nodes[n]
            yield motion(n, end, [Piece(x, y, theta, 4)], 1, 1, self.tick_s)

    def wait(self, n, h):
        x, y, _ = self.nodes[n]
        return motion(n, n, [Piece(x, y, h, 0)], 1, 1, self.tick_s)


def problem(nav=None, **kwargs):
    return TimedProblem(
        nav or TinyNav(),
        {"A": ("a", 0), "B": ("c", pi / 2)},
        dict(A="b", B="d"),
        {},
        dict(A=0, B=0),
        horizon=12,
        time_limit_s=3,
        max_expansions=50000,
        gap=0,
        **kwargs
    )


def test_ackermann_cannot_reverse_or_spin():
    r = robot("A", 0, 0, "agv")
    r.kinematics.step(r.state, ControlCommand(0, 2), 1)
    assert (r.state.x, r.state.y, r.state.theta) == (0, 0, 0)
    r.kinematics.step(r.state, ControlCommand(-1, 1), 1)
    assert r.state.v == 0 and r.state.omega == 0
    r.kinematics.step(r.state, ControlCommand(1, 10), 0.2)
    assert r.state.v > 0 and abs(r.state.omega) <= r.state.v / 0.8


def test_rounded_loop_is_continuous_forward_and_has_radius():
    pieces = rounded_loop([(0, 0), (10, 0), (10, 10), (0, 10)], 0.8, 3)
    for a, b in zip(pieces, pieces[1:] + pieces[:1]):
        end = a.pose(1)
        assert hypot(end[0] - b.x, end[1] - b.y) < 1e-8
        assert abs((end[2] - b.theta + pi) % (2 * pi) - pi) < 1e-8
        assert abs(a.curvature) <= 1 / 0.8
        assert a.length > 0 and a.spin == 0


@pytest.mark.parametrize("solver", ["whca", "cbs_astar", "joint_astar"])
def test_three_temporal_solvers_resolve_geometric_crossing(solver):
    p = problem()
    result = solve_timed(p, solver, {"max_ct_nodes": 500, "max_wait_steps": 12})
    assert result.plans, result.reason
    check = Search(p, {})
    assert check.conflict(result.plans) is None
    assert any(s.move.length for plan in result.plans.values() for s in plan.slots)
    assert result.metrics["optimality_proven"] is False


def test_shared_corridor_blocks_opponents_even_far_apart():
    nav = TinyNav()
    nav.corridors = [("lane", (-100, 0), (100, 0), 0.6)]
    nav.nodes["c"] = (80, 0, pi)
    nav.robots["B"].state.x = 80
    nav.robots["B"].state.y = 0
    p = problem(nav)
    assert (
        Search(p, {}).collision("A", Schedule((-80, 0, 0)), "B", Schedule((80, 0, pi)))
        is not None
    )
    assert (
        Search(p, {}).collision("A", Schedule((-80, 0, 0)), "B", Schedule((80, 5, pi)))
        is None
    )


def test_budget_is_shared_and_never_returns_unchecked_collision():
    p = problem()
    p.max_expansions = 1
    for solver in ["whca", "cbs_astar", "joint_astar"]:
        result = solve_timed(p, solver, {})
        assert result.expanded <= 1
        assert result.reason == "expansion_limit"
        assert not result.plans


def test_gml_transform_contraction_preserves_polyline_and_strict_json(tmp_path):
    source = tmp_path / "graph.gml"
    source.write_text(
        "graph [ directed 0 node [ id 0 world 0 world 0 ] node [ id 1 world 1 world 0 ] node [ id 2 world 1 world 1 ] edge [ source 0 target 1 ] edge [ source 1 target 2 ] ]"
    )
    target = tmp_path / "graph.geojson"
    stats = convert(
        source, target, scale=2, rotation=90, offset=(10, 20), max_chain_m=8
    )
    graph = load_geojson_graph(target)
    assert stats["working"] == dict(nodes=2, edges=1)
    edge = next(iter(graph.edges.values()))
    assert edge.coordinates == pytest.approx([(10, 20), (10, 22), (8, 22)])
    assert source.exists()
    json.loads(target.read_text(), parse_constant=lambda x: pytest.fail(x))


def test_objective_handles_zero_deliveries_and_penalizes_collisions():
    m = dict(
        mean_delivery_s=None,
        distance_m=0,
        conflict_episodes=0,
        delivery_imbalance=0,
        energy=0,
        duration_s=100,
        collision_episodes=0,
        incomplete_fraction=1,
    )
    score = objective(m)
    assert score["value"] >= 10
    assert objective({**m, "collision_episodes": 1})["value"] == score["value"] + 1000
    scales = baseline_scales(m, DEFAULTS["scales"])
    assert all(x > 0 for x in scales.values())


def test_mixed_headless_run_and_fixed_exogenous_releases():
    from backend.research.runner import evaluate, ROOT
    from backend.workspace.reports import read_report

    d = json.loads((ROOT / "examples/scenarios/factory_mixed_20.json").read_text())

    async def run():
        a = await evaluate(d, parameters={"max_expansions": 1}, duration_s=5)
        b = await evaluate(d, parameters={"max_expansions": 10000}, duration_s=5)
        return a, b

    a, b = asyncio.run(run())
    assert [(o["id"], o["release_time"]) for o in a["orders"]] == [
        (o["id"], o["release_time"]) for o in b["orders"]
    ]
    assert a["summary"]["duration_sim_s"] == 5
    assert a["research"]["metrics"]["autonomous_robot_s"] > 0
    assert not a["history"]["frames"]
    assert a["inputs"]["assets"]["amr_graph_asset"]["sha256"]


def test_offline_waits_five_seconds_then_crosses_without_sensing():
    from backend.research.traffic import MixedTraffic, Agent

    nav = TinyNav()
    nav.zones = [("cross", 0, 0, 0.2)]
    traffic = MixedTraffic.__new__(MixedTraffic)
    traffic.network = nav
    traffic.robots = nav.robots
    traffic.agents = {"A": Agent("AMR", "a")}
    traffic.routes = {"A": ["b"]}
    traffic.passes = {"A": set()}
    traffic.active = {}
    traffic.now = 0
    traffic.offline("A")
    assert not traffic.active and traffic.agents["A"].wait_until == 5
    traffic.now = 4.95
    traffic.offline("A")
    assert not traffic.active
    # B may occupy the crossing: offline execution deliberately never checks it.
    nav.robots["B"].state.x = nav.robots["B"].state.y = 0
    traffic.now = 5
    traffic.offline("A")
    assert "A" in traffic.active
    traffic.active.clear()
    traffic.routes["A"] = []
    traffic.offline("A")
    assert not traffic.active


def test_motion_execution_does_not_wait_for_other_robot():
    from backend.research.traffic import MixedTraffic, Agent

    nav = TinyNav()
    nav.robot_layers = {"A": "amr", "B": "amr"}
    traffic = MixedTraffic.__new__(MixedTraffic)
    traffic.network = nav
    traffic.robots = nav.robots
    traffic.agents = {"A": Agent("AMR", "a"), "B": Agent("AMR", "c")}
    traffic.active = {
        "A": (motion("a", "b", [Piece(-2, 0, 0, 1)], 1, 1, 0.5), 0),
        "B": (motion("c", "d", [Piece(0, -2, pi / 2, 2)], 1, 1, 0.5), 0),
    }
    traffic.queues = {"A": [], "B": []}
    traffic.gap = 0
    traffic.now = 0
    traffic.tick_s = 0.5
    traffic.next_plan = 2
    for _ in range(20):
        traffic.update_robots(None, 0.05)
    assert "A" not in traffic.active and "B" in traffic.active
    assert nav.robots["A"].state.x == pytest.approx(-1)


def test_contacts_count_once_until_separation():
    from backend.collisions import CollisionChecker

    a, b = robot("A", 0, 0), robot("B", 0, 0)
    c = CollisionChecker()
    c.episodes = True
    assert len(c.check([a, b], None, 0)) == 1
    assert not c.check([a, b], None, 10)
    b.state.x = 2
    assert not c.check([a, b], None, 11)
    b.state.x = 0
    assert len(c.check([a, b], None, 12)) == 1


def test_search_freezes_baseline_scales_and_seed(tmp_path, monkeypatch):
    from backend.research import runner

    calls = []

    async def fake(doc, solver, params, duration, scoring):
        calls.append((params, dict(scoring["scales"])))
        m = dict(
            mean_delivery_s=20,
            distance_m=100,
            conflict_episodes=0,
            delivery_imbalance=0,
            energy=30,
            duration_s=10,
            collision_episodes=0,
            incomplete_fraction=0.5,
            completed=2,
            released=4,
        )
        return dict(
            summary={}, research=dict(metrics=m, objective=objective(m, scoring))
        )

    monkeypatch.setattr(runner, "evaluate", fake)
    cfg = dict(
        seed=42,
        budget=2,
        space=dict(W=[2, 4], T_replan=[1, 4], C_wait=[1, 2], k_wait=[1], k_goal=[0.1]),
    )
    a = asyncio.run(runner.search({}, cfg, tmp_path / "a"))
    b = asyncio.run(runner.search({}, cfg, tmp_path / "b"))
    assert a["candidates"] == b["candidates"] and a["completed"] == 2
    assert calls[1][1] == calls[2][1] == a["objective"]["scales"]
    assert a["objective"]["scales"]["distance_m"] == 100
    assert a["objective"]["scales"]["conflict_episodes"] > 0
    assert (tmp_path / "a" / "summary.csv").exists()
