import asyncio
from copy import deepcopy
from math import hypot
from pathlib import Path

from backend.collisions import CollisionMode
from backend.core.simulation_engine import SimulationEngine, SimulationSettings
from backend.maps.geojson_graph import GeoJsonRouteGraph, GraphNode, GraphEdge
from backend.robots.factory import create_robots_from_fleet_dict
from backend.wms.orders import WmsScenario
from backend.workspace.storage import WorkspaceStore

ROOT = Path(__file__).resolve().parents[1]


async def one_robot():
    graph = GeoJsonRouteGraph()
    for key, xy in dict(A=(0, 0), J=(10, 0), B=(20, 0), C=(20, 20), D=(0, 20)).items():
        graph.nodes[key] = GraphNode(
            key,
            xy[0] + 10,
            xy[1] + 10,
            properties=(
                {"junction_id": "J", "control_radius_m": 3} if key == "J" else {}
            ),
        )
    seq = ["A", "J", "B", "C", "D", "A"]
    for a, b in zip(seq, seq[1:]):
        u, v = graph.nodes[a], graph.nodes[b]
        graph.edges[a + b] = GraphEdge(a + b, a, b, [(u.x, u.y), (v.x, v.y)])
    graph.routes = [dict(route_id="R", node_ids=seq)]
    fleet = {
        "robots": [
            dict(
                id="R",
                initial_pose=dict(x=10, y=10, theta=0),
                footprint=dict(length=1.5, width=0.6),
                parameters=dict(max_linear=1, max_angular=1.5),
                route_follower=dict(
                    max_linear=1, waypoint_tolerance=0.02, stop_and_turn_angle=0.02
                ),
            )
        ]
    }
    cfg = dict(
        assignments=[
            dict(
                robot_id="R",
                route_id="R",
                start_index=0,
                pickup_index=0,
                dropoff_index=4,
                repeat_orders=False,
            )
        ],
        service_time_s=0,
        cell_size_m=5,
    )
    engine = SimulationEngine()
    engine.use_comms = False
    await engine.reset(
        robots=create_robots_from_fleet_dict(fleet),
        graph=graph,
        wms=WmsScenario.from_dict(dict(fixed_loops=cfg, orders=[])),
        settings=SimulationSettings(
            max_sim_time=100, collision_mode=CollisionMode.COUNT_ONLY
        ),
    )
    await engine.start()
    return engine


def test_offline_waits_five_seconds_once_and_delivers_without_planner():
    async def run():
        e = await one_robot()
        t = e.traffic
        t.set_connection(e, False)
        waits = []
        last = None
        for _ in range(1500):
            await e.step()
            a = t.agents["R"]
            if a.offline_wait_until is not None:
                waits.append(e.sim_time)
                assert 14.9 < e.robots["R"].state.x < 15.1  # outside marked zone
            if a.delivered:
                break
        assert a.delivered == 1
        assert 4.9 <= a.junction_wait_s <= 5.1
        assert len(waits) >= 98
        assert t.calls == 0
        assert t.autonomy_history()[0]["reason"] == "connection_lost"

    asyncio.run(run())


def test_reconnect_inside_junction_drains_then_replans_without_teleport():
    async def run():
        e = await one_robot()
        t = e.traffic
        t.set_connection(e, False)
        for _ in range(1600):
            await e.step()
            if e.robots["R"].state.x >= 19:
                break
        assert e.robots["R"].state.x >= 19
        t.set_connection(e, True)
        previous = e.robots["R"].state.x
        assert t.autonomous_reason == "connection_lost"
        for _ in range(1000):
            await e.step()
            x = e.robots["R"].state.x
            assert -1e-8 <= x - previous <= 0.051
            previous = x
            if t.autonomous_reason is None:
                assert x > 23.8
                break
        else:
            raise AssertionError("Central control did not recover")
        assert t.autonomy_history()[-1]["end_s"] is not None
        assert t.calls > 0

    asyncio.run(run())


def test_low_budget_keeps_autonomous_robots_moving_and_recovers_when_budget_returns():
    async def run():
        e = await one_robot()
        t = e.traffic
        t.time_limit = 0
        for _ in range(450):
            await e.step()
        assert e.robots["R"].state.x > 15
        assert t.autonomous_reason == "time_limit"
        t.time_limit = 0.2
        for _ in range(1000):
            await e.step()
            if t.autonomous_reason is None:
                break
        assert t.autonomous_reason is None

    asyncio.run(run())


def test_central_wait_does_not_expire_after_five_seconds():
    async def run():
        e = await one_robot()
        t = e.traffic
        from backend.mapf import MAPFSolution

        class Wait:
            name = "wait"

            def solve(self, p):
                return MAPFSolution(
                    paths={r: [v, v] for r, v in p.starts.items()},
                    status="partial",
                    success=False,
                )

        t.solver = Wait()
        for _ in range(160):
            await e.step()
        assert e.robots["R"].state.x == 10
        assert t.autonomous_reason is None

    asyncio.run(run())


def test_stress_scenario_has_valid_30_robots_on_ten_loops():
    import json

    store = WorkspaceStore(ROOT)
    doc = dict(
        schema_version=1,
        kind="fleetsim.scenario",
        name="Stress",
        map_asset="examples/maps/factory.dxf",
        graph_asset="examples/graphs/factory_routes.geojson",
        fleet=json.loads((ROOT / "examples/fleets/factory_stress_30.json").read_text()),
        wms=json.loads(
            (ROOT / "examples/scenarios/factory_stress_30.json").read_text()
        ),
        simulation=dict(duration_s=600, collision_mode="count_only"),
    )
    valid = store.validate(doc)
    assert len(valid["fleet"]["robots"]) == 30
    assert (
        len({a["route_id"] for a in valid["wms"]["fixed_loops"]["assignments"]}) == 10
    )


def test_local_distance_controller_does_not_yield_to_crossing_traffic():
    from dataclasses import replace
    from math import pi
    from backend.robots.controllers import Waypoint
    from backend.robots.factory import create_robot_from_config

    robot = create_robot_from_config(
        dict(id="follower", initial_pose=dict(x=10, y=10, theta=0))
    )
    crossing = create_robot_from_config(
        dict(id="crossing", initial_pose=dict(x=11, y=10, theta=pi / 2))
    )
    robot.set_route([Waypoint(20, 10)])
    robot.update(
        0.05, neighbors=[replace(robot.state), replace(crossing.state)], safety_gap=1
    )
    assert robot.state.x > 10
