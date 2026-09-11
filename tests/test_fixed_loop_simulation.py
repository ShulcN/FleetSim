import asyncio
from dataclasses import replace
from math import hypot
from pathlib import Path

import pytest

from backend.collisions import CollisionMode
from backend.core.simulation_engine import SimulationEngine
from backend.mapf import MAPFSolution
from backend.mapf.joint_astar import resources_conflict, segment_distance
from backend.robots.commands import ControlCommand
from backend.robots.factory import load_fleet_from_file, create_robot_from_config

ROOT = Path(__file__).resolve().parents[1]


async def demo(with_map=False):
    engine = SimulationEngine()
    engine.use_comms = False
    await engine.configure_from_files(
        load_fleet_from_file(ROOT/'examples/fleets/factory_agv_15.json'),
        ROOT/'examples/maps/factory.dxf' if with_map else None,
        ROOT/'examples/graphs/factory_routes.geojson',
        ROOT/'examples/scenarios/factory_agv_15.json',600,CollisionMode.STOP_ON_COLLISION,
    )
    return engine


def test_ten_minute_factory_run_delivers_with_15_agvs_on_10_loops():
    async def run():
        engine = await demo(with_map=True)
        traffic = engine.traffic
        assert len(engine.robots) == 15
        assert len({a.route_id for a in traffic.agents.values()}) == 10
        assert len(traffic.network.loops) == 10
        await engine.start()
        zone_visits = {}
        last_indices = {r:a.index for r,a in traffic.agents.items()}
        while engine.status == 'running':
            await engine.step()
            robots = list(engine.robots.values())
            for i,robot in enumerate(robots):
                s = robot.state
                assert s.length == 1.5 and s.width == 0.6
                assert abs(s.v) <= 1.0
                for other in robots[:i]:
                    q = other.state
                    assert hypot(s.x-q.x,s.y-q.y) >= s.collision_radius+q.collision_radius+1.0-1e-6
            # Independently verify actual footprint occupancy of intersections.
            for node in engine.graph.nodes.values():
                occupants = [r.state.id for r in robots if hypot(r.state.x-node.x,r.state.y-node.y) <= 3+r.state.collision_radius]
                assert len(occupants) <= 1
                zone_visits.setdefault(node.id,set()).update(occupants)
            for rid,agent in traffic.agents.items():
                loop = traffic.network.loops[agent.route_id]
                assert agent.index in (last_indices[rid],(last_indices[rid]+1)%len(loop.nodes))
                last_indices[rid] = agent.index
                # AGV stays on its fixed segment, including during rotations.
                start = traffic.network.graph.nodes[loop.nodes[agent.index]]
                end_index = traffic.targets.get(rid,agent.index)
                end = traffic.network.graph.nodes[loop.nodes[end_index]]
                p = engine.robots[rid].state
                assert segment_distance((p.x,p.y),(p.x,p.y),(start.x,start.y),(end.x,end.y)) < 0.06
            uses = list(traffic.reservations.values())
            assert not any(resources_conflict(a,b) for i,a in enumerate(uses) for b in uses[:i])
        assert engine.status == 'finished'
        assert not engine.metrics.collisions
        assert all(a.delivered >= 1 for a in traffic.agents.values())
        assert all(d > 200 for d in engine.metrics.robot_distance_m.values())
        assert sum(a.delivered for a in traffic.agents.values()) >= 20
        assert all(r.state.active_order_id for r in engine.robots.values())
        assert any(a.waiting_s > 3 for a in traffic.agents.values())
        assert sum(len(visitors)>1 for visitors in zone_visits.values()) >= 5
        assert traffic.calls > 10
        assert traffic.max_solve_s < 0.35  # scheduling tolerance over the 200 ms search budget
        exported = await engine.export_metrics()
        assert exported['mapf']['solver'] == 'joint_astar'
        return exported
    asyncio.run(run())


def test_timeout_preserves_inflight_grants_then_enters_autonomy():
    async def run():
        engine = await demo()
        await engine.start()
        while not engine.traffic.targets:
            await engine.step()
        traffic = engine.traffic
        targets = dict(traffic.targets)
        reservations = dict(traffic.reservations)
        calls = traffic.calls
        traffic.time_limit = 0
        await engine.step()
        assert traffic.targets == targets and traffic.reservations == reservations
        assert traffic.calls == calls
        for _ in range(400):
            await engine.step()
            if traffic.calls > calls:
                break
        assert traffic.autonomous_reason == 'time_limit'
        assert traffic.last_plan['status'] == 'timeout'
        positions = [(r.state.x,r.state.y) for r in engine.robots.values()]
        for _ in range(30):
            await engine.step()
        assert positions != [(r.state.x,r.state.y) for r in engine.robots.values()]
        assert traffic.autonomy_history()[0]['reason'] == 'time_limit'
    asyncio.run(run())


def test_stop_restart_retains_plan_and_manual_routes_cannot_override_it():
    async def run():
        engine = await demo()
        await engine.start()
        while not engine.traffic.targets:
            await engine.step()
        reservations = dict(engine.traffic.reservations)
        positions = [(r.state.x,r.state.y) for r in engine.robots.values()]
        await engine.stop()
        await engine.step()
        assert positions == [(r.state.x,r.state.y) for r in engine.robots.values()]
        assert engine.traffic.reservations == reservations
        with pytest.raises(ValueError,match='centrally controlled'):
            await engine.set_robot_command('AGV01',ControlCommand(linear=1))
        with pytest.raises(ValueError,match='centrally controlled'):
            await engine.set_robot_route_by_nodes('AGV01',['J01','N1'])
        await engine.start()
        await engine.step()
        assert positions != [(r.state.x,r.state.y) for r in engine.robots.values()]
    asyncio.run(run())


def test_executor_rejects_solver_path_outside_fixed_loop():
    async def run():
        engine = await demo()
        class InvalidSolver:
            name = 'invalid'
            def solve(self,problem):
                paths = {r:[p[0],p[0]] for r,p in problem.fixed_paths.items()}
                paths['AGV01'][1] = 'not_on_this_loop'
                return MAPFSolution(paths)
        engine.traffic.solver = InvalidSolver()
        await engine.start()
        with pytest.raises(ValueError,match='leave a fixed route'):
            await engine.step()
        assert not engine.traffic.targets
        assert not any(r.has_active_route() for r in engine.robots.values())
    asyncio.run(run())


def test_invalid_configuration_does_not_replace_running_world():
    async def run():
        engine = await demo()
        before = engine.robots
        wms = engine.wms
        wms.fixed_loops = {**wms.fixed_loops,'solver':'missing'}
        with pytest.raises(ValueError,match='Unknown MAPF solver'):
            await engine.reset([],graph=engine.graph,wms=wms)
        assert engine.robots is before
    asyncio.run(run())


def test_agv_local_distance_keeping_stops_behind_leader_without_changing_route():
    follower,leader = [create_robot_from_config(dict(id=str(i),initial_pose=dict(x=x,y=0),
                         footprint=dict(length=1.5,width=0.6))) for i,x in enumerate((0,2.7))]
    from backend.robots.controllers import Waypoint
    follower.set_route([Waypoint(20,0)])
    for _ in range(100):
        follower.update(0.05,neighbors=[replace(follower.state),replace(leader.state)],safety_gap=1.0)
    assert follower.state.x < 0.1
    assert follower.state.v < 1e-6
    assert follower.has_active_route()
    assert leader.state.x-follower.state.x-1.5 >= 1.0
