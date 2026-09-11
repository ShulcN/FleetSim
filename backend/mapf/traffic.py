"""Fixed-loop delivery executor. Solvers grant moves; AGVs follow and brake locally."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field, asdict
from math import hypot, isfinite

from backend.robots.controllers import Waypoint
from backend.wms.orders import Order, OrderStatus
from . import create_solver
from .base import MAPFProblem
from .joint_astar import resources_conflict, segment_distance
from .loop_network import LoopNetwork


@dataclass
class LoopAgent:
    route_id: str
    index: int
    pickup: int
    dropoff: int
    phase: str = "to_pickup"
    dwell_until: float = 0.0
    cycle: int = 0
    distance_steps: int = 0
    delivered: int = 0
    waiting_s: float = 0.0
    orders: list[dict] = field(default_factory=list)
    repeat_orders: bool = True
    release_time: float = 0.0
    service_time: float = 3.0
    offline_wait_until: float | None = None
    junction_passes: set[str] = field(default_factory=set)
    autonomous_s: float = 0.0
    junction_wait_s: float = 0.0


class LoopTraffic:
    def __init__(self, graph, robots, config):
        self.config = config
        self.horizon = int(config.get("horizon_steps", 4))
        self.time_limit = float(config.get("time_limit_s", 0.2))
        self.gap = float(config.get("safety_gap_m", 1.0))
        self.dwell_s = float(config.get("service_time_s", 3.0))
        if self.horizon < 1 or any(
            not isfinite(v) or v < 0 for v in (self.time_limit, self.gap, self.dwell_s)
        ):
            raise ValueError("Invalid fixed-loop planning settings")
        self.solver = create_solver(
            config.get("solver", "joint_astar"), config.get("solver_parameters")
        )
        radius = max((r.state.collision_radius for r in robots.values()), default=0.81)
        self.network = LoopNetwork(
            graph,
            float(config.get("cell_size_m", 5)),
            float(config.get("zone_radius_m", 3)),
            radius,
        )
        self.agents = {}
        for raw in config["assignments"]:
            rid = raw["robot_id"]
            if rid not in robots or rid in self.agents:
                raise ValueError(f"Unknown or duplicate robot: {rid}")
            if raw["route_id"] not in self.network.loops:
                raise ValueError(f"Unknown fixed loop: {raw['route_id']}")
            follower = robots[rid].route_follower
            if (
                follower is None
                or not (0 < follower.config.waypoint_tolerance <= 0.05)
                or not (0 < follower.config.stop_and_turn_angle <= 0.05)
            ):
                raise ValueError(
                    f"{rid}: fixed loops require waypoint_tolerance and stop_and_turn_angle in (0, 0.05]"
                )
            loop = self.network.loops[raw["route_id"]]
            indices = [
                int(raw[k]) for k in ("start_index", "pickup_index", "dropoff_index")
            ]
            if (
                any(i < 0 or i >= len(loop.nodes) for i in indices)
                or indices[1] == indices[2]
            ):
                raise ValueError(f"Invalid station or start indices: {rid}")
            node = self.network.graph.nodes[loop.nodes[indices[0]]]
            if hypot(robots[rid].state.x - node.x, robots[rid].state.y - node.y) > 0.05:
                raise ValueError(f"{rid} must start at its assigned loop position")
            specs = raw.get("orders") or [
                dict(
                    pickup_index=indices[1],
                    dropoff_index=indices[2],
                    release_time=0,
                    service_time_s=self.dwell_s,
                    cargo_type=raw["route_id"],
                )
            ]
            for spec in specs:
                if (
                    any(
                        not isinstance(spec[k], int)
                        or not 0 <= spec[k] < len(loop.nodes)
                        for k in ("pickup_index", "dropoff_index")
                    )
                    or spec["pickup_index"] == spec["dropoff_index"]
                ):
                    raise ValueError(f"{rid}: invalid delivery station indices")
                if any(
                    not isfinite(float(spec.get(k, 0))) or float(spec.get(k, 0)) < 0
                    for k in ("release_time", "service_time_s")
                ):
                    raise ValueError(f"{rid}: invalid delivery timing")
            self.agents[rid] = LoopAgent(
                raw["route_id"],
                *indices,
                orders=specs,
                repeat_orders=raw.get("repeat_orders", True),
            )
        if set(self.agents) != set(robots):
            raise ValueError("Every robot must have one fixed-loop assignment")
        self.targets = {}
        self.reservations = {}
        self.last_plan = None
        self.last_rejection = None
        self.calls = self.timeouts = self.partial_plans = 0
        self.max_solve_s = 0.0
        self.next_retry = 0.0
        self.started = False
        self.halted = set()
        self.connected = True
        self.autonomous_reason = None
        self.autonomy_events = []
        self.collision_context = []
        self._autonomy_started = None
        self._recovery_deadline = None
        self._now = 0.0
        self.junctions = {
            n.id: (
                n.x,
                n.y,
                float(
                    n.properties.get("control_radius_m", config.get("zone_radius_m", 3))
                ),
            )
            for n in graph.nodes.values()
            if n.properties.get("junction_id")
        }
        # Check initial reservations without initiating a search.
        ids = list(self.agents)
        for i, rid in enumerate(ids):
            a = self.agents[rid]
            loop = self.network.loops[a.route_id]
            p = robots[rid].state
            for other in ids[:i]:
                b = self.agents[other]
                q = robots[other].state
                if (
                    resources_conflict(
                        loop.resources[a.index],
                        self.network.loops[b.route_id].resources[b.index],
                    )
                    or hypot(p.x - q.x, p.y - q.y)
                    <= p.collision_radius + q.collision_radius + self.gap + 0.15
                ):
                    raise ValueError(f"Conflicting initial placement: {rid}, {other}")

    def _new_order(self, engine, rid, agent):
        if agent.cycle >= len(agent.orders) and not agent.repeat_orders:
            agent.phase = "complete"
            engine.robots[rid].clear_task()
            return
        spec = agent.orders[agent.cycle % len(agent.orders)]
        agent.cycle += 1
        agent.pickup, agent.dropoff = spec["pickup_index"], spec["dropoff_index"]
        agent.release_time = float(spec.get("release_time", 0))
        agent.service_time = float(spec.get("service_time_s", self.dwell_s))
        loop = self.network.loops[agent.route_id]
        order = Order(
            f"{rid}-{agent.cycle:04d}",
            agent.release_time,
            spec.get("cargo_type", agent.route_id),
            loop.nodes[agent.pickup],
            loop.nodes[agent.dropoff],
            [rid],
        )
        engine.wms.orders.append(order)
        robot = engine.robots[rid]
        robot.state.active_order_id = order.id
        robot.state.cargo_type = order.cargo_type
        robot.state.status = agent.phase = "waiting_release"
        robot.state.target_node = order.pickup_node
        if engine.sim_time >= agent.release_time:
            self._activate_order(engine, rid, agent, order)

    def _activate_order(self, engine, rid, agent, order):
        order.status = OrderStatus.ASSIGNED
        order.assigned_robot = rid
        order.released_at = order.assigned_at = engine.sim_time
        engine.fifo.register_released(order)
        for event in ("released", "assigned"):
            engine.metrics.record_order_event(engine.sim_time, event, order, rid)
        engine.robots[rid].state.status = agent.phase = "to_pickup"

    def _stations(self, engine):
        for rid, agent in self.agents.items():
            robot = engine.robots[rid]
            order = engine.wms.get_order(robot.state.active_order_id)
            if agent.phase == "complete" or rid in self.targets:
                continue
            if agent.phase == "waiting_release":
                if engine.sim_time < agent.release_time:
                    continue
                self._activate_order(engine, rid, agent, order)
            if agent.phase in ("loading", "unloading"):
                if engine.sim_time < agent.dwell_until:
                    continue
                if agent.phase == "loading":
                    order.status = OrderStatus.PICKED
                    order.picked_at = engine.sim_time
                    engine.metrics.record_order_event(
                        engine.sim_time, "picked", order, rid
                    )
                    agent.phase = robot.state.status = "to_dropoff"
                    robot.state.target_node = order.dropoff_node
                else:
                    order.status = OrderStatus.DELIVERED
                    order.delivered_at = engine.sim_time
                    engine.metrics.record_order_event(
                        engine.sim_time, "delivered", order, rid
                    )
                    engine.metrics.record_fifo_violation(
                        engine.fifo.register_delivered(order, engine.sim_time)
                    )
                    agent.delivered += 1
                    self._new_order(engine, rid, agent)
            target = agent.pickup if agent.phase == "to_pickup" else agent.dropoff
            if agent.phase in ("to_pickup", "to_dropoff") and agent.index == target:
                agent.phase = "loading" if agent.phase == "to_pickup" else "unloading"
                robot.state.status = agent.phase
                agent.dwell_until = engine.sim_time + agent.service_time

    async def before_step(self, engine):
        if not self.started:
            for rid, agent in self.agents.items():
                self._new_order(engine, rid, agent)
            self.started = True
        self._now = engine.sim_time
        if not self.connected:
            self._set_autonomy(engine, "connection_lost")
        self._stations(engine)
        if self.autonomous_reason:
            # Drain autonomous moves to their real endpoint outside junctions.
            # No position restoration or teleportation is used for recovery.
            recovering = self.connected and engine.sim_time >= self.next_retry
            if recovering and self._recovery_deadline is None:
                self._recovery_deadline = engine.sim_time + 2.0
            if recovering and engine.sim_time >= self._recovery_deadline:
                # Do not let a synchronization barrier block an autonomous queue:
                # release everyone and retry at a later set of route endpoints.
                recovering = False
                self.next_retry = engine.sim_time + 5.0
                self._recovery_deadline = None
            self._autonomous_step(engine, recovering)
            if (
                not recovering
                or self.targets
                or any(
                    self._inside_junction(engine, rid)
                    for rid in self.agents
                    if rid not in self.halted
                )
            ):
                return
        elif self.targets:
            return
        if not self.connected or engine.sim_time < self.next_retry:
            return
        self._recovery_deadline = None
        paths, resources = {}, {}
        for rid, agent in self.agents.items():
            loop = self.network.loops[agent.route_id]
            positions = [agent.index]
            if agent.phase in ("to_pickup", "to_dropoff") and rid not in self.halted:
                station = agent.pickup if agent.phase == "to_pickup" else agent.dropoff
                for _ in range(self.horizon):
                    nxt = (positions[-1] + 1) % len(loop.nodes)
                    positions.append(nxt)
                    if nxt == station:
                        break
            paths[rid] = [loop.nodes[i] for i in positions]
            resources[rid] = [loop.resources[i] for i in positions]
        problem = MAPFProblem(
            self.network.graph,
            {r: p[0] for r, p in paths.items()},
            {r: p[-1] for r, p in paths.items()},
            fixed_paths=paths,
            resources=resources,
            radii={
                r: robot.state.collision_radius for r, robot in engine.robots.items()
            },
            clearance=self.gap + 0.15,
            time_limit_s=self.time_limit,
            max_expansions=int(self.config.get("max_expansions", 100_000)),
        )
        self.last_rejection = None
        plan = await asyncio.to_thread(self.solver.solve, problem)
        self.calls += 1
        self.last_plan = dict(
            status=plan.status,
            message=plan.message,
            expanded=plan.expanded,
            elapsed_s=plan.elapsed_s,
            termination_reason=plan.metrics.get("termination_reason", plan.status),
        )
        self.max_solve_s = max(self.max_solve_s, plan.elapsed_s)
        self.timeouts += plan.status == "timeout"
        self.partial_plans += plan.status == "partial"
        if (
            plan.success
            and all(len(p) == 1 for p in problem.fixed_paths.values())
            and plan.paths == {r: [p[0]] for r, p in problem.fixed_paths.items()}
        ):
            self._set_autonomy(engine, None)
            self.next_retry = engine.sim_time + engine.settings.dt
            self._record_plan(engine, problem, plan)
            return
        if not plan.paths or min(map(len, plan.paths.values()), default=0) < 2:
            self.next_retry = engine.sim_time + 1.0
            self._set_autonomy(
                engine, plan.metrics.get("termination_reason", plan.status)
            )
            self._autonomous_step(engine, False)
            self._record_plan(engine, problem, plan)
            return
        self._commit(engine, problem, plan)
        if self.last_rejection:
            self._set_autonomy(engine, "execution_rejected")
            self._autonomous_step(engine, False)
        else:
            self._set_autonomy(engine, None)
        self._record_plan(engine, problem, plan)

    def _record_plan(self, engine, problem, plan):
        if not hasattr(engine, "recording"):
            return
        decisions = {}
        for rid, agent in self.agents.items():
            path = plan.paths.get(rid, [])
            reason = (
                "granted"
                if rid in self.targets
                else (
                    agent.phase
                    if agent.phase not in ("to_pickup", "to_dropoff")
                    else (
                        "halted"
                        if rid in self.halted
                        else (
                            "no_safe_prefix"
                            if len(path) < 2
                            else (
                                "execution_rejected"
                                if "safety check" in self.last_plan["message"]
                                else "solver_wait"
                            )
                        )
                    )
                )
            )
            decisions[rid] = dict(
                action="move" if rid in self.targets else "wait",
                reason=(
                    "junction_stop"
                    if agent.offline_wait_until is not None
                    else self.autonomous_reason
                )
                or reason,
                control_mode="autonomous" if self.autonomous_reason else "central",
            )
            if reason == "solver_wait" and len(problem.fixed_paths[rid]) > 1:
                blocked = []
                candidate = problem.resources[rid][0] + problem.resources[rid][1]
                for other, entries in problem.resources.items():
                    if other == rid:
                        continue
                    other_path = plan.paths.get(other, [])
                    step = int(len(other_path) > 1 and other_path[0] != other_path[1])
                    occupied = entries[0] + entries[step]
                    keys = sorted(
                        {
                            a.key
                            for a in candidate
                            for b in occupied
                            if resources_conflict((a,), (b,))
                        }
                    )
                    if keys:
                        blocked.append(dict(robot_id=other, resources=keys))
                decisions[rid]["observed_blockers"] = blocked
        engine.recording.journal.append(
            dict(
                id=self.calls,
                time=engine.sim_time,
                solver=self.solver.name,
                status=plan.status,
                message=self.last_plan["message"],
                elapsed_s=plan.elapsed_s,
                expanded=plan.expanded,
                cost=plan.cost,
                success=plan.success,
                metrics=plan.metrics,
                waiting_total_s=sum(a.waiting_s for a in self.agents.values()),
                timeout_total=self.timeouts,
                execution_rejection=self.last_rejection,
                mapf_snapshot=self.snapshot(),
                input=dict(
                    starts=problem.starts,
                    goals=problem.goals,
                    fixed_paths=problem.fixed_paths,
                    resources={
                        r: [[asdict(u) for u in uses] for uses in entries]
                        for r, entries in problem.resources.items()
                    },
                ),
                paths=plan.paths,
                decisions=decisions,
                robot_positions={
                    r: dict(
                        x=b.state.x,
                        y=b.state.y,
                        theta=b.state.theta,
                        status=b.state.status,
                        v=b.state.v,
                        omega=b.state.omega,
                        active_order_id=b.state.active_order_id,
                    )
                    for r, b in engine.robots.items()
                },
                reservations={
                    r: [asdict(u) for u in uses]
                    for r, uses in self.reservations.items()
                },
            )
        )

    def _commit(self, engine, problem, plan):
        """Validate plugin output independently before permitting a joint move."""
        if set(plan.paths) != set(self.agents):
            raise ValueError("Solver omitted robots")
        moves = {}
        for rid, agent in self.agents.items():
            path = plan.paths[rid]
            if path[0] != problem.starts[rid]:
                raise ValueError("Solver returned a stale start")
            step = 0 if path[1] == path[0] else 1
            if (
                step >= len(problem.fixed_paths[rid])
                or path[1] != problem.fixed_paths[rid][step]
            ):
                raise ValueError("Solver attempted to leave a fixed route")
            uses = problem.resources[rid][0] + problem.resources[rid][step]
            robot = engine.robots[rid]
            end = self.network.graph.nodes[path[1]]
            moves[rid] = ((robot.state.x, robot.state.y), (end.x, end.y), uses, step)
        ids = list(moves)
        for i, rid in enumerate(ids):
            a, b, uses, _ = moves[rid]
            for other in ids[:i]:
                c, d, other_uses, _ = moves[other]
                clearance = (
                    engine.robots[rid].state.collision_radius
                    + engine.robots[other].state.collision_radius
                    + self.gap
                    + 0.05
                )
                if (
                    resources_conflict(uses, other_uses)
                    or segment_distance(a, b, c, d) <= clearance
                ):
                    # Actual position deviation: retain current occupancy and retry.
                    self.next_retry = engine.sim_time + 1.0
                    self.last_plan["message"] = (
                        "Actual-position safety check prevented execution"
                    )
                    self.last_rejection = dict(
                        robots=[rid, other],
                        resource_conflict=resources_conflict(uses, other_uses),
                        separation_m=segment_distance(a, b, c, d),
                        required_separation_m=clearance,
                    )
                    return
        self.reservations = {rid: move[2] for rid, move in moves.items()}
        for rid, (_, end, _, step) in moves.items():
            if step:
                agent = self.agents[rid]
                self.targets[rid] = (agent.index + 1) % len(
                    self.network.loops[agent.route_id].nodes
                )
                engine.robots[rid].set_route([Waypoint(*end)])

    def set_connection(self, engine, connected):
        self.connected = connected
        self._recovery_deadline = None
        self.next_retry = engine.sim_time
        engine.recording.controls.append(
            dict(time=engine.sim_time, event="planner_connection", connected=connected)
        )
        if not connected:
            self._set_autonomy(engine, "connection_lost")

    def _set_autonomy(self, engine, reason):
        if reason == self.autonomous_reason:
            return
        if self._autonomy_started is not None:
            self.autonomy_events[-1].update(
                end_s=engine.sim_time,
                duration_s=engine.sim_time - self._autonomy_started,
            )
        self.autonomous_reason = reason
        self._autonomy_started = engine.sim_time if reason else None
        if reason:
            for rid, agent in self.agents.items():
                robot = engine.robots[rid].state
                target = (
                    self.network.graph.nodes[
                        self.network.loops[agent.route_id].nodes[self.targets[rid]]
                    ]
                    if rid in self.targets
                    else None
                )
                for jid, (x, y, radius) in self.junctions.items():
                    if (
                        hypot(robot.x - x, robot.y - y)
                        <= radius + robot.collision_radius
                        or target is not None
                        and segment_distance(
                            (robot.x, robot.y), (target.x, target.y), (x, y), (x, y)
                        )
                        <= radius + robot.collision_radius
                    ):
                        # Already granted entry completes without a new stop inside.
                        agent.junction_passes.add(jid)
            self.autonomy_events.append(
                dict(start_s=engine.sim_time, end_s=None, duration_s=0, reason=reason)
            )
            self.reservations.clear()
        else:
            self._recovery_deadline = None
            for agent in self.agents.values():
                agent.offline_wait_until = None
                agent.junction_passes.clear()

    def _inside_junction(self, engine, rid):
        robot = engine.robots[rid].state
        return any(
            hypot(robot.x - x, robot.y - y) <= radius + robot.collision_radius
            for x, y, radius in self.junctions.values()
        )

    def _autonomous_step(self, engine, recovering):
        draining = {rid for rid in self.agents if self._inside_junction(engine, rid)}
        # A leader just beyond a junction must not block a robot leaving it.
        if recovering:
            from math import cos, sin

            while True:
                leaders = set(draining)
                for rid in draining:
                    p = engine.robots[rid].state
                    for other, robot in engine.robots.items():
                        q = robot.state
                        dx, dy = q.x - p.x, q.y - p.y
                        if (
                            0
                            < dx * cos(p.theta) + dy * sin(p.theta)
                            < 2 * float(self.config.get("cell_size_m", 5))
                            + self.gap
                            + p.length
                            + q.length
                            and abs(-dx * sin(p.theta) + dy * cos(p.theta))
                            < (p.width + q.width) / 2
                            and cos(q.theta - p.theta) >= 0.8
                        ):
                            leaders.add(other)
                if leaders == draining:
                    break
                draining = leaders
        for rid, agent in self.agents.items():
            if (
                rid in self.targets
                or rid in self.halted
                or agent.phase not in ("to_pickup", "to_dropoff")
            ):
                continue
            if recovering and rid not in draining:
                continue
            robot = engine.robots[rid]
            loop = self.network.loops[agent.route_id]
            index = (agent.index + 1) % len(loop.nodes)
            node = self.network.graph.nodes[loop.nodes[index]]
            # Stop before the segment that enters a marked junction, once per visit.
            touching = {
                jid
                for jid, (x, y, radius) in self.junctions.items()
                if segment_distance(
                    (robot.state.x, robot.state.y), (node.x, node.y), (x, y), (x, y)
                )
                <= radius + robot.state.collision_radius
            }
            agent.junction_passes.intersection_update(touching)
            pending = touching - agent.junction_passes
            if pending:
                if agent.offline_wait_until is None:
                    agent.offline_wait_until = engine.sim_time + 5.0
                if engine.sim_time + 1e-8 < agent.offline_wait_until:
                    continue
                agent.junction_passes.update(pending)
                agent.offline_wait_until = None
            self.targets[rid] = index
            robot.set_route([Waypoint(node.x, node.y)])

    def after_step(self, engine):
        self._now = engine.sim_time + engine.settings.dt
        for rid, agent in self.agents.items():
            if self.autonomous_reason:
                agent.autonomous_s += engine.settings.dt
                if agent.offline_wait_until is not None and rid not in self.targets:
                    agent.junction_wait_s += engine.settings.dt
            if rid not in self.targets and agent.phase in ("to_pickup", "to_dropoff"):
                agent.waiting_s += engine.settings.dt
        if self.targets and (
            self.autonomous_reason
            or all(engine.robots[r].route_completed() for r in self.targets)
        ):
            for rid, index in list(self.targets.items()):
                if not engine.robots[rid].route_completed():
                    continue
                self.agents[rid].index = index
                self.agents[rid].distance_steps += 1
                engine.robots[rid].acknowledge_route_completed()
                del self.targets[rid]
            if not self.targets:
                self.reservations.clear()

    def autonomy_history(self):
        events = [dict(e) for e in self.autonomy_events]
        if events and events[-1]["end_s"] is None:
            events[-1]["duration_s"] = max(0, self._now - events[-1]["start_s"])
        return events

    def snapshot(self):
        return dict(
            solver=self.solver.name,
            connected=self.connected,
            control_mode="autonomous" if self.autonomous_reason else "central",
            recovering=self._recovery_deadline is not None,
            autonomous_reason=self.autonomous_reason,
            autonomy_events=self.autonomy_history(),
            horizon_steps=self.horizon,
            time_limit_s=self.time_limit,
            calls=self.calls,
            timeouts=self.timeouts,
            partial_plans=self.partial_plans,
            max_solve_s=self.max_solve_s,
            last_plan=self.last_plan,
            active_moves=list(self.targets),
            halted=list(self.halted),
            reservations={
                r: [dict(key=u.key, direction=u.direction) for u in set(uses)]
                for r, uses in self.reservations.items()
            },
            agents={
                r: dict(
                    route_id=a.route_id,
                    index=a.index,
                    phase=a.phase,
                    delivered=a.delivered,
                    steps=a.distance_steps,
                    waiting_s=a.waiting_s,
                    autonomous_s=a.autonomous_s,
                    junction_wait_s=a.junction_wait_s,
                    control_reason=(
                        "junction_stop"
                        if a.offline_wait_until is not None
                        else self.autonomous_reason
                    ),
                    junction_wait_remaining_s=max(
                        0, (a.offline_wait_until or 0) - self._now
                    ),
                )
                for r, a in self.agents.items()
            },
        )
