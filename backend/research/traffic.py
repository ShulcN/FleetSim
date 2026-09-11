"""Asynchronous heterogeneous fleet execution and independent order releases."""

from dataclasses import dataclass
from math import hypot, cos, sin, ceil, isfinite
import asyncio
from .network import Navigation
from .planning import Schedule, Slot, TimedProblem
from backend.mapf import create_solver
from backend.wms.orders import Order, OrderStatus


@dataclass
class Agent:
    route_id: str
    node: str
    index: int = 0
    phase: str = "idle"
    delivered: int = 0
    waiting_s: float = 0
    autonomous_s: float = 0
    junction_wait_s: float = 0
    idle_s: float = 0
    service_until: float = 0
    reason: str | None = None
    wait_until: float | None = None
    delayed: bool = False
    delay_episodes: int = 0
    conflicts: int = 0
    vertex_conflicts: int = 0
    edge_conflicts: int = 0
    blocked: bool = False
    rerouting: bool = False


class MixedTraffic:
    temporal = True

    def __init__(self, graph, amr_graph, robots, config, dxf=None):
        self.config = config
        self.robots = robots
        self.network = Navigation(graph, amr_graph, robots, config, dxf)
        self.solver = create_solver(
            config.get("solver", "whca"), config.get("solver_parameters")
        )
        params = self.solver.parameters
        self.horizon = int(params.get("W", config.get("horizon_steps", 16)))
        self.tick_s = self.network.tick_s
        self.period = int(params.get("T_replan", config.get("replan_ticks", 4)))
        self.time_limit = float(params.get("time_limit_s", 0.2))
        self.gap = float(config.get("safety_gap_m", 1))
        if (
            not 0.05 <= self.tick_s <= 5
            or abs(self.tick_s / 0.05 - round(self.tick_s / 0.05)) > 1e-8
        ):
            raise ValueError(
                "planning_tick_s must be a multiple of 0.05, between 0.05 and 5"
            )
        if self.period < 1 or self.period > self.horizon:
            raise ValueError("T_replan must not exceed W/horizon_steps")
        if not isfinite(self.gap) or self.gap < 0:
            raise ValueError("Safety gap must be finite and nonnegative")
        self.agents = {}
        assign = {a["robot_id"]: a for a in config.get("assignments", [])}
        self.station_nodes = {
            s["id"]: "amr:" + str(s["node_id"]).removeprefix("amr:")
            for s in config.get("stations", [])
        }
        if len(self.station_nodes) != len(config.get("stations", [])):
            raise ValueError("Station IDs must be unique")
        if any(n not in self.network.nodes for n in self.station_nodes.values()):
            raise ValueError("Unknown AMR station vertex")
        for rid, r in robots.items():
            layer = self.network.robot_layers[rid]
            if layer == "amr":
                node = "amr:" + str(
                    r.state.parameters.get("start_node", "")
                ).removeprefix("amr:")
            else:
                points = self.network.loops[layer]
                index = int(assign[rid]["start_index"])
                if not 0 <= index < len(points):
                    raise ValueError("AGV start_index outside rounded route")
                node = points[index]
            if node not in self.network.nodes:
                raise ValueError(f"{rid}: invalid starting vertex")
            x, y, theta = self.network.nodes[node]
            if hypot(r.state.x - x, r.state.y - y) > 0.03:
                raise ValueError(
                    f"{rid}: initial pose must match selected route vertex ({x:.3f}, {y:.3f})"
                )
            if (
                layer != "amr"
                and abs(
                    __import__(
                        "backend.robots.base", fromlist=["normalize_angle"]
                    ).normalize_angle(r.state.theta - theta)
                )
                > 0.01
            ):
                raise ValueError(
                    f"{rid}: initial heading must match tangent of AGV route"
                )
            if not self.network.walls.clear((x, y), (x, y), r.state.collision_radius):
                raise ValueError(f"{rid}: starting footprint intersects obstacle")
            self.agents[rid] = Agent(
                assign[rid]["route_id"] if layer != "amr" else "AMR", node
            )
        self.streams = []
        seen_agv = set()
        for raw in config.get("streams", []):
            period = float(raw["period_s"])
            release = float(raw.get("start_s", 0))
            service = float(raw.get("service_time_s", 3))
            if (
                any(not isfinite(x) for x in [period, release, service])
                or period <= 0
                or release < 0
                or service < 0
            ):
                raise ValueError("Invalid order stream timing")
            if raw["kind"] not in ("agv", "amr"):
                raise ValueError("Unknown stream kind")
            if raw["kind"] == "agv":
                rid = raw["robot_id"]
                if rid in seen_agv:
                    raise ValueError("AGV requires one fixed delivery stream")
                seen_agv.add(rid)
                layer = self.network.robot_layers[rid]
                if layer == "amr":
                    raise ValueError("AGV stream assigned to AMR")
                nodes = self.network.loops[layer]
                if any(
                    isinstance(raw[k], bool)
                    or int(raw[k]) != raw[k]
                    or not 0 <= raw[k] < len(nodes)
                    for k in ("pickup_index", "dropoff_index")
                ):
                    raise ValueError("AGV station index outside rounded loop")
                a, b = nodes[int(raw["pickup_index"])], nodes[int(raw["dropoff_index"])]
                eligible = [rid]
            else:
                a, b = (
                    self.station_nodes[raw["pickup"]],
                    self.station_nodes[raw["dropoff"]],
                )
                eligible = sorted(
                    r for r in self.agents if self.network.robot_layers[r] == "amr"
                )
            if a == b:
                raise ValueError("Pickup and dropoff must differ")
            if not eligible:
                raise ValueError("Order stream has no compatible robots")
            self.streams.append(
                dict(
                    raw=raw,
                    next=release,
                    period=period,
                    service=service,
                    pickup=a,
                    dropoff=b,
                    eligible=eligible,
                    count=0,
                )
            )
        # Bound station reachability at preparation, with actual footprint clearance.
        for stream in self.streams:
            if not any(
                self.network.route(
                    r,
                    stream["pickup"],
                    stream["dropoff"],
                    self.network.nodes[stream["pickup"]][2],
                )
                is not None
                for r in stream["eligible"]
            ):
                raise ValueError(
                    "Order stations are disconnected for the eligible fleet"
                )
        ids = list(robots)
        for i, rid in enumerate(ids):
            for sid in ids[:i]:
                a, b = robots[rid].state, robots[sid].state
                if (
                    hypot(a.x - b.x, a.y - b.y)
                    <= a.collision_radius + b.collision_radius + self.gap
                ):
                    raise ValueError(f"Conflicting initial positions: {rid}, {sid}")
        self.active = {}
        self.queues = {r: [] for r in robots}
        self.routes = {}
        self.passes = {r: set() for r in robots}
        self.order_services = {}
        self.connected = True
        self.next_plan = 0.0
        self.calls = self.timeouts = self.partial_plans = 0
        self.max_solve_s = 0.0
        self.last_plan = None
        self.halted = set()
        self.reservations = {}
        self.autonomous_reason = None
        self.autonomy_events = []
        self.collision_context = []
        self.now = 0.0
        self.planned_paths = {}

    def set_connection(self, engine, connected):
        self.connected = connected
        self.next_plan = engine.sim_time
        engine.recording.controls.append(
            dict(time=engine.sim_time, event="planner_connection", connected=connected)
        )
        if not connected:
            for rid, a in self.agents.items():
                a.reason = "connection_lost"
                self.queues[rid] = []
        self.sync_mode()

    def sync_mode(self):
        reason = next((a.reason for a in self.agents.values() if a.reason), None)
        if reason == self.autonomous_reason:
            return
        if self.autonomy_events and self.autonomy_events[-1]["end_s"] is None:
            self.autonomy_events[-1].update(
                end_s=self.now,
                duration_s=self.now - self.autonomy_events[-1]["start_s"],
            )
        if reason:
            self.autonomy_events.append(
                dict(start_s=self.now, end_s=None, duration_s=0, reason=reason)
            )
        self.autonomous_reason = reason

    def autonomy_history(self):
        out = [dict(e) for e in self.autonomy_events]
        if out and out[-1]["end_s"] is None:
            out[-1]["duration_s"] = self.now - out[-1]["start_s"]
        return out

    def goal(self, engine, rid):
        a = self.agents[rid]
        order = engine.wms.get_order(self.robots[rid].state.active_order_id)
        if not order:
            return None
        return (
            order.pickup_node
            if a.phase == "to_pickup"
            else order.dropoff_node if a.phase == "to_dropoff" else None
        )

    def release_and_dispatch(self, engine):
        for i, stream in enumerate(self.streams):
            while stream["next"] <= self.now + 1e-8:
                stream["count"] += 1
                oid = f'S{i+1}-{stream["count"]:06}'
                order = Order(
                    oid,
                    stream["next"],
                    stream["raw"]["kind"],
                    stream["pickup"],
                    stream["dropoff"],
                    stream["eligible"],
                    status=OrderStatus.RELEASED,
                    released_at=stream["next"],
                )
                engine.wms.orders.append(order)
                self.order_services[oid] = stream["service"]
                stream["next"] += stream["period"]
                engine.metrics.record_order_event(self.now, "released", order, None)
        for order in sorted(
            engine.wms.pending_orders(), key=lambda o: (o.release_time, o.id)
        ):
            candidates = []
            for r in order.eligible_robots:
                if self.agents[r].phase != "idle" or r in self.halted:
                    continue
                robot = self.robots[r]
                route = self.network.route(
                    r, self.agents[r].node, order.pickup_node, robot.state.theta
                )
                if route is not None:
                    candidates.append((sum(m.duration for m in route), r, route))
            if not candidates:
                continue
            _, rid, route = min(candidates, key=lambda x: (x[0], x[1]))
            a = self.agents[rid]
            order.status = OrderStatus.ASSIGNED
            order.assigned_robot = rid
            order.assigned_at = self.now
            self.robots[rid].state.active_order_id = order.id
            self.robots[rid].state.cargo_type = order.cargo_type
            a.phase = "to_pickup"
            self.routes[rid] = (
                [m.end for m in route]
                if self.network.robot_layers[rid] != "amr"
                else []
            )
            engine.metrics.record_order_event(self.now, "assigned", order, rid)

    def stations(self, engine):
        for rid, a in self.agents.items():
            if rid in self.active:
                continue
            order = engine.wms.get_order(self.robots[rid].state.active_order_id)
            if not order:
                continue
            if (
                a.phase in ("loading", "unloading")
                and self.now + 1e-8 >= a.service_until
            ):
                if a.phase == "loading":
                    order.status = OrderStatus.PICKED
                    order.picked_at = self.now
                    a.phase = "to_dropoff"
                    route = self.network.route(
                        rid, a.node, order.dropoff_node, self.robots[rid].state.theta
                    )
                    self.routes[rid] = (
                        [m.end for m in route]
                        if route is not None and self.network.robot_layers[rid] != "amr"
                        else []
                    )
                    engine.metrics.record_order_event(self.now, "picked", order, rid)
                else:
                    order.status = OrderStatus.DELIVERED
                    order.delivered_at = self.now
                    a.delivered += 1
                    a.phase = "idle"
                    self.robots[rid].clear_task()
                    engine.metrics.record_order_event(self.now, "delivered", order, rid)
            goal = self.goal(engine, rid)
            if goal == a.node:
                a.phase = "loading" if a.phase == "to_pickup" else "unloading"
                a.service_until = self.now + self.order_services[order.id]
                self.queues[rid] = []
            self.robots[rid].state.status = a.phase

    def inside(self, rid):
        s = self.robots[rid].state
        return {
            z
            for z, x, y, r in self.network.zones
            if hypot(s.x - x, s.y - y) <= r + s.collision_radius
        }

    def fixed_schedule(self, rid):
        r = self.robots[rid].state
        slots = []
        if rid in self.active:
            move, elapsed = self.active[rid]
            slots = [Slot(0, move, elapsed)]
        for slot in self.queues[rid]:
            if slot.time >= self.now - 1e-8:
                slots.append(Slot(slot.time - self.now, slot.move))
        # A disconnected robot already in a zone keeps its known exit route.
        if self.agents[rid].reason and self.inside(rid) and slots:
            node = slots[-1].move.end
            heading = slots[-1].move.heading
            time = slots[-1].end
            route = list(self.routes.get(rid, []))
            while route and route[0] == node:
                route.pop(0)
            for target in route:
                x, y, _ = self.network.nodes[node]
                if not any(
                    hypot(x - zx, y - zy) <= radius + r.collision_radius
                    for _, zx, zy, radius in self.network.zones
                ):
                    break
                move = next(
                    (
                        m
                        for m in self.network.neighbors(rid, node, heading)
                        if m.end == target
                    ),
                    None,
                )
                if move is None:
                    break
                slots.append(Slot(time, move))
                time += move.duration
                node = target
                heading = move.heading
        return Schedule((r.x, r.y, r.theta), slots)

    async def before_step(self, engine):
        self.now = engine.sim_time
        self.release_and_dispatch(engine)
        self.stations(engine)
        for rid, a in self.agents.items():
            if not self.connected:
                a.reason = "connection_lost"
                self.queues[rid] = []
        if self.connected and self.now + 1e-8 >= self.next_plan:
            free = {
                r: (a.node, self.robots[r].state.theta)
                for r, a in self.agents.items()
                if r not in self.active
                and r not in self.halted
                and self.goal(engine, r)
                and not (a.reason and self.inside(r))
            }
            if free:
                fixed = {
                    r: self.fixed_schedule(r) for r in self.agents if r not in free
                }
                p = TimedProblem(
                    self.network,
                    free,
                    {r: self.goal(engine, r) for r in free},
                    fixed,
                    {r: self.agents[r].waiting_s for r in free},
                    self.horizon,
                    self.time_limit,
                    int(self.solver.parameters.get("max_expansions", 100000)),
                    self.gap,
                )
                result = await asyncio.to_thread(self.solver.solve_timed, p)
                self.calls += 1
                self.timeouts += result.status == "timeout"
                self.partial_plans += result.status == "partial"
                self.max_solve_s = max(self.max_solve_s, result.elapsed_s)
                self.last_plan = dict(
                    status=result.status,
                    message=result.reason,
                    termination_reason=result.reason,
                    expanded=result.expanded,
                    elapsed_s=result.elapsed_s,
                )
                decisions = {}
                for rid in free:
                    a = self.agents[rid]
                    plan = result.plans.get(rid)
                    if plan is None:
                        a.reason = (
                            result.reason
                            if result.reason != "window_complete"
                            else "no_path_for_agent"
                        )
                        self.queues[rid] = []
                    else:
                        a.reason = None
                        a.wait_until = None
                        self.passes[rid].clear()
                        self.queues[rid] = [
                            Slot(self.now + s.time, s.move) for s in plan.slots
                        ]
                        # Retain the selected AMR branch, completing to the station geometrically.
                        end = plan.slots[-1].move.end if plan.slots else a.node
                        theta = plan.pose(plan.end)[2]
                        tail = self.network.route(rid, end, p.goals[rid], theta)
                        self.routes[rid] = [
                            s.move.end for s in plan.slots if s.move.start != s.move.end
                        ] + ([m.end for m in tail] if tail else [])
                        static = self.network.route(
                            rid, a.node, p.goals[rid], self.robots[rid].state.theta
                        )
                        delayed = not plan.slots or plan.slots[0].move.length == 0
                        rerouted = bool(
                            plan.slots
                            and static
                            and plan.slots[0].move.length
                            and plan.slots[0].move.end != static[0].end
                        )
                        if (
                            (delayed or rerouted)
                            and not (a.delayed or a.rerouting)
                            and static
                        ):
                            # Count a realised intervention only when the nominal move conflicts.
                            from .planning import Search

                            checker = Search(
                                TimedProblem(
                                    self.network,
                                    p.starts,
                                    p.goals,
                                    p.fixed,
                                    p.waiting,
                                    self.horizon,
                                    60,
                                    100000,
                                    self.gap,
                                ),
                                {},
                            )
                            nominal = Schedule(plan.pose0, [Slot(0, static[0])])
                            peers = {**fixed, **result.plans}
                            hit = next(
                                (
                                    h
                                    for other, schedule in peers.items()
                                    if other != rid
                                    and (
                                        h := checker.collision(
                                            rid, nominal, other, schedule, nominal.end
                                        )
                                    )
                                ),
                                None,
                            )
                            if hit:
                                a.vertex_conflicts += int(hit[1] == "vertex")
                                a.edge_conflicts += int(hit[1] == "edge")
                        a.delayed = delayed
                        a.rerouting = rerouted
                    decisions[rid] = dict(
                        action=(
                            "move"
                            if self.queues[rid] and self.queues[rid][0].move.length
                            else "wait"
                        ),
                        reason=a.reason or ("solver_wait" if a.delayed else "granted"),
                        control_mode="autonomous" if a.reason else "central",
                    )
                self.planned_paths = {
                    r: [
                        dict(
                            x=s.move.pose(k / 8 * s.move.duration)[0],
                            y=s.move.pose(k / 8 * s.move.duration)[1],
                            time=self.now + s.time + k / 8 * s.move.duration,
                        )
                        for s in plan.slots
                        for k in range(9)
                    ]
                    for r, plan in result.plans.items()
                }
                engine.recording.journal.append(
                    dict(
                        id=self.calls,
                        time=self.now,
                        solver=self.solver.name,
                        status=result.status,
                        message=result.reason,
                        elapsed_s=result.elapsed_s,
                        expanded=result.expanded,
                        cost=max((v.end for v in result.plans.values()), default=0)
                        / self.tick_s,
                        success=result.status == "solved",
                        metrics=result.metrics,
                        waiting_total_s=sum(a.waiting_s for a in self.agents.values()),
                        timeout_total=self.timeouts,
                        input=dict(
                            starts={r: n for r, (n, h) in free.items()},
                            goals=p.goals,
                            fixed_paths={},
                        ),
                        paths={
                            r: [s.move.end for s in v.slots]
                            for r, v in result.plans.items()
                        },
                        decisions=decisions,
                        robot_positions={
                            r: dict(x=b.state.x, y=b.state.y, theta=b.state.theta)
                            for r, b in self.robots.items()
                        },
                        reservations={},
                        execution_rejection=None,
                        mapf_snapshot=self.snapshot(),
                    )
                )
            self.next_plan = self.now + self.period * self.tick_s
        for rid, a in self.agents.items():
            if rid in self.active or rid in self.halted or not self.goal(engine, rid):
                continue
            if a.reason:
                self.offline(rid)
            elif self.queues[rid] and self.queues[rid][0].time <= self.now + 1e-8:
                slot = self.queues[rid].pop(0)
                self.active[rid] = (slot.move, 0.0)
        self.sync_mode()

    def offline(self, rid):
        a = self.agents[rid]
        route = self.routes.get(rid, [])
        while route and route[0] == a.node:
            route.pop(0)
        if not route:
            return  # AMR without a route waits; no local path invention.
        options = list(
            self.network.neighbors(rid, a.node, self.robots[rid].state.theta)
        )
        move = next((m for m in options if m.end == route[0]), None)
        if move is None:
            return
        touching = set()
        n = max(1, ceil(move.length / 0.25))
        for z, x, y, radius in self.network.zones:
            if any(
                hypot(
                    move.pose(i / n * move.duration)[0] - x,
                    move.pose(i / n * move.duration)[1] - y,
                )
                <= radius + self.robots[rid].state.collision_radius + 0.15
                for i in range(n + 1)
            ):
                touching.add(z)
        self.passes[rid].intersection_update(touching)
        self.passes[rid].update(self.inside(rid))
        if touching - self.passes[rid]:
            if a.wait_until is None:
                a.wait_until = self.now + 5
            if self.now + 1e-8 < a.wait_until:
                return
            self.passes[rid].update(touching)
            a.wait_until = None
        self.active[rid] = (move, 0.0)

    def update_robots(self, engine, dt):
        frozen = {
            r: (
                b.state.x,
                b.state.y,
                b.state.theta,
                b.state.width,
                b.state.collision_radius,
            )
            for r, b in self.robots.items()
        }
        for rid, robot in self.robots.items():
            robot.state.v = robot.state.omega = 0
            if rid not in self.active:
                continue
            move, elapsed = self.active[rid]
            step = min(dt, move.duration - elapsed)
            before = move.pose(elapsed)
            after = move.pose(elapsed + step)
            distance = hypot(after[0] - before[0], after[1] - before[1])
            if distance:
                for other, (x, y, theta, width, radius) in frozen.items():
                    if other == rid or cos(theta - before[2]) < 0.8:
                        continue
                    dx, dy = x - before[0], y - before[1]
                    if (
                        dx * cos(before[2]) + dy * sin(before[2]) <= 0
                        or abs(-dx * sin(before[2]) + dy * cos(before[2]))
                        > (width + robot.state.width) / 2
                    ):
                        continue
                    free = max(
                        0,
                        hypot(dx, dy)
                        - radius
                        - robot.state.collision_radius
                        - self.gap,
                    )
                    step = min(step, dt * min(1, free / (2 * distance)))
            if step < dt - 1e-6 and elapsed + step < move.duration - 1e-6:
                # Cancel unstarted commands when physical progress misses the schedule.
                for r in self.queues:
                    self.queues[r] = []
                self.next_plan = min(self.next_plan, self.now + self.tick_s)
            after = move.pose(elapsed + step)
            robot.state.x, robot.state.y, robot.state.theta = after
            robot.state.v = (
                move.distance(elapsed + step) - move.distance(elapsed)
            ) / dt
            from backend.robots.base import normalize_angle

            robot.state.omega = normalize_angle(after[2] - before[2]) / dt
            robot.state.mode = "route"
            self.active[rid] = (move, elapsed + step)
            if elapsed + step >= move.duration - 1e-8:
                self.agents[rid].node = move.end
                del self.active[rid]
                if self.network.robot_layers[rid] != "amr":
                    self.agents[rid].index = self.network.loops[
                        self.network.robot_layers[rid]
                    ].index(move.end)

    def after_step(self, engine):
        self.now = engine.sim_time + engine.settings.dt
        for r, a in self.agents.items():
            dt = engine.settings.dt
            if a.reason:
                a.autonomous_s += dt
            if a.wait_until is not None:
                a.junction_wait_s += dt
            if (
                abs(self.robots[r].state.v) < 1e-8
                and abs(self.robots[r].state.omega) < 1e-8
            ):
                a.idle_s += dt
                if a.phase in ("to_pickup", "to_dropoff"):
                    a.waiting_s += dt
            blocked = (
                abs(self.robots[r].state.v) < 1e-8
                and abs(self.robots[r].state.omega) < 1e-8
                and a.phase in ("to_pickup", "to_dropoff")
            )
            if blocked and not a.blocked:
                a.delay_episodes += 1
            a.blocked = blocked
            a.conflicts = a.vertex_conflicts + a.edge_conflicts + a.delay_episodes
        self.sync_mode()

    def snapshot(self):
        return dict(
            temporal=True,
            solver=self.solver.name,
            connected=self.connected,
            control_mode="autonomous" if self.autonomous_reason else "central",
            autonomous_reason=self.autonomous_reason,
            planning_tick_s=self.tick_s,
            horizon_steps=self.horizon,
            window_s=self.horizon * self.tick_s,
            replan_s=self.period * self.tick_s,
            calls=self.calls,
            timeouts=self.timeouts,
            partial_plans=self.partial_plans,
            max_solve_s=self.max_solve_s,
            last_plan=self.last_plan,
            active_moves=list(self.active),
            halted=list(self.halted),
            reservations={},
            planned_paths=self.planned_paths,
            agents={
                r: dict(
                    route_id=a.route_id,
                    index=a.index,
                    node=a.node,
                    phase=a.phase,
                    delivered=a.delivered,
                    waiting_s=a.waiting_s,
                    autonomous_s=a.autonomous_s,
                    junction_wait_s=a.junction_wait_s,
                    control_reason=a.reason,
                    junction_wait_remaining_s=max(0, (a.wait_until or 0) - self.now),
                    delay_episodes=a.delay_episodes,
                    vertex_conflicts=a.vertex_conflicts,
                    edge_conflicts=a.edge_conflicts,
                    conflict_episodes=a.conflicts,
                )
                for r, a in self.agents.items()
            },
        )
