from __future__ import annotations

import asyncio
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from backend.collisions import CollisionChecker, CollisionMode
from backend.maps.dxf_map import DxfMap, load_dxf_map
from backend.maps.geojson_graph import GeoJsonRouteGraph, load_geojson_graph
from backend.metrics import MetricsCollector
from backend.robots.base import RobotBase
from backend.robots.commands import ControlCommand
from backend.robots.controllers import Waypoint
from backend.wms.fifo_validator import FifoValidator
from backend.wms.orders import Order, OrderStatus, WmsScenario
from backend.comms import FleetCommLink
from .world_state import WorldConfig, WorldSnapshot


@dataclass
class SimulationSettings:
    dt: float = 0.05  # 20 Hz simulation
    realtime_factor: float = 1.0
    max_sim_time: float = 120.0
    collision_mode: CollisionMode = CollisionMode.COUNT_ONLY


class SimulationEngine:
    """Simulation core."""

    def __init__(self, world: WorldConfig | None = None, settings: SimulationSettings | None = None):
        self.world = world or WorldConfig()
        self.settings = settings or SimulationSettings()
        self.robots: dict[str, RobotBase] = {}
        self.sim_time = 0.0
        self.status = "stopped"  # stopped | running | paused | finished | collision_stopped
        self.dxf_map: DxfMap | None = None
        self.graph: GeoJsonRouteGraph | None = None
        self.wms = WmsScenario()
        self.metrics = MetricsCollector()
        self.fifo = FifoValidator()
        self.collision_checker = CollisionChecker()
        self.last_collisions: list[dict] = []
        self.map_id = "factory_map"

        self.use_comms = True
        self.comms = FleetCommLink()
        
        self._lock = asyncio.Lock()

    async def reset(
        self,
        robots: Iterable[RobotBase] | None = None,
        dxf_map: DxfMap | None = None,
        graph: GeoJsonRouteGraph | None = None,
        wms: WmsScenario | None = None,
        settings: SimulationSettings | None = None,
    ) -> None:
        async with self._lock:
            if settings is not None:
                self.settings = settings
            self.robots = {robot.state.id: robot for robot in (robots or [])}
            self.dxf_map = dxf_map
            self.graph = graph
            self.wms = wms or WmsScenario()
            self.metrics = MetricsCollector()
            self.fifo = FifoValidator()
            self.collision_checker = CollisionChecker()
            self.last_collisions = []
            self.sim_time = 0.0
            self.status = "stopped"
            await self._restart_comms()
            self._update_world_size_from_map_or_graph()
            

    def _apply_order_from_comms(self, robot_id: str | None, waypoints: list) -> None:
        robot = self.robots.get(robot_id) if robot_id else None
        if robot is not None:
            robot.set_route(waypoints)

    def _send_route(self, robot: RobotBase, order_id: str, waypoints: list) -> None:
        if self.use_comms:
            self.comms.send_order(robot.state.id, order_id, waypoints)
        else:
            robot.set_route(waypoints)


    async def configure_from_files(
        self,
        fleet_robots: Iterable[RobotBase],
        map_dxf_path: str | Path | None,
        graph_geojson_path: str | Path | None,
        scenario_json_path: str | Path | None,
        max_sim_time: float,
        collision_mode: CollisionMode,
    ) -> None:
        dxf_map = load_dxf_map(map_dxf_path) if map_dxf_path else None
        graph = load_geojson_graph(graph_geojson_path) if graph_geojson_path else None
        wms = WmsScenario.from_file(scenario_json_path) if scenario_json_path else WmsScenario()
        await self.reset(
            robots=fleet_robots,
            dxf_map=dxf_map,
            graph=graph,
            wms=wms,
            settings=SimulationSettings(
                dt=self.settings.dt,
                realtime_factor=self.settings.realtime_factor,
                max_sim_time=max_sim_time,
                collision_mode=collision_mode,
            ),
        )

    async def _restart_comms(self) -> None:
        await self.comms.stop()
        if self.use_comms and self.robots:
            await self.comms.start(self.robots.values(), self._apply_order_from_comms)


    async def add_robot(self, robot: RobotBase) -> None:
        async with self._lock:
            self.robots[robot.state.id] = robot

    async def add_robots(self, robots: Iterable[RobotBase]) -> None:
        async with self._lock:
            for robot in robots:
                self.robots[robot.state.id] = robot

    async def start(self) -> None:
        async with self._lock:
            if self.status in {"stopped", "paused"}:
                self.status = "running"

    async def pause(self) -> None:
        async with self._lock:
            if self.status == "running":
                self.status = "paused"

    async def resume(self) -> None:
        async with self._lock:
            if self.status == "paused":
                self.status = "running"

    async def stop(self) -> None:
        async with self._lock:
            self.status = "stopped"
            for robot in self.robots.values():
                robot.stop()

    async def set_robot_command(self, robot_id: str, command: ControlCommand) -> bool:
        async with self._lock:
            robot = self.robots.get(robot_id)
            if robot is None:
                return False
            robot.set_command(command)
            return True

    async def set_robot_route(self, robot_id: str, waypoints: list[Waypoint]) -> bool:
        async with self._lock:
            robot = self.robots.get(robot_id)
            if robot is None:
                return False
            robot.set_route(waypoints)
            return True

    async def set_robot_route_by_nodes(self, robot_id: str, node_ids: list[str]) -> bool:
        async with self._lock:
            robot = self.robots.get(robot_id)
            if robot is None or self.graph is None:
                return False
            if any(node_id not in self.graph.nodes for node_id in node_ids):
                return False
            robot.set_route(self.graph.nodes_to_waypoints(node_ids))
            return True

    async def stop_robot(self, robot_id: str) -> bool:
        async with self._lock:
            robot = self.robots.get(robot_id)
            if robot is None:
                return False
            robot.stop()
            return True

    async def step(self) -> None:
        dt = self.settings.dt
        async with self._lock:
            if self.status != "running":
                return
            if self.sim_time >= self.settings.max_sim_time:
                self.status = "finished"
                return

            self._release_due_orders()
            self._dispatch_pending_orders()

            for robot in self.robots.values():
                robot.update(dt)
                self._keep_robot_inside_world(robot)

            self._handle_route_completions()

            collision_events = self.collision_checker.check(
                list(self.robots.values()), self.dxf_map, self.sim_time
            )
            self.last_collisions = [event.to_dict() for event in collision_events]
            if collision_events:
                self.metrics.record_collisions(collision_events)
                if self.settings.collision_mode == CollisionMode.STOP_ON_COLLISION:
                    self.status = "collision_stopped"
                    for event in collision_events:
                        robot = self.robots.get(event.robot_id)
                        if robot is not None:
                            robot.set_collision_status()
                    # For robot-robot collisions, mark the second robot too.
                    for event in collision_events:
                        if event.type == "robot_robot" and event.other_id in self.robots:
                            self.robots[event.other_id].set_collision_status()

            self.metrics.observe_robots([robot.snapshot() for robot in self.robots.values()])
            self.sim_time += dt

    def _release_due_orders(self) -> None:
        for order in self.wms.release_due_orders(self.sim_time):
            self.fifo.register_released(order)
            self.metrics.record_order_event(self.sim_time, "released", order)

    def _dispatch_pending_orders(self) -> None:
        if self.graph is None:
            return

        for order in self.wms.pending_orders():
            robot = self._select_robot_for_order(order)
            if robot is None:
                continue

            start_node = self.graph.nearest_node(robot.state.x, robot.state.y)
            if start_node is None:
                continue

            try:
                node_path = self.graph.shortest_path(start_node, order.pickup_node)
            except ValueError as exc:
                order.status = OrderStatus.FAILED
                self.metrics.record_order_event(self.sim_time, f"failed: {exc}", order)
                continue

            order.status = OrderStatus.ASSIGNED
            order.assigned_robot = robot.state.id
            order.assigned_at = self.sim_time
            robot.state.active_order_id = order.id
            robot.state.cargo_type = order.cargo_type
            robot.state.target_node = order.pickup_node
            robot.state.status = "to_pickup"

            robot.set_route(self.graph.nodes_to_waypoints(node_path))
            self.metrics.record_order_event(self.sim_time, "assigned", order, robot.state.id)

    def _select_robot_for_order(self, order: Order) -> RobotBase | None:
        allowed = set(order.eligible_robots)
        candidates = []
        for robot in self.robots.values():
            if not robot.is_idle:
                continue
            if allowed and robot.state.id not in allowed:
                continue
            candidates.append(robot)

        if not candidates:
            return None

        if self.graph is None or order.pickup_node not in self.graph.nodes:
            return candidates[0]

        target = self.graph.nodes[order.pickup_node]
        return min(candidates, key=lambda r: (r.state.x - target.x) ** 2 + (r.state.y - target.y) ** 2)

    def _handle_route_completions(self) -> None:
        if self.graph is None:
            return

        for robot in self.robots.values():
            if not robot.route_completed():
                continue
            robot.acknowledge_route_completed()

            order_id = robot.state.active_order_id
            if order_id is None:
                robot.clear_task()
                continue

            order = self.wms.get_order(order_id)
            if order is None:
                robot.clear_task()
                continue

            if robot.state.status == "to_pickup":
                order.status = OrderStatus.PICKED
                order.picked_at = self.sim_time
                robot.state.status = "to_dropoff"
                robot.state.target_node = order.dropoff_node
                self.metrics.record_order_event(self.sim_time, "picked", order, robot.state.id)

                try:
                    node_path = self.graph.shortest_path(order.pickup_node, order.dropoff_node)
                except ValueError as exc:
                    order.status = OrderStatus.FAILED
                    self.metrics.record_order_event(self.sim_time, f"failed: {exc}", order, robot.state.id)
                    robot.clear_task()
                    continue

                robot.set_route(self.graph.nodes_to_waypoints(node_path))

            elif robot.state.status == "to_dropoff":
                order.status = OrderStatus.DELIVERED
                order.delivered_at = self.sim_time
                self.metrics.record_order_event(self.sim_time, "delivered", order, robot.state.id)
                violation = self.fifo.register_delivered(order, self.sim_time)
                self.metrics.record_fifo_violation(violation)
                robot.clear_task()

    def _keep_robot_inside_world(self, robot: RobotBase) -> None:
        s = robot.state
        margin = 0.2
        s.x = max(margin, min(self.world.width_m - margin, s.x))
        s.y = max(margin, min(self.world.height_m - margin, s.y))

    def _update_world_size_from_map_or_graph(self) -> None:
        min_x = min_y = 0.0
        max_x = self.world.width_m
        max_y = self.world.height_m

        bounds = self.dxf_map.bounds if self.dxf_map is not None else None
        if bounds:
            min_x = min(min_x, float(bounds.get("min_x", 0.0)))
            min_y = min(min_y, float(bounds.get("min_y", 0.0)))
            max_x = max(max_x, float(bounds.get("max_x", max_x)))
            max_y = max(max_y, float(bounds.get("max_y", max_y)))

        if self.graph is not None and self.graph.nodes:
            xs = [n.x for n in self.graph.nodes.values()]
            ys = [n.y for n in self.graph.nodes.values()]
            min_x = min(min_x, min(xs))
            min_y = min(min_y, min(ys))
            max_x = max(max_x, max(xs))
            max_y = max(max_y, max(ys))

        self.world.width_m = max(5.0, max_x - min(0.0, min_x) + 1.0)
        self.world.height_m = max(5.0, max_y - min(0.0, min_y) + 1.0)

    async def snapshot(self) -> dict:
        async with self._lock:
            metrics = self.metrics.to_dict(self.sim_time)
            summary = {
                "collision_count": metrics["collision_count"],
                "orders_delivered": metrics["orders_delivered"],
                "fifo_violation_count": metrics["fifo_violation_count"],
            }
            return WorldSnapshot(
                time=self.sim_time,
                width_m=self.world.width_m,
                height_m=self.world.height_m,
                status=self.status,
                collision_mode=str(self.settings.collision_mode),
                max_sim_time=self.settings.max_sim_time,
                robots=[robot.snapshot() for robot in self.robots.values()],
                map=self.dxf_map.to_dict() if self.dxf_map is not None else None,
                graph=self.graph.to_dict() if self.graph is not None else None,
                wms=self.wms.to_dict(),
                metrics_summary=summary,
                last_collisions=self.last_collisions,
            ).to_dict()

    async def export_metrics(self) -> dict:
        async with self._lock:
            return {
                "status": self.status,
                "settings": {
                    "dt": self.settings.dt,
                    "realtime_factor": self.settings.realtime_factor,
                    "max_sim_time": self.settings.max_sim_time,
                    "collision_mode": str(self.settings.collision_mode),
                },
                "metrics": self.metrics.to_dict(self.sim_time),
                "fifo": self.fifo.to_dict(),
                "orders": self.wms.to_dict()["orders"],
            }
