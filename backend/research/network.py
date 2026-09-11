"""Two navigation layers sharing metric coordinates and physical obstacles."""

from math import atan2, hypot, ceil
from heapq import heappop, heappush
from .motion import rounded_loop, line, Piece, motion, WallIndex
from backend.robots.base import normalize_angle


class Navigation:
    def __init__(self, graph, amr_graph, robots, config, dxf=None):
        self.nodes = {}
        self.edges = {}
        self.reverse = {}
        self.loops = {}
        self.robot_layers = {}
        self.walls = WallIndex(dxf, config.get("collision_layers"))
        self.zones = [
            (n.id, n.x, n.y, float(n.properties.get("control_radius_m", 3)))
            for n in graph.nodes.values()
            if n.properties.get("junction_id")
        ]
        self.zones.extend(
            (z["id"], float(z["x"]), float(z["y"]), float(z["radius"]))
            for z in config.get("zones", [])
        )
        self.corridors = []
        for e in graph.edges.values():
            for i, (a, b) in enumerate(zip(e.coordinates, e.coordinates[1:])):
                length = hypot(b[0] - a[0], b[1] - a[1])
                if length > 2:
                    self.corridors.append(
                        (
                            e.id + ":" + str(i),
                            a,
                            b,
                            float(e.properties.get("corridor_half_width_m", 0.6)),
                        )
                    )
        self.tick_s = float(config.get("planning_tick_s", 0.5))
        self._motion_cache = {}
        self._distance_cache = {}
        self._clear_cache = {}
        self._route_cache = {}
        self.robots = robots
        routes = {r["route_id"]: r["node_ids"][:-1] for r in graph.routes}
        for assignment in config.get("assignments", []):
            rid = assignment["robot_id"]
            robot = robots[rid]
            route = assignment["route_id"]
            radius = robot.kinematics.min_turn_radius
            key = f"agv:{route}:{radius}"
            if key not in self.loops:
                points = [(graph.nodes[n].x, graph.nodes[n].y) for n in routes[route]]
                pieces = rounded_loop(
                    points, radius, float(config.get("cell_size_m", 3))
                )
                names = [f"{key}:{i}" for i in range(len(pieces))]
                for i, p in enumerate(pieces):
                    self.nodes[names[i]] = p.pose(0)
                    self.add_edge(names[i], names[(i + 1) % len(names)], (p,))
                self.loops[key] = names
            self.robot_layers[rid] = key
        if amr_graph:
            for nid, node in amr_graph.nodes.items():
                self.nodes["amr:" + nid] = (node.x, node.y, 0)
            for edge in amr_graph.edges.values():
                coordinates = edge.coordinates
                if (
                    hypot(
                        coordinates[0][0] - amr_graph.nodes[edge.start].x,
                        coordinates[0][1] - amr_graph.nodes[edge.start].y,
                    )
                    > 1e-5
                ):
                    raise ValueError("AMR edge geometry/start mismatch")
                if (
                    hypot(
                        coordinates[-1][0] - amr_graph.nodes[edge.end].x,
                        coordinates[-1][1] - amr_graph.nodes[edge.end].y,
                    )
                    > 1e-5
                ):
                    raise ValueError("AMR edge geometry/end mismatch")
                pieces = tuple(
                    line(a, b) for a, b in zip(coordinates, coordinates[1:]) if a != b
                )
                if pieces:
                    self.add_edge("amr:" + edge.start, "amr:" + edge.end, pieces)
                if edge.bidirectional and pieces:
                    self.add_edge(
                        "amr:" + edge.end,
                        "amr:" + edge.start,
                        tuple(
                            line(b, a)
                            for a, b in reversed(
                                list(zip(coordinates, coordinates[1:]))
                            )
                        ),
                    )
        for rid, robot in robots.items():
            if robot.state.parameters.get("robot_class") == "amr":
                if not amr_graph:
                    raise ValueError("AMR fleet requires amr_graph_asset")
                self.robot_layers[rid] = "amr"
            elif rid not in self.robot_layers:
                raise ValueError(f"{rid}: AGV requires a fixed-loop assignment")
        for rid, layer in self.robot_layers.items():
            if layer == "amr":
                continue
            for a in self.loops[layer]:
                for b, pieces in self.edges[a]:
                    if not self.clear(rid, a, b, pieces):
                        raise ValueError(
                            f"{rid}: rounded AGV path does not fit obstacles at {a}"
                        )

    def add_edge(self, a, b, pieces):
        self.edges.setdefault(a, []).append((b, pieces))
        self.reverse.setdefault(b, []).append((a, sum(p.length for p in pieces)))

    def clear(self, rid, a, b, pieces):
        radius = self.robots[rid].state.collision_radius
        key = (radius, a, b)
        if key not in self._clear_cache:
            self._clear_cache[key] = all(
                self.walls.piece_clear(p, radius) for p in pieces
            )
        return self._clear_cache[key]

    def neighbors(self, rid, node, heading):
        robot = self.robots[rid]
        speed = robot.route_follower.config.max_linear
        for end, pieces in self.edges.get(node, []):
            if not self.clear(rid, node, end, pieces):
                continue
            key = (rid, node, end, round(heading, 7))
            if key not in self._motion_cache:
                output = []
                theta = heading
                for p in pieces:
                    delta = normalize_angle(p.theta - theta)
                    if abs(delta) > 1e-6:
                        if robot.state.parameters["robot_class"] == "agv":
                            if abs(delta) > 0.01:
                                raise ValueError("AGV path heading discontinuity")
                        else:
                            output.append(Piece(p.x, p.y, theta, 0, spin=delta))
                    output.append(p)
                    theta = p.pose(1)[2]
                self._motion_cache[key] = motion(
                    node, end, output, speed, robot.kinematics.max_angular, self.tick_s
                )
            yield self._motion_cache[key]

    def wait(self, node, heading):
        x, y, _ = self.nodes[node]
        return motion(node, node, [Piece(x, y, heading, 0)], 1, 1, self.tick_s)

    def distances(self, goal):
        """Static reverse Dijkstra abstraction reused by windowed space-time A*."""
        if goal not in self._distance_cache:
            dist = {goal: 0}
            heap = [(0, goal)]
            while heap:
                d, n = heappop(heap)
                if d != dist[n]:
                    continue
                for prev, cost in self.reverse.get(n, []):
                    nd = d + cost
                    if nd < dist.get(prev, float("inf")):
                        dist[prev] = nd
                        heappush(heap, (nd, prev))
            self._distance_cache[goal] = dist
        return self._distance_cache[goal]

    def route(self, rid, start, goal, heading):
        key = (rid, start, goal, round(normalize_angle(heading), 6))
        if key not in self._route_cache:
            if len(self._route_cache) > 8192:
                self._route_cache.clear()
            result = self._route(rid, start, goal, heading)
            self._route_cache[key] = tuple(result) if result is not None else None
        result = self._route_cache[key]
        return list(result) if result is not None else None

    def _route(self, rid, start, goal, heading):
        """Static route retained for offline execution; physical reachability checked."""
        dist = self.distances(goal)
        speed = self.robots[rid].route_follower.config.max_linear
        from itertools import count

        serial = count()
        key = lambda n, h: (n, round(normalize_angle(h), 6))
        heap = [
            (dist.get(start, float("inf")) / speed, 0, next(serial), start, heading, [])
        ]
        best = {key(start, heading): 0}
        while heap:
            _, cost, _, node, theta, path = heappop(heap)
            if node == goal:
                return path
            if cost != best[key(node, theta)]:
                continue
            for move in self.neighbors(rid, node, theta):
                nd = cost + move.duration
                k = key(move.end, move.heading)
                if nd < best.get(k, float("inf")):
                    best[k] = nd
                    heappush(
                        heap,
                        (
                            nd + dist.get(move.end, float("inf")) / speed,
                            nd,
                            next(serial),
                            move.end,
                            move.heading,
                            path + [move],
                        ),
                    )
        return None
