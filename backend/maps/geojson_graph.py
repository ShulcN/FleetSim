from __future__ import annotations

import heapq
import json
from dataclasses import asdict, dataclass, field
from math import hypot
from pathlib import Path
from typing import Any

from backend.robots.controllers import Waypoint


@dataclass
class GraphNode:
    id: str
    x: float
    y: float
    properties: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class GraphEdge:
    id: str
    start: str
    end: str
    coordinates: list[tuple[float, float]]
    bidirectional: bool = True
    cost: float | None = None
    properties: dict[str, Any] = field(default_factory=dict)

    def length(self) -> float:
        if len(self.coordinates) < 2:
            return 0.0
        return sum(
            hypot(self.coordinates[i + 1][0] - self.coordinates[i][0], self.coordinates[i + 1][1] - self.coordinates[i][1])
            for i in range(len(self.coordinates) - 1)
        )

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "start": self.start,
            "end": self.end,
            "coordinates": self.coordinates,
            "bidirectional": self.bidirectional,
            "cost": self.cost if self.cost is not None else self.length(),
            "properties": self.properties,
        }


@dataclass
class GeoJsonRouteGraph:
    nodes: dict[str, GraphNode] = field(default_factory=dict)
    edges: dict[str, GraphEdge] = field(default_factory=dict)
    source: str | None = None

    def to_dict(self) -> dict:
        return {
            "source": self.source,
            "nodes": [n.to_dict() for n in self.nodes.values()],
            "edges": [e.to_dict() for e in self.edges.values()],
        }


    def find_edge_between(self, start_id: str, end_id: str) -> GraphEdge | None:
        for edge in self.edges.values():
            if edge.start == start_id and edge.end == end_id:
                return edge
            if edge.bidirectional and edge.start == end_id and edge.end == start_id:
                return edge
        return None

    def nearest_node(self, x: float, y: float) -> str | None:
        if not self.nodes:
            return None
        return min(self.nodes.values(), key=lambda n: hypot(n.x - x, n.y - y)).id

    def shortest_path(self, start_id: str, goal_id: str) -> list[str]:
        if start_id not in self.nodes:
            raise ValueError(f"Unknown start node: {start_id}")
        if goal_id not in self.nodes:
            raise ValueError(f"Unknown goal node: {goal_id}")
        if start_id == goal_id:
            return [start_id]

        adj: dict[str, list[tuple[str, float]]] = {node_id: [] for node_id in self.nodes}
        for edge in self.edges.values():
            cost = float(edge.cost if edge.cost is not None else edge.length())
            adj.setdefault(edge.start, []).append((edge.end, cost))
            if edge.bidirectional:
                adj.setdefault(edge.end, []).append((edge.start, cost))

        dist = {start_id: 0.0}
        parent: dict[str, str] = {}
        heap = [(0.0, start_id)]

        while heap:
            current_dist, node = heapq.heappop(heap)
            if node == goal_id:
                break
            if current_dist != dist.get(node):
                continue
            for nxt, cost in adj.get(node, []):
                nd = current_dist + cost
                if nd < dist.get(nxt, float("inf")):
                    dist[nxt] = nd
                    parent[nxt] = node
                    heapq.heappush(heap, (nd, nxt))

        if goal_id not in dist:
            raise ValueError(f"No path from {start_id} to {goal_id}")

        path = [goal_id]
        while path[-1] != start_id:
            path.append(parent[path[-1]])
        path.reverse()
        return path

    def nodes_to_waypoints(self, node_ids: list[str]) -> list[Waypoint]:
        return [Waypoint(self.nodes[node_id].x, self.nodes[node_id].y) for node_id in node_ids]


def load_geojson_graph(path: str | Path) -> GeoJsonRouteGraph:
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    graph = GeoJsonRouteGraph(source=str(path))

    for index, feature in enumerate(data.get("features", [])):
        geometry = feature.get("geometry") or {}
        props = feature.get("properties") or {}
        geom_type = geometry.get("type")
        feature_type = str(props.get("type", "")).lower()

        if geom_type == "Point" and (feature_type in {"node", "station", "waypoint"} or "id" in props):
            coords = geometry.get("coordinates", [0.0, 0.0])
            node_id = str(props.get("id") or props.get("node_id") or f"N{index}")
            graph.nodes[node_id] = GraphNode(
                id=node_id,
                x=float(coords[0]),
                y=float(coords[1]),
                properties=props,
            )

    for index, feature in enumerate(data.get("features", [])):
        geometry = feature.get("geometry") or {}
        props = feature.get("properties") or {}
        geom_type = geometry.get("type")
        feature_type = str(props.get("type", "")).lower()

        if geom_type not in {"LineString", "MultiLineString"}:
            continue
        if feature_type and feature_type not in {"edge", "route", "path"}:
            continue

        edge_id = str(props.get("id") or props.get("edge_id") or f"E{index}")
        start = str(props.get("startid") or props.get("start") or props.get("from") or "")
        end = str(props.get("endid") or props.get("end") or props.get("to") or "")
        direction = str(props.get("direction", "bidirectional")).lower()
        bidirectional = direction not in {"oneway", "one_way", "forward"}

        coordinates: list[tuple[float, float]] = []
        if geom_type == "LineString":
            coordinates = [(float(x), float(y)) for x, y, *_ in geometry.get("coordinates", [])]
        elif geom_type == "MultiLineString":
            for line in geometry.get("coordinates", []):
                coordinates.extend((float(x), float(y)) for x, y, *_ in line)

        if not start or not end:
            if coordinates and graph.nodes:
                start = graph.nearest_node(*coordinates[0]) or ""
                end = graph.nearest_node(*coordinates[-1]) or ""

        if start and end:
            graph.edges[edge_id] = GraphEdge(
                id=edge_id,
                start=start,
                end=end,
                coordinates=coordinates,
                bidirectional=bidirectional,
                cost=float(props["cost"]) if "cost" in props else None,
                properties=props,
            )

    return graph
