"""Discretize straight graph edges without changing the ten fixed loops."""
from __future__ import annotations

from dataclasses import dataclass
from math import ceil, hypot

from backend.maps.geojson_graph import GeoJsonRouteGraph, GraphEdge, GraphNode
from .base import ResourceUse


@dataclass
class FixedLoop:
    id: str
    nodes: list[str]
    resources: list[tuple[ResourceUse, ...]]


class LoopNetwork:
    def __init__(self, source: GeoJsonRouteGraph, cell_size_m: float = 5.0, zone_radius_m: float = 3.0,
                 footprint_radius_m: float = 0.81):
        if cell_size_m <= 0 or zone_radius_m <= 0:
            raise ValueError("Cell size and zone radius must be positive")
        self.graph = GeoJsonRouteGraph()
        self.loops = {}
        subdivided = {}
        for edge in source.edges.values():
            a,b = source.nodes[edge.start],source.nodes[edge.end]
            length = hypot(b.x-a.x,b.y-a.y)
            if abs(edge.length()-length) > 1e-6 or length == 0:
                raise ValueError("Fixed-loop edges must be straight and nonzero; add nodes at turns")
            n = max(1,ceil(length/cell_size_m))
            nodes = [edge.start]+[f'{edge.id}:{i}' for i in range(1,n)]+[edge.end]
            for i,node in enumerate(nodes):
                self.graph.nodes[node] = GraphNode(node,a.x+(b.x-a.x)*i/n,a.y+(b.y-a.y)*i/n)
            for i,(start,end) in enumerate(zip(nodes,nodes[1:])):
                u,v = self.graph.nodes[start],self.graph.nodes[end]
                eid = f'{edge.id}:{i}'
                self.graph.edges[eid] = GraphEdge(eid,start,end,[(u.x,u.y),(v.x,v.y)],edge.bidirectional)
            subdivided[edge.id] = nodes
        for raw in source.routes:
            sequence = raw['node_ids']
            if len(sequence) < 4 or sequence[0] != sequence[-1]:
                raise ValueError(f"Route {raw['route_id']} must be a closed loop")
            nodes, outgoing = [],[]
            for start,end in zip(sequence,sequence[1:]):
                edge = source.find_edge_between(start,end)
                if edge is None:
                    raise ValueError(f"Illegal directed edge {start} -> {end}")
                direction = 1 if start == edge.start else -1
                parts = subdivided[edge.id] if direction == 1 else list(reversed(subdivided[edge.id]))
                nodes.extend(parts[:-1])
                outgoing.extend([ResourceUse('corridor:'+edge.id,direction)]*(len(parts)-1))
            resources = []
            for i,node in enumerate(nodes):
                p = self.graph.nodes[node]
                uses = {outgoing[i-1],outgoing[i]}
                # Include the complete footprint in zone occupancy, also at corners.
                for junction in source.nodes.values():
                    if hypot(p.x-junction.x,p.y-junction.y) <= zone_radius_m+footprint_radius_m:
                        uses.add(ResourceUse('zone:'+junction.id))
                resources.append(tuple(sorted(uses,key=lambda r:(r.key,r.direction))))
            rid = raw['route_id']
            if rid in self.loops:
                raise ValueError(f"Duplicate loop: {rid}")
            self.loops[rid] = FixedLoop(rid,nodes,resources)
        if not self.loops:
            raise ValueError("Graph has no fixed routes metadata")
