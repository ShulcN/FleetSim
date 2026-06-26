import pytest

from backend.maps.geojson_graph import GeoJsonRouteGraph, GraphEdge, GraphNode
from backend.robots.controllers import Waypoint


def line_graph():
    graph = GeoJsonRouteGraph()
    graph.nodes = {
        "A": GraphNode(id="A", x=0.0, y=0.0),
        "B": GraphNode(id="B", x=1.0, y=0.0),
        "C": GraphNode(id="C", x=2.0, y=0.0),
    }
    graph.edges = {
        "AB": GraphEdge(id="AB", start="A", end="B", coordinates=[(0.0, 0.0), (1.0, 0.0)]),
        "BC": GraphEdge(id="BC", start="B", end="C", coordinates=[(1.0, 0.0), (2.0, 0.0)]),
    }
    return graph


def test_shortest_path():
    graph = line_graph()
    assert graph.shortest_path("A", "C") == ["A", "B", "C"]
    assert graph.shortest_path("A", "A") == ["A"]


def test_nearest_node():
    graph = line_graph()
    assert graph.nearest_node(0.1, 0.0) == "A"
    assert graph.nearest_node(1.9, 0.0) == "C"


def test_nodes_to_waypoints():
    graph = line_graph()
    waypoints = graph.nodes_to_waypoints(["A", "B"])
    assert waypoints == [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0)]


def test_unknown_node_raises():
    graph = line_graph()
    with pytest.raises(ValueError):
        graph.shortest_path("A", "Z")
