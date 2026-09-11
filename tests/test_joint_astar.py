import pytest

from backend.mapf import (
    MAPFProblem,
    JointAStarSolver,
    ResourceUse,
    create_solver,
    register_solver,
)
from backend.maps.geojson_graph import GeoJsonRouteGraph, GraphNode, GraphEdge


def problem_for(coordinates, paths, **kwargs):
    graph = GeoJsonRouteGraph()
    for name, xy in coordinates.items():
        graph.nodes[name] = GraphNode(name, *xy)
    for path in paths.values():
        for a, b in zip(path, path[1:]):
            if a != b:
                graph.edges[a + b] = GraphEdge(
                    a + b, a, b, [coordinates[a], coordinates[b]]
                )
    return MAPFProblem(
        graph,
        {r: p[0] for r, p in paths.items()},
        {r: p[-1] for r, p in paths.items()},
        fixed_paths=paths,
        clearance=0,
        **kwargs,
    )


def crossing():
    return problem_for(
        dict(A=(-10, 0), O=(0, 0), B=(10, 0), C=(0, -10), D=(0, 10)),
        dict(R1=["A", "O", "B"], R2=["C", "O", "D"]),
    )


def test_joint_astar_crossing_requires_waits_and_optimal_four_rounds():
    result = JointAStarSolver().solve(crossing())
    assert result.success
    assert result.cost == 4  # Crossing remains reserved through the exit movement.
    assert any(a == b for path in result.paths.values() for a, b in zip(path, path[1:]))
    for t in range(len(result.paths["R1"]) - 1):
        a, b = result.paths["R1"][t : t + 2]
        c, d = result.paths["R2"][t : t + 2]
        assert b != d and not (a == d and b == c)
        assert not ("O" in (a, b) and "O" in (c, d))


def test_joint_astar_prevents_head_on_swap():
    p = problem_for(dict(A=(0, 0), B=(10, 0)), dict(R1=["A", "B"], R2=["B", "A"]))
    result = JointAStarSolver().solve(p)
    assert not result.success and result.status == "infeasible"
    assert result.paths == dict(R1=["A"], R2=["B"])


def test_exclusive_zone_beyond_geometric_intersection():
    p = problem_for(
        dict(A=(0, 0), B=(5, 0), C=(0, 20), D=(5, 20)),
        dict(R1=["A", "B"], R2=["C", "D"]),
    )
    # Disjoint geometry still cannot share an exclusive intersection zone.
    zone = (ResourceUse("zone"),)
    p.resources = dict(R1=[(), zone], R2=[(), zone])
    result = JointAStarSolver().solve(p)
    assert not result.success


def test_same_direction_corridor_allows_separated_robots_but_not_opposing():
    p = problem_for(
        dict(A=(0, 0), B=(5, 0), C=(20, 0), D=(25, 0)),
        dict(R1=["A", "B"], R2=["C", "D"]),
    )
    forward = (ResourceUse("corridor", 1),)
    p.resources = dict(R1=[forward, forward], R2=[forward, forward])
    assert JointAStarSolver().solve(p).success
    reverse = (ResourceUse("corridor", -1),)
    p.resources["R2"] = [reverse, reverse]
    assert JointAStarSolver().solve(p).status == "infeasible"


def test_zero_budget_returns_no_unauthorized_movement():
    p = crossing()
    p.time_limit_s = 0
    result = JointAStarSolver().solve(p)
    assert result.status == "timeout"
    assert result.paths == {r: [n] for r, n in p.starts.items()}
    assert result.elapsed_s < 0.05


def test_expansion_limit_returns_only_a_complete_safe_joint_prefix():
    p = crossing()
    p.max_expansions = 3
    result = JointAStarSolver().solve(p)
    assert result.status == "partial" and not result.success
    assert set(map(len, result.paths.values())) == {2}
    assert sum(path[0] != path[1] for path in result.paths.values()) == 1


def test_invalid_edges_and_generic_constraints_are_rejected():
    p = crossing()
    p.graph.edges.clear()
    with pytest.raises(ValueError, match="illegal edge"):
        JointAStarSolver().solve(p)
    p = crossing()
    p.constraints = [object()]
    with pytest.raises(ValueError, match="unsupported"):
        JointAStarSolver().solve(p)


def test_solver_factory_supports_additional_implementations():
    from backend.mapf import MAPFSolver, MAPFSolution

    class WaitSolver(MAPFSolver):
        def solve(self, problem):
            return MAPFSolution(
                {r: [s] for r, s in problem.starts.items()},
                status="timeout",
                success=False,
            )

    register_solver("test_wait", WaitSolver)
    assert isinstance(create_solver("test_wait"), WaitSolver)
    with pytest.raises(ValueError, match="Unknown MAPF solver"):
        create_solver("not_installed")


def test_goal_generated_at_expansion_boundary_is_complete_not_partial():
    p = problem_for(dict(A=(0, 0), B=(5, 0)), dict(R=["A", "B"]), max_expansions=1)
    result = JointAStarSolver().solve(p)
    assert result.status == "solved" and result.paths["R"] == ["A", "B"]
    assert result.expanded == 1
    assert result.metrics["optimality_proven"] is False


def test_termination_reason_distinguishes_expansion_and_time_budgets():
    p = crossing()
    p.max_expansions = 1
    assert (
        JointAStarSolver().solve(p).metrics["termination_reason"] == "expansion_limit"
    )
    p.max_expansions = 100
    p.time_limit_s = 0
    assert JointAStarSolver().solve(p).metrics["termination_reason"] == "time_limit"


def test_fifteen_robots_need_fifteen_expansions_for_first_joint_action():
    coords = {f"{i}_{j}": (j * 5, i * 10) for i in range(15) for j in range(5)}
    paths = {f"R{i:02}": [f"{i}_{j}" for j in range(5)] for i in range(15)}
    for limit, rounds, status in [
        (14, 0, "timeout"),
        (15, 1, "partial"),
        (60, 4, "solved"),
    ]:
        p = problem_for(coords, paths, max_expansions=limit, time_limit_s=5)
        result = JointAStarSolver().solve(p)
        assert result.expanded <= limit
        assert result.status == status
        assert result.cost == rounds
        assert {len(path) for path in result.paths.values()} == {rounds + 1}
