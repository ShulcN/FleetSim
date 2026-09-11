import pytest
from backend.mapf import CBSSolver, create_solver, solver_catalog
from test_joint_astar import crossing, problem_for
from backend.mapf.joint_astar import resources_conflict, segment_distance


def assert_safe(problem, solution):
    assert set(solution.paths) == set(problem.starts)
    assert len({len(p) for p in solution.paths.values()}) == 1
    indices = {r: 0 for r in solution.paths}
    for t in range(len(next(iter(solution.paths.values()))) - 1):
        actions = {}
        for r, path in solution.paths.items():
            a = indices[r]
            b = a + (path[t + 1] != path[t])
            assert path[t : t + 2] == [
                problem.fixed_paths[r][a],
                problem.fixed_paths[r][b],
            ]
            coords = [
                (problem.graph.nodes[n].x, problem.graph.nodes[n].y)
                for n in path[t : t + 2]
            ]
            uses = problem.resources.get(r, [()] * len(problem.fixed_paths[r]))
            actions[r] = (coords, uses[a] + uses[b])
            indices[r] = b
        ids = list(actions)
        for i, r in enumerate(ids):
            for s in ids[:i]:
                a, uses = actions[r]
                b, other = actions[s]
                assert not resources_conflict(uses, other)
                assert (
                    segment_distance(*a, *b)
                    > problem.radii.get(r, 0)
                    + problem.radii.get(s, 0)
                    + problem.clearance
                )


@pytest.mark.parametrize("objective,expected", [("makespan", 4), ("sum_of_costs", 6)])
def test_cbs_resolves_crossing_optimally(objective, expected):
    p = crossing()
    p.time_limit_s = 5
    solver = create_solver("cbs_astar", {"objective": objective})
    result = solver.solve(p)
    assert result.success and result.cost == expected
    assert result.metrics["optimality_proven"]
    assert result.metrics["ct_expanded"] > 1
    assert result.metrics["low_level_calls"] > 2
    assert (
        result.expanded
        == result.metrics["ct_expanded"] + result.metrics["low_level_expanded"]
    )
    assert_safe(p, result)


def test_cbs_resource_only_conflict_and_footprints():
    from backend.mapf import ResourceUse

    p = problem_for(
        dict(A=(0, 0), B=(5, 0), C=(10, 0), D=(0, 20), E=(5, 20), F=(10, 20)),
        dict(R1=["A", "B", "C"], R2=["D", "E", "F"]),
    )
    p.resources = {r: [(), (ResourceUse("junction"),), ()] for r in p.starts}
    result = CBSSolver().solve(p)
    assert result.success and result.cost == 4
    assert_safe(p, result)
    p.resources = {}
    p.radii = {"R1": 10, "R2": 10}
    assert CBSSolver().solve(p).status == "infeasible"


def test_cbs_budgets_never_return_conflicting_partial_plan():
    for budget in (0, 1, 6, 10, 20, 40):
        p = crossing()
        p.max_expansions = budget
        p.time_limit_s = 5
        result = CBSSolver().solve(p)
        assert result.expanded <= budget
        assert_safe(p, result)
        if not result.success:
            assert result.metrics["termination_reason"] == "expansion_limit"
    p = crossing()
    p.time_limit_s = 0
    assert CBSSolver().solve(p).metrics["termination_reason"] == "time_limit"


def test_cbs_ct_limit_and_wait_bound():
    result = create_solver("cbs_astar", {"max_ct_nodes": 1}).solve(crossing())
    assert result.metrics["termination_reason"] == "ct_node_limit"
    assert result.metrics["ct_expanded"] == 1
    assert_safe(crossing(), result)
    result = create_solver("cbs_astar", {"max_wait_steps": 0}).solve(crossing())
    assert not result.success and result.metrics["termination_reason"] == "wait_limit"
    assert_safe(crossing(), result)


def test_cbs_goal_occupancy_persists_and_rejects_head_on_swap():
    p = problem_for(dict(A=(0, 0), B=(5, 0)), dict(R1=["A", "B"], R2=["B", "A"]))
    solver = create_solver("cbs_astar", {"max_wait_steps": 2})
    result = solver.solve(p)
    assert not result.success
    assert_safe(p, result)
    p = problem_for(dict(A=(0, 0), B=(5, 0)), dict(R1=["A", "B"], R2=["B"]))
    result = solver.solve(p)
    assert not result.success
    assert_safe(p, result)


def test_cbs_cache_does_not_change_optimum_and_catalog_describes_fields():
    results = [
        create_solver("cbs_astar", {"cache_low_level": v}).solve(crossing())
        for v in (True, False)
    ]
    assert all(r.success and r.cost == 4 for r in results)
    descriptor = next(s for s in solver_catalog() if s["id"] == "cbs_astar")
    assert all(p.get("description") for p in descriptor["parameters"])
    assert {m["key"] for m in descriptor["metrics"]} <= results[0].metrics.keys()


def test_goal_tail_constraint_delays_arrival_instead_of_disappearing_robot():
    p = problem_for(
        dict(A=(-10, 0), O=(0, 0), C=(0, -20), D=(0, -10), E=(0, 10)),
        dict(R1=["A", "O"], R2=["C", "D", "O", "E"]),
    )
    p.time_limit_s = 5
    result = create_solver("cbs_astar").solve(p)
    assert result.success and result.cost == 4
    assert result.paths["R1"][-2:] == ["A", "O"]
    assert_safe(p, result)


def test_three_agents_match_joint_astar_optimal_makespan():
    from backend.mapf import JointAStarSolver

    p = problem_for(
        dict(
            A=(-10, 0),
            B=(10, 0),
            C=(0, -10),
            D=(0, 10),
            E=(-10, -10),
            F=(10, 10),
            O=(0, 0),
        ),
        dict(R1=["A", "O", "B"], R2=["C", "O", "D"], R3=["E", "O", "F"]),
    )
    p.time_limit_s = 5
    expected = JointAStarSolver().solve(p)
    result = create_solver("cbs_astar").solve(p)
    assert expected.success and result.success and result.cost == expected.cost == 6
    assert_safe(p, result)
