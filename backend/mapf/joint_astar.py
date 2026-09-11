"""Joint A* with operator decomposition over fixed route positions.

Partial joint actions live in the OPEN heap, avoiding materializing 2**N
successors. Only complete collision-free joint actions are executable. A round
costs one; max remaining route steps is an admissible makespan heuristic.
"""

from __future__ import annotations

import heapq
from itertools import count
from math import hypot, isfinite
from time import monotonic

from .base import MAPFProblem, MAPFSolution, MAPFSolver


def segment_distance(a, b, c, d):
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def point_segment(p, u, v):
        dx, dy = v[0] - u[0], v[1] - u[1]
        t = (
            max(
                0.0,
                min(
                    1.0, ((p[0] - u[0]) * dx + (p[1] - u[1]) * dy) / (dx * dx + dy * dy)
                ),
            )
            if dx or dy
            else 0.0
        )
        return hypot(p[0] - u[0] - t * dx, p[1] - u[1] - t * dy)

    if cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0:
        return 0.0
    return min(
        point_segment(a, c, d),
        point_segment(b, c, d),
        point_segment(c, a, b),
        point_segment(d, a, b),
    )


def resources_conflict(left, right):
    return any(
        a.key == b.key
        and (a.direction == 0 or b.direction == 0 or a.direction != b.direction)
        for a in left
        for b in right
    )


class JointAStarSolver(MAPFSolver):
    name = "joint_astar"

    def solve(self, problem: MAPFProblem) -> MAPFSolution:
        begun = monotonic()
        if problem.constraints:
            raise ValueError(
                "joint_astar accepts resource constraints; generic constraints are unsupported"
            )
        ids = tuple(sorted(problem.starts))
        if set(ids) != set(problem.goals) or set(ids) != set(problem.fixed_paths):
            raise ValueError(
                "starts, goals and fixed_paths must contain the same robots"
            )
        if (
            not isfinite(problem.time_limit_s)
            or problem.time_limit_s < 0
            or problem.max_expansions < 0
        ):
            raise ValueError("Planning limits must be finite and nonnegative")
        if not isfinite(problem.clearance) or problem.clearance < 0:
            raise ValueError("clearance must be finite and nonnegative")
        paths = [problem.fixed_paths[r] for r in ids]
        for rid, path in zip(ids, paths):
            if (
                not path
                or path[0] != problem.starts[rid]
                or path[-1] != problem.goals[rid]
            ):
                raise ValueError(f"Invalid fixed path endpoints for {rid}")
            if any(n not in problem.graph.nodes for n in path):
                raise ValueError(f"Unknown graph node in path for {rid}")
            if any(
                a != b and problem.graph.find_edge_between(a, b) is None
                for a, b in zip(path, path[1:])
            ):
                raise ValueError(f"Fixed path for {rid} contains an illegal edge")
            if rid in problem.resources and len(problem.resources[rid]) != len(path):
                raise ValueError(f"Resource count does not match path for {rid}")
        radii = [problem.radii.get(r, 0.0) for r in ids]
        if any(not isfinite(r) or r < 0 for r in radii):
            raise ValueError("Robot radii must be finite and nonnegative")
        coords = [
            [(problem.graph.nodes[n].x, problem.graph.nodes[n].y) for n in p]
            for p in paths
        ]
        uses = [problem.resources.get(r, [()] * len(p)) for r, p in zip(ids, paths)]
        target = tuple(len(p) - 1 for p in paths)
        origin = (0,) * len(ids)
        cache = {}

        def compatible(i, a, b, j, c, d):
            key = i, a, b, j, c, d
            if key not in cache:
                cache[key] = (
                    not resources_conflict(
                        uses[i][a] + uses[i][b], uses[j][c] + uses[j][d]
                    )
                    and segment_distance(
                        coords[i][a], coords[i][b], coords[j][c], coords[j][d]
                    )
                    > radii[i] + radii[j] + problem.clearance + 1e-9
                )
            return cache[key]

        def result(
            end,
            status,
            expanded,
            message,
            termination_reason=None,
            optimality_proven=False,
        ):
            sequence = [end]
            while sequence[-1] in parent:
                sequence.append(parent[sequence[-1]])
            sequence.reverse()
            return MAPFSolution(
                paths={
                    rid: [paths[i][s[i]] for s in sequence] for i, rid in enumerate(ids)
                },
                cost=len(sequence) - 1,
                success=status == "solved",
                message=message,
                status=status,
                expanded=expanded,
                elapsed_s=monotonic() - begun,
                metrics={
                    "termination_reason": termination_reason or status,
                    "optimality_proven": optimality_proven,
                },
            )

        parent = {}
        if any(
            not compatible(i, 0, 0, j, 0, 0) for i in range(len(ids)) for j in range(i)
        ):
            return result(
                origin,
                "infeasible",
                0,
                "Initial positions or resource reservations conflict",
            )
        serial = count()
        # Heap entries: f, progress tie-break, operator depth tie-break, serial,
        # completed rounds, source joint state, partial next joint state.
        heap = [(max(target, default=0), 0, 0, next(serial), 0, origin, ())]
        distances = {origin: 0}
        best = origin
        expanded = 0
        deadline = begun + problem.time_limit_s
        while heap and monotonic() < deadline and expanded < problem.max_expansions:
            _, _, _, _, g, state, partial = heapq.heappop(heap)
            if distances.get(state) != g:
                continue
            if not partial and state == target:
                return result(
                    state,
                    "solved",
                    expanded,
                    "Window goals reached",
                    "goal_reached",
                    True,
                )
            expanded += 1
            i = len(partial)
            if i == len(ids):
                continue
            for nxt in (state[i] + 1, state[i]):
                if nxt > target[i]:
                    continue
                if any(
                    not compatible(i, state[i], nxt, j, state[j], partial[j])
                    for j in range(i)
                ):
                    continue
                chosen = partial + (nxt,)
                if len(chosen) == len(ids):
                    # An all-wait round cannot improve a static planning problem.
                    if chosen == state or g + 1 >= distances.get(chosen, float("inf")):
                        continue
                    distances[chosen] = g + 1
                    parent[chosen] = state
                    if sum(chosen) > sum(best):
                        best = chosen
                    h = max((t - p for t, p in zip(target, chosen)), default=0)
                    heapq.heappush(
                        heap,
                        (g + 1 + h, -sum(chosen), 0, next(serial), g + 1, chosen, ()),
                    )
                else:
                    optimistic = chosen + tuple(
                        min(p + 1, t)
                        for p, t in zip(state[len(chosen) :], target[len(chosen) :])
                    )
                    h = max(t - p for t, p in zip(target, optimistic))
                    heapq.heappush(
                        heap,
                        (
                            g + 1 + h,
                            -sum(optimistic),
                            -len(chosen),
                            next(serial),
                            g,
                            state,
                            chosen,
                        ),
                    )
        # A goal generated on the last allowed expansion is already a complete plan.
        if best == target:
            return result(
                best, "solved", expanded, "Window goals reached", "goal_reached"
            )
        limited = bool(heap)
        reason = (
            ("expansion_limit" if expanded >= problem.max_expansions else "time_limit")
            if limited
            else "exhausted"
        )
        status = (
            "partial" if best != origin else ("timeout" if limited else "infeasible")
        )
        return result(
            best,
            status,
            expanded,
            "Planning limit reached" if limited else "Window goals blocked",
            reason,
        )
