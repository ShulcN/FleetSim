"""CBS with individual space-time A* on fixed paths and swept resources.

Constraints forbid an individual (round, source-index, destination-index) action.
Splitting a conflicting pair into those two prohibitions is exhaustive, including
geometric and corridor conflicts. Goal occupancy persists after arrival.
"""

from __future__ import annotations

import heapq
from itertools import count
from math import isfinite
from time import monotonic

from .base import MAPFProblem, MAPFSolution, MAPFSolver
from .joint_astar import resources_conflict, segment_distance


class SearchLimit(Exception):
    pass


class CBSSolver(MAPFSolver):
    name = "cbs_astar"

    def solve(self, problem: MAPFProblem) -> MAPFSolution:
        begun = monotonic()
        params = getattr(self, "parameters", {})
        max_wait = int(params.get("max_wait_steps", 16))
        max_ct = int(params.get("max_ct_nodes", 2000))
        objective = params.get("objective", "makespan")
        use_cache = params.get("cache_low_level", True)
        if max_wait < 0 or max_ct < 1 or objective not in ("makespan", "sum_of_costs"):
            raise ValueError("Invalid CBS settings")
        if problem.constraints:
            raise ValueError("CBS generic external constraints are unsupported")
        ids = tuple(sorted(problem.starts))
        if set(ids) != set(problem.goals) or set(ids) != set(problem.fixed_paths):
            raise ValueError(
                "starts, goals and fixed_paths must contain the same robots"
            )
        if any(
            not isfinite(v) or v < 0
            for v in (problem.time_limit_s, problem.max_expansions, problem.clearance)
        ):
            raise ValueError(
                "Planning limits and clearance must be finite and nonnegative"
            )
        paths = problem.fixed_paths
        for rid, path in paths.items():
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
            if not isfinite(problem.radii.get(rid, 0)) or problem.radii.get(rid, 0) < 0:
                raise ValueError("Robot radii must be finite and nonnegative")
        coords = {
            r: [(problem.graph.nodes[n].x, problem.graph.nodes[n].y) for n in p]
            for r, p in paths.items()
        }
        resources = {
            r: problem.resources.get(r, [()] * len(p)) for r, p in paths.items()
        }
        metrics = dict(
            ct_expanded=0,
            ct_generated=0,
            low_level_expanded=0,
            low_level_calls=0,
            cache_hits=0,
            conflicts_detected=0,
            peak_ct_open=0,
            max_constraint_depth=0,
        )
        deadline = begun + problem.time_limit_s
        serial = count()
        best = {r: [0] for r in ids}
        best_progress = 0
        cache = {}
        conflict_cache = {}

        def check_budget(expanding=False):
            if monotonic() >= deadline:
                raise SearchLimit("time_limit")
            if (
                expanding
                and metrics["ct_expanded"] + metrics["low_level_expanded"]
                >= problem.max_expansions
            ):
                raise SearchLimit("expansion_limit")

        def conflict(r, a, b, s, c, d):
            key = (r, a, b, s, c, d)
            if key not in conflict_cache:
                conflict_cache[key] = (
                    resources_conflict(
                        resources[r][a] + resources[r][b],
                        resources[s][c] + resources[s][d],
                    )
                    or segment_distance(
                        coords[r][a], coords[r][b], coords[s][c], coords[s][d]
                    )
                    <= problem.radii.get(r, 0)
                    + problem.radii.get(s, 0)
                    + problem.clearance
                    + 1e-9
                )
            return conflict_cache[key]

        def low_level(r, bans):
            metrics["low_level_calls"] += 1
            key = (r, bans)
            if use_cache and key in cache:
                metrics["cache_hits"] += 1
                return cache[key]
            goal = len(paths[r]) - 1
            # Bounds the number of inserted waits before arrival, not terminal padding.
            heap = [(goal, 0, 0)]  # f, -progress, time
            seen = {(0, 0)}
            parent = {}
            result = None
            while heap:
                check_budget(expanding=True)
                _, negpos, t = heapq.heappop(heap)
                pos = -negpos
                metrics["low_level_expanded"] += 1
                if pos == goal and not any(
                    a == b == goal and when >= t for when, a, b in bans
                ):
                    state = (pos, t)
                    seq = [pos]
                    while state in parent:
                        state = parent[state]
                        seq.append(state[0])
                    result = tuple(reversed(seq))
                    break
                for nxt in (pos + 1, pos):
                    if nxt > goal or t + 1 - nxt > max_wait or (t, pos, nxt) in bans:
                        continue
                    state = (nxt, t + 1)
                    if state in seen:
                        continue
                    seen.add(state)
                    parent[state] = (pos, t)
                    heapq.heappush(heap, (t + 1 + goal - nxt, -nxt, t + 1))
            if use_cache:
                cache[key] = result
            return result

        def padded(plan):
            length = max(map(len, plan.values()), default=1)
            return {
                r: list(seq) + [seq[-1]] * (length - len(seq))
                for r, seq in plan.items()
            }

        def first_conflict(plan):
            nonlocal best, best_progress
            joint = padded(plan)
            length = max(map(len, joint.values()), default=1)
            found = None
            safe_rounds = length - 1
            for t in range(length - 1):
                check_budget()
                for i, r in enumerate(ids):
                    for s in ids[:i]:
                        if conflict(r, *joint[r][t : t + 2], s, *joint[s][t : t + 2]):
                            found = (
                                t,
                                r,
                                s,
                                joint[r][t],
                                joint[r][t + 1],
                                joint[s][t],
                                joint[s][t + 1],
                            )
                            safe_rounds = t
                            break
                    if found:
                        break
                if found:
                    break
            progress = sum(seq[safe_rounds] for seq in joint.values())
            if progress > best_progress:
                best_progress = progress
                best = {r: seq[: safe_rounds + 1] for r, seq in joint.items()}
            if found:
                metrics["conflicts_detected"] += 1
            return found

        def cost(plan):
            arrivals = [len(seq) - 1 for seq in plan.values()]
            return (
                sum(arrivals)
                if objective == "sum_of_costs"
                else max(arrivals, default=0)
            )

        def finish(plan, status, reason, optimal=False):
            joint = padded(plan)
            # The window has no external time-dependent constraints. A common
            # leading wait only shifts the same safe schedule; remove it so a
            # rolling executor cannot keep replanning the identical idle state.
            while joint and all(
                len(seq) > 1 and seq[0] == seq[1] for seq in joint.values()
            ):
                joint = {r: seq[1:] for r, seq in joint.items()}
            # Arrival in a returned prefix is the last progress, not padded waits.
            arrivals = [
                max((t for t in range(1, len(seq)) if seq[t] != seq[t - 1]), default=0)
                for seq in joint.values()
            ]
            details = dict(
                metrics,
                termination_reason=reason,
                optimality_proven=optimal,
                objective=objective,
                makespan=max((len(seq) - 1 for seq in joint.values()), default=0),
                sum_of_costs=sum(arrivals),
            )
            result_cost = (
                details["sum_of_costs"]
                if objective == "sum_of_costs"
                else details["makespan"]
            )
            return MAPFSolution(
                paths={r: [paths[r][i] for i in seq] for r, seq in joint.items()},
                cost=result_cost,
                success=status == "solved",
                status=status,
                message=(
                    "Window goals reached"
                    if status == "solved"
                    else "CBS search stopped: " + reason
                ),
                expanded=metrics["ct_expanded"] + metrics["low_level_expanded"],
                elapsed_s=monotonic() - begun,
                metrics=details,
            )

        for i, r in enumerate(ids):
            for s in ids[:i]:
                if conflict(r, 0, 0, s, 0, 0):
                    return finish(best, "infeasible", "infeasible")
        try:
            root = {}
            for r in ids:
                root[r] = low_level(r, frozenset())
            # Root individual paths always exist for validated, unconstrained fixed routes.
            root_conflict = first_conflict(root)
            empty = tuple(frozenset() for _ in ids)
            heap = [(cost(root), next(serial), empty, root, root_conflict)]
            visited = {empty}
            metrics["ct_generated"] = metrics["peak_ct_open"] = 1
            while heap:
                check_budget(expanding=True)
                if metrics["ct_expanded"] >= max_ct:
                    raise SearchLimit("ct_node_limit")
                _, _, constraints, plan, hit = heapq.heappop(heap)
                metrics["ct_expanded"] += 1
                if hit is None:
                    return finish(plan, "solved", "goal_reached", True)
                t, r, s, a, b, c, d = hit
                for robot, src, dst in ((r, a, b), (s, c, d)):
                    check_budget()
                    index = ids.index(robot)
                    child = list(constraints)
                    child[index] = child[index] | {(t, src, dst)}
                    child = tuple(child)
                    if child in visited:
                        continue
                    visited.add(child)
                    metrics["max_constraint_depth"] = max(
                        metrics["max_constraint_depth"], sum(map(len, child))
                    )
                    replacement = low_level(robot, child[index])
                    if replacement is None:
                        continue
                    updated = dict(plan)
                    updated[robot] = replacement
                    hit_child = first_conflict(updated)  # Save only validated prefixes.
                    heapq.heappush(
                        heap, (cost(updated), next(serial), child, updated, hit_child)
                    )
                    metrics["ct_generated"] += 1
                    metrics["peak_ct_open"] = max(metrics["peak_ct_open"], len(heap))
            return finish(
                best, "partial" if best_progress else "infeasible", "wait_limit"
            )
        except SearchLimit as exc:
            # A complete validated candidate may already have been generated.
            complete = all(best[r][-1] == len(paths[r]) - 1 for r in ids)
            if complete:
                return finish(best, "solved", str(exc), False)
            return finish(best, "partial" if best_progress else "timeout", str(exc))
