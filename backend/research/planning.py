"""Windowed space-time planners over physical motion primitives.

Three independent searches share geometry: prioritized WHCA*, conflict-tree CBS,
operator-decomposed joint A*. Paths include time and full motion primitives.
"""

from dataclasses import dataclass, field
from math import hypot, ceil, floor, cos, sin
from heapq import heappop, heappush
from itertools import count
from time import monotonic
from .motion import Motion
from backend.mapf.joint_astar import segment_distance


@dataclass(frozen=True)
class Slot:
    time: float
    move: Motion
    offset: float = 0

    @property
    def end(self):
        return self.time + self.move.duration - self.offset


@dataclass
class Schedule:
    pose0: tuple
    slots: list = field(default_factory=list)

    def pose(self, t):
        p = self.pose0
        for s in self.slots:
            if t < s.time:
                return p
            if t <= s.end:
                return s.move.pose(t - s.time + s.offset)
            p = s.move.pose(s.move.duration)
        return p

    def at(self, t, nav):
        for s in self.slots:
            if s.time <= t < s.end - 1e-8:
                return s
        node = self.slots[-1].move.end if self.slots else None
        return None

    @property
    def end(self):
        return self.slots[-1].end if self.slots else 0


@dataclass
class TimedProblem:
    nav: object
    starts: dict  # free robot -> (node, heading)
    goals: dict
    fixed: dict  # in-flight, service and idle occupancy; cannot be cancelled
    waiting: dict
    horizon: int = 16
    time_limit_s: float = 0.2
    max_expansions: int = 100000
    gap: float = 1


@dataclass
class TimedResult:
    plans: dict
    status: str
    reason: str
    expanded: int
    elapsed_s: float
    metrics: dict


class Budget(Exception):
    pass


class Search:
    def __init__(self, p, params):
        self.p = p
        self.nav = p.nav
        self.params = params
        self.begun = monotonic()
        self.deadline = self.begun + p.time_limit_s
        self.expanded = 0
        self.serial = count()
        self.H = p.horizon * self.nav.tick_s
        self.metrics = dict(
            low_level_expanded=0,
            ct_expanded=0,
            ct_generated=0,
            low_level_calls=0,
            cache_hits=0,
            peak_ct_open=0,
            max_constraint_depth=0,
            conflicts_detected=0,
            reservation_checks=0,
            planned_agents=0,
            heuristic_nodes=0,
        )
        self.dist = {}
        for r, g in p.goals.items():
            self.dist[r] = self.nav.distances(g)
        self.metrics["heuristic_nodes"] = sum(len(d) for d in self.dist.values())
        self.cache = {}

    def budget(self, expand=False):
        if monotonic() >= self.deadline:
            raise Budget("time_limit")
        if expand:
            if self.expanded >= self.p.max_expansions:
                raise Budget("expansion_limit")
            self.expanded += 1

    def stationary(self, r):
        n, h = self.p.starts[r]
        x, y, _ = self.nav.nodes[n]
        return Schedule((x, y, h))

    def collision(self, r, a, s, b, until=None):
        self.metrics["reservation_checks"] += 1
        radius = (
            self.nav.robots[r].state.collision_radius
            + self.nav.robots[s].state.collision_radius
            + self.p.gap
            + 0.01
        )
        end = max(self.H, a.end, b.end) if until is None else until
        step = min(0.1, self.nav.tick_s / 5)

        # Fast conservative bounds reject separated schedules before temporal sampling.
        def bounds(schedule):
            pts = [schedule.pose0[:2]]
            for slot in schedule.slots:
                for piece in slot.move.pieces:
                    pts.extend(piece.pose(i / 4)[:2] for i in range(5))
            return (
                min(x for x, y in pts),
                min(y for x, y in pts),
                max(x for x, y in pts),
                max(y for x, y in pts),
            )

        aa, bb = bounds(a), bounds(b)
        shared_corridors = []
        for corridor in getattr(self.nav, "corridors", []):
            _, ca, cb, width = corridor
            width += max(
                self.nav.robots[r].state.collision_radius,
                self.nav.robots[s].state.collision_radius,
            )

            def overlaps(box):
                return not (
                    box[2] + width < min(ca[0], cb[0])
                    or box[0] - width > max(ca[0], cb[0])
                    or box[3] + width < min(ca[1], cb[1])
                    or box[1] - width > max(ca[1], cb[1])
                )

            if overlaps(aa) and overlaps(bb):
                shared_corridors.append(corridor)
        zone_margin = max((z[3] * 2 for z in self.nav.zones), default=0) + radius
        if not shared_corridors and (
            aa[2] + zone_margin < bb[0]
            or bb[2] + zone_margin < aa[0]
            or aa[3] + zone_margin < bb[1]
            or bb[3] + zone_margin < aa[1]
        ):
            return None
        last_a = a.pose(0)
        last_b = b.pose(0)
        for i in range(1, ceil(end / step) + 1):
            if i % 32 == 0:
                self.budget()
            t = min(end, i * step)
            pa, pb = a.pose(t), b.pose(t)
            # Minimum simultaneous relative separation along this short interval.
            u = (last_a[0] - last_b[0], last_a[1] - last_b[1])
            v = (pa[0] - pb[0], pa[1] - pb[1])
            if segment_distance(u, v, (0, 0), (0, 0)) <= radius:
                return max(0, t - step), "edge"
            for _, x, y, zr in self.nav.zones:
                if (
                    hypot(pa[0] - x, pa[1] - y)
                    <= zr + self.nav.robots[r].state.collision_radius + 0.1
                    and hypot(pb[0] - x, pb[1] - y)
                    <= zr + self.nav.robots[s].state.collision_radius + 0.1
                ):
                    return max(0, t - step), "vertex"
            # Shared physical corridor resources work across both navigation layers.
            for _, ca, cb, halfwidth in shared_corridors:
                dx, dy = cb[0] - ca[0], cb[1] - ca[1]
                length = hypot(dx, dy)
                ux, uy = dx / length, dy / length

                def direction(p, robot):
                    along = (p[0] - ca[0]) * ux + (p[1] - ca[1]) * uy
                    lateral = abs(-(p[0] - ca[0]) * uy + (p[1] - ca[1]) * ux)
                    if (
                        not -robot.state.collision_radius
                        <= along
                        <= length + robot.state.collision_radius
                        or lateral > halfwidth + robot.state.width / 2
                    ):
                        return 0
                    d = cos(p[2]) * ux + sin(p[2]) * uy
                    return 1 if d > 0.7 else -1 if d < -0.7 else 0

                if (
                    direction(pa, self.nav.robots[r])
                    * direction(pb, self.nav.robots[s])
                    < 0
                ):
                    return max(0, t - step), "edge"
            last_a, last_b = pa, pb
        return None

    def valid(self, r, plan, reservations, until=None):
        return all(
            self.collision(r, plan, s, other, until) is None
            for s, other in reservations.items()
            if s != r
        )

    def banned(self, r, plan, bans, node, heading):
        for t, m in bans:
            if any(abs(s.time - t) < 1e-8 and s.move == m for s in plan.slots):
                return True
            if t >= plan.end - 1e-8 and m.start == m.end == node:
                return True
        return False

    def individual(self, r, reservations, bans=frozenset(), wait_cost=1):
        self.metrics["low_level_calls"] += 1
        node, heading = self.p.starts[r]
        goal = self.p.goals[r]
        speed = self.nav.robots[r].route_follower.config.max_linear

        def heuristic(n):
            return self.dist[r].get(n, 1e12) / (speed * self.nav.tick_s)

        def statekey(n, h, t, plan):
            return (
                n,
                round(h, 6),
                t,
                (
                    sum(s.move.start == s.move.end for s in plan.slots)
                    if "max_wait_steps" in self.params
                    else 0
                ),
            )

        root = self.stationary(r)
        heap = [(heuristic(node), 0, next(self.serial), node, heading, 0, root)]
        seen = {statekey(node, heading, 0, root): 0}
        best = None
        best_key = None
        while heap:
            self.budget(True)
            self.metrics["low_level_expanded"] += 1
            _, g, _, n, h, t, plan = heappop(heap)
            if g > seen.get(statekey(n, h, t, plan), float("inf")):
                continue
            if n == goal or t >= self.p.horizon:
                if not self.banned(r, plan, bans, n, h) and self.valid(
                    r, plan, reservations
                ):
                    key = (heuristic(n), g)
                    if best_key is None or key < best_key:
                        best, best_key = plan, key
                    return best
                continue
            for move in [*self.nav.neighbors(r, n, h), self.nav.wait(n, h)]:
                if (t * self.nav.tick_s, move) in bans:
                    continue
                if move.start == move.end and sum(
                    s.move.start == s.move.end for s in plan.slots
                ) >= self.params.get("max_wait_steps", self.p.horizon):
                    continue
                nt = t + move.ticks
                candidate = Schedule(
                    plan.pose0, plan.slots + [Slot(t * self.nav.tick_s, move)]
                )
                # Reservations include endpoint occupancy to the window end.
                if not self.valid(r, candidate, reservations, candidate.end):
                    continue
                ng = g + (wait_cost if move.start == move.end else move.ticks)
                key = statekey(move.end, move.heading, nt, candidate)
                if ng >= seen.get(key, float("inf")):
                    continue
                seen[key] = ng
                heappush(
                    heap,
                    (
                        ng + heuristic(move.end),
                        ng,
                        next(self.serial),
                        move.end,
                        move.heading,
                        nt,
                        candidate,
                    ),
                )
            # Once a frontier exists, prioritize latency over exhaustive horizon endpoints.
            if best is not None and len(heap) > 1000:
                break
        return best

    def result(self, plans, reason):
        complete = (
            bool(plans)
            and len(plans) == len(self.p.starts)
            and all(
                (v.slots[-1].move.end if v.slots else self.p.starts[r][0])
                == self.p.goals[r]
                for r, v in plans.items()
            )
        )
        status = "solved" if complete else ("partial" if plans else "timeout")
        self.metrics["planned_agents"] = len(plans)
        self.metrics["makespan"] = (
            max((p.end for p in plans.values()), default=0) / self.nav.tick_s
        )
        self.metrics["sum_of_costs"] = (
            sum(p.end for p in plans.values()) / self.nav.tick_s
        )
        self.metrics["termination_reason"] = reason
        self.metrics["optimality_proven"] = False
        self.metrics["window_s"] = self.H
        return TimedResult(
            plans,
            status,
            reason,
            self.expanded,
            monotonic() - self.begun,
            dict(self.metrics),
        )

    def whca(self):
        book = {**self.p.fixed, **{r: self.stationary(r) for r in self.p.starts}}
        plans = {}
        kw = self.params.get("k_wait", 1)
        kg = self.params.get("k_goal", 0.1)
        order = sorted(
            self.p.starts,
            key=lambda r: (
                -(
                    kw * self.p.waiting.get(r, 0)
                    - kg * self.dist[r].get(self.p.starts[r][0], 1e12)
                ),
                r,
            ),
        )
        self.metrics["priority_order"] = order
        try:
            for r in order:
                self.budget()
                others = {s: v for s, v in book.items() if s != r}
                plan = self.individual(
                    r, others, wait_cost=self.params.get("C_wait", 1)
                )
                if plan is None:
                    continue
                book[r] = plans[r] = plan
            return self.result(plans, "window_complete")
        except Budget as exc:
            return self.result(plans, str(exc))

    def conflict(self, plans):
        ids = sorted(plans)
        found = None
        for i, r in enumerate(ids):
            for s in ids[:i]:
                hit = self.collision(r, plans[r], s, plans[s])
                if hit and (found is None or hit[0] < found[0]):
                    found = (hit[0], r, s, hit[1])
        if found:
            self.metrics["conflicts_detected"] += 1
        return found

    def prefix(self, plans, hit):
        if hit is None:
            return plans
        cutoff = hit[0]
        out = {}
        for r, p in plans.items():
            out[r] = Schedule(p.pose0, [s for s in p.slots if s.end <= cutoff + 1e-8])
        # Truncation changes terminal occupancy: revalidate against each other/fixed.
        if any(not self.valid(r, v, {**self.p.fixed, **out}) for r, v in out.items()):
            return {}
        return out if any(s.move.length for v in out.values() for s in v.slots) else {}

    def cbs(self):
        best = {}
        cache = {}
        heap = []
        visited = set()

        def cost(plans):
            times = [p.end for p in plans.values()]
            return (
                sum(times)
                if self.params.get("objective") == "sum_of_costs"
                else max(times, default=0)
            )

        def ll(r, bans):
            key = (r, bans)
            if self.params.get("cache_low_level", True) and key in cache:
                self.metrics["cache_hits"] += 1
                return cache[key]
            value = self.individual(r, self.p.fixed, bans)
            cache[key] = value
            return value

        try:
            root = {r: ll(r, frozenset()) for r in self.p.starts}
            if any(v is None for v in root.values()):
                return self.result({}, "no_path")
            hit = self.conflict(root)
            best = self.prefix(root, hit)
            heappush(heap, (cost(root), next(self.serial), {}, root, hit))
            self.metrics["ct_generated"] = 1
            while heap:
                self.metrics["peak_ct_open"] = max(
                    self.metrics["peak_ct_open"], len(heap)
                )
                if self.metrics["ct_expanded"] >= self.params.get("max_ct_nodes", 2000):
                    raise Budget("ct_node_limit")
                self.budget(True)
                self.metrics["ct_expanded"] += 1
                _, _, bans, plans, hit = heappop(heap)
                if hit is None:
                    return self.result(plans, "window_complete")
                time, r, s, _ = hit
                for who in (r, s):
                    slot = plans[who].at(time, self.nav)
                    if slot:
                        constraint = (slot.time, slot.move)
                    else:
                        node = (
                            plans[who].slots[-1].move.end
                            if plans[who].slots
                            else self.p.starts[who][0]
                        )
                        constraint = (
                            floor(time / self.nav.tick_s) * self.nav.tick_s,
                            self.nav.wait(node, plans[who].pose(time)[2]),
                        )
                    child = {**bans, who: bans.get(who, frozenset()) | {constraint}}
                    key = tuple(sorted(child.items()))
                    if key in visited:
                        continue
                    visited.add(key)
                    self.metrics["max_constraint_depth"] = max(
                        self.metrics["max_constraint_depth"],
                        sum(map(len, child.values())),
                    )
                    plan = ll(who, child[who])
                    if plan is None:
                        continue
                    candidate = {**plans, who: plan}
                    collision = self.conflict(candidate)
                    prefix = self.prefix(candidate, collision)
                    if sum(x.end for x in prefix.values()) > sum(
                        x.end for x in best.values()
                    ):
                        best = prefix
                    self.metrics["ct_generated"] += 1
                    heappush(
                        heap,
                        (
                            cost(candidate),
                            next(self.serial),
                            child,
                            candidate,
                            collision,
                        ),
                    )
            return self.result(best, "no_path")
        except Budget as exc:
            return self.result(best, str(exc))

    def joint(self):
        """Joint A* at planning ticks with operator decomposition of simultaneous actions."""
        ids = tuple(sorted(self.p.starts))
        origin = tuple((n, h, None, 0) for n, h in (self.p.starts[r] for r in ids))
        roots = tuple(self.stationary(r) for r in ids)
        best = {}
        best_progress = -1e30

        def heuristic(state):
            return max(
                (
                    self.dist[r].get(s[2].end if s[2] else s[0], 1e12)
                    / (
                        self.nav.robots[r].route_follower.config.max_linear
                        * self.nav.tick_s
                    )
                    for r, s in zip(ids, state)
                ),
                default=0,
            )

        heap = [(heuristic(origin), 0, next(self.serial), 0, origin, (), roots)]
        seen = {(0, origin)}
        try:
            while heap:
                self.budget(True)
                _, _, _, t, state, partial, plans = heappop(heap)
                i = len(partial)
                if i == len(ids):
                    nextstate = partial
                    nt = t + 1
                    if (nt, nextstate) in seen:
                        continue
                    seen.add((nt, nextstate))
                    candidate = dict(zip(ids, plans))
                    progress = -heuristic(nextstate)
                    if progress > best_progress:
                        if all(
                            self.valid(r, v, {**self.p.fixed, **candidate})
                            for r, v in candidate.items()
                        ):
                            best = candidate
                            best_progress = progress
                    if all(
                        s[0] == self.p.goals[r] and s[2] is None
                        for r, s in zip(ids, nextstate)
                    ):
                        if best == candidate:
                            return self.result(candidate, "window_complete")
                    if nt < self.p.horizon:
                        heappush(
                            heap,
                            (
                                nt + heuristic(nextstate),
                                0,
                                next(self.serial),
                                nt,
                                nextstate,
                                (),
                                plans,
                            ),
                        )
                    continue
                r = ids[i]
                node, h, busy, offset = state[i]
                moves = (
                    [busy]
                    if busy
                    else [*self.nav.neighbors(r, node, h), self.nav.wait(node, h)]
                )
                for move in moves:
                    plan = (
                        plans[i]
                        if busy
                        else Schedule(
                            plans[i].pose0,
                            plans[i].slots + [Slot(t * self.nav.tick_s, move)],
                        )
                    )
                    # Only selected peers are committed; unknown peers will get their action in this joint tick.
                    peers = {ids[j]: plans[j] for j in range(i)}
                    if not self.valid(
                        r, plan, {**self.p.fixed, **peers}, (t + 1) * self.nav.tick_s
                    ):
                        continue
                    noffset = offset + 1 if busy else 1
                    nxt = (
                        (move.end, move.heading, None, 0)
                        if noffset >= move.ticks
                        else (node, h, move, noffset)
                    )
                    updated = list(plans)
                    updated[i] = plan
                    heappush(
                        heap,
                        (
                            t + heuristic(state),
                            -len(partial) - 1,
                            next(self.serial),
                            t,
                            state,
                            partial + (nxt,),
                            tuple(updated),
                        ),
                    )
            return self.result(best, "window_complete" if best else "no_path")
        except Budget as exc:
            return self.result(best, str(exc))


def solve_timed(problem, solver, parameters):
    search = Search(problem, parameters)
    if solver == "whca":
        return search.whca()
    if solver == "cbs_astar":
        return search.cbs()
    if solver == "joint_astar":
        return search.joint()
    raise ValueError("Unknown temporal solver")
