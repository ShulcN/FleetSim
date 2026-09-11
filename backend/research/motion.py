"""Exact line/arc/spin motion primitives and conservative swept-circle geometry."""

from dataclasses import dataclass
from math import atan2, cos, sin, hypot, pi, tan, ceil
from bisect import bisect_right
from backend.robots.base import normalize_angle
from backend.mapf.joint_astar import segment_distance


@dataclass(frozen=True)
class Piece:
    x: float
    y: float
    theta: float
    length: float
    curvature: float = 0.0
    spin: float = 0.0

    def pose(self, fraction):
        if self.spin:
            return self.x, self.y, normalize_angle(self.theta + self.spin * fraction)
        distance = self.length * fraction
        angle = self.theta + distance * self.curvature
        if abs(self.curvature) > 1e-10:
            return (
                self.x + (sin(angle) - sin(self.theta)) / self.curvature,
                self.y + (cos(self.theta) - cos(angle)) / self.curvature,
                normalize_angle(angle),
            )
        return (
            self.x + distance * cos(self.theta),
            self.y + distance * sin(self.theta),
            normalize_angle(angle),
        )


@dataclass(frozen=True)
class Motion:
    start: str
    end: str
    pieces: tuple
    durations: tuple
    ticks: int
    tick_s: float

    @property
    def duration(self):
        return self.ticks * self.tick_s

    @property
    def length(self):
        return sum(p.length for p in self.pieces)

    @property
    def heading(self):
        return self.pieces[-1].pose(1)[2]

    def distance(self, time):
        result = 0
        for p, d in zip(self.pieces, self.durations):
            if time <= 0:
                break
            result += p.length * min(1, time / d)
            time -= d
        return result

    def pose(self, time):
        time = max(0, min(time, self.duration))
        for p, d in zip(self.pieces, self.durations):
            if time <= d + 1e-10:
                return p.pose(min(1, time / d) if d else 1)
            time -= d
        return self.pieces[-1].pose(1)


def motion(start, end, pieces, speed, angular, tick_s):
    durations = [
        max(
            p.length / speed,
            abs(p.spin) / angular,
            abs(p.length * p.curvature) / angular,
            1e-9,
        )
        for p in pieces
    ]
    total = sum(durations)
    ticks = max(1, ceil(total / tick_s - 1e-9))
    scale = ticks * tick_s / total
    return Motion(
        start, end, tuple(pieces), tuple(d * scale for d in durations), ticks, tick_s
    )


def line(a, b):
    return Piece(
        a[0], a[1], atan2(b[1] - a[1], b[0] - a[0]), hypot(b[0] - a[0], b[1] - a[1])
    )


def rounded_loop(points, radius=0.8, cell=3.0):
    """Return tangent line/arc pieces. Original corners are replaced, never snapped."""
    corners = []
    for i, b in enumerate(points):
        a = points[i - 1]
        c = points[(i + 1) % len(points)]
        incoming = atan2(b[1] - a[1], b[0] - a[0])
        outgoing = atan2(c[1] - b[1], c[0] - b[0])
        turn = normalize_angle(outgoing - incoming)
        if abs(turn) > pi - 0.01:
            raise ValueError("AGV loop contains a reversal")
        trim = radius * abs(tan(turn / 2))
        if (
            trim
            > min(hypot(b[0] - a[0], b[1] - a[1]), hypot(c[0] - b[0], c[1] - b[1])) / 2
            - 0.01
        ):
            raise ValueError("AGV turn radius does not fit adjacent edges")
        entry = (b[0] - trim * cos(incoming), b[1] - trim * sin(incoming))
        exit = (b[0] + trim * cos(outgoing), b[1] + trim * sin(outgoing))
        arc = (
            Piece(
                *entry, incoming, abs(turn) * radius, (1 if turn > 0 else -1) / radius
            )
            if abs(turn) > 1e-8
            else None
        )
        corners.append((entry, exit, arc))
    pieces = []
    for i, (_, exit, _) in enumerate(corners):
        entry, _, arc = corners[(i + 1) % len(points)]
        straight = line(exit, entry)
        n = max(1, ceil(straight.length / cell))
        for j in range(n):
            x, y, theta = straight.pose(j / n)
            pieces.append(Piece(x, y, theta, straight.length / n))
        if arc:
            pieces.append(arc)
    return pieces


class WallIndex:
    def __init__(self, dxf, layers=None, cell=8):
        self.cell = cell
        self.buckets = {}
        from math import floor

        for s in (dxf.segments if dxf else []):
            if layers and s.layer not in layers:
                continue
            a = (s.start.x, s.start.y)
            b = (s.end.x, s.end.y)
            for x in range(
                floor(min(a[0], b[0]) / cell), floor(max(a[0], b[0]) / cell) + 1
            ):
                for y in range(
                    floor(min(a[1], b[1]) / cell), floor(max(a[1], b[1]) / cell) + 1
                ):
                    self.buckets.setdefault((x, y), []).append((a, b))

    def clear(self, a, b, radius):
        from math import floor

        for x in range(
            floor((min(a[0], b[0]) - radius) / self.cell),
            floor((max(a[0], b[0]) + radius) / self.cell) + 1,
        ):
            for y in range(
                floor((min(a[1], b[1]) - radius) / self.cell),
                floor((max(a[1], b[1]) + radius) / self.cell) + 1,
            ):
                if any(
                    segment_distance(a, b, c, d) <= radius
                    for c, d in self.buckets.get((x, y), ())
                ):
                    return False
        return True

    def piece_clear(self, piece, radius):
        # A 5 cm arc chord has <0.4 mm sagitta at radius 0.8 m.
        n = max(1, ceil(piece.length / 0.05))
        last = piece.pose(0)[:2]
        for i in range(1, n + 1):
            p = piece.pose(i / n)[:2]
            if not self.clear(last, p, radius + 0.001):
                return False
            last = p
        return True
