from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import cos, pi, sin
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class Segment2D:
    start: Point2D
    end: Point2D
    layer: str = "0"

    def to_dict(self) -> dict:
        return {
            "start": self.start.to_dict(),
            "end": self.end.to_dict(),
            "layer": self.layer,
        }


@dataclass
class DxfMap:
    """2D DXF map converted to primitives.
    """

    segments: list[Segment2D] = field(default_factory=list)
    source: str | None = None
    bounds: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "source": self.source,
            "bounds": self.bounds,
            "segments": [s.to_dict() for s in self.segments],
        }


def load_dxf_map(
    path: str | Path,
    collision_layers: Iterable[str] | None = None,
    circle_segments: int = 32,
) -> DxfMap:
    """Load a 2D DXF map.

    Supported entities for now:
    - LINE
    - LWPOLYLINE
    - POLYLINE
    """

    path = Path(path)
    layers = {layer.upper() for layer in collision_layers or []}

    try:
        return _load_with_ezdxf(path, layers, circle_segments)
    except ModuleNotFoundError:
        print("Error loading DXF map:", path)
        return DxfMap() 


def _layer_allowed(layer: str, layers: set[str]) -> bool:
    return not layers or layer.upper() in layers


def _load_with_ezdxf(path: Path, layers: set[str], circle_segments: int) -> DxfMap:
    import ezdxf  # type: ignore

    doc = ezdxf.readfile(str(path))
    msp = doc.modelspace()
    segments: list[Segment2D] = []

    for entity in msp:
        layer = getattr(entity.dxf, "layer", "0")
        if not _layer_allowed(layer, layers):
            continue

        entity_type = entity.dxftype()

        if entity_type == "LINE":
            start = entity.dxf.start
            end = entity.dxf.end
            segments.append(
                Segment2D(Point2D(float(start.x), float(start.y)), Point2D(float(end.x), float(end.y)), layer)
            )

        elif entity_type == "LWPOLYLINE":
            points = [Point2D(float(p[0]), float(p[1])) for p in entity.get_points()]
            segments.extend(_polyline_to_segments(points, bool(entity.closed), layer))

        elif entity_type == "POLYLINE":
            points = [Point2D(float(v.dxf.location.x), float(v.dxf.location.y)) for v in entity.vertices]
            segments.extend(_polyline_to_segments(points, bool(entity.is_closed), layer))


    return DxfMap(segments=segments, source=str(path), bounds=_compute_bounds(segments))


def _polyline_to_segments(points: list[Point2D], closed: bool, layer: str) -> list[Segment2D]:
    if len(points) < 2:
        return []
    result = [Segment2D(points[i], points[i + 1], layer) for i in range(len(points) - 1)]
    if closed:
        result.append(Segment2D(points[-1], points[0], layer))
    return result


def _compute_bounds(segments: list[Segment2D]) -> dict:
    if not segments:
        return {"min_x": 0.0, "min_y": 0.0, "max_x": 20.0, "max_y": 12.0}

    xs = []
    ys = []
    for segment in segments:
        xs.extend([segment.start.x, segment.end.x])
        ys.extend([segment.start.y, segment.end.y])

    return {
        "min_x": min(xs),
        "min_y": min(ys),
        "max_x": max(xs),
        "max_y": max(ys),
    }
