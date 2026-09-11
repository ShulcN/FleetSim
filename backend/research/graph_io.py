"""Strict local-metre GeoJSON conversion, registration and lossless chain contraction."""

from pathlib import Path
import json, shlex, math, argparse
from backend.maps.geojson_graph import load_geojson_graph


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def read_gml(path):
    lexer = shlex.shlex(Path(path).read_text(), posix=True)
    lexer.whitespace_split = True
    lexer.commenters = "#"
    tokens = iter(lexer)

    def block():
        data = {}
        for key in tokens:
            if key == "]":
                return data
            value = next(tokens)
            if value == "[":
                value = block()
            else:
                try:
                    value = (
                        float(value)
                        if any(c in value.lower() for c in ".e")
                        else int(value)
                    )
                except ValueError:
                    pass
            data.setdefault(key, []).append(value)
        return data

    raw = block()["graph"][0]
    features = []
    nodes = {}
    for n in raw.get("node", []):
        identifier = str(n.get("label", n["id"])[0])
        coords = n.get("world", [])
        if len(coords) < 2:
            raise ValueError(f"{identifier}: GML node has no world coordinates")
        nodes[str(n["id"][0])] = identifier
        features.append(
            dict(
                type="Feature",
                geometry=dict(type="Point", coordinates=coords[:2]),
                properties=dict(type="node", id=identifier),
            )
        )
    xy = {f["properties"]["id"]: f["geometry"]["coordinates"] for f in features}
    for i, e in enumerate(raw.get("edge", [])):
        a, b = nodes[str(e["source"][0])], nodes[str(e["target"][0])]
        features.append(
            dict(
                type="Feature",
                geometry=dict(type="LineString", coordinates=[xy[a], xy[b]]),
                properties=dict(
                    type="edge",
                    id=f"E{i}",
                    start=a,
                    end=b,
                    direction=(
                        "oneway" if raw.get("directed", [0])[0] else "bidirectional"
                    ),
                ),
            )
        )
    return dict(type="FeatureCollection", features=features)


def convert(source, output, scale=1, rotation=0, offset=(0, 0), keep=(), max_chain_m=8):
    if (
        not math.isfinite(scale)
        or scale <= 0
        or not math.isfinite(rotation)
        or not all(math.isfinite(x) for x in offset)
    ):
        raise ValueError("Invalid coordinate transform")
    data = (
        read_gml(source)
        if Path(source).suffix.lower() == ".gml"
        else json.loads(Path(source).read_text())
    )
    theta = math.radians(rotation)
    c, s = math.cos(theta), math.sin(theta)

    def point(p):
        x, y = float(p[0]), float(p[1])
        if not math.isfinite(x + y):
            raise ValueError("Nonfinite graph coordinates")
        return [
            offset[0] + scale * (c * x - s * y),
            offset[1] + scale * (s * x + c * y),
        ]

    nodes = {}
    edges = {}
    for i, f in enumerate(data["features"]):
        g = f["geometry"]
        p = clean(f.get("properties", {}))
        if g["type"] == "Point":
            key = str(p.get("id", i))
            nodes[key] = dict(
                type="Feature",
                geometry=dict(type="Point", coordinates=point(g["coordinates"])),
                properties={**p, "id": key, "type": "node"},
            )
        elif g["type"] == "LineString":
            a = p.get("start", p.get("startid", p.get("source")))
            b = p.get("end", p.get("endid", p.get("target")))
            if a is None or b is None:
                raise ValueError("Edges need explicit endpoint IDs")
            eid = str(p.get("id", f"E{i}"))
            edges[eid] = dict(
                type="Feature",
                geometry=dict(
                    type="LineString", coordinates=[point(x) for x in g["coordinates"]]
                ),
                properties={
                    **p,
                    "id": eid,
                    "type": "edge",
                    "start": str(a),
                    "end": str(b),
                },
            )
    if any(
        e["properties"][k] not in nodes
        for e in edges.values()
        for k in ["start", "end"]
    ):
        raise ValueError("Edge endpoint is absent; import the combined graph")
    original = dict(nodes=len(nodes), edges=len(edges))
    adj = {n: set() for n in nodes}
    for eid, e in edges.items():
        adj[e["properties"]["start"]].add(eid)
        adj[e["properties"]["end"]].add(eid)
    protected = set(map(str, keep))
    for n in list(nodes):
        if n not in adj or n in protected or len(adj[n]) != 2:
            continue
        e1, e2 = sorted(adj[n])
        one, two = edges[e1], edges[e2]
        if any(
            e["properties"].get("direction", "bidirectional") != "bidirectional"
            for e in (one, two)
        ):
            continue

        def orient(e, to_node):
            p = e["properties"]
            xy = e["geometry"]["coordinates"]
            return (
                (p["start"], xy)
                if p["end"] == to_node
                else (p["end"], list(reversed(xy)))
            )

        a, xy1 = orient(one, n)
        b, xy2 = orient(two, n)
        if a == b:
            continue
        xy = xy1 + list(reversed(xy2))[1:]
        length = sum(math.dist(u, v) for u, v in zip(xy, xy[1:]))
        if length > max_chain_m:
            continue
        merged = dict(
            type="Feature",
            geometry=dict(type="LineString", coordinates=xy),
            properties=dict(
                type="edge", id=e1, start=a, end=b, direction="bidirectional"
            ),
        )
        for eid, e in [(e1, one), (e2, two)]:
            for end in [e["properties"]["start"], e["properties"]["end"]]:
                adj[end].discard(eid)
            del edges[eid]
        edges[e1] = merged
        adj[a].add(e1)
        adj[b].add(e1)
        del nodes[n]
        del adj[n]
    result = dict(
        type="FeatureCollection",
        coordinate_system="local_meters",
        registration=dict(scale=scale, rotation_deg=rotation, offset_m=list(offset)),
        source=str(source),
        statistics=dict(
            original=original, working=dict(nodes=len(nodes), edges=len(edges))
        ),
        features=list(nodes.values()) + list(edges.values()),
    )
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(
        json.dumps(result, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        + "\n"
    )
    return result["statistics"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--scale", type=float, default=1)
    p.add_argument("--rotation", type=float, default=0)
    p.add_argument("--offset", nargs=2, type=float, default=[0, 0])
    p.add_argument("--keep", nargs="*", default=[])
    p.add_argument("--max-chain-m", type=float, default=8)
    a = p.parse_args()
    print(
        json.dumps(
            convert(
                a.input, a.output, a.scale, a.rotation, a.offset, a.keep, a.max_chain_m
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
