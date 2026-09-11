from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from math import hypot, isfinite
from pathlib import Path
import re
from uuid import uuid4

from backend.collisions import CollisionMode
from backend.maps.dxf_map import load_dxf_map
from backend.maps.geojson_graph import load_geojson_graph
from backend.mapf.loop_network import LoopNetwork
from backend.mapf import validate_parameters
from backend.mapf.traffic import LoopTraffic
from backend.robots.factory import create_robots_from_fleet_dict
from .recording import utc_now


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name("." + path.name + "." + uuid4().hex + ".tmp")
    try:
        temp.write_text(
            json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def document_id(value):
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,100}", value):
        raise ValueError("Invalid document identifier")
    return value


class WorkspaceStore:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.scenarios = self.root / "workspace/scenarios"
        self.reports = self.root / "workspace/reports"
        self.assets_dir = self.root / "workspace/assets"
        self._previews = {}

    def seed(self):
        path = self.scenarios / "factory-agv-15.json"
        if not path.exists():
            wms = json.loads(
                (self.root / "examples/scenarios/factory_agv_15.json").read_text()
            )
            doc = dict(
                schema_version=1,
                kind="fleetsim.scenario",
                id="factory-agv-15",
                name="Завод · 15 AGV / 10 петель",
                description="Повторяющиеся доставки между цехами",
                map_asset="examples/maps/factory.dxf",
                graph_asset="examples/graphs/factory_routes.geojson",
                fleet=json.loads(
                    (self.root / "examples/fleets/factory_agv_15.json").read_text()
                ),
                wms=wms,
                simulation=dict(duration_s=600, collision_mode="stop_on_collision"),
                created_at=utc_now(),
                updated_at=utc_now(),
            )
            write_json(path, doc)

        stress_path = self.scenarios / "factory-stress-30.json"
        source = self.root / "examples/scenarios/factory_stress_30.json"
        if source.exists() and not stress_path.exists():
            doc = dict(
                schema_version=1,
                kind="fleetsim.scenario",
                id="factory-stress-30",
                name="Перекрёстки · 30 AGV / 10 петель",
                description="Фиксированный стресс-сценарий: 0.6 / 1.0 / 1.4 м/с, короткое обслуживание, повторные доставки. Столкновения учитываются без остановки запуска.",
                map_asset="examples/maps/factory.dxf",
                graph_asset="examples/graphs/factory_routes.geojson",
                fleet=json.loads(
                    (self.root / "examples/fleets/factory_stress_30.json").read_text()
                ),
                wms=json.loads(source.read_text()),
                simulation=dict(duration_s=600, collision_mode="count_only"),
                created_at=utc_now(),
                updated_at=utc_now(),
            )
            write_json(stress_path, doc)

        mixed_source = self.root / "examples/scenarios/factory_mixed_20.json"
        if (
            mixed_source.exists()
            and not (self.scenarios / "factory-mixed-20.json").exists()
        ):
            doc = json.loads(mixed_source.read_text())
            doc.update(created_at=utc_now(), updated_at=utc_now())
            write_json(self.scenarios / "factory-mixed-20.json", doc)

    def asset(self, relative, suffix):
        if not isinstance(relative, str):
            raise ValueError("Invalid asset path")
        path = (self.root / relative).resolve()
        roots = [
            self.root / "examples/maps",
            self.root / "examples/graphs",
            self.assets_dir,
        ]
        if (
            not any(path.is_relative_to(r.resolve()) for r in roots)
            or path.suffix.lower() != suffix
            or not path.is_file()
        ):
            raise ValueError(f"Unknown {suffix} asset")
        return path

    def assets(self):
        result = dict(maps=[], graphs=[])
        for folder in (
            self.root / "examples/maps",
            self.root / "examples/graphs",
            self.assets_dir,
        ):
            if not folder.exists():
                continue
            for path in sorted(folder.iterdir()):
                if path.suffix.lower() in (".dxf", ".geojson"):
                    key = "maps" if path.suffix.lower() == ".dxf" else "graphs"
                    result[key].append(
                        dict(id=str(path.relative_to(self.root)), name=path.name)
                    )
        return result

    def load(self, sid):
        path = self.scenarios / (document_id(sid) + ".json")
        if not path.exists():
            raise ValueError("Scenario not found")
        return json.loads(path.read_text(encoding="utf-8"))

    def list(self):
        return [
            dict(
                id=d["id"],
                name=d["name"],
                description=d.get("description", ""),
                updated_at=d.get("updated_at"),
                robots=len(d["fleet"]["robots"]),
                map_asset=d["map_asset"],
            )
            for p in sorted(self.scenarios.glob("*.json"))
            for d in [json.loads(p.read_text())]
        ]

    def validate(self, document):
        try:
            return self._validate(document)
        except (KeyError, TypeError, IndexError, AttributeError, OverflowError) as exc:
            raise ValueError(f"Invalid scenario structure: {exc}") from exc

    def _validate(self, document):
        d = deepcopy(document)
        if d.get("schema_version") != 1 or d.get("kind") != "fleetsim.scenario":
            raise ValueError("Unsupported scenario format/version")
        if not isinstance(d.get("name"), str) or not d["name"].strip():
            raise ValueError("Scenario name is required")
        graph = load_geojson_graph(self.asset(d["graph_asset"], ".geojson"))
        self.asset(d["map_asset"], ".dxf")
        robots = create_robots_from_fleet_dict(d["fleet"])
        if (
            not robots
            or len(robots) > 100
            or len({r.state.id for r in robots}) != len(robots)
        ):
            raise ValueError("Scenario requires 1–100 robots with unique IDs")
        for robot in robots:
            s = robot.state
            if any(
                not isfinite(v) or v <= 0
                for v in (
                    s.length,
                    s.width,
                    robot.kinematics.max_linear,
                    robot.kinematics.max_angular,
                    robot.route_follower.config.max_linear,
                )
            ):
                raise ValueError(f"{s.id}: invalid dimensions/speed")
            for key in ("energy_alpha", "energy_beta"):
                value = float(s.parameters.get(key, 1))
                if not isfinite(value) or value < 0:
                    raise ValueError(
                        "Energy coefficients must be finite and nonnegative"
                    )
            if not isfinite(s.collision_radius) or s.collision_radius < 0.5 * hypot(
                s.length, s.width
            ):
                raise ValueError("Collision radius must contain the robot footprint")
            if not all(isfinite(v) for v in (s.x, s.y, s.theta)):
                raise ValueError("Initial poses must be finite")
            if robot.route_follower.config.max_linear > robot.kinematics.max_linear:
                raise ValueError("Cruise speed exceeds maximum speed")
        settings = d["simulation"]
        duration = settings["duration_s"]
        if (
            isinstance(duration, bool)
            or not isinstance(duration, (int, float))
            or not isfinite(duration)
            or not 0 < duration <= 86400
        ):
            raise ValueError("Duration must be in (0, 86400] seconds")
        CollisionMode(settings["collision_mode"])
        config = d["wms"].get("fixed_loops")
        if not config:
            raise ValueError("The scenario editor requires fixed-loop routes")
        if d["wms"].get("orders"):
            raise ValueError("Fixed-loop deliveries belong in assignments[].orders")
        cell = float(config.get("cell_size_m", 5))
        if not isfinite(cell) or not 1 <= cell <= 20:
            raise ValueError("Cell size must be in [1,20] metres")
        validate_parameters(
            config.get("solver", "joint_astar"),
            {
                **{
                    k: config[k]
                    for k in ("horizon_steps", "time_limit_s", "max_expansions")
                    if k in config
                },
                **config.get("solver_parameters", {}),
            },
        )
        if config.get("mode") == "mixed":
            from backend.research.traffic import MixedTraffic

            amr = (
                load_geojson_graph(self.asset(d["amr_graph_asset"], ".geojson"))
                if d.get("amr_graph_asset")
                else None
            )
            MixedTraffic(
                graph,
                amr,
                {r.state.id: r for r in robots},
                config,
                load_dxf_map(self.asset(d["map_asset"], ".dxf")),
            )
        else:
            LoopTraffic(graph, {r.state.id: r for r in robots}, config)
        return d

    def save(self, document, sid=None):
        d = self.validate(document)
        d["id"] = document_id(sid) if sid else uuid4().hex
        path = self.scenarios / (d["id"] + ".json")
        d["created_at"] = (
            self.load(sid)["created_at"] if sid and path.exists() else utc_now()
        )
        d["updated_at"] = utc_now()
        write_json(path, d)
        return d

    def preview(self, map_asset, graph_asset, cell_size=5):
        map_path, graph_path = self.asset(map_asset, ".dxf"), self.asset(
            graph_asset, ".geojson"
        )
        key = (
            str(map_path),
            map_path.stat().st_mtime_ns,
            str(graph_path),
            graph_path.stat().st_mtime_ns,
            cell_size,
        )
        if key not in self._previews:
            graph = load_geojson_graph(graph_path)
            network = LoopNetwork(graph, cell_size)
            self._previews = {
                key: dict(
                    map=load_dxf_map(map_path).to_dict(),
                    graph=graph.to_dict(),
                    loops={
                        rid: [
                            dict(
                                id=n,
                                x=network.graph.nodes[n].x,
                                y=network.graph.nodes[n].y,
                            )
                            for n in loop.nodes
                        ]
                        for rid, loop in network.loops.items()
                    },
                )
            }
        return self._previews[key]

    def fingerprint(self, document):
        return {
            key: dict(
                path=document[key],
                sha256=hashlib.sha256(
                    self.asset(document[key], suffix).read_bytes()
                ).hexdigest(),
            )
            for key, suffix in [
                ("map_asset", ".dxf"),
                ("graph_asset", ".geojson"),
                ("amr_graph_asset", ".geojson"),
            ]
            if document.get(key)
        }
