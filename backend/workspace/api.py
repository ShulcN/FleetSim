from copy import deepcopy
import hashlib
import platform
import sys
from math import isfinite
from pathlib import Path
from uuid import uuid4

from fastapi import APIRouter, Body, File, HTTPException, UploadFile
from fastapi.responses import FileResponse

from backend.collisions import CollisionMode
from backend.core.simulation_engine import SimulationSettings
from backend.maps.dxf_map import load_dxf_map
from backend.maps.geojson_graph import load_geojson_graph
from backend.mapf import solver_catalog, validate_parameters
from backend.robots.factory import create_robots_from_fleet_dict
from backend.wms.orders import WmsScenario
from .reports import build_report, save_report
from .storage import WorkspaceStore, document_id


def router_for(engine, root):
    router = APIRouter(prefix="/api/workspace")
    store = WorkspaceStore(root)
    store.seed()

    def editable():
        if engine.recording.started and engine.status not in (
            "finished",
            "collision_stopped",
        ):
            raise HTTPException(
                409, "Сначала остановите запуск и нажмите «К подготовке»."
            )

    @router.get("/algorithm-help")
    async def algorithm_help(solver: str = "joint_astar"):
        files = {
            "joint_astar": "solver-reference.md",
            "cbs_astar": "cbs-astar.md",
            "whca": "whca.md",
        }
        if solver not in files:
            raise HTTPException(404, "Unknown solver help")
        return FileResponse(
            Path(__file__).resolve().parents[2] / "docs" / files[solver],
            media_type="text/plain; charset=utf-8",
        )

    @router.get("/catalog")
    async def catalog():
        return dict(
            scenarios=store.list(), solvers=solver_catalog(), assets=store.assets()
        )

    @router.get("/scenarios/{sid}")
    async def scenario(sid: str):
        return store.load(sid)

    @router.post("/scenarios")
    async def create(data: dict = Body(...)):
        editable()
        return store.save(data)

    @router.put("/scenarios/{sid}")
    async def update(sid: str, data: dict = Body(...)):
        editable()
        return store.save(data, sid)

    @router.post("/scenarios/{sid}/copy")
    async def copy(sid: str):
        editable()
        d = store.load(sid)
        d["name"] += " · копия"
        return store.save(d)

    @router.post("/migrate")
    async def migrate_scenario(data: dict = Body(...)):
        editable()
        from backend.research.migrate import migrate

        return migrate(data["document"], store, float(data.get("period_s", 100)))

    @router.post("/preview")
    async def preview(data: dict = Body(...)):
        size = float(data.get("cell_size_m", 5))
        if not isfinite(size) or not 1 <= size <= 20:
            raise ValueError("Cell size must be in [1,20]")
        if (
            data.get("document", {}).get("wms", {}).get("fixed_loops", {}).get("mode")
            == "mixed"
        ):
            from backend.research.network import Navigation

            d = data["document"]
            graph = load_geojson_graph(store.asset(d["graph_asset"], ".geojson"))
            amr = (
                load_geojson_graph(store.asset(d["amr_graph_asset"], ".geojson"))
                if d.get("amr_graph_asset")
                else None
            )
            dxf = load_dxf_map(store.asset(d["map_asset"], ".dxf"))
            robots = {r.state.id: r for r in create_robots_from_fleet_dict(d["fleet"])}
            nav = Navigation(graph, amr, robots, d["wms"]["fixed_loops"], dxf)
            loops = {
                r: [
                    dict(
                        id=n,
                        x=nav.nodes[n][0],
                        y=nav.nodes[n][1],
                        theta=nav.nodes[n][2],
                    )
                    for n in nav.loops[layer]
                ]
                for r, layer in nav.robot_layers.items()
                if layer != "amr"
            }
            agv_paths = {
                rid: [
                    dict(x=p.pose(i / 8)[0], y=p.pose(i / 8)[1])
                    for n in nav.loops[layer]
                    for _, pieces in nav.edges[n]
                    for p in pieces
                    for i in range(9)
                ]
                for rid, layer in nav.robot_layers.items()
                if layer != "amr"
            }
            return dict(
                agv_paths=agv_paths,
                map=dxf.to_dict(),
                graph=graph.to_dict(),
                loops={},
                robot_loops=loops,
                amr_graph=amr.to_dict() if amr else None,
            )
        return store.preview(data["map_asset"], data["graph_asset"], size)

    @router.post("/assets")
    async def upload_asset(file: UploadFile = File(...)):
        editable()
        suffix = Path(file.filename or "").suffix.lower()
        if suffix not in (".dxf", ".geojson", ".gml"):
            raise ValueError("Upload a DXF or GeoJSON file")
        data = await file.read(30 * 1024 * 1024 + 1)
        if len(data) > 30 * 1024 * 1024:
            raise ValueError("Asset exceeds 30 MB")
        store.assets_dir.mkdir(parents=True, exist_ok=True)
        path = store.assets_dir / (uuid4().hex + suffix)
        path.write_bytes(data)
        try:
            if suffix == ".dxf":
                load_dxf_map(path)
            elif suffix == ".gml":
                from backend.research.graph_io import read_gml

                read_gml(path)
            else:
                g = load_geojson_graph(path)
                if not g.nodes or not g.edges:
                    raise ValueError("Graph requires vertices and edges")
        except Exception:
            path.unlink(missing_ok=True)
            raise ValueError("Invalid map or route graph")
        result = dict(id=str(path.relative_to(store.root)), name=file.filename)
        if suffix == ".gml":
            from backend.research.graph_io import convert

            output = path.with_suffix(".geojson")
            result["statistics"] = convert(path, output)
            result["original_asset"] = result["id"]
            result["id"] = str(output.relative_to(store.root))
        return result

    @router.post("/convert-graph")
    async def convert_graph(data: dict = Body(...)):
        editable()
        from backend.research.graph_io import convert

        source = store.asset(data["asset"], Path(data["asset"]).suffix.lower())
        if source.suffix.lower() not in (".geojson", ".gml"):
            raise ValueError("Expected GML or GeoJSON")
        store.assets_dir.mkdir(parents=True, exist_ok=True)
        output = store.assets_dir / (uuid4().hex + ".geojson")
        stats = convert(
            source,
            output,
            scale=float(data.get("scale", 1)),
            rotation=float(data.get("rotation", 0)),
            offset=tuple(data.get("offset", [0, 0])),
            keep=set(data.get("keep", [])),
            max_chain_m=float(data.get("max_chain_m", 8)),
        )
        return dict(id=str(output.relative_to(store.root)), stats=stats)

    @router.post("/prepare")
    async def prepare(data: dict = Body(...)):
        if engine.status in ("running", "paused"):
            raise HTTPException(
                409, "Остановите текущий запуск перед подготовкой нового."
            )
        d = store.validate(store.load(data["scenario_id"]))
        solver = data["solver"]
        if solver == "whca" and d["wms"]["fixed_loops"].get("mode") != "mixed":
            raise ValueError(
                "WHCA* требует исследовательский сценарий. В редакторе создайте копию кнопкой «Перевести в новую кинематику AGV»."
            )
        params = validate_parameters(solver, data.get("parameters"))
        cfg = d["wms"]["fixed_loops"]
        cfg.update(solver=solver, solver_parameters=params)
        cfg.update(
            {k: params[k] for k in ("horizon_steps", "time_limit_s", "max_expansions")}
        )
        duration = data.get("duration_s", d["simulation"]["duration_s"])
        if (
            not isinstance(duration, (int, float))
            or not isfinite(duration)
            or not 0 < duration <= 86400
        ):
            raise ValueError("Invalid duration")
        backend_root = Path(__file__).resolve().parents[1]
        code_hash = hashlib.sha256()
        for source in sorted(backend_root.rglob("*.py")):
            code_hash.update(str(source.relative_to(backend_root)).encode())
            code_hash.update(source.read_bytes())
        inputs = dict(
            scenario=deepcopy(d),
            assets=store.fingerprint(d),
            software=dict(
                backend_sha256=code_hash.hexdigest(),
                python=sys.version,
                platform=platform.platform(),
            ),
            solver=dict(
                id=solver,
                parameters=params,
                descriptor=next(x for x in solver_catalog() if x["id"] == solver),
            ),
            simulation=dict(
                dt_s=0.05,
                duration_s=duration,
                collision_mode=d["simulation"]["collision_mode"],
            ),
        )
        await engine.reset(
            robots=create_robots_from_fleet_dict(d["fleet"]),
            dxf_map=load_dxf_map(store.asset(d["map_asset"], ".dxf")),
            graph=load_geojson_graph(store.asset(d["graph_asset"], ".geojson")),
            amr_graph=(
                load_geojson_graph(store.asset(d["amr_graph_asset"], ".geojson"))
                if d.get("amr_graph_asset")
                else None
            ),
            wms=WmsScenario.from_dict(d["wms"]),
            recording_inputs=inputs,
            settings=SimulationSettings(
                dt=0.05,
                max_sim_time=duration,
                collision_mode=CollisionMode(d["simulation"]["collision_mode"]),
            ),
        )
        return await engine.snapshot()

    @router.post("/unlock")
    async def unlock():
        if engine.status in ("running", "paused"):
            raise HTTPException(409, "Сначала остановите текущий запуск.")
        # The UI makes this explicit: the previous recording should be saved first.
        await engine.reset()
        return await engine.snapshot()

    @router.post("/speed")
    async def speed(data: dict = Body(...)):
        factor = data.get("factor")
        if (
            isinstance(factor, bool)
            or not isinstance(factor, (int, float))
            or not isfinite(factor)
            or not 0.25 <= factor <= 10
        ):
            raise ValueError("Speed must be in [0.25,10]")
        async with engine._lock:
            engine.settings.realtime_factor = factor
            engine.recording.controls.append(
                dict(time=engine.sim_time, event="speed", factor=factor)
            )
        return dict(factor=factor)

    @router.post("/planner-connection")
    async def planner_connection(data: dict = Body(...)):
        connected = data.get("connected")
        if not isinstance(connected, bool):
            raise ValueError("connected must be boolean")
        async with engine._lock:
            if engine.traffic is None or engine.status not in ("running", "paused"):
                raise HTTPException(
                    409, "Связь переключается во время активного запуска."
                )
            engine.traffic.set_connection(engine, connected)
            engine.recording.capture(engine, force=True)
        return dict(connected=connected)

    @router.get("/run")
    async def current_run():
        async with engine._lock:
            return dict(
                recording=engine.recording.info(),
                inputs=deepcopy(engine.recording.inputs),
            )

    @router.get("/history")
    async def history(at: float = 0):
        if not isfinite(at) or at < 0:
            raise ValueError("Invalid history time")
        async with engine._lock:
            return engine.recording.at(at)

    @router.get("/journal")
    async def journal(after: int = 0, limit: int = 200):
        if after < 0 or not 1 <= limit <= 1000:
            raise ValueError("Invalid journal pagination")
        async with engine._lock:
            return dict(
                run_id=engine.recording.id,
                total=len(engine.recording.journal),
                entries=deepcopy(engine.recording.journal[after : after + limit]),
            )

    @router.post("/reports")
    async def report():
        if not engine.recording.started:
            raise ValueError("Запустите симуляцию перед сохранением отчёта.")
        if not engine.recording.inputs.get("solver"):
            raise ValueError(
                "Подготовьте запуск через Solver Lab для сохранения полного отчёта."
            )
        # Snapshot under the same lock used by physics and planning.
        async with engine._lock:
            engine.recording.capture(engine, force=True)
            metrics = dict(
                metrics=engine.metrics.to_dict(engine.sim_time),
                orders=engine.wms.to_dict()["orders"],
            )
            document = build_report(engine, metrics)
        import asyncio

        return await asyncio.to_thread(save_report, store, document)

    @router.get("/reports/{filename}")
    async def download_report(filename: str):
        suffix = Path(filename).suffix
        if suffix not in (".json", ".html"):
            raise ValueError("Unknown report format")
        document_id(Path(filename).stem)
        path = store.reports / filename
        if not path.is_file():
            raise HTTPException(404, "Report not found")
        return FileResponse(
            path,
            media_type="application/json" if suffix == ".json" else "text/html",
            filename=filename if suffix == ".json" else None,
        )

    return router
