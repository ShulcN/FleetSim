from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, File, Form, HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from backend.api.websocket_manager import WebSocketManager
from backend.collisions import CollisionMode
from backend.core.loop import run_simulation_loop
from backend.core.simulation_engine import SimulationEngine, SimulationSettings
from backend.core.world_state import WorldConfig
from backend.robots.commands import ControlCommand
from backend.robots.controllers import Waypoint
from backend.robots.factory import load_fleet_from_file

ROOT_DIR = Path(__file__).resolve().parents[2]
FRONTEND_DIR = ROOT_DIR / "frontend"
EXAMPLES_DIR = ROOT_DIR / "examples"
RUNTIME_DIR = ROOT_DIR / ".runtime"
RUNTIME_DIR.mkdir(exist_ok=True)

engine = SimulationEngine(
    world=WorldConfig(width_m=20.0, height_m=12.0),
    settings=SimulationSettings(dt=0.05, realtime_factor=1.0, max_sim_time=120.0),
)
ws_manager = WebSocketManager()


class VelocityCommandRequest(BaseModel):
    linear: float = Field(default=0.0, description="Forward velocity, m/s")
    angular: float = Field(default=0.0, description="Yaw rate, rad/s")


class WaypointRequest(BaseModel):
    x: float
    y: float


class RouteRequest(BaseModel):
    waypoints: list[WaypointRequest]


class NodeRouteRequest(BaseModel):
    node_ids: list[str]


async def _load_default_demo() -> None:
    await engine.configure_from_files(
        fleet_robots=load_fleet_from_file(EXAMPLES_DIR / "fleets" / "sample_fleet.json"),
        map_dxf_path=EXAMPLES_DIR / "maps" / "sample_factory.dxf",
        graph_geojson_path=EXAMPLES_DIR / "graphs" / "sample_routes.geojson",
        scenario_json_path=EXAMPLES_DIR / "scenarios" / "sample_wms_orders.json",
        max_sim_time=120.0,
        collision_mode=CollisionMode.COUNT_ONLY,
    )


@asynccontextmanager
async def lifespan(app: FastAPI):
    await _load_default_demo()

    loop_task = asyncio.create_task(
        run_simulation_loop(engine, ws_manager.broadcast_json)
    )

    try:
        yield
    finally:
        loop_task.cancel()
        try:
            await loop_task
        except asyncio.CancelledError:
            pass


app = FastAPI(title="Fleet Sim MVP", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")


@app.get("/")
async def index():
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/api/state")
async def get_state():
    return await engine.snapshot()


@app.post("/api/simulation/start")
async def start_simulation():
    await engine.start()
    return {"ok": True, "status": "running"}


@app.post("/api/simulation/pause")
async def pause_simulation():
    await engine.pause()
    return {"ok": True, "status": "paused"}


@app.post("/api/simulation/resume")
async def resume_simulation():
    await engine.resume()
    return {"ok": True, "status": "running"}


@app.post("/api/simulation/stop")
async def stop_simulation():
    await engine.stop()
    return {"ok": True, "status": "stopped"}


@app.post("/api/simulation/load-demo")
async def load_demo():
    await _load_default_demo()
    return {"ok": True, "message": "Default demo files loaded"}


@app.post("/api/simulation/configure")
async def configure_simulation(
    map_dxf: UploadFile | None = File(default=None),
    graph_geojson: UploadFile | None = File(default=None),
    fleet_json: UploadFile | None = File(default=None),
    scenario_json: UploadFile | None = File(default=None),
    duration: float = Form(default=120.0),
    collision_mode: str = Form(default="count_only"),
    autostart: bool = Form(default=True),
):
    try:
        mode = CollisionMode(collision_mode)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=f"Unknown collision mode: {collision_mode}") from exc

    map_path = await _save_upload(map_dxf, "map.dxf") if map_dxf else EXAMPLES_DIR / "maps" / "sample_factory.dxf"
    graph_path = await _save_upload(graph_geojson, "graph.geojson") if graph_geojson else EXAMPLES_DIR / "graphs" / "sample_routes.geojson"
    scenario_path = await _save_upload(scenario_json, "scenario.json") if scenario_json else EXAMPLES_DIR / "scenarios" / "sample_wms_orders.json"

    if fleet_json is not None:
        fleet_path = await _save_upload(fleet_json, "fleet.json")
        robots = load_fleet_from_file(fleet_path)
    else:
        robots = load_fleet_from_file(EXAMPLES_DIR / "fleets" / "sample_fleet.json")

    await engine.configure_from_files(
        fleet_robots=robots,
        map_dxf_path=map_path,
        graph_geojson_path=graph_path,
        scenario_json_path=scenario_path,
        max_sim_time=duration,
        collision_mode=mode,
    )
    if autostart:
        await engine.start()

    return {"ok": True, "state": await engine.snapshot()}


@app.get("/api/simulation/export-stats")
async def export_stats():
    data = await engine.export_metrics()
    payload = json.dumps(data, ensure_ascii=False, indent=2)
    return Response(
        content=payload,
        media_type="application/json",
        headers={"Content-Disposition": "attachment; filename=fleet_sim_stats.json"},
    )


@app.post("/api/robots/{robot_id}/cmd")
async def set_robot_command(robot_id: str, request: VelocityCommandRequest):
    ok = await engine.set_robot_command(
        robot_id,
        ControlCommand(linear=request.linear, angular=request.angular),
    )
    if not ok:
        raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
    return {"ok": True, "robot_id": robot_id, "command": request.model_dump()}


@app.post("/api/robots/{robot_id}/route")
async def set_robot_route(robot_id: str, request: RouteRequest):
    waypoints = [Waypoint(x=p.x, y=p.y) for p in request.waypoints]
    ok = await engine.set_robot_route(robot_id, waypoints)
    if not ok:
        raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
    return {"ok": True, "robot_id": robot_id, "route": request.model_dump()}


@app.post("/api/robots/{robot_id}/route-nodes")
async def set_robot_route_by_nodes(robot_id: str, request: NodeRouteRequest):
    ok = await engine.set_robot_route_by_nodes(robot_id, request.node_ids)
    if not ok:
        raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found or graph not loaded")
    return {"ok": True, "robot_id": robot_id, "node_ids": request.node_ids}


@app.post("/api/robots/{robot_id}/stop")
async def stop_robot(robot_id: str):
    ok = await engine.stop_robot(robot_id)
    if not ok:
        raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
    return {"ok": True, "robot_id": robot_id}


@app.websocket("/ws/state")
async def websocket_state(websocket: WebSocket):
    await ws_manager.connect(websocket)
    await websocket.send_json(await engine.snapshot())

    try:
        # detect client disconnects.
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)


async def _save_upload(upload: UploadFile, filename: str) -> Path:
    safe_name = Path(filename).name
    path = RUNTIME_DIR / safe_name
    path.write_bytes(await upload.read())
    return path
