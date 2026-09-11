import asyncio
from copy import deepcopy
import json
from pathlib import Path
import shutil

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.testclient import TestClient
import pytest

from backend.core.simulation_engine import SimulationEngine
from backend.mapf import MAPFSolver, MAPFSolution, register_solver
from backend.workspace.api import router_for
from backend.workspace.storage import WorkspaceStore
from backend.workspace.reports import read_report

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def workspace(tmp_path):
    for relative in [
        "maps/factory.dxf",
        "graphs/factory_routes.geojson",
        "fleets/factory_agv_15.json",
        "scenarios/factory_agv_15.json",
    ]:
        target = tmp_path / "examples" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / "examples" / relative, target)
    engine = SimulationEngine()
    engine.use_comms = False
    app = FastAPI()
    app.include_router(router_for(engine, tmp_path))

    @app.exception_handler(ValueError)
    async def invalid(request, exc):
        return JSONResponse(status_code=400, content={"detail": str(exc)})

    with TestClient(app) as client:
        yield client, engine, WorkspaceStore(tmp_path)


def prepare(client, scenario="factory-agv-15", **kwargs):
    r = client.post(
        "/api/workspace/prepare",
        json=dict(scenario_id=scenario, solver="joint_astar", parameters={}, **kwargs),
    )
    assert r.status_code == 200, r.text
    return r.json()


def test_scenarios_copy_save_validation_and_asset_paths(workspace):
    c, e, store = workspace
    original = c.get("/api/workspace/scenarios/factory-agv-15").json()
    copied = c.post("/api/workspace/scenarios/factory-agv-15/copy").json()
    assert copied["id"] != original["id"]
    copied["name"] = "Эксперимент 2"
    assert (
        c.put("/api/workspace/scenarios/" + copied["id"], json=copied).status_code
        == 200
    )
    assert store.load("factory-agv-15")["name"] == original["name"]
    assert store.load(copied["id"])["name"] == "Эксперимент 2"
    for invalid in [
        dict(copied, schema_version=99),
        dict(copied, map_asset="../private.dxf"),
        {},
        dict(copied, fleet={"robots": []}),
    ]:
        assert c.post("/api/workspace/scenarios", json=invalid).status_code == 400
    bad = deepcopy(copied)
    bad["fleet"]["robots"][0]["initial_pose"]["x"] += 1
    assert c.post("/api/workspace/scenarios", json=bad).status_code == 400


def test_history_is_read_only_and_report_round_trips(workspace):
    c, e, store = workspace
    original = prepare(c, duration_s=12)

    async def advance():
        await e.start()
        for _ in range(241):
            await e.step()

    c.portal.call(advance)
    positions = [(r.state.x, r.state.y) for r in e.robots.values()]
    history = c.get("/api/workspace/history?at=0").json()
    assert history["frame"]["time"] == 0
    assert history["run_id"] == original["recording"]["run_id"]
    assert positions == [(r.state.x, r.state.y) for r in e.robots.values()]
    journal = c.get("/api/workspace/journal").json()
    assert journal["total"] > 0
    assert set(journal["entries"][0]["decisions"]) == set(e.robots)
    result = c.post("/api/workspace/reports")
    assert result.status_code == 200, result.text
    rid = result.json()["id"]
    report = read_report(store.reports / (rid + ".json"))
    assert report["kind"] == "fleetsim.report" and report["schema_version"] == 1
    assert report["inputs"]["solver"]["parameters"]["time_limit_s"] == 0.2
    assert report["inputs"]["assets"]["map_asset"]["sha256"]
    assert len(report["inputs"]["scenario"]["fleet"]["robots"]) == 15
    assert report["summary"]["solver_calls"] == len(report["solver_journal"])
    assert report["summary"]["robot_count"] == 15
    assert len(report["history"]["frames"]) > 10
    assert c.get(result.json()["html_url"]).status_code == 200
    assert c.get(result.json()["json_url"]).json()["report_id"] == rid
    # File edits after a run do not alter its frozen input document.
    d = store.load("factory-agv-15")
    d["name"] = "Changed"
    store.save(d, d["id"])
    assert e.recording.inputs["scenario"]["name"] != "Changed"


def test_active_run_locks_editor_but_speed_and_history_are_available(workspace):
    c, e, store = workspace
    prepare(c)
    c.portal.call(e.start)
    doc = store.load("factory-agv-15")
    assert c.post("/api/workspace/scenarios", json=doc).status_code == 409
    assert (
        c.post(
            "/api/workspace/prepare",
            json=dict(scenario_id=doc["id"], solver="joint_astar"),
        ).status_code
        == 409
    )
    assert c.post("/api/workspace/unlock").status_code == 409
    assert c.post("/api/workspace/speed", json={"factor": 10}).status_code == 200
    assert e.settings.dt == 0.05 and e.settings.realtime_factor == 10
    assert c.post("/api/workspace/speed", json={"factor": 20}).status_code == 400
    c.portal.call(e.stop)
    assert c.post("/api/workspace/scenarios", json=doc).status_code == 409
    assert c.post("/api/workspace/unlock").status_code == 200
    assert not e.recording.started
    assert c.post("/api/workspace/scenarios", json=doc).status_code == 200


def test_solver_descriptor_and_parameters_are_dynamic(workspace):
    c, e, store = workspace

    class Policy(MAPFSolver):
        name = "workspace_test_policy"

        def solve(self, problem):
            return MAPFSolution(
                {r: [s] for r, s in problem.starts.items()},
                success=False,
                status="timeout",
                metrics={"confidence": 0.7},
            )

    register_solver(
        Policy.name,
        Policy,
        parameters=[
            dict(
                key="device",
                type="enum",
                label="Устройство",
                default="cpu",
                options=["cpu", "gpu"],
            )
        ],
        metrics=[dict(key="confidence", label="Уверенность", unit="")],
    )
    item = next(
        s
        for s in c.get("/api/workspace/catalog").json()["solvers"]
        if s["id"] == Policy.name
    )
    assert item["parameters"][-1]["key"] == "device"
    result = c.post(
        "/api/workspace/prepare",
        json=dict(
            scenario_id="factory-agv-15",
            solver=Policy.name,
            parameters={"device": "gpu"},
        ),
    )
    assert result.status_code == 200, result.text
    assert e.traffic.solver.parameters["device"] == "gpu"
    for params in (
        {"device": "bad"},
        {"unknown": 1},
        {"horizon_steps": 1.5},
        {"time_limit_s": -1},
    ):
        assert (
            c.post(
                "/api/workspace/prepare",
                json=dict(
                    scenario_id="factory-agv-15", solver=Policy.name, parameters=params
                ),
            ).status_code
            == 400
        )


def test_finite_delivery_queue_respects_release_and_finishes(workspace):
    c, e, store = workspace
    d = store.load("factory-agv-15")
    d["fleet"]["robots"] = d["fleet"]["robots"][:1]
    a = d["wms"]["fixed_loops"]["assignments"][0]
    a["repeat_orders"] = False
    a["orders"] = [
        dict(
            pickup_index=3,
            dropoff_index=4,
            release_time=5,
            service_time_s=0.5,
            cargo_type="parts",
        ),
        dict(
            pickup_index=4,
            dropoff_index=5,
            release_time=20,
            service_time_s=0.5,
            cargo_type="paint",
        ),
    ]
    d["wms"]["fixed_loops"]["assignments"] = [a]
    doc = store.save(d)
    prepare(c, doc["id"], duration_s=60)

    async def advance():
        await e.start()
        for _ in range(80):
            await e.step()
        assert e.wms.orders[0].status == "scheduled"
        assert e.traffic.agents["AGV01"].distance_steps == 0
        for _ in range(1121):
            await e.step()

    c.portal.call(advance)
    assert len(e.wms.orders) == 2
    assert all(o.status == "delivered" for o in e.wms.orders)
    assert e.wms.orders[1].released_at >= 20
    assert e.traffic.agents["AGV01"].phase == "complete"


def test_report_loader_rejects_unknown_versions(tmp_path):
    path = tmp_path / "future.json"
    path.write_text(json.dumps(dict(kind="fleetsim.report", schema_version=9)))
    with pytest.raises(ValueError, match="Unsupported"):
        read_report(path)


def test_report_html_escapes_scenario_text(workspace):
    from backend.workspace.reports import report_html, build_report

    c, e, store = workspace
    prepare(c)
    e.recording.inputs["scenario"]["name"] = "<script>alert(1)</script>"
    doc = build_report(e, dict(metrics=e.metrics.to_dict(0), orders=[]))
    html = report_html(doc)
    assert "<script>alert(1)</script>" not in html
    assert "&lt;script&gt;" in html


def test_time_acceleration_preserves_physics_step():
    from types import SimpleNamespace
    from backend.core.loop import run_simulation_loop

    class Engine:
        def __init__(self):
            self.status = "running"
            self.settings = SimpleNamespace(dt=0.05, realtime_factor=10)
            self.recording = SimpleNamespace(id="test")
            self.sim_time = 0
            self.actual_realtime_factor = 0
            self.steps = []

        async def step(self):
            self.steps.append(self.settings.dt)
            self.sim_time += self.settings.dt

        async def snapshot(self):
            return {"time": self.sim_time}

    async def run():
        e = Engine()
        frames = []

        async def broadcast(frame):
            frames.append(frame)

        task = asyncio.create_task(run_simulation_loop(e, broadcast))
        try:
            await asyncio.sleep(0.25)
            assert e.sim_time > 1
            assert set(e.steps) == {0.05}
            assert 1 <= len(frames) <= 4
            e.status = "paused"
            await asyncio.sleep(0.04)
            at = e.sim_time
            await asyncio.sleep(0.1)
            assert e.sim_time == at
        finally:
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task

    asyncio.run(run())


def test_connection_control_is_recorded_in_history_and_report(workspace):
    c, e, store = workspace
    prepare(c)
    assert (
        c.post(
            "/api/workspace/planner-connection", json={"connected": False}
        ).status_code
        == 409
    )
    c.portal.call(e.start)
    assert (
        c.post(
            "/api/workspace/planner-connection", json={"connected": False}
        ).status_code
        == 200
    )

    async def advance():
        for _ in range(40):
            await e.step()

    c.portal.call(advance)
    r = c.post("/api/workspace/reports")
    assert r.status_code == 200
    doc = c.get(r.json()["json_url"]).json()
    assert doc["autonomy_events"][0]["reason"] == "connection_lost"
    assert doc["summary"]["autonomous_duration_s"] > 1.9
    assert doc["controls"][-1]["event"] == "planner_connection"
    assert doc["history"]["frames"][-1]["mapf"]["connected"] is False
    assert (
        c.post(
            "/api/workspace/planner-connection", json={"connected": "false"}
        ).status_code
        == 400
    )
    assert (
        c.post(
            "/api/workspace/planner-connection", json={"connected": True}
        ).status_code
        == 200
    )


def test_cbs_parameters_metrics_and_report_roundtrip(workspace):
    c, e, store = workspace
    params = dict(
        objective="sum_of_costs",
        max_wait_steps=8,
        max_ct_nodes=300,
        cache_low_level=False,
    )
    response = c.post(
        "/api/workspace/prepare",
        json=dict(
            scenario_id="factory-agv-15",
            solver="cbs_astar",
            parameters=params,
            duration_s=12,
        ),
    )
    assert response.status_code == 200, response.text

    async def advance():
        await e.start()
        for _ in range(241):
            await e.step()

    c.portal.call(advance)
    entries = c.get("/api/workspace/journal").json()["entries"]
    assert entries and all(x["solver"] == "cbs_astar" for x in entries)
    assert all(
        x["expanded"]
        == x["metrics"]["ct_expanded"] + x["metrics"]["low_level_expanded"]
        for x in entries
    )
    result = c.post("/api/workspace/reports")
    assert result.status_code == 200, result.text
    doc = c.get(result.json()["json_url"]).json()
    assert doc["inputs"]["solver"]["parameters"]["max_wait_steps"] == 8
    assert doc["inputs"]["solver"]["parameters"]["objective"] == "sum_of_costs"
    assert doc["summary"]["solver_metrics"]["ct_expanded"]["total"] == sum(
        x["metrics"]["ct_expanded"] for x in entries
    )
    html = c.get(result.json()["html_url"]).text
    assert "Раскрыто узлов CBS" in html
    assert c.get("/api/workspace/algorithm-help?solver=cbs_astar").status_code == 200


def test_mixed_preview_cbs_prepare_and_report(workspace):
    c, e, store = workspace
    relative = "examples/graphs/factory_amr_working.geojson"
    shutil.copyfile(ROOT / relative, store.root / relative)
    doc = json.loads((ROOT / "examples/scenarios/factory_mixed_20.json").read_text())
    response = c.post("/api/workspace/scenarios", json=doc)
    assert response.status_code == 200, response.text
    saved = response.json()
    preview = c.post(
        "/api/workspace/preview",
        json=dict(
            document=saved,
            map_asset=saved["map_asset"],
            graph_asset=saved["graph_asset"],
        ),
    )
    assert preview.status_code == 200, preview.text
    assert len(preview.json()["robot_loops"]) == 10
    assert len(preview.json()["amr_graph"]["nodes"]) > 1000
    prepared = c.post(
        "/api/workspace/prepare",
        json=dict(
            scenario_id=saved["id"],
            solver="cbs_astar",
            parameters=dict(horizon_steps=8),
            duration_s=5,
        ),
    )
    assert prepared.status_code == 200, prepared.text

    async def run():
        await e.start()
        for _ in range(70):
            await e.step()
        await e.stop()

    asyncio.run(run())
    assert e.traffic.calls > 0 and e.traffic.solver.name == "cbs_astar"
    response = c.post("/api/workspace/reports", json={})
    assert response.status_code == 200, response.text
    report = read_report(store.reports / (response.json()["id"] + ".json"))
    assert "research" in report["summary"]
    assert report["inputs"]["assets"]["amr_graph_asset"]["sha256"]
    assert c.get("/api/workspace/algorithm-help?solver=whca").status_code == 200


def test_upload_gml_retains_source_and_converts(workspace):
    c, e, store = workspace
    gml = "graph [ directed 1 node [ id 1 world 0 world 0 ] node [ id 2 world 5 world 0 ] edge [ source 1 target 2 ] ]"
    response = c.post(
        "/api/workspace/assets", files={"file": ("small.gml", gml, "text/plain")}
    )
    assert response.status_code == 200, response.text
    result = response.json()
    assert (store.root / result["original_asset"]).exists()
    assert (store.root / result["id"]).exists()
    transformed = c.post(
        "/api/workspace/convert-graph",
        json=dict(asset=result["id"], scale=2, rotation=90, offset=[10, 20]),
    )
    assert transformed.status_code == 200, transformed.text
    assert transformed.json()["id"] != result["id"]
