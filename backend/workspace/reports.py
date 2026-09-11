"""Version 1 JSON is the canonical report; HTML is a derived standalone view."""

from collections import Counter
from copy import deepcopy
from html import escape
import json
from statistics import mean
from math import isfinite
from uuid import uuid4

from .recording import utc_now
from .storage import write_json, document_id


def build_report(engine, metrics):
    journal = deepcopy(engine.recording.journal)
    durations = sorted(e["elapsed_s"] for e in journal)
    agents = engine.traffic.agents if engine.traffic else {}
    summary = dict(
        duration_sim_s=engine.sim_time,
        robot_count=len(engine.robots),
        route_count=len({a.route_id for a in agents.values()}),
        orders_delivered=metrics["metrics"]["orders_delivered"],
        collision_count=metrics["metrics"]["collision_count"],
        distance_total_m=sum(metrics["metrics"]["robot_distance_m"].values()),
        waiting_total_s=sum(a.waiting_s for a in agents.values()),
        solver_calls=len(journal),
        solver_mean_s=mean(durations) if durations else 0,
        solver_p95_s=(
            durations[min(len(durations) - 1, int(0.95 * (len(durations) - 1)))]
            if durations
            else 0
        ),
        solver_max_s=max(durations, default=0),
        solver_status_counts=dict(Counter(e["status"] for e in journal)),
    )
    autonomy = engine.traffic.autonomy_history() if engine.traffic else []
    summary["autonomous_duration_s"] = sum(e["duration_s"] for e in autonomy)
    summary["autonomy_reason_counts"] = dict(Counter(e["reason"] for e in autonomy))
    summary["solver_termination_counts"] = dict(
        Counter(
            e.get("metrics", {}).get("termination_reason", e["status"]) for e in journal
        )
    )
    summary.update(
        {
            f"solver_{status}_count": summary["solver_status_counts"].get(status, 0)
            for status in ("solved", "partial", "timeout", "infeasible")
        }
    )
    descriptors = (
        engine.recording.inputs.get("solver", {})
        .get("descriptor", {})
        .get("metrics", [])
    )
    summary["solver_metrics"] = {}
    for descriptor in descriptors:
        key = descriptor["key"]
        values = [e.get("metrics", {}).get(key) for e in journal]
        values = [
            v
            for v in values
            if isinstance(v, (int, float)) and not isinstance(v, bool) and isfinite(v)
        ]
        if values:
            summary["solver_metrics"][key] = dict(
                count=len(values), total=sum(values), mean=mean(values), max=max(values)
            )
    summary["throughput_per_sim_hour"] = (
        summary["orders_delivered"] * 3600 / engine.sim_time if engine.sim_time else 0
    )
    delivery_times = [
        o["delivered_at"] - o["released_at"]
        for o in metrics["orders"]
        if o["status"] == "delivered" and o["released_at"] is not None
    ]
    summary["mean_delivery_s"] = mean(delivery_times) if delivery_times else None
    if getattr(engine.traffic, "temporal", False):
        from backend.research.metrics import measure, objective

        study = measure(engine)
        summary["research"] = dict(metrics=study, objective=objective(study))
    robots = [
        dict(
            id=rid,
            route_id=a.route_id,
            delivered=a.delivered,
            waiting_s=a.waiting_s,
            autonomous_s=a.autonomous_s,
            junction_wait_s=a.junction_wait_s,
            distance_m=metrics["metrics"]["robot_distance_m"].get(rid, 0),
        )
        for rid, a in agents.items()
    ]
    return dict(
        kind="fleetsim.report",
        schema_version=1,
        report_id=uuid4().hex,
        run_id=engine.recording.id,
        created_at=utc_now(),
        run_created_at=engine.recording.created_at,
        completion="complete" if engine.status == "finished" else "partial",
        status=engine.status,
        inputs=deepcopy(engine.recording.inputs),
        summary=summary,
        robots=robots,
        metrics=metrics["metrics"],
        orders=metrics["orders"],
        solver_journal=journal,
        autonomy_events=autonomy,
        collision_context=(
            deepcopy(engine.traffic.collision_context) if engine.traffic else []
        ),
        controls=deepcopy(engine.recording.controls),
        history=dict(
            interval_s=engine.recording.interval_s,
            frames=deepcopy(engine.recording.frames),
        ),
        scene=dict(
            map=engine.dxf_map.to_dict() if engine.dxf_map else None,
            graph=engine.graph.to_dict() if engine.graph else None,
        ),
    )


def read_report(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("kind") != "fleetsim.report" or data.get("schema_version") != 1:
        raise ValueError("Unsupported report format/version")
    for key in (
        "report_id",
        "run_id",
        "inputs",
        "summary",
        "solver_journal",
        "completion",
        "history",
        "scene",
        "metrics",
        "orders",
        "robots",
    ):
        if key not in data:
            raise ValueError(f"Missing report field: {key}")
    if (
        not isinstance(data["inputs"], dict)
        or not isinstance(data["summary"], dict)
        or not isinstance(data["solver_journal"], list)
    ):
        raise ValueError("Invalid report structure")
    if data["completion"] not in ("complete", "partial"):
        raise ValueError("Invalid report completion status")
    for key in (
        "duration_sim_s",
        "robot_count",
        "route_count",
        "orders_delivered",
        "collision_count",
        "solver_calls",
        "solver_mean_s",
        "solver_p95_s",
        "solver_max_s",
    ):
        value = data["summary"].get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (float, int))
            or not isfinite(value)
            or value < 0
        ):
            raise ValueError(f"Invalid report summary field: {key}")
    return data


def report_html(report):
    def table(headers, rows):
        return (
            "<table><thead><tr>"
            + "".join("<th>" + escape(str(h)) + "</th>" for h in headers)
            + "</tr></thead><tbody>"
            + "".join(
                "<tr>"
                + "".join("<td>" + escape(str(v)) + "</td>" for v in row)
                + "</tr>"
                for row in rows
            )
            + "</tbody></table>"
        )

    summary = report["summary"]
    events = report["solver_journal"]
    title = report["inputs"].get("scenario", {}).get("name", "Запуск симуляции")
    max_ms = max((e["elapsed_s"] * 1000 for e in events), default=1) or 1
    points = " ".join(
        f'{30+940*i/max(1,len(events)-1):.1f},{190-e["elapsed_s"]*1000/max_ms*160:.1f}'
        for i, e in enumerate(events)
    )
    chart = f'<svg viewBox="0 0 1000 220" role="img" aria-label="Длительность вызовов solver в миллисекундах"><path d="M30 20V190H980" stroke="#cbd5e1" fill="none"/><polyline points="{points}" fill="none" stroke="#167f78" stroke-width="2"/><text x="30" y="15">{max_ms:.1f} мс</text><text x="30" y="215">Вызовы планирования →</text></svg>'
    cards = "".join(
        f"<div><span>{label}</span><strong>{value}</strong></div>"
        for label, value in [
            ("Симуляция", f'{summary["duration_sim_s"]:.1f} с'),
            ("Роботы", summary["robot_count"]),
            ("Доставки", summary["orders_delivered"]),
            ("Столкновения", summary["collision_count"]),
            ("Поиск · среднее", f'{1000*summary["solver_mean_s"]:.2f} мс'),
            ("Поиск · p95", f'{1000*summary["solver_p95_s"]:.2f} мс'),
        ]
    )
    robots = table(
        ["AGV", "Петля", "Доставки", "Путь, м", "Ожидание, с"],
        [
            (
                r["id"],
                r["route_id"],
                r["delivered"],
                round(r["distance_m"], 2),
                round(r["waiting_s"], 2),
            )
            for r in report["robots"]
        ],
    )
    calls = table(
        ["№", "Время, с", "Результат", "Поиск, мс", "Раскрыто", "Стоимость"],
        [
            (
                e["id"],
                round(e["time"], 2),
                e["status"],
                round(e["elapsed_s"] * 1000, 2),
                e["expanded"],
                e["cost"],
            )
            for e in events[-100:]
        ],
    )
    autonomy_table = table(
        ["Начало, с", "Конец, с", "Длительность, с", "Причина"],
        [
            (round(e["start_s"], 2), e["end_s"], round(e["duration_s"], 2), e["reason"])
            for e in report.get("autonomy_events", [])
        ],
    )
    metric_labels = {
        m["key"]: m.get("label", m["key"])
        for m in report["inputs"]
        .get("solver", {})
        .get("descriptor", {})
        .get("metrics", [])
    }
    solver_metrics_table = table(
        ["Метрика", "Измерений", "Сумма", "Среднее", "Максимум"],
        [
            (
                metric_labels.get(key, key),
                values["count"],
                round(values["total"], 3),
                round(values["mean"], 3),
                round(values["max"], 3),
            )
            for key, values in summary.get("solver_metrics", {}).items()
        ],
    )
    research = summary.get("research", {})
    research_table = (
        (
            "<h2>Исследовательский критерий</h2>"
            + table(["Метрика", "Значение"], list(research.get("metrics", {}).items()))
            + "<pre>"
            + escape(
                json.dumps(research.get("objective", {}), ensure_ascii=False, indent=2)
            )
            + "</pre>"
        )
        if research
        else ""
    )
    settings = escape(
        json.dumps(report["inputs"].get("solver", {}), ensure_ascii=False, indent=2)
    )
    return f"""<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{escape(title)} — отчёт</title>
<style>body{{font:15px system-ui;color:#26354a;background:#f1f5f7;margin:0}}main{{max-width:1100px;margin:40px auto;background:white;padding:40px;border-radius:18px}}h1{{font-size:30px}}h2{{margin-top:36px}}p,span{{color:#64748b}}.cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}}.cards div{{background:#edf7f5;padding:20px;border-radius:12px}}strong{{display:block;font-size:28px;margin-top:8px}}table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{text-align:left;padding:10px;border-bottom:1px solid #e2e8f0}}th{{background:#f8fafc}}svg{{width:100%;font:12px system-ui}}pre{{white-space:pre-wrap;background:#f8fafc;padding:18px}}@media print{{body{{background:white}}main{{margin:0;padding:0}}}}@media(max-width:650px){{main{{margin:0;padding:15px}}.cards{{grid-template-columns:1fr 1fr}}}}</style>
<main><p>FLEET SIM · ОТЧЁТ ЗАПУСКА · v1</p><h1>{escape(title)}</h1>
<p>{escape(report['created_at'])} · {'Завершённый' if report['completion']=='complete' else 'Промежуточный'} отчёт</p>
<p>Run ID: {escape(report['run_id'])}<br>Report ID: {escape(report['report_id'])}</p>
<section class="cards">{cards}</section>{research_table}<h2>Работа solver’а</h2>{chart}<pre>{settings}</pre><h2>Метрики выбранного solver</h2>{solver_metrics_table}
<h2>Автономное движение</h2><p>Причины: connection_lost — потеря связи; expansion_limit — лимит раскрытий; time_limit — лимит времени; infeasible / exhausted — нет допустимого плана; execution_rejected — отклонение при исполнении; ct_node_limit — лимит узлов CBS; wait_limit — нет плана в пределах допустимых ожиданий. Пять секунд остановки не гарантируют отсутствия столкновений.</p>{autonomy_table}<h2>Роботы</h2>{robots}<h2>Последние 100 вызовов</h2>{calls}
<p>Полная история, все вызовы и исходные настройки находятся в JSON с тем же идентификатором отчёта. Единицы измерения указаны в названиях полей; время поиска измерено в реальном времени, движение — в симуляционном.</p></main></html>"""


def save_report(store, report):
    rid = document_id(report["report_id"])
    write_json(store.reports / (rid + ".json"), report)
    path = store.reports / (rid + ".html")
    path.write_text(report_html(report), encoding="utf-8")
    return dict(
        id=rid,
        json_url=f"/api/workspace/reports/{rid}.json",
        html_url=f"/api/workspace/reports/{rid}.html",
        summary=report["summary"],
    )
