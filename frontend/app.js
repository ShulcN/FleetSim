import { MapView } from "./map.js";
const $ = (id) => document.getElementById(id);
const esc = (value) =>
  String(value ?? "").replace(
    /[&<>"']/g,
    (c) =>
      ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[
        c
      ],
  );
const clone = (value) => structuredClone(value);
const labels = {
  running: "В работе",
  stopped: "Остановлен",
  paused: "Пауза",
  finished: "Завершён",
  collision_stopped: "Столкновение",
  solved: "Решено",
  partial: "Частичный план",
  timeout: "Лимит поиска",
  infeasible: "Нет решения",
  to_pickup: "К загрузке",
  to_dropoff: "К выгрузке",
  loading: "Загрузка",
  unloading: "Выгрузка",
  waiting_release: "Ожидание заказа",
  complete: "Завершено",
  idle: "Свободен",
  scheduled: "Запланирован",
  assigned: "Назначен",
  picked: "Груз забран",
  delivered: "Доставлен",
};
const reasons = {
  granted: "Движение разрешено",
  window_complete: "Окно обработано",
  no_path: "План не найден",
  no_path_for_agent: "Не найдено разрешение для робота",
  connection_lost: "Потеря связи",
  expansion_limit: "Лимит раскрытий",
  ct_node_limit: "Лимит узлов CBS",
  wait_limit: "Нет полного плана при заданном пределе ожиданий",
  time_limit: "Лимит времени поиска",
  exhausted: "Поиск исчерпан: план не найден",
  infeasible: "Конфликт исходных позиций или ресурсов",
  junction_stop: "Автономная остановка: 5 с",
  goal_reached: "Цели окна достигнуты",
  solver_wait: "Solver выбрал ожидание",
  no_safe_prefix: "Нет безопасного фрагмента плана",
  execution_rejected: "Отклонено проверкой фактических позиций",
  halted: "AGV остановлен",
  loading: "Загрузка",
  unloading: "Выгрузка",
  complete: "Очередь завершена",
  waiting_release: "Время заказа ещё не наступило",
};
let catalog,
  draft,
  scene,
  live,
  viewed = null,
  view = "simulation",
  selected = null,
  dirty = false,
  parameters = {},
  journal = [],
  selectedEvent = null,
  currentRun = null,
  journalBusy = false,
  seekSerial = 0,
  lastUI = 0,
  activeAnalysis = "solver",
  orderPlacement = 0;
const map = new MapView(
  $("mapCanvas"),
  (id) => {
    selected = id;
    map.selected = id;
    renderInspector();
    map.draw();
  },
  placeRobot,
);
function toast(text, error = false) {
  $("toast").textContent = text;
  $("toast").classList.toggle("error", error);
  $("toast").hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(
    () => ($("toast").hidden = true),
    error ? 8000 : 3500,
  );
}
async function api(path, data, method) {
  const response = await fetch("/api/" + path, {
    method: method || (data === undefined ? "GET" : "POST"),
    headers: data === undefined ? {} : { "Content-Type": "application/json" },
    body: data === undefined ? undefined : JSON.stringify(data),
  });
  if (!response.ok) {
    let msg = await response.text();
    try {
      const d = JSON.parse(msg);
      msg = typeof d.detail === "string" ? d.detail : JSON.stringify(d.detail);
    } catch {}
    throw new Error(msg);
  }
  return response.json();
}
function action(fn) {
  return async (...args) => {
    try {
      await fn(...args);
    } catch (e) {
      toast(e.message, true);
    }
  };
}
function started() {
  return !!live?.recording?.started;
}
function fmt(t) {
  t = Math.max(0, t || 0);
  return `${String(Math.floor(t / 60)).padStart(2, "0")}:${(t % 60).toFixed(1).padStart(4, "0")}`;
}
function num(v, d = 2) {
  return Number(v || 0).toFixed(d);
}
function option(value, label, current) {
  return `<option value="${esc(value)}" ${String(value) === String(current) ? "selected" : ""}>${esc(label)}</option>`;
}
function field(label, value, attr, type = "number") {
  return `<label>${esc(label)}<input type="${type}" value="${esc(value)}" ${attr}></label>`;
}
function kpi(label, value, accent = false) {
  return `<div class="kpi ${accent ? "accent" : ""}"><span>${esc(label)}</span><strong>${esc(value)}</strong></div>`;
}
function assignment(rid) {
  return draft?.wms.fixed_loops.assignments.find((a) => a.robot_id === rid);
}
function normalize(d) {
  if (d.wms.fixed_loops.mode === "mixed") return d;
  for (const a of d.wms.fixed_loops.assignments) {
    a.orders ??= [
      {
        pickup_index: a.pickup_index,
        dropoff_index: a.dropoff_index,
        release_time: 0,
        service_time_s: d.wms.fixed_loops.service_time_s || 3,
        cargo_type: a.route_id,
      },
    ];
    a.repeat_orders ??= true;
  }
  return d;
}
async function refreshCatalog() {
  catalog = await api("workspace/catalog");
  const sid = draft?.id || $("scenarioSelect").value;
  const solver = $("solverSelect").value;
  $("scenarioSelect").innerHTML = catalog.scenarios
    .map((s) => option(s.id, s.name, sid))
    .join("");
  $("solverSelect").innerHTML = catalog.solvers
    .map((s) => option(s.id, s.label, solver || "joint_astar"))
    .join("");
  renderLibrary();
}
async function loadScenario(id) {
  draft = normalize(await api("workspace/scenarios/" + encodeURIComponent(id)));
  dirty = false;
  selected = null;
  selectedEvent = null;
  $("scenarioSelect").value = id;
  $("duration").value = draft.simulation.duration_s;
  $("solverSelect").value = draft.wms.fixed_loops.solver || "joint_astar";
  parameters[$("solverSelect").value] = clone(
    draft.wms.fixed_loops.solver_parameters || {},
  );
  await preview();
  renderLibrary();
  renderInspector();
  updateUI();
}
async function preview() {
  scene = await api("workspace/preview", {
    document: isMixed() ? draft : undefined,
    map_asset: draft.map_asset,
    graph_asset: draft.graph_asset,
    cell_size_m: draft.wms.fixed_loops.cell_size_m || 5,
  });
  map.setScene(scene);
  renderMap();
}
function previewState() {
  return {
    robots:
      draft?.fleet.robots.map((r) => ({
        id: r.id,
        ...r.initial_pose,
        length: r.footprint.length,
        width: r.footprint.width,
        color: r.color,
        status: "idle",
      })) || [],
    mapf: {
      agents: Object.fromEntries(
        (draft?.wms.fixed_loops.assignments || []).map((a) => [
          a.robot_id,
          { route_id: a.route_id },
        ]),
      ),
    },
  };
}
function displayed() {
  return view === "scenarios" || !started() ? previewState() : viewed || live;
}
function renderMap() {
  map.state = displayed();
  map.editor = view === "scenarios" && !started();
  map.selected = selected;
  map.plan = view === "simulation" ? selectedEvent : null;
  map.stations = [];
  if (selected) {
    const a = assignment(selected);
    const task =
      view === "scenarios"
        ? a?.orders?.[orderPlacement] || a?.orders?.[0]
        : null;
    if (task) {
      for (const [kind, key] of [
        ["З", "pickup_index"],
        ["В", "dropoff_index"],
      ]) {
        const p = scene?.loops?.[a.route_id]?.[task[key]];
        if (p) map.stations.push({ ...p, label: kind });
      }
    } else {
      const r = map.state?.robots?.find((r) => r.id === selected),
        orders = viewed?.orders || live?.wms?.orders || [],
        o = orders.find((o) => o.id === r?.active_order_id);
      if (o)
        for (const [kind, key] of [
          ["З", "pickup_node"],
          ["В", "dropoff_node"],
        ]) {
          const p = map.nodes.get(o[key]);
          if (p) map.stations.push({ ...p, label: kind });
        }
    }
  }
  if (isMixed()) {
    const stream = draft.wms.fixed_loops.streams.find(
      (s) => s.robot_id === selected,
    );
    if (stream)
      for (const [key, label] of [
        ["pickup_index", "З"],
        ["dropoff_index", "В"],
      ]) {
        const p = scene.robot_loops?.[selected]?.[stream[key]];
        if (p) map.stations.push({ ...p, label });
      }
    for (const s of draft.wms.fixed_loops.stations) {
      const p = scene?.amr_graph?.nodes.find((p) => p.id === s.node_id);
      if (p) map.stations.push({ ...p, label: s.id });
    }
  }
  map.draw();
  $("mapTitle").textContent = draft?.name || "Карта";
  $("mapMode").textContent =
    view === "scenarios"
      ? "РЕДАКТОР СЦЕНАРИЯ"
      : viewed
        ? "ПРОСМОТР ИСТОРИИ"
        : "СИМУЛЯЦИЯ";
  $("mapSubtitle").textContent =
    `${map.state?.robots?.length || 0} роботов · ${scene?.graph?.routes?.length || 0} фиксированных петель`;
  $("mapHint").textContent = map.placement
    ? `Выберите на карте точку: ${map.placement === "pickup" ? "загрузка" : map.placement === "dropoff" ? "выгрузка" : "начальное положение"}`
    : map.editor
      ? "Перетащите робота — привязка к его графу · клик — выбор"
      : "Колесо — масштаб · перетаскивание — панорама · клик — выбор робота";
  $("mapLegend").innerHTML = (scene?.graph?.routes || [])
    .map(
      (r) =>
        `<span><i class="swatch" style="background:${/^#[0-9a-f]{6}$/i.test(r.color) ? r.color : "#567"}"></i>${esc(r.route_id)}</span>`,
    )
    .join("");
}
function setView(next) {
  if (next === "scenarios" && started()) {
    toast("Нажмите «К подготовке», чтобы редактировать сценарии.", true);
    return;
  }
  view = next;
  map.placement = null;
  $("analysis").hidden = view === "scenarios";
  document.querySelector(".transport").hidden = view === "scenarios";
  $("library").hidden = view !== "scenarios";
  $("scenariosTab").classList.toggle("active", view === "scenarios");
  $("simulationTab").classList.toggle("active", view === "simulation");
  $("inspector").hidden = false;
  $("openInspector").hidden = true;
  renderInspector();
  renderMap();
  setTimeout(() => map.fit(), 30);
}
function renderLibrary() {
  if (!catalog) return;
  $("scenarioCards").innerHTML = catalog.scenarios
    .map(
      (s) =>
        `<button class="scenario-card ${s.id === draft?.id ? "active" : ""}" data-scenario="${esc(s.id)}"><strong>${esc(s.name)}</strong><small>${s.robots} роботов · ${esc((s.updated_at || "").slice(0, 10))}</small></button>`,
    )
    .join("");
  $("scenarioCards")
    .querySelectorAll("[data-scenario]")
    .forEach(
      (b) =>
        (b.onclick = action(async () => {
          if (
            dirty &&
            !confirm(
              "Оставить несохранённые изменения и открыть другой сценарий?",
            )
          )
            return;
          await loadScenario(b.dataset.scenario);
        })),
    );
}
function markDirty() {
  dirty = true;
  $("runLabel").textContent = "Есть несохранённые изменения";
  renderMap();
}
async function saveScenario() {
  draft.simulation.duration_s = Number($("duration").value);
  const path =
    "workspace/scenarios" +
    (draft.id ? "/" + encodeURIComponent(draft.id) : "");
  draft = normalize(await api(path, draft, draft.id ? "PUT" : "POST"));
  dirty = false;
  await refreshCatalog();
  $("scenarioSelect").value = draft.id;
  renderInspector();
  updateUI();
  toast("Сценарий сохранён");
}
function download(data, name) {
  const url = URL.createObjectURL(
    new Blob([JSON.stringify(data, null, 2)], { type: "application/json" }),
  );
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function solverMessage(message) {
  if (message?.startsWith("CBS search stopped: ")) {
    const reason = message.slice("CBS search stopped: ".length);
    return "Поиск CBS завершён: " + (reasons[reason] || reason);
  }
  return (
    {
      "Window goals reached": "Цели текущего окна достигнуты",
      "Planning limit reached": "Достигнут лимит поиска",
      "Window goals blocked": "Цели текущего окна заблокированы",
      "Actual-position safety check prevented execution":
        "Движение отклонено проверкой фактических положений",
      "Initial positions or resource reservations conflict":
        "Конфликт начальных позиций или резервирований",
    }[message] || message
  );
}
function renderInspector() {
  if (!draft) return;
  if (view === "scenarios") {
    renderEditor();
    return;
  }
  $("inspectorTitle").textContent = selectedEvent
    ? `Решение #${selectedEvent.id}`
    : selected || "Обзор запуска";
  const state = displayed();
  const agents = state?.mapf?.agents || {};
  if (selectedEvent) {
    const e = selectedEvent;
    $("inspectorBody").innerHTML =
      `<div class="kpis">${kpi("Время поиска", num(e.elapsed_s * 1000) + " мс", true)}${kpi("Раскрыто", e.expanded)}</div><dl class="detail-list"><dt>Момент</dt><dd>${fmt(e.time)}</dd><dt>Результат</dt><dd>${esc(labels[e.status] || e.status)}</dd><dt>Стоимость</dt><dd>${num(e.cost, 0)}</dd></dl><p class="notice">${esc(solverMessage(e.message))} · ${esc(reasons[e.metrics?.termination_reason] || e.metrics?.termination_reason || "")}</p><h3>Решения для роботов</h3>${Object.entries(
        e.decisions,
      )
        .map(
          ([id, d]) =>
            `<button class="robot-row" data-robot="${esc(id)}"><span>${esc(id)}</span><small>${esc(reasons[d.reason] || d.reason)}</small></button>`,
        )
        .join("")}<h3>Диагностика</h3><pre class="code">${esc(
        JSON.stringify(
          {
            metrics: e.metrics || {},
            execution_rejection: e.execution_rejection,
            observed_blockers: Object.fromEntries(
              Object.entries(e.decisions)
                .filter(([, d]) => d.observed_blockers?.length)
                .map(([r, d]) => [r, d.observed_blockers]),
            ),
          },
          null,
          2,
        ),
      )}</pre><button id="clearEvent" class="secondary">Снять выбор решения</button>`;
    $("clearEvent").onclick = () => {
      selectedEvent = null;
      renderInspector();
      renderMap();
    };
  } else if (selected) {
    const r = state.robots?.find((r) => r.id === selected);
    if (!r) {
      selected = null;
      renderInspector();
      return;
    }
    const a = agents[selected] || {};
    $("inspectorBody").innerHTML =
      `<span class="pill">${esc(labels[r.status] || r.status)}</span><dl class="detail-list"><dt>Петля</dt><dd>${esc(a.route_id || "—")}</dd><dt>Скорость</dt><dd>${num(r.v)} м/с</dd><dt>Положение</dt><dd>${num(r.x, 1)}, ${num(r.y, 1)}</dd><dt>Габариты</dt><dd>${r.length} × ${r.width} м</dd><dt>Заказ</dt><dd>${esc(r.active_order_id || "—")}</dd><dt>Доставки</dt><dd>${a.delivered || 0}</dd><dt>Ожидание</dt><dd>${num(a.waiting_s, 1)} с</dd><dt>Управление</dt><dd>${esc(reasons[a.control_reason] || "Центральное")}</dd><dt>Остановка перед проездом</dt><dd>${num(a.junction_wait_remaining_s || 0, 1)} с осталось</dd><dt>Автономно</dt><dd>${num(a.autonomous_s || 0, 1)} с</dd></dl><h3>Текущие разрешения</h3><pre class="code">${esc(JSON.stringify(state.mapf?.reservations?.[selected] || [], null, 2))}</pre><button id="clearRobot" class="secondary">Все роботы</button>`;
    $("clearRobot").onclick = () => {
      selected = null;
      renderInspector();
      renderMap();
    };
  } else {
    $("inspectorBody").innerHTML =
      `<div class="kpis">${kpi("Роботы", state?.robots?.length || 0)}${kpi("Доставки", state?.metrics_summary?.orders_delivered || 0, true)}${kpi("Столкновения", state?.metrics_summary?.collision_count || 0)}${kpi("Вызовы solver", state?.mapf?.calls || 0)}</div><h3>Флот</h3>${(state?.robots || []).map((r) => `<button class="robot-row" data-robot="${esc(r.id)}"><span><i class="dot" style="background:${/^#[0-9a-f]{6}$/i.test(r.color) ? r.color : "#567"}"></i>${esc(r.id)}</span><small>${esc(labels[r.status] || r.status)} · ${esc(agents[r.id]?.route_id || "")}</small></button>`).join("")}<p class="notice">Выберите робота на карте для просмотра состояния. Выберите вызов в журнале, чтобы увидеть его план и разрешения.</p>`;
  }
  $("inspectorBody")
    .querySelectorAll("[data-robot]")
    .forEach(
      (b) =>
        (b.onclick = () => {
          selected = b.dataset.robot;
          map.selected = selected;
          if (!selectedEvent) renderInspector();
          renderMap();
        }),
    );
}
function renderEditor() {
  if (isMixed()) return renderMixedEditor();
  const a = assignment(selected),
    robot = draft.fleet.robots.find((r) => r.id === selected);
  $("inspectorTitle").textContent = "Редактор сценария";
  $("inspectorBody").innerHTML =
    `<div class="form-stack"><label>Название<input id="scenarioName" value="${esc(draft.name)}"></label><label>Описание<textarea id="scenarioDescription" rows="2">${esc(draft.description || "")}</textarea></label><label>Карта<select id="mapAsset">${catalog.assets.maps.map((x) => option(x.id, x.name, draft.map_asset)).join("")}</select></label><label>Граф маршрутов<select id="graphAsset">${catalog.assets.graphs.map((x) => option(x.id, x.name, draft.graph_asset)).join("")}</select></label><label class="check-row"><input id="stopOnCollision" type="checkbox" ${draft.simulation.collision_mode === "stop_on_collision" ? "checked" : ""}> Останавливать при столкновении</label><small class="muted">Если выключено, столкновения записываются в отчёт, а симуляция продолжается. Настройка сохраняется в сценарии и меняется до запуска.</small></div><div class="form-actions"><button id="saveScenario" class="primary">Сохранить</button><button id="copyScenario">Копия</button><button id="exportScenario">↓ JSON</button></div><div class="editor-section"><div class="panel-heading"><h2>Роботы</h2><button id="addRobot">＋</button></div><select id="editRobot" class="robot-select">${option("", "Выберите AGV", selected)}${draft.fleet.robots.map((r) => option(r.id, r.id + " · " + assignment(r.id).route_id, selected)).join("")}</select>${robot ? robotForm(robot, a) : '<p class="notice">Выберите робота на карте или добавьте нового. Начальное положение меняется перетаскиванием с привязкой к назначенной петле.</p>'}</div>`;
  const migrateButton = document.createElement("button");
  migrateButton.textContent = "Перевести в новую кинематику AGV";
  migrateButton.onclick = action(async () => {
    draft = await api("workspace/migrate", { document: draft, period_s: 100 });
    selected = null;
    markDirty();
    await preview();
    renderEditor();
    renderMap();
  });
  $("inspectorBody").prepend(migrateButton);
  $("scenarioName").onchange = (e) => {
    draft.name = e.target.value;
    markDirty();
  };
  $("scenarioDescription").onchange = (e) => {
    draft.description = e.target.value;
    markDirty();
  };
  $("stopOnCollision").onchange = (e) => {
    draft.simulation.collision_mode = e.target.checked
      ? "stop_on_collision"
      : "count_only";
    markDirty();
  };
  $("saveScenario").onclick = action(saveScenario);
  $("copyScenario").onclick = () => {
    draft = clone(draft);
    delete draft.id;
    draft.name += " · копия";
    markDirty();
    renderEditor();
  };
  $("exportScenario").onclick = () =>
    download(draft, (draft.id || "scenario") + ".json");
  $("editRobot").onchange = (e) => {
    selected = e.target.value || null;
    map.placement = null;
    renderEditor();
    renderMap();
  };
  $("addRobot").onclick = () => addRobot();
  for (const [id, key] of [
    ["mapAsset", "map_asset"],
    ["graphAsset", "graph_asset"],
  ])
    $(id).onchange = action(async (e) => {
      const old = draft[key];
      draft[key] = e.target.value;
      try {
        await preview();
        markDirty();
      } catch (err) {
        draft[key] = old;
        $(id).value = old;
        throw err;
      }
    });
  if (!robot) return;
  $("robotRoute").onchange = (e) => {
    a.route_id = e.target.value;
    const n = scene.loops[a.route_id].length;
    a.start_index = 0;
    a.pickup_index = 0;
    a.dropoff_index = Math.min(4, n - 1);
    a.orders = [
      {
        pickup_index: 0,
        dropoff_index: a.dropoff_index,
        release_time: 0,
        service_time_s: 3,
        cargo_type: a.route_id,
      },
    ];
    positionAt(robot, a, freeIndex(a.route_id));
    markDirty();
    renderEditor();
  };
  $("robotName").onchange = (e) => {
    const name = e.target.value.trim();
    if (!name || draft.fleet.robots.some((r) => r !== robot && r.id === name)) {
      toast("ID должен быть непустым и уникальным", true);
      e.target.value = robot.id;
      return;
    }
    a.robot_id = robot.id = selected = name;
    markDirty();
    renderEditor();
  };
  $("inspectorBody")
    .querySelectorAll("[data-robotfield]")
    .forEach(
      (input) =>
        (input.onchange = () => {
          const [group, key] = input.dataset.robotfield.split(".");
          robot[group][key] = Number(input.value);
          markDirty();
        }),
    );
  $("startIndex").onchange = (e) => {
    positionAt(robot, a, Number(e.target.value));
    markDirty();
  };
  $("repeatOrders").onchange = (e) => {
    a.repeat_orders = e.target.checked;
    markDirty();
  };
  $("duplicateRobot").onclick = () => addRobot(robot);
  $("removeRobot").onclick = () => {
    draft.fleet.robots = draft.fleet.robots.filter((r) => r !== robot);
    draft.wms.fixed_loops.assignments =
      draft.wms.fixed_loops.assignments.filter((x) => x !== a);
    selected = null;
    markDirty();
    renderEditor();
  };
  $("addOrder").onclick = () => {
    a.orders.push(clone(a.orders.at(-1)));
    markDirty();
    renderEditor();
  };
  $("inspectorBody")
    .querySelectorAll("[data-orderfield]")
    .forEach(
      (input) =>
        (input.onchange = () => {
          const i = Number(input.dataset.order);
          a.orders[i][input.dataset.orderfield] =
            input.type === "text" ? input.value : Number(input.value);
          a.pickup_index = a.orders[0].pickup_index;
          a.dropoff_index = a.orders[0].dropoff_index;
          markDirty();
        }),
    );
  $("inspectorBody")
    .querySelectorAll("[data-removeorder]")
    .forEach(
      (b) =>
        (b.onclick = () => {
          if (a.orders.length <= 1) {
            toast("Оставьте хотя бы одну доставку", true);
            return;
          }
          a.orders.splice(Number(b.dataset.removeorder), 1);
          a.pickup_index = a.orders[0].pickup_index;
          a.dropoff_index = a.orders[0].dropoff_index;
          markDirty();
          renderEditor();
        }),
    );
  $("inspectorBody")
    .querySelectorAll("[data-place]")
    .forEach(
      (b) =>
        (b.onclick = () => {
          map.placement = b.dataset.place;
          orderPlacement = Number(b.dataset.order || 0);
          toast("Выберите точку на назначенной петле");
          renderMap();
        }),
    );
}
function robotForm(r, a) {
  const points = scene?.loops[a.route_id] || [],
    routes = Object.keys(scene?.loops || {});
  return `<div class="form-grid">${field("ID", r.id, 'id="robotName"', "text")}<label>Петля<select id="robotRoute">${routes.map((id) => option(id, id, a.route_id)).join("")}</select></label>${field("Длина, м", r.footprint.length, 'data-robotfield="footprint.length" min="0.1" step="0.1"')}${field("Ширина, м", r.footprint.width, 'data-robotfield="footprint.width" min="0.1" step="0.1"')}${field("Макс. скорость, м/с", r.parameters.max_linear, 'data-robotfield="parameters.max_linear" min="0.1" step="0.1"')}${field("Крейсерская, м/с", r.route_follower.max_linear, 'data-robotfield="route_follower.max_linear" min="0.1" step="0.1"')}${field("Начальная точка", a.start_index, `id="startIndex" min="0" max="${points.length - 1}" step="1"`)}<button data-place="start">Указать на карте</button></div><div class="form-actions"><button id="duplicateRobot">Дублировать AGV</button><button id="removeRobot" class="danger">Удалить</button></div><h3>Очередь доставок</h3><label class="check-row"><input id="repeatOrders" type="checkbox" ${a.repeat_orders ? "checked" : ""}> Повторять очередь</label>${a.orders.map((o, i) => `<div class="delivery-card"><div class="panel-heading"><strong>Доставка ${i + 1}</strong><button class="quiet" data-removeorder="${i}" title="Удалить доставку">×</button></div><div class="form-grid">${field("Загрузка · индекс", o.pickup_index, `data-order="${i}" data-orderfield="pickup_index" min="0" max="${points.length - 1}" step="1"`)}${field("Выгрузка · индекс", o.dropoff_index, `data-order="${i}" data-orderfield="dropoff_index" min="0" max="${points.length - 1}" step="1"`)}<button data-place="pickup" data-order="${i}">На карте ↗</button><button data-place="dropoff" data-order="${i}">На карте ↗</button>${field("Выпуск, с", o.release_time, `data-order="${i}" data-orderfield="release_time" min="0"`)}${field("Погрузка / разгрузка, с на каждую", o.service_time_s, `data-order="${i}" data-orderfield="service_time_s" min="0" step="0.5"`)}${field("Тип груза", o.cargo_type, `data-order="${i}" data-orderfield="cargo_type"`, "text")}</div></div>`).join("")}<button id="addOrder" class="secondary">＋ Доставка</button><p class="muted">Индексы точек: 0–${points.length - 1}. Выпуск — абсолютное время симуляции. Погрузка / разгрузка — две отдельные остановки указанной длительности (3 с = 3 с погрузки + 3 с разгрузки). Проверка размещения выполняется при сохранении.</p>`;
}
function positionAt(r, a, index) {
  const points = scene.loops[a.route_id];
  index = Math.round(index);
  if (index < 0 || index >= points.length) return;
  const p = points[index],
    next = points[(index + 1) % points.length];
  a.start_index = index;
  r.initial_pose = {
    x: p.x,
    y: p.y,
    theta: Math.atan2(next.y - p.y, next.x - p.x),
  };
}
function freeIndex(route) {
  const points = scene.loops[route];
  let best = 0;
  for (let i = 0; i < points.length; i++)
    if (
      draft.fleet.robots.every(
        (r) =>
          Math.hypot(
            r.initial_pose.x - points[i].x,
            r.initial_pose.y - points[i].y,
          ) > 6,
      )
    ) {
      best = i;
      break;
    }
  return best;
}
function addRobot(source) {
  const r = clone(
    source ||
      draft.fleet.robots[0] || {
        id: "AGV01",
        type: "differential_drive",
        color: "#248b79",
        footprint: { length: 1.5, width: 0.6 },
        parameters: { max_linear: 1, max_angular: 1.5 },
        route_follower: {
          max_linear: 0.7,
          max_angular: 1.4,
          waypoint_tolerance: 0.02,
          stop_and_turn_angle: 0.02,
          k_linear: 1.5,
        },
      },
  );
  let i = 1;
  while (
    draft.fleet.robots.some((r) => r.id === `AGV${String(i).padStart(2, "0")}`)
  )
    i++;
  r.id = `AGV${String(i).padStart(2, "0")}`;
  const route = source
    ? assignment(source.id).route_id
    : Object.keys(scene.loops)[0];
  const start = freeIndex(route),
    n = scene.loops[route].length;
  const a = {
    robot_id: r.id,
    route_id: route,
    start_index: start,
    pickup_index: start,
    dropoff_index: (start + 4) % n,
    repeat_orders: true,
  };
  a.orders = [
    {
      pickup_index: a.pickup_index,
      dropoff_index: a.dropoff_index,
      release_time: 0,
      service_time_s: 3,
      cargo_type: route,
    },
  ];
  positionAt(r, a, start);
  draft.fleet.robots.push(r);
  draft.wms.fixed_loops.assignments.push(a);
  selected = r.id;
  markDirty();
  renderEditor();
}
function placeRobot(rid, p, kind, final) {
  if (isMixed() && !started()) {
    const robot = draft.fleet.robots.find((r) => r.id === rid);
    const station = kind.startsWith("station:");
    const points = station
      ? scene.amr_graph?.nodes || []
      : robot
        ? mixedPoints(robot)
        : [];
    let index = 0;
    points.forEach((q, i) => {
      if (
        Math.hypot(q.x - p.x, q.y - p.y) <
        Math.hypot(points[index].x - p.x, points[index].y - p.y)
      )
        index = i;
    });
    if (!points.length) return;
    if (station)
      draft.wms.fixed_loops.stations[Number(kind.split(":")[1])].node_id =
        points[index].id;
    else mixedPosition(robot, index);
    markDirty();
    if (final) {
      map.placement = null;
      renderEditor();
    }
    renderMap();
    return;
  }
  if (started()) return;
  const a = assignment(rid),
    r = draft.fleet.robots.find((r) => r.id === rid);
  if (!a || !r) return;
  const points = scene.loops[a.route_id];
  let idx = 0,
    best = Infinity;
  points.forEach((q, i) => {
    const d = Math.hypot(q.x - p.x, q.y - p.y);
    if (d < best) {
      idx = i;
      best = d;
    }
  });
  if (kind === "start") positionAt(r, a, idx);
  else {
    a.orders[orderPlacement][kind + "_index"] = idx;
    a.pickup_index = a.orders[0].pickup_index;
    a.dropoff_index = a.orders[0].dropoff_index;
  }
  markDirty();
  if (final) {
    map.placement = null;
    renderEditor();
  } else if ($("startIndex")) $("startIndex").value = idx;
}
function parameterValues() {
  const solver = catalog.solvers.find((s) => s.id === $("solverSelect").value);
  parameters[solver.id] = {
    ...Object.fromEntries(solver.parameters.map((p) => [p.key, p.default])),
    ...(parameters[solver.id] || {}),
  };
  return parameters[solver.id];
}
function updateMetricOptions() {
  const solver = catalog.solvers.find((s) => s.id === $("solverSelect").value);
  $("chartMetric")
    .querySelectorAll("[data-extra]")
    .forEach((e) => e.remove());
  for (const m of solver.metrics) {
    const o = document.createElement("option");
    o.value = "metrics." + m.key;
    o.textContent = m.label + (m.unit ? " · " + m.unit : "");
    o.dataset.extra = "true";
    $("chartMetric").append(o);
  }
}
function openSettings() {
  const solver = catalog.solvers.find((s) => s.id === $("solverSelect").value),
    values = parameterValues();
  $("solverDialogTitle").textContent = solver.label;
  $("solverDescription").textContent = solver.description || "";
  $("solverHelp").href = solver.help_url || "/api/workspace/algorithm-help";
  $("solverHelp").textContent =
    "Справка: " + solver.label + ", параметры и метрики ↗";
  $("parameterFields").innerHTML = solver.parameters
    .map(
      (p) =>
        `<label>${esc(p.label)}${p.unit ? " · " + esc(p.unit) : ""}${p.options ? `<select data-param="${esc(p.key)}">${p.options.map((v) => option(v, v, values[p.key])).join("")}</select>` : p.type === "boolean" ? `<input type="checkbox" data-param="${esc(p.key)}" ${values[p.key] ? "checked" : ""}>` : `<input data-param="${esc(p.key)}" type="${["number", "integer"].includes(p.type) ? "number" : "text"}" value="${esc(values[p.key])}" ${p.min !== undefined ? 'min="' + p.min + '"' : ""} ${p.max !== undefined ? 'max="' + p.max + '"' : ""} step="${p.type === "integer" ? 1 : p.step || "any"}">`}${p.description ? `<small class="muted">${esc(p.description)}</small>` : ""}</label>`,
    )
    .join("");
  updateMetricOptions();
  $("solverMetrics").textContent = solver.metrics.length
    ? "Дополнительные метрики: " +
      solver.metrics.map((m) => m.label || m.key).join(", ")
    : "Стандартные метрики: время поиска, раскрытия, стоимость и результат.";
  $("settingsDialog").showModal();
}
$("settingsForm").onsubmit = action(async (e) => {
  e.preventDefault();
  const solver = catalog.solvers.find((s) => s.id === $("solverSelect").value),
    values = {};
  for (const p of solver.parameters) {
    const el = [...$("parameterFields").querySelectorAll("[data-param]")].find(
      (e) => e.dataset.param === p.key,
    );
    values[p.key] =
      p.type === "boolean"
        ? el.checked
        : ["integer", "number"].includes(p.type)
          ? Number(el.value)
          : el.value;
  }
  parameters[solver.id] = values;
  $("settingsDialog").close();
  toast("Параметры применены для следующего запуска");
});
async function begin() {
  if (viewed) {
    returnLive();
    return;
  }
  if (started()) {
    await api(
      live.status === "paused" ? "simulation/resume" : "simulation/start",
      {},
    );
    return;
  }
  if (dirty || !draft.id) await saveScenario();
  setView("simulation");
  live = await api("workspace/prepare", {
    scenario_id: draft.id,
    solver: $("solverSelect").value,
    parameters: parameterValues(),
    duration_s: Number($("duration").value),
  });
  await api("workspace/speed", { factor: Number($("speed").value) });
  await api("simulation/start", {});
  live.recording.started = true;
  live.status = "running";
  updateUI();
}
async function seek(time) {
  if (!started()) return;
  const request = ++seekSerial,
    run = currentRun;
  if (live.status === "running") {
    await api("simulation/pause", {});
    live.status = "paused";
  }
  const data = await api("workspace/history?at=" + Math.max(0, time));
  if (
    request !== seekSerial ||
    run !== currentRun ||
    data.run_id !== currentRun
  )
    return;
  viewed = data.frame;
  updateUI(true);
}
function returnLive() {
  seekSerial++;
  viewed = null;
  selectedEvent = null;
  updateUI(true);
}
async function getJournal() {
  if (journalBusy || !started()) return;
  journalBusy = true;
  const run = currentRun;
  try {
    const data = await api(
      `workspace/journal?after=${journal.length}&limit=200`,
    );
    if (data.run_id !== currentRun || run !== currentRun) return;
    journal.push(...data.entries);
    if (data.entries.length) {
      renderAnalysis();
    }
  } finally {
    journalBusy = false;
  }
}
function renderAnalysis() {
  let events = journal;
  if (viewed) events = events.filter((e) => e.time <= viewed.time);
  $("journalCount").textContent = events.length + " вызовов";
  $("emptyJournal").hidden = events.length > 0;
  $("journalRows").innerHTML = events
    .slice(-100)
    .reverse()
    .map(
      (e) =>
        `<tr data-event="${e.id}" class="${selectedEvent?.id === e.id ? "selected" : ""}"><td>#${e.id}</td><td>${fmt(e.time)}</td><td><span class="pill ${esc(e.status)}">${esc(labels[e.status] || e.status)}</span></td><td>${num(e.elapsed_s * 1000)}</td><td>${e.expanded}</td></tr>`,
    )
    .join("");
  $("journalRows")
    .querySelectorAll("[data-event]")
    .forEach(
      (row) =>
        (row.onclick = action(async () => {
          const e = journal.find((e) => e.id === Number(row.dataset.event));
          await seek(e.time);
          selectedEvent = e;
          selected = null;
          if (viewed) {
            viewed = clone(viewed);
            viewed.robots = viewed.robots.map((r) => ({
              ...r,
              ...e.robot_positions[r.id],
            }));
            viewed.time = e.time;
            viewed.mapf = e.mapf_snapshot || {
              ...viewed.mapf,
              reservations: e.reservations,
            };
          }
          renderInspector();
          renderMap();
          renderAnalysis();
        })),
    );
  drawChart(events);
  const orders = viewed?.orders || live?.wms?.orders || [];
  $("ordersContent").innerHTML =
    "<table><thead><tr><th>Заказ</th><th>AGV</th><th>Груз</th><th>Откуда → куда</th><th>Статус</th><th>Выпуск</th></tr></thead><tbody>" +
    orders
      .map(
        (o) =>
          `<tr><td>${esc(o.id)}</td><td>${esc(o.assigned_robot || o.eligible_robots?.[0] || "—")}</td><td>${esc(o.cargo_type)}</td><td>${esc(o.pickup_node)} → ${esc(o.dropoff_node)}</td><td>${esc(labels[o.status] || o.status)}</td><td>${fmt(o.release_time)}</td></tr>`,
      )
      .join("") +
    "</tbody></table>";
}
function drawChart(events) {
  const key = $("chartMetric").value;
  const value = (e) =>
    key === "elapsed_s"
      ? e.elapsed_s * 1000
      : key.startsWith("metrics.")
        ? Number(e.metrics?.[key.slice(8)] || 0)
        : Number(e[key] || 0);
  const canvas = $("solverChart"),
    r = canvas.getBoundingClientRect();
  if (r.width < 1 || r.height < 1) return;
  const d = devicePixelRatio || 1;
  canvas.width = r.width * d;
  canvas.height = r.height * d;
  const c = canvas.getContext("2d");
  c.setTransform(d, 0, 0, d, 0, 0);
  const w = r.width,
    h = r.height,
    max = Math.max(1, ...events.map(value)),
    time = Math.max(1, ...events.map((e) => e.time));
  c.font = "9px system-ui";
  c.fillStyle = "#98a5b3";
  c.strokeStyle = "#e9eef2";
  for (let i = 0; i < 3; i++) {
    const y = 15 + ((h - 30) * i) / 2;
    c.beginPath();
    c.moveTo(30, y);
    c.lineTo(w, y);
    c.stroke();
    c.fillText(Math.round(max * (1 - i / 2)), 0, y + 3);
  }
  c.beginPath();
  events.forEach((e, i) => {
    const x = 30 + (e.time / time) * (w - 35),
      y = h - 15 - (value(e) / max) * (h - 30);
    if (i) c.lineTo(x, y);
    else c.moveTo(x, y);
  });
  c.strokeStyle = "#249987";
  c.lineWidth = 1.7;
  c.stroke();
  const mean =
    events.reduce((a, e) => a + e.elapsed_s * 1000, 0) / (events.length || 1);
  $("chartStats").innerHTML =
    `<span>Среднее <b>${num(mean)} мс</b></span><span>Максимум <b>${events.length ? num(Math.max(...events.map((e) => e.elapsed_s * 1000))) : "0"} мс</b></span><span>Частичных <b>${events.filter((e) => e.status === "partial").length}</b></span>`;
}
function updateUI(force = false) {
  const locked = started();
  $("scenarioSelect").disabled =
    $("solverSelect").disabled =
    $("solverSettings").disabled =
    $("duration").disabled =
      locked;
  $("newRun").hidden = !locked;
  $("saveReport").disabled = !locked;
  $("pause").disabled = !locked || live.status !== "running" || !!viewed;
  $("stop").disabled = !locked || !["running", "paused"].includes(live.status);
  $("start").disabled =
    locked &&
    ["running", "finished", "collision_stopped"].includes(live.status) &&
    !viewed;
  $("start").textContent = viewed
    ? "↗ К текущему"
    : !locked
      ? "▶ Запустить"
      : live.status === "paused"
        ? "▶ Продолжить"
        : "▶ Запустить";
  $("statusPill").textContent = viewed
    ? "История"
    : locked
      ? labels[live.status] || live.status
      : "Подготовка";
  $("statusPill").className = "pill " + (locked ? live.status : "");
  $("runLabel").textContent = dirty
    ? "Есть несохранённые изменения"
    : locked
      ? "Запуск " + live.recording.run_id.slice(0, 8)
      : "Подготовка эксперимента";
  $("historyBadge").hidden = !viewed;
  $("rewind").disabled = !locked;
  $("timeline").disabled = !locked;
  $("timeline").max = Math.max(
    Number($("duration").value),
    live?.recording?.last || 0,
  );
  $("timeline").value = viewed?.time ?? live?.time ?? 0;
  $("timeNow").textContent = fmt(viewed?.time ?? (locked ? live?.time : 0));
  $("timeEnd").textContent = fmt(Number($("duration").value));
  $("actualSpeed").textContent = "Факт: " + num(live?.speed?.actual, 1) + "×";
  const traffic = viewed?.mapf || live?.mapf;
  $("plannerConnection").disabled =
    !locked || !!viewed || !["running", "paused"].includes(live?.status);
  $("plannerConnection").textContent =
    live?.mapf?.connected === false ? "Восстановить связь" : "Отключить связь";
  $("controlMode").textContent =
    traffic?.control_mode === "autonomous"
      ? "Автономно · " +
        (reasons[traffic.autonomous_reason] || traffic.autonomous_reason) +
        (traffic.recovering ? " · подготовка перепланирования" : "")
      : "Центральное управление";
  renderMap();
  if (force || performance.now() - lastUI > 500) {
    lastUI = performance.now();
    if (view === "simulation") renderInspector();
    renderAnalysis();
  }
}
$("chartMetric").onchange = () =>
  drawChart(viewed ? journal.filter((e) => e.time <= viewed.time) : journal);
$("simulationTab").onclick = () => setView("simulation");
$("scenariosTab").onclick = () => setView("scenarios");
$("scenarioSelect").onchange = action((e) => loadScenario(e.target.value));
$("solverSettings").onclick = openSettings;
$("solverSelect").onchange = () => updateMetricOptions();
$("closeSettings").onclick = () => $("settingsDialog").close();
$("closeReport").onclick = () => $("reportDialog").close();
$("start").onclick = action(begin);
$("pause").onclick = action(() => api("simulation/pause", {}));
$("stop").onclick = action(() => api("simulation/stop", {}));
$("plannerConnection").onclick = action(async () => {
  await api("workspace/planner-connection", {
    connected: live?.mapf?.connected === false,
  });
});
$("speed").onchange = action((e) =>
  api("workspace/speed", { factor: Number(e.target.value) }),
);
$("returnLive").onclick = returnLive;
$("rewind").onclick = action(() =>
  seek((viewed?.time ?? live?.time ?? 0) - 10),
);
let seekTimer;
$("timeline").oninput = (e) => {
  const t = Number(e.target.value);
  clearTimeout(seekTimer);
  seekTimer = setTimeout(() => action(() => seek(t))(), 100);
};
$("newRun").onclick = action(async () => {
  if (
    !confirm(
      "Вернуться к подготовке? Несохранённая запись текущего запуска будет удалена.",
    )
  )
    return;
  await api("simulation/stop", {});
  live = await api("workspace/unlock", {});
  currentRun = live.recording.run_id;
  journal = [];
  viewed = null;
  selectedEvent = null;
  updateUI(true);
});
$("saveReport").onclick = action(async () => {
  $("saveReport").disabled = true;
  try {
    const result = await api("workspace/reports", {});
    $("reportLinks").innerHTML =
      `<a href="${esc(result.html_url)}" target="_blank" rel="noopener">Открыть HTML ↗</a><a href="${esc(result.json_url)}" download>Скачать JSON ↓</a>`;
    $("reportDialog").showModal();
  } finally {
    $("saveReport").disabled = false;
  }
});
$("newScenario").onclick = () => {
  draft = clone(draft);
  delete draft.id;
  draft.name = "Новый сценарий";
  draft.description = "";
  if (isMixed()) {
    selected = null;
    markDirty();
    renderEditor();
    renderMap();
    return;
  }
  draft.fleet.robots = draft.fleet.robots.slice(0, 1);
  draft.wms.fixed_loops.assignments = draft.wms.fixed_loops.assignments.slice(
    0,
    1,
  );
  selected = null;
  markDirty();
  renderEditor();
};
$("importScenario").onclick = () => $("scenarioFile").click();
$("scenarioFile").onchange = action(async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const d = JSON.parse(await file.text());
  const saved = await api("workspace/scenarios", d);
  await refreshCatalog();
  await loadScenario(saved.id);
  toast("Сценарий импортирован");
  e.target.value = "";
});
$("importAsset").onclick = () => $("assetFile").click();
$("assetFile").onchange = action(async (e) => {
  const form = new FormData();
  form.append("file", e.target.files[0]);
  const response = await fetch("/api/workspace/assets", {
    method: "POST",
    body: form,
  });
  if (!response.ok) throw new Error((await response.json()).detail);
  await refreshCatalog();
  renderEditor();
  toast("Файл добавлен в список карт / графов");
  e.target.value = "";
});
$("fitMap").onclick = () => map.fit();
$("zoomIn").onclick = () => map.zoom(1.3);
$("zoomOut").onclick = () => map.zoom(1 / 1.3);
document.querySelectorAll("[data-layer]").forEach(
  (el) =>
    (el.onchange = () => {
      map.layers[el.dataset.layer] = el.checked;
      map.draw();
    }),
);
$("closeInspector").onclick = () => {
  $("inspector").hidden = true;
  $("openInspector").hidden = false;
};
$("openInspector").onclick = () => {
  $("inspector").hidden = false;
  $("openInspector").hidden = true;
};
$("toggleAnalysis").onclick = () => {
  $("analysis").classList.toggle("collapsed");
  $("toggleAnalysis").textContent = $("analysis").classList.contains(
    "collapsed",
  )
    ? "Развернуть ↑"
    : "Свернуть ↓";
  setTimeout(() => drawChart(journal), 50);
};
document.querySelectorAll("[data-analysis]").forEach(
  (b) =>
    (b.onclick = () => {
      activeAnalysis = b.dataset.analysis;
      document
        .querySelectorAll("[data-analysis]")
        .forEach((x) => x.classList.toggle("active", x === b));
      $("analysisContent").hidden = activeAnalysis !== "solver";
      $("ordersContent").hidden = activeAnalysis !== "orders";
      renderAnalysis();
    }),
);
function connect() {
  const socket = new WebSocket(
    `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/state`,
  );
  socket.onopen = () => {
    $("connection").textContent = "Backend подключён";
    $("connection").classList.add("online");
  };
  socket.onmessage = (e) => {
    const next = JSON.parse(e.data);
    if (currentRun !== next.recording?.run_id) {
      currentRun = next.recording?.run_id;
      journal = [];
      selectedEvent = null;
      viewed = null;
      seekSerial++;
    }
    live = next;
    updateUI();
    action(getJournal)();
  };
  const timer = setInterval(() => {
    if (socket.readyState === 1) socket.send("ping");
  }, 15000);
  socket.onclose = () => {
    clearInterval(timer);
    $("connection").textContent = "Переподключение…";
    $("connection").classList.remove("online");
    setTimeout(connect, 1000);
  };
}
await action(async () => {
  await refreshCatalog();
  live = await api("state");
  currentRun = live.recording?.run_id;
  const saved = await api("workspace/run");
  if (live.recording?.started && saved.inputs.scenario) {
    draft = normalize(saved.inputs.scenario);
    $("scenarioSelect").value = draft.id;
    $("solverSelect").value = saved.inputs.solver.id;
    parameters[saved.inputs.solver.id] = saved.inputs.solver.parameters;
    $("duration").value = saved.inputs.simulation.duration_s;
    await preview();
    renderInspector();
  } else await loadScenario(catalog.scenarios[0].id);
  updateMetricOptions();
  updateUI(true);
  connect();
})();

function isMixed() {
  return draft?.wms.fixed_loops.mode === "mixed";
}
function mixedPoints(robot) {
  return robot.type === "agv"
    ? scene?.robot_loops?.[robot.id] || []
    : scene?.amr_graph?.nodes || [];
}
function mixedPosition(robot, index) {
  const points = mixedPoints(robot),
    p = points[index];
  if (!p) return;
  robot.initial_pose = {
    x: p.x,
    y: p.y,
    theta: robot.type === "agv" ? p.theta : robot.initial_pose.theta || 0,
  };
  if (robot.type === "agv") assignment(robot.id).start_index = index;
  else robot.parameters.start_node = p.id;
}
async function mixedRebuild() {
  const positions = new Map(
    draft.fleet.robots.map((r) => [r.id, r.initial_pose]),
  );
  await preview();
  for (const r of draft.fleet.robots) {
    if (r.type !== "agv") continue;
    const p = positions.get(r.id),
      points = mixedPoints(r);
    let index = 0;
    points.forEach((q, i) => {
      if (
        Math.hypot(q.x - p.x, q.y - p.y) <
        Math.hypot(points[index].x - p.x, points[index].y - p.y)
      )
        index = i;
    });
    mixedPosition(r, index);
    for (const stream of draft.wms.fixed_loops.streams.filter(
      (s) => s.robot_id === r.id,
    )) {
      stream.pickup_index = Math.min(stream.pickup_index, points.length - 1);
      stream.dropoff_index = Math.min(stream.dropoff_index, points.length - 1);
    }
  }
  markDirty();
  renderEditor();
  renderMap();
}
function renderMixedEditor() {
  const cfg = draft.wms.fixed_loops,
    robot = draft.fleet.robots.find((r) => r.id === selected),
    a = assignment(selected);
  $("inspectorTitle").textContent = "AGV / AMR · сценарий";
  const input = (label, value, attr) =>
    field(label, value, attr + ' step="any"');
  $("inspectorBody").innerHTML = `<div class="form-stack">
    ${field("Название", draft.name, 'id="mixedName"', "text")}
    <label>Карта<select id="mixedMap">${catalog.assets.maps.map((x) => option(x.id, x.name, draft.map_asset)).join("")}</select></label>
    <label>Петли AGV<select id="mixedGraph">${catalog.assets.graphs.map((x) => option(x.id, x.name, draft.graph_asset)).join("")}</select></label>
    <label>Рабочий граф AMR<select id="mixedAmrGraph">${option("", "Не подключён", draft.amr_graph_asset || "")}${catalog.assets.graphs.map((x) => option(x.id, x.name, draft.amr_graph_asset)).join("")}</select></label>
    ${input("Такт планирования, с", cfg.planning_tick_s, 'id="mixedTick"')}
    <label class="check-row"><input id="mixedStop" type="checkbox" ${draft.simulation.collision_mode === "stop_on_collision" ? "checked" : ""}>Останавливать при столкновении</label>
    <small>Физика: 0.05 с. AGV — только вперёд по скруглённой петле. AMR — выбор пути между станциями. Поступления заказов не зависят от завершения предыдущих.</small>
    <div class="form-actions"><button id="mixedSave" class="primary">Сохранить</button><button id="mixedCopy">Копия</button><button id="mixedExport">↓ JSON</button></div>
    <div class="form-actions"><button id="mixedAddAgv">＋ AGV</button><button id="mixedAddAmr">＋ AMR</button></div>
    <select id="mixedRobot">${option("", "Выберите робота", selected)}${draft.fleet.robots.map((r) => option(r.id, r.id + " · " + r.type.toUpperCase(), selected)).join("")}</select>
    ${
      robot
        ? `<div class="form-grid">${field("ID", robot.id, 'id="mixedRobotName"', "text")}
      ${input("Длина, м", robot.footprint.length, 'data-mixed-field="footprint.length"')}${input("Ширина, м", robot.footprint.width, 'data-mixed-field="footprint.width"')}
      ${input("Максимальная скорость, м/с", robot.parameters.max_linear, 'data-mixed-field="parameters.max_linear"')}
      ${input("Скорость движения, м/с", robot.route_follower.max_linear, 'data-mixed-field="route_follower.max_linear"')}
      ${input("Угловая скорость, рад/с", robot.parameters.max_angular, 'data-mixed-field="parameters.max_angular"')}
      ${input("Энергия α / метр", robot.parameters.energy_alpha ?? 1, 'data-mixed-field="parameters.energy_alpha"')}
      ${input("Энергия β / с простоя", robot.parameters.energy_beta ?? 0.1, 'data-mixed-field="parameters.energy_beta"')}
      ${robot.type === "agv" ? input("Мин. радиус поворота, м", robot.parameters.min_turn_radius, 'data-mixed-field="parameters.min_turn_radius"') + `<label>Петля<select id="mixedRoute">${scene.graph.routes.map((r) => option(r.route_id, r.route_id, a.route_id)).join("")}</select></label>` : input("Начальный угол, рад", robot.initial_pose.theta, 'data-mixed-field="initial_pose.theta"')}
      ${field(robot.type === "agv" ? "Начальный индекс" : "Начальный узел AMR", robot.type === "agv" ? a.start_index : robot.parameters.start_node, 'id="mixedStart"', robot.type === "agv" ? "number" : "text")}</div>
      <small>Перетащите робота на карту: положение привяжется к его навигационному слою.</small><button id="mixedRemove">Удалить робота</button>`
        : ""
    }
    <div class="panel-heading"><h2>Станции AMR</h2><button id="mixedAddStation">＋</button></div>
    ${cfg.stations.map((s, i) => `<div class="form-grid">${field("ID станции", s.id, `data-station="${i}.id"`, "text")}${field("Узел AMR", s.node_id, `data-station="${i}.node_id"`, "text")}<button data-station-place="${i}">◎ На карте</button><button data-station-remove="${i}">Удалить</button></div>`).join("")}
    <div class="panel-heading"><h2>Потоки заказов</h2><button id="mixedAddStream">＋</button></div>
    <small>Обслуживание — отдельная неподвижная загрузка и выгрузка, по указанному числу секунд каждая.</small>
    ${cfg.streams.map((s, i) => `<div class="editor-section"><strong>${esc(s.kind === "agv" ? s.robot_id : "AMR · любой свободный")}</strong><div class="form-grid">${s.kind === "agv" ? input("Индекс загрузки", s.pickup_index, `data-stream="${i}.pickup_index"`) + input("Индекс выгрузки", s.dropoff_index, `data-stream="${i}.dropoff_index"`) : ["pickup", "dropoff"].map((k) => `<label>${k === "pickup" ? "Загрузка" : "Выгрузка"}<select data-stream="${i}.${k}">${cfg.stations.map((t) => option(t.id, t.id, s[k])).join("")}</select></label>`).join("")}${input("Первый заказ, с", s.start_s ?? 0, `data-stream="${i}.start_s"`)}${input("Период поступления, с", s.period_s, `data-stream="${i}.period_s"`)}${input("Обслуживание, с", s.service_time_s, `data-stream="${i}.service_time_s"`)}</div><button data-stream-remove="${i}">Удалить поток</button></div>`).join("")}
    <details><summary>Совмещение графа AMR</summary><p>Преобразование создаёт новый файл. Исходник сохраняется. Проверьте совпадение с DXF и проходимость рёбер.</p>${input("Масштаб", 1, 'id="graphScale"')}${input("Поворот, градусы", 0, 'id="graphRotation"')}${input("Смещение X, м", 0, 'id="graphOffsetX"')}${input("Смещение Y, м", 0, 'id="graphOffsetY"')}<button id="transformAmr">Создать преобразованный граф</button></details>
    </div>`;
  $("mixedName").onchange = (e) => {
    draft.name = e.target.value;
    markDirty();
  };
  $("mixedTick").onchange = (e) => {
    cfg.planning_tick_s = Number(e.target.value);
    markDirty();
  };
  $("mixedStop").onchange = (e) => {
    draft.simulation.collision_mode = e.target.checked
      ? "stop_on_collision"
      : "count_only";
    markDirty();
  };
  $("mixedSave").onclick = action(saveScenario);
  $("mixedCopy").onclick = () => {
    draft = clone(draft);
    delete draft.id;
    draft.name += " · копия";
    markDirty();
    renderEditor();
  };
  $("mixedExport").onclick = () =>
    download(draft, (draft.id || "mixed") + ".json");
  for (const [id, key] of [
    ["mixedMap", "map_asset"],
    ["mixedGraph", "graph_asset"],
    ["mixedAmrGraph", "amr_graph_asset"],
  ])
    $(id).onchange = action(async (e) => {
      const old = draft[key];
      draft[key] = e.target.value;
      try {
        await mixedRebuild();
      } catch (error) {
        draft[key] = old;
        e.target.value = old || "";
        throw error;
      }
    });
  $("mixedRobot").onchange = (e) => {
    selected = e.target.value;
    renderEditor();
    renderMap();
  };
  for (const [id, type] of [
    ["mixedAddAgv", "agv"],
    ["mixedAddAmr", "amr"],
  ])
    $(id).onclick = action(async () => {
      if (type === "amr" && !scene.amr_graph?.nodes?.length) {
        toast("Сначала выберите рабочий граф AMR", true);
        return;
      }
      const source = draft.fleet.robots.find((r) => r.type === type);
      const r = source
        ? clone(source)
        : {
            type,
            footprint: { length: type === "agv" ? 1.5 : 0.8, width: 0.6 },
            parameters: {
              max_linear: 1,
              max_angular: 1.5,
              min_turn_radius: 0.8,
            },
            route_follower: { max_linear: 0.7 },
            initial_pose: { x: 0, y: 0, theta: 0 },
          };
      let n = 1;
      while (draft.fleet.robots.some((r) => r.id === type.toUpperCase() + n))
        n++;
      r.id = type.toUpperCase() + n;
      draft.fleet.robots.push(r);
      if (type === "agv")
        cfg.assignments.push({
          robot_id: r.id,
          route_id: source
            ? assignment(source.id).route_id
            : scene.graph.routes[0].route_id,
          start_index: 0,
        });
      selected = r.id;
      await preview();
      const points = mixedPoints(r);
      let best = 0,
        clearance = -1;
      points.forEach((p, i) => {
        const d = Math.min(
          ...draft.fleet.robots
            .filter((b) => b !== r)
            .map((b) =>
              Math.hypot(p.x - b.initial_pose.x, p.y - b.initial_pose.y),
            ),
        );
        if (d > clearance) {
          best = i;
          clearance = d;
        }
      });
      mixedPosition(r, best);
      if (type === "agv")
        cfg.streams.push({
          kind: "agv",
          robot_id: r.id,
          pickup_index: best,
          dropoff_index: (best + Math.floor(points.length / 2)) % points.length,
          period_s: 100,
          start_s: 0,
          service_time_s: 3,
        });
      markDirty();
      renderEditor();
      renderMap();
    });
  if (robot) {
    $("mixedRobotName").onchange = (e) => {
      const id = e.target.value.trim();
      if (!id || draft.fleet.robots.some((r) => r !== robot && r.id === id))
        throw Error("ID должен быть уникальным");
      for (const s of cfg.streams) if (s.robot_id === robot.id) s.robot_id = id;
      if (a) a.robot_id = id;
      robot.id = id;
      selected = id;
      action(mixedRebuild)();
    };
    document.querySelectorAll("[data-mixed-field]").forEach(
      (el) =>
        (el.onchange = action(async () => {
          const [group, key] = el.dataset.mixedField.split(".");
          robot[group][key] = Number(el.value);
          markDirty();
          if (group === "footprint") delete robot.footprint.collision_radius;
          if (key === "min_turn_radius") await mixedRebuild();
          else markDirty();
        })),
    );
    if (a)
      $("mixedRoute").onchange = action(async (e) => {
        a.route_id = e.target.value;
        await mixedRebuild();
      });
    $("mixedStart").onchange = (e) => {
      const points = mixedPoints(robot);
      mixedPosition(
        robot,
        robot.type === "agv"
          ? Number(e.target.value)
          : points.findIndex((p) => p.id === e.target.value),
      );
      markDirty();
      renderMap();
    };
    $("mixedRemove").onclick = () => {
      draft.fleet.robots = draft.fleet.robots.filter((r) => r !== robot);
      cfg.assignments = cfg.assignments.filter((x) => x !== a);
      cfg.streams = cfg.streams.filter((s) => s.robot_id !== robot.id);
      selected = null;
      markDirty();
      renderEditor();
      renderMap();
    };
  }
  document.querySelectorAll("[data-station]").forEach(
    (el) =>
      (el.onchange = () => {
        const [i, k] = el.dataset.station.split(".");
        const s = cfg.stations[i];
        if (k === "id") {
          for (const stream of cfg.streams)
            for (const field of ["pickup", "dropoff"])
              if (stream[field] === s.id) stream[field] = el.value;
        }
        s[k] = el.value;
        markDirty();
        renderEditor();
        renderMap();
      }),
  );
  document.querySelectorAll("[data-station-place]").forEach(
    (el) =>
      (el.onclick = () => {
        map.placement = "station:" + el.dataset.stationPlace;
        renderMap();
      }),
  );
  document.querySelectorAll("[data-station-remove]").forEach(
    (el) =>
      (el.onclick = () => {
        const s = cfg.stations[el.dataset.stationRemove];
        if (cfg.streams.some((t) => t.pickup === s.id || t.dropoff === s.id)) {
          toast("Сначала удалите или измените потоки этой станции", true);
          return;
        }
        cfg.stations.splice(Number(el.dataset.stationRemove), 1);
        markDirty();
        renderEditor();
        renderMap();
      }),
  );
  $("mixedAddStation").onclick = () => {
    if (!scene.amr_graph?.nodes?.length) {
      toast("Сначала выберите рабочий граф AMR", true);
      return;
    }
    let n = 1;
    while (cfg.stations.some((s) => s.id === "S" + n)) n++;
    cfg.stations.push({
      id: "S" + n,
      node_id: (scene.amr_graph?.nodes || [])[0].id,
    });
    markDirty();
    renderEditor();
    renderMap();
  };
  document.querySelectorAll("[data-stream]").forEach(
    (el) =>
      (el.onchange = () => {
        const [i, k] = el.dataset.stream.split(".");
        cfg.streams[i][k] = ["pickup", "dropoff"].includes(k)
          ? el.value
          : Number(el.value);
        markDirty();
      }),
  );
  document.querySelectorAll("[data-stream-remove]").forEach(
    (el) =>
      (el.onclick = () => {
        cfg.streams.splice(Number(el.dataset.streamRemove), 1);
        markDirty();
        renderEditor();
      }),
  );
  $("mixedAddStream").onclick = () => {
    if (robot?.type === "agv") {
      if (cfg.streams.some((s) => s.robot_id === robot.id)) {
        toast("У AGV одна фиксированная пара доставки", true);
        return;
      }
      cfg.streams.push({
        kind: "agv",
        robot_id: robot.id,
        pickup_index: 0,
        dropoff_index: Math.floor(mixedPoints(robot).length / 2),
        period_s: 100,
        start_s: 0,
        service_time_s: 3,
      });
    } else {
      if (cfg.stations.length < 2) {
        toast("Добавьте две станции AMR", true);
        return;
      }
      cfg.streams.push({
        kind: "amr",
        pickup: cfg.stations[0].id,
        dropoff: cfg.stations[1].id,
        period_s: 100,
        start_s: 0,
        service_time_s: 3,
      });
    }
    markDirty();
    renderEditor();
  };
  $("transformAmr").onclick = action(async () => {
    const result = await api("workspace/convert-graph", {
      asset: draft.amr_graph_asset,
      scale: Number($("graphScale").value),
      rotation: Number($("graphRotation").value),
      offset: [
        Number($("graphOffsetX").value),
        Number($("graphOffsetY").value),
      ],
      keep: [
        ...cfg.stations.map((s) => s.node_id),
        ...draft.fleet.robots
          .filter((r) => r.type === "amr")
          .map((r) => r.parameters.start_node),
      ],
    });
    draft.amr_graph_asset = result.id;
    await refreshCatalog();
    await preview();
    for (const r of draft.fleet.robots.filter((r) => r.type === "amr"))
      mixedPosition(
        r,
        mixedPoints(r).findIndex((p) => p.id === r.parameters.start_node),
      );
    markDirty();
    renderEditor();
    renderMap();
  });
}
