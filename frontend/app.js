const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");
const statusBox = document.getElementById("statusBox");
const ordersBox = document.getElementById("ordersBox");

let state = null;
let scale = 45;
let offsetX = 30;
let offsetY = 30;
let isPanning = false;
let lastMouse = null;

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * devicePixelRatio;
  canvas.height = rect.height * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();

function worldToScreen(x, y) {
  return {
    x: offsetX + x * scale,
    y: canvas.clientHeight - offsetY - y * scale,
  };
}

function screenToWorld(x, y) {
  return {
    x: (x - offsetX) / scale,
    y: (canvas.clientHeight - offsetY - y) / scale,
  };
}

function draw() {
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  drawGrid();

  if (!state) {
    requestAnimationFrame(draw);
    return;
  }

  drawMap(state.map);
  drawGraph(state.graph);
  const renderRobots = state.robots || [];
  drawRoutes(renderRobots);
  drawRobots(renderRobots);
  drawCollisions(state.last_collisions || []);
  updateStatusPanel();
  updateOrdersPanel();

  requestAnimationFrame(draw);
}

function drawGrid() {
  ctx.save();
  ctx.strokeStyle = "#0f172a";
  ctx.lineWidth = 1;
  const step = scale;
  for (let x = offsetX % step; x < canvas.clientWidth; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.clientHeight);
    ctx.stroke();
  }
  for (let y = canvas.clientHeight - offsetY; y > 0; y -= step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.clientWidth, y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawMap(map) {
  if (!map || !map.segments) return;
  ctx.save();
  ctx.strokeStyle = "#e2e8f0";
  ctx.lineWidth = 2;
  for (const segment of map.segments) {
    const a = worldToScreen(segment.start.x, segment.start.y);
    const b = worldToScreen(segment.end.x, segment.end.y);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawGraph(graph) {
  if (!graph) return;
  ctx.save();
  ctx.strokeStyle = "#22c55e";
  ctx.lineWidth = 1.5;
  ctx.globalAlpha = 0.75;

  for (const edge of graph.edges || []) {
    if (!edge.coordinates || edge.coordinates.length < 2) continue;
    ctx.beginPath();
    const first = worldToScreen(edge.coordinates[0][0], edge.coordinates[0][1]);
    ctx.moveTo(first.x, first.y);
    for (const [x, y] of edge.coordinates.slice(1)) {
      const p = worldToScreen(x, y);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
  }

  ctx.globalAlpha = 1.0;
  for (const node of graph.nodes || []) {
    const p = worldToScreen(node.x, node.y);
    ctx.fillStyle = "#84cc16";
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#bbf7d0";
    ctx.font = "11px sans-serif";
    ctx.fillText(node.id, p.x + 6, p.y - 6);
  }
  ctx.restore();
}

function drawRoutes(robots) {
  for (const robot of robots) {
    const follower = robot.route_follower;
    if (!follower || !follower.route || follower.route.length === 0) continue;

    ctx.save();
    ctx.strokeStyle = robot.color || "#60a5fa";
    ctx.globalAlpha = 0.55;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    let start = worldToScreen(robot.x, robot.y);
    ctx.moveTo(start.x, start.y);
    for (const wp of follower.route) {
      const p = worldToScreen(wp.x, wp.y);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    for (let i = 0; i < follower.route.length; i++) {
      const wp = follower.route[i];
      const p = worldToScreen(wp.x, wp.y);
      ctx.fillStyle = i === follower.current_index ? "#facc15" : robot.color || "#60a5fa";
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3.5, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }
}

function drawRobots(robots) {
  for (const robot of robots) {
    const p = worldToScreen(robot.x, robot.y);
    const theta = -robot.theta;
    const length = robot.length * scale;
    const width = robot.width * scale;

    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(theta);
    ctx.fillStyle = robot.color || "#60a5fa";
    ctx.strokeStyle = robot.status === "collision" ? "#f97316" : "#ffffff";
    ctx.lineWidth = robot.status === "collision" ? 3 : 1;
    ctx.beginPath();
    ctx.roundRect(-length / 2, -width / 2, length, width, 6);
    ctx.fill();
    ctx.stroke();

    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(length / 2, 0);
    ctx.stroke();
    ctx.restore();

    ctx.save();
    ctx.fillStyle = "#e5e7eb";
    ctx.font = "12px sans-serif";
    ctx.fillText(`${robot.id} ${robot.status}`, p.x + 8, p.y - 8);
    if (robot.active_order_id) {
      ctx.fillText(robot.active_order_id, p.x + 8, p.y + 8);
    }
    ctx.restore();
  }
}

function drawCollisions(collisions) {
  ctx.save();
  for (const c of collisions) {
    if (c.x == null || c.y == null) continue;
    const p = worldToScreen(c.x, c.y);
    ctx.strokeStyle = "#f97316";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(p.x - 10, p.y - 10);
    ctx.lineTo(p.x + 10, p.y + 10);
    ctx.moveTo(p.x + 10, p.y - 10);
    ctx.lineTo(p.x - 10, p.y + 10);
    ctx.stroke();
  }
  ctx.restore();
}

function updateStatusPanel() {
  if (!state) return;
  statusBox.textContent = JSON.stringify(
    {
      time: Number(state.time).toFixed(2),
      status: state.status,
      collision_mode: state.collision_mode,
      max_sim_time: state.max_sim_time,
      metrics: state.metrics_summary,
      comms: state.comms
        ? {
            running: state.comms.running,
            port: state.comms.port,
            robots_connected: state.comms.robots_connected,
          }
        : null,
      robots: (state.robots || []).map((r) => ({
        id: r.id,
        status: r.status,
        mode: r.mode,
        order: r.active_order_id,
        x: Number(r.x).toFixed(2),
        y: Number(r.y).toFixed(2),
      })),
    },
    null,
    2,
  );
}

function updateOrdersPanel() {
  const orders = state?.wms?.orders || [];
  ordersBox.innerHTML = "";
  for (const order of orders) {
    const div = document.createElement("div");
    div.className = "order";
    div.innerHTML = `
      <strong>${escapeHtml(order.id)}</strong> [${escapeHtml(order.status)}]<br/>
      type=${escapeHtml(order.cargo_type)} ${escapeHtml(order.pickup_node)} → ${escapeHtml(order.dropoff_node)}<br/>
      t=${Number(order.release_time).toFixed(1)}, robot=${escapeHtml(order.assigned_robot || "-")}
    `;
    ordersBox.appendChild(div);
  }
}

function escapeHtml(value) {
  return String(value).replace(/[&<>'"]/g, (ch) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    "'": "&#39;",
    '"': "&quot;",
  })[ch]);
}

async function postJson(url, body = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!response.ok) throw new Error(await response.text());
  return response.json();
}

async function post(url) {
  const response = await fetch(url, { method: "POST" });
  if (!response.ok) throw new Error(await response.text());
  return response.json();
}

async function sendCommand(command) {
  const robotId = document.getElementById("robotIdInput").value || "R1";
  const commands = {
    forward: { linear: 0.7, angular: 0.0 },
    backward: { linear: -0.4, angular: 0.0 },
    left: { linear: 0.0, angular: 0.9 },
    right: { linear: 0.0, angular: -0.9 },
    stop: { linear: 0.0, angular: 0.0 },
  };
  await postJson(`/api/robots/${encodeURIComponent(robotId)}/cmd`, commands[command]);
}

async function configureAndStart() {
  const form = new FormData();
  const files = [
    ["map_dxf", "mapFile"],
    ["graph_geojson", "graphFile"],
    ["fleet_json", "fleetFile"],
    ["scenario_json", "scenarioFile"],
  ];
  for (const [field, id] of files) {
    const input = document.getElementById(id);
    if (input.files && input.files[0]) form.append(field, input.files[0]);
  }
  form.append("duration", document.getElementById("durationInput").value || "120");
  form.append("collision_mode", document.getElementById("collisionModeSelect").value);
  form.append("autostart", "true");

  const response = await fetch("/api/simulation/configure", { method: "POST", body: form });
  if (!response.ok) alert(await response.text());
}

document.querySelectorAll("button[data-cmd]").forEach((button) => {
  button.addEventListener("click", () => sendCommand(button.dataset.cmd));
});

document.getElementById("loadDemoBtn").addEventListener("click", () => post("/api/simulation/load-demo"));
document.getElementById("configureStartBtn").addEventListener("click", configureAndStart);
document.getElementById("startBtn").addEventListener("click", () => post("/api/simulation/start"));
document.getElementById("pauseBtn").addEventListener("click", () => post("/api/simulation/pause"));
document.getElementById("resumeBtn").addEventListener("click", () => post("/api/simulation/resume"));
document.getElementById("stopBtn").addEventListener("click", () => post("/api/simulation/stop"));

window.addEventListener("keydown", (event) => {
  if (event.target.tagName === "INPUT") return;
  const keyMap = { w: "forward", s: "backward", a: "left", d: "right", " ": "stop" };
  const command = keyMap[event.key.toLowerCase()];
  if (command) {
    event.preventDefault();
    sendCommand(command);
  }
});

canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const mouse = { x: event.clientX - rect.left, y: event.clientY - rect.top };
  const before = screenToWorld(mouse.x, mouse.y);
  const factor = event.deltaY < 0 ? 1.1 : 0.9;
  scale = Math.max(10, Math.min(200, scale * factor));
  const after = screenToWorld(mouse.x, mouse.y);
  offsetX += (after.x - before.x) * scale;
  offsetY -= (after.y - before.y) * scale;
});

canvas.addEventListener("mousedown", (event) => {
  isPanning = true;
  lastMouse = { x: event.clientX, y: event.clientY };
});

window.addEventListener("mouseup", () => {
  isPanning = false;
  lastMouse = null;
});

window.addEventListener("mousemove", (event) => {
  if (!isPanning || !lastMouse) return;
  offsetX += event.clientX - lastMouse.x;
  offsetY -= event.clientY - lastMouse.y;
  lastMouse = { x: event.clientX, y: event.clientY };
});

function connectWebSocket() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${protocol}://${location.host}/ws/state`);
  socket.onmessage = (event) => {
    state = JSON.parse(event.data);
  };
  socket.onclose = () => setTimeout(connectWebSocket, 1000);
  setInterval(() => {
    if (socket.readyState === WebSocket.OPEN) socket.send("ping");
  }, 1000);
}

connectWebSocket();
requestAnimationFrame(draw);
