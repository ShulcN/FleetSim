export class MapView {
  constructor(canvas, onSelect, onPlace) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.onSelect = onSelect;
    this.onPlace = onPlace;
    this.scale = 1;
    this.ox = 40;
    this.oy = 40;
    this.layers = {
      obstacles: true,
      routes: true,
      zones: true,
      labels: true,
      plan: true,
    };
    this.scene = null;
    this.state = null;
    this.editor = false;
    this.selected = null;
    this.plan = null;
    this.placement = null;
    new ResizeObserver(() => {
      this.resize();
      if (this.scene) this.fit();
      else this.draw();
    }).observe(canvas);
    canvas.addEventListener(
      "wheel",
      (e) => {
        e.preventDefault();
        const p = this.pointer(e),
          w = this.world(p);
        this.scale = Math.max(
          0.1,
          Math.min(90, this.scale * (e.deltaY < 0 ? 1.12 : 1 / 1.12)),
        );
        this.ox = p.x - w.x * this.scale;
        this.oy = this.height - p.y - w.y * this.scale;
        this.draw();
      },
      { passive: false },
    );
    canvas.addEventListener("pointerdown", (e) => {
      const p = this.pointer(e),
        r = this.hit(p);
      canvas.setPointerCapture(e.pointerId);
      this.drag = {
        p,
        last: p,
        robot: this.editor && r ? r.id : null,
        moved: false,
      };
      if (r) {
        this.selected = r.id;
        this.onSelect(r.id);
      }
    });
    canvas.addEventListener("pointermove", (e) => {
      if (!this.drag) return;
      const p = this.pointer(e);
      const dx = p.x - this.drag.last.x,
        dy = p.y - this.drag.last.y;
      if (Math.hypot(p.x - this.drag.p.x, p.y - this.drag.p.y) > 3)
        this.drag.moved = true;
      if (this.drag.robot) {
        this.onPlace(this.drag.robot, this.world(p), "start", false);
      } else {
        this.ox += dx;
        this.oy -= dy;
      }
      this.drag.last = p;
      this.draw();
    });
    canvas.addEventListener("pointerup", (e) => {
      if (
        this.drag &&
        !this.drag.moved &&
        this.editor &&
        (this.selected || this.placement?.startsWith("station:")) &&
        this.placement
      )
        this.onPlace(
          this.selected,
          this.world(this.pointer(e)),
          this.placement,
          true,
        );
      this.drag = null;
      this.draw();
    });
  }
  resize() {
    const r = this.canvas.getBoundingClientRect();
    this.width = r.width;
    this.height = r.height;
    const d = devicePixelRatio || 1;
    this.canvas.width = Math.round(r.width * d);
    this.canvas.height = Math.round(r.height * d);
    this.ctx.setTransform(d, 0, 0, d, 0, 0);
  }
  pointer(e) {
    const r = this.canvas.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  }
  point(x, y) {
    return {
      x: this.ox + x * this.scale,
      y: this.height - this.oy - y * this.scale,
    };
  }
  world(p) {
    return {
      x: (p.x - this.ox) / this.scale,
      y: (this.height - this.oy - p.y) / this.scale,
    };
  }
  setScene(scene) {
    this.scene = scene;
    this.nodes = new Map((scene?.graph?.nodes || []).map((n) => [n.id, n]));
    for (const points of Object.values(scene?.loops || {}))
      for (const p of points) this.nodes.set(p.id, p);
    for (const points of Object.values(scene?.robot_loops || {}))
      for (const p of points) this.nodes.set(p.id, p);
    for (const p of scene?.amr_graph?.nodes || [])
      this.nodes.set("amr:" + p.id, p);
    this.groups = [];
    for (const seg of scene?.map?.segments || []) {
      const prev = this.groups.at(-1);
      if (
        prev &&
        prev.layer === seg.layer &&
        prev.points.at(-1).x === seg.start.x &&
        prev.points.at(-1).y === seg.start.y
      )
        prev.points.push(seg.end);
      else this.groups.push({ layer: seg.layer, points: [seg.start, seg.end] });
    }
    this.fit();
  }
  fit() {
    if (!this.scene) return;
    const b = this.scene.map?.bounds || {
      min_x: 0,
      min_y: 0,
      max_x: 400,
      max_y: 400,
    };
    const left = this.width > 700 ? 125 : 45,
      top = 75;
    this.scale = Math.max(
      0.1,
      Math.min(
        (this.width - left - 45) / (b.max_x - b.min_x || 1),
        (this.height - top - 35) / (b.max_y - b.min_y || 1),
      ),
    );
    this.ox =
      left -
      b.min_x * this.scale +
      (this.width - left - 45 - (b.max_x - b.min_x) * this.scale) / 2;
    this.oy = 25 - b.min_y * this.scale;
    this.draw();
  }
  zoom(factor) {
    const w = this.world({ x: this.width / 2, y: this.height / 2 });
    this.scale = Math.max(0.1, Math.min(90, this.scale * factor));
    this.ox = this.width / 2 - w.x * this.scale;
    this.oy = this.height / 2 - w.y * this.scale;
    this.draw();
  }
  hit(p) {
    let best = null,
      d = 15;
    for (const r of this.state?.robots || []) {
      const q = this.point(r.x, r.y),
        dist = Math.hypot(p.x - q.x, p.y - q.y);
      if (dist < d) {
        d = dist;
        best = r;
      }
    }
    return best;
  }
  line(points, color, width = 1, dash = []) {
    const c = this.ctx;
    if (!points.length) return;
    c.strokeStyle = color;
    c.lineWidth = width;
    c.setLineDash(dash);
    c.beginPath();
    points.forEach((p, i) => {
      const q = this.point(p.x ?? p[0], p.y ?? p[1]);
      if (i) c.lineTo(q.x, q.y);
      else c.moveTo(q.x, q.y);
    });
    c.stroke();
    c.setLineDash([]);
  }
  draw() {
    const c = this.ctx;
    if (!c || !this.width) return;
    c.clearRect(0, 0, this.width, this.height);
    c.fillStyle = "#eef2f3";
    c.fillRect(0, 0, this.width, this.height);
    const grid = this.scale * 10;
    c.fillStyle = "#d8e0e3";
    if (grid > 8)
      for (let x = ((this.ox % grid) + grid) % grid; x < this.width; x += grid)
        for (
          let y = (((this.height - this.oy) % grid) + grid) % grid;
          y < this.height;
          y += grid
        )
          c.fillRect(x, y, 1, 1);
    if (!this.scene) return;
    if (this.layers.obstacles)
      for (const group of this.groups) {
        const visual =
          group.layer.startsWith("VIS") || group.layer.startsWith("NAV");
        if (visual) continue;
        const closed =
          group.points.length > 3 &&
          group.points[0].x === group.points.at(-1).x &&
          group.points[0].y === group.points.at(-1).y;
        if (closed) {
          c.beginPath();
          group.points.forEach((p, i) => {
            const q = this.point(p.x, p.y);
            if (i) c.lineTo(q.x, q.y);
            else c.moveTo(q.x, q.y);
          });
          c.closePath();
          c.fillStyle = group.layer.includes("COLUMN")
            ? "#cbd5d8"
            : group.layer.includes("WALL")
              ? "#c1ccd2"
              : "#dbe2e5";
          c.fill();
        }
        this.line(
          group.points,
          group.layer.includes("WALL") ? "#a2b1ba" : "#bbc8cf",
          group.layer.includes("WALL") ? 1.4 : 0.65,
        );
      }
    const selectedRoute = this.state?.mapf?.agents?.[this.selected]?.route_id;
    if (this.layers.amr) {
      for (const edge of this.scene.amr_graph?.edges || []) {
        this.line(
          edge.coordinates.map(([x, y]) => ({ x, y })),
          "#9aa7b455",
          1,
        );
      }
    }
    if (this.layers.plan) {
      const paths =
        this.plan?.mapf_snapshot?.planned_paths ||
        this.state?.mapf?.planned_paths ||
        {};
      for (const [rid, points] of Object.entries(paths)) {
        if (this.selected && rid !== this.selected) continue;
        this.line(points, "#1f9ee7", 2);
      }
    }
    if (this.layers.routes && this.scene.agv_paths) {
      for (const [rid, points] of Object.entries(this.scene.agv_paths)) {
        const color =
          this.state?.robots?.find((r) => r.id === rid)?.color || "#57807c";
        this.line(points, color, 1.5);
      }
    }
    if (this.layers.routes && !this.scene.agv_paths)
      for (const route of this.scene.graph?.routes || []) {
        c.globalAlpha =
          selectedRoute && selectedRoute !== route.route_id ? 0.18 : 0.7;
        const points = route.node_ids
          .map((id) => this.nodes.get(id))
          .filter(Boolean);
        this.line(
          points,
          route.color || "#459387",
          selectedRoute === route.route_id ? 2.5 : 1.3,
        );
        for (let i = 1; i < points.length; i++) {
          const a = this.point(points[i - 1].x, points[i - 1].y),
            b = this.point(points[i].x, points[i].y);
          if (Math.hypot(a.x - b.x, a.y - b.y) < 25) continue;
          const x = (a.x + b.x) / 2,
            y = (a.y + b.y) / 2,
            ang = Math.atan2(b.y - a.y, b.x - a.x);
          c.save();
          c.translate(x, y);
          c.rotate(ang);
          c.fillStyle = route.color || "#459387";
          c.beginPath();
          c.moveTo(4, 0);
          c.lineTo(-3, -2.5);
          c.lineTo(-3, 2.5);
          c.fill();
          c.restore();
        }
      }
    c.globalAlpha = 1;
    const reservations =
      this.plan?.reservations || this.state?.mapf?.reservations || {};
    if (this.layers.zones) {
      const seen = new Set();
      for (const uses of Object.values(reservations))
        for (const use of uses) {
          if (seen.has(use.key)) continue;
          seen.add(use.key);
          if (use.key.startsWith("zone:")) {
            const n = this.nodes.get(use.key.slice(5));
            if (n) {
              const p = this.point(n.x, n.y);
              c.fillStyle = "#e8b95435";
              c.strokeStyle = "#d1a651";
              c.lineWidth = 1;
              c.beginPath();
              c.arc(p.x, p.y, Math.max(5, 3 * this.scale), 0, Math.PI * 2);
              c.fill();
              c.stroke();
            }
          } else {
            const e = this.scene.graph?.edges.find(
              (e) => "corridor:" + e.id === use.key,
            );
            if (e)
              this.line(
                e.coordinates,
                "#e9b95745",
                Math.max(4, 1.5 * this.scale),
              );
          }
        }
    }
    if (this.layers.plan && this.plan)
      for (const [rid, path] of Object.entries(this.plan.paths)) {
        if (this.selected && rid !== this.selected) continue;
        const robot = this.state?.robots?.find((r) => r.id === rid);
        this.line(
          path.map((n) => this.nodes.get(n)).filter(Boolean),
          robot?.color || "#1e756c",
          3,
          [5, 4],
        );
      }
    for (const station of this.stations || []) {
      const p = this.point(station.x, station.y);
      c.fillStyle = station.label === "З" ? "#fff" : "#ffefcf";
      c.strokeStyle = station.label === "З" ? "#148772" : "#c28f37";
      c.lineWidth = 1.5;
      c.beginPath();
      c.arc(p.x, p.y, 7, 0, Math.PI * 2);
      c.fill();
      c.stroke();
      c.font = "600 9px system-ui";
      c.fillStyle = c.strokeStyle;
      c.fillText(station.label, p.x - 3, p.y + 3);
    }
    if (this.editor && this.placement && selectedRoute)
      for (const n of this.scene.loops[selectedRoute] || []) {
        const p = this.point(n.x, n.y);
        c.fillStyle = "#598c8370";
        c.beginPath();
        c.arc(p.x, p.y, 2, 0, Math.PI * 2);
        c.fill();
      }
    const labelBoxes = [];
    for (const r of this.state?.robots || []) {
      const p = this.point(r.x, r.y),
        selected = r.id === this.selected;
      c.save();
      c.translate(p.x, p.y);
      if (selected) {
        c.strokeStyle = "#178675";
        c.lineWidth = 1.5;
        c.fillStyle = "#17867515";
        c.beginPath();
        c.arc(0, 0, Math.max(11, r.length * this.scale), 0, Math.PI * 2);
        c.fill();
        c.stroke();
      }
      c.rotate(-r.theta);
      const len = Math.max(9, r.length * this.scale),
        w = Math.max(5, r.width * this.scale);
      c.fillStyle = r.color || "#267f76";
      c.strokeStyle = "#fff";
      c.lineWidth = 1.4;
      c.beginPath();
      c.roundRect(-len / 2, -w / 2, len, w, 2);
      c.fill();
      c.stroke();
      c.fillStyle = "#fff";
      c.beginPath();
      c.moveTo(len / 2 - 1, 0);
      c.lineTo(len / 2 - 4, -2);
      c.lineTo(len / 2 - 4, 2);
      c.fill();
      c.restore();
      if (this.layers.labels || selected) {
        c.font = (selected ? "600 " : "") + "10px system-ui";
        c.fillStyle = "#3e5666";
        const tw = c.measureText(r.id).width;
        let label = null;
        for (const [dx, dy] of [
          [9, -7],
          [9, 12],
          [-tw - 10, -7],
          [-tw - 10, 12],
          [9, -21],
          [9, 26],
        ]) {
          const box = { x: p.x + dx, y: p.y + dy - 10, w: tw + 3, h: 13 };
          if (
            !labelBoxes.some(
              (b) =>
                box.x < b.x + b.w &&
                box.x + box.w > b.x &&
                box.y < b.y + b.h &&
                box.y + box.h > b.y,
            )
          ) {
            label = { box, dx, dy };
            break;
          }
        }
        if (label) {
          labelBoxes.push(label.box);
          c.fillText(r.id, p.x + label.dx, p.y + label.dy);
        }
      }
    }
    const distance = this.scale < 2 ? 50 : this.scale < 8 ? 10 : 2;
    const el = document.getElementById("mapScale");
    el.style.width = distance * this.scale + "px";
    el.textContent = distance + " м";
  }
}
