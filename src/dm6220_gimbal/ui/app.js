const API = "/api/v1";
const $ = (id) => document.getElementById(id);
const yawHistory = [];
const pitchHistory = [];
const MAX_POINTS = 300;

function log(message) {
  const el = $("log");
  const t = new Date().toLocaleTimeString();
  el.textContent = `[${t}] ${message}\n` + el.textContent;
  el.textContent = el.textContent.split("\n").slice(0, 80).join("\n");
}

async function call(path, method = "GET", body = null) {
  const opts = { method, headers: {} };
  if (body !== null) {
    opts.headers["Content-Type"] = "application/json";
    opts.body = JSON.stringify(body);
  }
  const res = await fetch(API + path, opts);
  let data = null;
  try { data = await res.json(); } catch (_) {}
  if (!res.ok) {
    const detail = data && data.detail ? data.detail : res.statusText;
    log(`${method} ${path} failed: ${detail}`);
    throw new Error(detail);
  }
  log(`${method} ${path} ok`);
  return data;
}

function setBusy(button, busy) {
  button.disabled = busy;
}

async function action(path, button, body = null, method = "POST") {
  setBusy(button, true);
  try {
    const data = await call(path, method, body);
    if (data && data.snapshot) update(data.snapshot);
  } catch (_) {
  } finally {
    setBusy(button, false);
  }
}

$("btn-connect").onclick = () => action("/connect", $("btn-connect"));
$("btn-disconnect").onclick = () => action("/disconnect", $("btn-disconnect"));
$("btn-enable").onclick = () => action("/enable", $("btn-enable"));
$("btn-disable").onclick = () => action("/disable", $("btn-disable"));
$("btn-estop").onclick = () => action("/estop", $("btn-estop"));
$("btn-clear").onclick = () => action("/clear-fault", $("btn-clear"));

$("btn-go").onclick = () => {
  const yaw = Number($("yaw").value);
  const pitch = Number($("pitch").value);
  const duration = Number($("duration").value);
  action("/target/attitude", $("btn-go"), {
    yaw_deg: yaw,
    pitch_deg: pitch,
    duration_s: duration,
    ttl_ms: 5000,
    source: "ui",
  });
};

function badge(state) {
  const el = $("state-badge");
  el.textContent = state || "—";
  el.className = "badge";
  if (state === "ENABLED" || state === "RUNNING") el.classList.add("ok");
  else if (state === "FAULT" || state === "ESTOP") el.classList.add("danger");
  else if (state === "CONNECTED") el.classList.add("warn");
}

function fmt(value, digits = 3, suffix = "") {
  return value === null || value === undefined || Number.isNaN(value)
    ? "—"
    : Number(value).toFixed(digits) + suffix;
}

function update(snap) {
  if (!snap) return;
  badge(snap.state);
  const att = snap.attitude;
  $("att-yaw").textContent = att ? fmt(att.yaw_deg, 2, "°") : "—";
  $("att-pitch").textContent = att ? fmt(att.pitch_deg, 2, "°") : "—";
  const joints = snap.joints;
  $("joint-yaw").textContent = joints ? fmt(joints.yaw_rad, 4, " rad") : "—";
  $("joint-pitch").textContent = joints ? fmt(joints.pitch_rad, 4, " rad") : "—";
  const loop = snap.loop || {};
  $("loop").textContent = loop.count
    ? `${fmt(loop.rate_hz, 0)} Hz · p99 ${fmt(loop.jitter_ms_p99, 2)} ms · ${loop.overruns} overrun`
    : "—";
  $("fault").textContent = snap.fault || "—";

  const body = $("motor-body");
  body.innerHTML = "";
  const motors = snap.motors || {};
  for (const [name, m] of Object.entries(motors)) {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${name}</td>
      <td>${m.status} ${m.status_name || ""}</td>
      <td>${fmt(m.pos_rad, 4)}</td>
      <td>${fmt(m.raw_pos_rad, 4)}</td>
      <td>${fmt(m.vel_rad_s, 4)}</td>
      <td>${fmt(m.tau_nm, 4)}</td>
      <td>${fmt(m.t_mos_c, 0)} / ${fmt(m.t_rotor_c, 0)} °C</td>
      <td>${m.rx_age_ms === null ? "—" : fmt(m.rx_age_ms, 1, " ms")}</td>`;
    body.appendChild(tr);
  }

  if (att) {
    yawHistory.push(att.yaw_deg);
    pitchHistory.push(att.pitch_deg);
    if (yawHistory.length > MAX_POINTS) yawHistory.shift();
    if (pitchHistory.length > MAX_POINTS) pitchHistory.shift();
    drawChart();
  }
}

function drawChart() {
  const canvas = $("chart");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
  }
  const ctx = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = "#2b3644";
  ctx.lineWidth = 1;
  for (let i = 1; i < 5; i++) {
    const y = (h * i) / 5;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  const values = yawHistory.concat(pitchHistory);
  if (values.length < 2) return;
  let lo = Math.min(...values), hi = Math.max(...values);
  const pad = Math.max(1, (hi - lo) * 0.15);
  lo -= pad; hi += pad;
  const plot = (arr, color) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    for (let i = 0; i < arr.length; i++) {
      const x = (i / (MAX_POINTS - 1)) * w;
      const y = h - ((arr[i] - lo) / (hi - lo)) * h;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  };
  plot(yawHistory, "#58a6ff");
  plot(pitchHistory, "#d29922");
  ctx.font = `${12 * dpr}px sans-serif`;
  ctx.fillStyle = "#58a6ff";
  ctx.fillText("yaw", 10 * dpr, 16 * dpr);
  ctx.fillStyle = "#d29922";
  ctx.fillText("pitch", 52 * dpr, 16 * dpr);
}

let ws = null;
function connectWs() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`);
  ws.onopen = () => log("telemetry connected");
  ws.onmessage = (event) => {
    try { update(JSON.parse(event.data)); } catch (e) { console.error(e); }
  };
  ws.onclose = () => {
    log("telemetry disconnected; retrying");
    setTimeout(connectWs, 1500);
  };
  ws.onerror = () => ws.close();
}
connectWs();
