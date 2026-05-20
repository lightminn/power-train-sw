let panels = [];
let t0 = null;

async function postCommand(envelope) {
  const r = await fetch("/api/command", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(envelope),
  });
  return r.json();
}

function controlPanel(device, caps) {
  const ops = caps.commands[device];
  const wrap = document.createElement("div");
  wrap.className = "panel";
  wrap.innerHTML = `<h3>${device}</h3>`;

  if (ops.includes("set_mode")) {
    wrap.appendChild(rowSelect("control_mode", ["position", "velocity", "torque"],
      (v) => postCommand({ target: device, op: "set_mode", args: { control_mode: v } })));
  }
  if (ops.includes("set_input")) {
    const key = device === "ak" ? "pos_deg" : "vel";
    wrap.appendChild(rowNumber(key, (v) =>
      postCommand({ target: device, op: "set_input", args: { [key]: v } })));
  }
  if (ops.includes("set_gain")) {
    wrap.appendChild(rowNumber("vel_gain", (v) =>
      postCommand({ target: device, op: "set_gain", args: { vel_gain: v } })));
  }
  if (ops.includes("calibrate")) {
    wrap.appendChild(rowButton("캘리브레이션", () =>
      postCommand({ target: device, op: "calibrate", args: {} })));
  }
  if (ops.includes("save_nvm")) {
    wrap.appendChild(rowButton("NVM 저장", () =>
      postCommand({ target: device, op: "save_nvm", args: {} })));
  }
  if (ops.includes("clear_errors")) {
    wrap.appendChild(rowButton("에러 클리어", () =>
      postCommand({ target: device, op: "clear_errors", args: {} })));
  }
  return wrap;
}

function rowNumber(label, onSet) {
  const row = el("div", "row");
  row.innerHTML = `<label>${label}</label><input type="number" step="0.1" />`;
  const inp = row.querySelector("input");
  inp.addEventListener("change", () => onSet(parseFloat(inp.value)));
  return row;
}
function rowSelect(label, options, onSet) {
  const row = el("div", "row");
  row.innerHTML = `<label>${label}</label><select>${
    options.map((o) => `<option>${o}</option>`).join("")}</select>`;
  const sel = row.querySelector("select");
  sel.addEventListener("change", () => onSet(sel.value));
  return row;
}
function rowButton(label, onClick) {
  const row = el("div", "row");
  const b = document.createElement("button");
  b.textContent = label;
  b.addEventListener("click", onClick);
  row.appendChild(b);
  return row;
}
function el(tag, cls) { const e = document.createElement(tag); e.className = cls; return e; }

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
  ws.onopen = () => (document.getElementById("status").textContent = "● live");
  ws.onclose = () => {
    document.getElementById("status").textContent = "○ reconnecting…";
    setTimeout(connectWS, 1000);
  };
  ws.onmessage = (ev) => {
    const s = JSON.parse(ev.data);
    if (t0 === null) t0 = s.t_mono;
    const t = s.t_mono - t0;
    panels.forEach((p) => p.push(t, s));
  };
}

function renderLoop() {
  panels.forEach((p) => p.redraw());
  requestAnimationFrame(renderLoop);   // 디스플레이는 ~60fps, 수집은 100Hz
}

async function main() {
  const caps = await (await fetch("/api/capabilities")).json();
  document.getElementById("track").textContent = `[${caps.track}]`;
  const controls = document.getElementById("controls");
  caps.devices.forEach((d) => controls.appendChild(controlPanel(d, caps)));
  document.getElementById("estop").addEventListener("click", () =>
    postCommand({ target: caps.devices[0], op: "estop", args: {} }));
  panels = window.MGPlots.buildPanels(caps.signals);
  connectWS();
  requestAnimationFrame(renderLoop);
}

main();
