let panels = [];
let t0 = null;
let caps = null;
let lastState = null;
const lastErrs = {};

function logMsg(text, cls) {
  const log = document.getElementById("log");
  if (!log) return;
  const line = document.createElement("div");
  line.className = "logline" + (cls ? " " + cls : "");
  line.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
  log.appendChild(line);
  while (log.querySelectorAll(".logline").length > 200) {
    log.removeChild(log.querySelectorAll(".logline")[0]);
  }
  log.scrollTop = log.scrollHeight;
}

async function postCommand(envelope) {
  try {
    const r = await fetch("/api/command", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(envelope),
    });
    const ack = await r.json();
    if (!ack.ok) logMsg(`명령 거부: ${envelope.target}.${envelope.op} — ${ack.detail}`, "err");
    return ack;
  } catch (e) {
    logMsg(`명령 전송 실패: ${e}`, "err");
    return { ok: false, detail: String(e) };
  }
}

function el(tag, cls) { const e = document.createElement(tag); if (cls) e.className = cls; return e; }

function rowNumber(label, onSet, step, initial) {
  const row = el("div", "row");
  const lab = el("label"); lab.textContent = label; row.appendChild(lab);
  const inp = document.createElement("input");
  inp.type = "number"; inp.step = step || "0.1";
  if (initial !== undefined && initial !== null) inp.value = initial;
  inp.addEventListener("change", () => { const v = parseFloat(inp.value); if (!isNaN(v)) onSet(v); });
  row.appendChild(inp);
  return { row, inp, lab };
}
function rowSelect(label, options, onSet) {
  const row = el("div", "row");
  const lab = el("label"); lab.textContent = label; row.appendChild(lab);
  const sel = document.createElement("select");
  options.forEach((o) => { const op = document.createElement("option"); op.textContent = o; sel.appendChild(op); });
  sel.addEventListener("change", () => onSet(sel.value));
  row.appendChild(sel);
  return { row, sel };
}
function rowButton(label, onClick) {
  const row = el("div", "row");
  const b = document.createElement("button"); b.textContent = label;
  b.addEventListener("click", onClick); row.appendChild(b);
  return row;
}
function subhead(text) { const d = el("div", "subhead"); d.textContent = text; return d; }

function controlPanel(device, caps, tunVals) {
  const ops = (caps.commands && caps.commands[device]) || [];
  const wrap = el("div", "panel");
  const h = el("h3"); h.textContent = device; wrap.appendChild(h);

  if (ops.includes("set_state")) {
    wrap.appendChild(rowButton("폐루프 진입", () =>
      postCommand({ target: device, op: "set_state", args: { state: "closed_loop" } })));
    wrap.appendChild(rowButton("IDLE", () =>
      postCommand({ target: device, op: "set_state", args: { state: "idle" } })));
  }

  const modes = caps.control_modes && caps.control_modes[device];
  const inputs = caps.inputs && caps.inputs[device];
  if (modes && inputs) {
    let curMode = modes[0];
    const tgt = rowNumber("목표값", (v) => {
      const spec = inputs[curMode];
      postCommand({ target: device, op: "set_input", args: { [spec.key]: v } });
    });
    const applyMode = (m) => {
      curMode = m;
      const spec = inputs[m];
      tgt.lab.textContent = `${spec.label} [${spec.unit}]`;
    };
    const ms = rowSelect("제어 모드", modes, (m) => {
      postCommand({ target: device, op: "set_mode", args: { control_mode: m } });
      applyMode(m);
    });
    wrap.appendChild(ms.row);
    wrap.appendChild(tgt.row);
    applyMode(curMode);
    if (ops.includes("set_origin")) {
      wrap.appendChild(rowButton("영점 설정 (현재 위치를 0)", () => {
        postCommand({ target: device, op: "set_origin", args: {} });
        logMsg(`${device}: 현재 위치를 0 으로 재정의`);
      }));
    }
  } else if (ops.includes("set_input")) {
    const r = rowNumber("목표 위치 [°]", (v) =>
      postCommand({ target: device, op: "set_input", args: { pos_deg: v } }));
    wrap.appendChild(r.row);
    if (ops.includes("set_origin"))
      wrap.appendChild(rowButton("원점 설정", () =>
        postCommand({ target: device, op: "set_origin", args: {} })));
  }

  const tunables = caps.tunables && caps.tunables[device];
  if (tunables && tunables.length) {
    wrap.appendChild(subhead("튜닝 (현재값 prefill, 입력 후 Enter)"));
    const tv = (tunVals && tunVals[device]) || {};
    tunables.forEach((t) => {
      wrap.appendChild(rowNumber(t.label, (v) =>
        postCommand({ target: device, op: t.op, args: { [t.key]: v } }), "0.001", tv[t.key]).row);
    });
  }

  const actions = [];
  if (ops.includes("calibrate")) actions.push(["캘리브레이션", "calibrate"]);
  if (ops.includes("anticogging")) actions.push(["anticogging 캘리 (코깅 보상)", "anticogging"]);
  if (ops.includes("save_nvm")) actions.push(["NVM 저장", "save_nvm"]);
  if (ops.includes("clear_errors")) actions.push(["에러 클리어", "clear_errors"]);
  if (actions.length) {
    wrap.appendChild(subhead("동작"));
    actions.forEach(([label, op]) =>
      wrap.appendChild(rowButton(label, () => {
        postCommand({ target: device, op, args: {} });
        if (op === "anticogging")
          logMsg(`${device}: anticogging 캘리 시작 — 폐루프 position 상태에서 모터가 천천히 스윕합니다`);
      })));
  }
  return wrap;
}

const ERR_KEYS = ["odrive.axis_err", "odrive.motor_err", "odrive.enc_err", "odrive.ctrl_err"];
function monitorSample(s) {
  if ("error" in s) { logMsg(`샘플 에러: ${s.error}`, "err"); return; }
  if ("odrive.state" in s && s["odrive.state"] !== lastState) {
    if (lastState !== null) logMsg(`상태 변경: ${lastState} → ${s["odrive.state"]} (8=폐루프, 1=IDLE)`);
    lastState = s["odrive.state"];
  }
  ERR_KEYS.forEach((k) => {
    if (!(k in s)) return;
    const v = s[k] | 0;
    const prev = lastErrs[k] || 0;
    if (v !== prev) {
      if (v !== 0) logMsg(`${k} = 0x${v.toString(16)}`, "err");
      else logMsg(`${k} 해제`);
      lastErrs[k] = v;
    }
  });
  if ("ak.fault" in s) {
    const v = s["ak.fault"] | 0;
    if (v !== (lastErrs["ak.fault"] || 0)) {
      if (v !== 0) logMsg(`ak.fault = ${v}`, "err");
      lastErrs["ak.fault"] = v;
    }
  }
}

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
  ws.onopen = () => { document.getElementById("status").textContent = "● live"; logMsg("WS 연결됨"); };
  ws.onerror = () => logMsg("WS 오류", "err");
  ws.onclose = () => {
    document.getElementById("status").textContent = "○ reconnecting…";
    logMsg("WS 끊김 — 재연결", "err");
    setTimeout(connectWS, 1000);
  };
  ws.onmessage = (ev) => {
    const s = JSON.parse(ev.data);
    if (t0 === null) t0 = s.t_mono;
    panels.forEach((p) => p.push(s.t_mono - t0, s));
    monitorSample(s);
  };
}

function renderLoop() {
  panels.forEach((p) => p.redraw());
  requestAnimationFrame(renderLoop);
}

async function main() {
  caps = await (await fetch("/api/capabilities")).json();
  document.getElementById("track").textContent = `[${caps.track}]`;
  let tunVals = {};
  try {
    const flat = await (await fetch("/api/tunables")).json();
    // 백엔드는 odrive 기준 flat dict 반환 → device 별 맵으로 래핑 (현재는 odrive 만)
    tunVals = { odrive: flat };
  } catch (e) {
    logMsg("튜닝 현재값 조회 실패", "err");
  }
  const controls = document.getElementById("controls");
  caps.devices.forEach((d) => controls.appendChild(controlPanel(d, caps, tunVals)));
  document.getElementById("estop").addEventListener("click", () => {
    logMsg("E-STOP 발동", "err");
    postCommand({ target: caps.devices[0], op: "estop", args: {} });
  });
  panels = window.MGPlots.buildPanels(caps.signals, caps.signal_meta || {});
  logMsg(`연결: track=${caps.track} devices=${caps.devices.join(",")}`);
  connectWS();
  requestAnimationFrame(renderLoop);
}

main();
