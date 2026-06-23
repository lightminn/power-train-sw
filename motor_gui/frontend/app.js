let panels = [];
let t0 = null;
let caps = null;
let lastState = null;
const lastErrs = {};

// fw 0.5.x ODrive 에러 비트 → 이름 (axis/motor/encoder/controller)
const ERROR_BITS = {
  "odrive.axis_err": {
    0x1: "INVALID_STATE", 0x2: "DC_BUS_UNDER_VOLTAGE", 0x4: "DC_BUS_OVER_VOLTAGE",
    0x8: "CURRENT_MEASUREMENT_TIMEOUT", 0x10: "BRAKE_RESISTOR_DISARMED",
    0x20: "MOTOR_DISARMED", 0x40: "MOTOR_FAILED", 0x80: "SENSORLESS_ESTIMATOR_FAILED",
    0x100: "ENCODER_FAILED", 0x200: "CONTROLLER_FAILED", 0x400: "POS_CTRL_DURING_SENSORLESS",
    0x800: "WATCHDOG_TIMER_EXPIRED", 0x1000: "MIN_ENDSTOP_PRESSED",
    0x2000: "MAX_ENDSTOP_PRESSED", 0x4000: "ESTOP_REQUESTED", 0x20000: "OVER_TEMP",
    0x40000: "UNKNOWN_POSITION",
  },
  "odrive.motor_err": {
    0x1: "PHASE_RESISTANCE_OUT_OF_RANGE", 0x2: "PHASE_INDUCTANCE_OUT_OF_RANGE",
    0x8: "DRV_FAULT", 0x10: "CONTROL_DEADLINE_MISSED", 0x80: "MODULATION_MAGNITUDE",
    0x400: "CURRENT_SENSE_SATURATION", 0x1000: "CURRENT_LIMIT_VIOLATION",
    0x10000: "MODULATION_IS_NAN", 0x20000: "MOTOR_THERMISTOR_OVER_TEMP",
    0x40000: "FET_THERMISTOR_OVER_TEMP", 0x100000: "TIMER_UPDATE_MISSED",
  },
  "odrive.enc_err": {
    0x1: "UNSTABLE_GAIN", 0x2: "CPR_POLEPAIRS_MISMATCH", 0x4: "NO_RESPONSE",
    0x8: "UNSUPPORTED_ENCODER_MODE", 0x10: "ILLEGAL_HALL_STATE",
    0x20: "INDEX_NOT_FOUND_YET", 0x40: "ABS_SPI_TIMEOUT", 0x80: "ABS_SPI_COM_FAIL",
    0x100: "ABS_SPI_NOT_READY",
  },
  "odrive.ctrl_err": {
    0x1: "OVERSPEED", 0x2: "INVALID_INPUT_MODE", 0x4: "UNSTABLE_GAIN",
    0x8: "INVALID_MIRROR_AXIS", 0x10: "INVALID_LOAD_ENCODER", 0x20: "INVALID_ESTIMATE",
    0x40: "INVALID_CIRCULAR_RANGE", 0x80: "SPINOUT_DETECTED",
  },
};

function decodeErr(key, value) {
  const v = value | 0;
  if (v === 0) return "0x0";
  const map = ERROR_BITS[key] || {};
  const names = [];
  for (const bit in map) {
    if (v & Number(bit)) names.push(map[bit]);
  }
  const hex = "0x" + (v >>> 0).toString(16);
  return names.length ? `${hex} (${names.join(" | ")})` : hex;
}

// VESC/AK mc_fault_code (enum 값 → 이름; 비트필드 아님)
const AK_FAULT_CODES = {
  0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV", 4: "ABS_OVER_CURRENT",
  5: "OVER_TEMP_FET", 6: "OVER_TEMP_MOTOR", 7: "GATE_DRIVER_OVER_VOLTAGE",
  8: "GATE_DRIVER_UNDER_VOLTAGE", 9: "MCU_UNDER_VOLTAGE", 10: "BOOTING_FROM_WATCHDOG_RESET",
  11: "ENCODER_SPI", 12: "ENCODER_SINCOS_BELOW_MIN", 13: "ENCODER_SINCOS_ABOVE_MAX",
  14: "FLASH_CORRUPTION", 18: "UNBALANCED_CURRENTS",
};
function decodeAkFault(v) {
  return AK_FAULT_CODES[v] !== undefined ? `${v} (${AK_FAULT_CODES[v]})` : String(v);
}

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

function rowNumber(label, onSet, step, initial, title) {
  const row = el("div", "row");
  const lab = el("label"); lab.textContent = label; row.appendChild(lab);
  const inp = document.createElement("input");
  inp.type = "number"; inp.step = step || "0.1";
  if (title) { lab.title = title; inp.title = title; }
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
function helpLine(text) {
  const d = el("div");
  d.textContent = text;
  d.style.cssText = "grid-column:1/-1; font-size:.72rem; opacity:.6; margin:0 0 .35rem .2rem;";
  return d;
}

function motorInfoPanel(caps) {
  const info = caps.motor_info && caps.motor_info.odrive;
  if (!info) return null;
  const wrap = el("div", "panel");
  const h = el("h3"); h.textContent = "모터 정보 (캘리브레이션 결과)"; wrap.appendChild(h);
  const fmt = {
    phase_resistance: ["상 저항 R", "Ω", 4],
    phase_inductance: ["상 인덕턴스 L", "H", 7],
    torque_constant: ["토크 상수 Kt", "Nm/A", 4],
    pole_pairs: ["극쌍수", "", 0],
    current_lim: ["전류 한계", "A", 1],
  };
  Object.keys(fmt).forEach((k) => {
    if (!(k in info)) return;
    const [label, unit, dp] = fmt[k];
    const row = el("div", "row");
    const lab = el("label"); lab.textContent = label; lab.style.width = "120px";
    const val = el("span");
    const num = Number(info[k]);
    val.textContent = (dp ? num.toFixed(dp) : String(num)) + (unit ? " " + unit : "");
    row.appendChild(lab); row.appendChild(val); wrap.appendChild(row);
  });
  return wrap;
}

function recordingPanel() {
  const wrap = el("div", "panel");
  const h = el("h3"); h.textContent = "로깅 (CSV)"; wrap.appendChild(h);
  const startBtn = document.createElement("button");
  startBtn.textContent = "● 로깅 시작";
  const stopBtn = document.createElement("button");
  stopBtn.textContent = "■ 로깅 종료";
  stopBtn.disabled = true;
  const setState = (rec) => { startBtn.disabled = rec; stopBtn.disabled = !rec; };
  startBtn.addEventListener("click", async () => {
    try {
      const ack = await (await fetch("/api/record/start", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ fmt: "csv" }),
      })).json();
      if (ack.ok) { setState(true); logMsg("로깅 시작 — " + ack.detail); }
      else logMsg("로깅 시작 실패: " + ack.detail, "err");
    } catch (e) { logMsg("로깅 시작 오류: " + e, "err"); }
  });
  stopBtn.addEventListener("click", async () => {
    try {
      const ack = await (await fetch("/api/record/stop", { method: "POST" })).json();
      setState(false); logMsg("로깅 종료 — " + ack.detail);
    } catch (e) { logMsg("로깅 종료 오류: " + e, "err"); }
  });
  const row = el("div", "row");
  row.appendChild(startBtn); row.appendChild(stopBtn);
  wrap.appendChild(row);
  return wrap;
}

function canIdPanel(caps) {
  const ids = caps.can_ids;
  if (!ids || !Object.keys(ids).length) return null;
  const wrap = el("div", "panel");
  const h = el("h3"); h.textContent = "CAN ID 선택"; wrap.appendChild(h);
  const inputs = {};
  Object.keys(ids).forEach((dev) => {
    const spec = ids[dev];
    const r = rowNumber(spec.label || dev, () => {}, "1", spec.id,
      `${dev} CAN ID (${spec.min}~${spec.max}). 적용 시 해당 ID로 재연결.`);
    inputs[dev] = r.inp;
    wrap.appendChild(r.row);
  });
  const btn = document.createElement("button");
  btn.textContent = "적용 (재연결)";
  btn.addEventListener("click", async () => {
    const payload = {};
    Object.keys(inputs).forEach((dev) => {
      const v = parseInt(inputs[dev].value, 10);
      if (!isNaN(v)) payload[dev] = v;
    });
    btn.disabled = true;
    logMsg(`CAN ID 적용: ${JSON.stringify(payload)} — 재연결…`);
    try {
      const ack = await (await fetch("/api/can_id", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ids: payload }),
      })).json();
      if (ack.ok) {
        logMsg("CAN ID 변경·재연결 성공 — 새로고침");
        setTimeout(() => location.reload(), 400);
      } else {
        logMsg("CAN ID 변경 실패: " + (ack.detail || ""), "err");
        btn.disabled = false;
      }
    } catch (e) {
      logMsg("CAN ID 변경 오류: " + e, "err");
      btn.disabled = false;
    }
  });
  const brow = el("div", "row"); brow.appendChild(btn); wrap.appendChild(brow);
  return wrap;
}

function controlPanel(device, caps, tunVals) {
  const ops = (caps.commands && caps.commands[device]) || [];
  const wrap = el("div", "panel");
  const h = el("h3"); h.textContent = device; wrap.appendChild(h);
  const grid = el("div", "pgrid");
  wrap.appendChild(grid);

  if (ops.includes("set_state")) {
    grid.appendChild(rowButton("폐루프 진입", () =>
      postCommand({ target: device, op: "set_state", args: { state: "closed_loop" } })));
    grid.appendChild(rowButton("IDLE", () =>
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
    const modeHelp = helpLine("");
    const applyMode = (m) => {
      curMode = m;
      const spec = inputs[m];
      tgt.lab.textContent = `${spec.label} [${spec.unit}]`;
      modeHelp.textContent = spec.help || "";
    };
    const ms = rowSelect("제어 모드", modes, (m) => {
      postCommand({ target: device, op: "set_mode", args: { control_mode: m } });
      applyMode(m);
    });
    grid.appendChild(ms.row);
    grid.appendChild(tgt.row);
    grid.appendChild(modeHelp);
    applyMode(curMode);
    if (ops.includes("set_origin")) {
      grid.appendChild(rowButton("영점 설정 (현재 위치를 0)", () => {
        postCommand({ target: device, op: "set_origin", args: {} });
        logMsg(`${device}: 현재 위치를 0 으로 재정의`);
      }));
      grid.appendChild(helpLine("현재 물리 위치를 0 기준으로 재정의합니다."));
    }
  } else if (ops.includes("set_input")) {
    const r = rowNumber("목표 위치 [°]", (v) =>
      postCommand({ target: device, op: "set_input", args: { pos_deg: v } }));
    grid.appendChild(r.row);
    if (ops.includes("set_origin"))
      grid.appendChild(rowButton("원점 설정", () =>
        postCommand({ target: device, op: "set_origin", args: {} })));
  }

  const tunables = caps.tunables && caps.tunables[device];
  if (tunables && tunables.length) {
    grid.appendChild(subhead("튜닝 (현재값 prefill, 입력 후 Enter)"));
    const tv = (tunVals && tunVals[device]) || {};
    tunables.forEach((t) => {
      const init = (t.value !== undefined && t.value !== null) ? t.value : tv[t.key];
      grid.appendChild(rowNumber(t.label, (v) =>
        postCommand({ target: device, op: t.op, args: { [t.key]: v } }), "0.001", init, t.help).row);
    });
  }

  const actions = [];
  if (ops.includes("calibrate")) actions.push(["캘리브레이션", "calibrate"]);
  if (ops.includes("save_nvm")) actions.push(["NVM 저장", "save_nvm"]);
  if (ops.includes("clear_errors")) actions.push(["에러 클리어", "clear_errors"]);
  if (actions.length) {
    grid.appendChild(subhead("동작"));
    actions.forEach(([label, op]) =>
      grid.appendChild(rowButton(label, () => postCommand({ target: device, op, args: {} }))));
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
      if (v !== 0) logMsg(`${k.replace("odrive.", "")} = ${decodeErr(k, v)}`, "err");
      else logMsg(`${k.replace("odrive.", "")} 해제`);
      lastErrs[k] = v;
    }
  });
  if ("ak.fault" in s) {
    const v = s["ak.fault"] | 0;
    if (v !== (lastErrs["ak.fault"] || 0)) {
      if (v !== 0) logMsg(`ak.fault = ${decodeAkFault(v)}`, "err");
      lastErrs["ak.fault"] = v;
    }
  }
  if ("ak.tripped" in s) {
    const v = s["ak.tripped"] | 0;
    if (v !== (lastErrs["ak.tripped"] || 0)) {
      if (v !== 0) logMsg("AK 과전류 자동정지 — 명령 해제됨 (모드 재설정 필요)", "err");
      else logMsg("AK 과전류정지 해제");
      lastErrs["ak.tripped"] = v;
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
  controls.appendChild(recordingPanel());
  const cidp = canIdPanel(caps);
  if (cidp) controls.appendChild(cidp);
  const mi = motorInfoPanel(caps);
  if (mi) controls.appendChild(mi);
  caps.devices.forEach((d) => controls.appendChild(controlPanel(d, caps, tunVals)));
  document.getElementById("estop").title = "모든 모터 즉시 정지";
  document.getElementById("estop").addEventListener("click", () => {
    logMsg("E-STOP 발동", "err");
    postCommand({ target: caps.devices[0], op: "estop", args: {} });
  });
  const rcBtn = document.getElementById("reconnect");
  if (rcBtn) rcBtn.addEventListener("click", async () => {
    rcBtn.disabled = true;
    logMsg("하드웨어 재연결 시도…");
    try {
      const ack = await (await fetch("/api/reconnect", { method: "POST" })).json();
      if (ack.ok) logMsg("재연결 성공 — " + (ack.detail || ""));
      else logMsg("재연결 실패: " + (ack.detail || ""), "err");
    } catch (e) {
      logMsg("재연결 오류: " + e, "err");
    }
    rcBtn.disabled = false;
  });
  panels = window.MGPlots.buildPanels(caps.signals, caps.signal_meta || {});
  logMsg(`연결: track=${caps.track} devices=${caps.devices.join(",")}`);
  connectWS();
  requestAnimationFrame(renderLoop);
}

main();
