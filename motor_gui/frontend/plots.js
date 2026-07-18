// uPlot 패널 묶음. signal_meta 로 범례 라벨+단위 표시.
const WINDOW_SEC = 20;
const MAX_PTS = WINDOW_SEC * 100; // 100 Hz
const COLORS = ["#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8"];

// 속도 단위 (rev/s ↔ m/s). 휠 반경 0.1m. localStorage 로 app.js 와 공유.
const PLOT_VEL_CIRC = 2 * Math.PI * 0.1;   // app.js 의 VEL_CIRC 와 전역 충돌 방지 위해 별도 이름
const VEL_SIG = new Set(["odrive.vel", "odrive.vel_setpoint"]);
const WHEEL_REV_UNIT = "wheel rev/s";
function plotVelUnit() { return localStorage.getItem("velUnit") || "rev/s"; }
function plotVelFactor() { return plotVelUnit() === "m/s" ? PLOT_VEL_CIRC : 1; }

// 명령(점선) → 실제(실선) 매핑: 같은 색으로 짝지어 오버레이
const CMD_OF = {
  "odrive.pos_setpoint": "odrive.pos", "odrive.vel_setpoint": "odrive.vel",
  "ak.pos_cmd": "ak.pos_deg", "ak.speed_cmd": "ak.speed",
};

function seriesLabel(key, meta) {
  const m = meta[key];
  if (!m) return key;
  return m.unit ? `${m.label} [${m.unit}]` : m.label;
}

function makePanel(title, sigKeys, meta) {
  const data = [[]];
  sigKeys.forEach(() => data.push([]));
  const baseColor = {};
  let ci = 0;
  const series = [{ label: "t [s]" }].concat(sigKeys.map((k) => {
    const cmd = k in CMD_OF;
    const base = cmd ? CMD_OF[k] : k;       // 명령 시리즈는 실제와 같은 색
    if (!(base in baseColor)) baseColor[base] = COLORS[ci++ % COLORS.length];
    const s = { label: seriesLabel(k, meta), stroke: baseColor[base] };
    if (cmd) s.dash = [6, 4];               // 명령은 점선
    return s;
  }));
  const opts = {
    title, width: 820, height: 200,
    scales: { x: { time: false } },
    series,
    axes: [
      { stroke: "#888", grid: { stroke: "#222" } },
      { stroke: "#888", grid: { stroke: "#222" } },
    ],
  };
  const el = document.createElement("div");
  el.className = "panel";
  document.getElementById("plots").appendChild(el);
  const u = new uPlot(opts, data, el);
  return {
    push(t, sample) {
      data[0].push(t);
      const f = plotVelFactor();
      sigKeys.forEach((k, i) => {
        let v = sample[k];
        if (v != null && VEL_SIG.has(k)) v = v * f;   // 속도 신호는 선택 단위로 스케일
        data[i + 1].push(v ?? null);
      });
      while (data[0].length > MAX_PTS) data.forEach((arr) => arr.shift());
    },
    redraw() { u.setData(data); },
  };
}

function buildPanels(signals, meta) {
  meta = meta || {};
  meta = Object.assign({}, meta);
  const wheelUnit = plotVelUnit() === "m/s" ? "wheel m/s" : WHEEL_REV_UNIT;
  VEL_SIG.forEach((k) => {
    if (meta[k]) meta[k] = Object.assign({}, meta[k], { unit: wheelUnit });
  });
  const has = (k) => signals.includes(k);
  const panels = [];
  if (has("odrive.pos") || has("odrive.vel"))
    panels.push(makePanel("ODrive 위치/속도 (실선=실제, 명령=setpoint)",
      ["odrive.pos", "odrive.pos_setpoint", "odrive.vel", "odrive.vel_setpoint"].filter(has), meta));
  if (has("odrive.iq_meas"))
    panels.push(makePanel("ODrive 전류 (Iq=토크축, Id=자속축)",
      ["odrive.iq_meas", "odrive.iq_set", "odrive.id_meas", "odrive.id_set"].filter(has), meta));
  if (has("odrive.torque_est"))
    panels.push(makePanel("ODrive 추정 토크 (Iq×Kt)", ["odrive.torque_est"].filter(has), meta));
  if (has("odrive.temp_fet") || has("odrive.vbus"))
    panels.push(makePanel("ODrive 온도/버스", ["odrive.temp_fet", "odrive.vbus", "odrive.ibus"].filter(has), meta));
  if (has("ak.pos_deg"))
    panels.push(makePanel("AK 위치 (실선=실제, 점선=명령)",
      ["ak.pos_deg", "ak.pos_cmd"].filter(has), meta));
  if (has("ak.speed"))
    panels.push(makePanel("AK 속도 (실선=실제, 점선=명령)",
      ["ak.speed", "ak.speed_cmd"].filter(has), meta));
  if (has("ak.current") || has("ak.temp"))
    panels.push(makePanel("AK 전류/온도", ["ak.current", "ak.temp"].filter(has), meta));
  return panels;
}

window.MGPlots = { buildPanels };
