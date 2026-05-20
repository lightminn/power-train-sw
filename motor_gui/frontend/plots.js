// uPlot 패널 묶음. signal_meta 로 범례 라벨+단위 표시.
const WINDOW_SEC = 20;
const MAX_PTS = WINDOW_SEC * 100; // 100 Hz
const COLORS = ["#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8"];

function seriesLabel(key, meta) {
  const m = meta[key];
  if (!m) return key;
  return m.unit ? `${m.label} [${m.unit}]` : m.label;
}

function makePanel(title, sigKeys, meta) {
  const data = [[]];
  sigKeys.forEach(() => data.push([]));
  // actual 과 그 _setpoint 는 같은 색, setpoint 는 점선으로 짝지어 표시.
  const baseColor = {};
  let ci = 0;
  const series = [{ label: "t [s]" }].concat(sigKeys.map((k) => {
    const isSet = k.endsWith("_setpoint");
    const base = isSet ? k.slice(0, -9) : k; // "_setpoint".length === 9
    if (!(base in baseColor)) baseColor[base] = COLORS[ci++ % COLORS.length];
    const s = { label: seriesLabel(k, meta), stroke: baseColor[base] };
    if (isSet) s.dash = [6, 4];
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
      sigKeys.forEach((k, i) => data[i + 1].push(sample[k] ?? null));
      while (data[0].length > MAX_PTS) data.forEach((arr) => arr.shift());
    },
    redraw() { u.setData(data); },
  };
}

function buildPanels(signals, meta) {
  meta = meta || {};
  const has = (k) => signals.includes(k);
  const panels = [];
  if (has("odrive.pos") || has("odrive.vel"))
    panels.push(makePanel("ODrive 위치/속도 (실선=실제, 명령=setpoint)",
      ["odrive.pos", "odrive.pos_setpoint", "odrive.vel", "odrive.vel_setpoint"].filter(has), meta));
  if (has("odrive.iq_meas"))
    panels.push(makePanel("ODrive 전류(≈토크)", ["odrive.iq_meas", "odrive.iq_set"].filter(has), meta));
  if (has("odrive.temp_fet") || has("odrive.vbus"))
    panels.push(makePanel("ODrive 온도/버스", ["odrive.temp_fet", "odrive.vbus", "odrive.ibus"].filter(has), meta));
  if (has("ak.pos_deg"))
    panels.push(makePanel("AK 조향", ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp"].filter(has), meta));
  return panels;
}

window.MGPlots = { buildPanels };
