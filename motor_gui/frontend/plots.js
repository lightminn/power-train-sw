// uPlot 패널 묶음. 각 패널은 신호 키 리스트를 plot.
const WINDOW_SEC = 20;
const MAX_PTS = WINDOW_SEC * 100; // 100 Hz

function makePanel(title, sigKeys) {
  const data = [[]];                       // [x, ...series]
  sigKeys.forEach(() => data.push([]));
  const opts = {
    title, width: 800, height: 200,
    scales: { x: { time: false } },
    series: [{ label: "t" }].concat(
      sigKeys.map((k, i) => ({
        label: k, stroke: ["#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8"][i % 5],
      }))),
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
    keys: sigKeys,
    push(t, sample) {
      data[0].push(t);
      sigKeys.forEach((k, i) => data[i + 1].push(sample[k] ?? null));
      while (data[0].length > MAX_PTS) data.forEach((arr) => arr.shift());
    },
    redraw() { u.setData(data); },
  };
}

// 신호 키 → 패널 그룹핑 규칙
function buildPanels(signals) {
  const has = (k) => signals.includes(k);
  const panels = [];
  if (has("odrive.pos") || has("odrive.vel"))
    panels.push(makePanel("ODrive 위치/속도",
      ["odrive.pos", "odrive.vel"].filter(has)));
  if (has("odrive.iq_meas"))
    panels.push(makePanel("ODrive 전류(토크)",
      ["odrive.iq_meas", "odrive.iq_set"].filter(has)));
  if (has("odrive.temp_fet") || has("odrive.vbus"))
    panels.push(makePanel("ODrive 온도/버스",
      ["odrive.temp_fet", "odrive.vbus", "odrive.ibus"].filter(has)));
  if (has("ak.pos_deg"))
    panels.push(makePanel("AK 조향",
      ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp"].filter(has)));
  return panels;
}

window.MGPlots = { buildPanels };
