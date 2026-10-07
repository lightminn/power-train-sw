/* 텔레옵 프론트엔드 — 키보드 + 게임패드를 브라우저에서 직접 읽는다.
 *
 * `control.js` 가 제어 모드에서만 `Teleop.init()` 을 부른다. 읽기 전용 모드에서는
 * 이 파일이 이벤트 리스너를 하나도 달지 않는다.
 *
 * ## 왜 브라우저가 입력을 읽나
 *
 * 기존 프론트엔드 둘은 각각 제약이 있다 — `keyboard_teleop` 은 tty 포커스가 필요해
 * launch 에 못 넣고(별도 터미널 필수), `joystick_teleop` 은 패드가 **Jetson 에**
 * 꽂혀 있어야 한다. SSH 로 붙은 원격 PC 에서는 둘 다 곤란하다. 브라우저는 키
 * 이벤트와 Gamepad API 를 이미 갖고 있어서, 원격 PC 의 키보드/패드가 그대로 쓰인다.
 *
 * ## curses 의 우회가 사라진다
 *
 * `keyboard_teleop_node` 는 키 릴리스 이벤트가 없어서 `HOLD_TIMEOUT_S=0.15` 동안
 * 키가 다시 안 보이면 "뗐다"로 **추측**해야 했다. 브라우저에는 `keyup` 이 있어
 * 그 추측이 통째로 없어진다 — 떼는 순간 정확히 멈춘다.
 *
 * ## 데드맨은 한 곳으로 수렴한다
 *
 * keyup · 창 포커스 상실 · 탭 숨김 · 페이지 이탈 · 패드 연결 해제 · 조종권 상실은
 * 전부 같은 `stopNow()` 로 간다. 어느 경로로 들어와도 결과는 "의도를 비우고 서버에
 * 해제를 알린다" 하나뿐이라, 경로가 하나 늘 때마다 안전 로직을 새로 쓰지 않는다.
 *
 * ## 전송
 *
 * 서버가 `teleop_publish_hz`(기본 20Hz)로 `/arm/teleop_jog` 를 재발행하므로, 여기서는
 * 같은 주기로 **의도만** 올린다. 움직임이 없으면 아무 요청도 보내지 않는다 —
 * 브라우저 연결이 오리진당 6개인데 SSE+MJPEG 가 이미 둘을 쓰고 있어서, 유휴 상태에서
 * 20Hz 로 POST 를 계속 밀면 연결이 아깝다. 이전 요청이 안 끝났으면 그 프레임은
 * 건너뛴다(낡은 속도를 뒤늦게 보내는 것보다 최신 값 한 번이 낫다).
 */
'use strict';

/* 파일 스코프 IIFE — 이 네 스크립트(app/teleop/calib/control)는 모듈이 아니라
 * classic script 라 최상위 `const`/`let` 이 **전역 렉시컬 스코프를 공유**한다.
 * 같은 이름을 두 파일이 선언하면 나중 파일이 통째로
 * `SyntaxError: Identifier 'x' has already been declared` 로 죽는데, 전역
 * 에러 핸들러가 없어서 **화면상으로는 그냥 제어 UI 가 안 나타날 뿐**이라
 * 원인을 찾기가 매우 어렵다(실제로 `el` 중복으로 calib/control 이 둘 다 죽어
 * "제어 모드인데 읽기 전용으로 보인다"를 두 세션에 걸쳐 디버깅했다).
 * 파일 간 참조는 전부 `window.*` 를 거치므로 가둬도 안전하다. */
(() => {

const T = {
  desc: null,
  joints: [],
  selected: 0,
  jogVel: 0.7,
  jogDelta: 0.2,
  maxVel: 1.0,
  pad: null,          // 게임패드 설정 {deadzone, deadman, joints:{name:{axis,scale,inverted,plus,minus}}}
  padIndex: null,     // 연결된 패드의 navigator.getGamepads() 인덱스
  padWarn: '',
  bindMode: false,    // 다음에 누른 버튼을 데드맨으로 바인딩
  slotMode: null,     // 'save' | 'goto' — 다음 숫자키가 슬롯 번호
  hint: '',
};
window.Teleop = T;

const LS_KEY = 'robot_arm_gui.gamepad';
const el = (id) => document.getElementById(id);

/* 키보드 조그 상태. 여러 키를 겹쳐 눌러도 마지막으로 눌린 방향을 쓴다.
 *
 * ⚠️ 키는 소문자로 정규화해서 담는다. `w` 를 누른 채 Shift 를 눌렀다 떼면 keydown 은
 * `w`, keyup 은 `W` 로 와서 — 정규화하지 않으면 **놓았는데 눌린 상태로 남는다.** */
const heldKeys = new Set();
const JOG_PLUS = new Set(['ArrowUp', 'w']);
const JOG_MINUS = new Set(['ArrowDown', 's']);

function normKey(ev) {
  return ev.key.length === 1 ? ev.key.toLowerCase() : ev.key;
}

/* ── 전송 루프 ─────────────────────────────────────────── */
let loopTimer = null;
let inflight = false;
let wasMoving = false;
let stopQueued = false;

function anyMotion(intent) {
  return Object.values(intent).some((v) => Math.abs(v) > 1e-6);
}

function computeIntent() {
  const out = {};
  const add = (name, v) => { out[name] = (out[name] || 0) + v; };

  // 키보드 — 선택된 축 하나만 움직인다(TUI 와 동일).
  const dir = keyDirection();
  if (dir !== 0 && T.joints[T.selected]) add(T.joints[T.selected], dir * T.jogVel);

  // 게임패드 — 데드맨을 누르고 있는 동안에만.
  const gp = livePad();
  if (gp && T.pad && gp.buttons[T.pad.deadman] && gp.buttons[T.pad.deadman].pressed) {
    for (const name of T.joints) {
      const m = T.pad.joints[name];
      if (!m) continue;
      if (m.axis >= 0) {
        const v = applyDeadzone(gp.axes[m.axis] || 0, T.pad.deadzone);
        if (v !== 0) add(name, (m.inverted ? -v : v) * m.scale);
      }
      if (m.plus >= 0 && gp.buttons[m.plus] && gp.buttons[m.plus].pressed) add(name, m.scale);
      if (m.minus >= 0 && gp.buttons[m.minus] && gp.buttons[m.minus].pressed) add(name, -m.scale);
    }
  }

  // 서버가 다시 clamp 하지만(권위는 teleop_core 의 max_vel_rad_s), 화면에 그리는
  // 값과 실제 나가는 값이 다르면 안 되니 여기서도 같은 상한을 적용한다.
  for (const name of Object.keys(out)) {
    out[name] = Math.max(-T.maxVel, Math.min(T.maxVel, out[name]));
  }
  return out;
}

function keyDirection() {
  let dir = 0;
  for (const key of heldKeys) {
    if (JOG_PLUS.has(key)) dir = 1;
    else if (JOG_MINUS.has(key)) dir = -1;
  }
  return dir;
}

function applyDeadzone(v, dz) {
  const a = Math.abs(v);
  if (a <= dz) return 0;
  return Math.sign(v) * ((a - dz) / (1 - dz));
}

function pump() {
  const C = window.Control;
  const intent = C.held() ? computeIntent() : {};
  const moving = anyMotion(intent);

  if (moving) {
    wasMoving = true;
    if (!inflight) sendJog(intent);
  } else if (wasMoving) {
    wasMoving = false;
    stopQueued = true;
  }
  if (stopQueued && !inflight && C.held()) {
    stopQueued = false;
    sendRelease();
  }
  render(intent);
}

async function sendJog(intent) {
  inflight = true;
  try {
    await window.Control.post('/api/teleop/jog',
                              { token: window.Control.token, velocities: intent });
  } catch (err) {
    // 조종권을 잃었다(만료·강탈) — 계속 밀어봐야 전부 409 다. 즉시 손을 뗀다.
    T.hint = `조그 전송 실패: ${err.message}`;
    onTokenLost();
  } finally {
    inflight = false;
  }
}

/* 정지는 "전송을 끊는다"가 아니라 **명시적으로 알린다.** 그냥 끊으면 서버 워치독이
 * 같은 일을 하지만, 그러면 키를 뗄 때마다 통신 두절 경고가 감사 로그에 쌓인다. */
async function sendRelease() {
  inflight = true;
  try {
    await window.Control.post('/api/teleop/release_jog', { token: window.Control.token });
  } catch (err) {
    onTokenLost();
  } finally {
    inflight = false;
  }
}

/* 데드맨 — 모든 중단 경로가 여기로 모인다.
 *
 * 해제를 여기서 직접 쏘지 않고 `pump()` 에 맡기는 이유: 조그 요청이 아직 날아가는
 * 중일 수 있고, 그 둘이 서로 다른 연결로 나가면 **해제가 먼저 도착하고 낡은 조그가
 * 뒤에 도착**할 수 있다. 그러면 서버가 다시 "신선한 의도"로 보고 워치독 시간만큼
 * 더 움직인다. pump 는 in-flight 가 없을 때만 해제를 보내므로 그 역전이 없다.
 */
function stopNow(reason) {
  heldKeys.clear();
  T.slotMode = null;
  if (reason) T.hint = reason;
  wasMoving = false;
  stopQueued = true;
  pump();
}
T.stopNow = stopNow;

function onTokenLost() {
  window.Control.token = null;
  heldKeys.clear();
  wasMoving = false;
  stopQueued = false;
}

/* ── 이산 명령 ─────────────────────────────────────────── */
async function sendCmd(cmd) {
  if (!window.Control.held()) { T.hint = '조종권을 먼저 획득하세요'; render({}); return; }
  try {
    await window.Control.post('/api/teleop/cmd', { token: window.Control.token, cmd });
    T.hint = `명령 전송: ${cmd}`;
  } catch (err) {
    T.hint = `✖ ${cmd} — ${err.message}`;
  }
  render({});
}
T.sendCmd = sendCmd;

/* ── 키보드 ────────────────────────────────────────────── */
function isTyping() {
  const node = document.activeElement;
  if (!node) return false;
  const tag = node.tagName;
  return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || node.isContentEditable;
}

function onKeyDown(ev) {
  if (isTyping() || ev.ctrlKey || ev.altKey || ev.metaKey) return;
  const key = normKey(ev);

  // auto-repeat 은 무시한다 — 누르고 있는 상태는 heldKeys 가 이미 알고 있다.
  if (ev.repeat) { if (JOG_PLUS.has(key) || JOG_MINUS.has(key) || key === ' ') ev.preventDefault(); return; }

  // 자세 슬롯 입력 모드: 숫자키가 축 선택이 아니라 슬롯 번호다.
  if (T.slotMode && /^[1-9]$/.test(key)) {
    ev.preventDefault();
    const mode = T.slotMode;
    T.slotMode = null;
    sendCmd(`${mode} ${key}`);
    return;
  }
  if (T.slotMode && key === 'Escape') {
    ev.preventDefault();
    const mode = T.slotMode;
    T.slotMode = null;
    if (mode === 'save') sendCmd('freedrive_cancel');   // 저장 없이 토크만 복귀
    else render({});
    return;
  }

  if (JOG_PLUS.has(key) || JOG_MINUS.has(key)) {
    ev.preventDefault();
    if (!window.Control.held()) { T.hint = '조종권을 먼저 획득하세요'; render({}); return; }
    heldKeys.add(key);
    return;
  }

  if (/^[1-9]$/.test(key)) {
    const idx = Number(key) - 1;
    if (idx < T.joints.length) {
      // 축을 바꾸는 순간 이전 축이 마지막 속도로 남지 않게 손을 뗀 것으로 본다.
      heldKeys.clear();
      T.selected = idx;
      T.hint = `축 선택: ${T.joints[idx]}`;
      render({});
    }
    return;
  }

  switch (key) {
    case ' ':
      ev.preventDefault();
      stopNow('space — 즉시 정지 + 토크 차단 (t 로 복귀)');
      sendCmd('stop');
      break;
    case '[':
      T.jogVel = Math.max(0.05, T.jogVel - T.jogDelta);
      T.hint = `조그 속도 ${T.jogVel.toFixed(2)} rad/s`;
      render({});
      break;
    case ']':
      T.jogVel = Math.min(T.maxVel, T.jogVel + T.jogDelta);
      T.hint = `조그 속도 ${T.jogVel.toFixed(2)} rad/s`;
      render({});
      break;
    case 't': sendCmd('resume'); break;
    case 'h': sendCmd('home'); break;
    case 'r':
      // TUI 와 동일 — r = 선택 축, Shift+R = 에러 latch 된 서보 전체.
      sendCmd(ev.shiftKey ? 'reboot all' : `reboot ${T.joints[T.selected]}`);
      break;
    case 'c': sendCmd('calib_start'); break;
    case 'm': sendCmd('calib_mark'); break;
    case 'x': sendCmd('calib_cancel'); break;
    case 'p':
      // TUI 와 같은 순서 — 토크를 먼저 풀고, 손으로 자세를 잡은 뒤 슬롯을 누른다.
      T.slotMode = 'save';
      sendCmd('freedrive');
      break;
    case 'g':
      T.slotMode = 'goto';
      T.hint = '이동할 슬롯 번호(1-9) 를 누르세요 — Esc 취소';
      render({});
      break;
    default:
      break;
  }
}

function onKeyUp(ev) {
  if (heldKeys.delete(normKey(ev))) render({});
}

/* ── 게임패드 ──────────────────────────────────────────── */
function livePad() {
  if (T.padIndex === null) return null;
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  return pads[T.padIndex] || null;
}

function defaultPadConfig() {
  const source = (T.desc.gamepad && T.desc.gamepad.joints) || [];
  const joints = {};
  for (const entry of source) {
    joints[entry.name] = {
      axis: entry.axis, scale: entry.scale, inverted: entry.inverted,
      plus: entry.button_plus, minus: entry.button_minus,
    };
  }
  return {
    deadzone: (T.desc.gamepad && T.desc.gamepad.deadzone) || 0.15,
    deadman: (T.desc.gamepad && T.desc.gamepad.deadman_button) || 9,
    joints,
  };
}

function loadPadConfig() {
  const base = defaultPadConfig();
  try {
    const saved = JSON.parse(localStorage.getItem(LS_KEY) || 'null');
    if (saved && saved.joints) {
      base.deadzone = Number(saved.deadzone) || base.deadzone;
      base.deadman = Number.isInteger(saved.deadman) ? saved.deadman : base.deadman;
      for (const name of Object.keys(base.joints)) {
        if (saved.joints[name]) Object.assign(base.joints[name], saved.joints[name]);
      }
    }
  } catch (err) { /* 저장값이 깨졌으면 기본값으로 간다 */ }
  return base;
}

function savePadConfig() {
  try { localStorage.setItem(LS_KEY, JSON.stringify(T.pad)); } catch (err) { /* 무시 */ }
}

function onPadConnected(ev) {
  T.padIndex = ev.gamepad.index;
  // ⚠️ 표준 매핑이 아니면 축·버튼 인덱스가 이 패드만의 것이다.
  T.padWarn = ev.gamepad.mapping === 'standard' ? ''
    : `이 패드는 표준 매핑이 아닙니다(mapping="${ev.gamepad.mapping}") — `
      + '축·버튼 인덱스를 아래 실시간 값으로 직접 확인하세요.';
  T.hint = `게임패드 연결: ${ev.gamepad.id}`;
  render({});
}

function onPadDisconnected(ev) {
  if (T.padIndex === ev.gamepad.index) T.padIndex = null;
  stopNow('게임패드 연결이 끊겨 정지했습니다');
}

/* ── 렌더 ──────────────────────────────────────────────── */
function render(intent) {
  const C = window.Control;
  const held = C.held();
  const esc = C.escapeHtml;

  const claimBtn = el('teleop-claim');
  if (claimBtn) claimBtn.textContent = held ? '조종권 반납' : '조종권 획득';

  // 워치독 잔여 — 조그 중에는 이 값이 계속 리셋된다. 0 이 되면 서버가 0 을 쏜다.
  const box = el('teleop-live');
  if (box) {
    const s = C.session || {};
    const age = s.jog_intent_age;
    const timeout = s.intent_timeout_s || T.desc.intent_timeout_s || 0.3;
    const left = age === null || age === undefined ? null : Math.max(0, timeout - age);
    const rows = [
      ['상태', held
        ? '<span class="state-good">✔ 조종 중</span>'
        : (s.holder ? `<span class="state-warning">관전 중 — ${esc(s.holder.label)} 조종</span>`
          : '<span class="muted">조종권 없음</span>')],
      ['선택 축', `${esc(T.joints[T.selected] || '—')} <span class="muted">(숫자키 1-${T.joints.length})</span>`],
      ['조그 속도', `${T.jogVel.toFixed(2)} rad/s <span class="muted">(상한 ${T.maxVel} · [ / ] 조절)</span>`],
      ['워치독', left === null
        ? '<span class="muted">유휴 — 조그 의도 없음</span>'
        : (left > 0 ? `${left.toFixed(2)}초 남음` : '<span class="muted">0 발행됨(정지)</span>')],
      ['마지막 정지', s.stop_reason === 'watchdog'
        ? '<span class="state-warning">▲ 워치독(의도 끊김)</span>'
        : (s.stop_reason === 'released' ? '해제(정상)' : '<span class="muted">—</span>')],
    ];
    box.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  }

  // 축별 속도 막대 — 지금 브라우저가 무엇을 밀고 있는지가 화면과 일치해야 한다.
  const bars = el('teleop-bars');
  if (bars) {
    bars.innerHTML = T.joints.map((name, i) => {
      const v = intent[name] || 0;
      const pct = Math.min(100, Math.abs(v) / T.maxVel * 100);
      const cls = v > 0 ? 'jog-bar-plus' : 'jog-bar-minus';
      return `<div class="jog-row${i === T.selected ? ' jog-row-sel' : ''}">`
        + `<span class="jog-name">${esc(name)}</span>`
        + `<span class="jog-track"><span class="jog-fill ${cls}" style="width:${pct}%"></span></span>`
        + `<span class="jog-val num">${v.toFixed(2)}</span></div>`;
    }).join('');
  }

  const hint = el('teleop-hint');
  if (hint) {
    hint.textContent = T.slotMode
      ? (T.slotMode === 'save'
        ? '자세 저장 — 팔 토크가 꺼졌습니다. 손으로 자세를 잡고 슬롯(1-9)을 누르세요 (Esc = 저장 없이 토크 복귀)'
        : '이동할 슬롯 번호(1-9) 를 누르세요 (Esc 취소)')
      : T.hint;
    hint.className = T.slotMode ? 'banner banner-serious' : 'muted small';
  }

  renderPad();
}

function renderPad() {
  const info = el('pad-info');
  if (!info) return;
  const gp = livePad();
  if (!gp) {
    info.innerHTML = '<span class="muted">패드 미연결 — 패드를 꽂고 아무 버튼이나 '
      + '한 번 누르세요(브라우저는 입력이 있어야 패드를 노출합니다).</span>';
    const axesBox = el('pad-axes');
    if (axesBox) axesBox.innerHTML = '';
    return;
  }
  const dead = gp.buttons[T.pad.deadman] && gp.buttons[T.pad.deadman].pressed;
  info.innerHTML = `<strong>${window.Control.escapeHtml(gp.id)}</strong> · `
    + `데드맨 buttons[${T.pad.deadman}] `
    + (dead ? '<span class="state-good">✔ 눌림</span>' : '<span class="muted">놓음</span>')
    + (T.bindMode ? ' · <span class="state-warning">바인딩 대기 — 데드맨으로 쓸 버튼을 누르세요</span>' : '');

  const axesBox = el('pad-axes');
  if (axesBox) {
    const axes = Array.from(gp.axes).map((v, i) =>
      `<span class="pad-cell">a${i} <b>${v.toFixed(2)}</b></span>`).join('');
    const buttons = Array.from(gp.buttons).map((b, i) =>
      `<span class="pad-cell${b.pressed ? ' pad-cell-on' : ''}">b${i}</span>`).join('');
    axesBox.innerHTML = axes + '<br>' + buttons;
  }

  if (T.bindMode) {
    const pressed = Array.from(gp.buttons).findIndex((b) => b.pressed);
    if (pressed >= 0) {
      T.pad.deadman = pressed;
      T.bindMode = false;
      savePadConfig();
      T.hint = `데드맨을 buttons[${pressed}] 로 바인딩했습니다`;
    }
  }
}

function renderPadMapping() {
  const body = el('pad-map-rows');
  if (!body) return;
  const esc = window.Control.escapeHtml;
  body.innerHTML = T.joints.map((name) => {
    const m = T.pad.joints[name] || { axis: -1, scale: 0.5, inverted: false, plus: -1, minus: -1 };
    return `<tr><td>${esc(name)}</td>`
      + `<td><input class="btn num-in" type="number" step="1" min="-1" value="${m.axis}" data-pad="axis" data-joint="${esc(name)}"></td>`
      + `<td><input class="btn num-in" type="number" step="0.05" min="0" value="${m.scale}" data-pad="scale" data-joint="${esc(name)}"></td>`
      + `<td><input type="checkbox" ${m.inverted ? 'checked' : ''} data-pad="inverted" data-joint="${esc(name)}"></td>`
      + `<td><input class="btn num-in" type="number" step="1" min="-1" value="${m.plus}" data-pad="plus" data-joint="${esc(name)}"></td>`
      + `<td><input class="btn num-in" type="number" step="1" min="-1" value="${m.minus}" data-pad="minus" data-joint="${esc(name)}"></td>`
      + '</tr>';
  }).join('');

  body.querySelectorAll('[data-pad]').forEach((input) => {
    input.addEventListener('change', () => {
      const m = T.pad.joints[input.dataset.joint];
      if (!m) return;
      const field = input.dataset.pad;
      m[field] = field === 'inverted' ? input.checked : Number(input.value);
      savePadConfig();
    });
  });
}

/* 명령 버튼은 서버가 준 어휘로 만든다 — JS 에 목록을 박아두면 teleop_core 가
 * 명령을 바꿨을 때 어디를 고쳐야 하는지 알 수 없게 된다. */
function renderCommandButtons() {
  const box = el('teleop-cmds');
  if (!box || !T.desc.commands) return;
  const esc = window.Control.escapeHtml;
  const cmds = T.desc.commands;
  const parts = [];
  for (const cmd of cmds.no_arg || []) {
    parts.push(`<button class="btn" type="button" data-cmd="${esc(cmd)}">${esc(cmd)}</button>`);
  }
  for (const cmd of cmds.threshold_arg || []) {
    parts.push(`<button class="btn" type="button" data-cmd="${esc(cmd)} on">${esc(cmd)} on</button>`);
    parts.push(`<button class="btn" type="button" data-cmd="${esc(cmd)} off">${esc(cmd)} off</button>`);
  }
  for (const cmd of cmds.toggle_arg || []) {
    parts.push(`<button class="btn" type="button" data-cmd="${esc(cmd)} on">${esc(cmd)} on</button>`);
    parts.push(`<button class="btn" type="button" data-cmd="${esc(cmd)} off">${esc(cmd)} off</button>`);
  }
  box.innerHTML = parts.join('');
  box.querySelectorAll('[data-cmd]').forEach((btn) => {
    btn.addEventListener('click', () => sendCmd(btn.dataset.cmd));
  });
}

/* ── 자세 저장/불러오기 ────────────────────────────────
 *
 * 저장은 **2단계**다. teleop_core 의 "프리드라이브 저장" 계약상 `save` 전에
 * `freedrive` 를 먼저 보내야 한다 — 토크가 걸린 채로는 손으로 자세를 잡을 수
 * 없기 때문이다. 사용자가 취소하면 `freedrive_cancel` 로 저장 없이 토크만
 * 되돌린다. 이 순서를 프론트엔드가 지켜야 하고, 안 지키면 "저장은 되는데 늘
 * 같은 자세만 저장되는" 형태로 조용히 잘못된다.
 *
 * 이름 규칙은 teleop_vocab._NAME_RE 와 같은 것을 여기서도 본다 — 서버가 어차피
 * 400 으로 막지만, 누르기 전에 알려주는 편이 낫다. */
const POSE_NAME_RE = /^[A-Za-z0-9_-]{1,32}$/;
let posePending = null;   // freedrive 중인 저장 대기 이름

function renderPoses(snap) {
  const box = el('pose-list');
  if (!box) return;
  const esc = window.Control.escapeHtml;
  const names = (snap && snap.teleop && snap.teleop.poses) || [];
  if (!names.length) {
    box.innerHTML = '<span class="muted small">저장된 자세 없음</span>';
    return;
  }
  // 이름 화이트리스트를 통과 못 하는 자세는 **삭제 명령도 거부된다**(같은 검증기를
  // 탄다) — 검증이 생기기 전에 저장된 항목이 실제로 남아 있었다. 누르면 400 이
  // 뜨는 버튼을 그리느니, 못 지운다는 걸 화면에서 말한다(정리는 poses_file 직접 편집).
  box.innerHTML = names.map((n) => {
    const removable = POSE_NAME_RE.test(n);
    return `<span class="pose-btn">` +
      `<button class="btn" type="button" data-goto="${esc(n)}" ` +
      `title="이 자세로 이동">${esc(n)}</button>` +
      (removable
        ? `<button class="btn pose-del" type="button" data-del="${esc(n)}" ` +
          `title="자세 삭제">×</button>`
        : `<button class="btn pose-del" type="button" disabled ` +
          `title="이름에 쓸 수 없는 문자가 있어 GUI 로는 지울 수 없습니다 — ` +
          `poses_file 을 직접 편집하세요">×</button>`) +
      `</span>`;
  }).join('');
  box.querySelectorAll('[data-goto]').forEach((b) => {
    b.addEventListener('click', () => sendCmd(`goto ${b.dataset.goto}`));
  });
  box.querySelectorAll('[data-del]').forEach((b) => {
    b.addEventListener('click', () => {
      const n = b.dataset.del;
      // 삭제는 되돌릴 수 없고 파일까지 바뀐다 — 한 번 묻는다.
      if (window.confirm(`자세 '${n}' 을(를) 삭제할까요?`)) sendCmd(`delete ${n}`);
    });
  });
}

function showFreedrive(on) {
  const bar = el('pose-freedrive');
  if (bar) bar.hidden = !on;
  const btn = el('pose-save');
  if (btn) btn.disabled = on;
}

async function beginPoseSave() {
  const input = el('pose-name');
  const name = (input.value || '').trim();
  if (!POSE_NAME_RE.test(name)) {
    T.hint = '✖ 자세 이름은 영문/숫자/_/- 만, 1~32자';
    render({});
    return;
  }
  posePending = name;
  T.slotMode = null;   // 키보드 슬롯 저장과 동시에 걸리면 freedrive 가 두 번 나간다
  const label = el('pose-pending-name');
  if (label) label.textContent = name;
  // 토크를 먼저 푼다 — 이게 성공해야 손으로 자세를 잡을 수 있다.
  await sendCmd('freedrive');
  showFreedrive(true);
}

async function confirmPoseSave() {
  if (!posePending) return;
  await sendCmd(`save ${posePending}`);   // teleop_core 가 저장 후 토크를 되켠다
  posePending = null;
  showFreedrive(false);
  const input = el('pose-name');
  if (input) input.value = '';
}

async function cancelPoseSave() {
  posePending = null;
  showFreedrive(false);
  await sendCmd('freedrive_cancel');
}

function wirePoseControls() {
  const save = el('pose-save');
  if (save) save.addEventListener('click', beginPoseSave);
  const confirm = el('pose-confirm');
  if (confirm) confirm.addEventListener('click', confirmPoseSave);
  const cancel = el('pose-cancel');
  if (cancel) cancel.addEventListener('click', cancelPoseSave);
  const input = el('pose-name');
  if (input) {
    // 조그 핸들러(window) 는 isTyping() 으로 이미 입력 칸을 걸러내므로
    // stopPropagation 은 필요 없다 — Enter 로 저장만 시작한다.
    input.addEventListener('keydown', (ev) => {
      if (ev.key === 'Enter') { ev.preventDefault(); beginPoseSave(); }
    });
  }
  window.onFull(renderPoses);
}

/* ── 초기화 ────────────────────────────────────────────── */
function init() {
  const C = window.Control;
  T.desc = C.describe || {};
  T.joints = (T.desc.joint_names || []).slice();
  T.maxVel = T.desc.max_vel_rad_s || 1.0;
  const kb = T.desc.keyboard || {};
  T.jogVel = Math.min(kb.jog_velocity_rad_s || 0.7, T.maxVel);
  T.jogDelta = kb.jog_velocity_delta || 0.2;
  T.pad = loadPadConfig();

  renderCommandButtons();
  renderPadMapping();
  wirePoseControls();

  // 이미 다른 프론트엔드가 /arm/teleop_jog 를 밀고 있으면 획득 전에 알려준다 —
  // 눌러 보고 나서 409 를 받는 것보다 낫다.
  if (T.desc.jog_conflict) {
    T.hint = `⚠ ${T.desc.jog_conflict}`;
    el('teleop-force-row').hidden = false;
  }

  el('teleop-claim').addEventListener('click', async () => {
    if (C.held()) { await C.release(); stopNow('조종권을 반납했습니다'); return; }
    try {
      await C.claim(false);
      T.hint = '조종권 획득 — 키보드/패드로 조종할 수 있습니다';
    } catch (err) {
      T.hint = `✖ ${err.message}`;
      const forceRow = el('teleop-force-row');
      if (forceRow) forceRow.hidden = false;
    }
    render({});
  });

  el('teleop-force').addEventListener('click', async () => {
    try {
      await C.claim(true);
      T.hint = '⚠ 강제 획득 — 발행자 충돌을 무시했습니다(이벤트에 기록됨)';
    } catch (err) {
      T.hint = `✖ ${err.message}`;
    }
    render({});
  });

  el('teleop-estop').addEventListener('click', () => {
    stopNow('정지 버튼 — 토크 차단됨 (resume 으로 복귀)');
    sendCmd('stop');
  });

  el('pad-bind').addEventListener('click', () => {
    T.bindMode = true;
    T.hint = '데드맨으로 쓸 버튼을 지금 누르세요';
    render({});
  });

  el('pad-deadzone').value = T.pad.deadzone;
  el('pad-deadzone').addEventListener('change', (ev) => {
    const v = Number(ev.target.value);
    if (Number.isFinite(v) && v >= 0 && v < 0.9) { T.pad.deadzone = v; savePadConfig(); }
  });

  // 데드맨 — 들어오는 경로는 여러 개지만 처리는 하나다.
  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('keyup', onKeyUp);
  window.addEventListener('blur', () => stopNow('창 포커스를 잃어 정지했습니다'));
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) stopNow('탭이 숨겨져 정지했습니다');
  });
  window.addEventListener('pagehide', () => stopNow(''));
  window.addEventListener('gamepadconnected', onPadConnected);
  window.addEventListener('gamepaddisconnected', onPadDisconnected);
  C.onControlChange(() => { if (!C.held()) { heldKeys.clear(); } render({}); });

  // 이미 꽂혀 있는 패드(페이지를 새로고침한 경우)를 한 번 훑는다.
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (let i = 0; i < pads.length; i += 1) {
    if (pads[i]) { T.padIndex = i; break; }
  }

  // 서버 재발행 주기와 같은 속도로 의도를 올린다. 움직이지 않는 동안에는 이 루프가
  // 네트워크를 전혀 쓰지 않는다(게임패드 폴링만 한다).
  loopTimer = setInterval(pump, Math.max(20, 1000 / (T.desc.publish_hz || 20.0)));
  render({});
}

T.init = init;

})();
