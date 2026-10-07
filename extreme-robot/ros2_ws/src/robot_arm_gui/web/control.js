/* 제어 계층 프론트엔드 — 세션·모델 교체.
 *
 * 읽기 전용 모드(control:=false)에서는 `/api/control` 이 `enabled:false` 를 돌려주고,
 * 이 파일은 제어 UI 를 **하나도 그리지 않는다**. app.js 는 이 파일의 존재를 몰라도
 * 예전과 똑같이 동작한다(렌더 훅으로만 붙는다).
 *
 * ## 조종권
 *
 * 탭 두 개가 동시에 조그를 밀면 서로의 속도를 덮어써서 어느 쪽도 자기가 명령한 대로
 * 움직이지 않는다. 그래서 토큰 하나만 살아 있고, 브라우저가 TTL 의 1/3 주기로 갱신한다.
 * 탭을 닫거나 페이지를 숨기면 즉시 반납하므로, 잊고 자리를 떠도 다음 사람이 잡을 수 있다.
 */
'use strict';

/* 파일 스코프 IIFE — 이유는 teleop.js 상단 주석 참고(전역 렉시컬 스코프 공유). */
(() => {

const C = {
  enabled: false,
  token: null,
  describe: {},
  session: null,
  models: null,
  label: `browser-${Math.random().toString(36).slice(2, 7)}`,
};
window.Control = C;

const el = (id) => document.getElementById(id);

/* ── HTTP ──────────────────────────────────────────────── */
/* 커스텀 헤더가 CSRF 방어의 절반이다 — 크로스오리진에서는 preflight 없이 못 붙이고,
 * 서버는 OPTIONS 를 구현하지 않으므로 남의 페이지에서는 요청이 성립하지 않는다. */
async function post(path, body) {
  const res = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', 'X-Monitor-Control': '1' },
    body: JSON.stringify(body || {}),
  });
  let data = {};
  try { data = await res.json(); } catch (err) { /* 본문 없는 응답도 있다 */ }
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}
C.post = post;

async function get(path) {
  const res = await fetch(path);
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return res.json();
}

/* ── 조종권 ────────────────────────────────────────────── */
let renewTimer = null;

async function claim(force) {
  const data = await post('/api/control/claim', { label: C.label, force: !!force });
  C.token = data.token;
  C.session = data.session;
  startRenew();
  notifyControlChange();
  return data;
}

async function release() {
  if (!C.token) return;
  const token = C.token;
  C.token = null;
  stopRenew();
  notifyControlChange();
  try { await post('/api/control/release', { token }); } catch (err) { /* 이미 만료 */ }
}

function startRenew() {
  stopRenew();
  const ttl = (C.session && C.session.token_ttl_s) || 5.0;
  renewTimer = setInterval(async () => {
    if (!C.token) return;
    try {
      const data = await post('/api/control/renew', { token: C.token });
      C.session = data.session;
    } catch (err) {
      // 만료됐거나 남이 강제로 뺏어갔다 — 조용히 놓아준다.
      C.token = null;
      stopRenew();
      notifyControlChange();
    }
  }, Math.max(500, (ttl * 1000) / 3));
}

function stopRenew() {
  if (renewTimer !== null) clearInterval(renewTimer);
  renewTimer = null;
}

C.claim = claim;
C.release = release;
C.held = () => C.token !== null;

const controlListeners = [];
C.onControlChange = (fn) => controlListeners.push(fn);
function notifyControlChange() {
  for (const fn of controlListeners) {
    try { fn(); } catch (err) { console.error(err); }
  }
  renderSession();
}

/* 자리를 뜨면 반납한다 — 조종권이 남아 있으면 다음 사람이 잡지 못한다.
 * sendBeacon 은 커스텀 헤더를 못 붙여 서버가 거절하므로 keepalive fetch 를 쓴다. */
window.addEventListener('pagehide', () => {
  if (!C.token) return;
  fetch('/api/control/release', {
    method: 'POST', keepalive: true,
    headers: { 'Content-Type': 'application/json', 'X-Monitor-Control': '1' },
    body: JSON.stringify({ token: C.token }),
  }).catch(() => {});
});

/* ── 세션 표시 ─────────────────────────────────────────── */
function renderSession() {
  const chip = el('control-chip');
  if (!chip) return;
  if (!C.enabled) {
    chip.textContent = '읽기 전용';
    chip.className = 'chip chip-muted';
    return;
  }
  const holder = C.session && C.session.holder;
  if (C.token) {
    const left = holder ? holder.expires_in_s : null;
    chip.textContent = `조종 중${left === null ? '' : ` · ${left.toFixed(1)}초`}`;
    chip.className = 'chip chip-good';
  } else if (holder) {
    chip.textContent = `관전 중 · ${holder.label} 조종`;
    chip.className = 'chip chip-warn';
  } else {
    chip.textContent = '조종권 없음';
    chip.className = 'chip chip-muted';
  }
}

/* ── 모델 패널 ─────────────────────────────────────────── */
async function refreshModels() {
  try {
    C.models = await get('/api/models');
  } catch (err) {
    C.models = { models: [], reason: String(err) };
  }
  renderModelOptions();
}

function renderModelOptions() {
  const select = el('model-select');
  if (!select || !C.models) return;
  const previous = select.value;
  select.innerHTML = (C.models.models || []).map((m) => {
    const size = m.size === null ? '없음' : `${(m.size / 1e6).toFixed(1)}MB`;
    const tag = m.source === 'preset' ? '프리셋' : '스캔';
    return `<option value="${escapeAttr(m.key)}"${m.exists ? '' : ' disabled'}>`
      + `${escapeHtml(m.label)} — ${tag} · ${size}</option>`;
  }).join('');
  if (previous) select.value = previous;
  syncModelForm();

  const note = el('model-dir-note');
  if (note) {
    note.textContent = C.models.models_dir
      ? `스캔 위치: ${C.models.models_dir} — 여기에 .pt 를 떨구면 목록에 바로 뜹니다`
      : (C.models.reason || '');
  }
  const restart = el('model-restart');
  if (restart) {
    restart.disabled = !C.models.can_restart || !C.token;
    restart.title = C.models.restart_reason || 'perception_node 프로세스를 다시 띄웁니다';
  }
}

/* 선택한 항목의 preset 값을 입력칸에 채운다(사용자가 고친 값은 덮지 않는다). */
function syncModelForm() {
  const select = el('model-select');
  if (!select || !C.models) return;
  const entry = (C.models.models || []).find((m) => m.key === select.value);
  if (!entry) return;
  const task = el('model-task');
  const classes = el('model-classes');
  const pick = el('model-pick');
  if (task) task.value = entry.task;
  if (classes) classes.value = entry.classes;
  if (pick) pick.value = entry.pick_classes;
  const hint = el('model-source-hint');
  if (hint) {
    hint.textContent = entry.source === 'preset'
      ? 'model_presets.py 에 등록된 프리셋 — 클래스가 이미 채워져 있습니다'
      : '스캔으로 찾은 파일 — task 와 클래스 이름은 직접 채워야 합니다'
        + ' (비우면 필터 없이 전체 통과)';
  }
}

function modelPayload() {
  return {
    key: el('model-select').value,
    task: el('model-task').value,
    classes: el('model-classes').value,
    pick_classes: el('model-pick').value,
  };
}

async function submitModel(kind) {
  const out = el('model-result');
  if (!C.token) {
    try { await claim(false); } catch (err) { out.textContent = `✖ ${err.message}`; return; }
  }
  out.textContent = '요청 중…';
  try {
    const data = await post('/api/task', {
      token: C.token, kind, payload: modelPayload(),
    });
    out.textContent = `요청 접수 (#${data.task_id}) — 결과 대기 중`;
    pollTask(data.task_id, out);
  } catch (err) {
    out.textContent = `✖ ${err.message}`;
  }
}

async function pollTask(taskId, out) {
  for (let i = 0; i < 60; i += 1) {
    await new Promise((r) => setTimeout(r, 500));
    let data;
    try { data = await get(`/api/tasks?since=${taskId - 1}`); } catch (err) { continue; }
    const entry = (data.results || []).find((e) => e.id === taskId);
    if (!entry) continue;
    if (entry.state === 'done') { out.textContent = `✔ ${entry.detail}`; return; }
    if (entry.state === 'error') { out.textContent = `✖ ${entry.detail}`; return; }
  }
  out.textContent = '⋯ 응답이 없습니다 (노드 로그를 확인하세요)';
}

/* 실제 로드 결과는 서비스 응답이 아니라 /perception/model_status 가 말해준다 —
 * 서비스 성공은 "요청을 받아들였다"는 뜻일 뿐이고 로드는 추론 스레드가 나중에 한다. */
function renderModelStatus(snap) {
  const box = el('model-status');
  if (!box) return;
  const m = snap.model;
  if (!m) {
    box.innerHTML = '<span class="muted">/perception/model_status 수신 없음 — '
      + 'perception_node 가 떠 있는지, 이 버전이 맞는지 확인하세요.</span>';
    return;
  }
  const rows = [
    ['상태', m.state === 'loaded' ? '✔ 로드됨'
      : m.state === 'loading' ? '⋯ 로드 중' : `■ ${m.state}`],
    ['모델', m.name || '—'],
    ['task', m.task || '—'],
    ['경로', m.path || '—'],
    ['교체 소요', m.seconds === null || m.seconds === undefined
      ? '—' : `${m.seconds.toFixed(2)}초`],
  ];
  if (m.detail) rows.push(['상세', m.detail]);
  box.innerHTML = rows.map(([k, v]) =>
    `<dt>${k}</dt><dd>${escapeHtml(String(v))}</dd>`).join('');

  const warn = el('model-stale-warn');
  if (warn) {
    // /pick_target 은 transient_local 이라 지울 방법이 없다. 새 모델이 아직
    // 아무것도 못 찾았어도 arm_fsm 은 이전 모델의 타깃을 계속 본다.
    warn.hidden = !(m.stale_pick_target && m.state === 'loaded');
  }
}

/* ── 유틸 ──────────────────────────────────────────────── */
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => (
    { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}
function escapeAttr(s) { return escapeHtml(s); }
C.escapeHtml = escapeHtml;

/* ── 초기화 ────────────────────────────────────────────── */
async function init() {
  let info;
  try {
    info = await get('/api/control');
  } catch (err) {
    // 조용히 돌아가면 '읽기 전용으로 뜬 것'과 구별이 안 된다 — 실제로 이 둘을
    // 헷갈려 한참을 잃었다(서버가 죽은 뒤 캐시된 페이지를 보고 있었다).
    const b = el('control-banner');
    if (b) {
      b.hidden = false;
      b.textContent = `■ 제어 계층을 확인하지 못했습니다 (${err.message}) — `
        + '모니터 노드가 떠 있는지 확인하세요. 화면의 모든 값이 낡았을 수 있습니다.';
    }
    return;
  }
  C.enabled = !!info.enabled;
  if (!C.enabled) { renderSession(); return; }
  C.describe = info;
  C.session = info.session;

  document.querySelectorAll('[data-control-only]').forEach((node) => {
    node.hidden = false;
  });
  const badge = document.querySelector('.badge-ro');
  if (badge) {
    badge.textContent = '제어 모드';
    badge.title = '/arm/teleop_jog·/arm/teleop_cmd 를 발행합니다. '
      + '계약 토픽과 /dynamixel/goal_position 은 발행하지 않습니다.';
  }
  document.title = '로봇팔 관제 — 제어 모드';

  el('model-select').addEventListener('change', syncModelForm);
  el('model-refresh').addEventListener('click', refreshModels);
  el('model-swap').addEventListener('click', () => submitModel('model_swap'));
  el('model-restart').addEventListener('click', () => submitModel('model_restart'));

  await refreshModels();
  window.onFull((snap) => {
    if (snap.control) C.session = snap.control;
    renderSession();
    renderModelStatus(snap);
  });
  window.onHot((snap) => {
    if (snap.control) { C.session = snap.control; renderSession(); }
  });
  if (window.Teleop) window.Teleop.init();
  if (window.Calib) window.Calib.init();
  renderSession();
}

init();

})();
