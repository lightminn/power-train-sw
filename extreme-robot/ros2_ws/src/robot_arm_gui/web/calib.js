/* 관절 캘리브 마법사 4종 — 가동범위 · 영점 · 기어비 · 그리퍼 끝단.
 *
 * `control.js` 가 제어 모드에서만 `Calib.init()` 을 부른다.
 *
 * ## 순서를 화면이 강제한다
 *
 * **기어비 → 영점 → 가동범위.** 기어비가 틀리면 영점 역산이 같이 틀어지고, 둘 중
 * 하나라도 바뀌면 기존 가동범위는 전부 무효다. 그래서 앞 단계를 이번 세션에서 다시
 * 잰 뒤에는 뒤 단계에 "다시 재야 함" 표시가 붙는다.
 *
 * ## 도메인 함정을 화면에 적는다
 *
 * `JOINT_CONFIG`/`joint_limits` 는 **관절각 도메인**, `teleop_core` 의 DEFAULT_CENTERS/
 * DEFAULT_*_RADS 는 **서보축 도메인**이라 숫자가 무관하다. 이 구분을 뭉개면 한쪽 값을
 * 다른 쪽에 복사하는 사고가 난다 — 화면이 매번 명시한다.
 *
 * ## 계산은 서버가 한다
 *
 * 수식은 `dynamixel_control/calib_math.py` 한 곳에 있고 `scripts/measure_*.py` 도 같은
 * 함수를 쓴다. 브라우저가 다시 계산하면 언젠가 갈라지므로, 여기서는 **캡처한 값만**
 * 올리고 결과(복사 블록 포함)는 SSE 로 받아 그린다.
 */
'use strict';

/* 파일 스코프 IIFE — 이유는 teleop.js 상단 주석 참고(전역 렉시컬 스코프 공유). */
(() => {

const K = {
  desc: null,          // describe().calib
  joints: [],
  snap: null,          // 마지막 full 스냅샷
  captures: {},        // 기어비/그리퍼 캡처 버퍼
  redo: {},            // 이번 세션에서 다시 잰 단계 → 뒤 단계에 경고
  result: null,
};
window.Calib = K;

const el = (id) => document.getElementById(id);

/* ── 작업 전송 ─────────────────────────────────────────── */
async function runTask(kind, payload, outId) {
  const C = window.Control;
  const out = el(outId);
  if (!C.held()) {
    try { await C.claim(false); } catch (err) { out.textContent = `✖ ${err.message}`; return; }
  }
  out.textContent = '요청 중…';
  try {
    const data = await C.post('/api/task', { token: C.token, kind, payload });
    out.textContent = `요청 접수 (#${data.task_id})`;
    pollTask(data.task_id, out);
  } catch (err) {
    out.textContent = `✖ ${err.message}`;
  }
}

async function pollTask(taskId, out) {
  for (let i = 0; i < 40; i += 1) {
    await new Promise((r) => setTimeout(r, 300));
    let data;
    try { data = await (await fetch(`/api/tasks?since=${taskId - 1}`)).json(); }
    catch (err) { continue; }
    const entry = (data.results || []).find((e) => e.id === taskId);
    if (!entry) continue;
    if (entry.state === 'done') { out.textContent = `✔ ${entry.detail}`; return; }
    if (entry.state === 'error') { out.textContent = `✖ ${entry.detail}`; return; }
  }
  out.textContent = '⋯ 응답이 없습니다 (노드 로그를 확인하세요)';
}

/* ── 현재 관절값 캡처 ──────────────────────────────────── */
/* 화면이 이미 /joint_states 를 그리고 있으니 그 값을 그대로 쓴다. 서버는 캡처가
 * 1초 이내 값인지 다시 확인한다 — 낡은 값을 캡처하면 손으로 이미 움직인 뒤의 자세를
 * 그 전 값으로 기록하게 된다. */
function jointRad(name) {
  const rows = (K.snap && K.snap.joints) || {};
  const slot = rows[name];
  return slot && typeof slot.position === 'number' ? slot.position : null;
}

/* ── 1. 가동범위 (teleop_core 가 절차를 갖고 있다) ─────── */
function renderRange() {
  const box = el('calib-range-status');
  if (!box) return;
  const info = (K.snap && K.snap.teleop && K.snap.teleop.calib_info) || null;
  if (!info) {
    box.innerHTML = '<span class="muted">/arm/calib_status 수신 없음 — '
      + 'teleop_core 가 떠 있는지 확인하세요.</span>';
    return;
  }
  const pct = info.state === 'active' ? Math.round(info.progress * 100) : null;
  box.innerHTML = `<div>${window.Control.escapeHtml(summaryOf(info))}</div>`
    + (pct === null ? ''
      : `<span class="jog-track" style="display:block;margin-top:6px">
           <span class="jog-fill jog-bar-plus" style="width:${pct}%"></span></span>`)
    + (info.state === 'unknown'
      ? `<div class="muted small">원문: <code>${window.Control.escapeHtml(info.raw)}</code></div>`
      : '');
}

/* 요약 문구는 서버(calib_status_parse.summary)와 같은 규칙이지만, SSE 에는 파싱
 * 결과만 실려 오므로 화면에서 문장을 만든다. 문구가 갈라져도 판단 근거(state)는
 * 하나라 안전하다. */
function summaryOf(info) {
  switch (info.state) {
    case 'idle': return '대기 중 — [측정 시작] 을 누르면 팔 토크가 꺼집니다';
    case 'active':
      return `측정 중 — ${info.joint} ${info.step_label} (${info.index}/${info.total}축)`;
    case 'done':
      return `완료 — ${info.applied}축 적용`
        + (info.rejected ? `, ${info.rejected}축 거절(재측정 필요)` : '');
    case 'cancelled': return '취소됨 — 기록값 폐기, 토크 복귀';
    default: return '알 수 없는 상태';
  }
}

/* ── 2. 영점 ───────────────────────────────────────────── */
function referencePayload() {
  const out = {};
  document.querySelectorAll('[data-zero-ref]').forEach((input) => {
    const v = Number(input.value);
    if (Number.isFinite(v) && v !== 0) out[input.dataset.zeroRef] = v;
  });
  return { reference: out };
}

function renderZeroForm() {
  const box = el('calib-zero-refs');
  if (!box) return;
  const esc = window.Control.escapeHtml;
  box.innerHTML = K.joints.map((name) =>
    `<label class="field"><span class="field-label">${esc(name)} 기준각 [rad]</span>`
    + `<input class="btn num-in" type="number" step="0.0001" value="0"`
    + ` data-zero-ref="${esc(name)}"></label>`).join('');
}

/* ── 3. 기어비 ─────────────────────────────────────────── */
function renderGearJoints() {
  const select = el('calib-gear-joint');
  if (!select) return;
  const esc = window.Control.escapeHtml;
  select.innerHTML = K.joints.map((n) => `<option value="${esc(n)}">${esc(n)}</option>`).join('');
}

function captureGear(which) {
  const name = el('calib-gear-joint').value;
  const rad = jointRad(name);
  const out = el('calib-gear-result');
  if (rad === null) {
    out.textContent = `✖ ${name} 의 /joint_states 값이 없습니다`;
    return;
  }
  K.captures[which] = rad;
  K.captures.joint = name;
  out.textContent = `${which === 'start' ? '시작' : '끝'} 캡처: ${rad.toFixed(4)} rad`;
  renderGearCaptures();
}

function renderGearCaptures() {
  const box = el('calib-gear-captures');
  if (!box) return;
  const fmt = (v) => (typeof v === 'number' ? `${v.toFixed(4)} rad` : '—');
  box.innerHTML = `<dt>시작</dt><dd>${fmt(K.captures.start)}</dd>`
    + `<dt>끝</dt><dd>${fmt(K.captures.end)}</dd>`;
}

/* ── 4. 그리퍼 끝단 ────────────────────────────────────── */
/* 그리퍼 관절 이름은 JOINT_CONFIG(팔 축만) 에 없다 — teleop_core 가 알려준
 * joint_names 중 팔 축이 아닌 것을 쓴다(그쪽이 조그 계약의 권위다). */
function gripperJoint() {
  const all = (window.Control.describe.joint_names || []);
  return all.find((n) => !K.joints.includes(n)) || 'gripper_left_pinion_joint';
}

function captureGripper(which) {
  const rad = jointRad(gripperJoint());
  const out = el('calib-gripper-result');
  if (rad === null) {
    out.textContent = `✖ ${gripperJoint()} 의 /joint_states 값이 없습니다`;
    return;
  }
  K.captures[which] = rad;
  out.textContent = `${which === 'closed' ? '닫힘' : '열림'} 캡처: ${rad.toFixed(4)} rad`;
  renderGripperCaptures();
}

function renderGripperCaptures() {
  const box = el('calib-gripper-captures');
  if (!box) return;
  const fmt = (v) => (typeof v === 'number' ? `${v.toFixed(4)} rad` : '—');
  box.innerHTML = `<dt>완전 닫힘</dt><dd>${fmt(K.captures.closed)}</dd>`
    + `<dt>완전 열림</dt><dd>${fmt(K.captures.opened)}</dd>`;
}

/* ── 결과 ──────────────────────────────────────────────── */
const RESULT_TITLES = { zero: '영점', gear: '기어비', gripper: '그리퍼 끝단' };

function renderResult(snap) {
  const box = el('calib-result');
  if (!box) return;
  const r = snap.calib_result;
  K.result = r;
  if (!r) {
    box.innerHTML = '<span class="muted">아직 계산 결과가 없습니다.</span>';
    return;
  }
  const esc = window.Control.escapeHtml;
  const head = `<div class="field-label">${RESULT_TITLES[r.kind] || r.kind} 결과</div>`;
  const warn = (r.warnings || []).map((w) =>
    `<div class="banner banner-serious">⚠ ${esc(w)}</div>`).join('');
  const rows = (r.rows || []).map((row) =>
    `<tr>${Object.entries(row).map(([, v]) => `<td>${esc(v)}</td>`).join('')}</tr>`).join('');
  const headers = r.rows && r.rows.length
    ? `<tr>${Object.keys(r.rows[0]).map((k) => `<th scope="col">${esc(k)}</th>`).join('')}</tr>`
    : '';

  box.innerHTML = head + warn
    + `<div class="table-wrap"><table class="tbl"><thead>${headers}</thead>`
    + `<tbody>${rows}</tbody></table></div>`
    + '<div class="field-label" style="margin-top:8px">복사용 블록</div>'
    + `<pre class="calib-block">${esc(r.block || '')}</pre>`
    + '<div class="btn-row">'
    + '<button id="calib-copy" class="btn" type="button">클립보드로 복사</button>'
    + `<button id="calib-apply" class="btn btn-primary" type="button">즉시 적용 (${esc(r.apply_target)})</button>`
    + '</div><div id="calib-apply-result" class="muted small">—</div>'
    + '<p class="footnote">즉시 적용은 <strong>브릿지 파라미터만</strong> 바꿉니다 — '
    + '프로세스를 다시 띄우면 사라집니다. 값이 맞다고 확인되면 위 블록을 소스에 '
    + '반영하세요(근거 주석을 사람이 같이 적어야 하므로 자동으로 쓰지 않습니다).</p>';

  el('calib-copy').addEventListener('click', () => {
    navigator.clipboard.writeText(r.block || '').then(
      () => { el('calib-apply-result').textContent = '복사했습니다'; },
      () => { el('calib-apply-result').textContent = '복사 실패 — 블록을 직접 선택하세요'; });
  });
  el('calib-apply').addEventListener('click', () => {
    runTask('calib_apply', { target: r.apply_target, values: r.apply_values },
            'calib-apply-result');
    K.redo[r.kind] = true;
    renderOrderWarning();
  });
}

/* 의존 순서 — 앞 단계를 다시 잰 뒤에는 뒤 단계가 무효다. */
function renderOrderWarning() {
  const box = el('calib-order-warn');
  if (!box) return;
  const notes = [];
  if (K.redo.gear) notes.push('기어비를 바꿨습니다 → <strong>영점을 다시 재세요</strong>');
  if (K.redo.gear || K.redo.zero) {
    notes.push('기어비·영점이 바뀌면 <strong>가동범위(리밋)는 전부 무효</strong>입니다 '
      + '— 마지막에 다시 측정하세요');
  }
  box.hidden = notes.length === 0;
  box.innerHTML = notes.map((n) => `<div>⚠ ${n}</div>`).join('');
}

/* ── 초기화 ────────────────────────────────────────────── */
function init() {
  const C = window.Control;
  K.desc = C.describe.calib || { available: false };
  const panel = el('calib-panel');
  if (!K.desc.available) {
    if (panel) {
      el('calib-unavailable').hidden = false;
      el('calib-unavailable').textContent = `캘리브 마법사 사용 불가 — ${K.desc.reason || ''}`;
    }
    return;
  }
  K.joints = Object.keys(K.desc.joints || {});

  renderZeroForm();
  renderGearJoints();
  renderGearCaptures();
  renderGripperCaptures();

  el('calib-range-start').addEventListener('click', () => window.Teleop.sendCmd('calib_start'));
  el('calib-range-mark').addEventListener('click', () => window.Teleop.sendCmd('calib_mark'));
  el('calib-range-cancel').addEventListener('click', () => window.Teleop.sendCmd('calib_cancel'));

  el('calib-zero-run').addEventListener('click', () => {
    K.redo.zero = true;
    renderOrderWarning();
    runTask('calib_zero', referencePayload(), 'calib-zero-result');
  });

  el('calib-gear-start').addEventListener('click', () => captureGear('start'));
  el('calib-gear-end').addEventListener('click', () => captureGear('end'));
  el('calib-gear-run').addEventListener('click', () => {
    if (typeof K.captures.start !== 'number' || typeof K.captures.end !== 'number') {
      el('calib-gear-result').textContent = '✖ 시작/끝을 모두 캡처하세요';
      return;
    }
    K.redo.gear = true;
    renderOrderWarning();
    runTask('calib_gear', {
      joint: K.captures.joint || el('calib-gear-joint').value,
      start_rad: K.captures.start, end_rad: K.captures.end,
      joint_deg: Number(el('calib-gear-deg').value),
    }, 'calib-gear-result');
  });

  el('calib-gripper-closed').addEventListener('click', () => captureGripper('closed'));
  el('calib-gripper-open').addEventListener('click', () => captureGripper('opened'));
  el('calib-gripper-run').addEventListener('click', () => {
    if (typeof K.captures.closed !== 'number' || typeof K.captures.opened !== 'number') {
      el('calib-gripper-result').textContent = '✖ 닫힘/열림을 모두 캡처하세요';
      return;
    }
    runTask('calib_gripper', {
      closed_rad: K.captures.closed, opened_rad: K.captures.opened,
      margin: Number(el('calib-gripper-margin').value) || 0,
    }, 'calib-gripper-result');
  });

  window.onFull((snap) => {
    K.snap = snap;
    renderRange();
    renderResult(snap);
  });
  renderRange();
}

K.init = init;

})();
