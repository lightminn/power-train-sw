/* 관제 GUI 프론트엔드 — 읽기 전용.
 *
 * 상시 연결은 SSE 1개 + MJPEG 1개로 고정한다. 브라우저는 오리진당 동시 연결이
 * 6개뿐이라, 탭을 여러 개 열면 금방 고갈된다(그래서 /api/health 가 접속 수를
 * 노출한다). 나머지 요청은 전부 단발 fetch 다.
 *
 * 오버레이(검출 박스)는 **브라우저에서** 그린다 — Jetson 이 다시 그릴 이유가 없다.
 * 다만 MJPEG 프레임과 검출 시각이 정확히 일치하지 않으므로 화면에 그 사실을 적는다.
 */
'use strict';

const $ = (id) => document.getElementById(id);

/* ── 테마 ──────────────────────────────────────────────── */
const savedTheme = localStorage.getItem('theme');
if (savedTheme) document.documentElement.dataset.theme = savedTheme;
$('theme-toggle').addEventListener('click', () => {
  const next = document.documentElement.dataset.theme === 'light' ? 'dark' : 'light';
  document.documentElement.dataset.theme = next;
  localStorage.setItem('theme', next);
});

/* ── 서식 헬퍼 ─────────────────────────────────────────── */
const fmtAge = (s) => (s === null || s === undefined) ? '—' : `${s.toFixed(2)}초 전`;
const fmtNum = (v, d = 0) => (v === null || v === undefined) ? '—' : v.toFixed(d);
const clockOf = (epoch) => new Date(epoch * 1000).toLocaleTimeString('ko-KR', { hour12: false });

function severityFor(ratio, warnAt) {
  if (ratio === null || ratio === undefined) return null;
  if (ratio >= 1.0) return 'critical';
  if (ratio >= 0.9) return 'serious';
  if (ratio >= (warnAt || 0.7)) return 'warning';
  return null;
}

const SEV_ICON = { good: '✔', warning: '▲', serious: '▲', critical: '■' };
function sevLabel(sev, text) {
  return sev ? `${SEV_ICON[sev]} ${text}` : text;
}

/* ── 상태 ──────────────────────────────────────────────── */
let contract = {};
let staleAfter = {};
let lastFull = null;

/* ── 렌더 훅 ───────────────────────────────────────────── */
/* 제어 계층(control.js)은 별도 파일이라 여기서 직접 부르지 않는다. 읽기 전용
 * 모드에서는 control.js 가 스스로 아무것도 그리지 않으므로, 훅이 비어 있어도
 * 이 파일의 동작은 예전과 완전히 같다. */
const hotHooks = [];
const fullHooks = [];
window.onHot = (fn) => hotHooks.push(fn);
window.onFull = (fn) => fullHooks.push(fn);
function runHooks(hooks, snap) {
  for (const fn of hooks) {
    try { fn(snap); } catch (err) { console.error('render hook 실패', err); }
  }
}

/* ── 미터 ──────────────────────────────────────────────── */
/* 단일 값 대 한계 → 가로 미터. 트랙은 같은 램프의 어두운/밝은 단계이고,
 * 채움이 심각도를 운반하며, 임계값은 트랙 위 hairline tick 이다. */
function meter(ratio, label, sev, tickAt) {
  if (ratio === null || ratio === undefined) {
    return `<div class="meter"><div class="meter-empty"></div>
      <div class="meter-label">${label}</div></div>`;
  }
  const pct = Math.max(0, Math.min(1, ratio)) * 100;
  const cls = sev ? ` ${sev}` : '';
  const tick = tickAt === undefined ? '' :
    `<span class="meter-tick" style="left:${Math.min(100, tickAt * 100)}%"></span>`;
  return `<div class="meter">
      <div class="meter-track" role="img" aria-label="${label}">
        <span class="meter-fill${cls}" style="width:${pct.toFixed(1)}%"></span>${tick}
      </div>
      <div class="meter-label">${sevLabel(sev, label)}</div>
    </div>`;
}

/* ── 스파크라인 ────────────────────────────────────────── */
/* 버킷은 [min, max, last]. 스파이크를 지우면 안 되는 데이터라 min~max 밴드를
 * 그대로 그리고 그 위에 last 선을 얹는다. 모터끼리 비교되도록 y 축은 공유한다. */
function sparkline(buckets, limit) {
  const W = 150, H = 30, N = 120;
  if (!buckets || !buckets.length) return `<svg class="spark" width="${W}" height="${H}"></svg>`;
  let peak = limit || 1;
  for (const b of buckets) if (b) peak = Math.max(peak, b[1]);
  const y = (v) => H - 1 - (v / peak) * (H - 2);
  const x = (i) => (i / (N - 1)) * W;
  const start = Math.max(0, buckets.length - N);
  const view = buckets.slice(start);

  const top = [], bottom = [], line = [];
  view.forEach((b, i) => {
    if (!b) return;
    const px = x(i + (N - view.length));
    top.push(`${px.toFixed(1)},${y(b[1]).toFixed(1)}`);
    bottom.unshift(`${px.toFixed(1)},${y(b[0]).toFixed(1)}`);
    line.push(`${px.toFixed(1)},${y(b[2]).toFixed(1)}`);
  });
  const band = top.length > 1
    ? `<polygon points="${top.concat(bottom).join(' ')}" fill="var(--series-band)"/>` : '';
  const path = line.length > 1
    ? `<polyline points="${line.join(' ')}" fill="none" stroke="var(--series-1)"
        stroke-width="2" stroke-linejoin="round"/>` : '';
  const rule = limit
    ? `<line x1="0" x2="${W}" y1="${y(limit).toFixed(1)}" y2="${y(limit).toFixed(1)}"
        stroke="var(--critical)" stroke-width="1" stroke-dasharray="3 3"/>` : '';
  return `<svg class="spark" width="${W}" height="${H}" role="img"
      aria-label="최근 60초 전류, 점선은 트립 임계">${band}${rule}${path}</svg>`;
}

/* ── 모터 표 ───────────────────────────────────────────── */
function renderMotors(snap) {
  const trip = snap.thresholds.trip;
  const spike = snap.thresholds.spike;
  const warnRatio = contract.warn_current_ratio || 0.7;
  const warnTemp = contract.warn_temp_c || 60;

  const src = (t) => t.source === 'runtime'
    ? `${t.value} (런타임 변경 ${t.at ? clockOf(t.at) : ''})` : `${t.value} (기동값)`;
  $('motor-thresholds').textContent =
    `트립 ${src(trip)}${trip.enabled ? '' : ' · 꺼짐'} / ` +
    `급변 ${src(spike)}${spike.enabled ? '' : ' · 꺼짐'}`;

  if (!snap.motors.length) return;
  const rows = snap.motors.map((m) => {
    const curSev = severityFor(m.trip_ratio, warnRatio);
    const spkSev = severityFor(m.spike_ratio, warnRatio);
    let tempSev = null;
    if (m.temp !== null) {
      if (m.temp >= 80) tempSev = 'critical';
      else if (m.temp >= 70) tempSev = 'serious';
      else if (m.temp >= warnTemp) tempSev = 'warning';
    }
    const worst = [curSev, spkSev, tempSev, m.hw_error ? 'critical' : null]
      .filter(Boolean);
    const rowCls = worst.includes('critical') || worst.includes('serious')
      ? 'attn' : (worst.length ? '' : 'recede');

    const curLabel = m.current === null ? '—'
      : `${m.current} (${fmtNum(m.current_ma, 0)} mA)` +
        (m.trip_headroom !== null ? ` · 여유 ${m.trip_headroom}` : '');
    const spkLabel = m.spike_delta === null
      ? 'baseline 수집 중' : `Δ${m.spike_delta} / ${spike.value}`;
    const tempLabel = m.temp === null
      ? '— 미수신' : `${m.temp} °C`;
    const hw = m.hw_error
      ? `<div class="meter-label state-critical">■ ${m.hw_error.labels.join(' | ')}</div>` : '';

    return `<tr class="${rowCls}">
      <td class="keep">${m.id}</td>
      <td class="keep">${m.joint || '—'}${hw}</td>
      <td class="num">${m.tick === null ? '—' : m.tick}</td>
      <td class="num">${m.goal_error === null ? '—' : m.goal_error}</td>
      <td class="num">${m.velocity === null ? '—' : m.velocity}</td>
      <td class="keep">${meter(m.trip_ratio, curLabel, curSev, 1.0)}</td>
      <td class="keep">${meter(m.spike_ratio, spkLabel, spkSev, 1.0)}</td>
      <td class="keep">${m.temp === null
        ? meter(null, tempLabel)
        : meter(m.temp / 80, tempLabel, tempSev, warnTemp / 80)}</td>
      <td>${sparkline(m.spark, trip.enabled ? trip.value : null)}</td>
    </tr>`;
  });
  $('motor-rows').innerHTML = rows.join('');
}

/* ── 상태 스트립 ───────────────────────────────────────── */
function renderStrip(snap) {
  const arm = snap.arm;
  const timeout = contract.heartbeat_timeout_s || 0.5;
  const armStale = arm.age === null || arm.age > timeout;
  const el = $('arm-status');
  el.textContent = arm.status || '—';
  el.className = armStale ? 'state-critical' : '';
  $('arm-mission').textContent = arm.mission_id === null || arm.mission_id === undefined
    ? '' : `mission ${arm.mission_id}`;
  $('arm-age').innerHTML = armStale
    ? `<span class="state-critical">■ STALE (${fmtAge(arm.age)}) — 파워트레인이 차를 세웁니다</span>`
    : `${fmtAge(arm.age)}${arm.stamp_age !== null && arm.stamp_age !== undefined
      ? ` · stamp ${arm.stamp_age.toFixed(2)}초` : ''}`;

  const mode = snap.chassis.mode;
  $('chassis-mode').textContent = mode || '—';
  const locked = mode && (contract.lock_modes || []).includes(mode);
  const permitted = mode === (contract.mission_stop || 'MISSION_STOP');
  $('chassis-gate').innerHTML = mode === null || mode === undefined
    ? '<span class="muted">수신 없음</span>'
    : permitted
      ? '<span class="state-good">✔ 작업 허가 (MISSION_STOP)</span>'
      : locked
        ? '<span class="state-warning">▲ 잠금 — 팔 작업 불가</span>'
        : '<span class="muted">허가 아님 (default-deny)</span>';

  const driveReady = (contract.drive_ready || []).includes(arm.status);
  $('drive-ready').innerHTML = driveReady
    ? '<span class="state-good">✔ 차 주행 가능 상태</span>'
    : `<span class="state-warning">▲ 차 주행 불가 — ${(contract.drive_ready || []).join(' / ')} 필요</span>`;

  const fault = snap.controller_fault;
  $('ctrl-fault').innerHTML = fault.value === null || fault.value === undefined
    ? '<span class="muted">controller_fault 수신 없음</span>'
    : fault.value
      ? '<span class="state-serious">▲ controller_fault — 어느 관절인지는 이 토픽에 없음</span>'
      : '<span class="state-good">✔ controller_fault 없음</span>';

  renderTopicDots(snap.topics);
}

function renderTopicDots(topics) {
  const names = Object.keys(topics).sort();
  $('topic-dots').innerHTML = names.map((name) => {
    const limit = staleAfter[name] !== undefined ? staleAfter[name] : (staleAfter._default || 5);
    const age = topics[name].age;
    const stale = age === null || age > limit;
    const short = name.replace(/^\/(dynamixel|perception|arm)\//, '');
    return `<span class="dot ${stale ? 'stale' : 'fresh'}"
      title="${name} — ${fmtAge(age)} (기준 ${limit}초)">${short}</span>`;
  }).join('');
}

/* ── 관절 ──────────────────────────────────────────────── */
function renderJoints(snap) {
  const d = snap.joint_domain || {};
  $('joint-domain').textContent = d.label || '';
  $('js-domain').textContent = d.label || '—';
  $('js-domain').className = d.severity ? `state-${d.severity}` : '';
  $('js-domain-note').textContent = d.note || '';

  const banner = $('js-banner');
  if (d.domain === 'conflict') {
    banner.hidden = false;
    banner.textContent = `■ ${d.label} — ${d.note}`;
  } else {
    banner.hidden = true;
  }

  const names = Object.keys(snap.joints).sort();
  $('joint-rows').innerHTML = names.length
    ? names.map((n) => {
      const j = snap.joints[n];
      return `<tr><td>${n}</td>
        <td class="num">${fmtNum(j.position, 4)}</td>
        <td class="num">${fmtNum(j.velocity, 3)}</td>
        <td class="num">${fmtNum(j.effort, 0)}</td></tr>`;
    }).join('')
    : '<tr><td colspan="4" class="muted">수신 대기…</td></tr>';

  const tl = snap.tick_limits;
  $('limit-note').innerHTML = !tl.received
    ? '<span class="muted">/dynamixel/tick_limits 미수신 — 소프트 리밋 상태를 알 수 없습니다.</span>'
    : tl.empty
      ? '<span class="state-serious">▲ 소프트 리밋 OFF — 목표값 clamp 방어선이 꺼져 있습니다.</span>'
      : `<span class="state-good">✔ 소프트 리밋 ${tl.count}축 적용</span>`;
}

/* ── 비전 ──────────────────────────────────────────────── */
const video = $('video');
const overlay = $('video-overlay');

function setVideoSource(source) {
  // 노드 쪽 기본값은 'none' 이다 — launch 가 파라미터를 YAML 로 넘기는데 YAML 1.1 이
  // 'off' 를 불리언으로 바꿔 버려서, 파라미터 어휘만 다르게 뒀다.
  if (source === 'none') source = 'off';
  localStorage.setItem('video_source', source);
  if (source === 'off') {
    video.hidden = true;
    overlay.hidden = true;
    $('video-off').hidden = false;
    video.removeAttribute('src');   // 연결을 끊어야 노드가 구독을 파괴한다
    return;
  }
  $('video-off').hidden = true;
  video.hidden = false;
  overlay.hidden = false;
  video.src = `/video/${source}?t=${Date.now()}`;
}

$('video-source').addEventListener('change', (e) => setVideoSource(e.target.value));
video.addEventListener('error', () => {
  // 구독이 살아나기까지 잠깐 빈 응답일 수 있다 — 조용히 재시도한다.
  const source = $('video-source').value;
  if (source !== 'off') setTimeout(() => setVideoSource(source), 2000);
});

function drawOverlay(snap) {
  if (overlay.hidden || !video.naturalWidth) return;
  // ⚠️ 이 캔버스가 그리는 박스는 **전방 캠**(/detected_objects) 것이다. 손목 캠 영상 위에
  // 겹쳐 그리면 다른 카메라의 좌표를 남의 그림에 얹는 셈이라 조용히 거짓말이 된다
  // (GUI 는 /wrist/detected_objects 를 구독하지 않는다). 손목 소스면 그리지 않는다 —
  // wrist_debug 는 노드가 이미 마스크·ROI 를 그려서 보내므로 아쉬울 것도 없다.
  if ($('video-source').value.startsWith('wrist')) {
    overlay.getContext('2d').clearRect(0, 0, overlay.width, overlay.height);
    return;
  }
  const w = video.clientWidth, h = video.clientHeight;
  if (overlay.width !== w || overlay.height !== h) { overlay.width = w; overlay.height = h; }
  const ctx = overlay.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  const sx = w / video.naturalWidth, sy = h / video.naturalHeight;
  const pick = snap.pick_target;
  const style = getComputedStyle(document.documentElement);
  const good = style.getPropertyValue('--good').trim();
  const series = style.getPropertyValue('--series-1').trim();

  (snap.detections.objects || []).forEach((o) => {
    const isPick = pick && pick.bbox && o.bbox[0] === pick.bbox[0] && o.bbox[1] === pick.bbox[1];
    ctx.strokeStyle = isPick ? good : series;
    ctx.lineWidth = 2;
    ctx.strokeRect(o.bbox[0] * sx, o.bbox[1] * sy, o.bbox[2] * sx, o.bbox[3] * sy);
  });
}

function renderVision(snap) {
  const det = snap.detections;
  $('detect-meta').textContent =
    `검출 ${det.hz === null ? '—' : det.hz + ' Hz'} · ${fmtAge(det.age)}` +
    ` · 영상 ${snap.video.fps || '—'} fps` +
    ' · 박스는 최신 검출 기준(영상 프레임과 ±1프레임 어긋남)';

  const pick = snap.pick_target;
  const items = [];
  if (pick) {
    items.push(`<li><span class="pick">★ PICK</span> ${pick.class_name}
      <span class="pos">${pick.confidence.toFixed(2)}</span>
      <span class="pos">(${pick.position.join(', ')})</span>
      ${pick.has_depth ? '' : '<span class="tag tag-nodepth">깊이 없음</span>'}
      <span class="tag">latched ${fmtAge(pick.age)}</span></li>`);
  }
  (det.objects || []).forEach((o) => {
    items.push(`<li>${o.class_name}
      <span class="pos">${o.confidence.toFixed(2)}</span>
      <span class="pos">(${o.position.join(', ')})</span>
      ${o.has_depth ? '' : '<span class="tag tag-nodepth">깊이 없음</span>'}</li>`);
  });
  $('detections').innerHTML = items.join('') ||
    '<li class="muted">검출 없음</li>';
  drawOverlay(snap);
}

/* ── 텔레옵 ────────────────────────────────────────────── */
function renderTeleop(snap) {
  const t = snap.teleop, joy = snap.joy;
  const active = t.jog_age !== null && t.jog_age < 0.5;
  const moving = (t.jog_velocities || []).some((v) => Math.abs(v) > 1e-3);

  const rows = [
    ['프론트엔드', (t.jog_publishers || []).join(', ') || '<span class="muted">없음</span>'],
    ['jog', active
      ? `<span class="state-good">✔ 활성</span> ${moving ? '· 이동 중' : '· 정지 명령'}
         (${fmtAge(t.jog_age)})`
      : `<span class="muted">유휴 (${fmtAge(t.jog_age)})</span>`],
    ['/joy', joy.age === null
      ? '<span class="muted">수신 없음</span>'
      : (joy.age > 0.5
        ? `<span class="state-warning">▲ 끊김 ${fmtAge(joy.age)}</span>`
        : `<span class="state-good">✔ ${fmtAge(joy.age)}</span>`)],
  ];

  const db = joy.deadman_button;
  const suffix = joy.params_resolved ? '' : ' <span class="muted">(기본값 가정 — 노드 미확인)</span>';
  rows.push(['데드맨', db === null || db < 0
    ? '<span class="muted">미배선</span>'
    : (joy.deadman_held === null
      ? `<span class="muted">buttons[${db}] — /joy 미수신</span>${suffix}`
      : (joy.deadman_held
        ? `<span class="state-good">✔ buttons[${db}] 눌림</span>${suffix}`
        : `<span class="muted">buttons[${db}] 놓음</span>${suffix}`))]);

  rows.push(['터보', joy.turbo_button === null || joy.turbo_button < 0
    ? '<span class="muted">미배선</span>'
    : `buttons[${joy.turbo_button}]`]);

  rows.push(['최근 명령', t.last_cmd
    ? `<span class="inferred">${t.last_cmd} (${fmtAge(t.last_cmd_age)}) — 추론</span>`
    : '<span class="muted">없음</span>']);
  rows.push(['저장 자세', (t.poses || []).join(', ') || '<span class="muted">없음</span>']);
  rows.push(['캘리브', t.calib || '<span class="muted">idle</span>']);

  $('teleop-kv').innerHTML = rows.map(([k, v]) =>
    `<dt>${k}</dt><dd${v.includes('inferred') ? ' class="inferred"' : ''}>${v}</dd>`).join('');

  const unobs = [
    joy.estop_button === null || joy.estop_button < 0
      ? 'E-stop 래치 — <code>estop_button</code> 이 -1(미배선)이라 래치 자체가 도달 불가'
      : 'E-stop 래치 — joystick_teleop 내부 변수라 토픽으로 나오지 않음',
    'teleop_core 의 stop 상태 — 노드에 상태 변수조차 없어 명령 로그로 <em>추론</em>만 가능',
    '입력 전압 — 어느 노드도 Present Input Voltage(주소 144)를 읽지 않음',
    '트립 당시 수치 상세 — 노드 로그 전용. 대신 아래 <strong>트립 블랙박스</strong>가 직전 3초를 보존',
    'moveit 브릿지 경로의 관절별 HW 에러 — <code>/dynamixel/controller_fault</code> Bool 하나로 축약',
    'arm_fsm 의 State 17종 — <code>/arm_status</code> 는 status 10종만 내보냄',
  ];
  $('unobservable-list').innerHTML = unobs.map((s) => `<li>${s}</li>`).join('');
}

/* ── 이벤트 ────────────────────────────────────────────── */
function renderEvents(snap) {
  const items = snap.events.slice().reverse().map((e) => {
    const cls = e.severity && e.severity !== 'info' ? ` state-${e.severity}` : '';
    const icon = SEV_ICON[e.severity] ? `${SEV_ICON[e.severity]} ` : '';
    const trace = e.trace
      ? ` <a class="btn" href="/api/trace/${e.trace}.jsonl">트레이스 받기</a>` : '';
    return `<li><time>${clockOf(e.wall)}</time>
      <span class="kind">${e.kind}</span>
      <span class="${cls}">${icon}${e.text}</span>${trace}</li>`;
  });
  $('events').innerHTML = items.join('') || '<li class="muted">이벤트 없음</li>';

  const blob = new Blob([snap.events.map((e) => JSON.stringify(e)).join('\n')],
    { type: 'application/x-ndjson' });
  const link = $('dl-events');
  if (link.dataset.url) URL.revokeObjectURL(link.dataset.url);
  link.href = link.dataset.url = URL.createObjectURL(blob);
}

/* ── Jetson ────────────────────────────────────────────── */
function renderSystem(snap) {
  const s = snap.system || {};
  const rows = [];
  rows.push(['CPU', s.cpu_percent === null || s.cpu_percent === undefined
    ? '<span class="muted">—</span>' : `${s.cpu_percent} %`]);
  if (s.memory) {
    rows.push(['메모리', `${s.memory.used_mb} / ${s.memory.total_mb} MB (${s.memory.percent} %)`]);
  }
  (s.thermal || []).forEach((z) => {
    const sev = z.celsius >= 85 ? 'critical' : z.celsius >= 75 ? 'serious'
      : z.celsius >= 65 ? 'warning' : null;
    rows.push([z.name, sev
      ? `<span class="state-${sev}">${SEV_ICON[sev]} ${z.celsius} °C</span>`
      : `${z.celsius} °C`]);
  });
  $('system-kv').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
}

/* ── HW 에러 배너 ──────────────────────────────────────── */
function renderHwBanner(snap) {
  const banner = $('hw-banner');
  if (!snap.hw_errors || !snap.hw_errors.length) { banner.hidden = true; return; }
  banner.hidden = false;
  const chips = snap.hw_errors.map((e) =>
    `<span>■ ${e.joint || '?'}(ID${e.dxl_id}) · ${e.labels.join(' | ')}</span>`).join(' ');
  banner.innerHTML = `${chips}
    <span class="banner-sub">reboot 전까지 latch 됩니다 (이 모니터는 읽기 전용이라 해제 버튼이 없습니다)</span>`;
}

/* ── 렌더 진입점 ───────────────────────────────────────── */
function applyHot(snap) {
  renderStrip({ ...(lastFull || {}), ...snap });
  renderMotors(snap);
  renderHwBanner(snap);
  runHooks(hotHooks, snap);
}

function applyFull(snap) {
  const first = lastFull === null;
  lastFull = snap;
  contract = snap.contract || contract;
  staleAfter = contract.stale_after || staleAfter;
  if (first && !localStorage.getItem('video_source') && contract.video_default_source) {
    $('video-source').value = contract.video_default_source;
    setVideoSource(contract.video_default_source);
  }
  renderStrip(snap);
  renderMotors(snap);
  renderHwBanner(snap);
  renderJoints(snap);
  renderVision(snap);
  renderTeleop(snap);
  renderEvents(snap);
  renderSystem(snap);
  runHooks(fullHooks, snap);
}

/* ── SSE ───────────────────────────────────────────────── */
let source = null;
function connect() {
  if (source) source.close();
  source = new EventSource('/api/stream');
  source.addEventListener('open', () => {
    $('conn').textContent = '연결됨';
    $('conn').className = 'chip chip-good';
  });
  source.addEventListener('error', () => {
    $('conn').textContent = '끊김 — 재연결 중';
    $('conn').className = 'chip chip-critical';
  });
  source.addEventListener('hot', (e) => applyHot(JSON.parse(e.data)));
  source.addEventListener('full', (e) => applyFull(JSON.parse(e.data)));
}

async function pollHealth() {
  try {
    const r = await fetch('/api/health');
    const h = await r.json();
    $('clients').textContent = `접속 ${h.sse_clients} · 영상 ${h.video_clients}`;
  } catch (err) { /* 서버가 잠깐 죽어도 화면은 유지한다 */ }
}

/* 초기화 */
(function init() {
  // 기본은 꺼짐 — 페이지를 여는 것만으로 인식 노드에 오버레이 비용이 생기면 안 된다.
  // (노드가 video_default_source 로 다른 값을 지정하면 첫 full 스냅샷에서 반영된다.)
  const saved = localStorage.getItem('video_source') || 'off';
  $('video-source').value = saved;
  setVideoSource(saved);
  connect();
  pollHealth();
  setInterval(pollHealth, 5000);
  window.addEventListener('resize', () => { if (lastFull) drawOverlay(lastFull); });
})();
