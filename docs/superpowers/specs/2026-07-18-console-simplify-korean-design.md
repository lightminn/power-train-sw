# 콘솔 간소화·한국어화(KGUI) 설계

**승인**: 2026-07-18. 참조 기준(사용자): 광운대 콘솔 — 사용자 조작 버튼은
비상정지·모듈 on/off·경고 초기화 수준. 결정 4건 + 보충 1건:

- **D1 기본 버튼**: 비상정지 · 경고 초기화(Reset) · 시동(Arm 1.5 s 홀드)/시동
  해제(Disarm) · 모듈 스위치 4행. 나머지(권한 3종·구조 탈출·팔 잠금 해제·
  hold 해제)는 "고급" Expander 안(기본 접힘).
- **D2 비상정지 버튼 신설, 무확인 즉시 발동**: 콘솔에는 현재 E-stop을 "걸"
  버튼이 없다. 신설하되 확인창 없음(비상 조작에 확인은 모순). 해제는 기존
  2단 Reset 그대로.
- **D3 전면 한국어화 + 상태코드 병기**: 버튼·패널 제목·확인문구·배너·상태
  전부 한국어, 안전 상태코드는 `비상정지(ESTOP)` 식 병기. 저널/로그 원문은
  영문 유지(대조용).
- **D4 텔레메트리 요약+상세 접기**: 전원/차대/팔 패널 기본 1~2줄 요약,
  상세 표는 Expander.
- **보충(사용자)**: 모듈 4종 기본값은 **전부 켜짐** — CMASK 무영속 계약
  그대로(재시작 = 전부 ON). UI가 이를 바꾸지 않는다.

## 아키텍처

기각안: gettext 국제화(단일 언어 전환에 과함), 간단/고급 이중 레이아웃
모드(복잡도 2배). 채택 = **순수 라벨 모듈 + 단일 레이아웃 재구성**.

### 1) 비상정지 경로 (ros)
- chassis_node에 Trigger 서비스 `~/estop`: `cm.estop("console", "operator
  emergency stop")` — 모든 모드에서 동작, 이미 latch면 멱등 성공. 응답
  message에 결과 모드.
- ops_contract: `"estop": ActionSpec(_CONSOLE, "service",
  ("/chassis_node/estop",))`. 일반 rate-limit(변이 직렬화 대상 — 단, 비상
  경로이므로 busy여도 거부하지 않고 **estop만은 즉시 실행** 여부 검토:
  구현 단순성을 위해 v1은 일반 직렬화를 따르되, ABANDON 데드라인(10 s)이
  이미 웨지를 방지하므로 수용. 스펙 확정: v1 = 일반 처리).
- broker OpsState에 `chassis_mode: str`(safety_state JSON `mode` — 이미 발행
  중) semantic 필드 추가, 부재 시 "UNKNOWN".

### 2) 콘솔 라벨·패널 (operator_console)
- **`labels.py` 신설**(순수, Gtk 무관): 모든 한국어 문자열 상수 + 헬퍼
  `mode_korean(mode)`("IDLE"→"대기(IDLE)"), `freshness_korean(state)`
  ("LIVE"→"정상수신(LIVE)" 등), 컴포넌트 표기(구동 모터/조향 모터/US-100
  안전/로봇팔), 켜짐/꺼짐.
- **ops_panel.py**: PanelAction에 `advanced: bool = False`,
  `GESTURE_IMMEDIATE` 신설(확인 없이 클릭 즉시 submit — 비상정지 전용).
  액션 표 재구성: 기본 = estop(빨강·IMMEDIATE)·estop_reset("경고 초기화")·
  arm("시동 — 1.5초 홀드")·disarm("시동 해제")·컴포넌트 4행("구동 모터
  [켜짐]" 식); 고급 = authority 3종·extraction_grant("구조 탈출 허가")·
  arm_lock_override("로봇팔 잠금 해제")·clear_transient_hold("일시 정지
  해제"). 확인문구 전부 한국어(US-100 위험 문구 유지).
- **app.py OpsPanel**: 기본/고급을 Gtk.Expander("고급")로 분리, 비상정지
  버튼은 빨강 스타일+큰 높이. 상태줄에 `모드: 대기(IDLE)`(OpsState
  chassis_mode) 상시 표시. **게이트 회색화**: 모터 토글은 chassis_mode가
  IDLE 아닐 때 비활성+틀팁/라벨 힌트 "대기(IDLE) 상태에서만", authority
  행은 authority 기능 비활성 배포에서 서버 거부 시 그대로(회색화는 상태
  정보가 없어 v1 보류 — 거부 사유가 ACK 라인에 한국어로 표시되는 것으로
  갈음). 최근 ACK 라인 한국어(성공/거부·사유).
- **배너·상태 한국어**: `OBSERVE: RX-ONLY | OPS: TOKEN-GATED` → `관측: 수신
  전용 | 조작: 토큰 인증`, SAFETY → `안전 정상(CLEAR)`/`비상정지(ESTOP)`/
  `안전 해제됨(US-100 꺼짐)`, `MASK: … OFF` → `꺼짐: 구동·US-100`. 패널
  Link 행 `정상수신(LIVE) · 순번 N · NN ms`.
- **텔레메트리 요약**(telemetry.py 순수 함수 + 패널 Expander):
  `power_summary(snapshot)` → `"47.6 V · 80% · 정상"`,
  `chassis_summary(snapshot)` → `"모드 대기(IDLE) · 안전 정상 · 바퀴 6/6"`,
  `arm_summary(snapshot)` → `"모터 2 · 최고 45 ℃ 정상"`(부재 시 "미수신").
  각 패널 기본 = 요약 1줄, 기존 상세 grid는 `▸ 상세` Expander로 이동.

### 3) 검증
- 순수: labels 헬퍼·요약 함수·액션 표(기본/고급 분류·estop IMMEDIATE·한국어
  라벨 존재)·모드 게이트 로직.
- **runtime_smoke**: 한국어 UI로 전 경로 실기동 통과(기존 게이트 그대로 —
  추가로 스모크 페이로드에 chassis_mode 반영).
- 실전 E2E(젯슨): ops로 `estop` → latch 확인 → `estop_reset` → IDLE 복구.
  콘솔 육안(사용자): 기본 화면 버튼 구성·한국어 표기.

## 비범위
- gettext/다국어 전환, 간단·고급 이중 모드, 텔레메트리 필드 삭제(표시만
  접음), DualSense 매핑 변경, 팔 레포.
