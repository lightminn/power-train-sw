# 컴포넌트 마스크(CMASK) 설계 — 콘솔 컴포넌트별 on/off와 estop 비체결

**승인**: 2026-07-18 사용자 결정 4건 반영(아래 D1~D4). 목적: 부분 하드웨어
벤치/현장 복구에서 OFF된 컴포넌트(구동·조향·US-100·로봇팔)의 무응답·오류가
ESTOP/HOLD를 체결하지 않게 한다. 조작은 콘솔 헌장대로 **ops 채널 경유만**.

## 사용자 결정

- **D1 모터 OFF = 미장착 모드**: 명령 미송신 + 감시(fault/stale) 제외.
  "명령 유지+오류 무시"안 기각.
- **D2 US-100 OFF 실기 허용 + 강한 경고**: 2단 확인 위험 문구, 콘솔 상단
  배너 상시 표시, 저널 기록. BENCH 한정 아님(대회 중 센서 고장 복구 수단).
- **D3 영속성 없음**: 재시작 시 무조건 전부 ON(fail-safe). 파일 저장 금지.
- **D4 로봇팔 OFF는 별도 플래그**: 기존 `arm_lock_override`(시한부, 미션
  abort 동반)와 통합하지 않는다. OFF = "이 세션엔 팔 스택 없음" 선언.

## 아키텍처 (A안: ComponentMask + 소스 억제)

컴포넌트 4종 `drive · steer · us100 · robot_arm`, 기본 전부 enabled.

1. **순수 코어** — `ChassisManager`가 마스크를 소유:
   `set_component_enabled(component, enabled, detail="") -> (ok, reason)`.
   - 모터(drive/steer) 토글은 **`mode == "IDLE"`에서만 수락**(아니면
     `(False, "not_idle")`) — 주행 중 미장착 선언은 모순.
   - us100/robot_arm은 어느 모드에서든 수락.
   - `snapshot()`에 마스크 노출(텔레메트리·검증용).
   - OFF 시점에 그 컴포넌트가 공급한 **활성 estop condition/hold 소스는
     해제**하되, **이미 latch된 ESTOP은 자동 해제하지 않는다**(reset은
     기존 Ops 액션만). ON 복귀 시 감시 즉시 재개, 액추에이터는 다음
     `arm()`부터 재편입.
2. **코너 계층** — `CornerModule`에 `drive_enabled`/`steer_enabled`
   (setter). disabled 측은 tick에서 명령 송신·fault/stale 판정·estop 발화를
   전부 스킵하고, `arm()`/`estop()`/`reset_fault()` 대상에서 제외. `state()`
   에 disabled 표시. ChassisManager가 마스크 변경 시 6코너에 전파.
3. **소스 유입 차단(chassis_node)** — US-100·로봇팔은 외부 토픽이 소스이므로
   노드에서 **단일 초크포인트 함수**를 거쳐 interlock에 공급하고, 그 함수가
   마스크를 참조해 차단한다(산재한 if 금지):
   - us100 OFF: `/safety_verdict` 신선도(0.75 s) estop, NO_RESPONSE·near
     estop, CHECKING hold 등 safety-verdict 유래 전부 미공급.
   - robot_arm OFF: `/arm_status` 신선도 처리·`robot_arm` hold 미공급.
   - `command_watchdog`·section·qualification·extraction 등 다른 소스는
     마스크 대상이 **아니다**.

## 표면

- **ops 계약**: `needs_bool` 액션 4종 `drive_enable`/`steer_enable`/
  `us100_enable`/`robot_arm_enable`(콘솔 role, 일반 rate-limit,
  `arm_lock_override` 패턴). chassis_node에 SetBool 서비스 4개
  `~/component_enable_drive`·`_steer`·`_us100`·`_robot_arm` →
  `set_component_enabled`. 거부 사유는 FINAL_REJECTED message로 반환.
- **콘솔 Ops 패널**: 토글 4행(현재 상태 표시 + 2단 확인). `us100_enable(False)`
  확인 문구는 "충돌 안전 센서를 끕니다 — 접근 시 자동 정지 없음"을 명시.
- **상단 배너**: OFF가 하나라도 있으면 주황으로 `MASK: <목록> OFF` 상시 표시.
- **:5005 텔레메트리**: additive 필드 `component_mask` (`{"drive":true,...}`)
  → 콘솔 파서 optional 수용, 해당 패널 값에 `DISABLED` 표기(us100 OFF 시
  SAFETY 배너는 "SAFETY DISABLED(US-100 OFF)" 주황).
- **저널**: 변경마다 `COMPONENT_MASK` 이벤트(component·enabled·요청 role).

## 테스트 / 검증

- 순수: 마스크 기본 ON·IDLE 게이트 거부·OFF 시 소스 해제·latch 비자동해제·
  ON 복귀 재편입·snapshot 노출.
- 코너: disabled 측 무명령·무감시(fake 액추에이터로 송신 0 확인)·enabled 측
  정상 유지.
- 노드: 서비스 왕복, us100 OFF에서 verdict stale/NO_RESPONSE에도 RUN 유지,
  ON 복귀 시 estop 재발동, robot_arm OFF에서 hold 미발생.
- 콘솔: 파서 하위호환(필드 부재), 배너 문자열, 계약 테스트(:5005).
- 젯슨 실기(비회전): US-100 OFF→NO_RESPONSE 유발→RUN 유지→ON→estop 재발동,
  ops 액션 왕복, parity.

## 비범위

- 마스크 영속화, Null 액추에이터 스왑, watchdog/section 소스 마스킹,
  kinematics 변경(조향 OFF 시에도 조향각 0 가정 유지), 팔 레포 수정.
