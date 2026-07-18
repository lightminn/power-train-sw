# ARM-CON 배치 구현 계획 — 로봇팔 콘솔 기능 (▶️ 착수 2026-07-18, r2)

> **상태: 사용자 지시(2026-07-18)로 착수.** 착수 전 체크 결과(r2 개정 사유):
> ①팀원 gateway WIP **미랜딩**(l515_dashboard M 파일 유지 — 파일 충돌 없음, 진행)
> ②팔 레포 워킹카피 재검증 — `perception_node.py` 미커밋 수정(+19/-4)에도 발행
> 계약 불변(/detected_objects·/pick_target·raw_image·PCA orientation 유지),
> `dynamixel_position_node.py` 무변경 ③**:5007 소스 전체 미사용 재확인**
> ④**중대 발견: 팔 레포에 `stream_node`(raw_image→SRT :5002)와
> `metadata_sender_node`(:5003, class_name 포함·yaw 없음)가 이미 존재** →
> **Task 4(arm_video_bridge) 폐기**, Task 1 송신②는 팔 sender와 동일 기본
> 스키마의 strict superset(yaw_rad·is_pick_target 추가)으로 확정. :5003은
> 단일 송신 원칙 — 우리 브리지 가동 시 팔 metadata_sender는 미기동(배포 조율,
> 팔 레포 수정 없음).
> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경 + 젯슨 실기 검증.

**Goal:** 로봇팔 팀 요청 4종 — ①다이나믹셀 전류·온도 ②원격조종 조인트 현황
③YOLO 검출(class·yaw) 표시 ④실시간 화면(최적화) — 을 **팔 레포 0수정**으로
operator_console에 구현한다.

**Architecture:** 우리 레포에만 3개 조각 — ①`arm_console_bridge` 노드(팔 토픽
read-only 구독 → 기존 콘솔 UDP 텔레메트리 패턴 :5007 + 기존 :5003 메타 포맷
미러) ②콘솔 `ArmTelemetryPanel`(A2b 패널 인프라 재사용) ③`arm_video_bridge`
(Image→appsrc→x264→SRT :5002, L515 검증 설정 재사용). 명령 표면 0 — 콘솔 헌장
(A2b 개정판: 관측 RX + ops 채널만) 그대로.

## 조사 스냅샷 (2026-07-18, `~/extreme-robot` @ `24f4c4c` read-only)

팔 팀 발행(코드 확인 완료 — `dynamixel_control/dynamixel_position_node.py`,
`robot_arm_perception/perception_node.py`):

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/dynamixel/state` | `std_msgs/Int32MultiArray`, 10 Hz | flat `[id, position(raw 0~4095), velocity, current, temperature] × N모터` (레지스터 126/146) |
| `/joint_states` | `sensor_msgs/JointState`, 10 Hz | 관절명·radian(`(raw-2048)·2π/4096`)·velocity |
| `/detected_objects` | `robot_arm_msgs/DetectedObjectArray` | class_id/class_name/confidence/bbox + `pose.orientation` = PCA yaw 쿼터니언 → `yaw = 2·atan2(z, w)` |
| `/pick_target` | `DetectedObject`, TRANSIENT_LOCAL | 최신 픽 타깃(참고 표시용) |
| `/perception/raw_image`·`/perception/debug_image` | `sensor_msgs/Image` | 비압축 — Wi-Fi 직송 부적합, ④의 소스 |

주의: ⓐ팀원 gateway WIP가 D435i 송신자를 품을 계획이었음 — 본 계획의
`arm_video_bridge`는 별도 소형 노드라 파일 충돌은 없지만 **랜딩 시 중복 정리
조율 필요**(착수 전 확인 항목). ⓑ`/dynamixel/state`는 표준 msg라 계약이
느슨 — 방어 파싱 + 계약 스냅샷 테스트 필수. ⓒ팔 스택은 DDS domain 0 —
브리지도 0에서 구독(테스트는 관례대로 77 격리 + fixture 발행).

## Global Constraints

- A2a/A2b 계획의 Global Constraints 승계. **`~/extreme-robot` 수정 절대 금지.**
- 콘솔 신규 표시는 전부 수신 전용 — ops 채널 외 송신 금지 계약 테스트(A2b Task 4)에
  저촉되지 않게 신규 소켓은 수신 bind만.
- 포트: **:5007**(arm 텔레메트리 UDP — 레포 전체 미사용 확인 필요, 착수 시 grep),
  기존 **:5003**(D435i 메타 — 콘솔 수신부 존재), 기존 **:5002**(D435i SRT — 콘솔
  패널 존재). `remote_video/contract.py`에 상수 추가.

---

### Task 1: `arm_console_bridge` 노드 — 텔레메트리·검출 미러 (S~M)

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/arm_console_bridge_node.py`
- Modify: `remote_video/contract.py` (`ARM_TELEMETRY_UDP_PORT = 5007`), `ros2/src/powertrain_ros/setup.py` (entry `arm_console_bridge`)
- Test: `ros2/src/powertrain_ros/test/test_arm_console_bridge.py`

**Interfaces:**
- 구독(전부 read-only): `/dynamixel/state`·`/joint_states`·`/detected_objects`·`/pick_target`.
- 송신 ①: UDP :5007(대상 호스트 파라미터 `console_host`) 5 Hz JSON —

```json
{"schema_version":1,"sequence":N,"stamp_s":...,
 "dynamixel":[{"id":11,"position_raw":2048,"position_deg":0.0,
               "velocity":0,"current":-12,"temperature_c":34}, ...],
 "joints":{"names":[...],"position_rad":[...],"velocity":[...]},
 "source_age_s":{"dynamixel":0.1,"joints":0.1,"detections":0.4}}
```

- 송신 ②: UDP :5003 — **확정(r2)**: 팔 `metadata_sender_node`와 동일 기본 스키마
  (`schema_version:1`·`capture_sequence`=stamp ns·`capture_stamp_ns`·
  `frame_width/height`·`frame_id`·`detections[{class_id,class_name,confidence,
  bbox_xywh,position_m}]`, 상한 2048 B — `operator_console/metadata.py`
  `parse_metadata`가 이 포맷을 그대로 파싱함을 코드로 확인) + 확장 필드
  `yaw_rad`(= `2*atan2(z,w)` 정규화 ±π)·`is_pick_target`(latched `/pick_target`과
  class_id+bbox 정확 일치 시 — best-effort). 콘솔 파서는 미지 필드 관대 →
  additive 안전. 초과 시 confidence 낮은 검출부터 절단.
- 구현 분할(레포 관례 — `chassis_telemetry_sender` 패턴): 순수 모듈
  `powertrain_ros/arm_console_mirror.py`(파싱·변환·페이로드 인코딩, rclpy 무관)
  + 얇은 노드 `arm_console_bridge_node.py`. 크로스 계약 테스트는 최상위
  `tests/`(빌드된 페이로드를 콘솔 파서로 직접 파싱 — beacon 계약 테스트 선례).
- 방어 파싱: `/dynamixel/state` 길이가 5의 배수 아니면 drop+WARN(1 Hz 스로틀),
  개수 상한 8모터, 값 int 범위 검증. 소스별 최신 stamp → `source_age_s`.
- 페이로드 상한 4096 B(기존 텔레메트리 계약과 동일), 초과 시 dynamixel 우선
  유지·detections 절단 + `truncated` 플래그.

**Steps (TDD):** ①테스트 — fixture 발행(도메인 77)→UDP 수신 캡처: 정상 미러·
5배수 위반 drop·yaw 복원 수치(`quat(z=sin(θ/2),w=cos(θ/2))`→θ)·절단 계약·
sequence 단조 ②RED ③구현 ④ros 컨테이너 GREEN ⑤커밋.

---

### Task 2: 콘솔 `ArmTelemetryPanel` (S~M)

**Files:**
- Create: `operator_console/arm_telemetry.py` (수신·파싱 순수 모듈 — `LatestTelemetryReceiver` 재사용/유사)
- Modify: `operator_console/app.py` (side box에 패널 추가, `--arm-telemetry-port` 기본 5007)
- Test: `operator_console/tests/test_arm_telemetry.py`

**Interfaces:**
- 표: 모터별 `ID | 각도° | 전류 | 온도℃` — 온도 임계(경고 55 ℃/위험 65 ℃ —
  **착수 시 팔 팀에 모델별 한계 확인**, 임시값 주석) 색 강조, stale(>1 s) 시
  기존 관례대로 `UNAVAILABLE`. 조인트 현황은 관절명+각도° 목록(원격조종 확인용).
- 순수 파싱 함수 `parse_arm_telemetry(payload) -> ArmTelemetry | None`
  (스키마 위반 None — 기존 `parse_telemetry` 관례), 테스트는 순수부만.

---

### Task 3: 검출 오버레이 확장 — class·yaw (S)

**Files:**
- Modify: `operator_console/metadata.py`(+`app.py` MetadataCanvas)
- Test: `operator_console/tests/` 기존 메타 테스트 확장

**Interfaces:** bbox 라벨을 `"{class_name} {conf:.2f} yaw {deg:+.0f}°"`로,
`is_pick_target`이면 강조색. 필드 부재 시 기존 표시 유지(하위 호환 —
스키마 additive).

---

### Task 4: ~~`arm_video_bridge`~~ — **폐기 (r2, 2026-07-18)**

**폐기 사유:** 착수 전 체크에서 팔 레포
`robot_arm_perception/stream_node.py`(raw_image→appsrc→SRT listener :5002,
setup entry `stream_node`)가 이미 존재함을 확인 — ④실시간 화면은 팔 스택의
stream_node + 우리 콘솔의 기존 D435i 패널(:5002 수신)로 이미 충족된다.
중복 구현 대신 Task 5에서 배포 확인(팔 stream_node 기동 → 콘솔 수신 육안)만
수행한다. 우리 레포 신규 코드 0.

---

### Task 5: 문서·검증·젯슨 실기 (리뷰어 주도)

- 핸드오프 §2 체인·§크로스팀 절에 ARM-CON 기록, CLAUDE.md operator_console 줄,
  `remote_video/contract.py` 주석. 팔 팀에 공유할 사용법 1절(콘솔 실행 옵션).
- 3환경 회귀 + **젯슨 실기**: 팔 스택 기동 상태에서(팔 팀 협조) bridge 노드
  실행 → 콘솔에서 다이나믹셀 표·조인트·오버레이·영상 육안 확인, `/dynamixel/state`
  10 Hz 실측 대비 :5007 5 Hz·age 표시 검증, 영상 지연 체감(목표: L515 경로와
  동급). 팔 스택 미가동 시 fixture 발행으로 대체하고 실검증은 협조 세션으로 이월.
- ⚠️ 착수 시 재확인 체크리스트: 팀원 gateway WIP 랜딩 여부(:5002 중복),
  `/dynamixel/state` 포맷 불변 여부, :5007 미사용 재grep, 온도 임계값(팔 팀).

## 완료 기준

- 콘솔에서 팔 요청 4종 전부 표시(팔 레포 diff 0), 헌장(송신 표면 계약) 무저촉.
- 브리지 방어 파싱·절단·age 계약 테스트 통과. 3환경 green + 젯슨 실기(협조 세션).
