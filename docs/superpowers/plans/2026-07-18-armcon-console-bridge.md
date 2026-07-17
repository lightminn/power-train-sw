# ARM-CON 배치 구현 계획 — 로봇팔 콘솔 기능 (⏸️ 계획만, 구현 보류)

> **상태: 사용자 지시(2026-07-18)로 계획서만 작성 — A/B/C 프로그램(A2b~C1) 완주 후
> 또는 별도 지시 시 착수.** 착수 전 필수: ①팀원 gateway WIP(l515_dashboard) 랜딩
> 여부 재확인(§주의 ⓐ) ②팔 팀 토픽 계약 재검증(아래 조사 스냅샷과 대조).
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

- 송신 ②: UDP :5003 — 기존 콘솔 metadata.py가 파싱하는 포맷 그대로(착수 시
  `operator_console/metadata.py` 스키마 확인 후 매핑) + 확장 필드
  `class_name`·`yaw_rad`(= `2*atan2(z,w)` 정규화 ±π)·`is_pick_target`.
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

### Task 4: `arm_video_bridge` — 실시간 화면 최적화 (M)

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/arm_video_bridge_node.py` (+setup entry)
- Test: `ros2/src/powertrain_ros/test/test_arm_video_bridge.py` (파이프라인 문자열·파라미터 계약 — GStreamer 실행은 실기 몫)

**Interfaces:**
- `/perception/raw_image` 구독(latest-only 슬롯 — autonomy 노드 워커 패턴 재사용)
  → GStreamer `appsrc → videoconvert → x264enc(ultrafast, zerolatency,
  bitrate 파라미터) → h264parse → mpegtsmux → srtsink(listener :5002,
  latency 60)` — `gst_stream.py`의 검증 설정(Orin NVENC 부재 → SW 인코딩,
  848×480/15 fps 기본, 파라미터화) 재사용.
- 파라미터: `source_topic`(기본 raw_image — debug_image로 전환 가능),
  `width/height/fps/bitrate_kbps`, `srt_port`(기본 5002).
- 콘솔 수신측 작업 0(기존 D435i 패널). 오버레이는 :5003 분리 원칙(영상 지연
  최소 — L515 관례) — debug_image 스트리밍은 폴백 옵션으로만.

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
