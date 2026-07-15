# 원격 운용 GUI·통신·스트리밍 인계

## 목적

2026 국방/극한로봇 파워트레인의 대회 운용용 원격 GUI를 설계·구현한다. 목표는 단순 영상 뷰어가 아니라, 운용자가 주행·로봇팔 작업·안전 상태를 판단할 수 있는 **분리 통신 기반 운용 화면**이다.

## 이미 확인된 프로젝트 사실

- 온보드: Jetson Orin Nano Super.
- 전용망: GL-SFT1200 AP. Jetson은 유선 LAN `192.168.8.106`, 운용 노트북은 5 GHz Wi-Fi. 실측 왕복 ping 평균 2.19 ms, loss 0%.
- L515는 파워트레인 전용 RGB/depth/IMU 센서이고 D435i는 로봇팔 전용이다. 서로의 SDK를 열면 안 된다.
- L515 production owner는 오직 `python3 -m l515_dashboard.gateway_main`이다. Gateway를 우회해 카메라를 직접 열지 않는다.
- L515 Gateway HIL: RGB 1280x720 30 fps, raw depth 640x480 10 Hz, H.264/MPEG-TS/SRT, x264 `ultrafast`, threads=3, 3000 kbit/s. Orin Nano에는 NVENC가 없으므로 `nvv4l2h264enc`를 사용/문서화하지 않는다.
- L515 SRT는 listener `:5000`이다. D435i raw RGB의 계획상 SRT listener는 `:5002`다.
- L515 Gateway는 RGB/depth/overlay를 동시 다중 인코딩하는 서비스가 아니다. RGB/Depth/overlay는 하나의 파이프라인에서 선택 전환하며, depth/overlay는 best effort다.
- 차체는 하나의 500 kbps `can0`으로 AK 조향 4개 + ODrive 구동 6개를 제어한다. `chassis_node`/`ChassisManager`가 실제 모터·E-stop 집행의 유일한 owner다.

## 작년 출품작에서 가져올 원칙

- KUDOS: 영상과 제어를 분리한 저지연 스트리밍, 인식 결과의 실시간 표시.
- Zenith Space: 현재 모드, 상태, 조정 가능 항목을 운용자가 확인하는 GUI.
- iRASC: 브라우저 기반 통합 관제의 정보 구조.
- RO:BIT: 다중 카메라를 운용 PC에서 보는 구성.

우리 구현은 이 패턴을 따르되, 현재 Orin Nano의 소프트웨어 x264 한계를 고려해 여러 L515 영상 채널을 항상 동시에 송출하지 않는다.

## 권장 아키텍처

```text
[Robot / Jetson]
L515 Gateway ── SRT :5000 ───────────────┐
D435i arm owner ── SRT :5002 ────────────┼──> [Operator laptop GUI]
D435i perception ── UDP JSON :5003 ──────┤      - dual video receiver
                                                   - client-side detection overlay
Remote-input gateway <── versioned input ──────── - DualSense DRIVE/ARM controls
chassis_node / CommandAuthority ── state ──────── - status + event view
                                                   - deadman / hold request
```

### 채널별 역할

| 채널 | 방향 | 데이터 | 정책 |
|---|---|---|---|
| SRT `:5000` | Jetson → PC | L515 주행 영상 | RGB 기본. Depth/overlay는 모드 전환. |
| SRT `:5002` | Jetson → PC | D435i 로봇팔 원본 영상 | 로봇팔 owner가 한 번 capture한 raw RGB. |
| UDP `:5003` | Jetson → PC | D435i YOLO metadata | bbox, class, confidence, 좌표. 영상과 별개 best effort. |
| remote input | PC → Jetson | versioned DualSense/운용 입력 | stale, sequence, deadman 검증 필요. |
| telemetry/events | Jetson → PC | 모드·안전·CAN·센서·frame age | 제어 권한을 갖지 않는 관측 채널. |

포트 `:5000`, `:5002`, `:5003`은 현재 계획의 값이다. 나머지 control/telemetry 포트·스키마는 충돌 없이 명시적으로 할당한다.

## 절대 지켜야 할 안전/권한 경계

- GUI는 CAN/모터를 직접 제어하지 않는다.
- remote input은 `/teleop/cmd_vel` 제안만 만들고, `chassis_node` 내부 `CommandAuthority`가 `/teleop/cmd_vel`과 `/autonomy/cmd_vel`을 선택·freshness 검사·zero-confirmed handover·arm interlock 후에만 `ChassisManager.set()`으로 전달한다.
- 외부 final `/cmd_vel` publisher를 새로 만들지 않는다.
- deadman 또는 remote freshness 상실은 `MOTION_HOLD`여야 한다. GUI 버튼 하나를 물리 E-stop의 대체물로 만들지 않는다.
- US-100 근접/연속 liveness 실패는 기존 latched E-stop 정책을 따른다.
- arm 작업은 `MISSION_STOP`과 실 wheel-stop 확인 뒤에만 허용된다. `DRIVE`와 arm jog를 동시에 만들 수 없다.
- 영상 frame loss/지연은 명확한 경고·운용 hold 근거가 될 수 있지만, 단독으로 차체 E-stop을 직접 집행하지 않는다.

## GUI 화면 우선순위

### 운용 화면 (우선 구현)

1. L515 주행 영상(메인) + D435i 로봇팔 영상(보조).
2. D435i UDP metadata를 노트북에서 영상 위에 합성.
3. 상단 health bar: link, L515/D435 frame age, CAN alive, safety verdict, E-stop, 현재 mode.
4. 현재 명령원(teleop/autonomy/none), deadman, `MOTION_HOLD`, arm lock 상태를 명확히 표시.
5. 이벤트 로그: frame stale, SRT reconnect, CAN error, safety transition, authority handover.
6. DRIVE/ARM/AUTO/MISSION_STOP은 권한 상태를 보여 주되, 안전 계약을 우회하는 버튼을 만들지 않는다.

### 정비 화면 (분리 유지)

기존 `motor_gui`는 실모터의 게인 조정·캘리브레이션·CAN ID 선택·직접 명령을 포함한 정비/HIL 도구다. 대회 운용 GUI에 합치지 않는다. 운용 GUI에는 `10/10 alive`, fault motor, heartbeat age, CAN error, 최대 전류/온도 등의 읽기 전용 요약만 둔다.

## 구현 순서

1. 노트북에서 L515 `:5000`과 D435i `:5002`를 동시에 수신·표시한다.
2. 채널별 decode/display FPS, frame age, sequence gap, SRT reconnect를 표시한다.
3. D435i metadata `:5003`를 client-side overlay로 표시한다.
4. 상태/이벤트 read-only 채널을 추가한다.
5. versioned remote input + deadman을 구현하되 CommandAuthority에만 접속한다.
6. arm interlock·MISSION_STOP 계약을 연결한다.
7. L515 RGB/Depth/overlay mode 전환과 DEGRADED 화면을 추가한다.

## 하지 말 것

- L515 RGB, depth, overlay를 독립 30 fps SRT 세 채널로 동시에 인코딩하지 말 것.
- L515를 Gateway 외의 Python/ROS 노드에서 직접 열지 말 것.
- D435i를 파워트레인 프로세스가 열지 말 것.
- `nvv4l2h264enc`를 쓸 수 있다고 가정하지 말 것.
- GUI에 motor_gui의 캘리브레이션·라이브 게인·직접 motor command를 넣지 말 것.
- SRT/영상, remote control, telemetry, E-stop을 동일 포트/프로세스/권한으로 섞지 말 것.

## 주요 근거 문서

- `docs/reports/2026-07-12-l515-gateway-performance-hil.md`
- `docs/plans/2026-07-13-observability-data-quality-remote-assist-plan.md`
- `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`
- `docs/specs/2026-07-10-wp5-control-safety-hardening-design.md`
- Notion: `통신 스트리밍 GUI 분석 및 우리 팀 적용 방향`
- Notion: `L515 Gateway·TUI 원격주행 — 비동기 RGB 30 fps·SRT HIL`
- Notion: `무선 라우터(GL-SFT1200) — 노트북↔젯슨 전용망 셋업`
