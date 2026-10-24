# Operator Console

## 비전 탭 (단계별 통합 중)

노트북 Graphite GUI(초록 강조·하단 탭·기존 3D 표시)를 기준으로 통합한다.
기존 L515 영상·지형 오버레이와 Top/Oblique 지도까지 연결했다.
차체 자세 영역은 원시 IMU·바퀴 피드백·로커/보기 선택 필드·지도 정지 기록을 표시한다.
보정된 차체 자세는 아직 미확인이며, 개략도에 미보정 roll/pitch를 적용하지 않는다.
협조구동의 **지형 보기** 버튼도 비전 탭을 연다.

```bash
# 운용 PC, 이 작업 브랜치의 저장소 루트
/usr/bin/python3 -m operator_console.app --host <Gateway-IP> --page vision
# 기존 페어링 설정을 사용하는 통합 진입점
/usr/bin/python3 -m operator_console --config ~/.config/powertrain/operator.json --page vision
```

비전은 기존 SRT :5000 디코더와 UDP :5009 지형 수신기를 공유한다.
탭을 나가면 기존 메인/PiP 배치를 복원한다. 비전에서는 L515 고정으로 V 전환을 막는다.
지형 토글 상태는 실시간 화면과 공유한다. 바닥 영역 mask는 현재 전송되지 않으며,
기존 바닥 상태 배지와 장애물 윤곽만 표시할 수 있다.
노트북 버전의 별도 장애물 :5009 수신기·HTTP 시험대 창은 이 경로에서 기동하지 않는다.

두 지도는 기존 UDP :5010 수신기를 공유한다. 지도 이미지는 별도 작업 스레드에서
각각 약 2 Hz로 생성하며, 비전 탭을 숨기면 지도 계산·화면 갱신을 멈추고 수신은 유지한다.
휠/+/−로 두 지도를 확대·축소하고, Top을 끌면 따라가기가 해제된다.
따라가기 중에는 로버 방향이 위이며, 해제하면 Top은 북쪽이 위이고 Oblique는 선택 위치를 기준으로 본다.
Top 범례는 송신 지도 클래스, Oblique 범례는 기준값 JSON의 표시용 재분류다.
미수신·끊김 시 원점 자세를 만들지 않으며, 오래된 Oblique 투영은 숨긴다.

```bash
# 운용 PC: 코드 수정 없이 기준값 교체 (적용 후 콘솔 재시작)
/usr/bin/python3 -m operator_console.app --host <Gateway-IP> --page vision \
  --l515-criteria /path/to/obstacle_criteria.json --l515-criteria-profile A_no_stop
# 통합 진입점에도 같은 CLI 옵션을 사용할 수 있다.
```

설정 JSON의 `l515_criteria`, `l515_criteria_profile`도 지원한다.
경로·프로필 각각 CLI > 설정 JSON > `L515_CRITERIA`/`L515_CRITERIA_PROFILE` > 저장소 기본값 순이다.
JSON의 상대 경로는 설정 파일 폴더 기준이다. 프로필을 생략하면 파일의 기본 프로필을 사용한다.
화면에 버전·프로필·SHA256 앞 8자, 툴팁에 파일 경로와 전체 해시를 표시한다.
v0.2에서 trench_depth_m이 없으면 기존 함수대로 step_up_m을 사용한다.
이 설정은 Gateway 송신 기준이나 주행 제어를 변경하지 않는다.

기존 GTK/GStreamer 의존성과 numpy, 지도 함수 재사용용 OpenCV가 필요하다.
OpenCV는
운용 PC의 **콘솔과 같은 Python**에 설치한다. 이 PC에서는 관리자 권한 없이
사용자 영역에 numpy 2.5.4 / OpenCV-headless 5.0.0 / pytest 9.1.1을 설치했다.
시스템 Python용 pip가 없다면 배포판 패키지 또는 `--system-site-packages` 가상환경을
사용하되, GTK/gtksink 접근 여부를 확인한다. RealSense SDK는 콘솔에 필요하지 않다.

[단계별 계획](../docs/plans/2026-10-23-vision-tab.md)과
[4단계 결과](../docs/reports/2026-10-23-vision-tab-step4.md)에 검증·미연결 항목을 기록한다.

자세 수치는 :5005의 roll_rad/pitch_rad를 도 단위로 바꾼 **원시값(장착 보정 미확인)**이다.
나이는 원 IMU 측정 시각이 없는 관계로 '패킷 수신 후'만 표시한다.
바퀴는 steer_deg와 drive_turns_per_s의 실제 수신값을 사용하며, stale/결측은 숫자 대신 표시한다.
Hall 저속 값(<0.3 rev/s)은 실제 정지와 다를 수 있으며 지령이나 0으로 바꾸지 않는다.
지도 기준면은 차체 자세와 다르며, 정지 기록은 n초 전의 관측으로만 표시한다.

로커/보기는 선택 필드 suspension_joints_rad, suspension_age_ms를 사용하고,
suspension_source·suspension_frame으로 출처와 기준을 받을 수 있다.
각도 키는 ml_rocker_joint/mr_rocker_joint/bl_bogie_joint/br_bogie_joint, 단위는 rad이다.
나이가 없으면 시간 미확인, 값이 없으면 미수신이며 0이나 추정값을 만들지 않는다.
실제 AS5048B 송신기·영점·부호 합의는 아직 없다. [필드 계약](../docs/reports/2026-10-23-vision-tab-step4.md)을 먼저 확인한다.

Integrated entrypoint: `python -m operator_console` uses the prepared paired-robot
config, reconnects automatically, supervises a separate gamepad process and offers
a held manual-start action. See [the integrated operation runbook](../docs/integrated-operation.md)
for setup, legacy-service migration and the pending real-hardware acceptance.
`python -m operator_console.app --host ...` remains the explicit legacy entrypoint.

Native GTK/GStreamer operator console. Observation is RX-only; operator
actions travel only through the token-gated ops channel. It embeds L515 driving
RGB SRT (`:5000`) and D435i raw RGB SRT (`:5002`) with per-channel receiver health.
It also listens for D435i YOLO metadata JSON on UDP `:5003`; stale metadata is
hidden rather than delaying raw video. Robot health snapshots use UDP `:5004`.
The console shows `UNAVAILABLE` until an approved data owner
(ChassisManager/odometry/power monitor) sends a real snapshot; it never probes
ODrive over USB or CAN to fill this panel.

Telemetry v1 is one bounded JSON UDP datagram. A future sender may omit an
unknown physical measurement as `null`, but must not invent it:

```json
{"schema_version":1,"sequence":42,"odometry_source":"wheel+imu","x_m":1.2,"y_m":-0.4,"yaw_rad":0.3,"voltage_v":null,"current_a":null,"power_w":null,"drive_state":"IDLE","can_state":"OK"}
```

`x_m`, `y_m`, and `yaw_rad` are the robot `odom` pose (wheel states fused with
L515 gyro/accel in WP6), not a camera-pixel coordinate. D435 YOLO object
coordinates stay associated with each camera bounding box on UDP `:5003`.

The chassis sender on UDP `:5005` also mirrors the Gateway's scalar L515
observability contract: native callback rates, all six ROS topic rates,
SRT submit/sent/drop rates, aligned-Depth age, and Gateway process CPU/RSS.
Missing Gateway data remains `null`/`UNAVAILABLE`; the console never opens a
camera to fill it.

Robot-arm telemetry arrives on UDP `:5007` (override with
`--arm-telemetry-port`) from the `arm_console_bridge` ROS node, which mirrors
the arm team's `/dynamixel/state`, `/joint_states`, `/detected_objects`, and
latched `/pick_target` without touching the arm repo. The panel lists each
Dynamixel as `ID · angle° · current(raw) · temperature[state]` — thresholds
WARN 55 ℃ / CRIT 65 ℃ are provisional until the arm team confirms per-model
limits — plus joint angles. Detection overlays on the D435i panel add
`yaw ±deg°` and highlight the latched pick target in orange. To start the
bridge on the robot (arm stack running, single :5003 sender rule — keep the
arm `metadata_sender_node` down while the bridge runs):

```bash
ros2 run powertrain_ros arm_console_bridge --ros-args -p console_host:=<laptop-ip>
```

```bash
/usr/bin/python3 -m operator_console.app --host 192.168.8.106
```

로봇 측에서는 `scripts/jetson_gui_up.sh --operator-host <laptop-ip>`를 사용한다.
이 시작 경로는 남아 있는 `moveit_dynamixel_bridge`를 먼저 종료하고, 토크·목표
위치 쓰기 없이 자동 도구 감지를 수행하는 읽기 전용 브릿지를 다시 올린 다음
`arm_console_bridge`로 상태를 관제 화면에 전달한다.

The judge-facing interface uses **실시간 화면** (mission/video) and **시스템 상태**
(integrated power, communication, and safety summaries with collapsed raw
diagnostics). Open **복구 · 설정** beside the two-tab navigation to reach the
existing token-gated controls for E-STOP reset, component settings, steering
mode, and advanced recovery actions. This non-modal window hides on close and
cancels any unfinished confirmation; reopening it keeps the same authenticated
ops session. Press `F11` for the competition display. Camera panels can be
clicked to exchange the large and preview views; technical stream errors
remain in the collapsed event log and robot diagnostics.

`/usr/bin/python3 -m operator_console.runtime_smoke` launches the real GTK
application under Xvfb, injects maximal synthetic datagrams into isolated UDP
ports, and verifies LIVE→STALE, environment rendering, sparse-payload,
clean-shutdown, traceback, role-layout, and no-automatic-camera-swap paths. A
loopback ops TCP fixture also sends an unknown steering mode through the real
client and checks that the GTK control stays unavailable. The vision stage also
decodes local synthetic SRT video and checks missing/live/aged/stale/recovery,
shared camera round trips (including D435 MAIN), hidden rendering pause with
continued reception, and worker/timer/receiver/pipeline shutdown. Run just this
stage with `--vision-only` to distinguish its result from existing arm failures.
These checks exercise neither a real network nor a hardware sender; Gateway/L515
acceptance and the earlier intermittent draw stall still require separate evidence.

The lower-right mission rail is the robot-arm/tool hub. It exposes only the
four confirmed equipment categories (그리퍼 1, 그리퍼 2, 청소 모듈, 환경 센서
모듈), plus attachment state, joint-load summary, and the explicit fact that
arm control-mode telemetry is not yet wired. Selecting a tool or pressing its
detail button opens a common non-modal detail window. The environment window
groups climate, air quality, and hazard readings. CO/LPG use plain numeric
values, flame detection is shown as O/X, and SGP30 eCO₂/TVOC values display
`예열 중` until warmup ends. Lost sensor power or stale input clears the previous
readings and displays 미연결. Source/sequence details stay collapsed.

The **시스템 상태** tab is a single RX-only live view for PDIST80B power,
communications, and safety. Drive speed and AI summaries are on the mission
page; drive pose/mode and standalone arm diagnostics are absent from the
operator surface. PDIST80B shows only fields already carried by telemetry:
voltage, charge/discharge current, power, SOC, operating/protection flags,
RS485 state, and freshness. Missing protocol fields remain `정보 없음` and are
never read directly by the GUI or synthesized; see `docs/ui_data_gap.md`.

The **협조구동** tab is the engineering-festival mobility surface. It combines
the L515 terrain estimate (path, confidence, offset, heading, bank, slope and
roughness), filtered IMU roll/pitch, six wheel Hall speed/command/Iq, active
faults, and chassis/autonomy/mission/section FSM states. Missing producers stay
`정보 없음`; Iq is labelled as a relative load indicator, not absolute wheel
load or energy. Its basic controls are shortcuts into the existing token-gated,
revision-checked confirmation panel. Velocity and direction remain on the
approved DualSense teleop path, and E-STOP remains the global immediate action.

The **시연 화면 → 화면 표시** checkboxes independently control YOLO
box/class/confidence and target-distance text. They do not stop metadata
reception or alter target selection, and stale metadata still hides every
overlay after the measured-inference-aware 500 ms deadline. A valid target is
held for at most 350 ms across a brief empty detection frame to prevent visual
flicker; the value is never extended past the stale deadline. The work-camera preview keeps
the native 848:480 ratio and becomes the main view only after its explicit
`클릭하여 크게 보기` action. Display quality gates omit detections clipped on
three or more frame borders; the immutable raw metadata snapshot remains
available to diagnostics.

### Isolated telemetry replay

Before hardware integration, captured JSONL packets can be sent through the
same UDP decoders on ports isolated from live operation. Each line uses
`{"t":0.0,"channel":"power","payload":{...}}`; the payload is transmitted
unchanged.

```bash
/usr/bin/python3 -m operator_console.app \
  --host 127.0.0.1 --metadata-port 15003 --telemetry-port 15004 \
  --chassis-telemetry-port 15005 --arm-telemetry-port 15007 \
  --input-source REPLAY

/usr/bin/python3 tools/operator_console_replay.py capture.jsonl
```

The replay tool defaults to 15003/15004/15005/15007 and rejects the live UDP
ports. Recorded video or a GStreamer test sender must still use the existing
SRT receiver contract; no DEMO control is added to the operator UI.

For a continuous hardware-free rehearsal, start the bounded TEST fixture and
point the same production receivers at its isolated ports:

```bash
python -m operator_console.demo_source

/usr/bin/python3 -m operator_console.app \
  --host 127.0.0.1 --d435-port 15002 --l515-port 15000 \
  --metadata-port 15003 --telemetry-port 15004 \
  --chassis-telemetry-port 15005 --arm-telemetry-port 15007 \
  --environment-telemetry-port 15008 --terrain-port 15009 --terrain-map-port 15010 --input-source TEST \
  --ops-token-file /tmp/operator-console-no-token
```

The fixture cycles through normal driving, rough terrain, traction assistance,
and obstacle hold. It opens no device or ops command channel and refuses every
live telemetry port. The field ownership, units, freshness rules, and remaining
team decisions are recorded in the
[festival GUI integration contract](../docs/operator-console-integration-contract.md).

Use the system interpreter explicitly: a conda-base `python3` has no GTK
bindings (`gi`) and dies with ModuleNotFoundError. The systemd unit already
pins `/usr/bin/python3`.

If startup fails with `no element "gtksink"`, install `gst-plugin-gtk` —
Arch split the GTK3 sink out of gst-plugins-good (bit us 2026-07-18).

The ops broker defaults to the same host on TCP `:9001`, with its token read
from `~/.config/powertrain/ops_console.token`. Override these independently
with `--ops-host`, `--ops-port`, and `--ops-token-file`. If the token file is
absent or empty, the command panel is disabled while every observation channel
continues normally.

For the integrated `python -m operator_console` entrypoint, the validated
`~/.config/powertrain/operator.json` keys include
`environment_telemetry_port` (default `5008`), `terrain_port` (default `5009`),
`terrain_display_delay_ms` (0–400, default 0), `terrain_map_port` (default `5010`) and
`terrain_map_display` (an object of `DisplayConfig` fields — out-of-range or non-integer values
refuse to start). Port overrides must be
integer ports from 1 through 65535 and must match the robot-side sender
configuration. The L515 terrain outlines are an operator visual aid only.

The **지형 지도** page draws the L515 2.5D terrain map (UDP `:5010`, `--terrain-map-port`):
heading-up while following the rover, raised cells red and ditches blue on their raw cell
boundaries, shaded heights, unobserved cells transparent, cells older than `--map-stale-s`
(default 30 s) desaturated. The status line shows status, epoch, distance, pose gaps,
received/total tiles, packet rate and sender; "수신 끊김" appears when no status packet
arrived for 0.5 s and the last pose turns gray. Scroll to zoom, drag to pan (stops following),
"화면에서 지우기" clears only the console's copy. Display numbers: `--map-display-upsample`,
`--map-hillshade-azimuth`, `--map-hillshade-altitude`, `--map-hillshade-exaggeration`,
`--map-stale-s`, `--map-max-render-pixels` (out of range is a usage error). Producer side and
parameters: `terrain_map/README.md`.

Observation remains caller/receiver-only: it never opens D435i/L515 or writes
CAN. Commands are possible only through the authenticated ops client and every
panel action passes state-revision revalidation plus its confirmation gesture.

### 비전 탭 성능 계측 (개발용)

비전 탭은 기존 L515 위젯·지도·자세 수신기를 공유한다. NumPy import 전
`OPENBLAS_NUM_THREADS=1`을 적용하고, 영상은 `sync=false`와 raw 프레임 1개 대기열을 사용한다.
지도·자세의 비전 전용 계산/표시는 탭이 보일 때만 갱신한다.

실제 카메라 없이 로컬 SRT/UDP 합성 입력으로 CPU/RSS·영상 수신/그리기·지도 반영률을
측정하려면 저장소 루트에서 다음을 실행한다. GTK/GStreamer·NumPy/OpenCV·pytest가 필요하다.

```bash
OPENBLAS_NUM_THREADS=8 xvfb-run -a /usr/bin/python3 \
  -m operator_console.tests.vision_benchmark --require-ac \
  --seconds 20 --output /tmp/vision-performance.json
```

인코더/송신은 별도 프로세스이며 측정 CPU/RSS에서 제외한다. 실기/전원 조건·그리기 정지
미해결 사항과 수치 해석은 [5단계 보고서](../docs/reports/2026-10-24-vision-tab-step5.md)를 따른다.
이 합성 시험은 Gateway/L515 10분 인수를 대체하지 않는다.
