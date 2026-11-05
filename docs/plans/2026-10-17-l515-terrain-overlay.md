# L515 지형 장애물 오버레이 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** L515 raw depth에서 바닥 위 장애물 윤곽을 찾아 UDP :5009로 보내고, 운용 콘솔의 전방 영상 위에 참고용으로 그린다.

**Architecture:**
- 순수 패키지 `terrain_overlay/`가 판정·투영·윤곽·패킷 계약·송신 틱 논리를 모두 갖는다. 이 중 `contract.py`만 표준 라이브러리로 되어 있어 콘솔도 쓴다.
- `powertrain_ros/terrain_overlay_node.py`는 ROS 메시지 변환과 UDP 송신만 하는 얇은 어댑터다.
- 콘솔 쪽은 `operator_console/terrain.py`가 세션 게이트·표시 지연·표시 판단을 맡는다. GTK `TerrainCanvas`는 그 결과를 그리기만 한다.

**Tech Stack:**
- Python 3.10(Jetson ROS Humble 이미지)과 3.14(운용 PC 시스템 python), numpy, opencv-python-headless
- rclpy(Humble), GTK3/cairo(PyGObject), Docker Compose

**Spec:** `docs/specs/2026-10-17-l515-terrain-overlay-design.md` (커밋 `78d14b1`). 실행자는 스펙과 이 계획을 함께 읽는다.

**작성 방식:** 이 계획의 모든 코드와 시험은 2026-10-17에 저장소 HEAD(`78d14b1`) 사본에서 먼저 돌려 보고 넣었다. 아래에 적은 통과 수는 그 실측값이다.
- 호스트 conda: 코어 116, lidar_dev 5, remote_video 30.
- 시스템 python: 계약 59, 콘솔 지형 25, GTK 배선 9.
- 로컬 `ros:humble-ros-base` 컨테이너: 노드 시험 10개(반복 통과), colcon 설치 엔트리포인트 스모크와 그 음성 대조.
- 콘솔 전체 스위트: 기준선과 실패 목록이 같았다.
- Codex 1–4회차 지적과 agy 독립 검토를 반영한 뒤의 값이다. 반영 내용은 끝의 "독립 검토" 절에 있다.

## Global Constraints

모든 Task의 요구사항에 아래가 포함된다.

**기능 범위**
- **운용자 시각 보조 전용.** ROS 토픽을 발행하지 않는다. 주행 제어·정지·속도 제한·`SafetyInterlock`·US-100에 어떤 입력도 주지 않는다(스펙 §1).

**패킷·포트**
- 채널은 UDP **:5009**, 패킷 v1, UTF-8 compact JSON, **≤ 1200 B**다(스펙 §5).
- 5001·5003은 쓰지 않는다.

**의존 방향**
- `terrain_overlay/contract.py`는 **표준 라이브러리만** import한다. 콘솔은 `/usr/bin/python3`로 돌고 거기엔 cv2가 없다.
- `terrain_overlay`는 rclpy·gi·`powertrain_ros`·`operator_console`을 import하지 않는다.

**입력·카메라**
- depth 토픽은 `/l515/depth/image_rect_raw`다. 16UC1, `is_bigendian == 0`, `step == width*2`.
- depth 스케일은 0.00025 m이고, 코어 입력은 raw uint16만 받는다.
- 컬러 프레임 좌표계는 1280×720이다.
- `color_offset_m`은 `[0.0, -0.024, 0.0]`이고 회전은 없다. 투영은 핀홀만 쓴다.

**판정 파라미터**
- 파라미터 기본값: `path_half_width_m 0.4445`, `front_offset_m 0.0`, `alignment_verified false`, `publish_hz 10`(허용 5–10), `depth_stale_s 0.5`, `ground_hold_s 0.5`.
- stale 계약: depth가 끊긴 뒤 ≤ `depth_stale_s + 1/publish_hz + 최대 처리시간 + 0.05 s` 안에 `SENSOR_ERROR`/`DEPTH_STALE`을 보낸다. 처리시간이 0이면 스펙의 0.65 s(5 Hz에서 0.75 s)다.
- 최대 처리시간은 한 틱(`1/publish_hz`) 이하여야 한다. 그러면 상한은 10 Hz에서 0.75 s, 5 Hz에서 0.95 s다(5 Hz에서도 처리 ≤ 100 ms이면 0.85 s).
  - 성공·예외와 관계없이 모든 처리 시도를 반올림 전 값으로 한 틱과 비교한다. 넘으면 `processing_overruns`를 세고, 일반 오류와 **다른 호출 위치**에서 경고를 남긴다(rclpy 로그 제한은 호출 위치별이다).
  - 노드 종료 때 제한 없는 요약 한 줄(`max … ms, overruns N`)을 남기고, §7.3-2는 이 줄로 판정한다(Codex 2회차 P2-1, 3회차 P2, 4회차 P2).
- 이 완화의 근거는 범위가 좁다. 콘솔은 **수신이 끊긴 동안에는** 0.5 s 연결 생존 한계로 독립적으로 윤곽을 지운다. 그러나 늦은 오류 패킷이 도착하면 연결이 되살아나므로, 그 뒤에는 표시 지연 버퍼와 depth 나이 규칙이 적용된다. 표시 지연 400 ms에서는 옛 OK 윤곽이 잠깐 다시 보일 수 있다. 이는 스펙 §4.6의 영상 시각 기준 표시 정책에 따른 동작이다(Codex 3회차 P3).

**콘솔 표시**
- 표시 지연 0–400 ms. 연결 생존 한계 0.5 s, depth 나이 한계 0.6 s.
- 윤곽은 영상 크기가 1280×720으로 확인됐을 때만 그린다.

**세션**
- 세션 키는 `(sender_boot_id, sender_start_mono_ns, sender_instance)`다.
- 다른 부팅의 세션으로는 1.0 s 동안 수신이 없을 때만 넘어간다.
- time namespace를 쓰는 배포는 지원하지 않는다.

**배포**
- `scripts/robot-start` 기본 목록에는 스펙 §7.3-2(두 인코더 부하 판정)를 통과하기 전까지 넣지 않는다.

**저장소 규칙**
- 커밋할 때 `git add`는 파일 단위로만 한다. `git add .`·`-A`·`commit -a`·`reset --hard`·`clean`은 쓰지 않는다.
- commit·push는 매번 사용자 확인을 받은 뒤에 한다.
- 커밋 메시지는 `feat(terrain): …한다` 형식이고, 끝에 `Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>`를 붙인다.
- 루트의 미추적 `zetin`과 `docs/plans/2026-10-15-mks-can-patch4.md`는 커밋에 넣지 않는다.
- `feat/l515-aligned-depth-slam` 브랜치는 건드리지 않는다.

**시험 실행**
- 호스트 pytest는 `PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros"`를 붙인다.
- conda: `/home/light/anaconda3/bin/python`.
- 콘솔: `/usr/bin/python3`. GTK 시험은 `env -u WAYLAND_DISPLAY GDK_BACKEND=x11 xvfb-run -a`로 돌린다.

**스펙과 다르게 정한 점** (근거와 함께, 스펙 개정은 결과 보고서에서 함께 한다)
- `terrain_overlay/camera.py`(`CameraModel`)와 `terrain_overlay/runtime.py`(ROS 없는 틱 논리)를 추가했다. 노드 틱 시험 대부분을 rclpy 없이 호스트에서 돌리기 위해서다.
- depth 구독 QoS를 best effort·volatile에 **KEEP_LAST 1**로 정했다. 기본 깊이 5에서는 처리가 느릴 때 쌓인 프레임이 늦게 "새로 수신"되어 stale 판정이 1.2 s로 늦어졌다(2026-10-17 시제작 실측).
- 노드는 **2-스레드 `MultiThreadedExecutor`**로 돈다. depth 구독은 자기 콜백 그룹에, CameraInfo 두 구독과 타이머는 같은 그룹에 둔다. 처리 중인 틱이 depth 수신 시각을 늦추지 않게 하기 위해서다(Codex 1회차 P2-3).
- 런타임은 시계를 직접 읽는다(`clock` 주입). 처리 뒤 시각을 다시 읽어 stale·만료·두 age를 **송신 시각** 기준으로 정한다(Codex 1회차 P2-2, 스펙 §4.1-4의 "송신 직전" 규칙). 처리시간(`proc_ms`)도 같은 시계로 잰다.
- stale 계약식에 최대 처리시간 항을 넣는다(위 "판정 파라미터"). 스펙 §4.1의 0.65 s는 처리시간을 0으로 본 값이다. 스펙 개정 대상은 §4.1·§6 오류표·§7.2 스모크·§11 재계산의 같은 수치다(Task 13 Step 8).
- compose 서비스를 opt-in 프로필 `terrain-overlay`로 둔다. 기본 기동 목록에서 빼는 것을 구조로 보장하기 위해서다.
- 윤곽을 숨겼을 때 콘솔 안내문에 이유(영상 끊김, 영상 크기 미확인/불일치)를 붙인다.
- 지형 캔버스는 영상 대기 화면보다 **위** 레이어에 둔다. 그래서 배지가 영상 연결 전·끊김 중에도 보이고, 그때 윤곽은 숨긴다(Codex P2-6). 입력은 아래로 통과시킨다.
- 글자는 PangoCairo로 그린다. cairo 기본 텍스트 API는 한글 대체 글꼴이 없어 빈 네모로 나온다(Codex P2-5, 재현함).
- 수신기는 `latest`·`display_frame`·`last_rx`를 한 락에서 읽는 `snapshot()`을 둔다(Codex P2-4).
- 빈 경로의 배지 문구는 "경로 돌출 장애물 없음"으로 한다. 구덩이가 없다는 뜻으로 읽히지 않게 하기 위해서다(agy).
- 콘솔 수신 스레드는 예외가 나도 살아 있게 한다. 패킷 파싱은 `RecursionError`·`OverflowError`와 잘못된 타입도 `ValueError`로 거부한다(Codex P2-9).
- 롤백: 모든 변경은 `feat/l515-terrain-overlay`에만 있고 main 병합 전이다. 기능을 버리면 병합하지 않거나 병합 커밋을 되돌린다. `lidar_dev` 구 파일은 git 이력에 남는다.

## Review Focus

스펙이 직접 시험하지 않지만, 사용자가 가장 먼저 겪을 가능성이 큰 입력 다섯 가지다. 가능성이 높은 순서다. 각 항목의 시험은 해당 Task에 넣었다.

1. **처리가 틱보다 느릴 때**(Jetson 처리 80–100 ms 예상). 기대 동작: 쌓인 depth가 새 프레임처럼 기록되지 않고, 처리 중에 온 depth도 그 즉시 수신 시각이 찍힌다. depth가 끊기면 `0.65 s + 최대 처리시간` 안에 `DEPTH_STALE`이 나가고, 한 틱을 넘는 처리(예외 포함)는 모두 기록된다. 시험: Task 4 (다)·처리시간 시험, Task 5 `test_depth_is_stamped_while_a_tick_is_still_processing`(결정적, 음성 대조로 확인)·`test_slow_processing_keeps_the_stale_contract`.
2. **세션 파일이 없거나 만료됐을 때**(인증된 콘솔 없음). 기대 동작: 노드가 죽지 않고 아무것도 보내지 않다가, 세션이 생기면 바로 송신을 재개한다. 시험: Task 5 `test_missing_session_file_sends_nothing_and_keeps_running`.
3. **SRT 영상 크기를 아직 모르거나 1280×720이 아닐 때.** 배지는 "지형 OK"인데 윤곽이 말없이 사라지면 운용자는 장애물이 없다고 오해한다. 기대 동작: 안내문에 "윤곽 숨김: …" 이유를 표시한다. 시험: Task 6 `test_hidden_outlines_explain_why_in_the_note`.
4. **:5009로 깨진 패킷이 올 때**(악의적 패킷, 다른 송신기의 오설정). 기대 동작: 패킷만 거부하고 무효 카운트를 올리며, 수신 스레드는 계속 산다. 시험: Task 6 `test_nested_or_oversize_datagrams_are_invalid`, `test_malformed_and_wrong_type_packets_count_as_invalid`.
5. **같은 세션의 송신 주소가 바뀔 때**(DHCP 갱신, 경로 변경). 기대 동작: sequence 하한을 유지하므로 옛 패킷이 되살아나지 않는다. 시험: Task 6 `test_address_move_keeps_the_sequence_floor`(실제 소켓 두 개).

---

## 파일 지도

| 파일 | 책임 | Task |
|---|---|---|
| `terrain_overlay/__init__.py` | 패키지 표식 | 1 |
| `terrain_overlay/contract.py` | 포트·스키마 상수, `encode_packet`·`encoded_size`·`parse_packet`, `TerrainFrame` (stdlib) | 1 |
| `remote_video/contract.py` | `L515_TERRAIN_UDP_PORT = 5009` 추가 | 1 |
| `terrain_overlay/camera.py` | `CameraModel`(핀홀, CameraInfo `k`에서 생성) | 2 |
| `terrain_overlay/detector.py` | `TerrainDetector`·`TerrainResult`·`ground_axes` | 2 |
| `terrain_overlay/tests/scenes.py` | 합성 raw depth 장면 생성기 | 2 |
| `terrain_overlay/projection.py` | `DepthToColorProjector` | 3 |
| `terrain_overlay/contours.py` | 윤곽·장애물별 거리·크기 제한 | 3 |
| `terrain_overlay/runtime.py` | `RuntimeConfig`·`SenderSession`·`TerrainOverlayRuntime`(틱·만료·자기 검증) | 4 |
| `ros2/src/powertrain_ros/powertrain_ros/terrain_overlay_node.py` | ROS 어댑터 | 5 |
| `ros2/src/powertrain_ros/setup.py` | `terrain_overlay` 엔트리포인트 | 5 |
| `ros2/src/powertrain_ros/test/test_terrain_overlay_node.py` | 노드 시험(도메인 77) | 5 |
| `ros2/src/powertrain_ros/test/installed_terrain_overlay.py` | 설치 엔트리포인트 스모크 + 음성 대조 | 5 |
| `operator_console/terrain.py` | 세션 게이트·수신기·`terrain_view` | 6 |
| `operator_console/app.py` | `TerrainCanvas`, `VideoPanel` 다중 캔버스, 체크박스, 인자 | 7 |
| `operator_console/__main__.py`·`integrated_runtime_smoke.py`·`render_status_screenshot.py` | `terrain_port`·지연 설정 | 7 |
| `operator_console/runtime_smoke.py` | terrain 채널 주입·단언 | 8 |
| `operator_console/demo_source.py` | terrain 데모 패킷, `LIVE_PORTS`에 5009 | 8 |
| `lidar_dev/*` | 오프라인 도구를 새 코어로 이전, 구 파일 삭제, README | 9 |
| `docker/docker-compose.integrated.yml` | `powertrain_terrain_overlay`(profile `terrain-overlay`) | 10 |
| `scripts/tests/test_integrated_launchers.py` | compose 계약 시험 | 10 |
| `docs/integrated-operation.md`·`docs/network-ip-setup.md`·`operator_console/README.md` | 포트·설정 문서 | 10 |
| `docs/reports/2026-10-xx-l515-terrain-overlay.md` | 실기 결과 | 13 |
| `.claude/CLAUDE.md`·`AGENTS.md` | 구조·현재 상태 한 줄(검증 후) | 14 |

---

### Task 0: 착수 전 상태 확인

**Files:** 없음(확인만)

- [ ] **Step 1: 브랜치와 미추적 파일 확인**

Run: `git status -sb && git log --oneline -3`
Expected: `## feat/l515-terrain-overlay`, HEAD `78d14b1 docs(terrain): …`, 미추적 `docs/plans/2026-10-15-mks-can-patch4.md`·`zetin`만(이 계획 파일을 아직 커밋하지 않았다면 그것도).

- [ ] **Step 2: 원격·Jetson 동기 확인(프로젝트 §2 규칙)**

Run: `git fetch origin && git log --oneline HEAD..origin/main | head`
그 뒤 다음 명령으로 Jetson 쪽도 본다.
`SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh -o ConnectTimeout=5 jetson 'cd ~/power-train-sw && git status -sb | head -5 && git log --oneline -1'`
Jetson에 닿으면 생산 이미지 의존성도 지금 확인한다. 없으면 이미지 갱신을 Task 12보다 앞서 사용자와 정한다.
`SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh jetson 'docker run --rm --entrypoint python3 powertrain-sw:ros -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"'`
Expected:
- origin/main에 새 커밋이 있으면 이 브랜치를 rebase할지 사용자에게 묻는다.
- Jetson이 응답하지 않으면 "Jetson 미확인"으로 기록하고 Task 1–11은 계속한다(Task 12에서 다시 확인).

- [ ] **Step 3: 알려진 기존 실패 기준선 기록**

이 실패들은 지형 작업과 무관하며, 2026-10-17 HEAD에서 이미 실패한다. 이후 Task에서 "새 실패 없음"을 판정할 때 이 목록을 기준으로 삼는다.
- 콘솔 전체 스위트 10건:
  - `arm_ops/test_arm_ops_boundary.py` 2건과 `arm_ui/test_arm_ui_boundary.py` 1건. 옛 기능 브랜치의 기준 커밋 `2ffbad6`과 비교하는 시험이다.
  - `arm_ops` 계약·정책·세션·GTK 어댑터 6건.
  - `test_runtime_smoke.py::test_console_survives_live_data_on_every_channel` 1건. 원인은 `arm tabs did not render live tool in safe disabled state`이며, PR #8 이후 로봇팔 수동 탭 위젯이 활성 상태다.
- `scripts/tests/test_integrated_launchers.py` 2건: `test_robot_prepare_builds_once…`, `test_robot_prepare_reuses_existing_images…`.

Run: `env -u WAYLAND_DISPLAY GDK_BACKEND=x11 PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" xvfb-run -a /usr/bin/python3 -m pytest -q -p no:cacheprovider -W ignore operator_console 2>&1 | tail -1`
Expected: `10 failed, 672 passed, 3 skipped` (2026-10-17 실측). 지형 작업을 모두 넣은 뒤에는 같은 10건 실패에 `706 passed`가 된다(시제작 실측).

---

### Task 1: 패킷 계약 v1 (`terrain_overlay/contract.py`)

**Files:**
- Create: `terrain_overlay/__init__.py`, `terrain_overlay/contract.py`, `terrain_overlay/tests/__init__.py`(빈 파일), `terrain_overlay/tests/test_contract.py`
- Modify: `remote_video/contract.py` (`L515_TERRAIN_UDP_PORT`)

**Interfaces:**
- Produces:
  - `UDP_PORT = 5009`, `MAX_PACKET_BYTES = 1200`, `FRAME_SIZE = (1280, 720)`, `MAX_OBSTACLES = 4`, `MIN_POLY_POINTS = 3`, `MAX_POLY_POINTS = 16`, `CAMERA_HEIGHT_RANGE_M = (0.25, 0.80)`, `SCHEMA_VERSION`, `PACKET_TYPE`
  - `encode_packet(dict) -> bytes`(초과·NaN이면 `ValueError`), `encoded_size(dict) -> int`
  - `parse_packet(bytes, received_monotonic_s=None) -> TerrainFrame`(위반이면 `ValueError`)
  - `TerrainFrame`(필드는 §5.2와 같은 이름, `zones: dict`, `obstacles: tuple[TerrainObstacle]`, `ground: GroundInfo | None`, `received_monotonic_s`, 속성 `session_key`)
  - `TerrainObstacle(dist_m, in_path, poly: tuple[tuple[int,int]])`, `GroundInfo(state, age_ms, camera_height_m, camera_tilt_deg)`
  - `remote_video.contract.L515_TERRAIN_UDP_PORT`

- [ ] **Step 1: 실패하는 시험 작성** — `terrain_overlay/tests/__init__.py`는 빈 파일로 만든다.

```python
"""Packet v1 contract — runs on both conda and the console's system python."""
import json

import pytest

from terrain_overlay import contract
from terrain_overlay.contract import encode_packet, parse_packet


def ok_payload(**overrides):
    payload = {
        "schema_version": 1,
        "type": "l515_terrain",
        "sender_boot_id": "0082f17fdc0a49969ff0fe1dc4979560",
        "sender_start_mono_ns": 707316067583855,
        "sender_instance": "3fa95c0e81d2b7a4",
        "sequence": 123,
        "sent_unix_ns": 1760000000000000000,
        "depth_stamp_ns": 1760000000000000000,
        "depth_age_ms": 35.2,
        "status": "OK",
        "reason": None,
        "frame_size": [1280, 720],
        "distance_origin": "l515_depth_optical",
        "alignment_verified": False,
        "ground": {
            "state": "FRESH", "age_ms": 35.2,
            "camera_height_m": 0.512, "camera_tilt_deg": 21.4,
        },
        "path_half_width_m": 0.4445,
        "zones": {"left": None, "center": 1.23, "right": 2.10},
        "obstacles": [{
            "dist_m": 1.23, "in_path": True,
            "poly": [[612, 410], [700, 410], [700, 520], [612, 520]],
        }],
        "proc_ms": 18.4,
    }
    payload.update(overrides)
    return payload


def error_payload(**overrides):
    fields = {
        "status": "SENSOR_ERROR", "reason": "DEPTH_STALE", "ground": None,
        "zones": {"left": None, "center": None, "right": None},
        "obstacles": [], "proc_ms": None,
    }
    fields.update(overrides)
    return ok_payload(**fields)


def raw(payload):
    return json.dumps(payload).encode("utf-8")


def test_spec_example_round_trips_and_fits_the_size_limit():
    encoded = encode_packet(ok_payload())
    assert len(encoded) <= contract.MAX_PACKET_BYTES
    frame = parse_packet(encoded, received_monotonic_s=12.5)
    assert frame.status == "OK"
    assert frame.ground.state == "FRESH"
    assert frame.ground.age_ms == pytest.approx(35.2)
    assert frame.zones == {"left": None, "center": 1.23, "right": 2.10}
    assert frame.obstacles[0].poly[0] == (612, 410)
    assert frame.obstacles[0].in_path is True
    assert frame.session_key == (
        "0082f17fdc0a49969ff0fe1dc4979560", 707316067583855, "3fa95c0e81d2b7a4",
    )
    assert frame.received_monotonic_s == 12.5


def test_error_packet_round_trips():
    frame = parse_packet(encode_packet(error_payload()))
    assert (frame.status, frame.reason, frame.ground, frame.obstacles) == (
        "SENSOR_ERROR", "DEPTH_STALE", None, (),
    )


def test_unknown_top_level_fields_are_ignored():
    assert parse_packet(raw(ok_payload(future_field={"x": 1}))).status == "OK"


def test_encode_refuses_nan_and_oversize():
    with pytest.raises(ValueError):
        encode_packet(ok_payload(depth_age_ms=float("nan")))
    with pytest.raises(ValueError, match="exceeds"):
        encode_packet(ok_payload(padding="x" * 1300))


@pytest.mark.parametrize("overrides", [
    {"schema_version": 2},
    {"schema_version": True},
    {"schema_version": 1.0},
    {"status": []},
    {"status": {"OK": 1}},
    {"reason": ["x"]},
    {"distance_origin": []},
    {"depth_age_ms": 10**400},
    {"path_half_width_m": 10**400},
    {"zones": {"left": None, "center": 10**400, "right": None}},
    {"type": "yolo"},
    {"sender_boot_id": "0082F17FDC0A49969FF0FE1DC4979560"},
    {"sender_boot_id": "0082f17f-dc0a-4996-9ff0-fe1dc4979560"},
    {"sender_instance": "3fa95c0e81d2b7a"},
    {"sender_start_mono_ns": -1},
    {"sender_start_mono_ns": True},
    {"sequence": 1.0},
    {"sequence": False},
    {"sent_unix_ns": None},
    {"depth_stamp_ns": -5},
    {"depth_age_ms": -0.1},
    {"status": "WARN"},
    {"reason": "DEPTH_STALE"},
    {"frame_size": [848, 480]},
    {"frame_size": [1280.0, 720]},
    {"distance_origin": "base_link"},
    {"alignment_verified": 0},
    {"path_half_width_m": 0},
    {"path_half_width_m": True},
    {"zones": {"left": None, "center": 1.0}},
    {"zones": {"left": None, "center": -1.0, "right": None}},
    {"proc_ms": -1},
    {"ground": None},
    {"ground": {"state": "STALE", "age_ms": 1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 20.0}},
    {"ground": {"state": "HELD", "age_ms": 1.0,
                "camera_height_m": 0.25, "camera_tilt_deg": 20.0}},
    {"ground": {"state": "HELD", "age_ms": 1.0,
                "camera_height_m": 0.80, "camera_tilt_deg": 20.0}},
    {"ground": {"state": "HELD", "age_ms": 1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 40.1}},
    {"ground": {"state": "HELD", "age_ms": -1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 20.0}},
    {"obstacles": [{"dist_m": 1.0, "in_path": 1, "poly": [[0, 0], [1, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1, 0]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0]] * 17}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1280, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1, 720], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1.5, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [True, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": float("inf"), "in_path": True,
                    "poly": [[0, 0], [1, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True,
                    "poly": [[0, 0], [1, 0], [1, 1]]}] * 5},
])
def test_each_ok_rule_rejects_the_whole_packet(overrides):
    with pytest.raises(ValueError):
        parse_packet(raw(ok_payload(**overrides)))


@pytest.mark.parametrize("overrides", [
    {"reason": None},
    {"reason": "GROUND_EXPIRED"},
    {"ground": {"state": "FRESH", "age_ms": 1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 20.0}},
    {"zones": {"left": None, "center": 1.0, "right": None}},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1, 0], [1, 1]]}]},
])
def test_non_ok_status_requires_empty_results(overrides):
    with pytest.raises(ValueError):
        parse_packet(raw(error_payload(**overrides)))


def test_no_ground_reasons():
    for reason in ("GROUND_NOT_FOUND", "GROUND_EXPIRED"):
        payload = error_payload(reason=reason)
        payload["status"] = "NO_GROUND"
        assert parse_packet(raw(payload)).reason == reason
    payload = error_payload(reason="DEPTH_STALE")
    payload["status"] = "NO_GROUND"
    with pytest.raises(ValueError):
        parse_packet(raw(payload))


def test_garbage_and_oversize_raw_are_rejected():
    for blob in (b"\xff\xfe", b"[]", b"not json", b"{" + b" " * 1300 + b"}"):
        with pytest.raises(ValueError):
            parse_packet(blob)


def test_port_matches_the_remote_video_registry():
    from remote_video.contract import L515_TERRAIN_UDP_PORT
    assert contract.UDP_PORT == L515_TERRAIN_UDP_PORT == 5009
```

- [ ] **Step 2: 실패 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_contract.py`
Expected: FAIL. `ModuleNotFoundError: No module named 'terrain_overlay'`(패키지가 아직 없으므로).

- [ ] **Step 3: 구현**

`terrain_overlay/__init__.py`:

```python
"""L515 terrain obstacle overlay core — operator visual aid only.

Spec: docs/specs/2026-10-17-l515-terrain-overlay-design.md
"""
```

`terrain_overlay/contract.py`:

```python
"""L515 terrain overlay UDP packet v1 — encode and strict validation.

Standard library only: the operator console runs on the system python, which
has no numpy or cv2.  Every rule here mirrors the design spec §5.2
(docs/specs/2026-10-17-l515-terrain-overlay-design.md); a packet that breaks
any rule is rejected whole, never partially displayed.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Any

UDP_PORT = 5009
SCHEMA_VERSION = 1
PACKET_TYPE = "l515_terrain"
MAX_PACKET_BYTES = 1200
FRAME_SIZE = (1280, 720)
MAX_OBSTACLES = 4
MIN_POLY_POINTS = 3
MAX_POLY_POINTS = 16
CAMERA_HEIGHT_RANGE_M = (0.25, 0.80)  # open interval
MAX_CAMERA_TILT_DEG = 40.0

STATUS_REASONS: dict[str, tuple[str | None, ...]] = {
    "OK": (None,),
    "NO_GROUND": ("GROUND_NOT_FOUND", "GROUND_EXPIRED"),
    "SENSOR_ERROR": (
        "DEPTH_STALE",
        "DEPTH_EMPTY",
        "DEPTH_ENCODING",
        "DEPTH_SHAPE_MISMATCH",
        "COLOR_SHAPE_MISMATCH",
        "CAMERA_INFO_MISSING",
        "PROCESSING_ERROR",
    ),
}
DISTANCE_ORIGINS = ("l515_depth_optical", "rover_front")
GROUND_STATES = ("FRESH", "HELD")
ZONE_NAMES = ("left", "center", "right")

_BOOT_ID = re.compile(r"[0-9a-f]{32}")
_INSTANCE = re.compile(r"[0-9a-f]{16}")


@dataclass(frozen=True)
class GroundInfo:
    state: str
    age_ms: float
    camera_height_m: float
    camera_tilt_deg: float


@dataclass(frozen=True)
class TerrainObstacle:
    dist_m: float
    in_path: bool
    poly: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class TerrainFrame:
    sender_boot_id: str
    sender_start_mono_ns: int
    sender_instance: str
    sequence: int
    sent_unix_ns: int
    depth_stamp_ns: int | None
    depth_age_ms: float | None
    status: str
    reason: str | None
    frame_size: tuple[int, int]
    distance_origin: str
    alignment_verified: bool
    ground: GroundInfo | None
    path_half_width_m: float
    zones: dict[str, float | None]
    obstacles: tuple[TerrainObstacle, ...]
    proc_ms: float | None
    received_monotonic_s: float | None = None

    @property
    def session_key(self) -> tuple[str, int, str]:
        return (self.sender_boot_id, self.sender_start_mono_ns, self.sender_instance)


def _dumps(payload: dict[str, Any]) -> bytes:
    return json.dumps(
        payload, separators=(",", ":"), ensure_ascii=False, allow_nan=False,
    ).encode("utf-8")


def encoded_size(payload: dict[str, Any]) -> int:
    """Byte length the payload would have on the wire (no size check)."""
    return len(_dumps(payload))


def encode_packet(payload: dict[str, Any]) -> bytes:
    """Compact UTF-8 JSON; NaN/Infinity and oversize packets are refused."""
    raw = _dumps(payload)
    if len(raw) > MAX_PACKET_BYTES:
        raise ValueError(f"terrain packet {len(raw)} B exceeds {MAX_PACKET_BYTES} B")
    return raw


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _nonneg_int(value: Any, name: str) -> int:
    if not _is_int(value) or value < 0:
        raise ValueError(f"{name}: non-negative integer required")
    return value


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: number required")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name}: finite number required") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name}: finite number required")
    return number


def _nonneg_finite(value: Any, name: str) -> float:
    number = _finite(value, name)
    if number < 0.0:
        raise ValueError(f"{name}: must be >= 0")
    return number


def _optional(value: Any, parse, name: str):
    return None if value is None else parse(value, name)


def _parse_ground(value: Any) -> GroundInfo:
    if not isinstance(value, dict):
        raise ValueError("ground: object required")
    state = value.get("state")
    if not isinstance(state, str) or state not in GROUND_STATES:
        raise ValueError("ground.state invalid")
    height = _finite(value.get("camera_height_m"), "ground.camera_height_m")
    low, high = CAMERA_HEIGHT_RANGE_M
    if not low < height < high:
        raise ValueError("ground.camera_height_m out of range")
    tilt = _finite(value.get("camera_tilt_deg"), "ground.camera_tilt_deg")
    if not 0.0 <= tilt <= MAX_CAMERA_TILT_DEG:
        raise ValueError("ground.camera_tilt_deg out of range")
    return GroundInfo(
        state=state,
        age_ms=_nonneg_finite(value.get("age_ms"), "ground.age_ms"),
        camera_height_m=height,
        camera_tilt_deg=tilt,
    )


def _parse_zones(value: Any) -> dict[str, float | None]:
    if not isinstance(value, dict) or any(name not in value for name in ZONE_NAMES):
        raise ValueError("zones: left/center/right required")
    return {
        name: _optional(value[name], _nonneg_finite, f"zones.{name}")
        for name in ZONE_NAMES
    }


def _parse_obstacle(value: Any) -> TerrainObstacle:
    if not isinstance(value, dict):
        raise ValueError("obstacle: object required")
    in_path = value.get("in_path")
    if not isinstance(in_path, bool):
        raise ValueError("obstacle.in_path: bool required")
    poly = value.get("poly")
    if (
        not isinstance(poly, list)
        or not MIN_POLY_POINTS <= len(poly) <= MAX_POLY_POINTS
    ):
        raise ValueError("obstacle.poly: 3-16 points required")
    width, height = FRAME_SIZE
    points = []
    for point in poly:
        if (
            not isinstance(point, list)
            or len(point) != 2
            or not all(_is_int(c) for c in point)
            or not 0 <= point[0] < width
            or not 0 <= point[1] < height
        ):
            raise ValueError("obstacle.poly: integer [x, y] inside the frame required")
        points.append((point[0], point[1]))
    return TerrainObstacle(
        dist_m=_nonneg_finite(value.get("dist_m"), "obstacle.dist_m"),
        in_path=in_path,
        poly=tuple(points),
    )


def parse_packet(raw: bytes, received_monotonic_s: float | None = None) -> TerrainFrame:
    """Validate one datagram against packet v1; raise ValueError on any breach."""
    if len(raw) > MAX_PACKET_BYTES:
        raise ValueError("oversize terrain packet")
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("terrain packet is not UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("terrain packet must be an object")
    if not _is_int(payload.get("schema_version")) or payload["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unsupported schema_version")
    if payload.get("type") != PACKET_TYPE:
        raise ValueError("not an l515_terrain packet")
    boot_id = payload.get("sender_boot_id")
    if not isinstance(boot_id, str) or not _BOOT_ID.fullmatch(boot_id):
        raise ValueError("sender_boot_id: 32 lowercase hex required")
    instance = payload.get("sender_instance")
    if not isinstance(instance, str) or not _INSTANCE.fullmatch(instance):
        raise ValueError("sender_instance: 16 lowercase hex required")
    status = payload.get("status")
    if not isinstance(status, str) or status not in STATUS_REASONS:
        raise ValueError("status invalid")
    reason = payload.get("reason")
    if (reason is not None and not isinstance(reason, str)) or reason not in STATUS_REASONS[status]:
        raise ValueError(f"reason {reason!r} invalid for {status}")
    frame_size = payload.get("frame_size")
    if (
        not isinstance(frame_size, list)
        or not all(_is_int(value) for value in frame_size)
        or frame_size != list(FRAME_SIZE)
    ):
        raise ValueError("frame_size must be [1280, 720]")
    origin = payload.get("distance_origin")
    if not isinstance(origin, str) or origin not in DISTANCE_ORIGINS:
        raise ValueError("distance_origin invalid")
    alignment_verified = payload.get("alignment_verified")
    if not isinstance(alignment_verified, bool):
        raise ValueError("alignment_verified: bool required")
    half_width = _finite(payload.get("path_half_width_m"), "path_half_width_m")
    if half_width <= 0.0:
        raise ValueError("path_half_width_m must be > 0")
    zones = _parse_zones(payload.get("zones"))
    raw_obstacles = payload.get("obstacles")
    if not isinstance(raw_obstacles, list) or len(raw_obstacles) > MAX_OBSTACLES:
        raise ValueError("obstacles: list of at most 4 required")
    obstacles = tuple(_parse_obstacle(item) for item in raw_obstacles)
    raw_ground = payload.get("ground")
    if status == "OK":
        ground = _parse_ground(raw_ground)
    else:
        if raw_ground is not None:
            raise ValueError("ground must be null unless status is OK")
        if any(value is not None for value in zones.values()):
            raise ValueError("zones must be null unless status is OK")
        if obstacles:
            raise ValueError("obstacles must be empty unless status is OK")
        ground = None
    return TerrainFrame(
        sender_boot_id=boot_id,
        sender_start_mono_ns=_nonneg_int(
            payload.get("sender_start_mono_ns"), "sender_start_mono_ns",
        ),
        sender_instance=instance,
        sequence=_nonneg_int(payload.get("sequence"), "sequence"),
        sent_unix_ns=_nonneg_int(payload.get("sent_unix_ns"), "sent_unix_ns"),
        depth_stamp_ns=_optional(
            payload.get("depth_stamp_ns"), _nonneg_int, "depth_stamp_ns",
        ),
        depth_age_ms=_optional(
            payload.get("depth_age_ms"), _nonneg_finite, "depth_age_ms",
        ),
        status=status,
        reason=reason,
        frame_size=FRAME_SIZE,
        distance_origin=origin,
        alignment_verified=alignment_verified,
        ground=ground,
        path_half_width_m=half_width,
        zones=zones,
        obstacles=obstacles,
        proc_ms=_optional(payload.get("proc_ms"), _nonneg_finite, "proc_ms"),
        received_monotonic_s=received_monotonic_s,
    )
```

`remote_video/contract.py`:

```diff
--- a/remote_video/contract.py
+++ b/remote_video/contract.py
@@ -31,5 +31,9 @@
 ARM_TELEMETRY_SCHEMA_VERSION = 1
 MAX_ARM_TELEMETRY_BYTES = 4096
 
+# L515 terrain obstacle outlines (operator visual aid only). The console-side
+# terrain_overlay.contract keeps the same value as UDP_PORT; a test ties them.
+L515_TERRAIN_UDP_PORT = 5009
+
 L515_UNAVAILABLE_VERDICT = "REMOTE_DRIVE_VIDEO_UNAVAILABLE"
 D435I_UNAVAILABLE_VERDICT = "REMOTE_ARM_VIDEO_UNAVAILABLE"
```

- [ ] **Step 4: 통과 확인(두 인터프리터)**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_contract.py remote_video`
Expected: `89 passed` (계약 59 + remote_video 30)
Run: `/usr/bin/python3 -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_contract.py`
Expected: `59 passed` — 시스템 python(cv2 없음)에서도 import가 성공한다는 증거다.

- [ ] **Step 5: 커밋(사용자 확인 후)**

```bash
git add terrain_overlay/__init__.py terrain_overlay/contract.py terrain_overlay/tests/__init__.py terrain_overlay/tests/test_contract.py remote_video/contract.py
git commit -m "feat(terrain): L515 지형 오버레이 UDP :5009 패킷 계약 v1을 둔다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: 판정기 `TerrainDetector`와 합성 장면

**Files:**
- Create: `terrain_overlay/camera.py`, `terrain_overlay/detector.py`, `terrain_overlay/tests/scenes.py`, `terrain_overlay/tests/test_detector.py`

**Interfaces:**
- Consumes: 없음(numpy, cv2)
- Produces:
  - `CameraModel(fx, fy, cx, cy, width, height)`, `CameraModel.from_k(k, width, height)`
  - `DetectorConfig(depth_scale_m=0.00025, ground_hold_s=0.5, path_half_width_m=0.4445, front_offset_m=0.0)`
  - `TerrainDetector(camera, config).process(depth_uint16, frame_rx_s) -> TerrainResult`. dtype가 다르면 `TypeError`, 크기가 다르면 `ValueError`.
  - `TerrainResult` 필드: `status`, `reason`, `ground_state`, `fresh_rx_s`, `camera_height_m`, `camera_tilt_deg`, `zones`, `obstacle_points (N,3) float32`, `obstacle_fwd (N)`, `obstacle_lat (N)`, `floor_mask`, `obstacle_mask`
  - `ground_axes(normal) -> (forward, right)`
  - 시험용: `scenes.DEPTH`, `scenes.COLOR`, `Box`, `pitched_normal`, `render_depth`, `default_scene`, `floor_only`

알고리즘은 스펙 §4.2 1–9를 그대로 따른다. 확인할 점:
- 평면 만료는 피팅 **전에** 적용한다.
- 평활은 없다.
- voxel 키는 정수 격자를 하나의 int64로 묶어 `np.unique`한다.
- 채점 표본은 최대 3000점이고, 고정 시드 `20261017`로 1000개 3점 조합을 100개씩 묶어 채점한다.
- 재피팅은 3×3 공분산 `eigh`로 한다.

- [ ] **Step 1: 합성 장면 생성기와 실패하는 시험 작성**

`terrain_overlay/tests/scenes.py`:

```python
"""Synthetic raw L515 depth scenes for detector/contour tests.

Geometry is defined physically: ``normal`` is the unit up-normal of the floor
in the depth optical frame (x right, y down, z forward) and the camera sits
``height_m`` above the floor, so floor points satisfy ``P·normal = -height_m``.
Boxes are vertical faces standing on the floor, placed ``fwd_m`` ahead along
the floor-projected optical axis and ``lat_m`` to the right of it.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import ground_axes

DEPTH = CameraModel(fx=460.0, fy=460.0, cx=320.0, cy=240.0, width=640, height=480)
COLOR = CameraModel(fx=910.0, fy=910.0, cx=640.0, cy=360.0, width=1280, height=720)
SCALE_M = 0.00025


@dataclass(frozen=True)
class Box:
    fwd_m: float
    lat_m: float
    width_m: float
    height_m: float


def pitched_normal(tilt_deg: float) -> np.ndarray:
    """Up-normal for a camera pitched down by ``tilt_deg`` with no roll."""
    tilt = math.radians(tilt_deg)
    return np.array([0.0, -math.cos(tilt), -math.sin(tilt)])


def render_depth(
    normal, height_m: float, boxes=(), camera: CameraModel = DEPTH,
    max_range_m: float = 6.0,
) -> np.ndarray:
    normal = np.asarray(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    forward, right = ground_axes(normal)
    v, u = np.mgrid[0:camera.height, 0:camera.width].astype(np.float64)
    rays = np.stack(((u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy,
                     np.ones_like(u)), axis=-1)
    along_normal = rays @ normal
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(along_normal < -1e-6, -height_m / along_normal, np.inf)
        along_forward = rays @ forward
        for box in boxes:
            t = np.where(along_forward > 1e-6, box.fwd_m / along_forward, np.inf)
            lat = t * (rays @ right)
            above = t * along_normal + height_m
            face = (
                np.isfinite(t)
                & (np.abs(lat - box.lat_m) <= box.width_m / 2.0)
                & (above >= 0.0) & (above <= box.height_m)
                & (t < z)
            )
            z = np.where(face, t, z)
    z[~np.isfinite(z) | (z > max_range_m)] = 0.0
    return np.round(z / SCALE_M).astype(np.uint16)


def default_scene() -> np.ndarray:
    """Spec §7.1: camera 0.5 m high, 20° down, 0.2 m box 1.2 m ahead."""
    return render_depth(pitched_normal(20.0), 0.5, [Box(1.2, 0.0, 0.3, 0.2)])


def floor_only(tilt_deg: float = 20.0, height_m: float = 0.5) -> np.ndarray:
    return render_depth(pitched_normal(tilt_deg), height_m)
```

`terrain_overlay/tests/test_detector.py`:

```python
"""TerrainDetector on synthetic raw depth (spec §4.2, §7.1)."""
import tracemalloc

import numpy as np
import pytest

from terrain_overlay import detector as detector_module
from terrain_overlay.detector import DetectorConfig, TerrainDetector
from terrain_overlay.tests.scenes import (
    DEPTH, Box, default_scene, floor_only, pitched_normal, render_depth,
)

ROLL_NORMAL = (0.55, -0.779423, 0.30)  # Codex 2026-10-17 counterexample, tilt 38.79°


def wall(distance_m=1.0):
    """Fully valid depth with no floor-like plane (fit always fails)."""
    return np.full((DEPTH.height, DEPTH.width), round(distance_m / 0.00025), np.uint16)


def assert_default_scene_ok(result):
    assert result.status == "OK"
    assert result.ground_state == "FRESH"
    assert result.camera_height_m == pytest.approx(0.5, abs=0.01)
    assert result.camera_tilt_deg == pytest.approx(20.0, abs=0.5)
    assert result.zones["center"] == pytest.approx(1.20, abs=0.03)
    assert result.zones["left"] is None and result.zones["right"] is None
    assert len(result.obstacle_fwd) > 0
    assert np.all(np.abs(result.obstacle_lat) <= 0.4445)


def test_default_scene_finds_the_box_in_the_center_zone():
    assert_default_scene_ok(TerrainDetector(DEPTH).process(default_scene(), 0.0))


def test_negative_control_double_scale_bug_fails_the_scene_check():
    # Re-inject the PR #13 defect: metres already scaled, scaled again.
    buggy = TerrainDetector(DEPTH, DetectorConfig(depth_scale_m=0.00025 * 0.00025))
    with pytest.raises(AssertionError):
        assert_default_scene_ok(buggy.process(default_scene(), 0.0))


def test_metre_arrays_are_refused():
    metres = default_scene().astype(np.float32) * 0.00025
    with pytest.raises(TypeError):
        TerrainDetector(DEPTH).process(metres, 0.0)


def test_shape_mismatch_is_refused():
    with pytest.raises(ValueError):
        TerrainDetector(DEPTH).process(np.zeros((240, 320), np.uint16), 0.0)


def test_empty_depth_is_a_sensor_error():
    depth = np.zeros((DEPTH.height, DEPTH.width), np.uint16)
    depth[:10, :] = 4000  # 2 % valid
    result = TerrainDetector(DEPTH).process(depth, 0.0)
    assert (result.status, result.reason) == ("SENSOR_ERROR", "DEPTH_EMPTY")


def test_rolled_camera_keeps_a_straight_ahead_obstacle_in_the_center():
    depth = render_depth(ROLL_NORMAL, 0.5, [Box(2.7, 0.0, 0.3, 0.4)])
    result = TerrainDetector(DEPTH).process(depth, 0.0)
    assert result.status == "OK"
    assert result.camera_tilt_deg == pytest.approx(38.79, abs=0.3)
    assert result.zones["center"] == pytest.approx(2.70, abs=0.05)
    assert result.zones["left"] is None and result.zones["right"] is None


def test_plane_lifetime_follows_receive_time():
    detector = TerrainDetector(DEPTH)
    assert detector.process(floor_only(), 0.0).ground_state == "FRESH"
    held = detector.process(wall(), 0.3)
    assert (held.status, held.ground_state, held.fresh_rx_s) == ("OK", "HELD", 0.0)
    expired = detector.process(wall(), 0.6)
    assert (expired.status, expired.reason) == ("NO_GROUND", "GROUND_EXPIRED")
    still = detector.process(wall(), 0.7)
    assert (still.status, still.reason) == ("NO_GROUND", "GROUND_EXPIRED")
    assert detector.process(floor_only(), 0.8).ground_state == "FRESH"
    again = detector.process(wall(), 1.0)
    assert (again.ground_state, again.fresh_rx_s) == ("HELD", 0.8)


def test_startup_failure_is_not_found():
    result = TerrainDetector(DEPTH).process(wall(), 0.0)
    assert (result.status, result.reason) == ("NO_GROUND", "GROUND_NOT_FOUND")


def test_gap_without_process_calls_still_expires():
    detector = TerrainDetector(DEPTH)
    detector.process(floor_only(), 0.0)
    result = detector.process(wall(), 2.0)
    assert (result.status, result.reason) == ("NO_GROUND", "GROUND_EXPIRED")


def test_first_fresh_after_expiry_uses_only_the_new_plane():
    detector = TerrainDetector(DEPTH)
    assert detector.process(floor_only(tilt_deg=0.0), 0.0).ground_state == "FRESH"
    result = detector.process(floor_only(tilt_deg=30.0), 3.0)
    assert result.ground_state == "FRESH"
    assert result.camera_tilt_deg == pytest.approx(30.0, abs=0.5)
    assert result.camera_height_m == pytest.approx(0.5, abs=0.01)
    assert all(value is None for value in result.zones.values())  # floor is not an obstacle


def test_ransac_is_deterministic():
    first = TerrainDetector(DEPTH).process(default_scene(), 0.0)
    second = TerrainDetector(DEPTH).process(default_scene(), 0.0)
    assert first.camera_height_m == second.camera_height_m
    assert first.camera_tilt_deg == second.camera_tilt_deg


@pytest.mark.parametrize("depth", [floor_only(), floor_only(35.0, 0.79)],
                         ids=["lower-half-floor", "fully-valid-frame"])
def test_processing_memory_stays_bounded(depth):
    tracemalloc.start()
    try:
        TerrainDetector(DEPTH).process(depth, 0.0)
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert peak < 50 * 1024 * 1024


@pytest.mark.parametrize("height_m, tilt_deg, accepted", [
    (0.26, 20.0, True), (0.24, 20.0, False),
    (0.79, 20.0, True), (0.81, 20.0, False),
    (0.5, 39.0, True), (0.5, 41.0, False),
])
def test_plane_acceptance_bounds(height_m, tilt_deg, accepted):
    result = TerrainDetector(DEPTH).process(floor_only(tilt_deg, height_m), 0.0)
    assert (result.status == "OK") is accepted


def test_front_offset_shifts_zone_distance_and_floors_at_zero():
    shifted = TerrainDetector(DEPTH, DetectorConfig(front_offset_m=0.2)).process(default_scene(), 0.0)
    assert shifted.zones["center"] == pytest.approx(1.00, abs=0.03)
    clipped = TerrainDetector(DEPTH, DetectorConfig(front_offset_m=5.0)).process(default_scene(), 0.0)
    assert clipped.zones["center"] == 0.0


def test_axes_are_orthogonal_for_the_roll_counterexample():
    normal = np.asarray(ROLL_NORMAL) / np.linalg.norm(ROLL_NORMAL)
    forward, right = detector_module.ground_axes(normal)
    assert abs(forward @ right) < 1e-9
    assert abs(forward @ normal) < 1e-9 and abs(right @ normal) < 1e-9
    assert right[0] > 0
```

- [ ] **Step 2: 실패 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_detector.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'terrain_overlay.camera'`

- [ ] **Step 3: 구현**

`terrain_overlay/camera.py`:

```python
"""Pinhole camera model shared by the detector and the projector."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(v) for v in values) or self.fx <= 0 or self.fy <= 0:
            raise ValueError("camera intrinsics must be finite with positive focal length")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera size must be positive")

    @classmethod
    def from_k(cls, k, width: int, height: int) -> "CameraModel":
        """Build from a row-major 3x3 ROS CameraInfo ``k``."""
        return cls(
            fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]),
            width=int(width), height=int(height),
        )
```

`terrain_overlay/detector.py`:

```python
"""Ground plane + obstacle classification from raw L515 depth (spec §4.2).

Operator visual aid only — never a control or safety input.  Every plane
lifetime decision uses ``frame_rx_s`` (the depth's receive monotonic time),
never the processing time.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from .camera import CameraModel

MIN_VALID_FRACTION = 0.05
VOXEL_M = 0.02
MIN_GROUND_POINTS = 200
RANSAC_SAMPLE_MAX = 3000
RANSAC_ITERATIONS = 1000
RANSAC_CHUNK = 100
RANSAC_INLIER_M = 0.02
RANSAC_SEED = 20261017
MAX_TILT_DEG = 40.0
CAMERA_HEIGHT_RANGE_M = (0.25, 0.80)  # open interval
FLOOR_BAND_M = 0.03
OBSTACLE_MIN_H_M = 0.05
OBSTACLE_MAX_H_M = 2.0
OBSTACLE_MAX_FWD_M = 3.0
OPEN_KERNEL = np.ones((5, 5), np.uint8)
ZONE_PERCENTILE = 2.0


@dataclass(frozen=True)
class DetectorConfig:
    depth_scale_m: float = 0.00025
    ground_hold_s: float = 0.5
    path_half_width_m: float = 0.4445
    front_offset_m: float = 0.0


@dataclass
class TerrainResult:
    status: str  # OK / NO_GROUND / SENSOR_ERROR
    reason: str | None
    ground_state: str | None = None  # FRESH / HELD when status is OK
    fresh_rx_s: float | None = None
    camera_height_m: float | None = None
    camera_tilt_deg: float | None = None
    zones: dict[str, float | None] = field(
        default_factory=lambda: {"left": None, "center": None, "right": None},
    )
    obstacle_points: np.ndarray = field(
        default_factory=lambda: np.zeros((0, 3), np.float32),
    )
    obstacle_fwd: np.ndarray = field(default_factory=lambda: np.zeros(0, np.float32))
    obstacle_lat: np.ndarray = field(default_factory=lambda: np.zeros(0, np.float32))
    floor_mask: np.ndarray | None = None
    obstacle_mask: np.ndarray | None = None


def tilt_deg(normal: np.ndarray) -> float:
    return math.degrees(math.acos(min(1.0, abs(float(normal[1])))))


def ground_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Orthogonal forward/right axes lying in the ground plane (spec §4.2-7)."""
    z_axis = np.array([0.0, 0.0, 1.0])
    forward = z_axis - float(z_axis @ normal) * normal
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, normal)
    right /= np.linalg.norm(right)
    if right[0] < 0.0:
        right = -right
    return forward, right


class TerrainDetector:
    def __init__(self, depth_camera: CameraModel, config: DetectorConfig | None = None) -> None:
        self.camera = depth_camera
        self.config = config or DetectorConfig()
        h, w = depth_camera.height, depth_camera.width
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        self._xn = (u - depth_camera.cx) / depth_camera.fx
        self._yn = (v - depth_camera.cy) / depth_camera.fy
        self._lower_half = v > h * 0.5
        self._normal: np.ndarray | None = None
        self._offset: float | None = None
        self._fresh_rx_s: float | None = None
        self._expired = False

    def process(self, depth: np.ndarray, frame_rx_s: float) -> TerrainResult:
        if not isinstance(depth, np.ndarray) or depth.dtype != np.uint16:
            raise TypeError("depth must be a raw uint16 array (not metres)")
        if depth.shape != (self.camera.height, self.camera.width):
            raise ValueError(f"depth shape {depth.shape} does not match camera info")
        t = float(frame_rx_s)
        z = depth.astype(np.float32) * np.float32(self.config.depth_scale_m)
        valid = z > 0.0
        if valid.mean() < MIN_VALID_FRACTION:
            return TerrainResult("SENSOR_ERROR", "DEPTH_EMPTY")
        points = np.stack((self._xn * z, self._yn * z, z), axis=-1)

        if (
            self._normal is not None
            and t - self._fresh_rx_s > self.config.ground_hold_s
        ):
            self._normal = self._offset = None
            self._expired = True

        fitted = self._fit_ground(points[valid & self._lower_half])
        if fitted is not None:
            self._normal, self._offset = fitted
            self._fresh_rx_s = t
            self._expired = False
            state = "FRESH"
        elif self._normal is not None:
            state = "HELD"
        else:
            reason = "GROUND_EXPIRED" if self._expired else "GROUND_NOT_FOUND"
            return TerrainResult("NO_GROUND", reason)
        return self._classify(points, valid, state)

    def _fit_ground(self, candidates: np.ndarray) -> tuple[np.ndarray, float] | None:
        if len(candidates) < MIN_GROUND_POINTS:
            return None
        keys = np.floor(candidates / VOXEL_M).astype(np.int64) + (1 << 20)
        packed = (keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]
        _, first = np.unique(packed, return_index=True)
        voxels = candidates[np.sort(first)].astype(np.float64)
        if len(voxels) < MIN_GROUND_POINTS:
            return None
        rng = np.random.default_rng(RANSAC_SEED)
        if len(voxels) > RANSAC_SAMPLE_MAX:
            sample = voxels[rng.choice(len(voxels), RANSAC_SAMPLE_MAX, replace=False)]
        else:
            sample = voxels
        sample32 = sample.astype(np.float32)
        triplets = rng.integers(0, len(sample), size=(RANSAC_ITERATIONS, 3))
        best_count, best = -1, None
        for start in range(0, RANSAC_ITERATIONS, RANSAC_CHUNK):
            chosen = triplets[start:start + RANSAC_CHUNK]
            p0, p1, p2 = sample[chosen[:, 0]], sample[chosen[:, 1]], sample[chosen[:, 2]]
            normals = np.cross(p1 - p0, p2 - p0)
            norms = np.linalg.norm(normals, axis=1)
            usable = norms > 1e-9
            if not usable.any():
                continue
            normals = normals[usable] / norms[usable, None]
            offsets = -np.einsum("ij,ij->i", normals, p0[usable])
            distances = np.abs(sample32 @ normals.T.astype(np.float32) + offsets.astype(np.float32))
            counts = (distances < RANSAC_INLIER_M).sum(axis=0)
            index = int(np.argmax(counts))
            if counts[index] > best_count:
                best_count, best = int(counts[index]), (normals[index], offsets[index])
        if best is None:
            return None
        normal, offset = best
        inliers = voxels[np.abs(voxels @ normal + offset) < RANSAC_INLIER_M]
        if len(inliers) < 3:
            return None
        centroid = inliers.mean(axis=0)
        centered = inliers - centroid
        _, vectors = np.linalg.eigh(centered.T @ centered)
        normal = vectors[:, 0]
        offset = -float(normal @ centroid)
        if offset < 0.0:
            normal, offset = -normal, -offset
        low, high = CAMERA_HEIGHT_RANGE_M
        if tilt_deg(normal) > MAX_TILT_DEG or not low < offset < high:
            return None
        return normal, offset

    def _classify(self, points: np.ndarray, valid: np.ndarray, state: str) -> TerrainResult:
        normal = self._normal.astype(np.float32)
        forward, right = (axis.astype(np.float32) for axis in ground_axes(self._normal))
        height = points @ normal + np.float32(self._offset)
        fwd = points @ forward
        lat = points @ right
        floor = valid & (np.abs(height) < FLOOR_BAND_M)
        obstacle = (
            valid
            & (height > OBSTACLE_MIN_H_M) & (height < OBSTACLE_MAX_H_M)
            & (fwd > 0.0) & (fwd < OBSTACLE_MAX_FWD_M)
        )
        obstacle = cv2.morphologyEx(
            obstacle.astype(np.uint8), cv2.MORPH_OPEN, OPEN_KERNEL,
        ).astype(bool) & valid
        obstacle_fwd = fwd[obstacle]
        obstacle_lat = lat[obstacle]
        half_width = self.config.path_half_width_m
        bands = {
            "left": obstacle_lat < -half_width,
            "center": np.abs(obstacle_lat) <= half_width,
            "right": obstacle_lat > half_width,
        }
        zones: dict[str, float | None] = {}
        for name, band in bands.items():
            zones[name] = (
                max(0.0, float(np.percentile(obstacle_fwd[band], ZONE_PERCENTILE))
                    - self.config.front_offset_m)
                if band.any() else None
            )
        return TerrainResult(
            status="OK", reason=None, ground_state=state,
            fresh_rx_s=self._fresh_rx_s,
            camera_height_m=float(self._offset),
            camera_tilt_deg=tilt_deg(self._normal),
            zones=zones,
            obstacle_points=points[obstacle],
            obstacle_fwd=obstacle_fwd,
            obstacle_lat=obstacle_lat,
            floor_mask=floor,
            obstacle_mask=obstacle,
        )
```

- [ ] **Step 4: 통과 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_detector.py`
Expected: `21 passed`. 시제작 측정치:
- 기본 장면 처리 약 60 ms(x86).
- `tracemalloc` 최대 18.7 MB(< 50 MB).
- 롤 반례에서 중앙 2.70 m.

- [ ] **Step 5: 롤 시험이 옛 결함을 잡는지 1회 확인(커밋하지 않는 수동 음성 대조)**

Run:
```bash
/home/light/anaconda3/bin/python - <<'EOF'
import numpy as np, terrain_overlay.detector as det
from terrain_overlay.tests.scenes import DEPTH, Box, render_depth
def old_axes(n):
    z, x = np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
    f = z - (z @ n) * n; f /= np.linalg.norm(f); r = x - (x @ n) * n; r /= np.linalg.norm(r); return f, r
det.ground_axes = old_axes
print(det.TerrainDetector(DEPTH).process(render_depth((0.55, -0.779423, 0.30), 0.5, [Box(2.7, 0.0, 0.3, 0.4)]), 0.0).zones)
EOF
```
Expected: `left`가 None이 아니다(2026-10-17 실측 `{'left': 2.6998…, 'center': 2.6998…}`). 즉 옛 비직교 축이라면 `test_rolled_camera…`가 실패한다.

- [ ] **Step 6: 커밋(사용자 확인 후)**

```bash
git add terrain_overlay/camera.py terrain_overlay/detector.py terrain_overlay/tests/scenes.py terrain_overlay/tests/test_detector.py
git commit -m "feat(terrain): raw depth 바닥 평면·장애물 판정기를 numpy로 옮긴다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: 투영과 윤곽

**Files:**
- Create: `terrain_overlay/projection.py`, `terrain_overlay/contours.py`, `terrain_overlay/tests/test_projection_contours.py`

**Interfaces:**
- Consumes: `CameraModel`, `TerrainDetector`, `contract.MAX_OBSTACLES`·`MAX_PACKET_BYTES`·`MAX_POLY_POINTS`·`MIN_POLY_POINTS`·`encoded_size`
- Produces:
  - `DEFAULT_COLOR_OFFSET_M = (0.0, -0.024, 0.0)`
  - `DepthToColorProjector(color_camera, color_offset_m).project(points) -> (pixels int32[M,2], kept bool[N])`
  - `outline_contours(color_mask) -> list[contour]`
  - `approximate_polygon(contour, width, height) -> list[[x, y]]`
  - `build_obstacles(points, fwd, lat, projector, *, half_width_m, front_offset_m) -> list[{"dist_m", "in_path", "poly"}]`(가까운 순, 최대 4개, `dist_m`은 소수 2자리)
  - `fit_packet_size(payload, limit=1200) -> payload`

- [ ] **Step 1: 실패하는 시험 작성**

```python
"""Depth→colour projection and outline polygons (spec §4.3, §4.4)."""
import numpy as np
import pytest

from terrain_overlay.contract import MAX_PACKET_BYTES, encoded_size
from terrain_overlay.contours import (
    approximate_polygon, build_obstacles, fit_packet_size, outline_contours,
)
from terrain_overlay.detector import TerrainDetector
from terrain_overlay.projection import DepthToColorProjector
from terrain_overlay.tests.scenes import COLOR, DEPTH, Box, default_scene, pitched_normal, render_depth


def test_centre_point_lands_24mm_lower_in_colour():
    pixels, kept = DepthToColorProjector(COLOR).project(np.array([[0.0, 0.0, 1.0]]))
    assert kept.tolist() == [True]
    assert pixels[0, 0] == 640
    assert pixels[0, 1] == pytest.approx(360 + COLOR.fy * 0.024, abs=1.0)


def test_zero_depth_and_out_of_frame_points_are_dropped_without_error():
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [5.0, 0.0, 1.0], [0.1, 0.1, -1.0]])
    pixels, kept = DepthToColorProjector(COLOR).project(points)
    assert kept.tolist() == [True, False, False, False]
    assert len(pixels) == 1


def test_thin_long_outline_keeps_at_least_three_points():
    # 1100×8 px collapses to 2 points under approxPolyDP (2026-10-17 repro).
    mask = np.zeros((720, 1280), np.uint8)
    mask[300:308, 100:1200] = 255
    contours = outline_contours(mask)
    assert len(contours) == 1
    assert len(approximate_polygon(contours[0], 1280, 720)) == 4


def test_default_scene_produces_one_in_path_outline():
    result = TerrainDetector(DEPTH).process(default_scene(), 0.0)
    obstacles = build_obstacles(
        result.obstacle_points, result.obstacle_fwd, result.obstacle_lat,
        DepthToColorProjector(COLOR), half_width_m=0.4445, front_offset_m=0.0,
    )
    assert len(obstacles) == 1
    assert obstacles[0]["in_path"] is True
    assert obstacles[0]["dist_m"] == pytest.approx(1.20, abs=0.03)
    assert 3 <= len(obstacles[0]["poly"]) <= 16


def test_crowded_scene_is_bounded_and_nearest_first():
    boxes = [Box(f, lat, 0.15, 0.3) for f in (0.9, 1.4, 1.9, 2.4) for lat in (-1.0, -0.5, 0.0, 0.5, 1.0)]
    result = TerrainDetector(DEPTH).process(render_depth(pitched_normal(20.0), 0.5, boxes), 0.0)
    obstacles = build_obstacles(
        result.obstacle_points, result.obstacle_fwd, result.obstacle_lat,
        DepthToColorProjector(COLOR), half_width_m=0.4445, front_offset_m=0.0,
    )
    assert len(obstacles) == 4
    distances = [item["dist_m"] for item in obstacles]
    assert distances == sorted(distances)
    for item in obstacles:
        assert 3 <= len(item["poly"]) <= 16
        assert all(0 <= x < 1280 and 0 <= y < 720 for x, y in item["poly"])


def _dense_payload(count=4, points=16):
    star = [[1279 - i, 719 - i] for i in range(points)]
    return {
        "header": "x" * 400,
        "obstacles": [{"dist_m": 1.0 + i, "in_path": True, "poly": list(star)} for i in range(count)],
    }


def test_fit_packet_size_halves_dense_polygons_first():
    payload = _dense_payload()
    assert encoded_size(payload) > MAX_PACKET_BYTES
    fitted = fit_packet_size(payload)
    assert encoded_size(fitted) <= MAX_PACKET_BYTES
    assert len(fitted["obstacles"]) == 4
    assert all(len(item["poly"]) == 8 for item in fitted["obstacles"])


def test_fit_packet_size_then_drops_the_farthest():
    fitted = fit_packet_size(_dense_payload(), limit=700)
    assert encoded_size(fitted) <= 700
    assert [item["dist_m"] for item in fitted["obstacles"]] == [1.0, 2.0][: len(fitted["obstacles"])]
    assert len(fitted["obstacles"]) < 4
```

- [ ] **Step 2: 실패 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_projection_contours.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'terrain_overlay.contours'`

- [ ] **Step 3: 구현**

`terrain_overlay/projection.py`:

```python
"""Depth optical points → L515 colour pixels (spec §4.3).

Pinhole only, translation only.  Distortion and the depth→colour rotation are
deliberately ignored; §7.3-3 measures whether that is good enough and the
packet's ``alignment_verified`` flag tells the operator when it is not proven.
"""
from __future__ import annotations

import numpy as np

from .camera import CameraModel

DEFAULT_COLOR_OFFSET_M = (0.0, -0.024, 0.0)


class DepthToColorProjector:
    def __init__(self, color_camera: CameraModel, color_offset_m=DEFAULT_COLOR_OFFSET_M) -> None:
        offset = np.asarray(color_offset_m, dtype=np.float64)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("color_offset_m must be three finite numbers")
        self.camera = color_camera
        self._offset = offset.astype(np.float32)

    def project(self, points_depth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(pixels int32[M, 2], kept bool[N])`` for depth-frame points.

        Points with z <= 0 (no depth) or outside the colour frame are dropped;
        ``kept`` marks which input rows produced the returned pixels.
        """
        points = np.asarray(points_depth, dtype=np.float32).reshape(-1, 3)
        color = points - self._offset
        z = color[:, 2]
        kept = (points[:, 2] > 0.0) & (z > 0.0)
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.floor(self.camera.fx * color[:, 0] / z + self.camera.cx)
            v = np.floor(self.camera.fy * color[:, 1] / z + self.camera.cy)
        kept &= (
            np.isfinite(u) & np.isfinite(v)
            & (u >= 0) & (u < self.camera.width)
            & (v >= 0) & (v < self.camera.height)
        )
        pixels = np.stack((u[kept], v[kept]), axis=1).astype(np.int32)
        return pixels, kept
```

`terrain_overlay/contours.py`:

```python
"""Obstacle points → colour-frame outline polygons and distances (spec §4.4)."""
from __future__ import annotations

import cv2
import numpy as np

from .contract import MAX_OBSTACLES, MAX_PACKET_BYTES, MAX_POLY_POINTS, MIN_POLY_POINTS, encoded_size
from .projection import DepthToColorProjector

CLOSE_KERNEL = np.ones((9, 9), np.uint8)
DILATE_KERNEL = np.ones((5, 5), np.uint8)
MIN_AREA_PX2 = 150.0
EPSILON_FRACTION = 0.01
EPSILON_GROWTH = 1.5
DISTANCE_PERCENTILE = 5.0
THIN_POLY_POINTS = 6


def approximate_polygon(contour: np.ndarray, width: int, height: int) -> list[list[int]]:
    epsilon = EPSILON_FRACTION * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    while len(approx) > MAX_POLY_POINTS:
        epsilon *= EPSILON_GROWTH
        approx = cv2.approxPolyDP(contour, epsilon, True)
    points = approx.reshape(-1, 2)
    if len(points) < MIN_POLY_POINTS:
        # A long thin outline can collapse to two points; keep it drawable.
        points = cv2.boxPoints(cv2.minAreaRect(contour))
    points = np.rint(points).astype(np.int64)
    points[:, 0] = np.clip(points[:, 0], 0, width - 1)
    points[:, 1] = np.clip(points[:, 1], 0, height - 1)
    return [[int(x), int(y)] for x, y in points]


def outline_contours(color_mask: np.ndarray) -> list[np.ndarray]:
    """Close/dilate the sparse projected mask and return outer contours ≥ 150 px²."""
    mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    mask = cv2.dilate(mask, DILATE_KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) >= MIN_AREA_PX2]


def build_obstacles(
    points: np.ndarray, fwd: np.ndarray, lat: np.ndarray,
    projector: DepthToColorProjector, *, half_width_m: float, front_offset_m: float,
) -> list[dict]:
    """Return at most four ``{dist_m, in_path, poly}`` dicts, nearest first."""
    width, height = projector.camera.width, projector.camera.height
    pixels, kept = projector.project(points)
    if len(pixels) == 0:
        return []
    fwd, lat = np.asarray(fwd)[kept], np.asarray(lat)[kept]
    mask = np.zeros((height, width), np.uint8)
    mask[pixels[:, 1], pixels[:, 0]] = 255
    labels = np.zeros((height, width), np.uint16)
    contours = outline_contours(mask)
    for index, contour in enumerate(contours[: np.iinfo(np.uint16).max - 1], start=1):
        cv2.drawContours(labels, [contour], -1, color=index, thickness=cv2.FILLED)
    point_labels = labels[pixels[:, 1], pixels[:, 0]]
    obstacles = []
    for index, contour in enumerate(contours, start=1):
        inside = point_labels == index
        if not inside.any():
            continue
        distance = float(np.percentile(fwd[inside], DISTANCE_PERCENTILE)) - front_offset_m
        obstacles.append({
            "dist_m": round(max(0.0, distance), 2),
            "in_path": bool((np.abs(lat[inside]) <= half_width_m).any()),
            "poly": approximate_polygon(contour, width, height),
        })
    obstacles.sort(key=lambda item: item["dist_m"])
    return obstacles[:MAX_OBSTACLES]


def fit_packet_size(payload: dict, limit: int = MAX_PACKET_BYTES) -> dict:
    """Halve dense polygons, then drop the farthest obstacles until it fits."""
    if encoded_size(payload) <= limit:
        return payload
    obstacles = [
        dict(item, poly=item["poly"][::2] if len(item["poly"]) > THIN_POLY_POINTS else item["poly"])
        for item in payload["obstacles"]
    ]
    payload = dict(payload, obstacles=obstacles)
    while obstacles and encoded_size(payload) > limit:
        obstacles = obstacles[:-1]
        payload = dict(payload, obstacles=obstacles)
    return payload
```

- [ ] **Step 4: 통과 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_projection_contours.py`
Expected: `7 passed`. 기본 장면의 윤곽은 4점, `dist_m` 1.2, `in_path` True다.

- [ ] **Step 5: 커밋(사용자 확인 후)**

```bash
git add terrain_overlay/projection.py terrain_overlay/contours.py terrain_overlay/tests/test_projection_contours.py
git commit -m "feat(terrain): depth→컬러 투영과 장애물 윤곽·패킷 크기 제한을 둔다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: ROS 없는 송신 틱 논리 (`terrain_overlay/runtime.py`)

**Files:**
- Create: `terrain_overlay/runtime.py`, `terrain_overlay/tests/test_runtime.py`

**Interfaces:**
- Consumes: Task 1–3의 공개 이름 전부
- Produces:
  - `RuntimeConfig(publish_hz, depth_scale_m, depth_stale_s, ground_hold_s, color_offset_m, path_half_width_m, front_offset_m, alignment_verified)`(범위 위반이면 `ValueError`)
  - `SenderSession(boot_id, start_mono_ns, instance)`와 `SenderSession.create(boot_id_path=…, monotonic_ns=…, token_hex=…)`
  - `TerrainOverlayRuntime(config, session, *, log_error, unix_ns, clock=time.monotonic)`. 메서드: `set_depth_info(CameraModel)`, `set_color_info(CameraModel)`, `on_depth(array, stamp_ns, rx_s)`, `on_bad_depth(rx_s)`(이 둘은 다른 executor 스레드에서 불려도 안전), `tick() -> bytes | None`. 속성 `sequence`.

틱 판단 순서는 다음과 같다(스펙 §4.1 1–5, §4.4 자기 검증).
1. `CAMERA_INFO_MISSING`
2. `COLOR_SHAPE_MISMATCH`
3. `DEPTH_ENCODING`: 마지막 불량 프레임이 마지막 정상 프레임보다 새롭고, 수신 후 stale 시간 안일 때.
4. `DEPTH_STALE`
5. `DEPTH_SHAPE_MISMATCH`
6. 새 stamp면 처리하고, 아니면 캐시를 쓴다.
7. 송신 직전 만료 검사: `now − fresh_rx_s > ground_hold_s`이면 `NO_GROUND`/`GROUND_EXPIRED`.

`tick()`은 위 판단을 두 번 한다. 첫 번째는 새 depth를 처리해 캐시를 채우고, 처리 뒤 시계를 다시 읽은 두 번째가 송신할 상태·stale·age를 정한다. 두 번째는 캐시를 재사용하므로 다시 처리하지 않는다.

- [ ] **Step 1: 실패하는 시험 작성** — (가)·(나) 처리 지연 반례, (다) 처리 시간 자체가 만료를 넘기는 반례, stale 계약, 캐시 재송신, 자기 검증 실패 경로를 포함한다. 시각은 `FakeClock`으로 주고, 처리 시간은 판정기 호출이 가짜 시계를 전진시키는 방식으로 흉내 낸다.

```python
"""Sender tick logic without ROS (spec §4.1, §4.4, §7.1 node-tick cases)."""
import numpy as np
import pytest

from terrain_overlay import runtime as runtime_module
from terrain_overlay.camera import CameraModel
from terrain_overlay.contract import parse_packet
from terrain_overlay.runtime import RuntimeConfig, SenderSession, TerrainOverlayRuntime
from terrain_overlay.tests.scenes import COLOR, DEPTH, default_scene, floor_only

SESSION = SenderSession("0082f17fdc0a49969ff0fe1dc4979560", 1000, "3fa95c0e81d2b7a4")


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def wall():
    return np.full((DEPTH.height, DEPTH.width), 4000, np.uint16)


def ready(config=None, errors=None):
    rt = TerrainOverlayRuntime(
        config or RuntimeConfig(), SESSION,
        log_error=(errors.append if errors is not None else lambda _m: None),
        unix_ns=lambda: 1_760_000_000_000_000_000,
        clock=FakeClock(),
    )
    rt.set_depth_info(DEPTH)
    rt.set_color_info(COLOR)
    return rt


def frame(rt, now):
    rt._clock.t = now
    raw = rt.tick()
    assert raw is not None
    return parse_packet(raw)


def slow_processing(rt, monkeypatch, seconds):
    """Make every detector call advance the fake clock, like real CPU time."""
    original = rt._detector.process

    def process(*args):
        rt._clock.t += seconds
        return original(*args)

    monkeypatch.setattr(rt._detector, "process", process)


def test_ok_packet_for_the_default_scene():
    rt = ready()
    rt.on_depth(default_scene(), stamp_ns=111, rx_s=10.0)
    packet = frame(rt, 10.03)
    assert (packet.status, packet.reason) == ("OK", None)
    assert packet.ground.state == "FRESH"
    assert packet.ground.age_ms == pytest.approx(30.0, abs=0.1)
    assert packet.depth_age_ms == pytest.approx(30.0, abs=0.1)
    assert packet.depth_stamp_ns == 111
    assert packet.zones["center"] == pytest.approx(1.20, abs=0.03)
    assert len(packet.obstacles) == 1 and packet.obstacles[0].in_path
    assert packet.distance_origin == "l515_depth_optical"
    assert packet.alignment_verified is False
    assert packet.proc_ms is not None
    assert packet.session_key == ("0082f17fdc0a49969ff0fe1dc4979560", 1000, "3fa95c0e81d2b7a4")


def test_camera_info_and_color_size_gate_processing():
    rt = TerrainOverlayRuntime(RuntimeConfig(), SESSION, clock=FakeClock())
    rt.on_depth(default_scene(), 1, 0.0)
    assert frame(rt, 0.05).reason == "CAMERA_INFO_MISSING"
    rt.set_depth_info(DEPTH)
    rt.set_color_info(CameraModel(615.0, 615.0, 320.0, 240.0, 640, 480))
    assert frame(rt, 0.06).reason == "COLOR_SHAPE_MISMATCH"
    rt.set_color_info(COLOR)
    assert frame(rt, 0.07).status == "OK"


def test_depth_shape_mismatch():
    rt = ready()
    rt.on_depth(np.zeros((240, 320), np.uint16), 1, 0.0)
    assert frame(rt, 0.05).reason == "DEPTH_SHAPE_MISMATCH"


def test_bad_encoding_is_reported_until_a_good_frame_arrives():
    rt = ready()
    rt.on_depth(default_scene(), 1, 0.0)
    rt.on_bad_depth(0.1)
    assert frame(rt, 0.15).reason == "DEPTH_ENCODING"
    rt.on_depth(default_scene(), 2, 0.2)
    assert frame(rt, 0.25).status == "OK"


def test_stale_contract_at_10_hz():
    rt = ready()
    rt.on_depth(default_scene(), 1, 0.0)
    statuses = [(round(t, 2), frame(rt, t).reason) for t in np.arange(0.05, 0.75, 0.1)]
    first_stale = next(t for t, reason in statuses if reason == "DEPTH_STALE")
    assert first_stale <= 0.5 + 0.1 + 0.05
    assert all(reason is None for t, reason in statuses if t <= 0.5)


def test_resend_keeps_the_stamp_and_ages_in_ms():
    rt = ready()
    rt.on_depth(default_scene(), 7, 0.0)
    first, second = frame(rt, 0.05), frame(rt, 0.15)
    assert first.depth_stamp_ns == second.depth_stamp_ns == 7
    assert second.ground.state == "FRESH"
    assert second.ground.age_ms == pytest.approx(150.0, abs=0.1)
    assert second.depth_age_ms == pytest.approx(150.0, abs=0.1)
    assert second.sequence == first.sequence + 1


def test_case_ga_expiry_is_judged_on_receive_time():
    # (가) t=0 FRESH, a frame received at 0.59 fails the fit: detector expires it.
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    assert frame(rt, 0.01).ground.state == "FRESH"
    rt.on_depth(wall(), 2, 0.59)
    packet = frame(rt, 0.60)
    assert (packet.status, packet.reason) == ("NO_GROUND", "GROUND_EXPIRED")


def test_case_na_send_time_expiry_overrides_a_held_result():
    # (나) the 0.45 frame fails (detector: HELD) but is sent at 0.56.
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    frame(rt, 0.01)
    rt.on_depth(wall(), 2, 0.45)
    packet = frame(rt, 0.56)
    assert (packet.status, packet.reason) == ("NO_GROUND", "GROUND_EXPIRED")


def test_cached_held_result_expires_on_resend():
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    frame(rt, 0.01)
    rt.on_depth(wall(), 2, 0.30)
    assert frame(rt, 0.31).ground.state == "HELD"
    assert frame(rt, 0.45).ground.state == "HELD"
    assert frame(rt, 0.55).reason == "GROUND_EXPIRED"


def test_processing_exception_becomes_a_processing_error(monkeypatch):
    errors = []
    rt = ready(errors=errors)

    def boom(*_args, **_kwargs):
        raise RuntimeError("synthetic core failure")

    monkeypatch.setattr(rt._detector, "process", boom)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.05)
    assert (packet.status, packet.reason, packet.proc_ms) == ("SENSOR_ERROR", "PROCESSING_ERROR", None)
    assert errors and "synthetic core failure" in errors[0]


def test_self_check_failure_sends_a_clean_processing_error(monkeypatch):
    errors = []
    rt = ready(errors=errors)
    original = rt._payload

    def corrupt(*args):
        payload = original(*args)
        payload["ground"]["camera_tilt_deg"] = 99.0
        return payload

    monkeypatch.setattr(rt, "_payload", corrupt)
    rt.on_depth(default_scene(), 5, 0.0)
    packet = frame(rt, 0.05)
    assert (packet.status, packet.reason) == ("SENSOR_ERROR", "PROCESSING_ERROR")
    assert packet.ground is None and packet.obstacles == () and packet.proc_ms is None
    assert all(value is None for value in packet.zones.values())
    assert packet.depth_stamp_ns == 5
    assert any("self-check" in message for message in errors)


def test_unsendable_error_packet_is_dropped(monkeypatch):
    rt = ready()
    monkeypatch.setattr(rt, "_payload", lambda *a: {"bad": float("nan")})
    monkeypatch.setattr(runtime_module, "PACKET_TYPE", "broken")
    rt.on_depth(default_scene(), 5, 0.0)
    rt._clock.t = 0.05
    assert rt.tick() is None
    assert rt.sequence == 0


def test_case_da_processing_time_counts_toward_expiry_and_ages(monkeypatch):
    # (다) a failed-fit frame received at 0.40 is HELD for the detector, but
    # processing takes 0.20 s, so at send time (0.61) the plane is 0.61 s old.
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    frame(rt, 0.01)
    slow_processing(rt, monkeypatch, 0.20)
    rt.on_depth(wall(), 2, 0.40)
    packet = frame(rt, 0.41)
    assert (packet.status, packet.reason) == ("NO_GROUND", "GROUND_EXPIRED")
    assert packet.depth_age_ms == pytest.approx(210.0, abs=0.1)


def test_processing_time_is_included_in_fresh_ages(monkeypatch):
    rt = ready()
    slow_processing(rt, monkeypatch, 0.08)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.02)
    assert packet.ground.state == "FRESH"
    assert packet.ground.age_ms == pytest.approx(100.0, abs=0.1)
    assert packet.depth_age_ms == pytest.approx(100.0, abs=0.1)


def test_depth_that_goes_stale_during_processing_is_reported_stale(monkeypatch):
    rt = ready()
    slow_processing(rt, monkeypatch, 0.30)
    rt.on_depth(default_scene(), 1, 0.0)
    assert frame(rt, 0.25).reason == "DEPTH_STALE"  # sent at 0.55 > 0.5


def test_processing_longer_than_a_tick_is_logged_and_bounds_stale(monkeypatch):
    # Codex 2026-10-17: a 0.6 s process delays the DEPTH_STALE send to 0.68 s.
    # The bound is stale + 1/hz + processing; the overrun is logged so the
    # Jetson load check (§7.3-2) can catch it.
    errors = []
    rt = ready(errors=errors)
    slow_processing(rt, monkeypatch, 0.6)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.08)
    assert (packet.reason, packet.depth_age_ms) == ("DEPTH_STALE", 680.0)
    assert any("exceeds the 100 ms tick" in message for message in errors)


def test_failed_processing_still_counts_toward_the_maximum(monkeypatch):
    errors = []
    rt = ready(errors=errors)

    def slow_failure(*_args):
        rt._clock.t += 0.6
        raise RuntimeError("synthetic core failure")

    monkeypatch.setattr(rt._detector, "process", slow_failure)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.08)
    assert (packet.reason, packet.depth_age_ms) == ("DEPTH_STALE", 680.0)
    assert rt.processing_overruns == 1
    assert rt.max_processing_s == pytest.approx(0.6)
    assert any("exceeds the 100 ms tick" in message for message in errors)


def test_overruns_use_their_own_log_sink(monkeypatch):
    errors, overruns = [], []
    rt = TerrainOverlayRuntime(RuntimeConfig(), SESSION, log_error=errors.append,
                               log_overrun=overruns.append, clock=FakeClock())
    rt._record_processing_time(0.2)
    assert len(overruns) == 1 and errors == []


def test_overrun_is_judged_before_rounding(monkeypatch):
    rt = ready()
    slow_processing(rt, monkeypatch, 0.10004)  # rounds to 100.0 ms
    rt.on_depth(default_scene(), 1, 0.0)
    frame(rt, 0.0)
    assert rt.processing_overruns == 1


def test_front_offset_switches_the_distance_origin():
    rt = ready(RuntimeConfig(front_offset_m=0.19))
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.05)
    assert packet.distance_origin == "rover_front"
    assert packet.zones["center"] == pytest.approx(1.01, abs=0.03)


@pytest.mark.parametrize("kwargs", [
    {"publish_hz": 4.9}, {"publish_hz": 10.1}, {"depth_stale_s": 0.0},
    {"ground_hold_s": -1.0}, {"path_half_width_m": 0.0}, {"front_offset_m": -0.1},
    {"publish_hz": float("nan")},
])
def test_invalid_config_is_refused(kwargs):
    with pytest.raises(ValueError):
        RuntimeConfig(**kwargs)


def test_session_identity(tmp_path):
    boot = tmp_path / "boot_id"
    boot.write_text("0082f17f-dc0a-4996-9ff0-fe1dc4979560\n")
    first = SenderSession.create(boot_id_path=boot, monotonic_ns=lambda: 42)
    second = SenderSession.create(boot_id_path=boot, monotonic_ns=lambda: 42)
    assert first.boot_id == "0082f17fdc0a49969ff0fe1dc4979560"
    assert len(first.instance) == 16
    assert (first.boot_id, first.start_mono_ns) == (second.boot_id, second.start_mono_ns)
    assert first.instance != second.instance  # equal start values still differ


def test_reprocesses_after_camera_info_changes():
    rt = ready()
    rt.on_depth(default_scene(), 1, 0.0)
    assert frame(rt, 0.05).status == "OK"
    rt.set_depth_info(CameraModel(400.0, 400.0, 320.0, 240.0, 640, 480))
    assert rt._cache is None
    frame(rt, 0.06)
    assert rt._cache is not None
```

- [ ] **Step 2: 실패 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_runtime.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'terrain_overlay.runtime'`

- [ ] **Step 3: 구현**

```python
"""ROS-free tick logic of the terrain overlay sender (spec §4.1, §4.4).

The ROS node only converts messages and sends bytes; everything that decides
what a packet says lives here so it can be tested on any host with numpy/cv2.
All times are monotonic seconds; milliseconds appear only in the packet.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import secrets
import threading
import time
from typing import Callable

import numpy as np

from .camera import CameraModel
from .contours import build_obstacles, fit_packet_size
from .contract import (
    CAMERA_HEIGHT_RANGE_M, FRAME_SIZE, PACKET_TYPE, SCHEMA_VERSION,
    encode_packet, parse_packet,
)
from .detector import DetectorConfig, TerrainDetector, TerrainResult
from .projection import DEFAULT_COLOR_OFFSET_M, DepthToColorProjector

BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
PUBLISH_HZ_RANGE = (5.0, 10.0)
EMPTY_ZONES = {"left": None, "center": None, "right": None}


@dataclass(frozen=True)
class RuntimeConfig:
    publish_hz: float = 10.0
    depth_scale_m: float = 0.00025
    depth_stale_s: float = 0.5
    ground_hold_s: float = 0.5
    color_offset_m: tuple[float, float, float] = DEFAULT_COLOR_OFFSET_M
    path_half_width_m: float = 0.4445
    front_offset_m: float = 0.0
    alignment_verified: bool = False

    def __post_init__(self) -> None:
        numbers = (self.publish_hz, self.depth_scale_m, self.depth_stale_s,
                   self.ground_hold_s, self.path_half_width_m, self.front_offset_m)
        if not all(math.isfinite(float(n)) for n in numbers):
            raise ValueError("terrain overlay parameters must be finite")
        low, high = PUBLISH_HZ_RANGE
        if not low <= self.publish_hz <= high:
            raise ValueError("publish_hz must be within 5-10 Hz (stale contract)")
        if min(self.depth_scale_m, self.depth_stale_s, self.ground_hold_s,
               self.path_half_width_m) <= 0.0:
            raise ValueError("scale, stale, hold and half width must be > 0")
        if self.front_offset_m < 0.0:
            raise ValueError("front_offset_m must be >= 0")
        if len(self.color_offset_m) != 3:
            raise ValueError("color_offset_m needs three values")
        if not isinstance(self.alignment_verified, bool):
            raise ValueError("alignment_verified must be bool")

    @property
    def detector_config(self) -> DetectorConfig:
        return DetectorConfig(
            depth_scale_m=self.depth_scale_m,
            ground_hold_s=self.ground_hold_s,
            path_half_width_m=self.path_half_width_m,
            front_offset_m=self.front_offset_m,
        )


@dataclass(frozen=True)
class SenderSession:
    boot_id: str
    start_mono_ns: int
    instance: str

    @classmethod
    def create(
        cls, *, boot_id_path: Path = BOOT_ID_PATH,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        token_hex: Callable[[int], str] = secrets.token_hex,
    ) -> "SenderSession":
        boot_id = boot_id_path.read_text(encoding="ascii").strip().replace("-", "").lower()
        if len(boot_id) != 32 or any(c not in "0123456789abcdef" for c in boot_id):
            raise ValueError(f"unexpected kernel boot_id {boot_id!r}")
        return cls(boot_id=boot_id, start_mono_ns=int(monotonic_ns()), instance=token_hex(8))


@dataclass(frozen=True)
class _Depth:
    array: np.ndarray
    stamp_ns: int
    rx_s: float


@dataclass(frozen=True)
class _Processed:
    stamp_ns: int
    rx_s: float
    status: str
    reason: str | None
    ground_state: str | None
    fresh_rx_s: float | None
    camera_height_m: float | None
    camera_tilt_deg: float | None
    zones: dict
    obstacles: list
    proc_ms: float | None


def _round_height(height_m: float) -> float:
    rounded = round(height_m, 3)
    low, high = CAMERA_HEIGHT_RANGE_M
    return rounded if low < rounded < high else height_m


def _round_zones(zones: dict) -> dict:
    return {name: None if value is None else round(value, 2) for name, value in zones.items()}


class TerrainOverlayRuntime:
    def __init__(
        self, config: RuntimeConfig, session: SenderSession, *,
        log_error: Callable[[str], None] = lambda _message: None,
        log_overrun: Callable[[str], None] | None = None,
        unix_ns: Callable[[], int] = time.time_ns,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.session = session
        self._log_error = log_error
        # A separate sink so log throttling of general errors can never hide
        # the processing-time evidence the load check (§7.3-2) relies on.
        self._log_overrun = log_error if log_overrun is None else log_overrun
        self._unix_ns = unix_ns
        self._clock = clock
        # on_depth/on_bad_depth may run on another executor thread than tick.
        self._input_lock = threading.Lock()
        self._depth_camera: CameraModel | None = None
        self._color_camera: CameraModel | None = None
        self._detector: TerrainDetector | None = None
        self._projector: DepthToColorProjector | None = None
        self._latest: _Depth | None = None
        self._bad_depth_rx_s: float | None = None
        self._cache: _Processed | None = None
        self.sequence = 0
        self.max_processing_s = 0.0
        self.processing_overruns = 0

    # -- inputs ---------------------------------------------------------
    def set_depth_info(self, camera: CameraModel) -> None:
        if camera != self._depth_camera:
            self._depth_camera = camera
            self._detector = TerrainDetector(camera, self.config.detector_config)
            self._cache = None

    def set_color_info(self, camera: CameraModel) -> None:
        if camera != self._color_camera:
            self._color_camera = camera
            self._projector = DepthToColorProjector(camera, self.config.color_offset_m)
            self._cache = None

    def on_depth(self, array: np.ndarray, stamp_ns: int, rx_s: float) -> None:
        with self._input_lock:
            self._latest = _Depth(array, int(stamp_ns), float(rx_s))

    def on_bad_depth(self, rx_s: float) -> None:
        """A depth message failed the 16UC1/endian/step check and was dropped."""
        with self._input_lock:
            self._bad_depth_rx_s = float(rx_s)

    # -- tick -----------------------------------------------------------
    def tick(self) -> bytes | None:
        """Build this tick's packet; None only if even the error packet is invalid.

        The first evaluation may process a new depth frame (tens of ms); the
        clock is read again afterwards so stale/expiry/ages are judged at send
        time (spec §4.1-4).  The second evaluation reuses the cache.
        """
        with self._input_lock:
            depth, bad_rx_s = self._latest, self._bad_depth_rx_s
        self._evaluate(self._clock(), depth, bad_rx_s)
        now_s = self._clock()
        status, reason, processed = self._evaluate(now_s, depth, bad_rx_s)
        stamp = None if depth is None else depth.stamp_ns
        age_ms = None if depth is None else round(max(0.0, now_s - depth.rx_s) * 1000.0, 1)
        try:
            payload = self._payload(now_s, status, reason, processed, stamp, age_ms)
            raw = encode_packet(fit_packet_size(payload))
            parse_packet(raw)
        except (ValueError, TypeError) as exc:
            self._log_error(f"terrain packet failed self-check: {exc}")
            raw = self._error_packet(now_s, stamp, age_ms)
            if raw is None:
                return None
        self.sequence += 1
        return raw

    def _evaluate(self, now_s: float, latest: _Depth | None, bad: float | None):
        cfg = self.config
        if self._depth_camera is None or self._color_camera is None:
            return "SENSOR_ERROR", "CAMERA_INFO_MISSING", None
        if (self._color_camera.width, self._color_camera.height) != FRAME_SIZE:
            return "SENSOR_ERROR", "COLOR_SHAPE_MISMATCH", None
        if (
            bad is not None and now_s - bad <= cfg.depth_stale_s
            and (latest is None or bad > latest.rx_s)
        ):
            return "SENSOR_ERROR", "DEPTH_ENCODING", None
        if latest is None or now_s - latest.rx_s > cfg.depth_stale_s:
            return "SENSOR_ERROR", "DEPTH_STALE", None
        if latest.array.shape != (self._depth_camera.height, self._depth_camera.width):
            return "SENSOR_ERROR", "DEPTH_SHAPE_MISMATCH", None
        if self._cache is None or self._cache.stamp_ns != latest.stamp_ns:
            self._cache = self._process(latest)
        processed = self._cache
        if (
            processed.status == "OK"
            and now_s - processed.fresh_rx_s > cfg.ground_hold_s
        ):
            return "NO_GROUND", "GROUND_EXPIRED", processed
        return processed.status, processed.reason, processed

    def _process(self, depth: _Depth) -> _Processed:
        started = self._clock()
        try:
            result: TerrainResult = self._detector.process(depth.array, depth.rx_s)
            obstacles = []
            if result.status == "OK":
                obstacles = build_obstacles(
                    result.obstacle_points, result.obstacle_fwd, result.obstacle_lat,
                    self._projector,
                    half_width_m=self.config.path_half_width_m,
                    front_offset_m=self.config.front_offset_m,
                )
        except Exception as exc:  # noqa: BLE001 — any core failure becomes a status
            self._record_processing_time(self._clock() - started)
            self._log_error(f"terrain processing failed: {type(exc).__name__}: {exc}")
            return _Processed(depth.stamp_ns, depth.rx_s, "SENSOR_ERROR", "PROCESSING_ERROR",
                              None, None, None, None, dict(EMPTY_ZONES), [], None)
        elapsed_s = self._clock() - started
        self._record_processing_time(elapsed_s)
        return _Processed(
            depth.stamp_ns, depth.rx_s, result.status, result.reason,
            result.ground_state, result.fresh_rx_s,
            result.camera_height_m, result.camera_tilt_deg,
            dict(result.zones), obstacles, round(elapsed_s * 1000.0, 1),
        )

    def _record_processing_time(self, elapsed_s: float) -> None:
        """Track every processing attempt — successful or not — against one tick.

        The DEPTH_STALE send bound is stale + 1/hz + processing + 0.05 s, so the
        load check (§7.3-2) needs the true maximum: compared unrounded, and
        including attempts that ended in an exception.
        """
        self.max_processing_s = max(self.max_processing_s, elapsed_s)
        tick_s = 1.0 / self.config.publish_hz
        if elapsed_s > tick_s:
            self.processing_overruns += 1
            self._log_overrun(
                f"terrain processing {elapsed_s * 1000.0:.2f} ms exceeds the "
                f"{tick_s * 1000.0:.0f} ms tick ({self.processing_overruns} overruns, "
                f"max {self.max_processing_s * 1000.0:.2f} ms)",
            )

    # -- packet ---------------------------------------------------------
    def _common(self, stamp, age_ms) -> dict:
        cfg = self.config
        return {
            "schema_version": SCHEMA_VERSION,
            "type": PACKET_TYPE,
            "sender_boot_id": self.session.boot_id,
            "sender_start_mono_ns": self.session.start_mono_ns,
            "sender_instance": self.session.instance,
            "sequence": self.sequence,
            "sent_unix_ns": int(self._unix_ns()),
            "depth_stamp_ns": stamp,
            "depth_age_ms": age_ms,
            "frame_size": list(FRAME_SIZE),
            "distance_origin": "rover_front" if cfg.front_offset_m > 0.0 else "l515_depth_optical",
            "alignment_verified": cfg.alignment_verified,
            "path_half_width_m": cfg.path_half_width_m,
        }

    def _payload(self, now_s, status, reason, processed, stamp, age_ms) -> dict:
        payload = self._common(stamp, age_ms)
        payload.update(status=status, reason=reason)
        if status == "OK":
            payload.update(
                ground={
                    "state": processed.ground_state,
                    "age_ms": round(max(0.0, now_s - processed.fresh_rx_s) * 1000.0, 1),
                    "camera_height_m": _round_height(processed.camera_height_m),
                    "camera_tilt_deg": round(processed.camera_tilt_deg, 1),
                },
                zones=_round_zones(processed.zones),
                obstacles=processed.obstacles,
                proc_ms=processed.proc_ms,
            )
        else:
            payload.update(
                ground=None, zones=dict(EMPTY_ZONES), obstacles=[],
                proc_ms=None if processed is None else processed.proc_ms,
            )
        return payload

    def _error_packet(self, now_s, stamp, age_ms) -> bytes | None:
        payload = self._common(stamp, age_ms)
        payload.update(status="SENSOR_ERROR", reason="PROCESSING_ERROR", ground=None,
                       zones=dict(EMPTY_ZONES), obstacles=[], proc_ms=None)
        try:
            raw = encode_packet(payload)
            parse_packet(raw)
        except (ValueError, TypeError) as exc:
            self._log_error(f"terrain error packet failed self-check, not sent: {exc}")
            return None
        return raw
```

- [ ] **Step 4: 통과 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay`
Expected: `116 passed` (계약 59, 판정기 21, 투영·윤곽 7, 런타임 29)

- [ ] **Step 5: 커밋(사용자 확인 후)**

```bash
git add terrain_overlay/runtime.py terrain_overlay/tests/test_runtime.py
git commit -m "feat(terrain): 송신 틱의 만료·stale·자기 검증 논리를 ROS 밖에 둔다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: ROS 어댑터 `terrain_overlay_node`

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/terrain_overlay_node.py`, `ros2/src/powertrain_ros/test/test_terrain_overlay_node.py`, `ros2/src/powertrain_ros/test/installed_terrain_overlay.py`
- Modify: `ros2/src/powertrain_ros/setup.py` (console_scripts)

**Interfaces:**
- Consumes:
  - `RuntimeConfig`, `SenderSession`, `TerrainOverlayRuntime`, `CameraModel.from_k`
  - `powertrain_runtime.telemetry.send_datagram(sock, payload, (host, port))`. 환경변수 `POWERTRAIN_OPERATOR_SESSION_FILE`이 있으면 세션 파일의 주소로 보내고, 파일이 없거나 만료면 `False`를 돌려준다(보내지 않음).
- Produces:
  - `ros2 run powertrain_ros terrain_overlay`
  - `TerrainOverlayNode(*, session=None, **node_kwargs)`. 시험은 `parameter_overrides`로 파라미터를 넣는다.
  - `depth_array(Image) -> ndarray | None`, `send_errors` 카운터

구독 QoS: depth는 `LATEST_DEPTH_QOS`(best effort, volatile, KEEP_LAST 1)를 쓴다. CameraInfo 두 토픽은 `qos_profile_sensor_data`를 쓴다.

실행 구조: `main()`은 `MultiThreadedExecutor(num_threads=2)`로 돈다. depth 구독은 `depth_group`에, CameraInfo 두 구독과 타이머는 `tick_group`에 둔다(각각 `MutuallyExclusiveCallbackGroup`). 시험도 같은 구조로 노드를 백그라운드 스레드에서 돌리고, 고정 장치는 시험 스레드에서 발행한다.

**시험 실행 환경:** rclpy가 필요하다.
- 1순위: Jetson의 `powertrain-sw:ros` 컨테이너(Task 12).
- Jetson에 닿지 않으면: 로컬 시험 이미지(x86, ROS Humble). 아래 Step 1에서 한 번 만든다.

- [ ] **Step 1: 로컬 ROS 시험 이미지 준비(없을 때만)**

```bash
mkdir -p /tmp/claude-1000/terrain-rosimg && cat > /tmp/claude-1000/terrain-rosimg/Dockerfile <<'EOF'
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-pytest python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir numpy opencv-python-headless
EOF
docker build -q -t terrain-ros-test:local /tmp/claude-1000/terrain-rosimg
```
Expected: `sha256:…`. 2026-10-17 기준 numpy 2.2.6, cv2 5.0.0이 설치된다.

- [ ] **Step 2: 실패하는 노드 시험 작성**

```python
"""terrain_overlay_node — ROS adapter around terrain_overlay.runtime (domain 77).

The node spins on a 2-thread MultiThreadedExecutor in a background thread,
exactly like ``main``; the fixture publishes from the test thread, so depth
keeps arriving independently of how long a tick takes.
"""
import json
import socket
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from powertrain_ros.terrain_overlay_node import TerrainOverlayNode, depth_array
from terrain_overlay.tests.scenes import COLOR, DEPTH, default_scene

STALE_CONTRACT_S = 0.5 + 0.1 + 0.05  # spec §4.1 at 10 Hz
SLACK_S = 0.05  # test-loop sampling


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def no_operator_session(monkeypatch):
    # Inside the production container the session file would redirect every
    # packet to the real console; tests must send to their local fixture.
    monkeypatch.delenv("POWERTRAIN_OPERATOR_SESSION_FILE", raising=False)


@pytest.fixture
def udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.setblocking(False)
    yield sock
    sock.close()


def info(camera):
    message = CameraInfo()
    message.width, message.height = camera.width, camera.height
    message.k = [camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0]
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    return message


def image(array, stamp_s=1, encoding="16UC1"):
    message = Image()
    message.header.stamp.sec = stamp_s
    message.height, message.width = array.shape
    message.encoding = encoding
    message.is_bigendian = 0
    message.step = array.shape[1] * 2
    message.data = array.tobytes()
    return message


class Spinning:
    """Run a node on the production executor shape in a background thread."""

    def __init__(self, node):
        self.node = node
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()


class Fixture:
    def __init__(self):
        self.node = rclpy.create_node("terrain_fixture")
        self.depth = self.node.create_publisher(Image, "/l515/depth/image_rect_raw", qos_profile_sensor_data)
        self.depth_info = self.node.create_publisher(CameraInfo, "/l515/depth/camera_info", qos_profile_sensor_data)
        self.color_info = self.node.create_publisher(CameraInfo, "/l515/color/camera_info", qos_profile_sensor_data)
        self.stamp = 0

    def publish(self, array=None, encoding="16UC1"):
        self.stamp += 1
        self.depth_info.publish(info(DEPTH))
        self.color_info.publish(info(COLOR))
        self.depth.publish(image(default_scene() if array is None else array,
                                 stamp_s=self.stamp, encoding=encoding))


def collect(udp, seconds, publish=None, period_s=0.1):
    """Publish at the Gateway's 10 Hz and gather (receive time, packet)."""
    packets = []
    deadline = time.monotonic() + seconds
    next_publish = time.monotonic()
    while time.monotonic() < deadline:
        if publish is not None and time.monotonic() >= next_publish:
            publish()
            next_publish += period_s
        time.sleep(0.005)
        while True:
            try:
                packets.append((time.monotonic(), json.loads(udp.recv(2048))))
            except BlockingIOError:
                break
    return packets


def make_node(port, **params):
    overrides = [Parameter("operator_host", value="127.0.0.1"),
                 Parameter("operator_port", value=port)]
    overrides += [Parameter(name, value=value) for name, value in params.items()]
    return TerrainOverlayNode(parameter_overrides=overrides)


@pytest.fixture
def rig(udp):
    spinning = Spinning(make_node(udp.getsockname()[1]))
    fixture = Fixture()
    yield spinning.node, fixture, udp
    spinning.stop()
    fixture.node.destroy_node()


def test_ok_packet_then_stale_after_depth_stops(rig):
    node, fixture, udp = rig
    packets = collect(udp, 2.0, fixture.publish)
    ok = [p for _, p in packets if p["status"] == "OK"]
    assert ok and ok[-1]["obstacles"] and ok[-1]["obstacles"][0]["in_path"] is True
    assert len(ok[-1]["sender_boot_id"]) == 32 and len(ok[-1]["sender_instance"]) == 16
    stopped = time.monotonic()
    stale_at = [t for t, p in collect(udp, 1.2) if p["reason"] == "DEPTH_STALE"]
    assert stale_at and stale_at[0] - stopped <= STALE_CONTRACT_S + SLACK_S


def test_wrong_encoding_is_reported(rig):
    node, fixture, udp = rig
    packets = collect(udp, 1.0, lambda: fixture.publish(encoding="mono16"))
    assert any(p["reason"] == "DEPTH_ENCODING" for _, p in packets)


def test_camera_info_missing_is_reported_without_info(rig):
    node, fixture, udp = rig
    packets = collect(udp, 0.5)
    assert packets and all(p["reason"] == "CAMERA_INFO_MISSING" for _, p in packets)


def test_send_failures_are_absorbed(udp):
    node = make_node(udp.getsockname()[1])
    try:
        def refuse(*_args, **_kwargs):
            raise OSError(101, "Network is unreachable")

        node._udp.close()
        node._udp = type("Sock", (), {"sendto": staticmethod(refuse), "close": lambda self: None})()
        node._tick()
        node._tick()
        assert node.send_errors == 2
    finally:
        node.destroy_node()


def test_missing_operator_host_without_session_file_refuses_to_start(monkeypatch):
    monkeypatch.delenv("POWERTRAIN_OPERATOR_SESSION_FILE", raising=False)
    with pytest.raises(ValueError):
        TerrainOverlayNode()


def test_missing_session_file_sends_nothing_and_keeps_running(udp, monkeypatch, tmp_path):
    session_file = tmp_path / "operator-session.json"  # absent = no authenticated console
    monkeypatch.setenv("POWERTRAIN_OPERATOR_SESSION_FILE", str(session_file))
    spinning = Spinning(TerrainOverlayNode(parameter_overrides=[
        Parameter("operator_port", value=udp.getsockname()[1])]))
    try:
        assert collect(udp, 0.4) == []
        session_file.write_text(json.dumps(
            {"host": "127.0.0.1", "lease_id": "x", "expires_at": time.monotonic() + 60}))
        assert collect(udp, 0.4)  # resumes once the session is valid
        assert spinning.node.send_errors == 0
    finally:
        spinning.stop()


def test_slow_processing_keeps_the_stale_contract(rig, monkeypatch):
    node, fixture, udp = rig
    collect(udp, 0.3, fixture.publish)  # camera info received, detector built
    original = node._runtime._detector.process

    def slow(*args):
        time.sleep(0.15)  # slower than the 0.1 s tick
        return original(*args)

    monkeypatch.setattr(node._runtime._detector, "process", slow)
    packets = collect(udp, 2.0, fixture.publish)
    assert len(packets) >= 8  # keeps sending while every tick processes
    stopped = time.monotonic()
    stale_at = [t for t, p in collect(udp, 1.5) if p["reason"] == "DEPTH_STALE"]
    # Depth is stamped on its own executor thread, so slow ticks do not delay it.
    assert stale_at and stale_at[0] - stopped <= STALE_CONTRACT_S + SLACK_S


def test_depth_is_stamped_while_a_tick_is_still_processing(rig, monkeypatch):
    # Deterministic form of the slow-tick case: with depth in the tick's
    # callback group (or a single-threaded executor) this fails, because the
    # frame published mid-tick is only taken — and stamped — after the tick.
    node, fixture, udp = rig
    collect(udp, 0.3, fixture.publish)
    original = node._runtime._detector.process
    processing = threading.Event()

    def slow(*args):
        processing.set()
        time.sleep(0.6)
        return original(*args)

    monkeypatch.setattr(node._runtime._detector, "process", slow)
    fixture.publish()  # new stamp: the next tick starts a 0.6 s process
    assert processing.wait(1.0)
    before = node._runtime._latest.stamp_ns
    published = time.monotonic()
    fixture.publish()
    deadline = published + 0.3
    while node._runtime._latest.stamp_ns == before and time.monotonic() < deadline:
        time.sleep(0.005)
    assert node._runtime._latest.stamp_ns != before
    assert node._runtime._latest.rx_s - published < 0.1


def test_overrun_survives_a_throttled_general_error_and_is_summarised(udp, capfd):
    node = make_node(udp.getsockname()[1])
    try:
        node._log_error("fixture general error")      # occupies the error site
        node._runtime._record_processing_time(0.6)      # within the same 5 s
        node._runtime._record_processing_time(0.02)     # recovery
        err = capfd.readouterr().err
        assert "exceeds the 100 ms tick" in err
        assert node.processing_summary() == (
            "terrain processing summary: max 600.00 ms, overruns 1")
    finally:
        node.destroy_node()
    assert "terrain processing summary: max 600.00 ms, overruns 1" in capfd.readouterr().err


def test_depth_array_checks_layout():
    good = image(np.zeros((4, 6), np.uint16))
    assert depth_array(good).shape == (4, 6)
    bad = image(np.zeros((4, 6), np.uint16))
    bad.step = 14
    assert depth_array(bad) is None
    big = image(np.zeros((4, 6), np.uint16))
    big.is_bigendian = 1
    assert depth_array(big) is None
```

- [ ] **Step 3: 실패 확인**

Run:
```bash
docker run --rm -v "$PWD":/workspace:ro -w /workspace -e ROS_DOMAIN_ID=77 -e PYTHONDONTWRITEBYTECODE=1 \
  -e PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros:/workspace/motor_control terrain-ros-test:local \
  bash -lc 'source /opt/ros/humble/setup.bash; python3 -m pytest -q -p no:cacheprovider ros2/src/powertrain_ros/test/test_terrain_overlay_node.py'
```
Expected: FAIL — `ModuleNotFoundError: No module named 'powertrain_ros.terrain_overlay_node'`

- [ ] **Step 4: 노드 구현과 엔트리포인트 등록**

`ros2/src/powertrain_ros/powertrain_ros/terrain_overlay_node.py`:

```python
"""L515 terrain obstacle outlines → operator console UDP :5009.

Operator visual aid only (2026-10-17 decision): this node never publishes to
ROS and gives nothing to control, stop, speed limits, SafetyInterlock or
US-100.  It subscribes to the Gateway's raw depth and both CameraInfo topics
and sends one packet per tick; all packet decisions live in the ROS-free
``terrain_overlay.runtime`` core.  Spec:
docs/specs/2026-10-17-l515-terrain-overlay-design.md
"""
from __future__ import annotations

import os
import socket
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from powertrain_runtime.telemetry import send_datagram
from terrain_overlay.camera import CameraModel
from terrain_overlay.runtime import RuntimeConfig, SenderSession, TerrainOverlayRuntime

DEPTH_TOPIC = "/l515/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/l515/depth/camera_info"
COLOR_INFO_TOPIC = "/l515/color/camera_info"
# Gateway-compatible (best effort, volatile) but latest-only: with the default
# depth 5 a slow tick drains queued frames late and stamps old depth as freshly
# received, delaying DEPTH_STALE (2026-10-17 slow-processing test: 1.2 s).
LATEST_DEPTH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=qos_profile_sensor_data.reliability,
    durability=qos_profile_sensor_data.durability,
)


def depth_array(message: Image) -> np.ndarray | None:
    """Return a private uint16 copy, or None if the message breaks 16UC1 LE."""
    if (
        message.encoding != "16UC1"
        or message.is_bigendian != 0
        or message.step != message.width * 2
        or len(message.data) != message.step * message.height
    ):
        return None
    return np.frombuffer(message.data, dtype=np.uint16).reshape(
        message.height, message.width,
    ).copy()


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class TerrainOverlayNode(Node):
    def __init__(self, *, session: SenderSession | None = None, **node_kwargs) -> None:
        super().__init__("terrain_overlay", **node_kwargs)
        self.declare_parameter("operator_host", "")
        self.declare_parameter("operator_port", 5009)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("depth_scale_m", 0.00025)
        self.declare_parameter("depth_stale_s", 0.5)
        self.declare_parameter("ground_hold_s", 0.5)
        self.declare_parameter("color_offset_m", [0.0, -0.024, 0.0])
        self.declare_parameter("path_half_width_m", 0.4445)
        self.declare_parameter("front_offset_m", 0.0)
        self.declare_parameter("alignment_verified", False)
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        host = str(value("operator_host"))
        port = int(value("operator_port"))
        if not host and not os.environ.get("POWERTRAIN_OPERATOR_SESSION_FILE"):
            raise ValueError("operator_host is required without POWERTRAIN_OPERATOR_SESSION_FILE")
        if not 1 <= port <= 65535:
            raise ValueError("operator_port must be 1-65535")
        config = RuntimeConfig(
            publish_hz=float(value("publish_hz")),
            depth_scale_m=float(value("depth_scale_m")),
            depth_stale_s=float(value("depth_stale_s")),
            ground_hold_s=float(value("ground_hold_s")),
            color_offset_m=tuple(float(v) for v in value("color_offset_m")),
            path_half_width_m=float(value("path_half_width_m")),
            front_offset_m=float(value("front_offset_m")),
            alignment_verified=bool(value("alignment_verified")),
        )
        self._endpoint = (host, port)
        self._runtime = TerrainOverlayRuntime(
            config, session or SenderSession.create(),
            log_error=self._log_error, log_overrun=self._log_overrun,
        )
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_errors = 0
        self._logged_info: set[str] = set()
        # Depth arrives on its own group so its receive time is stamped while a
        # tick is still processing (needs a multi-threaded executor, see main).
        # Camera info shares the tick's group so detectors never change mid-tick.
        depth_group = MutuallyExclusiveCallbackGroup()
        tick_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image, DEPTH_TOPIC, self._on_depth, LATEST_DEPTH_QOS, callback_group=depth_group,
        )
        self.create_subscription(
            CameraInfo, DEPTH_INFO_TOPIC,
            lambda message: self._on_info("depth", message), qos_profile_sensor_data,
            callback_group=tick_group,
        )
        self.create_subscription(
            CameraInfo, COLOR_INFO_TOPIC,
            lambda message: self._on_info("color", message), qos_profile_sensor_data,
            callback_group=tick_group,
        )
        self.create_timer(1.0 / config.publish_hz, self._tick, callback_group=tick_group)
        session_file = os.environ.get("POWERTRAIN_OPERATOR_SESSION_FILE")
        target = (f"operator session file {session_file} (fallback {host or '-'})"
                  if session_file else host)
        session_ = self._runtime.session
        self.get_logger().info(
            f"terrain overlay → {target}:{port} · session "
            f"{session_.boot_id[:8]}/{session_.start_mono_ns}/{session_.instance}",
        )

    def _log_error(self, message: str) -> None:
        self.get_logger().error(message, throttle_duration_sec=5.0)

    def _log_overrun(self, message: str) -> None:
        # Own call site → own throttle state (rclpy throttles per call site).
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def processing_summary(self) -> str:
        runtime = self._runtime
        return (f"terrain processing summary: max {runtime.max_processing_s * 1000.0:.2f} ms, "
                f"overruns {runtime.processing_overruns}")

    def _on_depth(self, message: Image) -> None:
        received = time.monotonic()
        array = depth_array(message)
        if array is None:
            self._runtime.on_bad_depth(received)
            self.get_logger().warning(
                f"depth dropped: encoding={message.encoding} bigendian={message.is_bigendian} "
                f"step={message.step} width={message.width}",
                throttle_duration_sec=5.0,
            )
            return
        self._runtime.on_depth(array, stamp_ns(message), received)

    def _on_info(self, kind: str, message: CameraInfo) -> None:
        try:
            camera = CameraModel.from_k(message.k, message.width, message.height)
        except (ValueError, IndexError) as exc:
            self.get_logger().warning(f"{kind} camera_info ignored: {exc}", throttle_duration_sec=5.0)
            return
        if kind not in self._logged_info:
            self._logged_info.add(kind)
            self.get_logger().info(
                f"{kind} camera_info {message.width}x{message.height} "
                f"model={message.distortion_model} d={list(message.d)} (pinhole used)",
            )
        if kind == "depth":
            self._runtime.set_depth_info(camera)
        else:
            self._runtime.set_color_info(camera)

    def _tick(self) -> None:
        raw = self._runtime.tick()
        if raw is None:
            return
        try:
            send_datagram(self._udp, raw, self._endpoint)
        except OSError as exc:
            self.send_errors += 1
            self.get_logger().warning(
                f"terrain UDP send failed ({self.send_errors} total): {exc}",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self) -> bool:
        # Unthrottled: the §7.3-2 load check reads this line after the run.
        self.get_logger().info(self.processing_summary())
        self._udp.close()
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TerrainOverlayNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

`ros2/src/powertrain_ros/setup.py`:

```diff
--- a/ros2/src/powertrain_ros/setup.py
+++ b/ros2/src/powertrain_ros/setup.py
@@ -72,6 +72,7 @@
             "teleop_command = powertrain_ros.teleop_command_node:main",
             "ops_broker = powertrain_ros.ops_broker_node:main",
             "chassis_telemetry = powertrain_ros.chassis_telemetry_sender_node:main",
+            "terrain_overlay = powertrain_ros.terrain_overlay_node:main",
             "arm_console_bridge = powertrain_ros.arm_console_bridge_node:main",
             "wp8_handshake_probe = powertrain_ros.wp8_handshake_probe:main",
         ],
```

- [ ] **Step 5: 통과 확인(3회 반복 — 타이밍 시험이라 흔들림을 본다)**

Run: Step 3 명령을 3번 반복한다.
Expected: 매번 `10 passed`(2026-10-17 실측 11 s 안팎).

- [ ] **Step 6: 콜백 그룹 시험이 결함을 잡는지 1회 확인(수동 음성 대조, 커밋 안 함)**

임시로 depth 구독의 `callback_group=depth_group`을 `tick_group`으로 바꿔 Step 3을 돌린다.
Expected: `test_depth_is_stamped_while_a_tick_is_still_processing`만 FAIL(2026-10-17 실측 3/3).
참고로 끝단 시험(`…stale_contract`)은 송신 시각 재판정 덕분에 이 변형에서도 통과했다. 그래서 결정적 시험을 따로 둔다.
확인 뒤 `git diff`로 원복됐는지 본다.

- [ ] **Step 7: 설치 엔트리포인트 스모크 작성**

```python
"""Installed-entrypoint smoke for terrain_overlay (spec §7.2, domain 77).

Run inside the ROS image after ``colcon build`` and sourcing the install
space.  Never touches a camera: a fixture node publishes synthetic depth and
CameraInfo.  Checks: OK packet with an in-path outline arrives; after the
fixture stops publishing depth, DEPTH_STALE arrives within the 0.65 s
contract; SIGINT stops the node cleanly.

    ROS_DOMAIN_ID=77 python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py
    # negative control: re-inject the PR #13 double depth scale → must FAIL
    ROS_DOMAIN_ID=77 python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py --inject-double-scale
"""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from terrain_overlay.tests.scenes import COLOR, DEPTH, default_scene

STALE_CONTRACT_S = 0.65
SLACK_S = 0.05


def info(camera):
    message = CameraInfo()
    message.width, message.height = camera.width, camera.height
    message.k = [camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0]
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    return message


def depth_image(array, stamp_s):
    message = Image()
    message.header.stamp.sec = stamp_s
    message.height, message.width = array.shape
    message.encoding, message.is_bigendian = "16UC1", 0
    message.step = array.shape[1] * 2
    message.data = array.tobytes()
    return message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inject-double-scale", action="store_true",
                        help="negative control: metres scaled twice (PR #13 defect)")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        print("refusing to run outside ROS_DOMAIN_ID=77", file=sys.stderr)
        return 2
    prefix = subprocess.run(["ros2", "pkg", "prefix", "powertrain_ros"],
                            capture_output=True, text=True, check=True).stdout.strip()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.setblocking(False)
    env = {k: v for k, v in os.environ.items() if k != "POWERTRAIN_OPERATOR_SESSION_FILE"}
    node = subprocess.Popen(
        ["ros2", "run", "powertrain_ros", "terrain_overlay", "--ros-args",
         "-p", "operator_host:=127.0.0.1", "-p", f"operator_port:={udp.getsockname()[1]}"]
        + (["-p", "depth_scale_m:=6.25e-08"] if args.inject_double_scale else []),
        env=env, start_new_session=True,
    )
    rclpy.init()
    fixture = rclpy.create_node("installed_terrain_fixture")
    depth = fixture.create_publisher(Image, "/l515/depth/image_rect_raw", qos_profile_sensor_data)
    depth_info = fixture.create_publisher(CameraInfo, "/l515/depth/camera_info", qos_profile_sensor_data)
    color_info = fixture.create_publisher(CameraInfo, "/l515/color/camera_info", qos_profile_sensor_data)
    scene = default_scene()
    packets = []

    def drain():
        while True:
            try:
                packets.append((time.monotonic(), json.loads(udp.recv(2048))))
            except BlockingIOError:
                return

    evidence = {"install_prefix": prefix}
    try:
        stamp = 0
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            stamp += 1
            depth_info.publish(info(DEPTH))
            color_info.publish(info(COLOR))
            depth.publish(depth_image(scene, stamp))
            time.sleep(0.1)
            drain()
            if any(p["status"] == "OK" and p["obstacles"] for _, p in packets):
                break
        ok = [p for _, p in packets if p["status"] == "OK" and p["obstacles"]]
        evidence["ok_packets"] = len(ok)
        if not ok or not ok[-1]["obstacles"][0]["in_path"]:
            print(json.dumps(dict(evidence, verdict="FAIL: no OK packet with an in-path outline")))
            return 1
        evidence["proc_ms"] = ok[-1]["proc_ms"]
        stopped = time.monotonic()
        while time.monotonic() < stopped + 1.5:
            time.sleep(0.01)
            drain()
        stale = [t for t, p in packets if t > stopped and p["reason"] == "DEPTH_STALE"]
        evidence["stale_after_s"] = None if not stale else round(stale[0] - stopped, 3)
        if not stale or stale[0] - stopped > STALE_CONTRACT_S + SLACK_S:
            print(json.dumps(dict(evidence, verdict="FAIL: DEPTH_STALE contract")))
            return 1
    finally:
        os.killpg(node.pid, signal.SIGINT)
        try:
            evidence["exit_code"] = node.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
            evidence["exit_code"] = "killed"
        fixture.destroy_node()
        rclpy.try_shutdown()
        udp.close()
    if evidence["exit_code"] != 0:
        print(json.dumps(dict(evidence, verdict="FAIL: node did not exit cleanly on SIGINT")))
        return 1
    print(json.dumps(dict(evidence, verdict="PASS")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 8: colcon 설치본으로 스모크 + 음성 대조**

Run:
```bash
docker run --rm -v "$PWD":/workspace:ro -e ROS_DOMAIN_ID=77 -e PYTHONDONTWRITEBYTECODE=1 terrain-ros-test:local bash -lc '
  set -e; source /opt/ros/humble/setup.bash; cp -r /workspace/ros2/src /tmp/src; cd /tmp
  colcon build --base-paths /tmp/src --packages-up-to powertrain_ros >/dev/null
  source /tmp/install/setup.bash; export PYTHONPATH=/workspace:$PYTHONPATH; cd /workspace
  python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py | tail -1
  python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py --inject-double-scale | tail -1 || true'
```
Expected:
- 첫 줄: `"verdict": "PASS"`, `stale_after_s` ≤ 0.70, `exit_code` 0. PASS는 SIGINT 뒤 종료 코드 0까지 포함한다. 2026-10-17 실측은 `proc_ms` 31.4, `stale_after_s` 0.454.
- 둘째 줄: `"verdict": "FAIL: no OK packet with an in-path outline"`.

- [ ] **Step 9: 커밋(사용자 확인 후)**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/terrain_overlay_node.py ros2/src/powertrain_ros/setup.py ros2/src/powertrain_ros/test/test_terrain_overlay_node.py ros2/src/powertrain_ros/test/installed_terrain_overlay.py
git commit -m "feat(terrain): L515 depth를 받아 :5009로 보내는 terrain_overlay 노드를 둔다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: 콘솔 수신·세션 게이트·표시 판단 (`operator_console/terrain.py`)

**Files:**
- Create: `operator_console/terrain.py`, `operator_console/tests/test_terrain.py`

**Interfaces:**
- Consumes: `terrain_overlay.contract.parse_packet`, `TerrainFrame`, `TerrainObstacle`, `MAX_PACKET_BYTES`
- Produces:
  - 상수 `BOOT_SWITCH_SILENCE_S = 1.0`, `LINK_STALE_S = 0.5`, `DEPTH_STALE_S = 0.6`, `MAX_DISPLAY_DELAY_S = 0.4`
  - `TerrainSessionGate().accept(frame, now_s) -> (accepted, session_changed)`
  - `LatestTerrainReceiver(port, display_delay_s=0.0)`. 속성: `port`, `display_delay_s`, `last_rx_monotonic`, `invalid_packet_count`, `rejected_packet_count`. 메서드: `ingest(raw, received_s) -> bool`, `latest()`, `display_frame(now_s)`, `snapshot(now_s) -> (latest, display, last_rx)`(한 락), `close()`.
  - `TerrainView(badge, tone, note, obstacles, probe_state)`
  - `terrain_view(latest, display, last_rx_s, now_s, display_delay_s, video_size, *, video_live=True) -> TerrainView`

- [ ] **Step 1: 실패하는 시험 작성** — Codex 4회차 수신 공백 반례, 시작값 동률, 다른 부팅 1 s, 표시 지연(T+0.41 s), Review Focus 3·4·5를 포함한다.

```python
"""Console terrain reception and display decisions (spec §4.6, §7.1)."""
import json

import pytest

from operator_console.terrain import (
    LatestTerrainReceiver, TerrainSessionGate, terrain_view,
)
from terrain_overlay.contract import parse_packet

BOOT = "0082f17fdc0a49969ff0fe1dc4979560"
OTHER_BOOT = "1111111111111111111111111111111a"
A = (BOOT, 1000, "aaaaaaaaaaaaaaaa")
B = (BOOT, 2000, "aaaaaaaaaaaaaaaa")


def payload(session=A, sequence=0, status="OK", **overrides):
    boot, start, instance = session
    body = {
        "schema_version": 1, "type": "l515_terrain",
        "sender_boot_id": boot, "sender_start_mono_ns": start, "sender_instance": instance,
        "sequence": sequence, "sent_unix_ns": 1, "depth_stamp_ns": 1, "depth_age_ms": 40.0,
        "status": status, "reason": None, "frame_size": [1280, 720],
        "distance_origin": "l515_depth_optical", "alignment_verified": False,
        "ground": {"state": "FRESH", "age_ms": 40.0, "camera_height_m": 0.5, "camera_tilt_deg": 20.0},
        "path_half_width_m": 0.4445,
        "zones": {"left": None, "center": 1.23, "right": None},
        "obstacles": [{"dist_m": 1.23, "in_path": True, "poly": [[10, 10], [50, 10], [50, 50]]}],
        "proc_ms": 20.0,
    }
    if status != "OK":
        body.update(reason=overrides.pop("reason"), ground=None,
                    zones={"left": None, "center": None, "right": None}, obstacles=[])
    body.update(overrides)
    return json.dumps(body).encode()


def frame(session=A, sequence=0, rx=0.0, **kw):
    return parse_packet(payload(session, sequence, **kw), received_monotonic_s=rx)


@pytest.fixture
def receiver():
    made = []

    def make(delay_s=0.0):
        item = LatestTerrainReceiver(0, display_delay_s=delay_s)
        made.append(item)
        return item

    yield make
    for item in made:
        item.close()


def feed(rx, events):
    """events: (time, session, sequence) → list of accepted booleans."""
    return [rx.ingest(payload(session, seq), t) for t, session, seq in events]


def test_same_session_rejects_duplicates_and_reordering(receiver):
    rx = receiver()
    assert feed(rx, [(0.0, A, 5), (0.1, A, 5), (0.2, A, 4), (0.3, A, 6)]) == [True, False, False, True]


def test_new_session_of_the_same_boot_is_accepted_immediately(receiver):
    rx = receiver()
    assert feed(rx, [(0.0, A, 100), (0.1, B, 0), (0.2, A, 101)]) == [True, True, False]


def test_receive_gap_counterexample_never_returns_to_the_old_session(receiver):
    rx = receiver()
    events = [(0.00, A, 100), (1.10, B, 0), (1.30, B, 1), (2.41, A, 99), (2.42, B, 12)]
    assert feed(rx, events) == [True, True, True, False, True]


def test_concurrent_senders_show_only_the_larger_session(receiver):
    rx = receiver()
    events = []
    for i in range(5):
        events += [(i * 0.1, A, i), (i * 0.1 + 0.01, B, i)]
    accepted = feed(rx, events)
    assert accepted[0] is True and accepted[1] is True
    assert accepted[2::2] == [False] * 4 and accepted[3::2] == [True] * 4


def test_equal_start_is_split_by_instance(receiver):
    low = (BOOT, 5000, "1111111111111111")
    high = (BOOT, 5000, "2222222222222222")
    rx = receiver()
    assert feed(rx, [(0.0, low, 100), (0.1, high, 0), (0.2, low, 101), (0.3, high, 1)]) == [
        True, True, False, True]


def test_other_boot_waits_for_one_second_of_silence(receiver):
    rx = receiver()
    other = (OTHER_BOOT, 1, "bbbbbbbbbbbbbbbb")
    assert feed(rx, [(0.0, A, 1), (0.5, other, 0), (0.9, A, 2), (1.95, other, 1)]) == [
        True, False, True, True]


def test_session_switch_clears_the_display_buffer(receiver):
    rx = receiver(0.3)
    feed(rx, [(0.0, A, 1), (0.1, A, 2), (0.2, B, 0)])
    assert rx.display_frame(0.45) is None  # A frames gone, B not yet 0.3 s old
    assert rx.display_frame(0.5).session_key == B


def test_malformed_and_wrong_type_packets_count_as_invalid(receiver):
    rx = receiver()
    assert rx.ingest(b"{}", 0.0) is False
    assert rx.ingest(json.dumps({"schema_version": 1, "type": "yolo"}).encode(), 0.0) is False
    assert rx.invalid_packet_count == 2


def test_display_delay_keeps_link_alive_and_picks_an_older_frame(receiver):
    rx = receiver(0.3)
    for i in range(20):
        rx.ingest(payload(A, i), i * 0.1)
    now = 1.95
    assert now - rx.last_rx_monotonic < 0.15
    shown = rx.display_frame(now)
    assert shown.sequence == 16  # received at 1.6 <= 1.95 - 0.3
    view = terrain_view(rx.latest(), shown, rx.last_rx_monotonic, now, 0.3, (1280, 720))
    assert view.probe_state == "LIVE" and view.obstacles


@pytest.mark.parametrize("delay_s", [-0.1, 0.5, float("nan")])
def test_display_delay_range_is_enforced(delay_s):
    with pytest.raises(ValueError):
        LatestTerrainReceiver(0, display_delay_s=delay_s)


def view_at(now, *, delay=0.0, rx=0.0, video=(1280, 720), live=True, **kw):
    f = frame(rx=rx, **kw)
    display = f if f.received_monotonic_s <= now - delay else None
    return terrain_view(f, display, rx, now, delay, video, video_live=live)


def test_depth_age_hides_an_old_frame_without_delay():
    assert view_at(0.10, depth_age_ms=490.0).obstacles
    assert not view_at(0.111, depth_age_ms=490.0).obstacles


def test_depth_age_with_delay_hides_at_t_plus_0_41():
    assert not view_at(0.29, delay=0.3, depth_age_ms=490.0).obstacles  # not displayed yet
    assert view_at(0.40, delay=0.3, depth_age_ms=490.0).obstacles
    assert not view_at(0.411, delay=0.3, depth_age_ms=490.0).obstacles


def test_link_age_drops_everything_after_half_a_second():
    view = view_at(0.51)
    assert (view.badge, view.obstacles, view.probe_state) == ("지형: 수신 없음", (), "STALE")


def test_size_mismatch_or_unknown_video_size_hides_outlines():
    assert not view_at(0.05, video=(848, 480)).obstacles
    assert not view_at(0.05, video=None).obstacles
    assert view_at(0.05).obstacles


def test_badges_per_status():
    assert view_at(0.05).badge == "지형 OK · 경로 1.23 m (카메라 기준)"
    assert view_at(0.05, distance_origin="rover_front").badge.endswith("(차체 앞 기준)")
    held = view_at(0.05, ground={"state": "HELD", "age_ms": 300.0,
                                 "camera_height_m": 0.5, "camera_tilt_deg": 20.0})
    assert held.badge == "지형: 바닥 유지 0.3 s" and held.obstacles
    assert view_at(0.05, status="NO_GROUND", reason="GROUND_EXPIRED").badge == "지형: 바닥 미검출"
    error = view_at(0.05, status="SENSOR_ERROR", reason="DEPTH_STALE")
    assert error.badge == "지형: 센서 오류(DEPTH_STALE)" and not error.obstacles
    assert view_at(0.05).note.startswith("정합 미검증")
    assert view_at(0.05, alignment_verified=True).note == "참고용 · 낮은 장애물/구덩이 미표시"


def test_no_data_yet_is_waiting():
    view = terrain_view(None, None, None, 5.0, 0.0, (1280, 720))
    assert (view.badge, view.probe_state) == ("지형: 수신 없음", "WAITING")


def test_real_socket_path_delivers_a_packet(receiver):
    import socket
    import time
    rx = receiver()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(payload(A, 3), ("127.0.0.1", rx.port))
        deadline = time.monotonic() + 2.0
        while rx.latest() is None and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        sender.close()
    assert rx.latest().sequence == 3


def test_hidden_outlines_explain_why_in_the_note():
    unknown = view_at(0.05, video=None)
    assert not unknown.obstacles and unknown.note.startswith("윤곽 숨김: 영상 크기 미확인")
    other = view_at(0.05, video=(960, 540))
    assert not other.obstacles and other.note.startswith("윤곽 숨김: 영상 960×540")
    assert not view_at(0.05).note.startswith("윤곽 숨김")


def test_address_move_keeps_the_sequence_floor(receiver):
    import socket
    import time
    rx = receiver()
    first = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # address 1
    second = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # address 2

    def wait_for(predicate):
        deadline = time.monotonic() + 2.0
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.005)
        assert predicate()

    try:
        first.sendto(payload(A, 100), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.latest() is not None and rx.latest().sequence == 100)
        second.sendto(payload(A, 99), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.rejected_packet_count == 1)
        second.sendto(payload(A, 101), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.latest().sequence == 101)
        first.sendto(payload(A, 102), ("127.0.0.1", rx.port))
        wait_for(lambda: rx.latest().sequence == 102)
        assert rx.rejected_packet_count == 1
    finally:
        first.close()
        second.close()


def test_nested_or_oversize_datagrams_are_invalid(receiver):
    rx = receiver()
    assert rx.ingest(b"[" * 600 + b"]" * 600, 0.0) is False
    assert rx.ingest(b" " * 1201, 0.0) is False
    assert rx.invalid_packet_count == 2


def test_video_loss_hides_outlines_but_keeps_the_badge():
    view = view_at(0.05, live=False)
    assert not view.obstacles
    assert view.badge.startswith("지형 OK")
    assert view.note.startswith("윤곽 숨김: 영상 끊김")


def test_empty_path_says_protruding_obstacles_only():
    view = view_at(0.05, zones={"left": None, "center": None, "right": None})
    assert view.badge == "지형 OK · 경로 돌출 장애물 없음 (카메라 기준)"


def test_snapshot_never_mixes_sessions(receiver):
    rx = receiver()
    feed(rx, [(0.0, A, 1), (0.1, B, 0)])
    latest, display, last_rx = rx.snapshot(0.2)
    assert latest.session_key == display.session_key == B and last_rx == 0.1
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest -q -p no:cacheprovider operator_console/tests/test_terrain.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'operator_console.terrain'`

- [ ] **Step 3: 구현**

```python
"""L515 terrain overlay reception: session gate, display delay, view decision.

Spec §4.6 (docs/specs/2026-10-17-l515-terrain-overlay-design.md).  Pure
Python — no GTK — so the decisions are testable without a display; the GTK
``TerrainCanvas`` in app.py only draws what ``terrain_view`` returns.
Operator visual aid only: nothing here feeds control or safety.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import socket
import threading
import time

from terrain_overlay.contract import (
    MAX_PACKET_BYTES, TerrainFrame, TerrainObstacle, parse_packet,
)

BOOT_SWITCH_SILENCE_S = 1.0
LINK_STALE_S = 0.5
DEPTH_STALE_S = 0.6
MAX_DISPLAY_DELAY_S = 0.4
BUFFER_MARGIN_S = 0.5
REFERENCE_NOTE = "참고용 · 낮은 장애물/구덩이 미표시"


class TerrainSessionGate:
    """Order senders by session, never by address (spec §4.6 세션 게이트)."""

    def __init__(self) -> None:
        self._session: tuple[str, int, str] | None = None
        self._last_sequence: int | None = None
        self._last_accepted_s: float | None = None

    def accept(self, frame: TerrainFrame, now_s: float) -> tuple[bool, bool]:
        """Return ``(accepted, session_changed)``."""
        key = frame.session_key
        current = self._session
        if current is None:
            changed = True
        elif key == current:
            if frame.sequence <= self._last_sequence:
                return False, False
            changed = False
        elif key[0] == current[0]:
            if key[1:] <= current[1:]:
                return False, False  # an older run of this boot, forever
            changed = True
        else:
            if now_s - self._last_accepted_s <= BOOT_SWITCH_SILENCE_S:
                return False, False
            changed = True
        self._session = key
        self._last_sequence = frame.sequence
        self._last_accepted_s = now_s
        return True, changed


def validate_display_delay_s(value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= MAX_DISPLAY_DELAY_S:
        raise ValueError("terrain display delay must be within 0-400 ms")
    return value


class LatestTerrainReceiver:
    """Daemon UDP receiver keeping a short, delay-aligned frame buffer."""

    def __init__(self, port: int, display_delay_s: float = 0.0) -> None:
        self.display_delay_s = validate_display_delay_s(display_delay_s)
        self._gate = TerrainSessionGate()
        self._buffer: deque[TerrainFrame] = deque()
        self._last_rx_s: float | None = None
        self._lock = threading.Lock()
        self._invalid_packet_count = 0
        self._rejected_packet_count = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name="l515-terrain", daemon=True)
        self._thread.start()

    @property
    def port(self) -> int:
        return self._socket.getsockname()[1]

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, _address = self._socket.recvfrom(MAX_PACKET_BYTES + 1)
            except OSError:
                continue
            try:
                self.ingest(raw, time.monotonic())
            except Exception:  # noqa: BLE001 — never let one datagram kill reception
                self._invalid_packet_count += 1

    def ingest(self, raw: bytes, received_s: float) -> bool:
        try:
            frame = parse_packet(raw, received_monotonic_s=received_s)
        except ValueError:
            self._invalid_packet_count += 1
            return False
        with self._lock:
            accepted, changed = self._gate.accept(frame, received_s)
            if not accepted:
                self._rejected_packet_count += 1
                return False
            if changed:
                self._buffer.clear()
            self._buffer.append(frame)
            self._last_rx_s = received_s
            horizon = received_s - (self.display_delay_s + BUFFER_MARGIN_S)
            while len(self._buffer) > 1 and self._buffer[0].received_monotonic_s < horizon:
                self._buffer.popleft()
        return True

    @property
    def invalid_packet_count(self) -> int:
        return self._invalid_packet_count

    @property
    def rejected_packet_count(self) -> int:
        return self._rejected_packet_count

    @property
    def last_rx_monotonic(self) -> float | None:
        with self._lock:
            return self._last_rx_s

    def latest(self) -> TerrainFrame | None:
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def display_frame(self, now_s: float) -> TerrainFrame | None:
        return self.snapshot(now_s)[1]

    def snapshot(self, now_s: float):
        """``(latest, display_frame, last_rx)`` read under one lock.

        Separate reads can straddle a session switch and pair an old frame
        with the new session's receive time (Codex 2026-10-17 interleaving).
        """
        cutoff = now_s - self.display_delay_s
        with self._lock:
            display = next(
                (f for f in reversed(self._buffer) if f.received_monotonic_s <= cutoff),
                None,
            )
            latest = self._buffer[-1] if self._buffer else None
            return latest, display, self._last_rx_s

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)


@dataclass(frozen=True)
class TerrainView:
    badge: str
    tone: str  # muted / ok / warn / error
    note: str
    obstacles: tuple[TerrainObstacle, ...]
    probe_state: str  # WAITING / LIVE / STALE (runtime_smoke)


def _basis(frame: TerrainFrame) -> str:
    return "차체 앞 기준" if frame.distance_origin == "rover_front" else "카메라 기준"


def _note(frame: TerrainFrame | None) -> str:
    if frame is not None and not frame.alignment_verified:
        return f"정합 미검증 · {REFERENCE_NOTE}"
    return REFERENCE_NOTE


def terrain_view(
    latest: TerrainFrame | None, display: TerrainFrame | None,
    last_rx_s: float | None, now_s: float, display_delay_s: float,
    video_size: tuple[int, int] | None,
    *, video_live: bool = True,
) -> TerrainView:
    if last_rx_s is None or latest is None:
        return TerrainView("지형: 수신 없음", "muted", REFERENCE_NOTE, (), "WAITING")
    link_age = now_s - last_rx_s
    if link_age > LINK_STALE_S:
        return TerrainView("지형: 수신 없음", "muted", REFERENCE_NOTE, (), "STALE")
    shown = display or latest
    aged = max(0.0, now_s - display_delay_s - shown.received_monotonic_s)
    if shown.status == "OK":
        if shown.ground.state == "HELD":
            held_s = shown.ground.age_ms / 1000.0 + aged
            badge, tone = f"지형: 바닥 유지 {held_s:.1f} s", "warn"
        else:
            center = shown.zones["center"]
            path = "경로 돌출 장애물 없음" if center is None else f"경로 {center:.2f} m"
            badge, tone = f"지형 OK · {path} ({_basis(shown)})", "ok"
    elif shown.status == "NO_GROUND":
        badge, tone = "지형: 바닥 미검출", "warn"
    else:
        badge, tone = f"지형: 센서 오류({shown.reason})", "error"
    obstacles: tuple[TerrainObstacle, ...] = ()
    note = _note(shown)
    fresh_ok = (
        display is not None
        and display.status == "OK"
        and display.depth_age_ms is not None
        and display.depth_age_ms / 1000.0 + aged <= DEPTH_STALE_S
    )
    if fresh_ok:
        if not video_live:
            note = f"윤곽 숨김: 영상 끊김 · {note}"
        elif video_size is None:
            note = f"윤곽 숨김: 영상 크기 미확인 · {note}"
        elif tuple(video_size) != display.frame_size:
            note = f"윤곽 숨김: 영상 {video_size[0]}×{video_size[1]} ≠ 1280×720 · {note}"
        else:
            obstacles = display.obstacles
    return TerrainView(badge, tone, note, obstacles, "LIVE")
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest -q -p no:cacheprovider operator_console/tests/test_terrain.py`
Expected: `25 passed`

- [ ] **Step 5: 커밋(사용자 확인 후)**

```bash
git add operator_console/terrain.py operator_console/tests/test_terrain.py
git commit -m "feat(console): L515 지형 패킷 세션 게이트와 표시 지연 수신기를 둔다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: 콘솔 GTK 배선 — `TerrainCanvas`, 다중 오버레이 `VideoPanel`, 체크박스, 설정

**Files:**
- Modify: `operator_console/app.py`, `operator_console/__main__.py`, `operator_console/integrated_runtime_smoke.py`, `operator_console/render_status_screenshot.py`
- Create: `operator_console/tests/test_terrain_gtk.py`

**Interfaces:**
- Consumes: `LatestTerrainReceiver`, `TerrainView`, `terrain_view`, 기존 `fit_overlay_transform`
- Produces:
  - `TerrainCanvas(receiver)`. 메서드: `set_video_size(w, h)`, `set_video_live(bool)`, `set_display_enabled(bool)`, `view(now_s=None) -> TerrainView`, `_layout(context, text, font, max_width)`(PangoCairo, 말줄임).
  - `VideoPanel(..., overlay_canvases=())`와 `VideoPanel.publish_video_size(w, h)`. GTK 스레드에서 자기 캔버스 모두에 크기를 전달한다. `overlay_canvases`는 대기 화면 위·헤더 아래 레이어에 pass-through로 붙고, `_refresh_video_health`가 영상 LIVE 여부를 `set_video_live`로 알린다.
  - `OperatorConsole(..., terrain_port=5009, terrain_display_delay_ms=0, ...)`
  - 새 속성: `_terrain_receiver`, `_terrain_canvas`, `_show_terrain`, `overlay_view_state["show_terrain"]`
  - 프로브 키 `terrain`(WAITING/LIVE/STALE), `terrain_badge`
  - CLI `--terrain-port`, `--terrain-display-delay-ms`(0–400 밖이면 `parser.error`)
  - 통합 설정 키 `terrain_port`, `terrain_display_delay_ms`

결정 사항(스펙 범위 안):
- "지형 장애물" 체크를 끄면 윤곽과 배지를 모두 그리지 않는다. 수신은 계속한다.
- 배지는 영상 영역의 왼쪽 아래에 둔다. 오른쪽 아래는 PiP, 오른쪽 위는 워터마크 자리다.
- D435 metadata 캔버스의 위치(영상 레이어 안)와 입력 처리(pass-through 미설정)는 그대로 둔다.
- 100 ms 재그리기 타이머는 캔버스 `destroy`에서 해제한다(Codex P3).

- [ ] **Step 1: 실패하는 GTK 시험 작성**

```python
"""Real GTK wiring of the L515 terrain overlay; run with xvfb-run + system python."""
import os
import socket
import time

os.environ.setdefault("GDK_BACKEND", "x11")

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.tests.test_terrain import payload  # noqa: E402


@pytest.fixture
def window(tmp_path):
    from operator_console.app import OperatorConsole, Gst
    if not Gtk.init_check()[0]:
        pytest.skip("display unavailable")
    Gst.init(None)
    console = OperatorConsole(
        "127.0.0.1", 51002, 51000, 0, 60, 0, 0,
        arm_telemetry_port=0, environment_telemetry_port=0, terrain_port=0,
        ops_token_file=str(tmp_path / "absent-token"),
    )
    console.show_all()
    yield console
    console.destroy()


def pump_until(predicate, timeout_s=3.0):
    from operator_console.app import GLib
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        while GLib.MainContext.default().pending():
            GLib.MainContext.default().iteration(False)
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_front_panel_hands_video_size_only_to_its_own_canvases(window):
    window._l515.publish_video_size(1280, 720)
    assert window._terrain_canvas._video_size == (1280, 720)
    assert window._d435._metadata_canvas._video_width is None
    window._d435.publish_video_size(848, 480)
    assert (window._d435._metadata_canvas._video_width,
            window._d435._metadata_canvas._video_height) == (848, 480)
    assert window._terrain_canvas._video_size == (1280, 720)


def test_live_packet_reaches_the_canvas_and_draws(window):
    import cairo
    window._l515.publish_video_size(1280, 720)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(payload(sequence=1), ("127.0.0.1", window._terrain_receiver.port))
        assert pump_until(lambda: window._terrain_receiver.latest() is not None)
    finally:
        sender.close()
    # Fake a fresh decoded frame at the video-health boundary (no real SRT).
    window._l515._pipeline_live = True
    window._l515._last_frame_monotonic = time.monotonic()
    window._l515._refresh_video_health()
    canvas = window._terrain_canvas
    # Judge 50 ms after reception: a loaded main loop may be slower than the 0.5 s link limit.
    view = canvas.view(now_s=window._terrain_receiver.last_rx_monotonic + 0.05)
    assert view.probe_state == "LIVE"
    assert view.badge.startswith("지형 OK")
    assert len(view.obstacles) == 1
    allocation = canvas.get_allocation()
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, allocation.width, allocation.height)
    canvas._on_draw(canvas, cairo.Context(surface))  # exceptions surface here
    surface.flush()
    assert any(surface.get_data())  # something was painted


def test_terrain_checkbox_only_hides_the_drawing(window):
    window._show_terrain.set_active(False)
    assert window.overlay_view_state["show_terrain"] is False
    assert window._terrain_canvas._enabled is False
    assert window._show_objects.get_active() is True  # D435 options untouched
    window._show_terrain.set_active(True)
    assert window._terrain_canvas._enabled is True


def test_badge_text_has_no_missing_glyphs(window):
    import cairo
    from operator_console.app import Pango, PangoCairo
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 400, 60)
    for text in ("지형: 수신 없음", "정합 미검증 · 참고용 · 낮은 장애물/구덩이 미표시"):
        layout = window._terrain_canvas._layout(cairo.Context(surface), text, "Sans 14", 380)
        assert layout.get_unknown_glyphs_count() == 0, text


def test_terrain_canvas_sits_above_the_video_standby_layer(window):
    stage = window._terrain_canvas.get_parent()
    children = stage.get_children()
    standby = window._l515._placeholder
    assert stage is standby.get_parent()
    assert children.index(window._terrain_canvas) > children.index(standby)
    assert stage.get_overlay_pass_through(window._terrain_canvas) is True


def test_video_health_drives_outline_visibility(window):
    window._l515._pipeline_live = False
    window._l515._refresh_video_health()
    assert window._terrain_canvas._video_live is False


def _fake_live_video(panel):
    panel._pipeline_live = True
    panel._last_frame_monotonic = time.monotonic()
    panel._refresh_video_health()


@pytest.mark.parametrize("kind", ["error", "eos", "restart"])
def test_video_loss_drops_outlines_immediately(window, kind):
    from operator_console.app import GLib, Gst
    panel = window._l515
    _fake_live_video(panel)
    assert window._terrain_canvas._video_live is True
    if kind == "error":
        error = GLib.Error.new_literal(Gst.ResourceError.quark(), "fixture", Gst.ResourceError.READ)
        panel._on_bus_message(None, Gst.Message.new_error(panel._pipeline, error, "fixture"))
    elif kind == "eos":
        panel._on_bus_message(None, Gst.Message.new_eos(panel._pipeline))
    else:
        panel._restart_pipeline()
    # No periodic health refresh in between: the change must be immediate.
    assert window._terrain_canvas._video_live is False
```

- [ ] **Step 2: 실패 확인**

Run: `env -u WAYLAND_DISPLAY GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m pytest -q -p no:cacheprovider -W ignore operator_console/tests/test_terrain_gtk.py`
Expected: FAIL — `TypeError: OperatorConsole.__init__() got an unexpected keyword argument 'terrain_port'`

- [ ] **Step 3: `app.py` 수정**

```diff
--- a/operator_console/app.py
+++ b/operator_console/app.py
@@ -51,7 +51,8 @@
 gi.require_version("Gdk", "3.0")
 gi.require_version("Gst", "1.0")
 gi.require_version("Rsvg", "2.0")
-from gi.repository import Gdk, GLib, Gst, Gtk, Pango, Rsvg  # noqa: E402
+gi.require_version("PangoCairo", "1.0")
+from gi.repository import Gdk, GLib, Gst, Gtk, Pango, PangoCairo, Rsvg  # noqa: E402
 
 from .arm_telemetry import (
     ArmTelemetrySnapshot,
@@ -81,6 +82,7 @@
     target_distance_m,
 )
 from .mobility_view import MobilityDashboard, mission_mobility_values
+from .terrain import LatestTerrainReceiver, TerrainView, terrain_view
 from .ops_client import ConsoleOpsClient
 from .ops_panel import (
     GESTURE_HOLD,
@@ -1734,6 +1736,118 @@
         return False
 
 
+TERRAIN_TONES = {
+    "ok": (0.0, 0.835, 1.0),
+    "warn": (1.0, 0.72, 0.0),
+    "error": (1.0, 0.32, 0.32),
+    "muted": (0.75, 0.78, 0.82),
+}
+
+
+class TerrainCanvas(Gtk.DrawingArea):
+    """Transparent L515 terrain outlines + status badge (operator aid only).
+
+    Sits above the video standby layer so the badge stays visible while the
+    SRT video is connecting or stale; outlines are hidden in that case.
+    Text goes through PangoCairo — cairo's toy text API has no Hangul
+    fallback and renders the Korean badge as empty boxes.
+    """
+
+    def __init__(self, receiver: LatestTerrainReceiver) -> None:
+        super().__init__()
+        self._receiver = receiver
+        self._enabled = True
+        self._video_size: tuple[int, int] | None = None
+        self._video_live = False
+        self.connect("draw", self._on_draw)
+        self._redraw_source = GLib.timeout_add(100, self._redraw)
+        self.connect("destroy", self._on_destroy)
+
+    def set_video_size(self, width: int, height: int) -> None:
+        size = (int(width), int(height))
+        if size != self._video_size:
+            self._video_size = size
+            self.queue_draw()
+
+    def set_video_live(self, live: bool) -> None:
+        if bool(live) != self._video_live:
+            self._video_live = bool(live)
+            self.queue_draw()  # drop outlines the moment the standby layer appears
+
+    def set_display_enabled(self, enabled: bool) -> None:
+        self._enabled = bool(enabled)
+        self.queue_draw()
+
+    def view(self, now_s: float | None = None) -> TerrainView:
+        now_s = time.monotonic() if now_s is None else now_s
+        latest, display, last_rx_s = self._receiver.snapshot(now_s)
+        return terrain_view(
+            latest, display, last_rx_s, now_s, self._receiver.display_delay_s,
+            self._video_size, video_live=self._video_live,
+        )
+
+    def _redraw(self) -> bool:
+        self.queue_draw()
+        return True
+
+    def _on_destroy(self, *_args: object) -> None:
+        if self._redraw_source is not None:
+            GLib.source_remove(self._redraw_source)
+            self._redraw_source = None
+
+    def _layout(self, context: object, text: str, font: str, max_width: float):
+        layout = PangoCairo.create_layout(context)
+        layout.set_font_description(Pango.FontDescription(font))
+        layout.set_width(int(max(40.0, max_width) * Pango.SCALE))
+        layout.set_ellipsize(Pango.EllipsizeMode.END)
+        layout.set_text(text, -1)
+        return layout
+
+    def _on_draw(self, _widget: Gtk.DrawingArea, context: object) -> bool:
+        if not self._enabled:
+            return False
+        view = self.view()
+        allocation = self.get_allocation()
+        frame_width, frame_height = self._video_size or (1280, 720)
+        scale, offset_x, offset_y = fit_overlay_transform(
+            allocation.width, allocation.height, frame_width, frame_height,
+        )
+        for obstacle in view.obstacles:
+            red = obstacle.in_path
+            rgb = (1.0, 0.25, 0.25) if red else (1.0, 0.6, 0.0)
+            first, *rest = obstacle.poly
+            context.move_to(offset_x + first[0] * scale, offset_y + first[1] * scale)
+            for x, y in rest:
+                context.line_to(offset_x + x * scale, offset_y + y * scale)
+            context.close_path()
+            context.set_source_rgba(*rgb, 0.22)
+            context.fill_preserve()
+            context.set_source_rgba(*rgb, 0.95)
+            context.set_line_width(3.0 if red else 2.0)
+            context.stroke()
+            top = min(obstacle.poly, key=lambda point: point[1])
+            label = self._layout(context, f"{obstacle.dist_m:.2f} m", "Sans Bold 13", 120.0)
+            context.move_to(offset_x + top[0] * scale, max(0.0, offset_y + top[1] * scale - 22.0))
+            PangoCairo.show_layout(context, label)
+        left = offset_x + 12.0
+        max_width = max(40.0, frame_width * scale - 24.0)
+        badge = self._layout(context, view.badge, "Sans Bold 14", max_width)
+        note = self._layout(context, view.note, "Sans 11", max_width)
+        badge_w, badge_h = badge.get_pixel_size()
+        note_w, note_h = note.get_pixel_size()
+        top = offset_y + frame_height * scale - 12.0 - badge_h - note_h - 8.0
+        context.set_source_rgba(0.0, 0.059, 0.094, 0.82)
+        context.rectangle(left - 6.0, top - 4.0, max(badge_w, note_w) + 12.0, badge_h + note_h + 12.0)
+        context.fill()
+        context.set_source_rgba(*TERRAIN_TONES[view.tone], 1.0)
+        context.move_to(left, top)
+        PangoCairo.show_layout(context, badge)
+        context.set_source_rgba(0.85, 0.88, 0.92, 0.95)
+        context.move_to(left, top + badge_h + 4.0)
+        PangoCairo.show_layout(context, note)
+        return False
+
+
 class ScanRing(Gtk.DrawingArea):
     """Static, restrained standby reticle for the camera empty state."""
 
@@ -2303,7 +2417,8 @@
     def __init__(self, name: str, host: str, port: int, latency_ms: int,
                  metadata_receiver: LatestMetadataReceiver | None = None,
                  target_tracker: DisplayTargetTracker | None = None,
-                 event_sink: Callable[[str, str], None] | None = None) -> None:
+                 event_sink: Callable[[str, str], None] | None = None,
+                 overlay_canvases: tuple[Gtk.Widget, ...] = ()) -> None:
         super().__init__(orientation=Gtk.Orientation.VERTICAL)
         _style(self, "video-card")
         self._name = name
@@ -2352,6 +2467,12 @@
         header_click.set_halign(Gtk.Align.FILL)
         header_click.set_valign(Gtk.Align.START)
         _style(header_click, "video-overlay")
+        # Every overlay canvas receives the negotiated video size.  The D435
+        # metadata canvas stays inside the video layer (unchanged); extra
+        # canvases sit above the standby layer (see video_stage below).
+        self._overlay_canvases: list[Gtk.Widget] = []
+        self._stage_canvases = tuple(overlay_canvases)
+        self._video_live_published: bool | None = None
         if metadata_receiver is None:
             visual = self._video_widget
         else:
@@ -2364,7 +2485,10 @@
             canvas.set_vexpand(True)
             metadata_overlay.add_overlay(canvas)
             self._metadata_canvas = canvas
+            self._overlay_canvases.append(canvas)
             visual = metadata_overlay
+        self._overlay_canvases.extend(self._stage_canvases)
+        if metadata_receiver is not None:
             self._metadata_receiver = metadata_receiver
             self._metadata_label = Gtk.Label(label="YOLO: UDP :5003 대기 중")
             self._metadata_state: tuple[str, int] | None = None
@@ -2392,6 +2516,11 @@
         self._rover_status_overlay = placeholder.rover
         standby.add_overlay(placeholder)
         video_stage.add_overlay(standby)
+        for canvas in self._stage_canvases:
+            canvas.set_hexpand(True)
+            canvas.set_vexpand(True)
+            video_stage.add_overlay(canvas)
+            video_stage.set_overlay_pass_through(canvas, True)
         video_stage.add_overlay(header_click)
         self._placeholder = standby
         self.pack_start(video_stage, True, True, 0)
@@ -2481,6 +2610,21 @@
                 show_distance=show_distance,
             )
 
+    def _publish_video_live(self, live: bool) -> None:
+        if live == self._video_live_published:
+            return
+        self._video_live_published = live
+        for canvas in self._stage_canvases:
+            setter = getattr(canvas, "set_video_live", None)
+            if setter is not None:
+                setter(live)
+
+    def publish_video_size(self, width: int, height: int) -> bool:
+        """GTK thread: hand the negotiated size to every overlay canvas."""
+        for canvas in self._overlay_canvases:
+            canvas.set_video_size(width, height)
+        return False
+
     @property
     def last_fps(self) -> float | None:
         return self._last_fps
@@ -2517,9 +2661,7 @@
                 and video_size != self._video_size
             ):
                 self._video_size = video_size
-                canvas = getattr(self, "_metadata_canvas", None)
-                if canvas is not None:
-                    GLib.idle_add(canvas.set_video_size, width, height)
+                GLib.idle_add(self.publish_video_size, width, height)
         now = time.monotonic()
         self._last_frame_monotonic = now
         elapsed = now - self._sample_start
@@ -2534,6 +2676,7 @@
     def _on_bus_message(self, _bus: Gst.Bus, message: Gst.Message) -> None:
         if message.type == Gst.MessageType.ERROR:
             self._placeholder.show_all()
+            self._publish_video_live(False)
             error, detail = message.parse_error()
             self._pipeline_live = False
             self._status.set_text(f"{self._name} 연결 대기")
@@ -2541,6 +2684,7 @@
             self._schedule_reconnect()
         elif message.type == Gst.MessageType.EOS:
             self._placeholder.show_all()
+            self._publish_video_live(False)
             self._pipeline_live = False
             self._status.set_text(f"{self._name} 연결 대기")
             self._schedule_reconnect()
@@ -2557,6 +2701,7 @@
     def _refresh_video_health(self) -> bool:
         if self._stopped:
             return False
+        self._publish_video_live(self.health_state() == "LIVE")
         if not self._pipeline_live:
             self._placeholder.show_all()
             self._header_state.set_text("연결 대기")
@@ -2616,6 +2761,7 @@
         # 다시 발화하고 표시도 거대한 stale 나이를 보인다 — 상태를 리셋한다.
         self._pipeline_live = False
         self._last_frame_monotonic = None
+        self._publish_video_live(False)
         self._last_fps = None
         self._frames = 0
         self._sample_start = time.monotonic()
@@ -3668,6 +3814,8 @@
                  latency_ms: int, telemetry_port: int, chassis_telemetry_port: int,
                  arm_telemetry_port: int = 5007,
                  environment_telemetry_port: int = 5008,
+                 terrain_port: int = 5009,
+                 terrain_display_delay_ms: int = 0,
                  ops_host: str | None = None, ops_port: int = 9001,
                  ops_token_file: str = DEFAULT_OPS_TOKEN_FILE,
                  smoke_probe_file: str | None = None,
@@ -3699,6 +3847,10 @@
         self._environment_receiver = LatestEnvironmentTelemetryReceiver(
             environment_telemetry_port,
         )
+        self._terrain_receiver = LatestTerrainReceiver(
+            terrain_port, display_delay_s=terrain_display_delay_ms / 1000.0,
+        )
+        self._terrain_canvas = TerrainCanvas(self._terrain_receiver)
         self._events = EventLog()
         self._mission_events = EventLog()
         self._d435 = VideoPanel("작업 카메라", host, d435_port, latency_ms,
@@ -3706,7 +3858,8 @@
                                 target_tracker=self._display_target_tracker,
                                 event_sink=self._add_event)
         self._l515 = VideoPanel("전방 카메라", host, l515_port, latency_ms,
-                                event_sink=self._add_event)
+                                event_sink=self._add_event,
+                                overlay_canvases=(self._terrain_canvas,))
         # Fixed judge layout: forward view owns the stage; work view is 848:480 PiP.
         videos = Gtk.Overlay()
         videos.set_hexpand(True)
@@ -3760,19 +3913,25 @@
             label="인식 대상"
         )
         self._show_distance = Gtk.CheckButton(label="대상 거리")
+        self._show_terrain = Gtk.CheckButton(label="지형 장애물")
         self.overlay_view_state = {
             "show_detection": True,
             "show_distance": True,
+            "show_terrain": True,
         }
         self._show_objects.set_active(True)
         self._show_distance.set_active(True)
+        self._show_terrain.set_active(True)
         _style(self._show_objects, "display-option")
         _style(self._show_distance, "display-option")
+        _style(self._show_terrain, "display-option")
         self._show_objects.connect("toggled", self._on_display_option_toggled)
         self._show_distance.connect("toggled", self._on_display_option_toggled)
+        self._show_terrain.connect("toggled", self._on_display_option_toggled)
         display_options.pack_start(display_title, False, False, 0)
         display_options.pack_start(self._show_objects, False, False, 0)
         display_options.pack_start(self._show_distance, False, False, 0)
+        display_options.pack_start(self._show_terrain, False, False, 0)
         self._display_options = display_options
         videos.connect("size-allocate", self._on_video_area_allocated)
         self._videos = videos
@@ -4728,6 +4887,12 @@
         self.overlay_view_state["show_distance"] = (
             self._show_distance.get_active()
         )
+        self.overlay_view_state["show_terrain"] = (
+            self._show_terrain.get_active()
+        )
+        self._terrain_canvas.set_display_enabled(
+            self.overlay_view_state["show_terrain"],
+        )
         self._d435.set_metadata_display_options(
             show_objects=self.overlay_view_state["show_detection"],
             show_distance=self.overlay_view_state["show_distance"],
@@ -5500,12 +5665,15 @@
             self._readiness_detail.set_text(
                 " · ".join(waiting_items) + "의 연결을 확인하고 있습니다"
             )
+        terrain_view_now = self._terrain_canvas.view()
         self._write_smoke_probe({
             "telemetry": telemetry,
             "chassis": chassis_state,
             "metadata": yolo,
             "arm": arm_state,
             "environment": environment_state,
+            "terrain": terrain_view_now.probe_state,
+            "terrain_badge": terrain_view_now.badge,
             "environment_climate": self._environment_status.probe_values()[0],
             "environment_air": self._environment_status.probe_values()[1],
             "environment_hazard": self._environment_status.probe_values()[2],
@@ -5580,6 +5748,7 @@
         self._chassis_receiver.close()
         self._arm_receiver.close()
         self._environment_receiver.close()
+        self._terrain_receiver.close()
         self._ops_panel.close()
         if Gtk.main_level():
             Gtk.main_quit()
@@ -5634,6 +5803,11 @@
     parser.add_argument("--chassis-telemetry-port", type=int, default=5005)
     parser.add_argument("--arm-telemetry-port", type=int, default=5007)
     parser.add_argument("--environment-telemetry-port", type=int, default=5008)
+    parser.add_argument("--terrain-port", type=int, default=5009)
+    parser.add_argument(
+        "--terrain-display-delay-ms", type=int, default=0,
+        help="L515 terrain outline delay to match SRT video (0-400 ms)",
+    )
     parser.add_argument(
         "--ops-host",
         default=None,
@@ -5659,12 +5833,16 @@
         help="developer-info provenance label only; does not change receivers",
     )
     args = parser.parse_args()
+    if not 0 <= args.terrain_display_delay_ms <= 400:
+        parser.error("--terrain-display-delay-ms must be within 0-400")
     Gst.init(None)
     console = OperatorConsole(args.host, args.d435_port, args.l515_port,
                               args.metadata_port, args.latency_ms, args.telemetry_port,
                               args.chassis_telemetry_port,
                               args.arm_telemetry_port,
                               args.environment_telemetry_port,
+                              terrain_port=args.terrain_port,
+                              terrain_display_delay_ms=args.terrain_display_delay_ms,
                               ops_host=args.host if args.ops_host is None else args.ops_host,
                               ops_port=args.ops_port,
                               ops_token_file=args.ops_token_file,
```

- [ ] **Step 4: 통합 진입점·스모크 설정·스크린샷 도구에 포트 전달**

```diff
--- a/operator_console/__main__.py
+++ b/operator_console/__main__.py
@@ -21,7 +21,8 @@
             raise ValueError('token_file이 비어 있습니다')
         defaults = dict(d435_port=5002, l515_port=5000, metadata_port=5003,
                         telemetry_port=5004, chassis_telemetry_port=5005,
-                        arm_telemetry_port=5007, environment_telemetry_port=5008)
+                        arm_telemetry_port=5007, environment_telemetry_port=5008,
+                        terrain_port=5009)
         ports = {key: config.get(key, value) for key, value in defaults.items()}
         for key, value in ports.items():
             if type(value) is not int or not 1 <= value <= 65535:
@@ -29,6 +30,9 @@
         latency = config.get('latency_ms', 60)
         if type(latency) is not int or not 0 <= latency <= 10000:
             raise ValueError('latency_ms: 0–10000 정수가 필요합니다')
+        terrain_delay = config.get('terrain_display_delay_ms', 0)
+        if type(terrain_delay) is not int or not 0 <= terrain_delay <= 400:
+            raise ValueError('terrain_display_delay_ms: 0–400 정수가 필요합니다')
         lock_root = Path(os.environ.get('XDG_RUNTIME_DIR', str(Path.home() / '.cache')))
         with ConsoleInstanceLock(lock_root / 'powertrain' / 'operator-console.lock'):
             from operator_console.app import OperatorConsole, Gst, Gtk, GLib, _add_unix_signal_watch
@@ -37,6 +41,7 @@
                 raise RuntimeError('화면에 연결할 수 없습니다. 데스크톱 세션에서 실행하세요')
             runtime = OperationRuntime(config)
             console = OperatorConsole('127.0.0.1', latency_ms=latency, **ports,
+                terrain_display_delay_ms=terrain_delay,
                 ops_token_file=config['token_file'], operation_runtime=runtime,
                 smoke_probe_file=args.smoke_probe_file)
             console.show_all()
```

```diff
--- a/operator_console/integrated_runtime_smoke.py
+++ b/operator_console/integrated_runtime_smoke.py
@@ -58,7 +58,8 @@
             session_port=session_port, token_file=str(token), controller_python=str(_controller_fixture(base, controller_python or sys.executable)),
             metadata_port=_free_udp_port(), telemetry_port=_free_udp_port(),
             chassis_telemetry_port=_free_udp_port(), arm_telemetry_port=_free_udp_port(),
-            environment_telemetry_port=_free_udp_port())))
+            environment_telemetry_port=_free_udp_port(),
+            terrain_port=_free_udp_port())))
         probe = base / 'probe.json'
         env = dict(os.environ, XDG_RUNTIME_DIR=str(base), GDK_BACKEND='x11',
                    GDK_SCALE='1', GDK_DPI_SCALE='1', SDL_VIDEODRIVER='dummy',
```

```diff
--- a/operator_console/render_status_screenshot.py
+++ b/operator_console/render_status_screenshot.py
@@ -68,6 +68,7 @@
         "127.0.0.1", _free_port(), _free_port(), ports["metadata"], 60,
         ports["power"], ports["chassis"], ports["arm"],
         environment_telemetry_port=ports["environment"],
+        terrain_port=_free_port(),
         ops_host="127.0.0.1", ops_port=_free_port(),
         ops_token_file=token_path,
     )
```

- [ ] **Step 5: 통과 확인**

Run: `env -u WAYLAND_DISPLAY GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m pytest -q -p no:cacheprovider -W ignore operator_console/tests/test_terrain_gtk.py operator_console/tests/test_integrated_app.py`
Expected: `28 passed` (GTK 지형 9 + 통합 앱 19). 지형 GTK 시험에는 한글 글리프 누락 0, 대기 화면 위 레이어 배치, 영상 상태에 따른 윤곽 숨김, ERROR·EOS·재시작 직후 즉시 숨김이 들어 있다.

- [ ] **Step 6: 커밋(사용자 확인 후)**

```bash
git add operator_console/app.py operator_console/__main__.py operator_console/integrated_runtime_smoke.py operator_console/render_status_screenshot.py operator_console/tests/test_terrain_gtk.py
git commit -m "feat(console): 전방 영상 위 L515 지형 윤곽 캔버스와 표시 옵션을 연결한다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 8: 실행 스모크와 데모 송신기에 terrain 채널

**Files:**
- Modify: `operator_console/runtime_smoke.py`, `operator_console/demo_source.py`, `operator_console/tests/test_demo_source.py`

**Interfaces:**
- Consumes: Task 7의 프로브 키 `terrain`·`terrain_badge`, CLI `--terrain-port`
- Produces:
  - `runtime_smoke.REQUIRED_PANELS`에 `"terrain"`, `TERRAIN_SMOKE_SESSION`, `_terrain_payload(sequence, *, status="OK")`, `_probe_terrain_badge(path)`
  - `demo_source.DEFAULT_TEST_PORTS["terrain"] = 15009`, `LIVE_PORTS`에 5009
  - `build_payloads(sequence, elapsed_s, terrain_session=DEMO_TERRAIN_SESSION)`. 키 `"terrain"`이 추가되고, `run()`은 실행마다 새 세션을 쓴다.

- [ ] **Step 1: 실패하는 시험 수정**

```diff
--- a/operator_console/tests/test_demo_source.py
+++ b/operator_console/tests/test_demo_source.py
@@ -12,6 +12,7 @@
 from operator_console.environment_telemetry import parse_environment_telemetry
 from operator_console.metadata import parse_metadata
 from operator_console.telemetry import parse_telemetry
+from terrain_overlay.contract import MAX_PACKET_BYTES, parse_packet
 
 
 def _raw(payload: dict) -> bytes:
@@ -48,11 +49,17 @@
     assert parse_metadata(encoded["metadata"], 10.0).detections[0].depth_m == 1.28
     assert parse_arm_telemetry(encoded["arm"], 10.0).end_effector_type == "dual_gripper"
     assert parse_environment_telemetry(encoded["environment"], 10.0).sensor_ok is True
+    assert len(encoded["terrain"]) <= MAX_PACKET_BYTES
+    assert parse_packet(encoded["terrain"]).obstacles[0].in_path is False
+    blocked = parse_packet(_raw(build_payloads(12, 19.0)["terrain"]))
+    assert blocked.obstacles[0].in_path is True and blocked.zones["center"] == 0.62
 
 
 def test_demo_source_rejects_live_or_colliding_ports():
     validate_ports(dict(DEFAULT_TEST_PORTS))
     with pytest.raises(ValueError, match="live UDP ports"):
         validate_ports(dict(DEFAULT_TEST_PORTS, chassis=5005))
+    with pytest.raises(ValueError, match="live UDP ports"):
+        validate_ports(dict(DEFAULT_TEST_PORTS, terrain=5009))
     with pytest.raises(ValueError, match="distinct"):
         validate_ports(dict(DEFAULT_TEST_PORTS, chassis=15004))
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest -q -p no:cacheprovider operator_console/tests/test_demo_source.py`
Expected: FAIL — `KeyError: 'terrain'`

- [ ] **Step 3: 데모 송신기와 스모크 수정**

```diff
--- a/operator_console/demo_source.py
+++ b/operator_console/demo_source.py
@@ -10,6 +10,7 @@
 import argparse
 import json
 import math
+import secrets
 import signal
 import socket
 import time
@@ -22,8 +23,12 @@
     "chassis": 15005,
     "arm": 15007,
     "environment": 15008,
+    "terrain": 15009,
 }
-LIVE_PORTS = frozenset({5003, 5004, 5005, 5007, 5008})
+LIVE_PORTS = frozenset({5003, 5004, 5005, 5007, 5008, 5009})
+# Fixed placeholder session; ``run`` gives every demo run its own session so
+# the console's terrain session gate accepts a restarted demo at once.
+DEMO_TERRAIN_SESSION = ("0" * 32, 0, "0" * 16)
 WHEEL_NAMES = (
     "front_left", "front_right", "mid_left",
     "mid_right", "rear_left", "rear_right",
@@ -82,7 +87,39 @@
     return payloads
 
 
-def build_payloads(sequence: int, elapsed_s: float) -> dict[str, dict]:
+def _terrain_payload(
+    sequence: int, scenario: str, session: tuple[str, int, str],
+) -> dict:
+    """L515 terrain packet v1: one outline, in the path only when blocked."""
+    boot_id, start_ns, instance = session
+    blocked = scenario == "BLOCKED_HOLD"
+    distance = 0.62 if blocked else 2.1
+    poly = (
+        [[520, 330], [760, 330], [760, 520], [520, 520]] if blocked
+        else [[930, 300], [1030, 300], [1030, 380], [930, 380]]
+    )
+    return {
+        "schema_version": 1, "type": "l515_terrain",
+        "sender_boot_id": boot_id, "sender_start_mono_ns": start_ns,
+        "sender_instance": instance, "sequence": sequence,
+        "sent_unix_ns": time.time_ns(), "depth_stamp_ns": time.time_ns(),
+        "depth_age_ms": 35.0, "status": "OK", "reason": None,
+        "frame_size": [1280, 720], "distance_origin": "l515_depth_optical",
+        "alignment_verified": False,
+        "ground": {"state": "FRESH", "age_ms": 35.0,
+                   "camera_height_m": 0.5, "camera_tilt_deg": 20.0},
+        "path_half_width_m": 0.4445,
+        "zones": {"left": None, "center": distance if blocked else None,
+                  "right": None if blocked else distance},
+        "obstacles": [{"dist_m": distance, "in_path": blocked, "poly": poly}],
+        "proc_ms": 20.0,
+    }
+
+
+def build_payloads(
+    sequence: int, elapsed_s: float,
+    terrain_session: tuple[str, int, str] = DEMO_TERRAIN_SESSION,
+) -> dict[str, dict]:
     """Build one coherent multi-channel frame for a deterministic scenario."""
     scenario = scenario_at(elapsed_s)
     path_available = scenario != "BLOCKED_HOLD"
@@ -254,6 +291,7 @@
         "chassis": chassis,
         "arm": arm,
         "environment": environment,
+        "terrain": _terrain_payload(sequence, scenario, terrain_session),
     }
 
 
@@ -281,6 +319,7 @@
         raise ValueError("duration_s must be greater than zero")
     sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
     started = time.monotonic()
+    terrain_session = ("0" * 32, time.monotonic_ns(), secrets.token_hex(8))
     sequence = 0
     try:
         while (
@@ -289,7 +328,8 @@
         ):
             elapsed = time.monotonic() - started
             sequence += 1
-            for channel, payload in build_payloads(sequence, elapsed).items():
+            payloads = build_payloads(sequence, elapsed, terrain_session)
+            for channel, payload in payloads.items():
                 raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
                 if len(raw) > 4096:
                     raise RuntimeError(f"{channel} demo datagram exceeds 4096 bytes")
```

```diff
--- a/operator_console/runtime_smoke.py
+++ b/operator_console/runtime_smoke.py
@@ -38,8 +38,10 @@
 # 이 패널들은 주입 중 LIVE 에 도달하고, 주입을 끊으면 STALE 로 전이해야 한다.
 # 하나라도 못 하면 그 채널은 실제로는 죽어 있는 것이다.
 REQUIRED_PANELS = frozenset({
-    "telemetry", "chassis", "metadata", "arm", "environment",
+    "telemetry", "chassis", "metadata", "arm", "environment", "terrain",
 })
+# One terrain sender session per smoke run (spec §4.6 session gate).
+TERRAIN_SMOKE_SESSION = ("5" * 32, time.monotonic_ns(), "5" * 16)
 
 
 def smoke_child_env(
@@ -90,6 +92,14 @@
     return str(value) if value is not None else None
 
 
+def _probe_terrain_badge(probe_file: Path) -> str | None:
+    try:
+        states = json.loads(probe_file.read_text(encoding="utf-8"))
+        return str(states["terrain_badge"])
+    except (OSError, ValueError, KeyError, TypeError):
+        return None
+
+
 def _probe_rover_widths(probe_file: Path) -> tuple[int, int] | None:
     try:
         states = json.loads(probe_file.read_text(encoding="utf-8"))
@@ -330,6 +340,35 @@
     }
 
 
+def _terrain_payload(sequence: int, *, status: str = "OK") -> dict:
+    boot_id, start_ns, instance = TERRAIN_SMOKE_SESSION
+    payload = {
+        "schema_version": 1, "type": "l515_terrain",
+        "sender_boot_id": boot_id, "sender_start_mono_ns": start_ns,
+        "sender_instance": instance, "sequence": sequence,
+        "sent_unix_ns": time.time_ns(), "depth_stamp_ns": sequence * 10**8,
+        "depth_age_ms": 30.0, "status": "OK", "reason": None,
+        "frame_size": [1280, 720], "distance_origin": "l515_depth_optical",
+        "alignment_verified": False,
+        "ground": {"state": "FRESH", "age_ms": 30.0,
+                   "camera_height_m": 0.5, "camera_tilt_deg": 20.0},
+        "path_half_width_m": 0.4445,
+        "zones": {"left": None, "center": 1.2, "right": 2.1},
+        "obstacles": [
+            {"dist_m": 1.2, "in_path": True,
+             "poly": [[560, 300], [720, 300], [720, 420], [560, 420]]},
+            {"dist_m": 2.1, "in_path": False,
+             "poly": [[900, 280], [980, 280], [940, 340]]},
+        ],
+        "proc_ms": 25.0,
+    }
+    if status != "OK":
+        payload.update(status=status, reason="DEPTH_STALE", ground=None,
+                       zones={"left": None, "center": None, "right": None},
+                       obstacles=[], proc_ms=None)
+    return payload
+
+
 def _arm_payload(sequence: int) -> dict:
     return {
         "schema_version": 1, "sequence": sequence, "stamp_s": time.time(),
@@ -405,7 +444,7 @@
     ports = {
         "metadata": _free_udp_port(), "telemetry": _free_udp_port(),
         "chassis": _free_udp_port(), "arm": _free_udp_port(),
-        "environment": _free_udp_port(),
+        "environment": _free_udp_port(), "terrain": _free_udp_port(),
     }
     ops_fixture = _OpsStateFixture()
     ops_fixture.start()
@@ -430,6 +469,7 @@
                 "--chassis-telemetry-port", str(ports["chassis"]),
                 "--arm-telemetry-port", str(ports["arm"]),
                 "--environment-telemetry-port", str(ports["environment"]),
+                "--terrain-port", str(ports["terrain"]),
                 "--ops-token-file", token_file,
                 "--ops-port", str(ops_fixture.port),
                 "--smoke-probe-file", str(probe_file),
@@ -448,13 +488,14 @@
     builders = {
         "telemetry": _telemetry_payload, "chassis": _chassis_payload,
         "metadata": _metadata_payload, "arm": _arm_payload,
-        "environment": _environment_payload,
+        "environment": _environment_payload, "terrain": _terrain_payload,
     }
     live_seen: set[str] = set()
     stale_seen: set[str] = set()
     unexpected_auto_swap_seen = False
     role_sized_rovers_seen = False
     environment_values_seen = False
+    terrain_ok_seen = False
     invalid_steering_held_seen = False
     arm_tabs_seen = False
     try:
@@ -494,6 +535,7 @@
                 _probe_main_video(probe_file) == "작업 카메라"
             )
             role_sized_rovers_seen |= _probe_rover_widths(probe_file) == (290, 90)
+            terrain_ok_seen |= (_probe_terrain_badge(probe_file) or "").startswith("지형 OK")
             environment_values = _probe_environment_values(probe_file)
             if environment_values is not None:
                 climate, air, hazard = environment_values
@@ -532,6 +574,7 @@
                 "schema_version": 1, "sequence": sequence,
                 "sensor_ok": False, "errors": ["fixture sparse packet"],
             },
+            "terrain": _terrain_payload(sequence, status="SENSOR_ERROR"),
         }
         for channel, payload in sparse.items():
             if channel == omit_channel_for_test:
@@ -605,6 +648,8 @@
             f"panels never went STALE after injection stopped: "
             f"{', '.join(missing_stale)}\n{text}"
         )
+    if not terrain_ok_seen:
+        return False, "terrain badge never showed 지형 OK while injecting\n" + text
     if unexpected_auto_swap_seen:
         return False, "D435i became MAIN without an operator click\n" + text
     if not role_sized_rovers_seen:
@@ -618,7 +663,7 @@
     if not arm_tabs_seen:
         return False, "arm tabs did not render live tool in safe disabled state\n" + text
     return True, (
-        f"PASS · {sequence} ticks on 5 channels · "
+        f"PASS · {sequence} ticks on 6 channels · "
         f"LIVE+STALE observed on {', '.join(sorted(REQUIRED_PANELS))} · "
         "invalid steering mode held disabled · "
         "arm tabs observed safely disabled · "
```

- [ ] **Step 4: 단위 시험 통과**

Run: `/usr/bin/python3 -m pytest -q -p no:cacheprovider operator_console/tests/test_demo_source.py`
Expected: `3 passed`

- [ ] **Step 5: 실행 스모크(프로젝트 §5 완료 규칙)**

Run: `/usr/bin/python3 -m operator_console.runtime_smoke`
Expected, 둘 중 하나:
- `PASS · … on 6 channels …`
- 기존 결함 때문에 `FAIL` + `arm tabs did not render live tool in safe disabled state`. 2026-10-17 실측은 이쪽이다.

지형 단언(traceback 없음, terrain LIVE→STALE, "지형 OK" 배지)은 모두 arm tabs 단언보다 **앞에** 있다. 그래서 다른 실패 사유가 나오면 회귀다.

- [ ] **Step 6: 게이트 음성 대조(지형 채널 주입 생략 → FAIL)**

Run: `/usr/bin/python3 -c "from operator_console.runtime_smoke import run_smoke; ok, r = run_smoke(omit_channel_for_test='terrain'); print(ok); print(r.splitlines()[0])"`
Expected: `False` / `panels never reached LIVE: terrain`

- [ ] **Step 7: 커밋(사용자 확인 후)**

```bash
git add operator_console/runtime_smoke.py operator_console/demo_source.py operator_console/tests/test_demo_source.py
git commit -m "feat(console): 실행 스모크와 데모 송신기에 L515 지형 채널을 넣는다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: `lidar_dev/` 정리 — 오프라인 도구를 새 코어로

**Files:**
- Delete: `lidar_dev/terrain.py`, `lidar_dev/packet.py`, `lidar_dev/terrain_node.py`, `lidar_dev/before.txt`, `lidar_dev/after.txt`, `lidar_dev/requirements.txt`
- Create: `lidar_dev/recording.py`, `lidar_dev/README.md`, `lidar_dev/tests/test_offline_tools.py`
- Rewrite: `lidar_dev/overlay.py`, `lidar_dev/scan_terrain.py`, `lidar_dev/ground.py`, `lidar_dev/view_depth.py`, `lidar_dev/record.py`

**Interfaces:**
- Consumes: `CameraModel`, `DetectorConfig`, `TerrainDetector`, `scenes`(시험)
- Produces:
  - `load_recording(folder) -> Recording`, `make_detector(recording, **config)`, `iter_frames(recording)`, `read_depth(path)`
  - `overlay.render(depth, result, depth_scale_m)`, `scan_terrain.summarize(folder) -> dict`, `ground.analyse(folder, index)`
  - 각 도구의 `main(argv)`. 실행은 저장소 루트에서 `python -m lidar_dev.<tool>`로 한다.

삭제 근거:
- `terrain.py`·`packet.py`·`terrain_node.py`는 새 코어로 대체된다.
- `before.txt`·`after.txt`는 빈 파일이다.
- `requirements.txt`는 macOS pip freeze다(스펙 §3.2).

`ground.py`의 "장애물 주 평면 각도" 분석(open3d RANSAC)은 옮기지 않는다. 판정기 범위 밖이고, open3d 없이는 돌 수 없기 때문이다.

- [ ] **Step 1: 실패하는 시험 작성**

```python
"""lidar_dev offline tools on a synthetic recording (conda: numpy + cv2)."""
import json

import cv2
import pytest

from lidar_dev import ground, overlay, scan_terrain
from lidar_dev.recording import load_recording
from terrain_overlay.tests.scenes import DEPTH, default_scene


def write_recording(folder, frames=3, legacy=False):
    (folder / "depth").mkdir(parents=True)
    intrinsics = ([DEPTH.fx, DEPTH.fy, DEPTH.cx, DEPTH.cy] if legacy else
                  {"fx": DEPTH.fx, "fy": DEPTH.fy, "ppx": DEPTH.cx, "ppy": DEPTH.cy,
                   "width": DEPTH.width, "height": DEPTH.height})
    (folder / "meta.json").write_text(json.dumps(
        {"depth_scale_m": 0.00025, "frame_period_s": 0.1, "depth_intrinsics": intrinsics}))
    for index in range(frames):
        cv2.imwrite(str(folder / "depth" / f"{index:06d}.png"), default_scene())
    return folder


@pytest.mark.parametrize("legacy", [False, True])
def test_recording_loads_both_meta_formats(tmp_path, legacy):
    recording = load_recording(write_recording(tmp_path / "scene", legacy=legacy))
    assert recording.depth_camera == DEPTH
    assert len(recording.depth_paths) == 3


def test_scan_summary_sees_the_box(tmp_path):
    summary = scan_terrain.summarize(write_recording(tmp_path / "scene"))
    assert summary["frames"] == 3
    assert summary["statuses"] == {"OK/FRESH": 3}
    assert summary["zones_median_m"]["center"] == pytest.approx(1.2, abs=0.03)


def test_overlay_writes_images(tmp_path, monkeypatch):
    folder = write_recording(tmp_path / "scene")
    monkeypatch.chdir(tmp_path)
    assert overlay.main([str(folder)]) == 0
    assert len(list((tmp_path / "out_overlay" / "scene").glob("*.png"))) == 3


def test_ground_writes_a_2d_result_without_open3d(tmp_path, monkeypatch, capsys):
    folder = write_recording(tmp_path / "flat", frames=1)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("NO3D", "1")
    assert ground.main([str(folder), "0"]) == 0
    assert (tmp_path / "out_flat_0.png").exists()
    assert "status: OK FRESH" in capsys.readouterr().out
```

- [ ] **Step 2: 실패 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider lidar_dev`
Expected: FAIL. 수집 단계에서 `ModuleNotFoundError: No module named 'lidar_dev.recording'`, 또는 기존 `overlay.py`의 `from terrain import load_detector` 실패.

- [ ] **Step 3: 구 파일 삭제와 도구 재작성**

```bash
git rm lidar_dev/terrain.py lidar_dev/packet.py lidar_dev/terrain_node.py lidar_dev/before.txt lidar_dev/after.txt lidar_dev/requirements.txt
```

`lidar_dev/recording.py`:

```python
"""Offline L515 recording format shared by the lidar_dev tools.

    <folder>/meta.json        depth_scale_m, frame_period_s, depth_intrinsics,
                              color_intrinsics (optional), depth_to_color (optional)
    <folder>/depth/000000.png raw, unaligned uint16 depth (16-bit PNG)
    <folder>/color/000000.jpg optional colour frames (1280×720)

Older recordings whose ``depth_intrinsics`` is a ``[fx, fy, cx, cy]`` list
are still readable; their size comes from the first depth PNG.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import DetectorConfig, TerrainDetector

DEFAULT_FRAME_PERIOD_S = 0.1


@dataclass(frozen=True)
class Recording:
    folder: Path
    depth_scale_m: float
    frame_period_s: float
    depth_camera: CameraModel
    color_camera: CameraModel | None
    depth_paths: tuple[Path, ...]


def _camera(entry, fallback_size=None) -> CameraModel:
    if isinstance(entry, dict):
        width = entry.get("width", fallback_size[0] if fallback_size else None)
        height = entry.get("height", fallback_size[1] if fallback_size else None)
        return CameraModel(
            fx=float(entry["fx"]), fy=float(entry["fy"]),
            cx=float(entry.get("ppx", entry.get("cx"))),
            cy=float(entry.get("ppy", entry.get("cy"))),
            width=int(width), height=int(height),
        )
    fx, fy, cx, cy = (float(value) for value in entry[:4])
    return CameraModel(fx, fy, cx, cy, int(fallback_size[0]), int(fallback_size[1]))


def read_depth(path: Path) -> np.ndarray:
    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.dtype != np.uint16:
        raise ValueError(f"not a 16-bit depth PNG: {path}")
    return depth


def load_recording(folder) -> Recording:
    folder = Path(folder).expanduser()
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    depth_paths = tuple(sorted((folder / "depth").glob("*.png")))
    if not depth_paths:
        raise ValueError(f"no depth/*.png under {folder}")
    first = read_depth(depth_paths[0])
    size = (first.shape[1], first.shape[0])
    color = meta.get("color_intrinsics")
    return Recording(
        folder=folder,
        depth_scale_m=float(meta.get("depth_scale_m", 0.00025)),
        frame_period_s=float(meta.get("frame_period_s", DEFAULT_FRAME_PERIOD_S)),
        depth_camera=_camera(meta["depth_intrinsics"], size),
        color_camera=None if color is None else _camera(color),
        depth_paths=depth_paths,
    )


def make_detector(recording: Recording, **config) -> TerrainDetector:
    return TerrainDetector(
        recording.depth_camera,
        DetectorConfig(depth_scale_m=recording.depth_scale_m, **config),
    )


def iter_frames(recording: Recording):
    """Yield ``(path, depth, frame_rx_s)``; time advances by the frame period."""
    for index, path in enumerate(recording.depth_paths):
        yield path, read_depth(path), index * recording.frame_period_s
```

`lidar_dev/overlay.py`:

```python
"""Render detector results over each recorded depth frame.

    python -m lidar_dev.overlay data/obj_50cm      → out_overlay/obj_50cm/*.png
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

from lidar_dev.recording import iter_frames, load_recording, make_detector


def render(depth: np.ndarray, result, depth_scale_m: float) -> np.ndarray:
    gray = (np.clip(depth.astype(np.float32) * depth_scale_m / 5.0, 0, 1) * 255).astype(np.uint8)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if result.status == "OK":
        floor, obstacle = result.floor_mask, result.obstacle_mask
        image[floor] = (0.5 * image[floor] + 0.5 * np.array([0, 200, 0])).astype(np.uint8)
        image[obstacle] = (0, 0, 255)
    width = image.shape[1]
    ok = result.status == "OK"
    cv2.rectangle(image, (0, 0), (width, 30), (0, 120, 0) if ok else (0, 0, 200), -1)
    fmt = lambda value: "-" if value is None else f"{value:.2f}m"  # noqa: E731
    zones = result.zones
    label = (f"{result.status} {result.reason or result.ground_state or ''}  "
             f"L {fmt(zones['left'])}  C {fmt(zones['center'])}  R {fmt(zones['right'])}")
    cv2.putText(image, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return image


def main(argv=None) -> int:
    folder = Path((argv or sys.argv[1:])[0])
    recording = load_recording(folder)
    detector = make_detector(recording)
    out = Path("out_overlay") / folder.name
    out.mkdir(parents=True, exist_ok=True)
    for path, depth, rx_s in iter_frames(recording):
        result = detector.process(depth, rx_s)
        cv2.imwrite(str(out / path.name), render(depth, result, recording.depth_scale_m))
    print("저장:", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`lidar_dev/scan_terrain.py`:

```python
"""Summarise every recording under data/: status counts, zone medians, height.

    python -m lidar_dev.scan_terrain [data]
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path
import sys

import numpy as np

from lidar_dev.recording import iter_frames, load_recording, make_detector


def summarize(folder) -> dict:
    recording = load_recording(folder)
    detector = make_detector(recording)
    statuses: Counter[str] = Counter()
    zones = {"left": [], "center": [], "right": []}
    heights = []
    for _path, depth, rx_s in iter_frames(recording):
        result = detector.process(depth, rx_s)
        statuses[f"{result.status}/{result.reason or result.ground_state}"] += 1
        for name, value in result.zones.items():
            if value is not None:
                zones[name].append(value)
        if result.camera_height_m is not None:
            heights.append(result.camera_height_m)
    return {
        "frames": sum(statuses.values()),
        "statuses": dict(statuses),
        "zones_median_m": {k: (float(np.median(v)) if v else None) for k, v in zones.items()},
        "camera_height_median_m": float(np.median(heights)) if heights else None,
    }


def main(argv=None) -> int:
    root = Path((argv or sys.argv[1:] or ["data"])[0])
    for folder in sorted(p for p in root.iterdir() if (p / "meta.json").exists()):
        print(f"{folder.name:14s} {summarize(folder)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`lidar_dev/ground.py`:

```python
"""One-frame ground/obstacle analysis with an optional open3d 3D view.

    python -m lidar_dev.ground data/flat 5        (NO3D=1 skips the 3D window)
"""
from __future__ import annotations

import os
from pathlib import Path
import sys

import cv2
import numpy as np

from lidar_dev.overlay import render
from lidar_dev.recording import load_recording, make_detector, read_depth


def analyse(folder, index: int = 0):
    recording = load_recording(folder)
    depth = read_depth(recording.depth_paths[index])
    result = make_detector(recording).process(depth, 0.0)
    return recording, depth, result


def main(argv=None) -> int:
    args = argv or sys.argv[1:]
    folder, index = Path(args[0]), int(args[1]) if len(args) > 1 else 0
    recording, depth, result = analyse(folder, index)
    valid = depth > 0
    print(f"status: {result.status} {result.reason or result.ground_state}")
    if result.status == "OK":
        print(f"카메라 높이(바닥까지): {result.camera_height_m * 100:.1f} cm, "
              f"기울기: {result.camera_tilt_deg:.1f}도")
        print(f"유효 픽셀 중 바닥 {100 * result.floor_mask.sum() / valid.sum():.1f}%  "
              f"장애물 {100 * result.obstacle_mask.sum() / valid.sum():.1f}%")
        print("zone:", result.zones)
    out = f"out_{folder.name}_{index}.png"
    cv2.imwrite(out, render(depth, result, recording.depth_scale_m))
    print("저장:", out)
    if os.environ.get("NO3D") == "1" or result.status != "OK":
        return 0
    try:
        import open3d as o3d
    except ImportError:
        print("open3d 없음 — 3D 보기는 건너뜀(선택 의존)")
        return 0
    camera = recording.depth_camera
    v, u = np.nonzero(valid)
    z = depth[valid].astype(np.float64) * recording.depth_scale_m
    points = np.stack(((u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z), axis=1)
    colors = np.tile([0.2, 0.5, 1.0], (len(points), 1))
    colors[result.floor_mask[valid]] = [0.6, 0.6, 0.6]
    colors[result.obstacle_mask[valid]] = [1.0, 0.0, 0.0]
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points * np.array([1, -1, -1]))
    cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries(
        [cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(0.3)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`lidar_dev/view_depth.py`:

```python
"""3D point-cloud view of one recorded depth frame (needs optional open3d).

    python -m lidar_dev.view_depth ~/Downloads/L515/20260930_212517 5
"""
from __future__ import annotations

from pathlib import Path
import sys

import cv2
import numpy as np

from lidar_dev.recording import load_recording, read_depth


def main(argv=None) -> int:
    args = argv or sys.argv[1:]
    recording = load_recording(Path(args[0]))
    index = int(args[1]) if len(args) > 1 else 0
    try:
        import open3d as o3d
    except ImportError:
        print("open3d 가 없습니다 — `pip install open3d` 후 다시 실행하세요(선택 의존)")
        return 1
    depth = read_depth(recording.depth_paths[index])
    camera = recording.depth_camera
    valid = depth > 0
    v, u = np.nonzero(valid)
    z = depth[valid].astype(np.float64) * recording.depth_scale_m
    points = np.stack(((u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z), axis=1)
    print(f"유효 픽셀 {100 * valid.mean():.1f}%  거리 {z.min():.2f} ~ {z.max():.2f} m  "
          f"중앙값 {np.median(z):.2f} m")
    scaled = ((z - z.min()) / (np.ptp(z) + 1e-6) * 255).astype(np.uint8)
    colors = cv2.applyColorMap(scaled.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)[:, ::-1] / 255.0
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points * np.array([1, -1, -1]))
    cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries(
        [cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`lidar_dev/record.py`:

```python
"""Record raw (unaligned) L515 depth + colour for the offline tools.

⚠️ The production L515 is owned by the Gateway (single owner).  Stop it first
(`docker compose ... stop powertrain_ros`) or record on a bench PC.

    python -m lidar_dev.record data/obj_50cm      → meta.json, depth/*.png, color/*.jpg
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

N_FRAMES, SKIP, FPS = 40, 3, 30  # keep every 3rd frame → 0.1 s, like the Gateway


def _intrinsics(profile) -> dict:
    i = profile.as_video_stream_profile().get_intrinsics()
    return {"fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
            "width": i.width, "height": i.height,
            "model": str(i.model), "coeffs": list(i.coeffs)}


def main(argv=None) -> int:
    import pyrealsense2 as rs

    folder = Path((argv or sys.argv[1:] or ["data/scene"])[0])
    (folder / "depth").mkdir(parents=True, exist_ok=True)
    (folder / "color").mkdir(parents=True, exist_ok=True)
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, FPS)
    profile = pipeline.start(config)
    try:
        depth_profile = profile.get_stream(rs.stream.depth)
        color_profile = profile.get_stream(rs.stream.color)
        extrinsics = depth_profile.get_extrinsics_to(color_profile)
        meta = {
            "depth_scale_m": profile.get_device().first_depth_sensor().get_depth_scale(),
            "frame_period_s": SKIP / FPS,
            "depth_intrinsics": _intrinsics(depth_profile),
            "color_intrinsics": _intrinsics(color_profile),
            "depth_to_color": {"rotation": list(extrinsics.rotation),
                               "translation_m": list(extrinsics.translation)},
        }
        (folder / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        for seconds in (3, 2, 1):
            print(f"{seconds}초 후 녹화 시작", end="\r", flush=True)
            time.sleep(1)
        saved = count = 0
        while saved < N_FRAMES:
            frames = pipeline.wait_for_frames()
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            count += 1
            if count % SKIP:
                continue
            cv2.imwrite(str(folder / "depth" / f"{saved:06d}.png"),
                        np.asanyarray(depth.get_data()).copy())
            cv2.imwrite(str(folder / "color" / f"{saved:06d}.jpg"),
                        np.asanyarray(color.get_data()).copy())
            saved += 1
            print(f"saved {saved}/{N_FRAMES}   ", end="\r", flush=True)
    finally:
        pipeline.stop()
    print(f"\n{folder} 저장 완료")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`lidar_dev/README.md`:

````markdown
# lidar_dev — L515 지형 판정 오프라인 도구

녹화한 L515 깊이로 `terrain_overlay` 판정기(실시간 노드와 같은 코어)를 돌려 보는 도구 모음이다.
실시간 경로는 `ros2 run powertrain_ros terrain_overlay`(UDP :5009 → 운용 콘솔)이며,
설계는 `docs/specs/2026-10-17-l515-terrain-overlay-design.md`를 따른다.
**운용자 시각 보조 전용**이다 — 주행 제어·정지·안전 판정에 쓰지 않는다.

## 의존

- numpy, opencv-python(-headless) — 필수
- open3d — 선택. `ground.py`의 3D 보기와 `view_depth.py`만 쓴다. 없으면 2D 결과만 낸다.
- pyrealsense2 — `record.py`만 쓴다.

## 녹화 형식

```
<폴더>/meta.json         depth_scale_m, frame_period_s, depth_intrinsics, color_intrinsics, depth_to_color
<폴더>/depth/000000.png  raw 비정렬 16비트 깊이 (640×480)
<폴더>/color/000000.jpg  컬러 (1280×720)
```

예전 녹화(`depth_intrinsics`가 `[fx, fy, cx, cy]` 목록)도 읽는다.

## 실행 (저장소 루트에서)

```bash
# 녹화 — ⚠️ 실기 L515 는 Gateway 단일 소유다. 먼저 Gateway 를 멈추거나 벤치 PC 에서 녹화한다.
python -m lidar_dev.record data/obj_50cm

python -m lidar_dev.overlay data/obj_50cm        # out_overlay/obj_50cm/*.png (초록=바닥, 빨강=장애물)
python -m lidar_dev.scan_terrain data            # 폴더별 상태·zone 중앙값·카메라 높이
NO3D=1 python -m lidar_dev.ground data/flat 5    # 한 프레임 분석 + out_flat_5.png
python -m lidar_dev.view_depth data/flat 5       # open3d 3D 보기
```

✅ 기대: `scan_terrain`에서 평지 녹화는 `OK/FRESH`가 대부분이고 카메라 높이가 실제 장착 높이(±2 cm)와 맞는다.
````

- [ ] **Step 4: 통과 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider lidar_dev`
Expected: `5 passed`

`record.py`·`view_depth.py`는 카메라와 open3d가 필요해 자동 시험이 없다. 문법만 확인한다.
Run: `/home/light/anaconda3/bin/python -m py_compile lidar_dev/record.py lidar_dev/view_depth.py && echo ok`
Expected: `ok`

- [ ] **Step 5: 커밋(사용자 확인 후)**

```bash
git add lidar_dev/recording.py lidar_dev/overlay.py lidar_dev/scan_terrain.py lidar_dev/ground.py lidar_dev/view_depth.py lidar_dev/record.py lidar_dev/README.md lidar_dev/tests/test_offline_tools.py
git commit -m "refactor(lidar_dev): 오프라인 도구를 terrain_overlay 코어로 옮기고 구 노드를 지운다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```
(삭제분은 Step 3의 `git rm`으로 이미 스테이징돼 있다.)

---

### Task 10: compose 서비스와 문서

**Files:**
- Modify: `docker/docker-compose.integrated.yml`, `scripts/tests/test_integrated_launchers.py`, `docs/integrated-operation.md`, `docs/network-ip-setup.md`, `operator_console/README.md`

**Interfaces:**
- Produces:
  - compose 서비스 `powertrain_terrain_overlay`(profile `terrain-overlay`, `cpus 1.0`, `mem_limit 512m`, OMP/OPENBLAS 1, 세션 파일 환경변수, `restart: unless-stopped`, healthcheck 비활성)
  - 켜는 법: `INTEGRATED_PROFILES=terrain-overlay INTEGRATED_SERVICES=powertrain_terrain_overlay scripts/robot-start`, 또는 `docker compose … --profile terrain-overlay up -d powertrain_terrain_overlay`

- [ ] **Step 1: 실패하는 시험 수정**

```diff
--- a/scripts/tests/test_integrated_launchers.py
+++ b/scripts/tests/test_integrated_launchers.py
@@ -14,10 +14,27 @@
 def test_observers_do_not_inherit_image_camera_healthcheck():
     import yaml
     services = yaml.safe_load((ROOT / "docker/docker-compose.integrated.yml").read_text())["services"]
-    for name in ("powertrain_chassis_telemetry", "powertrain_pdist80b_telemetry", "powertrain_arm_console_bridge"):
+    for name in ("powertrain_chassis_telemetry", "powertrain_pdist80b_telemetry",
+                 "powertrain_arm_console_bridge", "powertrain_terrain_overlay"):
         assert services[name].get("healthcheck", {}).get("disable") is True, name
 
 
+def test_terrain_overlay_service_is_opt_in_and_bounded():
+    import yaml
+    services = yaml.safe_load((ROOT / "docker/docker-compose.integrated.yml").read_text())["services"]
+    service = services["powertrain_terrain_overlay"]
+    assert service["profiles"] == ["terrain-overlay"]
+    assert (service["cpus"], service["mem_limit"]) == (1.0, "512m")
+    environment = service["environment"]
+    assert "OMP_NUM_THREADS=1" in environment and "OPENBLAS_NUM_THREADS=1" in environment
+    assert "POWERTRAIN_OPERATOR_SESSION_FILE=/run/powertrain/operator-session.json" in environment
+    command = "\n".join(service["command"])
+    assert "ros2 run powertrain_ros terrain_overlay" in command
+    assert "operator_port:=5009" in command
+    # Default bring-up excludes it until the two-encoder load check passes.
+    assert "powertrain_terrain_overlay" not in (ROOT / "scripts/robot-start").read_text()
+
+
 def test_ros_setup_optional_environment_is_allowed_by_daily_shell(tmp_path):
     import yaml
     fake_setup = tmp_path / "setup.bash"
```

- [ ] **Step 2: 실패 확인**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider scripts/tests/test_integrated_launchers.py -k "terrain_overlay_service or observers"`
Expected: FAIL — `KeyError: 'powertrain_terrain_overlay'`

- [ ] **Step 3: compose 서비스 추가**

```diff
--- a/docker/docker-compose.integrated.yml
+++ b/docker/docker-compose.integrated.yml
@@ -146,6 +146,43 @@
         exec ros2 run powertrain_ros arm_console_bridge --ros-args -p console_host:=127.0.0.1
     restart: unless-stopped
 
+  # L515 terrain obstacle outlines → operator console UDP :5009. Operator
+  # visual aid only (no control/safety input). Opt-in profile: not part of
+  # scripts/robot-start until the two-encoder load check (spec §7.3-2) passes.
+  powertrain_terrain_overlay:
+    healthcheck:
+      disable: true
+    profiles: [terrain-overlay]
+    image: powertrain-sw:ros
+    container_name: powertrain_terrain_overlay
+    network_mode: host
+    ipc: host
+    cpus: 1.0
+    mem_limit: 512m
+    environment:
+      - PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros
+      - POWERTRAIN_OPERATOR_SESSION_FILE=/run/powertrain/operator-session.json
+      - OMP_NUM_THREADS=1
+      - OPENBLAS_NUM_THREADS=1
+    volumes:
+      - ../:/workspace:ro
+      - type: bind
+        source: /run/powertrain
+        target: /run/powertrain
+        bind:
+          create_host_path: false
+    working_dir: /workspace
+    entrypoint: [/bin/bash, -lc]
+    command:
+      - |
+        set -e
+        test -f /workspace/ros2/install/setup.bash
+        source /opt/ros/humble/setup.bash
+        source /workspace/ros2/install/setup.bash
+        set -u
+        exec ros2 run powertrain_ros terrain_overlay --ros-args -p operator_host:=127.0.0.1 -p operator_port:=5009
+    restart: unless-stopped
+
   powertrain_session:
     image: powertrain-sw:ros
     container_name: powertrain_session
```

- [ ] **Step 4: 통과 확인 + 병합 설정 검증**

Run: `/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider scripts/tests/test_integrated_launchers.py`
Expected: `2 failed, 12 passed`. 2건은 Task 0의 기존 `robot_prepare` 실패이고, 새 실패는 없어야 한다.

Run: `docker compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml --profile terrain-overlay config --format json | python3 -c "import json,sys; s=json.load(sys.stdin)['services']['powertrain_terrain_overlay']; print(s['cpus'], s['mem_limit'], s['restart'])"`
Expected: `1 536870912 unless-stopped`

- [ ] **Step 5: 문서 수정**

```diff
--- a/docs/integrated-operation.md
+++ b/docs/integrated-operation.md
@@ -367,6 +367,7 @@
 | 전방 영상 | L515 Gateway 단일 소유, SRT :5000 |
 | 작업 영상·인식·로봇팔 상태 | 준비된 선택 스택 + SRT :5002 / UDP :5003·5007 |
 | 전원·차체 상태 | UDP :5004·5005, 현재 인증된 노트북 주소로 전송 |
+| 지형 장애물 윤곽(참고용) | 선택 서비스 `powertrain_terrain_overlay`(compose profile `terrain-overlay`), UDP :5009, 현재 인증된 노트북 주소로 전송 |
 | 부팅 순서/IP 변경 | 등록된 IPv4 호스트 후보 재해석, robot ID와 토큰으로 상대 확인 |
 
 세션 :9002는 동시 운용자 한 명만 허용한다. 연결 임대가 끝나면 입력/ops 프록시를
@@ -387,6 +388,10 @@
 환경 텔레메트리 포트는 `~/.config/powertrain/operator.json`의
 `environment_telemetry_port`로 지정하며 기본값은 `5008`이다. 1–65535 범위의 정수만
 허용되고, 잘못된 값은 세션·네트워크 시작 전에 콘솔 기동을 거부한다.
+L515 지형 장애물 윤곽은 `terrain_port`(기본 `5009`)로 받고, 영상과 맞추는 표시 지연은
+`terrain_display_delay_ms`(0–400 정수, 기본 0)로 정한다. 윤곽은 운용자 참고용이며
+주행 제어·안전 판정에 쓰이지 않는다. 송신 서비스는 기본 기동에 들어 있지 않다 —
+`INTEGRATED_PROFILES=terrain-overlay INTEGRATED_SERVICES=powertrain_terrain_overlay scripts/robot-start`로 켠다.
 
 ## 문제 확인
 
```

```diff
--- a/docs/network-ip-setup.md
+++ b/docs/network-ip-setup.md
@@ -78,6 +78,8 @@
 --telemetry-port 5004       파워트레인 텔레메트리(UDP)
 --chassis-telemetry-port 5005
 --arm-telemetry-port 5007   로봇팔 텔레메트리(UDP)
+--terrain-port 5009         L515 지형 장애물 윤곽(UDP, 참고용)
+--terrain-display-delay-ms 0  윤곽을 영상 지연에 맞추는 표시 지연(0–400)
 --ops-port 9001             조작 채널(TCP, 역할토큰)
 --ops-host                  ops 브로커가 다른 호스트면 지정(기본: --host 와 동일)
 --latency-ms 60             SRT 지연. 송신측 --srt-latency 와 같이 맞출 것
```

```diff
--- a/operator_console/README.md
+++ b/operator_console/README.md
@@ -143,7 +143,7 @@
   --host 127.0.0.1 --d435-port 15002 --l515-port 15000 \
   --metadata-port 15003 --telemetry-port 15004 \
   --chassis-telemetry-port 15005 --arm-telemetry-port 15007 \
-  --environment-telemetry-port 15008 --input-source TEST \
+  --environment-telemetry-port 15008 --terrain-port 15009 --input-source TEST \
   --ops-token-file /tmp/operator-console-no-token
 ```
 
@@ -168,8 +168,10 @@
 
 For the integrated `python -m operator_console` entrypoint, the validated
 `~/.config/powertrain/operator.json` keys include
-`environment_telemetry_port` (default `5008`). Overrides must be integer ports
-from 1 through 65535 and must match the robot-side sender configuration.
+`environment_telemetry_port` (default `5008`), `terrain_port` (default `5009`)
+and `terrain_display_delay_ms` (0–400, default 0). Port overrides must be
+integer ports from 1 through 65535 and must match the robot-side sender
+configuration. The L515 terrain outlines are an operator visual aid only.
 
 Observation remains caller/receiver-only: it never opens D435i/L515 or writes
 CAN. Commands are possible only through the authenticated ops client and every
```

- [ ] **Step 6: 커밋(사용자 확인 후)**

```bash
git add docker/docker-compose.integrated.yml scripts/tests/test_integrated_launchers.py docs/integrated-operation.md docs/network-ip-setup.md operator_console/README.md
git commit -m "feat(terrain): opt-in compose 서비스와 :5009 운용 문서를 더한다" -m "Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
```

---

### Task 11: 호스트 전체 확인과 Codex 리뷰

**Files:** 리뷰 지적에 따라 달라진다.

- [ ] **Step 1: 영향받는 스위트 전체 실행**

```bash
export PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros"
/home/light/anaconda3/bin/python -m pytest -q -p no:cacheprovider terrain_overlay lidar_dev remote_video scripts/tests tests
/usr/bin/python3 -m pytest -q -p no:cacheprovider terrain_overlay/tests/test_contract.py
env -u WAYLAND_DISPLAY GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m pytest -q -p no:cacheprovider -W ignore operator_console
```
Expected:
- conda 스위트: 새 실패 0. 기존 실패는 `scripts/tests` 2건과 `tests/` 2건(매니페스트 체크섬, CLAUDE.md §4 기준선)뿐이다.
- 콘솔: `10 failed` — Task 0과 같은 목록이어야 한다. `passed`는 기준선 672에서 지형 시험 수만큼 늘어난다.

실패 목록 비교는 다음으로 한다. 기준선 목록 파일은 Task 0에서 같은 방법으로 남긴다.
`… 2>&1 | grep -E "^FAILED" | sed 's/ - .*//' | sort`

- [ ] **Step 2: ROS 시험 재실행** (Task 5 Step 3·8 명령). Expected: `10 passed`, 스모크 PASS, 음성 대조 FAIL.

- [ ] **Step 3: Codex 적대적 리뷰(읽기 전용) — 수렴까지 반복**

```bash
node ~/.claude/plugins/cache/openai-codex/codex/1.0.6/scripts/codex-companion.mjs adversarial-review --base 78d14b1 > /tmp/claude-1000/codex_review_1.md 2>&1
```
`--base 78d14b1`을 반드시 준다. 인자가 없으면 미추적 파일(`zetin`·MKS 계획) 때문에 working-tree 범위가 선택돼 커밋된 구현이 빠진다(companion `lib/git.mjs` 176행, Codex 1회차 P2-7). 리뷰 전에 반영분을 모두 커밋해 둔다. 출력은 파일로 받아 전부 읽는다(`tail` 금지).
지적이 나오면 근거를 소스·실행으로 확인해 반영하고, 시험을 다시 돌린 뒤 같은 스레드에 재검토를 요청한다. "새 P1/P2 없음"이 나올 때까지 반복한다. 지적과 처리는 결과 보고서(Task 13)의 검토 절에 남긴다.

- [ ] **Step 4: Codex 프로세스 정리(RAM 위생)**

Run: `ps -eo pid,args | grep "[a]pp-server-broker"`
쉬고 있는 브로커는 트리째 SIGTERM으로 끝내고, 3초 뒤 남은 것만 SIGKILL한다(bash로). 그 뒤 `free -h`를 기록한다.

- [ ] **Step 5: 리뷰 반영 커밋(사용자 확인 후, 반영분이 있을 때만)**

---

### Task 12: Jetson 배포와 실행 스모크 (스펙 §7.2, §7.3-6)

**Files:** 없음(배포·확인). 이 Task는 **Jetson 접근과 사용자 승인**이 필요하다.

- [ ] **Step 1: Jetson 상태 확인**

Run: `SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh jetson 'cd ~/power-train-sw && git status -sb && git stash list | head -3 && git log --oneline -3'`
Expected: 미커밋·미추적 변경 목록을 기록한다. **팀원의 미커밋 파일은 보존한다.**

- [ ] **Step 2: 배포 경로를 사용자에게 확인받기**

배포 방법을 고른다.
- (a) 브랜치를 push한 뒤 Jetson에서 fetch·checkout한다.
- (b) 변경 파일만 scp한다.

push와 원격 변경은 사용자 승인 뒤에만 한다.

- [ ] **Step 3: ROS 재빌드(Jetson)**

Run (Jetson, 저장소 루트):
```bash
docker compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml run --rm --no-deps --entrypoint /bin/bash powertrain_control -lc \
  'set -e; source /opt/ros/humble/setup.bash; set -u; cd /workspace/ros2; colcon build --packages-select robot_arm_msgs powertrain_msgs dynamixel_control powertrain_ros'
```
Expected: `4 packages finished`(`scripts/robot_prepare.sh` 189–192행과 같은 명령).

- [ ] **Step 4: Jetson 이미지 의존 확인**

Run: `docker run --rm --entrypoint python3 powertrain-sw:ros -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"`
Expected: 버전이 출력된다. 실패하면 이미지 갱신은 별도 승인을 받는다(스펙 §8).

- [ ] **Step 5: 노드 시험과 설치 엔트리포인트 스모크(도메인 77)**

⚠️ `powertrain-sw:ros`의 기본 엔트리포인트(`docker/powertrain_ros_entrypoint.sh`)는 인자와 무관하게 Gateway를 띄운다. 반드시 엔트리포인트가 `bash -lc`인 terrain 서비스로 실행한다(Codex 1회차 P1).

Run (Jetson, 저장소 루트):
```bash
C="docker compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml --profile terrain-overlay"
$C run --rm --no-deps -e ROS_DOMAIN_ID=77 -e PYTHONDONTWRITEBYTECODE=1 powertrain_terrain_overlay \
  'source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash && cd /workspace && env -u POWERTRAIN_OPERATOR_SESSION_FILE python3 -m pytest -q -p no:cacheprovider ros2/src/powertrain_ros/test/test_terrain_overlay_node.py terrain_overlay'
$C run --rm --no-deps -e ROS_DOMAIN_ID=77 -e PYTHONDONTWRITEBYTECODE=1 powertrain_terrain_overlay \
  'source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash && cd /workspace && python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py; python3 ros2/src/powertrain_ros/test/installed_terrain_overlay.py --inject-double-scale || true'
```
terrain 서비스는 생산 세션 환경변수(`POWERTRAIN_OPERATOR_SESSION_FILE`)를 갖는다. 그대로 두면 시험 패킷이 실제 콘솔로 가거나 아예 나가지 않는다. 그래서 pytest 명령에서 `env -u`로 지운다(시험 모듈도 autouse fixture로 지운다 — 2026-10-17 이 변수를 켠 컨테이너에서도 통과 확인). 설치 스모크는 자식 프로세스 환경에서 이미 지운다(Codex 2회차 P2-2).
pytest가 이미지에 없으면 그 사실을 기록하고, 스모크 두 줄만 판정한다.
Expected:
- `126 passed` (노드 10 + 코어 116)
- 스모크 PASS. `proc_ms`와 `stale_after_s`를 기록한다(Jetson 처리시간 첫 실측).
- 음성 대조 FAIL.

- [ ] **Step 6: compose 기동과 무크래시 확인**

Run:
```bash
docker compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml --profile terrain-overlay up -d powertrain_terrain_overlay
sleep 20; docker inspect -f '{{.State.Status}} restarts={{.RestartCount}}' powertrain_terrain_overlay
docker logs --since 60s powertrain_terrain_overlay 2>&1 | grep -ciE "traceback|error" ; docker logs --since 60s powertrain_terrain_overlay 2>&1 | tail -5
```
Expected:
- `running restarts=0`, traceback 0건.
- 로그 첫 줄은 `terrain overlay → operator session file /run/powertrain/operator-session.json (fallback 127.0.0.1):5009 · session …`이다. 실제 목적지는 세션 파일의 노트북 주소이고, 127.0.0.1은 환경변수가 없을 때의 대체값이다.
- 두 `camera_info … (pinhole used)` 로그는 Gateway가 떠 있을 때만 나온다.
- 컬러 `distortion_model`·`d` 값을 보고서용으로 기록한다.

- [ ] **Step 7: 세션 전제 확인(스펙 §7.3-6) — 하나라도 다르면 배포 중단**

Run:
```bash
docker exec powertrain_terrain_overlay sh -c 'cat /proc/sys/kernel/random/boot_id; readlink /proc/self/ns/time; cat /proc/self/timens_offsets'
cat /proc/sys/kernel/random/boot_id
echo "$JETSON_SUDO_PASS" | sudo -S readlink /proc/1/ns/time
```
(Jetson에서는 원격 sudo 규칙대로 비밀번호를 ssh stdin으로 넘긴다.)
Expected:
- 두 boot_id가 같다.
- `time:[…]` 링크가 같다.
- `timens_offsets`의 monotonic·boottime이 모두 `0 0`이다.

---

### Task 13: 실기 E2E와 결과 보고 (스펙 §7.3-1~5, 7) — 정지 상태에서만

**Files:**
- Create: `docs/reports/2026-10-xx-l515-terrain-overlay.md`(날짜는 실측일)
- Modify(결과에 따라, 각각 별도 커밋): `docker/docker-compose.integrated.yml` 명령의 `-p alignment_verified:=true`·`-p front_offset_m:=…`, `operator_console` 기본 `--terrain-display-delay-ms`, `scripts/robot-start`(목록 추가)

로봇과 운용 PC가 필요하다. 모터는 구동하지 않는다. 각 항목의 측정값·스크린샷·판정을 보고서에 표로 남긴다.

- [ ] **Step 1: §7.3-1 표시 경로.** 실제 Gateway·SRT 위에 윤곽이 겹치는지 본다. MAIN↔PiP 전환과 L515 RGB/DEPTH/OVERLAY 모드에서 따라오는지도 확인하고, 스크린샷을 남긴다.
- [ ] **Step 2: §7.3-2 자원.** 두 SRT 인코딩이 도는 상태에서 서비스를 켜기 전과 후를 비교한다.
  - 판정 기준: `proc_ms` p95 < 80 ms **그리고 모든 처리 시도의 최대 ≤ 한 틱**(10 Hz면 100 ms, 5 Hz면 200 ms). 판정은 노드 종료 때 제한 없이 남는 `terrain processing summary: max … ms, overruns N` 한 줄로 한다 — `overruns 0`이어야 한다. 실행 중 `exceeds the … ms tick` 경고는 참고용이다(로그 제한이 있어 개수 판정에 쓰지 않는다), 노드 RSS 최대값, L515 화면 FPS, raw depth 주기, `chassis_node` 50 Hz tick 간격, 워치독 이벤트, `tegrastats`.
  - 악화되면 `publish_hz 5` 또는 RANSAC 횟수를 줄여 다시 잰다. 5 Hz로 내리면 stale 상한은 0.95 s가 되며, 같은 기준(최대 ≤ 한 틱)으로 다시 판정한다.
  - 통과하면 `robot-start` 추가를 별도 커밋으로 제안한다. 그 커밋은 `scripts/robot-start`의 서비스 목록, `test_terrain_overlay_service_is_opt_in_and_bounded`의 "robot-start에 없음" 단언(→ "있음"), compose 프로필 제거 여부, `docs/integrated-operation.md`의 켜는 법을 함께 바꾸고 Task 10·11 시험을 다시 돌린다(Codex 1회차 P2-10).
- [ ] **Step 3: §7.3-3 정합.** 상자를 1.0 m·2.0 m에 둔다. 중앙 ≤ 15 px, 가장자리(좌우 끝 20 %) ≤ 30 px면 `alignment_verified=true`(별도 커밋)이고, 미달이면 false를 유지한다. `dist_m`과 줄자 거리 차이도 기록한다.
- [ ] **Step 4: §7.3-4 지연.** 물체 출입을 10회 녹화한다. `Δ = 영상 출현 − 윤곽 출현`의 중앙값을 10 ms 단위로 반올림한다. Δ > 0이면 `d = min(Δ, 0.4 s)`, 아니면 `d = 0`이다. 기본값 변경은 별도 커밋으로 한다.
- [ ] **Step 5: §7.3-5 환경.** 대회장 조도에서 유효 깊이 화소 비율과 `NO_GROUND` 빈도를 잰다. 장착 높이(0.25–0.80 m)와, `front_offset_m`에 쓸 depth 원점–차체 앞 거리도 줄자로 잰다.
- [ ] **Step 6: §7.3-7 단절.**
  - Gateway를 멈추면 콘솔에 "센서 오류"가 뜨고, 재기동하면 복구된다.
  - `docker restart powertrain_terrain_overlay` 후 새 세션으로 복구되는지 보고, 걸린 시간을 기록한다.
  - 운용 PC 네트워크를 끊었다 이어도 노드가 살아 있는지 본다.
- [ ] **Step 7(선택, 별도 승인): 저속 주행 중 관찰.** 정지 시험만으로는 차체 흔들림과 영상 지연에 따른 어긋남을 보지 못한다(agy). 원격주행 인수와 같은 안전 조건에서 0.1–0.2 m/s로 장애물에 다가가며, 윤곽 흔들림과 영상 대비 어긋남을 녹화로 기록한다. 구동은 사용자가 그 자리에서 승인할 때만 한다.
- [ ] **Step 8: 보고서 작성과 스펙 개정.** 계획 단계에서 정한 차이(Global Constraints "스펙과 다르게 정한 점")를 스펙 §3.1·§4.1·§4.6·§6·§7.2·§8·§11에 반영한다. 보고서와 함께 사용자 확인 뒤 커밋한다.

---

### Task 14: 지침 문서·메모리 갱신(검증 뒤)

**Files:** `.claude/CLAUDE.md`, `AGENTS.md`(짝으로 같은 내용)

- [ ] **Step 1:** §3 구조에 두 줄을 넣는다.
  - `terrain_overlay/  L515 지형 장애물 윤곽 순수 코어(운용자 시각 보조 전용)`
  - `lidar_dev/  지형 판정 오프라인 도구`
- [ ] **Step 2:** §2 구현·검증 기록에 기능 한 줄을 넣는다. 실기 판정 결과와 보고서 경로를 함께 쓰고, Task 13 결과가 있을 때만 쓴다.
- [ ] **Step 3:** 두 파일이 같은지 확인한다(`diff <(sed -n …) …`). 사용자 확인 뒤 커밋한다.
- [ ] **Step 4:** 프로젝트 메모리 `lidar-dev-ownership.md`를 결과 기준으로 갱신한다.

---

## 자체 검토

### 스펙 대조 (writing-plans 자체 점검)

| 스펙 | Task |
|---|---|
| §2 P1 3건(무인자 생성·이중 스케일·토픽) | 5(CameraInfo 뒤 생성, 정본 토픽), 2(uint16 강제·음성 대조), 5 스모크 음성 대조 |
| §2 P2 :5003 충돌·open3d·intrinsics·비직교·평면 수명·sendto | 1, 2, 5, 2(롤 시험), 2·4, 5 |
| §2 P3 이중 인덱싱·stamp | 3(0-depth 시험), 4(stamp 유지 시험) |
| §3.1 모듈 경계·`remote_video` 상수 | 1, 2–6 |
| §3.2 lidar_dev 정리 | 9 |
| §4.1 파라미터·CameraInfo·인코딩·틱 1–5·stale 계약·세션 | 4, 5 |
| §4.2 판정 1–9 | 2 |
| §4.3 투영 | 3 |
| §4.4 윤곽·크기·자기 검증 | 3, 4 |
| §4.5 송신 | 5 |
| §4.6 세션 게이트·지연·배지·윤곽·`VideoPanel`·체크박스·인자·`LIVE_PORTS`·데모 | 6, 7, 8 |
| §5 패킷 v1 | 1 |
| §6 오류 표 | 4, 5, 6 |
| §7.1 단위 / §7.2 스모크 / §7.3 실기 | 1–9 / 5·8·12 / 13 |
| §8 배포 | 10, 12 |
| §9 문서 | 9, 10, 13, 14 |

빈칸·"TBD" 검사 결과:
- 보고서 파일명의 날짜(`2026-10-xx`)만 실측일로 정해진다.
- Task 13의 기본값 변경은 실측값에 따라 정해지는 단계다.

타입·이름 일관성 검사:
- `TerrainFrame.session_key`, `TerrainView.probe_state`, `publish_video_size`, `LATEST_DEPTH_QOS`는 정의한 Task와 쓰는 Task에서 이름이 같다.

### 독립 검토

#### ① 직접 재계산 (Claude, 2026-10-17)

- 코드의 수치 상수를 스펙과 하나씩 대조했고 모두 일치했다. 대상은 `detector.py`·`contours.py`·`contract.py`·`runtime.py`·`operator_console/terrain.py`의 모듈 상수다(0.05·0.02 m·200·3000·1000·100·40°·0.25–0.80 m·0.03/0.05/2.0/3.0 m·2/5 백분위·9×9/5×5·150 px²·1 %/×1.5·16·>6·1200 B·4·5–10 Hz·0.5/0.6/0.4/1.0 s).
- 모든 시험·스모크를 실제로 돌려 통과 수와 측정값을 얻었다(머리말 참조).
- 계획 안의 diff 블록 14개를 HEAD에 `git apply --check`로 적용해 봤다. 처음 조립에서는 `docs/integrated-operation.md` diff가 깨져 있었다. 원인은 조립 스크립트가 끝의 공백 context 행을 `rstrip()`으로 지운 것이었고, 고친 뒤 14/14 적용됨을 확인했다.
- 롤 반례에서 옛 비직교 축은 `left`에 장애물을 넣는다(실측). 롤 시험이 결함을 가려낸다.

#### ② Codex 적대 검토 1회차 (2026-10-17) — 판정: 미수렴(P1 1, P2 9, P3 2)

| # | 지적 | 확인 | 처리 |
|---|---|---|---|
| P1 | Task 12 시험 명령이 생산 이미지 엔트리포인트 때문에 Gateway를 띄움 | `docker/powertrain_ros_entrypoint.sh` 마지막 줄 `exec python3 -m l515_dashboard.gateway_main` 확인 | `bash -lc` 엔트리포인트인 terrain 서비스로 `compose run` 하도록 명령을 명시 |
| P2-2 | 처리 전 `now`를 처리 뒤에도 써서 만료·age가 처리 시간만큼 어긋남 | 맞음(구조상) | 런타임에 `clock` 주입, 처리 뒤 재판정. (다)·처리시간·처리 중 stale 시험 3개 추가 |
| P2-3 | 처리 중 도착한 depth가 늦게 꺼내져 수신 시각이 밀림, 느린 처리 시험이 0.85 s를 허용 | 끝단 시험으로는 재현이 확률적이었다(두 변형 모두 0.51–0.57 s) | 2-스레드 executor + 콜백 그룹 분리. 결정적 시험을 추가했고 음성 대조(depth를 틱 그룹으로)에서 3/3 FAIL. 끝단 시험 허용치를 0.70 s(계약 + 표본 여유)로 줄임 |
| P2-4 | 콘솔이 세 값을 따로 읽어 세션 전환 순간 옛 윤곽 + 새 수신 시각이 섞임 | 맞음 | `snapshot()` 한 락, 시험 추가 |
| P2-5 | cairo 기본 텍스트 API로 한글이 글리프 0 | 시스템 python에서 재현(`[0, 0, 3, 50, 46]`) | PangoCairo + 말줄임, "누락 글리프 0" 시험 |
| P2-6 | 영상 대기 화면이 "항상" 보여야 할 배지를 덮음 | 레이어 순서 확인 | 지형 캔버스를 대기 화면 위로(pass-through), 영상이 끊기면 윤곽만 숨김. 레이어·연동 시험 |
| P2-7 | 인자 없는 `adversarial-review`가 미추적 파일 때문에 working-tree 범위를 고름 | `lib/git.mjs` 176행 확인 | `--base 78d14b1` 명시 |
| P2-8 | 설치 스모크가 비정상 종료에도 PASS | 맞음 | 종료 코드 0을 PASS 조건에 포함 |
| P2-9 | `schema_version 1.0` 수용, `status=[]` → TypeError, `10**400` → OverflowError | 재현 | 타입 검사·OverflowError 처리, 거부 시험 8개 추가 |
| P2-10 | Task 13의 robot-start 추가가 "없음" 단언 시험과 충돌 | 맞음 | 전환 커밋에 시험·문서·재검증을 함께 넣도록 명시 |
| P3 | 100 ms 타이머 미해제, 긴 문구가 PiP에서 잘림 | 맞음 | destroy에서 해제, Pango 말줄임 |

#### ② Codex 적대 검토 2회차 (2026-10-17) — 판정: 미수렴(P1 0, P2 3, P3 1)

1회차 지적은 모두 해소됐다고 판정했다(두 번 평가하는 tick의 중복 처리 반례 없음, D435·PiP 입력 경로 회귀 없음).

| # | 지적 | 확인 | 처리 |
|---|---|---|---|
| P2-1 | 처리가 길면 stale 패킷 송신 자체가 늦어짐(처리 0.6 s → 0.68 s) | 가상시계 반례 맞음 | 계약식에 최대 처리시간 항을 넣고(0.75 s), 한 틱을 넘는 처리를 로그로 남김. §7.3-2 판정에 최대 ≤ 100 ms 추가. 반례를 시험으로 고정 |
| P2-2 | Task 12 pytest가 생산 세션 환경변수를 물려받아 fixture로 안 보냄 | `destination.py` 확인 | 시험 모듈 autouse fixture + 명령에서 `env -u`. 변수를 켠 컨테이너에서 122 통과 |
| P2-3 | 영상 ERROR/EOS 직후 다음 health 갱신 전까지 옛 윤곽이 대기 화면 위에 남음 | 맞음 | ERROR·EOS·재시작에서 즉시 `set_video_live(False)` + 다시 그리기, 세 경로 GTK 시험 |
| P3 | Task 11·12의 기대 통과 수 | 맞음 | 9, 122로 수정 |

#### ② Codex 적대 검토 3회차 (2026-10-17) — 판정: 미수렴(P1 0, P2 1, P3 1)

계약 개정으로 처리하는 방향은 "시각 보조 기능에서 가능한 절충"으로 수용됐다. 2회차 지적은 모두 해소됐다(세션 변수 격리 후 목적지 loopback 확인, ERROR·EOS·재시작·endpoint 변경에서 즉시 숨김 확인).

| # | 지적 | 확인 | 처리 |
|---|---|---|---|
| P2 | 예외로 끝난 처리가 최대 처리시간 판정에서 빠짐. 100.04 ms가 100.0으로 반올림돼 경고 안 남 | 재현 맞음 | 성공·예외 모두 반올림 전 값으로 기록(`max_processing_s`·`processing_overruns`·로그), 두 경우 시험 추가 |
| P3 | Review Focus의 0.65 s, 5 Hz 상한, 스펙 개정 범위, "배지만 늦어짐" 근거가 과함 | 맞음 | 수치(10 Hz 0.75 s, 5 Hz 0.95 s)·근거 범위·개정 절 목록 정리 |

#### ② Codex 적대 검토 4회차 (2026-10-17) — 판정: 미수렴(P1 0, P2 1)

3회차 지적과 P3 문서 수정은 해소됐다. 계약 완화 방향은 계속 수용됐다.

| # | 지적 | 확인 | 처리 |
|---|---|---|---|
| P2 | 노드의 `_log_error`가 한 호출 위치에서 5 s 제한이라, 일반 오류 직후 처리시간 초과 경고가 사라져 "경고 0건" 판정이 위반을 놓침 | rclpy Humble 로거는 호출 위치별 제한(Codex 재현) | 초과 경고를 별도 싱크·호출 위치로 분리하고, 종료 때 제한 없는 요약 줄을 남겨 그것으로 판정. "일반 오류 → 5 s 안 초과 → 복구" 노드 시험을 추가했고, 음성 대조(초과를 일반 위치로 되돌림)에서 FAIL 확인 |

#### ② Codex 적대 검토 5회차 (2026-10-17) — 판정: **수렴**(새 P1/P2 없음)

4회차 P2가 해소됐다. 로거 대역으로 "일반 오류 → 600 ms 실패 → 20 ms 복구"를 재실행해 초과 경고 1건과 `max 600.00 ms, overruns 1` 요약을 확인했다. 공용 싱크로 되돌린 변형에서도 종료 요약이 `overruns 1`을 남기므로, Task 13 판정이 이 반례를 거부한다. 수렴 범위는 계획과 사본이며, Jetson 처리시간·정합 등 실기 인수는 Task 12·13의 별도 게이트다.

반박 없음으로 확인된 점(1회차): 높이 반올림의 경계 보존, 세션 게이트의 수신 공백 반례, KEEP_LAST 1과 Gateway QoS 호환, D435 크기 전달·PiP 이동, Task 12 재빌드·명시 서비스 기동.

#### ③ 독립 검토 — `agy` Gemini 3.8 Flash (High), ①② 미공유, "적히지 않은 전제" 질문

| 지적 | 확인 | 처리 |
|---|---|---|
| 컨테이너에서 `terrain_overlay` import 실패 가능(PYTHONPATH) | compose 서비스에 `PYTHONPATH=/workspace:…`가 있다. 설치 스모크도 이 경로로 통과 | 반영 불필요 |
| 컬러 왜곡 무시로 가장자리 정합 실패 가능 | 스펙 §4.3에서 이미 정한 위험 | §7.3-3 판정 유지, Task 12에서 실제 `d` 값 기록 |
| 단일 스레드에서 RANSAC이 depth 수신을 굶김 | Codex P2-3과 같음 | 다중 스레드 executor로 해결 |
| 127.0.0.1 목적지와 세션 파일 관계가 불명확 | 세션 파일이 우선인데 로그는 127.0.0.1로 찍혀 오해 소지 | 로그에 세션 파일·대체값을 함께 표시, Task 12 기대값 수정 |
| 정지 시험만으로는 주행 중 어긋남을 못 봄 | 맞음(스펙은 정지 E2E만) | Task 13 Step 7에 선택·별도 승인 저속 관찰 추가 |
| 영상이 멈추면 오버레이도 안 그려짐 | `TerrainCanvas`에 자체 100 ms 타이머가 있다 | 반영 불필요(사실과 다름) |
| "경로 장애물 없음"이 구덩이 없음으로 읽힘 | 맞음 | "경로 돌출 장애물 없음"으로 변경 |
| Jetson 이미지 의존성 확인이 늦음 | 맞음 | Task 0으로 앞당김 |
| arm tabs 기존 실패가 지형 결함을 가림 | 스모크 단언은 프로세스 종료 뒤 순차 검사라 타이밍과 무관. 음성 대조도 있음 | 반영 불필요 |
| 롤백 절차 없음 | 브랜치 작업이라 병합 전 | Global Constraints에 한 줄 추가 |
