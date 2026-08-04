# USB 스키드 조향 레이어 + 콘솔 모드 선택 설계

- 작성일: 2026-08-04
- 대상: `motor_control/chassis/`, `motor_control/corner_module/`,
  `ros2/src/powertrain_ros/`, `operator_console/`
- 기준선 코드: `09430db` (main) + 미커밋 작업트리
- 선행 자산: `motor_control/drive/bl70200/dualsense_usb_teleop.py`
  (2026-08-02 23:56 작성, 미커밋, **벤치에서 3보드 6축 실회전 확인됨**)

## 0. 배경과 범위

사용자 요구 2건:

1. **AK 조향 모터를 전혀 쓰지 않고, USB 로 연결된 ODrive 만으로 스키드 조향하는
   레이어**를 만들 것.
2. **GUI 에서 CAN 모드 ↔ USB 모드, 애커만 조향 ↔ 스키드 조향을 각각 선택**할 수
   있게 할 것.

브레인스토밍에서 확정된 결정:

| 결정 항목 | 채택 |
|---|---|
| 구현 계층 | `ChassisManager` 정식 대안 구성 (`four_wheel_geometry()` 선례와 동형) |
| AK 물리 상태 | 장착돼 있으나 전원·CAN 미사용 |
| 구동축 | 3보드 6축 전부 |
| 진입점 | 빌더 + `teleop_server` 플래그 → **GUI 요구로 `chassis_node` 까지 확장** |
| 대상 GUI | `operator_console` (운용 GTK 콘솔) |
| 전환 시점 | 조향모드 = 런타임, 트랜스포트 = 기동 시 |
| USB × 애커만 | 비활성 (USB 스택에는 조향 액추에이터가 없음) |

**범위 밖.** 콘솔에서 제어 스택을 자동 재기동하는 것(§8 참조), 구동 USB +
조향 CAN 혼합 구성, 지상 주행 커미셔닝(`track_gain` 실측 확정), 조향축
기계적 고정(SW 범위 밖 — §12 P1 에서 필요 여부만 판정).

## 1. 조사 결과

### 1.1 스키드 키네마틱스는 이미 구현돼 있다

`chassis/kinematics.py:130` 의 고정 바퀴 분기가 정확히 스키드 조향 식이다.

```python
else:
    delta, speed = 0.0, vx        # 고정 바퀴 = 전진성분만
```

여기서 `vx = v − ω·wᵢ.y` (`kinematics.py:122`). 즉 **모든 바퀴를
`steerable=False` 로 둔 기하 하나면 수식이 끝난다.** 새 수학 모듈이 필요 없다.

부수 효과도 전부 유리한 방향이다.

- `_peak_steer()` (`kinematics.py:77-85`) 는 조향륜만 순회하므로 조향륜이 0개면
  항상 0.0 을 반환한다 → `_limit_omega()` 가 즉시 `(omega, False)` 로 빠져나가
  ω 클램프가 자동으로 비활성화된다. 스키드는 조향한계가 없으므로 옳다.
- 속도 상한 스케일링(`kinematics.py:134-136`)과 비유한 입출력 검사
  (`kinematics.py:112-113`, `144-149`)는 그대로 살아 있다.

### 1.2 USB 구동 드라이버는 사실상 비어 있다

`corner_module/drive_odrive_usb.py` 의 `DriveOdriveUsb` 는 `odrive.find_any()`
로 **보드 1장의 `axis1` 하나**만 잡는다. 없는 것:

- 3보드 6축 주소지정 (시리얼 + 축 인덱스)
- 감속비 5:1 변환 (모터 ↔ 바퀴 프레임)
- 우측 미러 장착 부호 반전
- `CornerModule.tick()` 이 요구하는 `stale` / `axis_error` 건강 키
  (`corner_module.py:163-170`)

이 클래스는 `corner_module/teleop_dualsense.py:101,113` 이 아직 쓰고 있으므로
**수정하지 않고 새 클래스를 추가**한다.

### 1.3 일요일 스크립트가 USB 브링업의 대부분을 이미 담고 있다

`drive/bl70200/dualsense_usb_teleop.py` 는 벤치에서 3보드 6축 실회전이 확인된
코드다. 다음이 검증된 형태로 들어 있다.

| 조각 | 함수 |
|---|---|
| USB 다보드 열거 (pyusb VID/PID `0x1209/0x0D32`, `find_any` 폴백) | `discover_serials()` |
| 시리얼 지정 연결 | `connect()` |
| 6축 수집 + **axis1 = 로봇 우측 → sign −1** | `collect_axes()` |
| 풀캘리 (`calib_scan_omega=6.0`, scan_distance 150, range 0.05) | `calibrate()` |
| 폐루프 진입 (에러 클리어 · `current_lim` · `ignore_illegal_hall_state` · `input_vel=0` 선행 · 진입 확인) | `arm()` |

**따라서 새 드라이버는 이 시퀀스를 이식하고 캐시형 `state()` 와 폴링 스케줄만
얹는다.** 검증된 값을 재발명하지 않는다.

없는 것은 셋이다: 좌/우 차동(스키드) 자체, 텔레메트리 읽기(루프에서 한 번도
읽지 않는다 — 따라서 **읽기를 포함한 50 Hz 예산은 미검증**), 보드↔축
(front/mid/rear) 배정(`sorted(set(serials))` 순서일 뿐).

### 1.4 결함 A — 스키드에서 오도메트리가 특이행렬로 죽는다

`chassis/odometry.py:136-143` 은 **조향륜에만** 측면식을 넣는다.

```python
if w.steerable:
    rows.append((w.name, [1.0, 0.0, -w.y], o.drive_mps * math.cos(d), weight))
    rows.append((w.name, [0.0, 1.0,  w.x], o.drive_mps * math.sin(d), weight))
else:
    rows.append((w.name, [1.0, 0.0, -w.y], o.drive_mps, weight))
```

조향륜이 0개면 모든 행이 `[1, 0, −y]` 뿐이라 정규방정식 행렬의 vy 열·행이
전부 0 이 된다. `_solve3()` 이 피벗 `< 1e-12` 를 만나 `None` 을 반환하고
(`odometry.py:106-107`), `solve_twist()` 는 fail-safe 로 `(0, 0, 0)` 을 낸다
(`odometry.py:224`).

**결과: 스키드 모드에서 오도메트리·`/odom`·콘솔 속도 표시가 항상 0.**

### 1.5 결함 B — 오도메트리 노드가 기하를 따로 들고 있다

오도메트리는 `chassis_node` 가 아니라 별도 노드
`ros2/src/powertrain_ros/powertrain_ros/state_estimation.py:267` 이 자신의
`self.geometry` 로 푼다. `chassis_node` 에서만 조향모드를 바꾸면 이쪽은 애커만
기하로 남는다.

그러면 `_rows()` 가 조향륜마다 측면식
`[0, 1, xᵢ]·(vx, vy, ω) = drive·sin(0°) = 0` 을 넣는데, `x = +0.4377` 과
`x = −0.4377` 이 동시에 걸려 **vy ≈ 0 뿐 아니라 ω ≈ 0 까지 강제**한다.
스키드 주행 중 요레이트 추정이 0 으로 눌린다.

결함 A 와 B 는 별개다. A 는 조향륜이 **0개**일 때 행렬이 특이해지는 문제,
B 는 **두 노드의 기하가 어긋나는** 문제다. 둘 다 고쳐야 한다.

## 2. 아키텍처

구성은 두 축의 곱이다.

|  | 애커만 | 스키드 |
|---|---|---|
| **CAN** (조향 AK×4 + 구동 ODrive×6) | 현행 정본 | ✅ 신규 — AK 가 0° 를 **능동 홀드** |
| **USB** (구동 ODrive×6, 조향 없음) | ⛔ 불가 — 조향 액추에이터 부재 | ✅ 신규 — 이번 요구의 본체 |

레이어별 변경 지점:

```
operator_console  ops_panel 행 1개 + 배지 2개            §9
      │ ops 채널 :9001 (역할토큰)
ops_broker        ACTIONS 항목 1개                        §7
      │ ROS service
chassis_node      SetBool 서비스 + drive_transport 파라미터  §7,§8,§10
      │
ChassisManager    조향모드 전환 상태머신 + geometry 스왑     §7
      │
build_usb_skid_corners()   보드 레지스트리 → (serial, axis)  §5
      │
DriveOdriveUsbAxis + UsbBoardPool                          §4
      │
kinematics.skid_geometry()                                 §3

state_estimation  조향모드 구독 → geometry 동기 스왑        §6.2
```

**CAN × 스키드가 USB × 스키드보다 안전하다.** AK 가 통전 상태면 `solve()` 가
주는 `steer_deg = 0` 을 능동으로 홀드하므로, §12 P1 의 "무통전 조향축이
밀리는가" 리스크가 이 조합에는 존재하지 않는다.

## 3. 키네마틱스 — `skid_geometry()`

`chassis/kinematics.py` 에 추가한다. `solve()` 는 손대지 않는다.

```python
def skid_geometry(track_gain: float = 1.0,
                  base: ChassisGeometry = None) -> ChassisGeometry:
```

- `base` (기본 `default_geometry()`) 의 바퀴를 전부 `steerable=False` 로 복제
- 각 바퀴의 `y` 에 `track_gain` 을 곱한다
- `wheel_radius_m` · `drive_limit_mps` 는 그대로 승계
- `track_gain` 은 유한하고 양수여야 한다 (아니면 `ValueError`)

### 3.1 `track_gain` 의 의미와 방향

실차 스키드는 타이어 측면 미끄럼 저항 때문에 **명령보다 덜 돈다.** 같은 ω 를
실제로 내려면 좌우 속도차를 더 벌려야 하므로 **유효 윤거를 키운다 →
`track_gain > 1.0` 이 보정 방향**이다. 기본값 1.0(무보정)으로 두고 벤치 실측으로
채운다(§12 P3).

기하에 굽는 이유는 **명령과 추정이 한 모델을 쓰게 하기 위해서**다. 오도메트리는
측정 바퀴속도에서 ω 를 역산하는데(`ω ≈ Δv / 2y_eff`), 유효 윤거가 크면 ω 추정도
같이 작아진다. 실제로 로봇이 덜 도는 물리와 방향이 일치한다.

### 3.2 as-built 윤거가 세 축 모두 다르다

앞 545 / 중간 719 / 뒤 425 mm (`kinematics.py:176-180`). 바퀴별 `y` 를 그대로
쓰므로 **강체 기준으로 종방향 슬립이 0** 이다 — 각 바퀴는 자기 위치가 요구하는
전진속도만 명령받는다. 미끄러지는 것은 측면 성분(`vy = ω·xᵢ`)뿐이며, 그것이
스키드 조향의 정의다.

탱크식(한 쪽 3바퀴를 같은 속도로)을 쓰지 않는 이유가 이것이다. 윤거가 다른데
같은 속도를 주면 같은 편 바퀴끼리 종방향으로 싸운다.

같은 편 최대 속도차는 ω=1.2 rad/s 에서
`(0.3595 − 0.2125) × 1.2 / 0.6506 = 0.27 turns/s` 이며,
`WheelConsistencyConfig.same_side_delta_turns_per_s = 0.75` 아래라 오경보를
만들지 않는다.

## 4. USB 다보드 드라이버

새 파일 `motor_control/corner_module/drive_odrive_usb_axis.py`.

### 4.1 `UsbBoardPool`

보드 1장당 odrive 핸들 1개를 소유한다. 같은 보드의 두 축이 공유하므로
`find_any()` 가 보드당 1회만 돈다.

- `discover_serials()` / `connect(serials)` / `calibrate(axis, …)` — 일요일
  스크립트에서 이식
- `close()` 에서 전 축 IDLE 후 정리
- odrive 라이브러리는 **지연 import** — 무하드웨어 pytest 가 이 모듈을 열 수
  있어야 한다 (`build_real_corners` 와 동일 규약)

### 4.2 `DriveOdriveUsbAxis(DriveActuator)`

```python
DriveOdriveUsbAxis(pool, serial, axis_index, *, gear_ratio=5.0, invert=False,
                   current_lim_a=9.0, stale_ms=500.0,
                   poll_slot=0, poll_period_ticks=6, clock=None)
```

계약은 `DriveOdriveCan` 과 동일하다.

- `set_velocity(turns_per_s)` — **바퀴 프레임**. 드라이버 바깥은 전부 바퀴
  프레임이라는 기존 규약을 지킨다.
- `tick()`
  - **쓰기는 매 tick**: `axis.controller.input_vel = target × gear_ratio × sign`
  - **읽기는 자기 슬롯 차례에만**: `encoder.vel_estimate`,
    `motor.current_control.Iq_measured`, `axis.error`, `axis.current_state`
- `state()` — **캐시만 반환하며 USB 를 절대 건드리지 않는다.**
  `CornerModule.tick()` 이 매 tick 호출하기 때문이다(`corner_module.py:162`).
  반환 키: `node_id, target_vel, actual_vel, cur_a, axis_error, axis_state,
  stale, last_rx_age_ms, rx_polls, error_count`
- 모든 USB 접근을 `except Exception` 으로 흡수하고, 실패 시 `_last_rx_ms` 를
  갱신하지 않는다 → `stale` 로 드러난다. `DriveOdriveCan._send` 의 `CanError`
  흡수(`drive_odrive_can.py:123-127`)와 동형이며, 제어 루프가 죽지 않는다.
- `arm()` / `disarm()` / `estop()` / `close()` — 일요일 스크립트 시퀀스 이식.
  `arm()` 은 캘리 미완료면 명확한 에러를 낸다.

`actual_vel` 은 `raw × sign / gear_ratio` 로 바퀴 프레임 환산해 반환한다
(`drive_odrive_can.py:260` 과 동일).

### 4.3 지연 예산 — 이 설계 최대의 미지수

50 Hz = 20 ms 안에 쓰기 6회 + 읽기 4회 = USB 왕복 10회가 들어가야 한다.
일요일 스크립트는 **쓰기 6회만** 돌려본 것이므로 읽기를 얹은 상태는 미검증이다.

기본 설정: 라운드로빈 6슬롯 → 축당 텔레메트리 갱신 주기 120 ms →
`stale_ms = 500.0`.

**§12 P0 에서 실측하고, 20 ms 예산을 넘으면 `loop_hz` 를 낮추거나
`poll_period_ticks` 를 늘린다.** 이 조정은 설계 실패가 아니라 예정된 커미셔닝
단계다.

⚠️ 캘리는 RAM-only 라 **전원 사이클마다** 필요하다(축당 ~55 s). 빌더에
`calibrate=False` 기본 옵션을 둔다.

## 5. 빌더 · 보드 레지스트리

`chassis/chassis_manager.py` 에 추가한다.

```python
def build_usb_skid_corners(registry_path, cfg=None, wheel_map=None,
                           gear_ratio=5.0, current_lim_a=9.0,
                           calibrate=False, find_timeout=20.0) -> dict:
```

- `drive.bl70200.board_registry.load()` → `{serial: (node_ax0, node_ax1)}` 를
  역인덱스해 `node → (serial, axis)` 를 만든다.
- **`DEFAULT_WHEEL_MAP` 의 `drive_node_id` 를 그대로 쓴다.** 바퀴↔노드 권위가
  CAN 경로와 하나로 유지된다. 새 매핑 표를 만들지 않는다.
- 조향은 전부 `NullSteer` — `steer_can_id` 는 무시한다.
- 반전은 기존 `RIGHT_WHEELS`(`chassis_manager.py:91`)에서 유도한다. CAN 빌더와
  같은 권위를 쓴다.
- **교차검증**: `RIGHT_WHEELS` 에 속한 바퀴가 전부 axis1 로 해석되는지 단언한다
  (실물 미러 장착 사실). 어긋나면 시작을 거부한다 — 오배선·레지스트리 오타
  조기 발견.
- 레지스트리에 없는 노드는 `ValueError`. **추측 배정을 하지 않는다.**

새 파일 `config/bl70200_boards.json` — 시리얼은 벤치에서 채운다. 파일이 없으면
진입점이 명확한 에러와 함께 `--print-serials` 사용법을 띄운다.

## 6. 오도메트리 수정

### 6.1 결함 A — 비홀로노믹 prior

`odometry._rows()` 가 **측면식을 한 줄도 만들지 못한 경우에만** prior 한 줄을
추가한다.

```python
if not any_lateral_row:
    rows.append((None, [0.0, 1.0, 0.0], 0.0, cfg.vy_prior_weight))
```

- `OdometryConfig.vy_prior_weight: float = 1e-3` 신설
- 이름 `None` 은 잔차 계산·이상치 배제·`min_wheels` 집계에서 제외한다
- **조건부라 기존 4WS 경로는 완전히 불변**이다 (회귀 0)

물리 의미: "스키드는 측면 속도를 명령하지 않는다." 스크럽으로 실제 측면 이동이
생겨도 바퀴로는 관측할 수 없으므로 0 이 최선의 사전분포다.

### 6.2 결함 B — 노드 간 기하 동기화

조향모드를 `/chassis/safety_state` (JSON `String`, `chassis_node.py:666-670`)
에 실어 방송하고 — `component_mask` 가 이미 그렇게 실려 나간다
(`chassis_node.py:1685-1687`) — `state_estimation` 이 구독해 자신의
`self.geometry` 를 함께 스왑한다. 필드 1개 + 구독 1개.

⚠️ 순서 보장이 없으므로, 두 노드의 기하가 잠시 어긋나는 구간이 존재한다.
전환은 `steer_mode_change` MOTION_HOLD 하에서만 일어나며(§7) 그 동안 차체는
정지 상태이므로, 이 과도구간에 잘못된 트위스트가 적분될 여지는 없다.

## 7. 조향모드 런타임 전환

CMASK 컴포넌트 토글과 **완전히 같은 모양**으로 붙인다.

| 계층 | 추가 |
|---|---|
| `ops_contract.py` | `"steer_mode_skid": ActionSpec(_CONSOLE, "service_setbool", ("/chassis_node/steer_mode_skid",))` — true=스키드, false=애커만 |
| `chassis_node` | `SetBool` 서비스 → `ChassisManager.request_steering_mode()` |
| `ops_panel.py` | `PanelAction(needs_bool=True, bool_value_from_state=…)` 행 1개 |
| ops state | `steering_mode`, `steering_available`, `drive_transport` 3필드 |

기존 `component_enable_*`(`ops_contract.py:102-117`)과
`_component_toggle_value`(`ops_panel.py:158-163`)가 그대로 본이다.

### 7.1 전환 안전 시퀀스 (`ChassisManager`)

1. `SafetyInterlock.set_motion_hold("steer_mode_change", True)` — 기존 hold
   기계장치를 그대로 쓴다
2. v = 0, ω = 0
3. 조향 4륜에 0° 를 명령하고 **실각 수렴을 대기**한다
   (`|actual_deg| < gate_deg` 또는 타임아웃)
4. `cfg.geometry` 스왑 + `WheelConsistencyMonitor` 재생성
   (`chassis_manager.py:188-191` 에서 생성자에 한 번만 만들어지므로 반드시
   같이 갱신해야 한다)
5. hold 해제

**3번이 핵심이다.** 45° 로 꺾인 상태에서 곧바로 차동 구동을 걸면 격렬한 스크럽이
난다. 타임아웃으로 수렴에 실패하면 전환을 거부하고 hold 를 유지한다.

스키드 → 애커만도 같은 경로를 탄다. 조향은 이미 0° 이므로 사실상 즉시
통과하지만, 대칭성과 "정지 후 전환" 보장을 위해 게이트를 공유한다.

### 7.2 가용성 판정

모든 코너의 steer 가 `NullSteer` 이면 애커만이 불가능하다 → 서비스가
`FINAL_REJECTED` 를 낸다. `steering_available=False` 로 ops state 에 실려
GUI 가 회색 처리한다. USB 스택이 여기 해당한다.

## 8. 트랜스포트 선택 (기동 시)

런타임 전환은 불가능하다. 드라이버 재생성 + `connect()` + 전원 사이클마다
축당 ~55 s 풀캘리가 필요하고, `ChassisManager` 는 corners 를 생성자에서 받아
기하↔코너 이름 일치를 거기서 검증한다(`chassis_manager.py:208-217`).

따라서 콘솔에서는:

- **현재 트랜스포트를 배지로 상시 표시**한다 (ops state `drive_transport`)
- 다른 트랜스포트를 고르면 **"재기동 필요" 확인 → 모드 파일 기록 + 정확한
  재기동 명령 표시**. 모드 파일은 `/etc/powertrain/drive_transport` 이며
  (`/etc/powertrain/ops_*.token` 과 같은 위치·소유 규약), 내용은 `can` 또는
  `usb` 한 줄이다. `control.launch.py` 가 기동 시 이 값을 읽어 `chassis_node`
  의 `drive_transport` 파라미터로 넘긴다. 파일이 없거나 값이 인식되지 않으면
  `can` 으로 기동한다.
- **콘솔이 서비스를 직접 재기동하는 것은 v1 범위 밖이다.** 콘솔 헌장이
  "관측 수신 + 게이트된 ops 명령"인데, 제어 스택 재기동은 그 등급을 넘는다.
  필요하면 별도 안건으로 다룬다.

## 9. GUI — `operator_console`

- **배지 2개**: `구동: CAN | USB`, `조향: 애커만 | 스키드`
- **토글 행 1개**: 조향모드 (§7). `GESTURE_STRIP` + `needs_bool=True`,
  기존 CMASK 행들과 같은 확인 흐름
- **USB × 애커만 칸은 회색**이며 사유를 표시한다 —
  "USB 스택에는 조향 액추에이터가 없습니다"
- 트랜스포트 선택은 §8 대로 "재기동 필요" 확인 경로

`ops_panel.py` 는 순수 정책 모듈이므로 회색 처리 판정도 여기서 하고,
`app.py` 는 렌더링만 한다(기존 분리 유지).

## 10. 진입점

### 10.1 `chassis_node` (GUI 경로의 본체)

- 파라미터 `drive_transport: {"can", "usb"}` (기본 `"can"`)
- 파라미터 `steering_mode: {"ackermann", "skid"}` (기동 초기값)
- 파라미터 `board_registry` (기본 `config/bl70200_boards.json`),
  `track_gain`, `calibrate`, `current_lim`
- `usb` 일 때 `build_usb_skid_corners()` 를 쓰고 **`RealCanSession` 을 획득하지
  않으며 `CanWatchdog` 을 띄우지 않는다** — can0 을 아예 건드리지 않는다
- `usb` + `steering_mode=ackermann` 조합은 기동 시 거부한다
- `control.launch.py` 에 대응 `DeclareLaunchArgument` 추가

### 10.2 `teleop_server` (무선 수동 주행)

`--skid-usb` (+ `--board-registry`, `--calibrate`, `--current-lim`,
`--track-gain`):

- `CanWatchdog` 미기동, `RealCanSession` 미획득
- `build_usb_skid_corners()` + `cfg.geometry = skid_geometry(args.track_gain)`
- **US-100 은 그대로 켠다** (UART 라 CAN 과 무관). 인터록·워치독·estop 전파
  전부 유효하다.
- `--diagnostic-direct-can` 게이트와 팔 stowed 확인은 **그대로 요구한다.**
  이름은 CAN 이지만 의미는 "ROS 우회 직접 모터 제어"로 동일하다.
- 노트북 클라이언트·프로토콜 무변경. 지금까지 버려지던 `left_x`
  (`dualsense_usb_teleop.py:194`)가 드디어 ω 로 쓰인다.
- `--four-wheel` 과 `--skid-usb` 동시 지정은 v1 에서 argparse 에러다.

## 11. 테스트 (무하드웨어 pytest)

| 파일 | 검증 |
|---|---|
| `chassis/tests/test_kinematics.py` | `skid_geometry` 전륜 non-steerable · 직진=좌우 동일 · 피벗(v=0,ω>0)=좌 음/우 양 · `track_gain` 2배 → 좌우차 2배 · `steer_deg` 전부 0 · `track_gain ≤ 0` 거부 |
| `chassis/tests/test_odometry.py` | 스키드 관측 → (v, ω) 복원 (**현재는 (0,0,0) — 음성 대조로 먼저 확인**) · 4WS 결과 불변 회귀 |
| `corner_module/tests/test_drive_odrive_usb_axis.py` | fake odrive 핸들 주입 — 기어비·반전 왕복 · 라운드로빈 슬롯 · USB 예외 → `stale` · **`state()` 가 USB 를 안 건드림** |
| `chassis/tests/test_chassis_manager.py` | 레지스트리 누락/중복/axis-side 불일치 거부 · 조향모드 전환 시퀀스(조향 미수렴 시 전환 거부·hold 유지·monitor 재생성) · `steering_available` |
| `chassis/tests/test_teleop.py` | `--skid-usb` 시 `CanWatchdog`·`RealCanSession` 미호출 · `--four-wheel` 조합 거부 |
| `powertrain_ros/test/test_ops_contract.py` | 새 액션 역할·타깃 · 기존 계약 불변 |
| `operator_console/tests/test_ops_panel.py` | 토글 행 · 회색 처리 판정 · 배지 문자열 |

## 12. 실행 검증 게이트

프로젝트 규칙상 **단위테스트 green 만으로 완료 선언 금지**다. 아래를 실제로
돌린다.

| 게이트 | 내용 | 통과 기준 |
|---|---|---|
| **P0 지연 실측** | 젯슨에서 6축 arm 후 50 Hz 100초 | tick p99 < 20 ms · 오버런 0 · 축별 텔레메트리 age < `stale_ms` |
| **P1 바퀴 들고 스키드 E2E** | 노트북 DualSense → 직진(6축 동방향) · 제자리선회(좌 음/우 양) · 좌우 비대칭 | 육안 + 텔레메트리 일치. **무통전 조향축이 밀리는지 육안 확인** |
| **P2 안전** | ○ estop · 링크 끊김 · US-100 근접 | 6축 전부 0 |
| **P3 지상 + `track_gain` 실측** | 조립 후 별도 판단 | 명령 ω 대비 실측 ω 비 → `track_gain` 확정 |
| **P4 조향모드 전환 E2E** | 45° 꺾인 상태에서 스키드 전환 | 조향 0° 수렴 **후에만** 차동이 걸림. 미수렴 시 전환 거부 |
| **P5 콘솔 실기동** | `python3 -m operator_console.runtime_smoke` | Xvfb 실기동 · 토글/회색처리/배지 · traceback 0 |

**검증 게이트 자체를 음성 대조로 증명한다** — 결함 A 는 수정 전에 (0,0,0) 이
나오는 것을 먼저 확인하고, P4 는 수렴 대기를 일부러 빼서 FAIL 이 나는지 본다.

### 12.1 무통전 AK 조향축 리스크

CAD 상 킹핀 스크럽 반경이 0 이므로(`kinematics.py:177-178` — 조향 4륜의 타이어
링크 원점이 킹핀 축과 Δ0.0 mm) 측면력이 킹핀 모멘트를 거의 만들지 않는다.
유리한 조건이다.

다만 타이어 셀프얼라이닝 토크 대 AK45-36 백드라이브 0.8 Nm 의 대소는 미지다.
**P1 에서 조향각이 밀리는지 육안 확인을 명시적 게이트로 둔다.** 밀리면 기계적
고정(핀·스토퍼)이 필요하고 그것은 SW 범위 밖이므로 보고만 한다.

CAN × 스키드에는 이 리스크가 없다 — AK 가 0° 를 능동 홀드한다.

## 13. 미해결 · 후속

- `track_gain` 실측값 (P3, 조립 후)
- `config/bl70200_boards.json` 시리얼 (벤치)
- USB 폴링 주기 최종값 (P0 결과에 따름)
- 콘솔발 제어 스택 재기동 (§8, 별도 안건)
- 구동 USB + 조향 CAN 혼합 구성 (빌더 구조는 나중에 끼울 수 있게 둔다)
- `skid_geometry(base=four_wheel_geometry())` 4륜 스키드 — 구조는 되지만 v1
  진입점에는 뚫지 않는다

## 14. 결정 기록

| # | 결정 | 근거 |
|---|---|---|
| 1 | `solve()` 를 재사용하고 별도 스키드 수식을 만들지 않는다 | 고정 바퀴 분기가 이미 정확한 식이다 (§1.1) |
| 2 | 탱크식(편측 동일 속도)을 쓰지 않는다 | as-built 윤거가 세 축 다르다 → 같은 편 종방향 싸움 (§3.2) |
| 3 | `track_gain` 을 기하에 굽는다 | 명령과 추정이 한 모델을 쓴다 (§3.1) |
| 4 | `DriveOdriveUsb` 를 수정하지 않고 새 클래스를 만든다 | `corner_module/teleop_dualsense.py` 가 아직 쓴다 (§1.2) |
| 5 | 일요일 스크립트 시퀀스를 이식한다 | 벤치 6축 실회전 검증됨 (§1.3) |
| 6 | 쓰기는 매 tick, 읽기는 라운드로빈 | USB 왕복 예산 (§4.3) |
| 7 | 보드 레지스트리 없으면 시작 거부 | 바퀴 오배정은 조용히 틀린다 (§5) |
| 8 | 오도메트리 prior 는 조건부 | 기존 4WS 회귀 0 (§6.1) |
| 9 | 조향모드만 런타임, 트랜스포트는 기동 시 | USB 캘리 ~55 s/축 · corners 는 생성자 고정 (§8) |
| 10 | USB × 애커만은 비활성 | 조향 AK 는 CAN 전용 (§7.2) |
| 11 | 콘솔 재기동은 v1 범위 밖 | 콘솔 헌장 등급 초과 (§8) |
