# L515 Gateway·TUI 원격주행 설계

> 작성일: 2026-07-11 KST
> 상태: Gateway 구조 승인
> 선행 작업: `2026-07-11-l515-lightweight-pipeline-design.md`

## 1. 목적과 범위

원격주행과 자율주행이 같은 L515 입력을 안정적으로 공유하도록, 시스템 전체에서 L515를
하나의 장기 실행 Gateway 프로세스만 점유한다. Gateway는 기존 ROS 발행, SDK 정합,
H.264/SRT 영상 송신을 함께 수행한다. 별도 Textual Dashboard 프로세스는 Unix domain
socket으로 상태를 표시하고 제어 명령만 전달한다.

핵심 계약은 다음과 같다.

- `l515_gateway`가 `pyrealsense2 2.50.0`과 L515 serial `00000000F0271544`를 단독 점유한다.
- Gateway가 기존 자율주행용 ROS 6개 토픽을 발행하므로 새 정합 토픽은 추가하지 않는다.
- RGB 1280×720×30, raw depth 640×480×30, accel/gyro를 수집한다.
- SDK 내부 정합 결과는 Gateway 내부 SRT 합성에만 사용한다.
- SRT 출력은 모든 모드에서 1280×720×30으로 고정해 GStreamer를 재시작하지 않는다.
- Dashboard/SSH가 종료돼도 Gateway와 원격주행 영상은 계속 동작한다.
- Gateway는 관리되는 서비스이며 system-wide singleton이다. Dashboard는 카메라를 열지 않는다.

객체 검출, 오도메트리, PointCloud2, 녹화, 웹 UI, 주행 명령 자체는 범위 밖이다.

## 2. 프로세스와 소유권

### 2.1 `l515_gateway`

- L515 SDK pipeline 하나 소유
- color/depth/IMU 수집과 timestamp 관리
- 기존 ROS 6개 토픽 발행
- `rs.align(rs.stream.color)` 기반 depth→RGB 정합
- RGB, 정합 Depth, 반투명 오버레이 생성
- GStreamer H.264/MPEG-TS/SRT 송신
- Unix socket 상태·명령 서버
- USB 분리 시 재연결과 stale frame 억제

### 2.2 `l515_dashboard`

- Unix socket client
- Gateway·센서·ROS·SRT·자원 상태 표시
- 영상 모드, 송신 활성화, Gateway 재시작·정지 명령
- L515, ROS Image, GStreamer stdin 직접 소유 금지

Dashboard `q`는 Dashboard만 종료한다. `Shift+Q`는 확인 뒤 Gateway 전체 정지를 요청한다.
SSH/SIGHUP으로 Dashboard가 사라져도 Gateway는 영향을 받지 않는다.

### 2.3 관리 주체

Gateway는 ROS 전용 Docker 컨테이너의 명시적 entrypoint/supervisor가 관리한다. persistent
`flock`은 system-wide singleton을 보장한다. Gateway가 살아 있는 상태에서 두 번째
Gateway는 실행을 거부한다. Dashboard는 여러 개가 읽기 전용으로 접속할 수 있지만, 상태 변경
명령은 서버가 직렬화한다.

`powertrain_ros`는 host `/run/powertrain`을 container의 같은 경로에 bind-mount한다. 따라서
교체·중복 container도 같은 `l515-gateway.lock` inode에서 경쟁한다. `network_mode: host`의
abstract endpoint도 host network namespace에서 하나뿐이다. 시작 순서는 반드시
`flock 획득 → abstract endpoint bind/listen → SDK → ROS → SRT`이며, lock 또는 bind 실패는
L515 SDK를 열기 전에 rollback한다.

## 3. 데이터 계약

### 3.1 SDK 프로파일

| 입력 | 프로파일 |
|---|---|
| Color | BGR8 1280×720×30 |
| Depth | Z16 640×480×30 |
| Accel | 장치 지원 기본 profile |
| Gyro | 장치 지원 기본 profile |

### 3.2 기존 ROS 출력

- `/l515/color/image_raw`
- `/l515/color/camera_info`
- `/l515/depth/image_rect_raw`
- `/l515/depth/camera_info`
- `/l515/accel/sample`
- `/l515/gyro/sample`

Color CameraInfo는 1280×720, raw Depth CameraInfo는 640×480이다. PointCloud2, aligned depth,
IR, confidence 토픽은 발행하지 않는다. 기존 소비자가 640×480 color를 가정한 경우 WP6/WP7
착수 전에 새 명시적 color 계약으로 변경한다.

### 3.3 내부 정합과 SRT

SDK frameset을 한 소유자 안에서 `rs.align(color)`로 처리한다. raw depth는 기존 ROS 출력에
사용하고 aligned depth는 ROS에 내보내지 않고 영상 합성 worker에만 넘긴다.

SRT canvas는 항상 1280×720 BGR8이다.

| 키/명령 | 영상 |
|---|---|
| `1` | RGB 원본 |
| `2` | RGB 좌표계에 정합된 Depth 컬러맵 |
| `3` | RGB + 정합 Depth 반투명 오버레이 |

모드 전환은 다음 출력 프레임부터 적용하며 SDK/GStreamer/ROS 프로세스를 재시작하지 않는다.
Depth가 없으면 모드 2·3의 송신을 중단하고 마지막 프레임을 반복하지 않는다.

## 4. Gateway 내부 구조

| 모듈 | 책임 |
|---|---|
| `resource_guard.py` | persistent regular-file `flock` 기반 단일 물리자원 소유권 |
| `gateway.py` | lifecycle, SDK reconnect, 상태 전이 |
| `ros_publisher.py` | 기존 6개 토픽 변환·발행 |
| `alignment.py` | SDK color alignment와 Depth 컬러맵/overlay |
| `streamer.py` | 고정 1280×720 GStreamer/SRT worker |
| `protocol.py` | versioned JSON command/status schema |
| `control_server.py` | Unix socket server와 client별 backpressure |
| `app.py` | Textual Dashboard client |

SDK worker는 최신 frameset 하나만 전달한다. ROS publisher와 SRT worker는 각자 bounded
latest-one-slot을 소비하며 서로를 block하지 않는다. 오래된 영상, SDK frame, status message를
무한히 쌓지 않는다.

## 5. Unix socket 프로토콜

기본 endpoint 표기는 `@powertrain-l515-gateway`이며 Linux abstract Unix 주소의 선행 NUL로
변환해 bind/connect한다. filesystem socket 경로는 만들거나 삭제하지 않는다. Abstract socket에는
파일 권한이 없으므로 서버는 `SO_PEERCRED`의 UID가 Gateway UID와 같은 client만 명령 처리한다.
메시지는 길이 제한이 있는 newline-delimited
JSON이며 `protocol_version`, `request_id`, `type`, `payload`를 가진다.

명령은 다음으로 제한한다.

- `get_status`
- `set_video_mode`: `rgb`, `depth`, `overlay`
- `set_streaming`: boolean
- `restart_gateway`
- `stop_gateway`: Dashboard에서 별도 확인 필요

상태는 Gateway state, SDK serial/profile, 각 stream FPS·age·gap·timestamp 이상, ROS publish
count, SRT client/송신/drop, CPU/RAM, 마지막 오류를 포함한다. 알 수 없는 version/type, 과대
메시지, 잘못된 값은 연결 단위 오류로 거부하고 Gateway를 종료하지 않는다.

## 6. 상태와 장애 처리

Gateway 상태는 `STARTING`, `RUNNING`, `DEGRADED`, `STOPPING`, `STOPPED`, `FAULT`다.

- L515 분리: `DEGRADED`, ROS/SRT stale replay 중단, 2초 간격 exact-serial 재탐색
- L515 복구: 새 세션 timestamp/dedup 상태 초기화 뒤 ROS·SRT 자동 재개
- GStreamer crash: sensor/ROS는 유지하고 streaming을 off, 상태 `DEGRADED`; 명시적 재시작 가능
- ROS publisher 오류: Gateway `FAULT` 후 전체 종료
- SDK unrecoverable 오류: `FAULT` 후 전체 종료
- Dashboard crash/disconnect: Gateway 상태 불변

원격주행에서 카메라/ROS 입력을 유지하는 편이 중요하므로 GStreamer 단독 장애는 Gateway 전체를
죽이지 않는다. 이는 이전 TUI-parent 설계의 전체 종료 정책을 대체한다.

## 7. 종료 안전성과 공통 resource guard

Gateway 하나가 SDK와 GStreamer 자식을 소유한다. 정상 종료는 신규 frame 차단 → GStreamer
stdin 종료·reap → SDK pipeline stop → ROS publisher 종료 → abstract socket close → lock unlock/close 순서다.
SIGINT/SIGTERM, container stop, 내부 예외는 하나의 멱등성 shutdown으로 합친다.

`resource_guard`는 `/run/powertrain/l515-gateway.lock`을 `O_NOFOLLOW`로 열고 owner-controlled
regular file인지 검증한 뒤 nonblocking exclusive `flock`을 잡는다. lock을 보유한 동안 PID와
`/proc/<pid>/stat` 시작 identity metadata를 덮어쓰고 fsync한다. 종료 시 unlock/close만 하며
lock pathname은 절대 unlink하지 않는다. stale metadata/file은 정상이며 다음 lock owner가 갱신한다. 이 유틸리티는
향후 US-100 UART, ODrive USB, CAN maintenance authority에 재사용 가능하지만 이번 작업에서
그 장치들의 동작은 변경하지 않는다.

Abstract server는 hardware보다 먼저 떠서 Gateway가 아직 `STARTING`일 수 있다. 이 구간에는
command accept gate가 상태 변경 명령을 거부하며, 모든 component가 준비된 뒤에만 허용한다.

## 8. 테스트와 HIL

자동시험:

- singleton 두 contender, persistent stale file, release/reacquire, symlink 거부
- SDK frameset 분배와 bounded latest slots
- 1280×720 RGB, aligned Depth, overlay 결과
- 6개 ROS 토픽 profile·timestamp 계약
- 고정 1280×720 GStreamer argv와 세 모드 무재시작 전환
- abstract socket framing, SO_PEERCRED UID 권한, version, 명령 직렬화, 과대·오염 입력
- Dashboard 접속·재접속·종료 독립성
- USB 분리·복구와 stale frame 0
- Gateway 정상/부분시작/신호/자식 crash 종료 뒤 고아 0

Jetson HIL:

1. Gateway 단독 실행과 Dashboard 접속·재접속
2. 기존 6개 ROS 토픽과 SRT 1280×720×30 동시 측정
3. RGB/Depth/overlay 전환 시 Gateway/GStreamer PID 유지
4. Dashboard/SSH 강제 종료 뒤 ROS·SRT 지속
5. 사용자 승인 후 L515 분리·복구
6. GStreamer crash 뒤 ROS 지속과 streaming 재시작
7. Gateway 종료 뒤 SDK/GStreamer/abstract socket listener/flock owner 0 (persistent lock file은 유지)
8. D435i 로봇팔 perception 동시부하와 USB 오류 delta 0

## 9. 완료 기준

- L515 system-wide owner가 Gateway 하나뿐이다.
- 자율주행 ROS 6개 토픽과 원격주행 SRT가 동시에 동작한다.
- 새 aligned depth ROS 토픽 없이 세 영상 모드를 무재시작 전환한다.
- Dashboard/SSH 종료가 Gateway, ROS, SRT를 끊지 않는다.
- USB 분리·재연결 때 stale frame이나 D435i fallback이 없다.
- Gateway 종료 뒤 모든 소유 자원과 자식 프로세스가 정리된다.
- 자동시험·Jetson HIL·최종 리뷰가 통과한다.
