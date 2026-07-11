# L515 TUI 진단·영상 송신 통합 관리자 설계

> 작성일: 2026-07-11 KST  
> 상태: 구현 전 승인 설계  
> 선행 작업: `2026-07-11-l515-lightweight-pipeline-design.md`

## 1. 목적과 범위

Jetson에서 L515 경량 ROS 파이프라인의 상태를 터미널에서 진단하고, 기존 H.264/SRT
영상 송신을 같은 생명주기로 제어하는 Python TUI를 만든다. TUI는 텍스트 진단만 표시하고
영상은 별도 SRT 수신 화면으로 전송한다.

본 작업의 핵심 계약은 다음과 같다.

- TUI가 L515 ROS 노드와 영상 송신기를 한 스택으로 시작·감시·종료한다.
- L515는 기존 `l515_camera` 노드만 직접 점유한다.
- 영상 송신기는 ROS Image 토픽을 구독하므로 카메라를 두 번 열지 않는다.
- Color, Depth 컬러맵, Color+Depth 나란히 화면을 실행 중 전환한다.
- 정상 종료, 중간 실패, SSH/터미널 단절 뒤에도 고아 프로세스가 남지 않는다.

오도메트리, 객체 검출, PointCloud2 생성, 녹화, 원격 웹 제어는 범위 밖이다.

## 2. 사용자 경험

대시보드를 실행하면 preflight 뒤 전체 스택이 자동으로 시작된다. 기존 프로세스에 붙는
attach 모드는 제공하지 않는다. 중복 실행은 거부한다.

화면은 다음 정보를 표시한다.

- 전체 상태: `STARTING`, `RUNNING`, `DEGRADED`, `STOPPING`, `STOPPED`, `FAULT`
- L515 USB·SDK·serial 상태
- color, depth, color/depth CameraInfo, accel, gyro 토픽별 FPS와 age
- 영상·IMU timestamp 비증가 횟수와 최근 최대 gap
- 현재 영상 모드, SRT 송신 FPS, frame drop, GStreamer 상태
- L515 스택 CPU/RAM과 최근 오류

기본 키는 다음과 같다.

| 키 | 동작 |
|---|---|
| `1` | Color 원본 송신 |
| `2` | Depth 컬러맵 송신 |
| `3` | Color+Depth 좌우 결합 송신 |
| `r` | 전체 스택 순차 재시작 |
| `q` | 전체 정상 종료 후 TUI 종료 |
| `?` | 도움말 표시 |

기본 SRT listener 포트는 기존 파이프라인과 같은 5000이며 latency·encoder·포트는 CLI 또는
설정으로 변경할 수 있다.

## 3. 아키텍처

새 top-level 패키지 `l515_dashboard/`를 둔다.

| 모듈 | 책임 |
|---|---|
| `app.py` | Textual 화면, 키 입력, 상태 렌더링 |
| `supervisor.py` | preflight, 자식 process group, 상태 전이, 재시작·종료 |
| `diagnostics.py` | ROS 구독, 토픽별 FPS·age·gap·timestamp 통계 |
| `streamer.py` | Image 구독, 모드별 프레임 생성, GStreamer stdin 공급 |
| `frame_modes.py` | Color, Depth 컬러맵, 좌우 결합 변환 |
| `config.py` | 포트, latency, encoder, 임계값, 종료 timeout |

기존 `motor_control/vision/gst_stream.py`의 encoder 선택과 SRT GStreamer argv 생성기를
재사용한다. 기존 `realsense_stream.py`는 SDK를 직접 점유하므로 실행하거나 재사용하지 않는다.

데이터 흐름은 다음과 같다.

1. supervisor가 `scripts/l515_preflight.sh`를 실행한다.
2. supervisor가 `l515_camera`를 새 process group에서 시작한다.
3. diagnostics/streamer ROS 노드가 여섯 토픽을 구독한다.
4. 필수 토픽이 들어오면 GStreamer를 시작하고 `RUNNING`으로 전이한다.
5. streamer는 최신 color/depth 한 장만 보관하며 선택한 모드로 변환해 SRT로 보낸다.
6. TUI는 진단 snapshot만 받아 렌더링하고 영상 데이터는 직접 그리지 않는다.

ROS QoS는 기존 sensor-data profile과 맞춘다. 느린 송신기가 ROS callback을 막지 않도록 최신
프레임 한 장 슬롯과 별도 worker를 사용한다. 오래된 프레임을 쌓거나 재전송하지 않는다.

## 4. 영상 모드

- Color: `/l515/color/image_raw`의 BGR8 640×480 프레임
- Depth: `/l515/depth/image_rect_raw`의 16UC1을 가시 범위로 정규화한 컬러맵
- 나란히: Color와 Depth 컬러맵을 같은 높이로 결합한 1280×480 프레임

모드 전환은 카메라나 GStreamer 프로세스를 재시작하지 않고 다음 출력 프레임부터 적용한다.
Depth 프레임이 아직 없으면 검은 화면을 재생하지 않고 해당 모드의 송신을 잠시 멈추며 TUI를
`DEGRADED`로 표시한다.

## 5. 프로세스 소유권과 종료 안전성

supervisor만 자식 프로세스를 만들고 소유한다. 자식은 새 session/process group에서 실행하며
직접 만든 `Popen` 객체와 group만 제어한다. 저장된 숫자 PID를 무조건 kill하지 않는다.

부모 사망 시 자식도 종료되도록 Linux `PR_SET_PDEATHSIG`를 적용한다. TUI는 SIGINT,
SIGTERM, SIGHUP, 예외, 정상 `q`, `atexit`을 하나의 멱등성 `shutdown()`으로 합친다.

정상 종료 순서는 고정한다.

1. 신규 영상 프레임 입력 차단
2. GStreamer stdin 닫기와 제한시간 대기
3. 남아 있으면 GStreamer process group에 SIGTERM, 제한시간 뒤 SIGKILL
4. L515 ROS 노드 process group에 SIGINT와 제한시간 대기
5. 남아 있으면 SIGTERM, 제한시간 뒤 SIGKILL
6. ROS 구독과 TUI 자원 해제
7. 자신이 만든 lockfile 제거

시작 도중 preflight만 끝난 경우, ROS만 시작된 경우, GStreamer만 생성된 경우에도 같은
`shutdown()`이 안전하게 동작해야 한다. shutdown은 여러 signal·예외 경로에서 반복 호출돼도
같은 결과를 내야 한다.

lockfile에는 PID만 쓰지 않고 process 시작 identity를 함께 기록한다. 살아 있는 동일
dashboard가 확인될 때만 두 번째 실행을 거부하고, 죽은 프로세스의 stale lock은 회수한다.

## 6. 장애 처리와 상태 전이

- L515 분리 또는 토픽 timeout: `DEGRADED`, 영상 입력 중단, 기존 ROS 재연결 유지
- L515 복구: 필수 토픽 freshness 회복 뒤 영상 자동 재개, `RUNNING`
- GStreamer 비정상 종료: `FAULT`, 전체 스택 종료
- L515 ROS 프로세스 비정상 종료: `FAULT`, 전체 스택 종료
- preflight 실패: 자식을 시작하지 않고 `FAULT`
- TUI 내부 예외: 오류를 기록하고 전체 스택 종료

마지막 영상 프레임 반복, D435i fallback, 영상 없이 센서만 남는 불완전 운영 상태는 허용하지
않는다. 사용자가 `r`을 누르면 현재 스택이 완전히 `STOPPED`가 된 뒤 새 스택을 시작한다.

## 7. 테스트 전략

하드웨어 독립 테스트는 fake ROS message source와 fake GStreamer process를 사용한다.

- FPS·age·gap·timestamp 비증가 통계
- 세 영상 모드와 런타임 전환
- latest-one-slot overwrite와 stale frame 미재생
- 정상 시작·종료와 역순 자원 정리
- preflight, ROS 시작, GStreamer 시작 각 단계 실패
- GStreamer crash와 ROS crash의 전체 `FAULT` 종료
- SIGINT, SIGTERM, SIGHUP, 내부 예외, 반복 shutdown
- graceful timeout 뒤 SIGTERM/SIGKILL escalation
- stale/live lockfile 구분과 PID 재사용 방지
- L515 분리·복구 상태 전이

subprocess 통합 테스트는 매 시나리오 뒤 dashboard가 만든 process group과 자식 PID가 0개인지
검사한다.

Jetson HIL은 마지막에 한 번 수행한다.

1. TUI로 전체 스택 시작
2. 노트북 `scripts/recv_stream.sh`로 SRT 수신
3. Color, Depth, 나란히 모드 전환과 진단 수치 확인
4. L515 USB 분리 시 영상 중단·`DEGRADED`·D435i 미선택 확인
5. 재연결 후 자동 복구 확인
6. `q`, SIGINT, SIGTERM, SIGHUP 각 종료 뒤 고아 프로세스 0 확인
7. USB 오류와 node/GStreamer 오류 로그 확인

## 8. 완료 기준

- TUI 한 명령으로 L515 ROS와 SRT 송신이 함께 시작·종료된다.
- 세 영상 모드를 프로세스 재시작 없이 전환한다.
- 텍스트 진단이 여섯 토픽과 송신 상태를 실시간 반영한다.
- L515 분리 때 stale 영상이나 D435i fallback 없이 자동 복구한다.
- 모든 단위·통합 테스트와 Jetson HIL 종료 시 고아 프로세스가 0개다.
- 기존 L515 30 Hz·timestamp·USB 안정성 계약을 유지한다.
