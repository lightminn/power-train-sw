# L515 Gateway 성능 개선 및 HIL 보고서

> 상태: **CONNECTED HIL COMPLETE / Notion 동기화만 미완료**
>
> 기준 commit: `e755b7b058e07952fc7e74140934ff725b9c35d1`
>
> exact image: `sha256:eef0a22a1745741b92bf2ad62074fed63ea08e806aa79cada469d7f83d096482`

## 1. 범위와 정본

파워트레인 L515 serial `00000000F0271544` 한 대를 Gateway가 독점하고, ROS Image/
CameraInfo/Imu 6토픽과 RGB/Depth/overlay SRT를 제공하는 경로를 Jetson Orin Nano에서
검증했다. D435IF serial `250222071245`는 로봇팔 perception 전용으로 유지했다.

배포 이미지는 pyrealsense2/librealsense 2.50.0 RSUSB, `videoconvert`와 `x264enc`
GStreamer 1.20.1, SRT 1.20.3, 기존 entrypoint를 포함한다. Orin Nano에는 NVENC가 없어
`nvv4l2h264enc`를 노출하지 않는다. 실제 L515 영상 기준 최종 encoder는
`x264enc tune=zerolatency speed-preset=ultrafast threads=3 bitrate=3000 key-int-max=30`이다.

## 2. 성능 문제와 수정

| 문제 | 실측 원인 | 수정 |
|---|---|---|
| active stream이 1~2초 뒤 끊김 | RSUSB 사용 중 새 context로 `query_devices()`를 반복하면 일시적으로 none 반환 | 시작 전 exact-serial 열거만 유지하고 active 상태는 video callback freshness로 판정 |
| 첫 alignment에서 Gateway FAULT | SDK callback 객체는 base `frame`, `rs.align.process`는 `composite_frame` 요구 | retained frame을 `as_frameset()`으로 변환 |
| raw Depth 약 6.4 Hz | 55 ms 작업 뒤 다시 100 ms를 기다리는 work+period cadence | deadline 기반 10 Hz, overrun catch-up burst 금지 |
| RGB SRT 약 23~24 Hz | all-zero benchmark가 실제 영상 x264 비용을 과소평가; 실제 frame의 superfast는 23.85 fps | real-frame sweep에서 ultrafast/threads=3 선택 |
| RGB writer와 alignment 경쟁 | RGB mode에도 불필요한 alignment 수행 | RGB에서는 alignment 0회, Depth/overlay에서만 best effort 수행 |
| in-process restart exit 139 | librealsense 2.50 RSUSB pipeline을 같은 프로세스에서 재사용 | 정리 후 exit 1, Compose `on-failure:5`가 새 프로세스로 supervised restart |

상태에는 SDK callback rate와 unique first/last/count/gap, ROS 토픽별 rate, SRT
submit/sent/drop rate, aligned-Depth age, process CPU/RSS가 포함된다.

## 3. 자동 검증

- host Dashboard suite: **255 passed**, 약 9.96초, `git diff --check` clean.
- exact runtime image: 253 passed, 1 skipped, 1 deselected. Deselect 한 항목은 runtime image에
  의도적으로 없는 Docker CLI를 호출하는 Compose render 시험이며 host 전체 suite에서 통과했다.
- exact clean isolated ROS build: 3 packages built, **91 tests / error 0 / failure 0 / skip 0**.
- exact image 내용: pyrealsense2 2.50.0, x264/videoconvert 1.20.1, SRT 1.20.3,
  Jetson Orin Nano Engineering Reference Developer Kit Super.

## 4. 연결 기능 HIL

- USB `8086:0b64`는 5000 Mbps, SDK는 canonical serial `f0271544`만 열었다.
- duplicate container는 owner color count가 467→526으로 계속 증가하는 동안
  `ResourceGuard`에서 exit 1했다. 카메라 접근 전 singleton 거부다.
- RGB→Depth→overlay→RGB 전환에서 Gateway PID 114634, GStreamer PID 116514가 유지됐다.
  best-effort 누적 SRT는 Depth 28.43 Hz, overlay 26.82 Hz였다.
- Dashboard PID 1470에 SIGHUP을 보내자 Dashboard만 종료되고 Gateway/GStreamer PID는 유지됐다.
- GStreamer SIGKILL 뒤 Gateway는 `DEGRADED`, `BrokenPipeError`, streaming off가 됐고 ROS는
  유지됐다. 명시적 streaming restart는 Gateway PID를 유지한 채 GStreamer
  116514→117387로 교체했다.
- `restart_gateway`는 container restart count 0→1, Gateway PID 118570→120753으로 바뀐 뒤
  exact serial RUNNING으로 복귀했다.
- cleanup rehearsal은 `stop_gateway` exit 0, abstract listener/SDK/GStreamer owner 0,
  persistent lock file 유지, `flock -n` 성공을 확인했다. restart policy는 clean exit를
  재시작하지 않았고 manual start는 RUNNING으로 복귀했다.

## 5. RGB 60초 성능 인수

60.009초 동안 13개 경계, 완전한 5초 창 12개를 기록했다.

| 항목 | 전체 delta | 모든 완전 5초 창 | 판정 |
|---|---:|---:|---|
| ROS color Image | 1801 | 150, 한 창만 151 = 30.0~30.2 Hz | PASS |
| ROS color CameraInfo | 1801 | color와 동일 | PASS |
| raw Depth Image | 600 | 전 창 50 = 10.0 Hz | PASS |
| raw Depth CameraInfo | 600 | depth와 동일 | PASS |
| SRT sent | 1803 | 150 또는 151 | PASS |
| SRT dropped | 0 | 전 창 0 | PASS |
| SDK color/depth unique | 1801 / 1800 | 모든 stream gap_count 0 | PASS |
| ROS accel/gyro | 약 88.2~89.6 Hz | 전 창 ≤100.5 Hz | PASS |

CPU는 100.3~110.3%, RSS는 초기 214 MB 뒤 206~212 MB로 안정했고 backlog나 단조 증가가
없었다. ffmpeg receiver는 초기 GOP 중간 합류로 PPS 경고 뒤 다음 keyframe에서 정상화됐고,
초기 buffered catch-up 이후 274 frame/9.16초 = **29.91 fps**를 수신했다.

## 6. 분리·재연결

사용자가 L515만 같은 포트에서 분리하고 5초 뒤 재연결했다. Docker restart 없이 같은
Gateway process가 복구했고 USB devnum은 006→007로 바뀌었다. 복구 상태는 exact
`f0271544`, RUNNING, ROS color 30.007 Hz, raw Depth 9.998 Hz였다.

새 capture session의 frame 통계는 color `first=62, count=608, gap=0`, depth
`first=0, count=607, gap=0`으로 초기화됐다. 분리 전 session은 color
`first=34, count=4770`이었다. 따라서 이전 frame replay와 D435 fallback은 없었다.
단, 사용자의 물리 동작 중 상태를 실시간 보존하지 못해 `DEGRADED` 순간값 자체는 로그로
남지 않았다. callback freshness 설계와 session reset/새 USB devnum/자동 복구는 관찰됐다.

## 7. D435IF 동시부하

기존 dirty robot-arm checkout을 수정하지 않고 기존 `ros2_humble`과
`run_perception.sh`를 사용했다. perception이 실행 중인 20.006초 observer 결과:

- `/detected_objects` 330, L515 color 541, raw Depth 167.
- L515 native color +607, gap 0; SRT +607, drop 0.
- L515 상태 RUNNING, serial `f0271544`; D435IF는 별도 USB port의 perception 입력이었다.
- kernel USB error/reset/disconnect/timeout delta 0.
- 동시 순간 자원: L515 container CPU 205.43%, 168.5 MiB; robot-arm container
  CPU 101.01%, 1.227 GiB.

## 8. 최종 정리와 외부 상태 복원

HIL Gateway는 exit 0으로 종료·삭제했다. abstract listener, SDK/GStreamer/test observer는
0이고 lock file은 남았으며 flock은 free였다. 시작 전 상태를 다음처럼 복원했다.

- `powertrain_ros` (`powertrain-sw:ros`) Up.
- `powertrain_jetson` Up.
- `powertrain_canwatchdog` Exited (137).
- `ros2_humble` Exited (137).
- Jetson powertrain checkout: `ec452f6474b6fc57437d576298f2bc954649be42`, 기존
  `motor_control/vision/tests/` untracked 그대로.
- robot-arm checkout: `279d691f773355b44d3f03b6deaccdc7c5c0d0d9`, 기존 modified/untracked 목록 그대로.

고유 snapshot/image는 감사 증거로 남겼고 production tag/container를 retag하거나 교체하지 않았다.

## 9. 남은 항목

- Depth/overlay SRT는 alignment 비용 때문에 best effort이며 RGB 29 Hz 인수 대상이 아니다.
- active Software Notion 페이지의 fetch-before/write/re-fetch는 현재 세션에 Notion connector가
  설치되지 않아 수행하지 못했다. 로컬 문서가 현재 정본이며 connector 설치 후 동기화해야 한다.
