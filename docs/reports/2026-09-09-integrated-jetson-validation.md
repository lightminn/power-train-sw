# 통합 운용 Jetson 배포·루프 검증 — 2026-09-09

최신 작업 소스를 Jetson에 배포했고 실제 노트북 콘솔 연결과 **가상 모터를 사용하는
설치된 ROS 전체 제어 루프**를 검증했다. **현재 인수 범위는 원격주행**이다.
사용자 지시로 미완성 자율주행 테스트는 이번 판정에서 제외했다. 빠진 선을 재연결한 뒤
**실제 CAN 10축 통신을 확인했다.** 후속 DualSense 시험에서 운전 시작과 실제 구동
6축 폐루프 진입까지 확인했다. 사용자는 아직 구동 트리거를 누르지 않았고, 비영 구동
지령·실제 전진·제동 인수는 수행하지 않았다. 이후 node 11·12가 무응답으로 바뀌어
현재 전원/배선 확인을 기다린다. 아래 앞부분은 각 단계 당시의 기록이다.
**US-100과 L515는 의도적 분리 조건으로 이번 CAN 시험에서 제외했다.** 후속 관측에서
L515 재연결·영상 LIVE는 확인했지만 센서별 정식 인수를 진행한 것은 아니다.
안전 게이트는 유지했다.

## 배포와 기존 작업 보존

- 노트북 소스: `main` HEAD `c3fd0786523542b945b38fa7347c16e0bffd1a78`의 미커밋 작업을
  포함한 파일 스냅샷. Git fetch 및 Draft PR #4 상태를 확인했으며 merge/commit/push는 하지 않았다.
- Jetson 기존 `~/power-train-sw`는 팀원 `feat/l515-aligned-depth-slam`, HEAD `310163a`다.
  기존 tracked/untracked 파일 **847개 SHA256이 작업 전후 동일**하다.
- 새 배포 위치: `/home/zetin/power-train-sw-integrated`. 기존 체크아웃을 덮어쓰지 않았다.
  `DEPLOYED_SOURCE.json`으로 노트북 파일과 배포 파일의 SHA256을 대조한다.
- 백업: Jetson `~/powertrain-deploy-backups/2026-09-09-integrated/`.
  기존 patch/status/파일 해시/컨테이너 정보와 root 전용 시스템 설정 사본을 보존했다.
  이전 이미지 태그는 `powertrain-sw:pre-integrated-20260909-ros`,
  `powertrain-sw:pre-integrated-20260909-jetson`이다. 토큰 값은 기록하지 않는다.
- 현재 Jetson은 외부 네트워크/레지스트리 접속이 안 돼 이미지 빌드가 실패했다.
  두 런타임 Dockerfile이 기존 소스와 동일하고 기존 ROS 이미지의 ROS/CAN/serial/NumPy/
  OpenCV/RealSense/Textual import가 가능함을 확인한 뒤 `--use-existing-images`로 배포했다.
  **이미지를 새로 빌드한 결과가 아니다.** ROS 세 패키지는 Jetson에서 colcon 재빌드했다.
- `STOP_MM=200`, `STOP_MM_PROVENANCE=BENCH`와 기존 역할 토큰을 보존했다.
  기존 호스트 텔레메트리 두 서비스는 명시적 전환 옵션으로 비활성화하고 Compose가 소유한다.
- Jetson 시스템 시계는 8/18, NTP 미동기화 상태였다. 이 보고서 날짜는 노트북의 작업일이며
  원격 로그 시각은 그대로 보존한다. 세션/입력 검증은 monotonic 시간으로 수행했다.

실제 최초 배포 명령:

```bash
# Jetson 호스트, 이미 준비된 이미지 재사용을 검증한 이 배포에 한정
cd ~/power-train-sw-integrated
sudo bash scripts/robot_prepare.sh \
  --robot-id zetin-rover \
  --token-file /etc/powertrain/ops_console.token \
  --replace-legacy-telemetry \
  --use-existing-images
./scripts/robot-start
```

두 명령 PASS. colcon 세 패키지 빌드 19.1초. `robot-start`를 재실행해 같은 8개 서비스가
재빌드 없이 유지되는 것도 확인했다. 부팅 preflight 유닛은 enabled/active이고 새 경로
drop-in을 설치했다. **전원 재인가와 새 부팅 경로는 실행하지 않았다.**
현재 daily 명령은 `~/power-train-sw-integrated/scripts/robot-start`다.
노트북에는 `~/.local/bin/powertrain-integrated-console`과 앱 메뉴 실행 항목을 설치했다.

## 실제 연결 및 ROS 전체 루프

| 경로 | 실행 결과 |
|---|---|
| 실제 Jetson 세션 | `zetin-rover` 인증 및 최신 ops 상태 수신 |
| 실제 GTK 콘솔 | 설치된 설정으로 22초 실행, 전원/차체 UDP **LIVE**, SIGINT 정상 종료, traceback 없음 |
| 실제 차체 상태 | authority IDLE, chassis ESTOP, 원인 `us100 / liveness_timeout` 유지 |
| 선 재연결 후 실제 CAN | ERROR-ACTIVE, TX/RX 오류 0, 구동 6축 조회 36/36건 응답 + 조향 4축 상태 수신 |
| 입력/ops 포트 | 공개 :9000/:9001/:9002는 인증 세션, 원래 노드는 127.0.0.1:19000/19001만 LISTEN |
| 가상 모터 전체 루프 | 아래 네 단계 **PASS**, 실제 노트북 UDP 33패킷 수신 |
| 중립 입력 음성 대조 | 시작부터 데드맨/트리거를 유지하자 준비 조건 실패, IDLE/속도 0 유지 — 예상대로 FAIL |

가상 모터 루프는 Jetson `powertrain-sw:ros`에서 설치된 `control.launch.py`, `chassis`,
`chassis_telemetry`와 실제 `powertrain_runtime.robot_service`를 실행했다. ROS domain 77,
공개 29000/29001/29002, 내부 39000/39001, UDP 15005로 생산 제어와 분리했다.
컨테이너는 UID 1000, cap-drop ALL, no-new-privileges, 소스 읽기 전용이며 실물 장치를
마운트하지 않았다. SafetyVerdict와 ArmStatus만 합성 fixture로 발행했다.

1. 명시적 홀드 없는 Start 거부, 1.5초 이상 홀드 뒤 실제 ops 명령으로 수동 권한/arm 성공.
2. 노트북의 실제 직렬화 패드 입력 → Jetson TCP 프록시 → ROS → 6축 FakeDrive 구동.
3. 실제 WheelStates → Jetson UDP 송신 → 노트북에서 0이 아닌 바퀴 피드백 확인.
4. 구동 입력 도중 입력 채널 종료 → 주행 해제 및 속도 0 → 재연결 후 IDLE 유지,
   자동 arm/이전 입력 재생 없음.

fixture와 자식 프로세스는 검증 후 종료했다. 이 시험은 ROS/소켓 제어 경로의 증거이며
실물 패드, CAN 모터, 제동거리, 로봇팔 팀 handshake의 증거는 아니다.

## 테스트 수치

서로 겹치는 스위트는 합산하지 않는다. 다음 표는 사용자와 확정한 **원격주행 범위**다.
세션·수동 입력·차체 제어·공통 안전/명령권·콘솔·영상/텔레메트리를 포함한다.
미완성 자율 제어기·접근/추종·구간/미션·지형 자격·자율 시나리오 매니페스트는 제외한다.
공통 안전/명령권 테스트의 모드 간 우선순위 단언은 유지한다.

| 실행 환경·스위트 | 결과 |
|---|---|
| 노트북 `motor_control` | **848 passed** |
| 노트북 `operator_console/tests` 실제 GTK/Xvfb | **300 passed** |
| 노트북 controller/console parent/launcher | **41 passed** |
| 노트북 원격 관측·세션·영상·GUI·배포·통합 계약 | **746 passed** |
| Jetson 원격주행 runtime + ROS | **631 passed**; 설치 공간 provenance 별도 **2 passed** |
| `operator_console.runtime_smoke` | **PASS** — 4채널 LIVE→STALE, 31 ticks, traceback 없음 |
| `operator_console.integrated_runtime_smoke` | **PASS** — 로봇/패드 대기, 중복 거부, 자식/잠금 정리 |
| 독립 변경분 리뷰 및 집중 회귀 | **97 passed**, 차단 결함 없음, `bash -n`/`git diff --check` PASS |

Jetson 테스트는 설치된 ROS 환경을 먼저 source하고 기존 ROS PYTHONPATH를 유지했다.
호스트 전용 pygame/GTK 설치기 검사는 해당 노트북 인터프리터에서 실행했다.
소스 AST 테스트가 sys.path를 수정하므로 설치 공간 provenance 테스트는 새 Python
프로세스로 분리했다. 이런 환경/수집 오류는 생산 결함으로 세지 않는다.
원격주행 재실행의 정확한 선택 목록은 증거 폴더의 `jetson-remote-tests.sh`에 있다.
호스트는 `powertrain_autonomy`와 WP8 handshake, 자율 환경 매니페스트 두 파일을
호출하지 않고 원격 관측/배포/콘솔 계약을 실행했다.

## 이번에 수정한 항목

- 오프라인 준비: 기본 이미지 빌드 실패는 여전히 오류다. 명시적
  `--use-existing-images`에서 두 이미지가 없으면 시스템 서비스 변경 전에 실패한다.
  두 신규 테스트의 수정 전 실패와 수정 후 런처 전체 12건 통과를 확인했다.
- FakeDrive 정지 피드백: CornerModule IDLE tick은 수신만 처리하므로 FakeDrive의
  미호출 tick에 있던 속도 0 처리가 실행되지 않았다. 가상 모델 disarm 시 목표/피드백을
  즉시 0으로 만든다. 수정 전 회귀는 `0.96875 != 0`, 수정 후 단위·실제 ROS 루프 PASS.
  실제 모터 드라이버·정지 판정·안전 문턱은 변경하지 않았다.
- ROS AST 하니스: 생산 코드가 사용하는 `steering_state_fields`, listener `_host`를
  테스트 하니스에 반영했다. 기존 단언과 생산 동작은 유지했다.
- 자율주행 합성 fixture: 오래된 80×60 입력을 현 자격 ROI 160×120에 맞췄다.
  초점거리를 함께 조정해 장면/FOV를 보존하고 해상도 변경 거부 시험도 유지했다.

## CAN 복구 및 실물 검증 범위

**최종 CAN 재시험 — 선 재연결 후 통과:** 사용자가 빠진 선을 확인하고 재연결한 뒤
can0가 ERROR-ACTIVE, 500 kbps, LOOPBACK off, TX/RX 오류 카운터 0인 것을 확인했다.
이번에는 인터페이스 재기동 없이 chassis만 잠시 멈추고 단독 소유권으로 같은 읽기 전용
조회를 실행했다. 6.002초 동안 3,037프레임을 수신했다.

- ODrive node 11~16: encoder estimates/Iq를 각각 3회 요청해 **36/36건 응답**.
  모든 축의 axis_error=0, axis_state=1(IDLE), 속도=0, 측정 Iq=0.
- AK id 1~4: 각각 약 50 Hz 상태 수신, 모두 fault=0. 위치는 각각
  -2.4°/1.0°/3.1°/2.0°였다. 실제 조향 움직임을 명령한 시험은 아니다.
- 시험 전후 ERROR-ACTIVE와 TX/RX 오류 0을 유지했다. 실제 TX 완료도 증가했다.
- 조회 후 chassis를 다시 기동했고 watchdog은 시험 중에도 유지했다.
  증거는 `can-reconnected-query.log`, 구조화 결과는 `can-reconnected-summary.json`이다.
- 이후 실제 GTK 콘솔을 22초 다시 실행해 인증 연결, 전원/차체 UDP LIVE, 정상 종료를
  확인했다. 이번에는 L515 영상도 LIVE였고 USB 장치 목록에 Intel RealSense 515,
  Gateway healthy를 확인했다. US-100 timeout ESTOP·패드 대기·운전 준비 false는 유지됐다.
  로그는 `live-console-reconnected.log`다. ESTOP 해제나 실제 주행은 수행하지 않았다.

아래 무응답 기록은 **선 재연결 전 기록**이며 현재 CAN 상태를 뜻하지 않는다.

**CAN:** 사용자 요청에 따라 watchdog만 잠시 멈추고 can0 down → 500 kbps,
loopback off, restart-ms 100, txqueuelen 1000 → up을 실행한 뒤 watchdog을 되살렸다.
재기동 후 `ERROR-PASSIVE`, tx error counter 128이며 수동 수신 5초 동안 프레임 0개다.
모터 전원/배선/종단 저항/상대 노드 상태는 원격 로그로 확정할 수 없다.
US-100 timeout ESTOP도 유지된다. 후속 사용자 확인으로 US-100 역시 의도적으로
분리한 시험 조건임을 확인했다. 모터 arm·캘리브레이션·NVM 쓰기를 수행하지 않았다.

**후속 CAN 재조회:** 누적 RX 170,359건이 보였으나 현재 수신 창 5초에서는 0건이었다.
watchdog을 잠시 중단하고 인터페이스를 재기동한 뒤에도 5초 수신 0건이었다.
이어서 chassis와 watchdog을 중단하고 `RealCanSession` 단독 소유권을 획득한 뒤,
ODrive node 11~16의 encoder estimates(0x09)/Iq(0x14) RTR만 3회씩 조회했다.
6.013초 동안 요청 36건은 소켓 송신 큐에 들어갔으나 실제 TX 완료 카운터는 475에서
늘지 않았고 수신은 0건이었다. AK status도 관측되지 않았다. can0는 UP/500 kbps,
LOOPBACK off지만 ERROR-PASSIVE, tx error counter 128을 유지했다.
따라서 과거 누적 수신량이나 socket send 성공을 현재 통신 복구로 간주하지 않는다.
조회 후 chassis(healthy)와 watchdog을 다시 기동했다. 조회 도구는 RTR만 사용했고
새 구동·조향 목표, arm, ESTOP 해제는 요청하지 않았다. 서비스 종료 시 기존 disarm/정지
처리는 그대로 실행됐다. 로그는 `can-recheck-passive.json`, `can-recheck-cycle.log`,
`can-controlled-recheck.log`, 재현 코드는 `can-readonly-probe.py`다.

**선 재연결 전 L515/패드 기록:** L515 의도적 분리 당시 Gateway health는 unhealthy였다. 나머지 control/chassis/
session/observability는 healthy이며 8개 컨테이너의 restart 0, 최근 로그 traceback 0을
확인했다. D435i는 연결되어 있지만 파워트레인이 대신 점유하지 않았다. 실제 패드는
미연결이며 원격 운전 준비 표시는 false다. L515 분리는 장애로 오인하지 않지만 영상
준비 게이트를 해제한 것도 아니다.

## 범위 확정 전 자율주행 진단 — 현재 원격주행 판정에서 제외

아래는 원격주행으로 범위를 확정하기 전에 실행한 참고 기록이다. 현재 인수 실패 수나
원격주행 완료 조건에 포함하지 않는다. 자율 SW/시험 파일을 삭제하거나 판정 자체를
성공으로 바꾼 것은 아니다.

- 범위 확정 전 Jetson runtime+전체 ROS: **761 passed, 5 failed**; 설치 공간 2 passed.
- 범위 확정 전 호스트 autonomy 포함 광역 검사: **963 passed, 2 failed**.
- 수정된 autonomy fixture의 AMD64 ROS 검사: **16 passed**; Jetson 결과와 별개다.

**Jetson 자율주행 5건:** 양의 속도 발행이 필요한 테스트에서 안전 신선도를 충족하지 못한다.
실제 합성 ROS 진단 8초에서 지형 path=true, confidence≈0.677, reject=[]이며 센서/gate
age는 정상이다. 하지만 estimator.update 중앙값≈0.25–0.26초, p95≈0.29–0.31초로,
이전 결과의 age가 0.45초 문턱을 반복 초과한다. `terrain_stale`과 `recovery_dwell`이
반복되어 속도 0을 유지한다. OpenBLAS/OMP 1thread 제한으로 해결되지 않았고 합성
렌더는 중앙값≈6ms여서 주요 원인이 아니다. timeout/freshness/recovery 기준을 늘리거나
테스트 구현을 skip 처리하지 않았다. 이 진단은 향후 자율주행 트랙의 참고 자료로 남긴다.
autonomy 배포는 이 통합 운용 프로필에서 계속 비활성이다.

3회 프로파일에서 estimator.update 누적 0.653초 중 `_quality_and_mask`가 0.358초,
`depth_quality._components`가 0.289초다(중첩 시간이므로 합산하지 않음). 연결성/마스크
계산이 큰 비용이다. ROS 테스트의 Image.data 바이트별 검증 비용을 없애는 임시
`array('B')` 후보도 전체 직렬화 바이트는 동일했으나 5건 중 2건이 계속 실패했다.
이 후보는 채택하지 않았다. 기존 커널 단독 자격 보고서의 7ms를 전체 경로 시간으로
해석할 수 없다.

**자율 시나리오의 기존 호스트 실패 2건:** `tests/test_environment_manifest.py`와
`tests/test_fixture_class_contracts.py`의 `procedural-dev-0/analytic` 체크섬 불일치다.
8/24 기준선에 있던 기존 실패가 재현됐으며 폐기 트랙의 수치/체크섬을 수정하지 않았다.

실행 로그와 일회성 재현 fixture는 노트북 `/tmp/powertrain-jetson-20260909/`,
Jetson의 위 배포 백업 폴더에 남겼다. 주요 파일은 `prepare-offline.log`,
`jetson-tests-final.log`, `loop-positive-final.log`, `loop-negative-final.log`,
`live-console-result-final.log`, `audit-final.log`, `autonomy-diagnostic.log`,
`autonomy-blas-diagnostic.log`, `autonomy-profile.log`, `autonomy-array-candidate.log`다.
현재 범위의 최종 결과는 `jetson-remote-tests.log`, `host-remote-tests.log`다.
로그의 시각·가상/실물 범위를 구분해 해석한다.

## 실물 패드 후속 시험과 수정

실제 DualSense USB 연결 및 중립 입력을 확인했다. 실차 연결에서 다음 문제를
재현하고 회귀시험 실패를 먼저 확인한 뒤 수정했다. 자율주행은 범위에 추가하지 않았다.

- ESTOP 중 패드 재연결이 반복 disarm을 발생시켜 ops 설정이 busy로 거부됐다.
  신선한 ESTOP·정지·비활성 권한이 확인된 경우에만 중복 stop을 생략한다. 입력은
  계속 거부하며, stale/이동/ARMED/PENDING으로 바뀌면 같은 세션에서도 다시 정지한다.
  실제 GUI에서 US-100 OFF와 별도 ESTOP 초기화가 성공했다. 신규 TCP 시험 11건 RED→GREEN.
- CAN 구동 6축 arm이 축마다 100 ms 대기하여 0.3초 입력 감시를 초과했다.
  호스트 실제 드라이버+시간 모사 버스로 0.60065초 및 HOLD를 독립 재현했다.
  구동은 기존 유한 비차단 수신, 조향은 poll(0)으로 바꿨다. 수신 없는 상태는 stale로
  유지한다. 실제 재시도에서 수동 권한+ARMED 성공과 CAN 6축 axis_state=8/axis_error=0을
  확인했다. 신규 지연·오류 시험은 수정 전 3 failed/1 passed였다.
- stop은 disarm ACK를 요구하고 authority_idle을 시도한다. 이미 걸린 HOLD 때문에
  authority_idle이 거부된 경우 신선한 비활성 차체+6축 정지가 확인되면 정지를 인정한다.
  HOLD/ESTOP는 자동으로 해제하지 않는다. disarm 실패·응답 불명·이동·stale는 성공으로
  바꾸지 않는다. 관련 8개 시험은 수정 전 3 failed/5 passed였다.
- IDLE에서는 encoder 조회가 멈춰 마지막 비영 속도가 남았다. IDLE/FAULT 및 전역
  ESTOP에서도 encoder/Iq RTR만 조회하며, state()는 수신 전용이다. 새 응답 없이 속도를
  0으로 덮지 않는다. encoder 신선도를 WheelStates.drive_stale로 전달하고 broker가
  6개 고유 바퀴의 유한 속도·구동 오류·신선도를 함께 확인한다. 전체 메시지 healthy나
  조향 상태로 구동 속도의 신선도를 대신하지 않는다.
- AK disarm도 4축 합계 약 1초를 막아 입력 재연결과 충돌했다. 기존 비상정지와 같은
  즉시 0 RPM 경로를 사용한다. 반복 정지는 제어 루프가 끝난 close()에 유지했다.
  disarm 회귀의 수정 전 실패를 확인했다.

후속 검증: 호스트 motor_control + ops 상태 소스 시험 **880 passed**, 세션·패드·런처
**101 passed**, GTK 통합 창/부모 시험 **14 passed**. 스위트는 서로 겹치므로 합산하지
않는다. 실물 패드가 연결된 호스트의 무패드 자식 프로세스 시험은 기존 콘솔 잠금과
실물 장치 때문에 처음 실패했고, 장치 없는 bwrap 환경에서 원래 단언 그대로 통과했다.
코드 검토는 비차단 처리·정지 ACK·속도 신선도·ESTOP 관측 경계를 각각 확인했다.

ROS 변경은 소스 복사만으로 설치 공간에 반영되지 않아 colcon 3패키지를 다시 실행했다
(4.13초). Jetson 시계 때문에 clock-skew 경고가 있었으며, 설치된 broker Python에서
6축 신선도 검사 코드가 실제 로드되는 것을 별도로 단언했다.

실물 6축 활성화 후 0이 아닌 구동 지령은 없었고, 사용자는 아직 L1+R2를 누르지
않았다고 확인했다. 이후 2초 CAN 직접 수신에서 node 13~16은 heartbeat와 encoder/Iq
응답 각각 100건, node 11·12는 조회 프레임만 있고 heartbeat/응답 0건이었다.
can0 자체는 ERROR-ACTIVE, TX/RX 오류 0이다. 이 상태를 모터 시험 통과로 간주하지
않으며 두 축을 담당하는 보드의 전원/배선 확인을 요청했다.

증거: `estop-retry-red.log`, `estop-retry-green-isolated.log`,
`can-arm-latency-red.log`, `stop-hold-red.log`, `stop-hold-green.log`,
`idle-feedback-red-core.log`, `live-drive-host-final.log`,
`live-drive-can-fixed.jsonl`, `live-drive-state-fixed.jsonl`,
`live-drive-colcon.log`. 로컬 및 Jetson의 기존 증거 디렉터리에 보존한다.

최종 Jetson 집중 시험은 **177 passed**(코너 드라이버·세션·시작/정지 서비스)와
별도 **23 passed**(ops 상태 소스)다. 한 프로세스에 합친 첫 실행은 로그 캡처
시험 1건 실패/199건 통과였고, 해당 시험 단독 및 위 두 프로세스 분리 실행에서
원래 단언 그대로 통과했다. 로컬 소스 834개와 새 Jetson 배포 파일의 SHA256을
대조했다. 실물 시험 중 서비스 재연결 뒤 SRT가 CONNECTING에 머무는 사례도 있었고
콘솔 창 재실행으로 LIVE가 복구됐다. 이 재연결 사례의 코드 수정은 이번에 하지 않았다.
콘솔은 켜둔 상태이며, 마지막 3초 CAN 재조회도 node 11·12 무응답, 나머지 4축 속도
0이었다. 이 결과는 실물 전진/제동 통과가 아니다.
