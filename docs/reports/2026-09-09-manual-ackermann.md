# 2026-09-09 수동 애커만 조향·URDF 적용 검증

운전자는 이전 부호 교정 후 **직선 전진/후진 정상**을 확인했다. 남은 마름모 조향은
정지 상태의 스틱을 yaw rate로 해석한 기존 피벗 경로였다. 이를 자동차식 독립 조향으로
개편하고 실제 제작 URDF 치수를 적용했다. **격리 소프트웨어 검증과 운영 배포 확인을 아래에 구분한다.
새 조향의 실차 운전자 인수는 별도이며, 자율주행은 이번 인수 범위가 아니다.**

## 원인과 변경

기존 실차 CAN 목표를 중륜 속도로 역산한 156개 표본은 모두 차체 v=0이었다.
요레이트 부호는 양/음 모두 존재하지만 AK raw 목표는 항상 [+45,-45,-45,+45]였다.
따라서 좌우 스틱의 부호가 소실된 것이 아니라 피벗의 같은 조향 배치였다.
[원인 분석 표본](2026-09-09-manual-ackermann-evidence/diamond-analysis.json)에 기록했다.

기본 수동 경로는 `ManualDriveCommand(speed_mps, steering)`를 전달한다.
L1 데드맨과 RT/LT 속도, −L스틱X 조향을 게이트하고 authority·DDS 수신시각·차체
워치독을 통과한다. 정지에서는 구동 0으로 조향만 하며 전후진에도 같은 조향각을
유지한다. 비중립 인계·잘못된 수치·지연 명령·ESTOP는 기존 안전 경계를 유지한다.
명시적 `manual_command_format=twist`만 레거시 요레이트 경로를 사용한다.
보조주행 `assist_enabled=true`는 현재 이 레거시에서만 허용한다.

## URDF 수치

정본은 `power-train-sim/rover_arm_integrated/urdf/2026_07_24_URDF.urdf`다.
전체 joint chain 회전·이동을 합성하고 CAD (X,Y,Z)를 REP103 (Y,-X,Z)로 바꿨다.
URDF 및 타이어 메시 6개의 해시가 해당 저장소의 SHA256SUMS와 일치했다.
[추출 근거](2026-09-09-ackermann-urdf-evidence.json)에 원본·대칭 좌표와 한계를 남겼다.

| 수치 | 적용값 |
|---|---:|
| 축거 | 875.494655 mm |
| 앞/중간/뒤 윤거 | 545 / 719 / 425 mm |
| 앞뒤축 중점 기준 중간축 x | −60.3355675 mm |
| 중간축 기준 앞/뒤 거리 | +498.082895 / −377.411760 mm |
| 무하중 타이어 반경 | 103.56 mm 유지 |
| 운용 조향 한계 | ±45° 유지 |

기존 치수와 0.1 mm 미만 차이여서 치수 반올림이 마름모 현상의 원인은 아니다.
다만 중간 고정축이 앞뒤축 중점과 일치하지 않으므로 새 수동 해법은 **중간축 연장선에
회전 중심을 둔다**. 중간륜의 횡방향 속도가 0이 되고 앞/뒤 4륜도 같은 회전 중심을
공유한다. 차체 원점과 오도메트리 좌표계는 유지한다. 비영점 body vy=-ω×x_mid는
이 배치의 정상 운동이며 오도메트리 왕복 테스트가 이를 확인한다.
URDF의 continuous 조향축에는 기계 한계가 없으므로 허용각을 확대하지 않았다.

## 실행 증거

증거 폴더: [manual-ackermann-evidence](2026-09-09-manual-ackermann-evidence/).

| 검증 | 실제 결과 |
|---|---|
| 신규 pure manual 계약 | RED 19 failed → 구현 후 통과 |
| URDF 추출기 | RED 10 failed → 10 passed; 실제 URDF+STL CLI 성공 |
| 기하/차체/코너+추출기 | 723 passed |
| motor_control 전체+gateway/typed adapter/wheel-stop/URDF | 1082 passed |
| 위 항목과 추가 배선·launch를 합친 최종 회귀 | **1126 passed** |
| ROS 어댑터·receipt·기존 assist/section 배선·launch 계약 | 116 passed (앞 행과 중복 포함) |
| Jetson 실제 ROS 타입·DDS·기동/종료 | 7 passed |
| 독립 수학/오도메트리 sweep | 5,427 케이스, 최대 잔차 2.40e−16 m/s |
| 설치 ROS + 가상 CAN 전체 루프 | RT/LT/STOP × 좌우 6케이스 PASS |
| 정지 조향 | raw CAN 구동 6축 모두 정확히 0, 앞 두 축 같은 방향/뒤 두 축 반대 |
| 전후진/정지 각도 일치 | 같은 스틱의 raw CAN 각도 차이 ≤0.001° |
| 공통 회전 중심 | 수신 raw CAN 각도에서 계산한 4개 교점 차이 <2 mm |
| 연결 단절/재접속 | 주행 해제·0속도·자동 재무장 없음 |
| 비중립 운전 시작 음성 대조 | 길게 눌러도 시작 거부, IDLE 유지 |
| 검증 함수 음성 대조 | 정지 비영속도/뒤집힌 조향/불일치 ICR 세 변이 모두 검출 |

전체 루프는 Jetson의 `powertrain-sw:ros`에서 **network none, domain77, vcan77**로
실행했다. `/sys/class/net`이 lo/vcan77뿐이며 can0가 없음을 단언한다. CAN 드라이버는
실제 구현이고 모터 피드백은 프로토콜 에뮬레이터다. 설치된 control/chassis/telemetry
엔트리포인트, 인증 세션·길게 누르기 운전 시작, 원격 입력 인코더, UDP 수신까지 사용한다.
생산 can0·USB·NVM 조작이나 실물 바퀴 회전 검증을 뜻하지 않는다.

Jetson 시계가 1970년이라 기존 설치물보다 소스 수정시각이 미래인 상태였다. 새 메시지를
빈 build/install 공간에 빌드하고, 수정한 Python 패키지는 이전 build/install을 보존 이동한
뒤 다시 빌드했다. 초기 재빌드 명령의 local_setup 경로 오류는 `share/<package>/local_setup.bash`로
바로잡았다. 이후 새 설치물의 실제 ROS 및 전체 루프가 통과했다.
정밀 좌표 반영에 따른 기존 피벗 오도메트리 회귀값은 0.960907 → 0.9608944 rad로
갱신했으며 허용오차/5% 물리 계약은 완화하지 않았다.

## 재현과 실차 확인

순수 테스트는 저장소 루트에서 `PYTHONPATH=$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros`
설정 후 pytest로 실행한다. URDF 추출은
`python tools/extract_ackermann_geometry.py <2026_07_24_URDF.urdf>`로 JSON을 얻는다.
격리 루프의 `loop_fixture.py`, `loop_client.py`, `run_loop.py` 및 결과 JSON을 보존했다.
ROS 메시지 패키지와 노드를 함께 재빌드하고 설치물을 검증한 뒤 배포한다.

운전자는 L1+L스틱으로 정지 좌/우 조향을 먼저 확인하고 RT/LT에서도 같은 각도가
유지되는지 확인한다. 실측 장착 부호와 영점/NVM은 이번 변경에서 다시 바꾸지 않았다.
지상 영점, 하중 상태 마찰·조향 도달오차, 최종 제동은 실차 인수 대상이다.

## 운영 배포 후 확인

구현 커밋 `b18b770`을 로컬 main·GitHub·Jetson 정본 체크아웃에 동기화했다.
팀원 체크아웃의 미커밋/미추적 파일은 보존했다. Jetson DNS 제약으로 검증된 Git bundle을
전달해 main을 fast-forward했다. control/session/chassis를 정지하고 기존 두 패키지의
build/install을 보존한 뒤 `powertrain_msgs`와 `powertrain_ros`를 재빌드했다.

- 설치 Python 50개와 변경 launch 2개가 소스와 바이트 단위 일치, 생성 메시지 타입 확인.
- `robot-start` PASS, control/chassis/session 모두 healthy, 새 기동 로그에 traceback 없음.
- 실제 can0는 500 kbps ERROR-ACTIVE, LOOPBACK/LISTEN-ONLY off.
- 실제 ROS 바퀴 피드백 6행에서 구동 6축·조향 4축 stale=false, error/fault=0,
  모든 구동 명령/피드백=0 확인. 실차에 운전 시작이나 시험 주행 명령을 보내지 않았다.
- 운영 콘솔 실행·Jetson 연결·telemetry/chassis LIVE 확인. 재시작은 구성요소를 모두
  ON으로 초기화하므로, 의도적으로 분리한 US-100의 liveness_timeout으로 ESTOP다.
  운전자가 기존 미장착 설정(US-100/로봇팔 OFF)과 비상정지 초기화를 확인한 뒤
  명시적으로 운전 시작해야 한다. 자동 초기화·재무장은 하지 않았다.

배포 증거는 같은 evidence 폴더의 `manual-deployed-equality.json`,
`manual-live-wheels.json`, `manual-robot-start.log`, `manual-deploy-build.log`에 있다.
