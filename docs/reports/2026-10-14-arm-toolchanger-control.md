# ID1 결합 잠금 제어 구현 결과

사용자 확인한 XL430 ID1, 1:1과 양방향22.5° 전환을 반영했다.
팔5축/그리퍼와 분리된 코어를 기존 MoveItDynamixelBridge의 단일 버스 소유권에 연결했다.
신규 모듈에서는 잠금 게이트가 기본ON이며 영점/결합 위치/토크 확인 전 도구 구동을 차단한다.
영점은 RAM 또는 명시한 host JSON 파일에 저장한다. 모터 EEPROM은 변경하지 않는다.

## 변경

- SET_ZERO/ENABLE/LOCK/UNLOCK/STOP 및 상태 관측, 실제 로터 각도 JointState 발행.
- 양방향256tick 전환, 단일회전 경계 래핑 거부, 도구 정지 후 분리.
- model/mode/drive mode/오류/위치/속도/토크/profile readback 검사.
- 목표 도달/속도0/토크ON 확인 전 MOVING 유지, 이동 중 timeout/통신실패 시 토크OFF 시도.
- stale feedback·비상정지·도구분리·미보정 상태에서 도구 게이트 차단.
- 잠금 변경은 MANUAL END_EFFECTOR_ONLY 벤치로 제한. FULL_ROBOT은 상태/게이트만 제공.
- 로터 URDF를 continuous로 변경해 사용자 확인한 양방향 반복 인덱싱을 표현.
- 실행 검사에서 드러난 기존 mock ID5 초기 effort=None 변환 예외를 NaN 발행으로 수정.

## 검증

- 컨테이너에서 robot_arm_msgs/dynamixel_control/robot_manual_gui 설치 빌드 통과.
- 최종 dynamixel_control/robot_arm_description 설치 빌드:2 packages finished.
- 잠금·단일/듀얼FSM·URDF 테스트:42 passed.
- ID1이 없는 기존 모듈을 명시한 GUI/hot-swap/FULL_ROBOT 회귀:15 passed.
- 영점 가드를 제거한 임시 복사본은 uncalibrated 검사1 failed. 원본에서는 통과.
- 설치된 ros2 run 엔트리포인트에 실제 ROS 메시지를 보내는 격리 mock E2E 통과:
  미보정 거부 → SET_ZERO/파일저장 → 현재 goal 동기화후 ENABLE → 기존 도구 재준비 →
  도구 토크허용 → UNLOCK(-1)/도구차단 → LOCK(+1) → 도구 재허용 → 비상정지.
- UNLOCK에서 2048→1792tick와 JointState -π/8rad 확인. LOCK에서2048tick 복귀 확인.
- 읽기 전용 콜드스타트에서 영점파일 복원, ENABLE 거부, 토크OFF·도구차단 확인.
- 실제 serial/장치 접근과 실기 구동은 수행하지 않았다.

재현(로컬 가상 검증 컨테이너, /tmp/arm_check에 설치 빌드가 있는 환경):

```bash
source /opt/ros/humble/setup.bash
source /tmp/arm_check/install/setup.bash
cd /tmp/arm_check
ROS_DOMAIN_ID=78 python3 src/dynamixel_control/scripts/verify_toolchanger_mock.py
```

기대 출력: installed ROS bridge PASS, read-only cold restart PASS.
시험은 installed mock 엔트리포인트만 실행하며 실제 serial을 열지 않는다.
컨테이너 image의 pip/system setuptools 충돌은 빌드 시 시스템 dist-packages를
PYTHONPATH 앞에 두어 해결했다. 이미지 설치나 의존성 변경은 하지 않았다.

## 이후 영점 작업

아래 메시지는 실제 하드웨어 mode/drive mode와 영점 위치를 확인한 뒤 사용하는 인터페이스다.
브리지는 `toolchanger_zero_file`을 쓰기 가능한 JSON 경로로 지정해 기동하고
MANUAL END_EFFECTOR_ONLY로 운용한다. read_only에서는 SET_ZERO/ENABLE도 거부한다.
토크OFF·정지 상태에서 실제 결합 여부를 확인하고 영점의 state를 선택한다.

```text
/toolchanger/command (std_msgs/String JSON)
{"command":"SET_ZERO","state":"LOCKED"}
{"command":"ENABLE"}
{"command":"UNLOCK","direction":-1}
{"command":"LOCK","direction":1}
{"command":"STOP"}
```

영점이 분리 상태라면 state=UNLOCKED를 사용한다.
LOCK/UNLOCK 요청은 이미 원하는 상태면 이동하지 않는다.
영점 활성화 이후 기존 `/tool/change`로 도구를 재준비한 뒤, 기존 context가 붙은 토크ENABLE을 사용한다.
초기 영점 경로는 도구 preparation을 FAILED로 둘 수 있으므로 현재 generation의 재준비가 필요하다.
LOCK 완료 후에도 도구 토크는 자동으로 다시 켜지지 않는다.
`toolchanger_zero_file` 미지정시 영점은 RAM만 유지된다.
저장 파일을 FULL_ROBOT 기동 시에도 지정하면 잠금 게이트의 영점 기준을 복원한다.

## 남은 검증

물리 ID1 연결/baud/mode/drive mode, 실제 영점·방향·결합/분리·하중 시험은 미완료다.
잠금 상태는 위치 기반 추정이며 결합 센서 확인을 대신하지 않는다.
기존 OMPL 시간초과, 팔 영점/부호/범위 및 전체축 경로 인수도 미완료다.
GUI 잠금 버튼, 자동 도구교체 시퀀스, 볼 캠 연동은 이번 범위 밖이다.
