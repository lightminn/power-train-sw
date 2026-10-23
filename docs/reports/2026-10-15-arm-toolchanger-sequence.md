# 자동 도구교체 시퀀스 구현 결과 (어댑터까지)

레퍼런스 [OSrobotToolChanger](https://github.com/isenklopr/OSrobotToolChanger)의
`changeStatus()` 제어 흐름을 참고해 ID1 잠금 위에 교체 시퀀스를 올렸다.
2026-10-15 사용자 결정에 따라 결합 확인은 **ID1 부하 프록시**를 쓰고,
레퍼런스의 `checkTime()` 주기 감시는 구현하지 않았다. GUI 잠금 버튼은 다음 작업이다.

## 변경

- `toolchanger_sequence.py` 신규: IDLE → TOOL_STOP → MOVING → CONFIRMED,
  이동 불필요 시 UNCHANGED, 모든 실패는 ABORTED.
- `/toolchanger/command`에 `CHANGE`(to/direction) 추가. 기존 `LOCK`/`UNLOCK` 유지.
- `/toolchanger/confirm` 신규: 성공 1회만 발행. 레퍼런스 confirm 릴레이에 대응한다.
- `_LockMotorIO.read()`에 XL430 Present Load(126, 2바이트 부호) 추가.
- 도구 정지·토크OFF 확인 전처리를 `_prepare_tool_for_lock()`으로 뽑아
  원시 LOCK/UNLOCK과 시퀀스가 같은 경로를 쓴다.
- `toolchanger_coupling_load_threshold`(기본0=프록시 OFF), `toolchanger_mock_load` 추가.
- 비상정지·도구분리·노드종료·STOP에서 진행 중 시퀀스를 닫는다.
- 상태 발행은 잠금 snapshot에 시퀀스 snapshot을 합친다.

## 검증 (x86 컨테이너 `extreme-robot-ros2`)

- 설치 빌드: `robot_arm_msgs` `robot_arm_description` `dynamixel_control` 3 packages finished.
- 잠금·시퀀스·브리지 테스트: **31 passed**.
- `dynamixel_control` 전체 스위트: **217 passed / 40 failed**.
  동일 환경의 HEAD 기준선은 201 passed / 44 failed이며 **실패 이름 집합 차집합은 공집합**이다.
  즉 신규 실패 0건이고, 40건은 전부 기존 실패다(flake8·pep257 포함).
  기준선에만 있던 4건은 `dynamixel_control`만 추출한 기준선 사본에서
  GUI/URDF 상대경로가 없어 난 수집 차이다.
- 신규 모듈 `toolchanger_sequence.py`는 flake8·pydocstyle 모두 0건.
  `dynamixel_control`+`test` flake8 총계는 기준선 58 → 현재 56건.
- 설치된 `ros2 run` 엔트리포인트 mock E2E 3블록 PASS, **2회 연속 재현**:
  기존 잠금 블록 → read-only 콜드스타트 → 신규 교체 시퀀스 블록.
- 시퀀스 블록이 확인한 것: UNLOCK CHANGE confirm(RELEASED),
  **음성 대조**(임계값120·주입부하0 → `NOT_ENGAGED` ABORTED, 토크OFF, confirm 미발행,
  `below threshold 120` 사유), 부하150 주입 후 재ENABLE → LOCK CHANGE
  confirm(ENGAGED, peak 150), 동일 상태 재요청 → UNCHANGED·confirm 미발행.
- `mechanically_confirmed`는 전 구간 false를 유지했다.
- 실제 serial/장치 접근과 실기 구동은 수행하지 않았다.

재현:

```bash
# 컨테이너 안
source /opt/ros/humble/setup.bash
source <ws>/install/setup.bash
cd <ws>
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
ROS_DOMAIN_ID=82 python3 src/dynamixel_control/scripts/verify_toolchanger_mock.py
```

## 남은 일

- **GUI 잠금 버튼**(다음 작업). `robot_manual_gui`는 이번에 건드리지 않았다.
- ⚠️ **주기 감시 미구현**: 잠금 이동 순간에만 결합을 추정한다. 그 뒤 정지 상태에서
  도구가 빠지는 것은 감지 경로가 없다. 레퍼런스가 `checkTime()`으로 막던 시나리오다.
- **결합 부하 임계값 실측 미완료.** 기본값0은 프록시를 끈 상태이며,
  실기에서 결합/무결합 스트로크 부하를 측정해 임계값을 정해야 기능이 켜진다.
- 부하 프록시는 결합 센서가 아니다. 물리 결합·탈착·하중 시험은 미완료다.
- 실제 ID1 baud/mode/drive mode, 영점·방향 확인도 기존대로 미완료다.

---

# GUI 잠금 패널 결과 (2026-10-16)

## 변경

- `robot_manual_gui`에 잠금 패널 추가: 상태6항목 표시, 영점 지정(결합/분리),
  잠금 모터 활성화/정지, 도구 결합/분리(CHANGE), 회전 방향 선택.
- `ros_interface`에 `/toolchanger/status` 구독과 `/toolchanger/command` 발행,
  `command_toolchanger()` 게이트를 추가했다. STOP은 막지 않고 나머지는
  쓰기 가능 + MANUAL END_EFFECTOR_ONLY에서만 보낸다.
- 결합 추정이 센서 확인이 아니라는 문구를 패널에 상시 노출한다.
- 한국어 표시 문자열을 `korean_text`에 추가했다. 프로토콜 값은 표시 단계에서만 번역한다.

## 검증 (x86 컨테이너 `extreme-robot-ros2`, offscreen Qt)

- `robot_manual_gui` 전체 스위트: **93 passed / 0 failed**(기존 회귀 없음).
- 신규 패널 단위테스트 9건: 상태 미수신 시 전 버튼 차단, 미보정에서 영점·정지만 허용,
  영점 설정 후 활성화 허용, 토크ON·인덱스 확정에서만 CHANGE 허용,
  FULL_ROBOT에서 정지만 유지, read_only 전면 차단, 방향 선택 페이로드, 경고 문구 상시 노출.
- 신규 ROS+Qt E2E 1건(실제 mock 브리지 + 실제 버튼 클릭):
  MANUAL 전환 → 영점 지정(2048/LOCKED) → 잠금 활성화 → 도구 준비·토크ON →
  도구 분리 CHANGE(−22.5° → 1792, RELEASED confirm, 도구 토크OFF) →
  **음성 대조**(부하0·임계값120 → ABORTED·NOT_ENGAGED·토크OFF·버튼 재차단) →
  부하150 주입 후 재활성화 → 분리 복귀 → 결합 CHANGE(ENGAGED, peak 150).
  전 구간 `mechanically_confirmed` false 유지.
- `robot_manual_gui` flake8 총계 기준선 64 → 현재 64건(신규 0). 신규 파일 findings 0.
- 모터1은 잠금 소유 경로로만 읽는다. `read_position(1)`은 allowlist가 막는 것을 확인했다.

## 남은 일

- GUI 패널은 벤치 수동 경로 전용이다. 자동 도구교체 미션 시퀀스(팔 이동 포함)는 미구현이다.
- ⚠️ 주기 감시 미구현은 그대로다. 정지 상태의 도구 분실은 감지하지 못한다.
- 결합 부하 임계값 실측과 물리 결합·탈착·하중 시험은 여전히 미완료다.
- 실기 GUI 조작 확인은 하지 않았다. 검증은 x86 mock 경로다.
