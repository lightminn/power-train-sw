# ID1 잠금 제어 구현 및 검증

1. 영점·256tick 양방향 상태 전환 코어 → 미보정/경계/통신오류/timeout 테스트.
2. 기존 버스 단일 소유권과 도구 게이트 통합 → 설치된 mock 브리지 ROS E2E.
3. 영점 파일 저장/복원 및 읽기 전용 재시작 → 콜드스타트 E2E.
4. 실제 ID1의 mode/drive mode/영점·결합 상태 확인 → 이후 실기 단계.

구현 파일: `toolchanger_lock.py`, `moveit_dynamixel_bridge.py`.
재현 스크립트: `extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_toolchanger_mock.py`.
영점 가드를 제거한 임시 복사본에서 uncalibrated 검사가 실패했고 원본에서는 통과했다.
모터에 접속하지 않는 mock 환경의 installed ros2 run 엔트리포인트에서
SET_ZERO/ENABLE/UNLOCK(-1)/LOCK(+1)/tool torque 게이트/비상정지 및
read_only 재시작을 확인한다. 실기 연결/구동/NVM 작업은 수행하지 않는다.

## 자동 교체 시퀀스 (2026-10-15)

5. 시퀀스 코어(도구정지 → 이동 → 도달검증 → 프록시 평가 → confirm) →
   verify: 중단 경로별 단위테스트(토크OFF 쓰기와 confirm 미발행 단언).
6. 브리지 `CHANGE` 명령과 `/toolchanger/confirm` → verify: 설치된 ros2 run mock E2E에
   음성 대조(임계값 미달 → ABORTED) 포함.
7. GUI 잠금 버튼 → verify: 패널 게이트 단위테스트 + 실제 mock 브리지에
   버튼을 눌러 교체 시퀀스를 확인하는 ROS+Qt E2E(음성 대조 포함). 2026-10-16 완료.
