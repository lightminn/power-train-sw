# robot_arm_msgs — 벤더링 사본 (⚠️ 원본 아님)

이 패키지는 **로봇팔 팀 소유**다. 정본:

    github.com/ksp118/extreme-robot : ros2_ws/src/robot_arm_msgs

우리(파워트레인)는 두 팀을 **분리 개발**하되 ROS2 메시지 계약만 공유한다. ROS2 타입은
wire 에서 **패키지명 + 구조 해시**로 매칭되므로, 동일한 `.msg` 로 각자 빌드하면 우리
빌드본과 그들 빌드본이 그대로 호환된다(그들 install 을 오버레이할 필요 없음 = 완전 분리).
그래서 `.msg` 정의만 이 폴더에 복사해 **우리가 직접 빌드**한다.

## 동기화 기준

- **SYNCED_FROM**: `ksp118/extreme-robot` @ `f976710` (origin/main, 2026-07-07 확인)
- **msg 마지막 변경 커밋**: `9bf6d428` (`20260626 젯슨-PC토픽파이프라인_1차`)
- 복사 대상: `msg/*.msg` 5개 + `package.xml` + `CMakeLists.txt` (원본 그대로)

## 드리프트 관리

로봇팔 팀이 `.msg` 를 바꾸면 이 사본과 어긋난다(= 계약 변경). 감지:

    bash ros2/scripts/sync_check_msgs.sh          # ~/extreme-robot 대비 diff

⚠️ 계약 변경은 **양 팀 합의 사항**이다 — 드리프트가 잡히면 임의 재복사하지 말고 먼저
합의한 뒤 재벤더 + `docs/plans/2026-07-02-...` 계약 절 갱신.
