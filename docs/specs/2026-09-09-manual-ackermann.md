# 수동 애커만 조향과 URDF 기하 계약

2026-09-09 사용자 승인: 자동차 핸들처럼 정지 상태에서도 L스틱으로 조향하고, RT/LT로 전진/후진한다. 자율주행 인수는 제외한다.

## 문제와 원인

기존 `left_x → angular.z` 경로에서 RT/LT를 놓으면 v=0, ω≠0인 피벗이 된다. 양방향 피벗은 동일한 조향 배치에 구동 부호만 바뀐다. 실제 CAN 관측 156개에서 모두 v=0이고 AK 1~4 raw 목표는 [+45,-45,-45,+45]였다. 좌우 부호를 추가로 뒤집어 고칠 문제가 아니다.

## 명령과 기하

- 기본 수동 경로는 `ManualDriveCommand(speed_mps, steering)` → `/teleop/drive_command` → 단일 authority → `ChassisManager.set(..., steering=)` → `solve_steering`이다. steering은 -1~+1, 양수=왼쪽, 최대 입력=기하로 허용되는 최대 곡률이다. v=0이어도 독립 전달한다.
- RT−LT는 속도, −L스틱X는 steering이다. 같은 steering이면 정지/전진/후진에서 같은 각도, 속도 부호만 반전한다. deadman, 중립 확인, 명시적 운전 시작, stale/ESTOP/권한 인계는 그대로 적용한다.
- `manual_command_format=twist`로 명시한 레거시 경로와 자율 Twist의 물리 요레이트 의미는 유지한다. 두 수동 토픽을 동시에 구독하지 않는다. 신규 메시지가 없으면 임의로 레거시로 전환하지 않는다.
- URDF 정본은 `power-train-sim/rover_arm_integrated/urdf/2026_07_24_URDF.urdf`, SHA256 `29581f39e888c6120fb4e31860609ff084c9233bdac53d54a3adfe47eac11afa`다. 전체 joint chain을 q=0으로 합성하고 CAD (X,Y,Z) → REP103 (Y,-X,Z)로 변환한다. 앞뒤 축 중점 원점과 좌우 대칭화를 유지한다. 수치는 `docs/reports/2026-09-09-ackermann-urdf-evidence.json`에 기록한다.
- 축거 875.494655 mm, 윤거 545/719/425 mm, 중간축 x=-60.3355675 mm다. 고정 중간륜이 옆으로 밀리지 않도록 회전 중심은 중간축 연장선 위에 둔다. 바퀴 횡속도는 ω×(x_i−x_mid), 전진성분은 v−ω×y_i다. 차체 원점에서 vy=-ω×x_mid는 정상이며 기존 오도메트리가 이를 추정한다.
- 바퀴 각도는 속도와 독립 계산한다. 곡률 한계는 모든 조향륜 ±45°를 만족하는 값으로 계산한다. 속도/요레이트 상한은 전체 구동속도를 함께 줄여 각도를 유지한다. 기본 수동 요레이트 상한은 1.2 rad/s다.
- URDF continuous 조향축은 기계적 허용각 증거가 아니다. 운용 한계 ±45°와 무하중 반경 103.56 mm를 유지한다. 실측 좌측 구동 invert, 전체 AK invert는 그대로다.

## 경계와 인수

- 명시적 skid 모드는 별도 차동 yaw 조작이며 정지 선회가 가능하다. 기본 ackermann에는 암묵적 피벗이 없다.
- `assist_enabled=true`는 신규 steering 명령 형식과 함께 시작할 수 없다. 기존 보조주행은 명시적 twist에서 지원한다. 속도와 조향각 의미를 보존하는 보조주행 결합은 별도 작업이다.
- `powertrain_msgs`와 `powertrain_ros`를 함께 재빌드/배포한다. 오래된 설치물이 남지 않도록 소스·설치물을 비교한다.
- 검증: 실패 선행 단위테스트 → 기하/odometry 왕복 → 설치 ROS 엔트리포인트와 가상 CAN의 인증 세션 전체 루프 → 운영 배포 상태 확인 → 운전자 실물 좌/우·정지·전후진 확인. 마지막 단계 전에는 실차 조향 해결 완료로 선언하지 않는다.
