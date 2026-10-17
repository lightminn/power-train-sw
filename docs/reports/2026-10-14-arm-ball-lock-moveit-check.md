# 2026-10-14 현재 로봇팔 자세 MoveIt 검사

## 판정 범위

사용자가 확인한 joint4가 내려간 현재 가상 자세를 그대로 읽었다.
해당 자세의 충돌 검사와 joint4 -0.02 rad 이동 및 +0.02 rad 복귀의
MoveIt PTP 계획·mock 실행을 확인했다. 실기 안전성이나 전체 작업공간 인수가 아니다.

## 초기 자세

- arm_joint_1: -0.786342924597 rad (-45.054131°)
- arm_joint_2: -0.813794742432 rad (-46.627004°)
- arm_joint_3: -0.935099151246 rad (-53.577235°)
- arm_joint_4: 0.880704779132 rad (50.460667°)
- arm_joint_5: -0.457769546879 rad (-26.228263°)

나머지 그리퍼·로터·볼 관절은 현재 수신값 0이다.
현재 상태 충돌 검사 결과 valid=true, contacts=[]였다.
원래 all-zero home 자세와 구분하며 생산 home/영점 설정은 변경하지 않았다.

## 이동 확인

- joint4: 0.880704779132 → 0.860704779132 rad, 약 1.145916° 이동.
- 다른 팔 관절은 고정했다.
- 이동/복귀 각각 중간 21개 자세의 충돌 검사 모두 통과.
- pilz_industrial_motion_planner / PTP로 이동/복귀 각각 경로점 10개 생성.
- MoveIt ExecuteTrajectory → arm_controller → GenericSystem → joint_states 경로로
  이동/복귀 모두 계획 code=1, 실행 code=1, 최종 관절 오차 0 rad.
- 마지막에는 처음 읽은 현재 자세로 복귀했다.

## 한계와 가상 환경 변경

이전 확인 때 설정한 랙-피니언 2쌍·볼-로터 8쌍의 내부 접촉 제외를
현재 가상 planning scene에서 유지했다. 저장소 SRDF에는 추가하지 않았다.
따라서 위 결과는 이 10쌍을 임시 제외한 조건에서만 유효하다.
팔의 arm_link3/tool_link 충돌은 제외하지 않았다.

5축을 모두 ±0.02 rad 움직이는 두 후보는 중간 자세 충돌로 탈락했다.
OMPL 계획은 관절 공간 강제 설정 전후 모두 시간초과(code -6)였다.
초기 가상 OMPL 설정에 기본 플러그인·시간 부여 어댑터가 누락돼 다른 계획기가
선택되고 시간정보 없는 경로가 생성된 시행은 컨트롤러가 거부했다.
기본 어댑터를 복원하고도 OMPL은 시간초과였다. 성공 판정은 위 PTP 시행에만 적용한다.

PTP 시험을 위해 컨테이너 안의 임시 설치 설정에 가속도 1 rad/s²,
감속도 -1 rad/s²를 추가했고 속도·가속도 scaling은 각각 0.1이었다.
이 값은 실측 모터 사양이 아니며 저장소·실기 제어 설정에는 반영하지 않았다.
가상 모터 컨테이너는 network none, ROS_DOMAIN_ID=77, 실제 모터 장치 미마운트다.

## 증거

- `/tmp/arm_current_pose.json`
- `/tmp/arm_current_ptp_outbound.json`
- `/tmp/arm_current_ptp_return.json`
- 컨테이너 `/tmp/arm_demo_ptp.log`

## 남은 일

OMPL 시간초과 원인, 기본 home 자기충돌, 내부 기구의 실제 연동·충돌행렬,
실기 영점·부호·한계와 잠금 모터 ID/결합 각도를 별도로 확인한다.
