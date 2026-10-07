# extreme-robot 폴더 통합

Power-train 레포의 `extreme-robot/`에 로봇팔·인식 레포의 추적 파일 전체를 포함한다.
일반 Git 파일이므로 Power-train을 clone하면 함께 내려오며 submodule 초기화가 필요 없다.

- 원본: https://github.com/ksp118/extreme-robot
- 가져온 브랜치: `main`
- 가져온 커밋: `a7b4f15dd56886253ba8168e43946fc3b88d0e8f`
- 가져온 날짜: 2026-10-07
- 포함: 원본 추적 파일 407개, Docker·ROS 소스·문서·메시·추적 모델 포함
- 제외: `.git`, 로컬 미추적 파일, 빌드·설치·로그 산출물

해당 커밋의 파일 내용과 실행 권한을 그대로 가져왔다. 원본 Git 이력은 원본 레포에
남아 있고 Power-train에는 한 번의 소스 가져오기 커밋으로 기록한다.
원본 README의 별도 clone 절차 대신 아래 내부 폴더를 사용한다.

## 로봇팔 개발 환경

호스트의 Power-train 레포 루트에서:

```bash
cd extreme-robot
docker compose config --quiet
docker compose build
docker compose up -d
docker compose exec ros2 bash
```

컨테이너 안에서:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

호스트의 `extreme-robot/ros2_ws`가 컨테이너 `/root/ros2_ws`에 마운트된다.
GUI·X11 준비와 Jetson GPU override는 `extreme-robot/README.md`를 따른다.
기존 별도 레포의 `ros2_humble` 컨테이너가 있다면 같은 이름의 컨테이너를 새로 만들기
전에 기존 환경의 작업을 정리하고 종료한다. 이번 통합은 실행 중인 컨테이너를 전환하지 않는다.

## 빌드·통신 경계

파워트레인은 `ros2/`, 로봇팔은 `extreme-robot/ros2_ws/`에서 각각 빌드한다.
두 곳 모두 `robot_arm_msgs`가 있으므로 레포 루트에서 두 워크스페이스를 함께
`colcon build`하지 않는다. 각 Docker Compose의 host network·host IPC 설정은 유지한다.
폴더를 함께 관리하는 변경이며 통합 컨테이너나 자동 동시 기동은 추가하지 않는다.

메시지 드리프트 검사는 내부 소스를 기본으로 사용한다:

```bash
bash ros2/scripts/sync_check_msgs.sh
```

가져온 소스에는 기존 파워트레인 벤더 사본보다 `TaskCommand.msg`·`TaskResult.msg`가
추가되어 있어 현재 검사는 종료 코드 1로 드리프트를 보고한다. 기존 메시지 5개의
내용은 일치한다. 계약 변경·재벤더는 별도 변경으로 처리한다.

원본에 추적되던 문서·모델은 내부 `.gitignore`에도 제외 패턴이 있으므로, 해당
파일을 수정할 때는 이미 추적되는 파일을 수정하고 신규 자료는 명시적으로 추가한다.
원본 레포와 자동 동기화하지 않으며 이후 로봇팔 변경도 Power-train PR로 관리한다.
