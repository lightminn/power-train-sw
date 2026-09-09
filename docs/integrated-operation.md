# 통합 운용: 젯슨 시작 1회, 노트북 콘솔 1회

이 경로는 **CAN 4WS용 통합 기동·연결 경로**다. 최초 준비 후 젯슨에서
`scripts/robot-start`로 서비스를 생성·기동하고, 노트북에서는 앱 메뉴의
**Powertrain Integrated Console**을 실행한다. 어느 쪽을
먼저 켜도 콘솔이 등록된 로봇을 찾아 연결하고, 패드·영상·상태를 각각 표시한다.
주행은 준비 표시를 확인하고 **운전 시작을 1.5초 누른 뒤** 기존 패드 데드맨을 사용한다.
현재 프로젝트 인수는 원격주행을 중심으로 한다. 미완성 자율주행은 이번 테스트·운용
판정 범위에서 제외한다(2026-09-09 사용자 지시).

**2026-09-09 CAN 수정 후 검증:** 단일 CAN/USB 소유권, 리셋 직렬화, 실제 축 상태 기반
무장 확인, 수신 시각과 정지 증거, 세션 복구를 보강했다. 젯슨의 설치된 ROS 노드와
가상 CAN 10축으로 시작→구동→입력 단절 정지→재연결 후 IDLE 유지가 통과했다.
FAKE 정지 증거와 비중립 패드는 운전 시작을 거부했다. 실제 can0는 최초 10모터가
모두 수신됐으나 배포 후 ODrive 13·14가 다시 무응답이 됐다. 제어·워치독 정지 후에도
8/10이었으나, 후속 USB 진단에서 MKS 보드의 node 13/14 설정을 확인하고 **보드 CAN부만
재초기화해 10/10을 복구**했다. 실제 firmware는 이 보드에서 0.5.1 unreleased로 확인했다.
13/14 제한 조회 60초에서는 지속 소실이 미재현이지만 일부 응답 누락은 있었다.
이전 지속 무응답의 원인은 확정하지 않았으며 MKS 송신 경합은 소스에서 확인한 원인 후보다.
후속으로 호스트 조회 분산과 MKS firmware patch1을 만들고 **13·14 보드 한 장**에 적용했다.
원본307설정과 캘리를 보존했고, 0속도에서 제어 단절 약302.5ms watchdog IDLE와 CAN 복구 후
정지 유지·명시 재무장을 검증했다. 300ms는 RAM 시험 후보이며 종료 후 원래 disabled로
복원했다. 전원 재인가1회에서 설정·캘리가 유지됐고, CAN Stuff error 복구로 남은 정지 래치는
안정 확인 후 명시 해제했다. 최종6축 조회600초는10모터 연속수신·stale0·host 오류/드롭0으로
통과했다. 보드 내부 TX drop 누계2→4와 달리 복구횟수2는 불변이었다. **지상 ±0.25 motor rev
목표 시험은 미달로 FAIL**이며 최종6축은IDLE/error0이다. 나머지 보드 firmware 적용과 실주행
인수는 남아 있다. 현재 인수 범위와 재현 경로는
[MKS 신뢰성 수정 기록](reports/2026-09-09-mks-can-reliability.md), 이전 진단은
[MKS USB 기록](reports/2026-09-09-mks-usb-can-diagnosis.md)을 따른다.
US-100·L515의 의도적 분리는 원격 CAN 검증의 센서 결함으로 판정하지 않는다.
상세 근거는 [CAN 수정·배포 검증 기록](reports/2026-09-09-can-remediation.md),
이전 실기 이력은 [통합 운용 검증 기록](reports/2026-09-09-integrated-jetson-validation.md)이다.

## 최초 준비 — 젯슨

젯슨의 기존 이미지 빌드 환경, `/etc/powertrain/powertrain.env`의 **검증된 STOP_MM과
출처**, ops 토큰, 센서 udev 규칙이 필요하다. STOP_MM은 생산 기본값이 없으며 임의의
숫자를 넣지 않는다. 기존 기동 전제는 [`ros2/README.md`](../ros2/README.md)에 있다.

젯슨 SSH 접속 직후, 실제 저장소 위치로 이동한다. 다음 명령의 robot ID는 노트북과
동일하게 정한다. 토큰 내용은 명령줄이나 문서에 붙여 넣지 않는다.

```bash
# 젯슨 호스트 — 최초 1회 (저장소 위치에 맞춰 이동)
cd ~/power-train-sw
sudo bash scripts/robot_prepare.sh \
  --robot-id zetin-rover \
  --token-file /etc/powertrain/ops_console.token
```

기대 출력: `robot_prepare PASS`. 이 단계에서 이미지를 빌드하고 ROS 설치 공간을
준비하며, 안전한 런타임 디렉터리와 부팅 시 CAN 준비 서비스를 설치한다.
ROS 코드·런치나 배포 구성이 바뀌었으면 이 준비 단계를 다시 실행한다.
보호된 토큰의 권한을 공개로 바꾸지 않는다. 과거 호스트 텔레메트리 서비스가 활성화돼
있으면 중복 소유를 피하기 위해 중단한다. 명시적으로 새 경로로 전환하려면 위 명령에
`--replace-legacy-telemetry`를 추가한다. 기존 파일·다른 팀 프로세스를 일괄 종료하지 않는다.

외부 인터넷이 없는 현장에서 호환되는 `powertrain-sw:jetson`과 `powertrain-sw:ros`
이미지를 이미 준비했다면 `--use-existing-images`를 명시할 수 있다. 두 이미지의 존재를
확인한 뒤 이미지 빌드만 생략하고 **ROS 소스는 다시 빌드**한다. 이미지의 의존성·Dockerfile이
현재 소스와 맞는지는 별도로 확인해야 한다. 이 옵션이 없으면 이미지 빌드 실패를 그대로
오류로 처리한다.

작업 카메라와 로봇팔 관측까지 사용할 때는 기존 로봇팔/D435 스택의 준비된 Compose
파일·서비스를 `--optional-compose-file`, `--optional-service`로 등록하고
`--with-arm-d435-bridge`를 사용한다. 옵션 상세는 `bash scripts/robot_prepare.sh --help`.
로봇팔 쪽 과거 `metadata_sender`와 새 bridge를 동시에 켜지 않는다.
새 스택은 `/detected_objects`, `/joint_states`, `/dynamixel/state`, `/pick_target`을
관측하며, D435 원본 장치는 로봇팔 팀이 계속 소유한다.

⚠️ 캘리브레이션 NVM 영속화가 검증되지 않은 모터는 전원 재인가 후 기존 벤치 절차가
필요할 수 있다. 통합 스크립트는 이를 자동 캘리브레이션이나 안전 조건 해제로 우회하지
않는다. **전원사이클 3회 × 6축 직진입 검증**은 목표 운용의 실차 인수 조건이다.

## 최초 준비 — 노트북

젯슨에 등록된 **같은 console 토큰 파일**을 기존 승인된 방식으로
`~/.config/powertrain/ops_console.token`에 준비한다. 파일은 소유자만 읽을 수 있게 둔다.
GTK/GStreamer/gtksink를 제공하는 Python과 pygame을 제공하는 Python을 각각 선택한다.

```bash
# 노트북 호스트 — 최초 1회 (저장소 위치에 맞춰 이동)
cd ~/ZETIN/robotics/power-train-sw
bash scripts/install_integrated_console.sh \
  --robot-id zetin-rover \
  --host jetson-orin.local \
  --token-file "$HOME/.config/powertrain/ops_console.token" \
  --console-python /usr/bin/python3 \
  --controller-python /usr/bin/python3
```

기대 출력: `integrated console install PASS`. 이후 앱 메뉴에서
**Powertrain Integrated Console**을 실행한다. 기존 자동 재시작 콘솔 서비스가 남아
있다면 설치기가 전환을 요구한다. 의도적으로 전환할 때 `--replace-legacy`를 추가한다.
여러 주소 후보가 필요하면 `--host`를 반복한다. IP 변경을 따르려면 이름 해석이 되는
`jetson-orin.local` 같은 호스트명을 사용한다. 서로 다른 네트워크 사이의 자동 경로
구성이나 임의 로봇 검색은 제공하지 않는다.

## 매일 사용하는 순서

1. 최초 기동을 마친 젯슨과 로봇의 전원을 켠다. 아래 부팅 복귀 조건을 충족하면
   필수 서비스는 자동 복귀하므로 매일 SSH 명령을 실행할 필요가 없다.
2. 노트북을 켜고 **Powertrain Integrated Console**을 연다.
3. 패드를 연결하고 스틱·버튼을 놓는다. 전방 영상·입력·안전 상태가 준비되면
   **운전 시작 (1.5초)**을 누른 채 확인한다.
4. 기존 패드 데드맨을 눌러 조종한다. 멈추고 주행 권한을 해제하려면 **주행 해제**.
   즉시 비상정지가 필요하면 기존 **긴급 정지** 또는 패드 ESTOP를 사용한다.

```bash
# 젯슨 호스트 — 최초 기동 또는 명시적 정비 정지 후 재개
~/power-train-sw/scripts/robot-start
```

2026-09-09 현장 배포는 기존 팀원 체크아웃을 보존하기 위해 별도
`~/power-train-sw-integrated`에 설치했다. 해당 배포를 명시적으로 기동하는 명령은
`~/power-train-sw-integrated/scripts/robot-start`다. `robot-start PASS`는 준비된
서비스 기동 성공이며 모터·센서의 주행 준비 완료를 뜻하지 않는다.

통합 Compose의 필수 8개 서비스는 `unless-stopped`를 사용한다. 최초
provisioning과 `robot-start`가 성공하고, Docker가 부팅 시 실행되며, 서비스를
명시적으로 정지하지 않은 상태가 매일 콘솔만 열 수 있는 전제다. 기존
`powertrain-bringup-preflight`와 안전한 런타임 디렉터리 준비도 유지해야 한다.
별도 stack 자동 기동 유닛은 만들지 않는다. 선택 로봇팔/D435·환경 센싱 스택의
자동 복귀는 해당 소유자의 설정과 별도 준비에 달려 있다.

정비에서 `docker compose stop`으로 명시적으로 멈춘 서비스는 재부팅해도
정지 상태를 유지한다. `down`으로 컨테이너를 제거했다면 자동 복귀할 대상도 없다.
정비를 마치고 안전 조건을 확인한 후 위 `robot-start`를 다시 실행한다. 콘솔은
꺼진 로봇 서비스를 SSH로 켜거나 자동 arm하지 않고 연결 대기를 표시한다.

이 정책 변경은 아직 로컬 병합 준비 단계다. 현재 Jetson 배포에 적용되었다고
가정하지 않는다. 실제 재부팅 복귀는 후속 배포·인수에서 확인해야 하며, 로컬
`docker compose ... config` 검증은 서비스 실행이나 실제 부팅 성공의 증거가 아니다.

노트북 터미널에서 같은 콘솔을 열려면 최초 설치가 만든
`~/.local/bin/powertrain-integrated-console`을 실행한다. 직접 실행은
`python -m operator_console --config ~/.config/powertrain/operator.json`이다.

‘운전 시작’은 현재 상태를 확인한 뒤 필요한 경우 일시 HOLD 해제 → 수동 권한(TELEOP)
→ arm 순서로 처리한다. 각 명령의 최종 ACK와 실제 상태를 확인한다.
**ESTOP 초기화는 별도 조작**이며, 네트워크 재연결·패드 재연결·창 재실행은 주행을
재개하지 않는다. 결과 불명은 성공으로 표시하지 않는다. 시작 결과가 불명확하면
주행 해제를 시도하고, 확인되지 않은 결과는 그대로 남긴다.

의도적으로 US-100을 분리한 현장 시험은 상단 두 탭 옆의
**복구 · 설정 → US-100 안전 → 확인**으로
미장착 상태를 선택한다. 자동 충돌정지 해제 배너를 확인하고, 활성 ESTOP 원인이
없어진 뒤 **경고 초기화 → 확인**을 별도로 실행한다. 미장착 설정은 차체 재시작 시
다시 ON으로 돌아간다. 단순 통신 장애를 미장착으로 처리하지 않는다.

**복구 · 설정**은 비모달 창이다. 창을 닫거나 초점을 잃으면 진행 중 확인 동작이
취소되며, 다시 열어도 같은 인증 ops 세션을 사용한다. 창을 여는 것만으로는 명령이
전송되지 않는다. **운전 시작·주행 해제·긴급 정지**는 기존 상단 위치에 계속 보인다.

초기 후보 패드 매핑은 **L1 데드맨 + R2 전진 / L2 후진**, 왼쪽 스틱 조향,
○ 비상정지다. 주행 해제는 실제 6축 속도의 정지·신선도와 차체 비활성 상태를
확인한다. 이미 걸린 MOTION_HOLD는 정지 확인 후에도 유지하며 자동 해제하지 않는다.

## 연결되는 기능과 상태 표시

| 기능 | 통합 경로 |
|---|---|
| 패드 수동 운전 | 콘솔이 별도 패드 프로세스를 관리, 인증된 입력 프록시 :9000 |
| 복구·ESTOP·운용 설정 | 두 탭 옆 **복구 · 설정** 창 → 기존 역할 토큰과 ops 게이트 :9001, ESTOP 초기화 별도 |
| 전방 영상 | L515 Gateway 단일 소유, SRT :5000 |
| 작업 영상·인식·로봇팔 상태 | 준비된 선택 스택 + SRT :5002 / UDP :5003·5007 |
| 전원·차체 상태 | UDP :5004·5005, 현재 인증된 노트북 주소로 전송 |
| 부팅 순서/IP 변경 | 등록된 IPv4 호스트 후보 재해석, robot ID와 토큰으로 상대 확인 |

세션 :9002는 동시 운용자 한 명만 허용한다. 연결 임대가 끝나면 입력/ops 프록시를
닫고 주행 해제를 요청한다. 원래 입력 게이트의 0.2초 신선도 감시는 그대로 유지한다.
패드 입력 채널만 끊어진 경우에도 세션을 끝낸다. 새 입력은 신선한 차체 IDLE·
바퀴 정지 확인 뒤에만 받으므로, 짧은 패드 분리에도 이전 주행 허가를 이어받지 않는다.
영상은 노트북이 SRT caller로 연결하고, UDP 목적지만 실제 인증된 연결 상대의 주소를
사용한다. 연결할 때마다 root 설정 파일을 고치거나 송신 서비스를 재시작하지 않는다.

HMAC은 짝 확인을 위한 인증이며 암호화 전송은 아니다. 기존 운용망 안에서 사용한다.
토큰·세션 ticket을 로그나 화면 캡처용 진단 자료에 출력하지 않는다.

자율주행 배포, 환경 센싱 Draft PR, 미완성 로봇팔 원격 조작을 자동으로 켜는 기능은
이 변경에 포함되지 않는다. 선택 센서가 없으면 해당 상태는 미수신으로 표시된다.
전방 영상 또는 패드 입력이 없으면 원격 운전 시작은 비활성화된다.
수신 포트 설정은 진단용으로 바꿀 수 있지만 송신 포트와 별도로 맞춰야 한다.
통합 세션은 UDP 포트를 협상하지 않으며 생산 기본값은 위 표의 포트다.
환경 텔레메트리 포트는 `~/.config/powertrain/operator.json`의
`environment_telemetry_port`로 지정하며 기본값은 `5008`이다. 1–65535 범위의 정수만
허용되고, 잘못된 값은 세션·네트워크 시작 전에 콘솔 기동을 거부한다.

## 문제 확인

| 표시/증상 | 확인할 항목 |
|---|---|
| 준비 매니페스트 없음 | 해당 저장소에서 최초 `robot_prepare.sh` 실행 여부 |
| 로봇 연결 대기 | 젯슨 `robot-start`, 같은 네트워크, 호스트명 해석, 양쪽 robot ID/토큰 |
| 이미 실행 중 | 기존 콘솔 창 또는 과거 자동 실행 서비스 확인 |
| 패드 대기 | 실제 연결, pygame 환경, 지원 DualSense GUID·USB/BT 매핑 |
| 전방 영상 대기 | L515 Gateway 로그와 SRT :5000; 다른 프로세스의 카메라 중복 소유 |
| 최신 상태 대기 | control/chassis 및 각 센서 상태. 안전 센서를 끄는 방식으로 우회하지 않음 |
| 거부/결과 불명 | 원인 확인 후 명시적으로 다시 조작. 비상정지 초기화는 별도 |

```bash
# 젯슨 호스트 — 해당 저장소에서 상태/로그 확인
docker compose -f docker/docker-compose.jetson.yml \
  -f docker/docker-compose.integrated.yml ps
docker compose -f docker/docker-compose.jetson.yml \
  -f docker/docker-compose.integrated.yml logs --tail=100 \
  powertrain_session powertrain_control powertrain_chassis
```

## 실차 인수 기준

- 젯슨 먼저 / 노트북 먼저 두 경우 모두 수동 IP 수정 없이 연결.
- 실제 전방·작업 SRT 화면과 4개 UDP 채널의 LIVE→STALE 전이 확인.
- 패드 제거·창 종료·네트워크 단절·프로세스 종료 후 실제 정지 확인.
- 연결 복구 뒤 바퀴가 자동으로 다시 움직이지 않음.
- ESTOP가 유지되고 별도 원인 해소/초기화 후에만 운전 시작 가능.
- 전원사이클 캘리브레이션 게이트, stop_mm, 지상 제동 검증 통과.
- 설치된 ROS 엔트리포인트 실기동, Compose health/journal 무크래시 확인.

설계는 [`docs/specs/2026-09-08-integrated-operation.md`](specs/2026-09-08-integrated-operation.md),
구현 당시 검증은 [9/8 기록](reports/2026-09-08-integrated-operation.md),
Jetson 배포·전체 루프 결과는 [9/9 기록](reports/2026-09-09-integrated-jetson-validation.md).
