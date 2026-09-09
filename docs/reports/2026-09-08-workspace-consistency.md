# 코드·문서·Notion 정합성 정리

확인일: 2026-09-08. 비교 기준: local main `c3fd0786`, 당일 origin 조회와 관련 ZETIN Notion.
코드·문서로 판정 가능한 불일치를 수정하고 아래 검증을 마쳤다. 실물·팀 확인이 필요한
항목은 마지막 절에 미완료로 구분했다.

## 수정 범위

| 항목 | 정정 내용 | 근거·수정 위치 |
|---|---|---|
| 모터 GUI 명령 timeout | pending 요청을 원자 취소해 지연 실행 방지. 실행을 시작한 I/O는 `OUTCOME_UNKNOWN` | `motor_gui/backend/worker.py`, worker 경쟁 테스트 |
| 모터 GUI 표시 | ODrive 기본 node 11·AK id 1. 미확정 결과를 거부/실패와 구분. 응답 대기 중 선택을 바꿔도 요청 당시 프로파일로 결과 표시 | `motor_gui/README.md`, `frontend/app.js` |
| 콘솔 조향 상태 | `ackermann`·`skid`만 인정. 누락·빈 값·미지 값은 `상태 미확인`과 비활성 | `operator_console/ops_panel.py`, 기존 상태·전환 테스트 |
| 콘솔 실행 검증 | 기존 GTK/UDP smoke에 loopback ops TCP 상태와 실제 GTK 버튼 검증 추가 | `runtime_smoke.py`, 기존 `app.py` smoke probe |
| ROS 계약 | `WheelState.command_turns_per_s` 존재, `/chassis_state`는 `std_msgs/String` | `WheelState.msg`, `chassis_node.py`, `ros2/README.md` |
| HIL 상태 | 옛 WP5.1 NOT RUN을 당시 기준선으로 구분. 7/11 벤치 HIL과 지상 제동 게이트 분리 | `ros2/README.md`, [WP5.1 보고서](2026-07-10-wp5-control-safety-hil.md) |
| 기하·속도 | 제작 v2: 반경 103.56 mm, 윤거 545/719/425 mm, 축간거리 CAD 875.5 mm·반올림한 코드 좌표 875.4 mm. v4 최적화 100 mm·50 kg와 구분 | `chassis/kinematics.py`, 지침 쌍, Notion 기하 표·실행 예시 |
| 운용 단위 | 설계 기본 0.80 m/s와 수동 운용 1.5 m/s·1.2 rad/s 구분. GUI 회전 부호와 감속 후 휠 속도 단위 구분 | `teleop_command_node.py`, GUI USB/CAN backend, 지침 쌍 |
| NVM | 무조건 RAM-only·매 전원 재캘리 설명 제거. CAN 캘리 명령은 RAM 갱신, 저장은 별도 USB 경로 | `bl70200_setup.py`, 도움말·preflight·USB 드라이버 문구, 운용 Notion |
| NumPy/JAX | 7/19 NumPy 선택 종결. JAX는 x86 실험 커널. 3케이스×100샘플과 30분 전체 E2E 자격 분리 | [선택 보고서](2026-07-19-terrain-backend-jetson-qualification.md), autonomy README·주석, 지침·세 계획 |
| autonomy 배포 | controller 코드는 존재하지만 standalone Compose는 idle. 잘못된 'controller absent' 메시지 정정 | `docker/Dockerfile.autonomy`, `docker-compose.jetson.yml` |
| 배포 준비·서비스 소유권 | runtime 준비 후 이미지 빌드, 실제 기동은 토큰·STOP_MM·preflight 운용 절차 연결. control=teleop/ops, chassis=차체/US-100 | 루트 README, Compose 안내, 전체계획·WP5.2, Notion 설명본 |
| 영상 feedback | 노트북 :5006 송신·정책 코어와 미연결 Jetson 수신·프로파일 적용 구분 | `receiver_feedback.py`, `recv_remote_operation.py`, README·계획·Notion |
| 환경 센싱 | SSH JSONL→UDP :5008→환경 탭은 main이 아닌 Draft PR #4. 불꽃 배선은 8/28 기록의 AOUT→AIN2로 정합화 | [PR #4](https://github.com/lightminn/power-train-sw/pull/4), 환경 센싱 Notion |
| L515 클라이언트 수 | 무제한 접속 표현을 기본 동시 8개로 정정 | `UnixControlServer(max_clients=8)`, L515 README |
| wheel-stop bag | 실제 3토픽과 run1 45.6초/2,475건, run4 74.8초/3,942건 기록 | [bag README](../hil_data/2026-07-16-wheelstop/README.md), SQLite·metadata |
| 계획·지침 권위 | 오래된 `.claude/AGENTS.md` 덤프 제거 후 정본 쌍 링크. '남은 SW 없음' 삭제 | 루트 `AGENTS.md`·`.claude/CLAUDE.md`, 세 기술 계획, Notion 전체 계획 |
| 종료·역사 자료 | MuJoCo 신규 완료 요건 제거. 구형 특허 수치·8모터·과거 출품 상태는 이력으로 한정 | [문서 안내](../README.md), 특허 Markdown 경고, 계획·Notion |
| Notion 내부 모순 | 1:5 속도 환산·cpr=60·폐기된 min_rev 플로어·질량 합계·중복 표·제목/본문 범위 정정 | 아래 재조회 목록 |

## 수치와 검증 범위

- 기하 예시는 현재 순수 함수 `solve(default_geometry(), 0.4, 0.4)`를 호스트에서 실행했다.
  앞 좌/우 조향 31.0°/19.0°, 뒤 좌/우 −29.1°/−19.8°다. 새로운 CAD·지상 실측은 아니다.
- NVM의 증거는 [운용 정본](https://www.notion.so/3b02d27b08d381d99641e3565fe40ca2)에 기록된
  node 11/12 전원사이클 3회 직진입과 node 13~16 재인가 후 재캘리 없는 운용 관찰이다.
  13~16의 동일 3회 반복시험·USB 플래그 직접 확인을 완료 처리하지 않았다.
- Notion의 질량 표 합계는 68.1 kg다. 뒤의 86 kg·가속력 100 N·토크/출력 결론은 혼합된
  가정의 초기 검토식으로 철회 표시했다. 이 값으로 완성차 질량이나 모터 요구 성능을 새로 확정하지 않았다.
- 부하 시험 페이지의 휠 1 m/s = 모터 7.9577 rev/s는 반경 0.1 m·감속 1:5 가정의 환산이다.
  예전 `input_vel=1.5915` 기록은 휠 0.2 m/s이며 실제 1 m/s 부하 시험 증거가 아니다.
- GTK/UDP/loopback TCP smoke와 fake HTTP E2E는 실제 원격 SRT 프레임·ROS·모터·지상 주행 시험과 다르다.

## 이번 실행 검증

최종 실행 결과를 아래에 기록한다. 서로 겹치는 좁은 테스트와 전체 스위트를 합산하지 않는다.

| 확인 | 결과 |
|---|---|
| motor_gui worker 경쟁 회귀 | 기존 HEAD에서 5 failed → 수정 후 5 passed |
| motor_gui 전체 | **141 passed** |
| motor_gui 실제 app.js Node VM | **9케이스 passed**. 이전 frontend는 미확정 메시지 assertion 실패, 선택 변경 결함은 TypeError 재현 후 수정 |
| fake 서버의 frontend 제공 | 실제 HTTP index→app.js 경로와 현재 파일 SHA-256 일치, 정상 startup/shutdown 로그 확인 |
| fake FastAPI/Uvicorn HTTP→worker→FakeTransport | capabilities·arm·mode·input·estop·safety 정상 응답, estop latched·armed false |
| 콘솔 상태 회귀 | 7 failed → 7 passed |
| 콘솔 전체 | **286 passed**, GTK 계열 경고 13건 |
| 실제 GTK/ops/UDP runtime smoke | **PASS**, 31 tick·4채널 LIVE→STALE. 실제 수신 revision=1·미지 mode와 GTK 비활성 확인. ops 미송신 및 기존 KeyError 재주입은 각각 FAIL |
| autonomy | 199 passed |
| 문자열만 정정한 motor/calibration/launch 관련 | 32 passed |
| ROS 순수 계약·CLI 관련 | 48 passed (설치 ROS 엔트리포인트 검증과 구분) |
| L515 feedback/control 관련 | 45 passed |
| 배포 하드닝 계약 | 17 passed |
| Compose 구문 및 실제 idle shell | `docker compose config --quiet` exit 0. false는 sleep, true/1은 exit 64 |
| 문구 정정 Python 11파일 | 문자열·문자열 보간의 고정 텍스트를 제외한 AST가 HEAD와 동일. 오래된 README 문자열 assertion 삭제는 별도 검토 |
| 추가 기하·preflight 문구 | ChassisManager **92 passed**. 하드웨어를 열지 않는 기하/캘리 안내 함수 4개 직접 실행·수치/문구 확인 |
| 레거시 manifest 확인 | **11 passed / 2 failed**, 기존 `procedural-dev-0/analytic` 체크섬 불일치 재현 |

실행 경로는 `PYTHONPATH=$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros`를 지정했다.
콘솔은 PyGObject가 있는 시스템 Python, 나머지는 NumPy·pytest·CAN 의존성이 있는 환경에서 실행했다.
테스트 수집 중 잘못된 PYTHONPATH로 난 import 오류는 경로 수정 후 재실행했고 제품 회귀로 세지 않았다.
HTTP 제공 검증에서 Uvicorn의 정상 SIGTERM 반환 `-15`를 최초 하니스가 실패로 해석했다.
정상 종료 로그·traceback 부재와 신호 종료 코드를 함께 검사하도록 하니스를 정정하고 재실행했다.

Codex 독립 교차검토를 수행했다. 문서 추가 지적 5건과 GUI의 미확정 표시·대기 중 선택 변경
오류를 수정하고 재확인했으며, 검토 범위의 잔여 지적은 없다. 브라우저 시각 검증이나 실기
검증을 이 코드 리뷰로 대신하지 않았다.

## Notion 반영·재조회

Claude Code에 연결된 Notion에서 매 세션 `notion-fetch id=self`로 ZETIN workspace
`a622d27b-08d3-81e5-bf6c-0003781fc028`을 확인했다. 사용자 요청 범위의 기존 페이지를
부분 치환·상태 주석으로 수정하고 재조회해 문구·수치·링크를 대조했다.
**25개 페이지**, 본문 부분 치환 83건·상태 주석 25건·제목 정정 2건을 반영했다.
같은 페이지의 후속 정정을 포함한 수이며 83개의 독립 결함을 뜻하지 않는다.
최종 응답에서 모든 치환 문구·상태 주석·제목을 대조했고, 자식 페이지·데이터베이스·
미해석 블록·이미지 참조 수가 유지됨을 확인했다. 중복 문단·표는 한 벌만 남겼다.
Notion 직렬화 과정에서 협업 문서 2개의 셸 줄 연결 문자가 사라지는 현상을 발견해
해당 명령 12블록을 코드 블록으로 복구하고 재조회했다. 긴 LM5116 페이지도 별도 fetch의
전체 응답을 읽어 상태 주석 외에는 수식 들여쓰기만 달라졌음을 확인했다.

| 수정·재조회 페이지 | 결과 |
|---|---|
| [📘 2026 국방로봇 자율주행 SW 전체 개발계획](https://www.notion.so/39c2d27b08d381728c1ade21cc72216b) | 반영 확인 |
| [🔄 Github 미러 — 레포 README·문서 지도](https://www.notion.so/35c2d27b08d380709e9ee06dc5b66664) | 반영 확인 |
| [🟢 4WS 애커만 키네마틱스 — 차체 명령(v, ω) → 바퀴 조향·속도](https://www.notion.so/3912d27b08d381a0a452fa4afdc61c45) | 반영 확인 |
| [🟢 차체 통합 제어 ChassisManager — 코너 6개를 하나의 4WS 차체로](https://www.notion.so/3912d27b08d381e79716e04398e34bd2) | 반영 확인 |
| [🟢 무선 원격주행 — DualSense→노트북→젯슨→10모터 4WS](https://www.notion.so/39b2d27b08d38140bf8df53fe7661c6c) | 반영 확인 |
| [🟢 L515 Gateway·TUI — 카메라 단일 소유·SRT 원격주행](https://www.notion.so/39a2d27b08d381eb8307fa7d136ad374) | 반영 확인 |
| [🟢 통신 GUI·스트리밍·전원 텔레메트리 — 통합 현황](https://www.notion.so/39d2d27b08d3815c907ae8aa338c5fa8) | 반영 확인 |
| [🌡️ 🌡️ 환경 센싱 엔드 이펙터 — 배선·RPi 임시 연결 기록](https://www.notion.so/3c32d27b08d3811aba24e3dbf70a63ef) | 반영 확인 |
| [구동모터 1 m/s 정속 회전 — 부하 토크 측정](https://www.notion.so/3892d27b08d38184bdbec82277c9963e) | 반영 확인 |
| [Motor and Weight](https://www.notion.so/32c2d27b08d3800a9fc6edcd6aa3d423) | 반영 확인 |
| [BOM (Bill of Materials)](https://www.notion.so/3342d27b08d380a380d8c2a1363fcb32) | 반영 확인 |
| [🟢 ODrive(BL70200) 셋업 — 공장초기화→구동](https://www.notion.so/3882d27b08d381fcbe3cd0c829687c3a) | 반영 확인 |
| [🟢 motor_gui — 웹 모터 진단·튜닝 GUI](https://www.notion.so/3892d27b08d3811eb174e787808db3c2) | 반영 확인 |
| [🟢 RViz 로봇 시각화 — 오도메트리·IMU·장애물 감지](https://www.notion.so/39b2d27b08d3815da7c6f46e173d7a8a) | 반영 확인 |
| [🗺️ 4륜 임시구동 · CAD URDF 형상 복원 · L515 RGB-D SLAM 파이프라인 (2026-07-14)](https://www.notion.so/39c2d27b08d3817493fff5867fb94cae) | 반영 확인 |
| [🟢 로커보기 파라미터 최적화 v4 — 결과·주행 애니메이션](https://www.notion.so/36b2d27b08d3819b9303d1f8554b0425) | 반영 확인 |
| [배선 정리](https://www.notion.so/36b2d27b08d38096bd50f98caa2c716a) | 반영 확인 |
| [48V - 18V conv + 18V - 5V conv + output + Denoise + protection](https://www.notion.so/38e2d27b08d38007b41ccb18cc8be7e8) | 반영 확인 |
| [로봇팔↔파워트레인 통합 개발 계획 (커스텀 msg 기반)](https://www.notion.so/38a2d27b08d381468b03f287251ccd42) | 반영 확인 |
| [로봇팔↔파워트레인 통합 개발 계획 (커스텀 msg 기반) (2026.07.13)](https://www.notion.so/39b2d27b08d38064bdb0cd764f749d2a) | 반영 확인 |
| [PCB 기초 용어](https://www.notion.so/3692d27b08d380459e72e00ce3a0b6ee) | 반영 확인 |
| [예시 Workflow](https://www.notion.so/3692d27b08d3808aa858f50db6c2c2ef) | 반영 확인 |
| [ODrive 전원용 벅 컨버터 BOM — 2026-05 초안](https://www.notion.so/36b2d27b08d3800e93a2d60060ac279b) | 반영 확인 |
| [입력 보호 회로 검토 — 킬스위치 설계 미작성](https://www.notion.so/36b2d27b08d38092b63af259764fe8ea) | 반영 확인 |
| [48V-12V Converter](https://www.notion.so/3642d27b08d380b3a6f0eb6b8c75657b) | 반영 확인 |

## 남겨 둔 실제 미완료 항목

- Jetson DNS 해석 실패로 그 checkout·실제 프로세스·모터 상태는 확인하지 못했다.
  origin 조회는 성공했고 local main은 당시 origin/main보다 1커밋 앞섰다.
- PR #4는 OPEN/DRAFT, 확인 head `0011da514af2eea58e28b8173a57f469fd99b986`다.
  팀원 PR을 병합하거나 Jetson에 배포하지 않았다.
- autonomy standalone 발행 연결, Jetson receiver feedback 적용, 부팅 자격의 실드라이버 증거
  연결은 별도 구현 작업이다. 현재 존재한다고 적힌 설명을 바로잡았으며 새 기능을 활성화하지 않았다.
- 장착·센서 교정·전체 파이프라인 장시간 부하·USB 스키드 실기 안전·최종 `stop_mm`·팔 전체
  핸드셰이크는 기록된 코어/벤치 시험으로 닫지 않았다.
- 전원 레일 A/B/C 도식과 배선 표 중 어느 쪽이 실물인지는 SW 문서만으로 선택할 수 없다.
  해당 Notion을 미확정 초안으로 표시했다. 공란 시험 DB·체크리스트도 측정값을 만들어 채우지 않았다.
- 종료된 `parameter_calc` CPU/JAX의 full-mass/half-side 차이는 보고만 한다. f_opt 재계산과
  폐기된 MuJoCo 코드·fixture checksum 갱신은 하지 않았다. 위 2개 기존 실패도 그 경계 안에 있다.
- 특허 Markdown 2개에는 아카이브 경고를 추가했다. Git 제외된 특허 PDF·Typst·도면·ZIP과
  과거 출품 DOCX는 원본을 보존했으며 새 현행 제출물을 만든 것은 아니다.
- 앞선 독해에서 미해석인 Notion 62개 임베드·외부 첨부 전체를 검증했다는 주장은 하지 않는다.

기존 콘솔 미커밋 4파일의 사용자 변경을 보존해 이번 수정만 추가했다. commit·push는 하지 않았다.
