# MKS 두 번째 보드 CAN 패치 적용

2026-09-09 밤 USB가 교체된 보드를 확인하고 2026-09-10 새벽 적용을 마쳤다.
대상은 **serial `337733643235`, node15/16, MKS v3.6-56V**다. 앞서 node13/14에 적용한
동일 바이너리로 **0.5.1 unreleased → 0.5.1-dev, `can.reliability_patch=1`**을 확인했다.
원본 저장 설정307개와 캘리브레이션은 유지됐다. node11/12는 아직 패치하지 않았다.

## 원본과 호환성

| 항목 | 확인 결과 |
|---|---|
| 적용 BIN | 247880 bytes, SHA256 `7567809465b9646d39b1f9595f8a8e037ed758344f90269ba49038f8c6bfc710` |
| 이 보드 원본 flash1MiB | SHA256 `7b5640d6b73d1a345655edb5f3fd271ca97f37fd57dccd77d01d9bab57be7ee7` |
| 원본 application768KiB | SHA256 `fcc331b60c2f704119115fb6a3e2e5be2287822ec7e48a7eeec97cd8e6644056`, 이전 보드 원본 application과 동일 |
| 원본 NVM256KiB | SHA256 `ab01ebabde5eb5e0a6de7e610816ff6fea7c65247eec763036466e095741688d` |
| 원본 보존 | ROM DFU에서 전체1MiB 두 번 읽기 일치, 로컬 회수 후 SHA 재대조 |
| 적용 전제 | 양축 IDLE·입력0·오류0·calibrated/ready·양쪽 pre_calibrated·모든 startup_* false |

`powertrain_session`, `powertrain_control`, `powertrain_chassis`, `powertrain_canwatchdog`를
정지하고 공유 motor_session 소유권 잠금 안에서 작업했다. Jetson 유지보수 컨테이너는
`powertrain-sw:jetson`, privileged, host network, `/dev:/dev`, `/run/powertrain` 공유를 사용했다.
`powertrain-sw:ros`는 odrive SDK가 없으므로 사용하지 않았다.

처음 DFU 전환 때 `/dev` bind가 빠진 컨테이너는 새 USB 장치를 보지 못했다. 호스트의
동일 serial·동일 물리 포트 `1-2.1`에서 ROM 장치를 확인하고 bind를 추가한 컨테이너로
원본 application에 돌아왔다. 이 단계에서는 flash 쓰기·지우기를 하지 않았다.

원본 application 재부팅 전후 RAM 설정은 양축 `controller.config.input_mode`만1→2로
달랐고 나머지305개는 같았다. 운용 arm은 RAM PASSTHROUGH(1)를 설정하고 저장 정본은
VEL_RAMP(2)이므로 재부팅에 따른 정상 복원이다. 최초 RAM 백업을 그대로 보존하고,
원본 재부팅 후307개를 패치 전후 비교 기준으로 별도 저장했다. 저장 설정을1로 덮어쓰지 않았다.
근거: `drive_odrive_can.py`의 arm 및 `bl70200_setup.py`의 VEL_RAMP 설정.

## 실행 검증

| 항목 | 결과 |
|---|---|
| guarded DFU 호스트 회귀 | **32 passed** |
| 최종 wrapper | serial·node·원본 SHA 고정, 기존 승인 후보 SHA 유지, 미고정 SHA의 쓰기 전 차단 검토 |
| 쓰기 직전 | 현재 flash1MiB가 해당 보드 원본 백업과 일치 |
| 쓰기 범위 | application sector0–9만 교체, NVM sector10–11 보존 |
| ROM 읽기 대조 | application768KiB 전체와 NVM256KiB 전체 바이트 검증 통과 |
| 부팅 후 USB | patch1,307설정 동일, 양축 IDLE/error0, calibrated/ready/pre_calibrated 유지 |
| 무송신 CAN 관측30초 | 구동6축+조향4축 모두 수신, 각1500–1501회, 구동6축 마지막 상태 IDLE/error0 |
| 패치 보드 최종 상태 | 양축 IDLE/error0, recovery latch false, velocity0 |
| CAN 진단 | 관측 시작 HAL오류·TX오류·TXdrop·복구횟수0; 종료 HAL오류0, 복구횟수0 유지 |

호스트 회귀와 wrapper/NVM 경계를 독립 검토했다. 검토 중 발견한 비교 기준 파일 참조를
원본 재부팅 후 기준으로 바로잡았고, 마지막 원본 SHA 고정 diff도 직접 대조했다.
캘리브레이션·설정 저장·게인/전류 변경·무장·모터 이동 시험은 하지 않았다.
이 보드의 watchdog/CAN 오류 주입 시험, 물리 전원 재인가 후 인수와 실주행은 별도다.
이전 보드의 시험 결과를 이 보드의 실측 결과로 대신하지 않는다.

Jetson 시계가1970년이므로 원시 로그 UTC를 실제 작업 날짜로 해석하지 않는다.
관측 시간은 monotonic 기준이다. 원본 compressed flash, 설정 두 상태, DFU 기록, 실행 helper와
결과는 [증거 폴더](2026-09-10-mks-second-board-evidence/)에 보존했다.
패치 원리·빌드 재현은 [기존 보고서](2026-09-09-mks-can-reliability.md)와
[펌웨어 문서](../../firmware/mks_odrive/README.md)를 따른다.
