# 콘솔 조향 조작 실투입 (2026-09-10)

사용자 긴급 실투입 지시에 따라 영점 저장과 기존 스키드 선택 상태 전달 수정을 적용했다.

- `복구 · 설정` → `현재 조향각을 영점으로 저장` → 정렬 확인 → 확인. 차체 IDLE, 구동 6축 IDLE/정지, 조향 4축 최신·저전류·정지 피드백을 요구한다.
- 기존 CAN 소유자가 원점 명령만 전송하고 200 ms 안에 명령 이후 새 0° 피드백을 확인한다. 부분 실패·미확인 축은 실패로 보고한다. 시동·회전·위치 명령은 보내지 않는다.
- `조향 방식 [애커만]` 선택을 활성화했다. 현재는 애커만 그대로이며 실제 스키드 전환은 수행하지 않았다.
- 콘솔/상태 전달 빠른 검사 85 passed, 원점 코어 관련 40 passed. 실제 GTK runtime smoke PASS. Jetson 설치 패키지 재빌드 성공, 변경 소스 8개 SHA-256 로컬과 일치.
- 실제 서비스 `powertrain_chassis`, `powertrain_control` running / restart count 0. 콘솔 PID 312096, 패드·입력 연결, IDLE·ESTOP 해제, 영점 버튼 활성화 및 애커만 상태 수신 확인.
- 재기동으로 초기화된 US-100·로봇팔 미사용 설정을 기존 운용 값으로 복원했다. 자동 시동하지 않았다.

새 버튼으로 실물 원점을 쓰는 시험, 스키드 실주행, 전원사이클 영점 유지 검증은 이번 실투입에서 수행하지 않았다. 앞서 사용자 요청으로 별도 원점 명령을 보냈을 때 4축 0° 수신은 확인했다. 스키드 전용 설치 GTK fixture 음성 대조는 timeout이 남아 있으며 통과로 산입하지 않는다. 사용자가 전체 시험보다 즉시 적용을 요청하여 긴 시험은 추가 진행하지 않았다.

증거: `/tmp/powertrain-console-skid-20260910/`의 deploy.log, deploy-sha.json, deploy-idle.json, quick-panel.log, zero-runtime-smoke.log. 배포 전 소스 백업: Jetson `/home/zetin/powertrain-can-evidence-20260910/console-steering-before`. 운용 체크아웃은 `/home/zetin/power-train-sw-integrated`, 팀원 체크아웃은 보존했다. 변경은 미커밋·미푸시 상태다.

## 최초 실사용 피드백 수정

사용자가 영점 버튼을 눌렀을 때 `transition_or_recovery_pending`으로 거부됐다. CAN 복구 상태는 IDLE였으며, `command_recovery`(새 주행 입력 요구)를 원점 저장 금지 조건에 포함한 결함이었다. 이 hold는 IDLE에서 남고 새로운 ARMED 입력에서만 해제되므로 정지 상태 영점 등록을 막으면 안 된다. 해당 조건만 제외하고 hold 자체 및 실제 CAN 복구·전환·모터 상태 검사는 유지했다. 재현을 포함한 원점 테스트 15 passed 후 차체 소스에 적용하고 차체만 재기동했다. 콘솔 세션은 유지했다. 증거: `/tmp/powertrain-console-skid-20260910/origin-hold-fix.json`.

## rear-left 미동작 제보 수동 수신 기록

사용자는 다른 3개 조향 바퀴만 움직인다고 제보했다. 서비스·지령을 변경하지 않고 CAN을 수신했다. 처음 IDLE 20초에서는 4축 각각 약 50 Hz 상태 응답, 오류 0, 위치 지령 0개였다. 이후 사용자 좌우 조향 15초 구간에서는 노드 1~4 모두 약 50 Hz 위치 지령과 상태 응답을 관측했다. 노드 3(rear-left 설정)의 지령은 −21.0016~+34.0691°, 모터 위치 피드백은 −21~+25°, 전류 최고 1.19 A, 오류 0이었다. 네 축 모두 속도 4500 ERPM·가속도 20000 ERPM/s²를 받았고, 관측 구간의 조향 버스에는 별도 RPM 정지/원점 명령이 없었다.

이는 노드 3에 조향 지령이 전달되지 않는 현상을 이번 재현에서 확인하지 못했다는 뜻이다. 실물 바퀴가 실제로 움직였다는 증거로 확대하지 않는다. 실물 정지가 계속되는지는 사용자 확인이 필요하며, 계속된다면 ID-실물 대응과 모터-바퀴 기계 연결을 구분해야 한다. 원자료: `/tmp/powertrain-console-skid-20260910/rear-left-active-frames.json`, 요약: `rear-left-active-summary.json`.
