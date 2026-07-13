# 3대 계획서 멀티모델 최종 타당성 검증 (2026-07-14)

> 대상: `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`,
> `docs/plans/2026-07-13-wp5.2-arm-collaboration-safety-plan.md`,
> `docs/plans/2026-07-13-observability-data-quality-remote-assist-plan.md`
> (기준 HEAD `36902ef` + 07-13 문구 수정 2건 반영 작업본)
>
> 방법: 서로 다른 검증 렌즈를 부여한 7개 모델의 독립 리뷰 → 모든 주장을 레포 원문·실행
> 테스트로 교차 판정. 외부 모델은 레포 접근이 없으므로(코덱스 제외) 코드 관련 주장은 전부
> 재검증했고, 기각된 주장도 근거와 함께 기록한다.

| 리뷰어 | 렌즈 | 결과 |
|---|---|---|
| Gemini 3.1 Pro (High) | WP5.2 인라인 참조 코드·계약 로직 | 4건 → 유효 3, 기각 1 |
| Gemini 3.5 Flash (High) | 관측성 아키텍처·소켓·profile FSM | 8건 → 유효 2, 기각/기채택 6 |
| Gemini 3.5 Flash (High) | 자율주행 배점·일정·내부모순 (Opus 4.6 타임아웃 대체) | 5건 → 유효 3, 기각 2 |
| Claude Sonnet 4.6 (Thinking) | 3개 FSM 전수 추적 | 8건 → 유효 6, 기채택 2 |
| GPT-OSS 120B | 로봇팔팀 입장 적대 리뷰 | 6건 → 협상 예측 자료 채택 |
| Codex gpt-5.6 (레포 접근) | 구현자 관점 implementability | 10건 → 전건 파일:라인 인용, 스팟체크 5/5 정확 |
| Claude Fable 5 (본 세션) | 참조 코드 실행 검증 + 주장 판정 | ArmInterlock 43/43 PASS, 커널 실측 1건, 기각 판정 다수 |

## 0. 종합 판정

**세 계획 모두 구조적으로 타당 — "조건부 승인" 유지. 생존한 S0 주장 0건.**
외부 모델들이 S0로 제기한 4건은 전부 실행 테스트·커널 실측·계획 원문 대조로 기각되거나
S1로 강등됐다. WP5.2 Task 1은 즉시 착수 가능하다(Codex 판정과 본 세션의 참조 코드
43/43 실행 검증이 일치).

단, 복수 모델이 독립적으로 수렴한 **반복 패턴 1개**가 확인됐다:
**"fail-closed 기본값에 명시적 탈출구가 없다"** — 안전 방향으로는 흠이 없으나, 아래
A-2(핸드오버 갇힘)·A-4(단독 주행 프로파일 부재)처럼 가용성·일정 리스크로 나타난다.
착수 전 A그룹 문구 수정을 권고한다.

## A. 설계 결함 — 계획 문구 수정 필요 (S1)

1. **`GRIP_LOST_HOLD` 이탈 전이 미명세** (Sonnet · WP5.2 Task 5)
   상위 자율주행 계획에는 이탈 조건("bounded regrasp 성공 또는 운영자 확인·승인")이 있으나
   실행 계획의 FSM 상태 목록에는 진입만 있고 이탈 전이가 없다. 구현자가 누락하면 20분
   제한 내 회복 불가 영구 hold.
   → Task 5 Step 3에 이탈 경로(운영자 승인 + `clear_grip_lost(authorized=True)` + 작업
   종류에 맞는 fresh locked 확인) 명시.

2. **`qualified=false` 개발 기본값에서 DRIVE↔ARM 핸드오버 갇힘** (Sonnet · WP5.2 Task 4)
   개발 기본 `wheel_stop.yaml`(qualified=false)에서 ARM 전환 요청 시
   `STOPPING_FOR_HANDOVER`/`STOPPING_FOR_ARM`이 영구 대기 — "명시적으로 실패하게 한다"의
   실패 형태(즉시 거부 vs 대기)가 미정의.
   → unqualified이면 전환 요청을 **즉시 거부하고 현재 상태 유지 + 운영자 통보**로 명세.
   두 STOPPING 상태에 timeout→MOTION_HOLD 전이 추가.

3. **`CARRYING_LOCKED`의 qualified-grip 요구가 문서 간 모순** (Sonnet + 본 세션 대조)
   WP5.2: "CARRYING_LOCKED는 qualified grip까지 필요" / 자율주행 §4.2: "자세 상태이며,
   파지 검출기가 qualification을 통과한 경우에만 파지 정상까지 의미". 그리퍼 파라미터가
   전부 placeholder인 현재, WP5.2 문구대로면 운반 미션 자체가 영구 불가(ARRIVED_DROP
   수락 조건도 연쇄 차단).
   → 자율주행 계획의 정의(자세 잠금 = 발행 조건, grip 의미는 detector qualified 시에만
   추가)로 WP5.2를 정렬. unqualified 시 보수적 운반 profile + 하역 전 재확인은 유지.

4. **파워트레인 단독 실차 주행 프로파일 부재** (Flash-자율주행, 레포 대조로 확정)
   WP5.2 Task 2 구현 즉시 팔 게이트는 무조건부(`fresh STOWED_LOCKED 없으면 drive 0`)가
   되는데, 팔 팀 v2 배포 전까지 실 heartbeat는 존재하지 않는다. mock은 계약시험 맥락에만
   정의돼 있어 **오도메트리·terrain·원격 리허설 등 모든 실차 주행 HIL이 팔 팀 일정에
   종속**된다. (배점 관점: 팔 협업 자체는 피아식별 20 + 물자 5 수준인데 전체 350점 검증
   경로를 차단.)
   → `arm_gate_mode: arm_absent_field` 프로파일을 명시 정의: 팔 노드가 그래프에 부재 +
   운영자가 기계적 접힘을 육안 확인(기존 wheels-up 확인 절차와 동급) + journal 기록
   조건에서만 mock locked heartbeat 허용. `MISSION_STOP`/`ArrivalStatus` 발행은 금지 유지.

5. **arm hold 해제 시 저장된 주행 명령 재생 가능** (Codex · `chassis_manager.py:151,280` 확인)
   `set()`은 `_v/_omega`를 보존하고 hold는 출력만 0으로 게이팅 — 계획 Task 2 Step 1의
   회귀시험("새 /cmd_vel 없이는 재개 금지")을 Step 3의 최소 API로는 통과할 수 없다.
   → hold 진입 시 pending 명령 폐기(또는 해제 후 fresh `set()` 요구)를 Step 3에 명시.

6. **관측성 계획 자기모순: 수정 금지 경계 vs Task 6 Files** (Codex)
   전역 제약은 command authority를 WP5.2 단일 소유·수정 금지로 선언하는데 Task 6이
   `command_authority.py`를 Modify 목록에 둔다.
   → Task 6의 수정 범위를 "WP5.2가 제공하는 remote-assist hook 연결에 한정"으로 명시
   재배정.

7. **L515 depth/overlay 동시 제공 vs 현행 단일 스트리머 구조 충돌** (Codex · `streamer.py:36,107` 확인)
   NORMAL profile은 raw RGB 유지 + operator-selected depth/overlay를 약속하지만 현행
   L515는 GStreamer 프로세스 1개·mode 1개 소유(`set_mode()`가 출력을 교체).
   → 별도 companion 포트/인코더 채널을 계약에 정의하거나 depth/overlay 원격 동시 전송
   요구를 제거.

8. **관측성 Task 3 health 필드가 현행 드라이버 API로 생성 불가** (Codex · `steer_ak40.py:76`, `drive_odrive_can.py:174` 확인)
   요구 필드(feedback age/rate, recovery count, axis state)를 현행 `state()`가 노출하지
   않는데 두 드라이버가 Files에 없다.
   → `corner_module/steer_ak40.py`, `drive_odrive_can.py` + 테스트를 Modify 목록에 추가.

9. **팔 stamp 도메인 단독 롤백 시 회복 불가 hold** (Gemini Pro, 본 세션에서 조건 축소)
   로컬 clock은 정상인데 팔 쪽 stamp만 큰 폭 과거로 점프하면(`use_sim_time` 불일치 등
   도메인 분리 사고) `_last_seen_stamp_s` 단조 가드에 걸려 새 heartbeat가 영원히 거부된다.
   같은 Jetson 단일 clock 전제라 발생 조건은 좁고 방향은 fail-safe(hold 유지)이므로
   자동 수용(리뷰어 제안)은 채택하지 않는다.
   → "큰 폭 역행 stamp 연속 수신은 `CONTRACT_VIOLATION`으로 기록하고 hold 유지, 복구는
   운영자 개입"을 명세(자동 도메인 리셋 금지).

## B. 기계적 수정 — verify 명령·파일 목록 (S1~S2)

10. `package.xml`에 `control_msgs` 의존 없음 + WP5.2 Task 4 Files에 `package.xml` 누락 (Codex, 원문 확인).
11. node 통합 verify가 `--entrypoint bash`로 colcon build를 우회 — `chassis_node`는 generated `powertrain_msgs`/`robot_arm_msgs` import가 필요. verify에 `colcon build --packages-select … && source ros2/install/setup.bash` 추가 (Codex, entrypoint 원문 확인).
12. verify 셸 체인에서 pytest가 마지막 명령이 아닌 곳은 `;`가 실패를 은폐(관측성 Task 7 등) — `&&` 또는 `set -e`로 교체 (Codex, 원문 확인).
13. `motor_control/drive/x2212_test/odrive_can_drive.py`가 실 can0 opener인데 Task 3 적용 대상 누락 — Files 추가 또는 legacy 디렉토리 allowlist 명시 (Codex, `:101` 확인).
14. Interfaces의 `RealCanSession`(context/lifecycle) vs 스니펫 `CanOwnerLock`(context 아님, owner metadata 없음) 이름·API 불일치 — Task 3에서 단일 이름 + context API + immutable owner snapshot(관측성 Task 3 소비용) 확정 (Codex).
15. `powertrain_control` compose service는 `command`만으론 시작 불가 — 현행 entrypoint가 무조건 L515 Gateway를 exec. `entrypoint:` override를 계획에 고정 (Codex, entrypoint 원문 확인).

## C. 참조 코드·표시 개선 (S2~S3)

16. **조회 메서드의 상태 변형 부작용** (본 세션 실행 검증 + Sonnet 독립 수렴): `fresh()`가 매 호출 `_last_now_s`를 갱신·rollback 처리 — 서로 다른 시계로 질의만 해도 유효 샘플이 무효화됨(fail-safe 방향의 가용성 결함). → 단일 단조 clock + tick 내 호출 순서(`update→drive_allowed→hold_reason`) 강제, 또는 `fresh()`의 상태 갱신 제거.
17. 미인식 status가 stamp를 소모(검증 순서) — status 검증을 `_last_seen_stamp_s` 전진보다 앞으로 (Gemini Pro).
18. `CanOwnerLock.acquire()`: `flock`이 `BlockingIOError` 외 `OSError`(EINTR/ENOLCK)를 던지면 fd 누수 — try/finally (Gemini Pro).
19. 원격 overlay: metadata 수신 TTL만으로는 추론 지연 시 위치 오정렬 표시 가능 — metadata의 capture stamp를 표시 프레임 stamp와 매칭해 초과 시 overlay 숨김 (Flash·관측성). overlay는 비권위 보조 정보라 안전 영향은 경계 안.
20. 1문장짜리 명세 보강 3건 (Sonnet): `STOW_VERIFY` 진입 트리거(`DONE` 수신 시), `FAILED_HOLD` 진입 시 작업 유형(PICKUP/DROP) latch, `CommandAuthority`↔`RemoteInputGateway`의 동일 tick 내 평가 순서.

## D. 기각된 주장 (증거 포함 — 재론 방지용)

| 주장 (리뷰어) | 기각 근거 |
|---|---|
| [S0] `fresh()` 음수 age가 정상 미래 stamp 차단 (Gemini Pro) | 계획의 freshness 정의 `0 ≤ now-stamp ≤ timeout`을 그대로 구현한 것. 본 세션 실행 테스트에서 의도 동작 확인 |
| [S0] journal 소켓 포화가 50 Hz 블로킹 (Flash·관측성) | 계획이 이미 "비차단 이벤트" + "producer 비블로킹 시험"을 명세 |
| [S1] SCM_CREDENTIALS는 송신측 bind/SO_PASSCRED 필요 (Flash·관측성) | **커널 실측 기각**: unbound·무옵션 송신자(fork 자식 포함)의 pid/uid가 수신측 SO_PASSCRED만으로 첨부됨 |
| [S0] 10 Hz heartbeat vs 50 Hz 루프 채터링 (Flash·자율주행) | 계획이 정확히 stamp-age 기반 비동기 판정을 명세 — 허수아비 |
| [S0] STOWED_LOCKED가 팔 FSM에 없어 콘솔 영구 대기 (Flash·관측성) | 알려진 크로스팀 산출물(계약 adapter)이며 Task 7 게이트 뒤에만 활성 — 신규 결함 아님 |
| joint_4/5 vs 3축 bridge (Flash 양쪽) | 계획이 이미 5축을 팔 팀 산출물 + ARM enable 기본 false로 게이트 |
| QoS Reliable 부담 → BestEffort 요구 (GPT-OSS) | 단일 Jetson 로컬 DDS — 재전송 비용 논거 성립 안 함 |
| 동적 비트레이트로 전환 (Flash·관측성) | 정적 사전 HIL profile은 의도된 안전 설계 |

## E. 로봇팔팀 협상 예측 (GPT-OSS 적대 리뷰 → 회의 준비 자료)

- 최대 반발 예상 2건: ① **10 Hz 상시 heartbeat**(현행: 상태 변화 시만 발행 — 주기 타이머
  발행자 추가 작업), ② **locked heartbeat 조건 전부 AND**(torque hold 미구현). 대응:
  로컬 DDS라 부하 논거는 기각 가능하나 구현 작업량은 실재 — pose+velocity+dwell 우선,
  torque hold 후속의 **단계 도입안**을 준비하되 조건 AND 의미는 유지(OR 완화는 거부).
- "바퀴 정지 판정 기준" 질문 예상 → 정지 판정은 파워트레인 단독 소유, 팔은 `MISSION_STOP`
  수신만 신뢰하면 됨을 선제 명시.
- GPT-OSS 판정: "조건부 서명" — A-3 정렬(grip 의미 조건부)이 반영되면 서명 장벽이 크게 낮아짐.

## F. 실행 검증 산출물

- `ArmInterlock` 참조 코드 경계 테스트 43/43 PASS (동일/역행/미래/NaN stamp, rollback
  2경로, latch, override 4경계, mission ACK, 무작위 3000회 `drive_allowed⇔hold_reason`
  일관성): scratchpad `mmv/exec_check/boundary_test.py`
- SO_PASSCRED/SCM_CREDENTIALS 커널 실측: `mmv/exec_check/scm_creds_test.py`
- 리뷰어 원문: scratchpad `mmv/r1.out`(Gemini Pro), `r2b.out`(Flash·관측성),
  `r3b.out`(Flash·자율주행), `r4.out`(GPT-OSS), `r5.out`(Sonnet), `r6.out`(Codex)

## G. 결론

- S0 없음. **WP5.2 Task 1 즉시 착수 가능** (Codex + 실행 검증 일치).
- 착수 전 A그룹 9건 + B그룹 6건의 계획 문구 수정 권고 — 전부 1~5문장 수준이며 아키텍처
  변경 없음. C그룹은 구현(TDD) 시점에 반영해도 됨.
- 크로스팀 최우선은 변함없이 **이번 주 enum 합의에 v2 어휘 전체 상정** + A-3(grip 의미
  조건부)와 A-4(단독 주행 프로파일)를 합의 안건에 포함.
