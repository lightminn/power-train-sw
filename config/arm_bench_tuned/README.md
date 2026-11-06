# 시연 팔 현장 튜닝값

2026-11-06 젯슨 `/session`에서 가져온 값이다. 실제로 쓰는 값은 **젯슨 `/session` 쪽**이고,
이 폴더는 이어서 튜닝하거나 복구할 때 쓰는 사본이다. 튜닝을 마치면 다시 가져와 커밋한다.

| 파일 | 내용 |
|---|---|
| `tool_slots.json` | 슬롯 자세(rad). 슬롯 1 = 굴삭기, 슬롯 2 = 랙앤피니언 그리퍼 |
| `arm_poses.json` | 저장 자세(`home`, `slot1` 등) |
| `pose_speeds.json` | 1~5축 자세 이동 속도(°/s). GUI `축별 속도 적용`이 쓴다 |
| `lock_points.json` | 볼락 잠김·열림 tick |
| `session.yaml` | 브릿지 파라미터(PWM, 허용치, 상승량 등) |

튜닝 절차와 값별 위치는 `docs/arm-bench-operation.md`의 `값 튜닝 이어가기`를 따른다.
`local_arm_bench_teleop.yaml`은 이 값과 같은 기본 설정이다.
