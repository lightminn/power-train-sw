# D435i 영상·Metadata 실기 감사

2026-07-28 노트북 `192.168.8.206`에서 Jetson `192.168.8.106`의 실제
SRT `:5002`와 UDP `:5003`을 직접 수신해 확인했다.

## 영상 계약

- H.264 constrained-baseline
- 848×480, 30/1 fps
- progressive
- pixel-aspect-ratio 1/1

따라서 송신 영상과 Metadata의 `frame_width=848`,
`frame_height=480`은 일치했다. 콘솔은 `gtksink force-aspect-ratio=true`와
동일한 letterbox offset 계산을 사용한다.

## 실제 Metadata 관측

- `frame_id=camera_color_optical_frame`
- class ID 0
- class name `box-segmentation`
- 관측 예: confidence 0.40~0.61, bbox `[1,0,320,473]`
- 관측 거리 예: 0.639~0.684 m

8초 추가 캡처에서는 검출 56건 중 `is_pick_target=true`가 1건뿐이었다.
`/detected_objects` 콜백 직후 `/pick_target`이 발행되는데 브리지가 이전
프레임 bbox와 완전 일치를 요구한 것이 거리 표시 누락의 직접 원인이었다.
브리지는 같은 class + IoU 0.5 이상, pick freshness 0.75초로 바꿨다.

콘솔은 원본 Metadata 버퍼를 유지하면서 confidence 0.5 미만과 프레임 경계
3개 이상에 잘린 검출을 표시 후보에서 제외한다. 실제 오검출
`[1,0,320,476]`은 이 품질 게이트에 걸리며, 픽 타깃/연속 타깃을 우선한 뒤
표시 후보의 거리만 최근 5개 median + EMA로 안정화한다.

15초/269프레임 추가 관측에서는 거리 포함 검출이 1개였고 0.684 m였다.
해당 구간에서는 3 m outlier를 재현하지 못했다.

## GitHub 송신자 감사와 수정

GitHub `ksp118/extreme-robot` main `3f048765`을 별도 체크아웃해 확인했다.

- 실제 weight: `robot_arm_perception/models/best.pt` (6.5 MB)
- weight 내부 class: ID 0 `box-segmentation`
- 학습 data 경로 기록: `box-segmentation-1/data.yaml`
- 기본 inference confidence: 0.4 → 0.55로 상향
- 기존 거리: 마스크 centroid를 depth로 투영한 뒤 주변 사각 patch median
- 수정 거리: 마스크 5×5 erosion → 최대 25개 color pixel을 depth로 투영
  → 3×3 유효 depth 수집 → 범위 필터 → median/MAD → meter 표준화
- 기존 SRT: 848×480×30, x264 ultrafast, 3000 kbit/s
- 수정 SRT 기본값: 4500 kbit/s (해상도·FPS·SRT 계약 불변)

`box-segmentation`은 실제 송신 모델의 class name이므로 UI에서 임의로
`상자` 등으로 바꾸지 않는다. 경계 3개 이상에 잘린 검출은 거리·픽 작업에
부적합하므로 송신자와 콘솔 양쪽에서 제외한다. 나머지 고신뢰 오인식은
학습 데이터/weight의 정밀도 문제이므로 GUI에서 이름을 바꾸거나 임의
confidence를 만드는 방식으로 숨기지 않는다.

## 검증

- operator_console: 209 passed
- arm console bridge: 28 passed, 2 기존 ROS환경 skip
- robot-arm perception quality: 4 passed
- production GTK runtime smoke: PASS, 31 ticks/4채널, LIVE+STALE,
  자동 카메라 스왑 0, traceback 0
- 새 경계 품질 게이트 negative control: 허용 경계를 2→3으로 풀면
  실측 오검출 회귀 테스트가 의도대로 FAIL

Jetson에는 로봇팔 저장소 변경을 아직 배포하지 않았다. 해당 호스트 SSH
인증이 없어 노트북 체크아웃과 단위 검증까지만 완료했다.
