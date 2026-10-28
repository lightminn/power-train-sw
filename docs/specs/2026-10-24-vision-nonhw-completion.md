# 비전 탭 HW 제외 잔여 구현

원본 보호 범위는 그대로 유지한다: `terrain_map/`, `operator_console/terrain.py`,
`terrain_map.py`, `terrain_map_panel.py`, 승인된 `mobility_view.py`는 수정하지 않는다.
수신 소켓·디코더·SDK 소유권과 Gateway의 센서 경로를 추가하지 않는다.

## 바닥 관측 격자

기존 `terrain_overlay` 검출기의 `points`, `valid`, `floor_mask`를 기존 Depth→Color projector로 투영한다.
RGB 1280×720 기준 10×10 px bin(128×72)마다 유효 투영 표본이 2개 이상이고
모든 표본이 바닥으로 분류된 경우만 1이다. 미관측/혼합 bin은 0(투명)이다.
구멍을 채우거나 hull을 만들어 미관측 영역을 바닥으로 추정하지 않는다.
이는 거친 바닥 관측 표시이며 통과 가능/안전 영역이 아니다. 기존 정합 미검증 문구를 유지한다.

`:5009` v1 JSON에 선택 필드 `floor_mask`만 추가한다.
`encoding=zlib-base64-msb-bitset`, `width=128`, `height=72`, `data=<base64>`이며,
행 우선·각 바이트 MSB부터 9216 bit(1152 byte)다. 압축 해제는 1153 byte로 상한을 걸고
정확한 길이·종료·잔여 스트림을 검사한다. `status=OK`에서만 허용한다.
1200 byte 초과 시 **바닥 필드를 먼저 생략**해 기존 장애물/거리 정보를 보존한다.
기존 패킷과 구형 수신기의 unknown-field 동작은 호환된다.

송신 ROS 노드의 `publish_floor_mask` 기본값은 true이며 false로 기존 송신 형식을 선택할 수 있다.
검출/분류 알고리즘이나 Gateway는 수정하지 않는다. 실제 Jetson에 새 송신기를 배포하지는 않았다.
공유 TerrainCanvas가 수신기의 같은 delay-aligned snapshot을 읽고 기존 video/link/depth/size 조건으로 숨긴다.
9KiB alpha surface 하나를 재사용하며 바닥 누락·stale·영상 끊김에는 그리지 않는다.
기존 오버레이 토글이 바닥과 장애물을 같이 제어한다.

## 명시적 자세 보정 설정

`--attitude-calibration PATH`, 통합 JSON `attitude_calibration`,
환경 변수 `L515_ATTITUDE_CALIBRATION` 순으로 선택한다(CLI>JSON>env>기본 비활성).
통합 JSON 상대 경로는 통합 설정 파일 폴더 기준이다. 파일 교체 후 재시작한다.
기본 템플릿은 `operator_console/config/attitude-calibration.pending.json`이고,
실측 각도나 항등 회전을 기본값으로 넣지 않는다.

활성 JSON에는 `schema_version=1`, `confirmed=true`, `confirmed_by`, `reference`,
`source_basis=udp5005_imu_filtered_euler_xyz`, `target_frame=base_link`,
`sensor_to_body_xyzw`(단위 quaternion), `raw_roll_sign`/`raw_pitch_sign`(+1/-1)이 필요하다.
이 회전은 원시 IMU 기준 벡터를 차체 기준으로 옮기며 합의한 장착 회전·영점을 표현한다.
특정 장착각을 코드에서 차감하지 않는다. 실제 확인자/기준값은 아직 전달되지 않았다.

원시 fixed-XYZ Euler에서 up=(-sin pitch, sin roll cos pitch, cos roll cos pitch)를 만들고
확인된 회전으로 body up을 구한 뒤 Euler Roll/Pitch를 계산한다.
원시값은 별도 표시하며 stale/결측/범위 밖 원시값을 보정 자세로 표시하지 않는다.
협조구동 탭의 원시값·코드는 그대로 둔다.

지도 비교는 `map_frame_confirmed=true`, `map_reference`까지 명시한 경우만 켠다.
현재 gravity가 존재하고 지도/telemetry가 live이며 두 수신 기준 age가 각각 0.2초 이하여야 한다.
지도와 동일한 축 고도각(asin up.x/up.y)으로 비교해 Euler 쌍과 직접 빼지 않는다.
현재 프로토콜에는 양쪽 센서의 공통 측정 시각이 없으므로 **정지 참고 차이·수신 시각 기준·센서 시각 미확인**으로 표시한다.
주행 중 gravity가 사라지면 과거 정지 기록은 남기되 차이 계산은 중지한다.
보정/지도 좌표 합의와 물리 시각 정합은 이 인터페이스 구현으로 확인된 것으로 간주하지 않는다.

## 안정성 검증 도구

성능 하니스의 beat/age/RSS/render 값은 온라인 집계하고 timeline/environment 보관 수를 제한한다.
전체 원시 표본은 `.samples.jsonl`로 순차 저장해 600초 이후에도 Python 목록이 계속 커지지 않게 한다.
RSS 표본과 선택 tracemalloc 기록으로 최초 렌더의 할당/캐시와 지속 증가를 구분한다.
합성 SRT 송신은 독립 gst-launch 프로세스, 데이터는 독립 Python 프로세스로 만들고 listener 준비를 확인한다.

실제 콘솔 관찰 도구는 기존 이벤트만 읽으며 backend·창 mapped/visible/state 변경과
0.5초 초과 heartbeat/draw 간격 시 sink age·frame counter를 최근 64건 기록한다.
창 숨김도 인수 시험에서 실패로 검출한다. 과거 로그에 없는 노출 상태를 소급 추정해
8.20초 기록의 원인을 확정하지 않는다. 실제 센서/네트워크 검증은 이번 범위에서 제외한다.
