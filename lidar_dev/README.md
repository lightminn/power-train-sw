# lidar_dev — L515 지형 판정 오프라인 도구

녹화한 L515 깊이로 `terrain_overlay` 판정기(실시간 노드와 같은 코어)를 돌려 보는 도구 모음이다.
실시간 경로는 `ros2 run powertrain_ros terrain_overlay`(UDP :5009 → 운용 콘솔)이며,
설계는 `docs/specs/2026-10-17-l515-terrain-overlay-design.md`를 따른다.
**운용자 시각 보조 전용**이다 — 주행 제어·정지·안전 판정에 쓰지 않는다.

## 의존

- numpy, opencv-python(-headless) — 필수
- open3d — 선택. `ground.py`의 3D 보기와 `view_depth.py`만 쓴다. 없으면 2D 결과만 낸다.
- pyrealsense2 — `record.py`만 쓴다.

## 녹화 형식

```
<폴더>/meta.json         depth_scale_m, frame_period_s, depth_intrinsics, color_intrinsics, depth_to_color
<폴더>/depth/000000.png  raw 비정렬 16비트 깊이 (640×480)
<폴더>/color/000000.jpg  컬러 (1280×720)
```

예전 녹화(`depth_intrinsics`가 `[fx, fy, cx, cy]` 목록)도 읽는다.

## 실행 (저장소 루트에서)

```bash
# 녹화 — ⚠️ 실기 L515 는 Gateway 단일 소유다. 먼저 Gateway 를 멈추거나 벤치 PC 에서 녹화한다.
python -m lidar_dev.record data/obj_50cm

python -m lidar_dev.overlay data/obj_50cm        # out_overlay/obj_50cm/*.png (초록=바닥, 빨강=장애물)
python -m lidar_dev.scan_terrain data            # 폴더별 상태·zone 중앙값·카메라 높이
NO3D=1 python -m lidar_dev.ground data/flat 5    # 한 프레임 분석 + out_flat_5.png
python -m lidar_dev.view_depth data/flat 5       # open3d 3D 보기
```

✅ 기대: `scan_terrain`에서 평지 녹화는 `OK/FRESH`가 대부분이고 카메라 높이가 실제 장착 높이(±2 cm)와 맞는다.
