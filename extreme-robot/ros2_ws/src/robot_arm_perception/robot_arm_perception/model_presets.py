"""미션 구간별 YOLO 모델 preset (perception_node 전용).

대회 5개 구간(미션1 대상 식별/미션2 박스 운반/미션3 마커 식별/미션4 노면 주행/미션5 선도차 추종)이
하나의 연속 주행 안에서 순차 진행되지만, 비전(카메라+YOLO 추론)은 이 노드 하나로 통합돼
있다. 구간이 바뀔 때 model_path/classes/pick_classes를 한 번에 맞추도록 묶어 두면
perception_node 재시작 시 model_name 하나만 바꿔서 launch할 수 있다. 새 구간용 모델을
추가할 때는 이 dict에 항목만 추가하면 됨(gripper_presets.py와 동일한 패턴).

`task`는 YOLO(model, task=...) 생성자에 그대로 전달된다(perception_node 참고) — 2026-07-22
TensorRT 백엔드(backend='trt') 실측으로 발견: .engine 파일은 task 메타데이터를 보존하지
않아 ultralytics가 자동으로 'detect'로 잘못 추정하고, seg 모델(box)이면 이때 r0.masks가
조용히 None이 돼 markerless pose(translation/PCA orientation)가 깨진다(에러 없이 그냥
빈 값). task를 preset에 박아 명시적으로 넘기면 .pt/.engine 둘 다 안전하다 — 새 preset
추가 시 반드시 이 필드도 채울 것.
"""

MODEL_PRESETS = {
    "box": {
        # 미션2 박스 운반 — ARM_GRASP_BOX. seg 모델(1클래스 box-segmentation)
        # → markerless pose(translation+PCA yaw) 전체 활성.
        "model_path": "src/robot_arm_perception/models/best.pt",
        "classes": "box-segmentation",
        "pick_classes": "box-segmentation",  # 팔이 실제로 집는 유일한 구간
        "task": "segment",
    },
    "traffic_light": {
        # 구간1 등 주행 중 신호등 판단 — 파워트레인 소유 로직이 /detected_objects로 소비.
        # detect 전용 모델(마스크 없음) → translation은 bbox 중심 depth 폴백,
        # orientation은 채워지지 않음. 관찰 전용(집지 않음) → pick_classes 비워서
        # /pick_target 후보에서 자동 제외.
        "model_path": "src/robot_arm_perception/models/traffic_light_best.pt",
        "classes": "green light,red light",
        "pick_classes": "",
        "task": "detect",
    },
    "iff": {
        # 미션1 대상 식별(대상 분류) — 파워트레인 소유 로직이 /detected_objects로 소비.
        # detect 전용 모델(마스크 없음) → translation은 bbox 중심 depth 폴백,
        # orientation은 채워지지 않음. 관찰 전용(집지 않음) → pick_classes 비워서
        # /pick_target 후보에서 자동 제외.
        "model_path": "src/robot_arm_perception/models/iff_best.pt",
        "classes": "Enemy uniform,ally uniform",
        "pick_classes": "",
        "task": "detect",
    },
    "vision_marker": {
        # 미션3 마커 식별 — 비전마커 식별, 자세 락만(집지 않음). 파워트레인 소유
        # 로직이 /detected_objects로 소비. detect 전용 모델(마스크 없음) →
        # translation은 bbox 중심 depth 폴백, orientation은 채워지지 않음. 관찰 전용 →
        # pick_classes 비워서 /pick_target 후보에서 자동 제외.
        # 클래스 8종(모델 실측, 2026-07-22): E/K/M/O/R/Y/a/heart.
        # ⚠️ 규정상 "5개 식별"과 개수가 안 맞아 보이지만 모순 아님(2026-07-24 규정
        # 원문 확인) — "5개"는 한 미션당 트랙에 놓이는 마커 개체 수고, 심볼 종류
        # 전체 가짓수가 아님. 당일 어떤 5개가 나올지 몰라 8종을 넓게 학습해둔 것.
        # classes 좁히지 말 것.
        "model_path": "src/robot_arm_perception/models/vision_marker_best.pt",
        "classes": "E,K,M,O,R,Y,a,heart",
        "pick_classes": "",
        "task": "detect",
    },
    "winter_terrain": {
        # 미션4 노면 주행(빙판/제설) — 자세 락만(집지 않음). ⚠️ TODO: 이 구간이
        # 실제로 YOLO 인식이 필요한 구간인지 자체가 파워트레인과 미확정(지형/주행
        # 문제라 비전 모델이 아예 불필요할 수도 있음) — 확인 후 불필요하면 이 항목
        # 통째로 삭제. 모델 파일(.pt) 도착 전 placeholder:
        # models/winter_terrain_best.pt에 배치 + classes를 실제 클래스명으로 교체할 것.
        "model_path": "src/robot_arm_perception/models/winter_terrain_best.pt",
        "classes": "",  # TODO: 모델 도착 후 확정 (빈값=전체 클래스 통과)
        "pick_classes": "",  # 관찰 전용 예상 — 집지 않음
        "task": "detect",  # TODO: 모델이 seg면 "segment"로 교체
    },
    "lead_robot": {
        # 미션5 선도차 추종 — 선도차 PID 추종, 자세 락만(집지 않음). 파워트레인 소유
        # 로직(PID 추종)이 /detected_objects로 소비하는 관찰 전용 패턴(traffic_light/iff/
        # vision_marker와 동일). 모델 파일(.pt) 도착 전 placeholder:
        # models/lead_robot_best.pt에 배치 + classes를 실제 클래스명으로 교체할 것.
        "model_path": "src/robot_arm_perception/models/lead_robot_best.pt",
        "classes": "",  # TODO: 모델 도착 후 확정 (빈값=전체 클래스 통과)
        "pick_classes": "",  # 관찰 전용 — 집지 않음
        "task": "detect",  # TODO: 모델이 seg면 "segment"로 교체
    },
}

DEFAULT_MODEL = "box"


def get_preset(model_name, logger=None):
    preset = MODEL_PRESETS.get(model_name)
    if preset is None:
        if logger is not None:
            logger.warn(
                f"Unknown model_name '{model_name}', falling back to '{DEFAULT_MODEL}'"
            )
        preset = MODEL_PRESETS[DEFAULT_MODEL]
    return preset
