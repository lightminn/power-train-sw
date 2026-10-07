#!/usr/bin/env python3
"""RealSense 실카메라 → perception_node(YOLO seg + PCA yaw) → SRT → 실시간 뷰어,
이 전체 파이프라인을 한 번에 켜고/끄는 호스트용 헬퍼.

호스트(Jetson 로그인 셸, DISPLAY 있는 상태)에서 실행한다 — ROS2 명령 자체는
전부 docker exec 로 컨테이너 안에서 돌리고, 이 스크립트는 그 기동/정지와
로컬 SRT 수신 창(gst-launch)만 오케스트레이션한다.

갱신(2026-07-13): 대회 타겟 커스텀 모델 models/best.pt를 재학습된 segmentation
가중치로 교체(task=segment, 클래스 'box-segmentation' 1개). PCA yaw는 다시 활성화됨
(_fill_markerless_pose에서 마스크 PCA 주축각 → orientation quaternion 대입 복원).

사용:
    python3 scripts/live_yaw_view.py start                                  # 기본: 대회 타겟 박스(seg, yaw 나옴)
    python3 scripts/live_yaw_view.py start --model-path /root/ros2_ws/yolov8n-seg.pt --target-class book  # 다른 seg 모델로 비교 검증
    python3 scripts/live_yaw_view.py stop
"""
import argparse
import subprocess
import time

CONTAINER = "ros2_humble"
MODEL_PATH = "/root/ros2_ws/src/robot_arm_perception/models/best.pt"
ROS_SOURCE = "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash"


def docker_exec_bg(inner_cmd: str, log_path: str) -> None:
    """컨테이너 안에서 백그라운드로 실행 (docker exec -d + 로그 리다이렉트)."""
    subprocess.run(
        ["docker", "exec", "-d", CONTAINER, "bash", "-c",
         f"{ROS_SOURCE} && {inner_cmd} > {log_path} 2>&1"],
        check=True,
    )


def kill_in_container(pattern: str) -> None:
    # 인자를 직접 넘겨 docker exec 가 셸을 거치지 않게 한다 — bash -c 로 감싸면
    # 그 래퍼 프로세스 자신의 커맨드라인에 pattern 문자열이 들어가 자기 자신을
    # 죽이는 사고가 난다 (실측 확인).
    subprocess.run(["docker", "exec", CONTAINER, "pkill", "-f", pattern])


def cmd_start(args: argparse.Namespace) -> None:
    target = args.target_class

    subprocess.run(["docker", "start", CONTAINER], check=True)
    time.sleep(1)

    # classes: YOLO 추론 자체를 이 클래스로 한정 (다른 객체는 검출·마스크·PCA 전부 미실행).
    # pick_classes: /pick_target 선별도 동일 클래스로 맞춤(둘 다 지정 안 하면 detected_objects
    # 는 전체 클래스가 여전히 나옴 — classes 가 진짜 스코프 제한).
    perception_cmd = (
        "ros2 run robot_arm_perception perception_node --ros-args "
        f"-p model_path:={args.model_path} -p camera_mode:=realsense "
        f"-p classes:='{target}' -p pick_classes:='{target}' "
        f"-p pick_min_conf:={args.min_conf} -p conf_threshold:={args.conf} "
        "-p require_depth:=false"
    )
    docker_exec_bg(perception_cmd, "/root/ros2_ws/perception_run.log")

    stream_cmd = (
        "ros2 run robot_arm_perception stream_node --ros-args "
        f"-p port:={args.port} -p fps:={args.fps} -p bitrate_kbps:=3000"
    )
    docker_exec_bg(stream_cmd, "/root/ros2_ws/stream_run.log")

    print(f"[live_yaw_view] perception_node + stream_node 기동 중... "
          f"(model = {args.model_path!r}, target class = {target!r})")
    time.sleep(6)

    recv_cmd = [
        "gst-launch-1.0", "-v",
        "srtsrc", f"uri=srt://127.0.0.1:{args.port}?mode=caller&latency=60",
        "!", "tsdemux", "!", "h264parse", "!", "openh264dec",
        "!", "videoconvert", "!", "autovideosink", "sync=false",
    ]
    print("[live_yaw_view] 수신 창 실행:", " ".join(recv_cmd))
    if args.foreground:
        subprocess.run(recv_cmd)
    else:
        subprocess.Popen(recv_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[live_yaw_view] 백그라운드로 띄웠습니다. 종료: python3 scripts/live_yaw_view.py stop")


def cmd_stop(_args: argparse.Namespace) -> None:
    print("[live_yaw_view] 수신 창 종료...")
    subprocess.run(["pkill", "-f", "srtsrc uri=srt://127.0.0.1"])
    print("[live_yaw_view] 컨테이너 노드 종료...")
    kill_in_container("perception_node")
    kill_in_container("stream_node")
    print("[live_yaw_view] 완료.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)

    p_start = sub.add_parser("start", help="perception_node+stream_node 기동 + 실시간 뷰어 창")
    p_start.add_argument("--model-path", default=MODEL_PATH,
                         help=f"YOLO 모델 경로 (컨테이너 내부 절대경로, 기본: {MODEL_PATH})")
    p_start.add_argument("--target-class", default="red and blue box",
                         help="YOLO 추론·yaw 판독을 이 클래스 하나로 한정 (기본: 대회 타겟 'red and blue box')")
    p_start.add_argument("--conf", type=float, default=0.25, help="YOLO conf_threshold")
    p_start.add_argument("--min-conf", type=float, default=0.25, help="pick_target pick_min_conf")
    p_start.add_argument("--port", type=int, default=5000, help="SRT 포트")
    p_start.add_argument("--fps", type=int, default=15)
    p_start.add_argument("--foreground", action="store_true",
                         help="뷰어 창을 포그라운드로 실행 (창 닫힐 때까지 스크립트가 대기)")
    p_start.set_defaults(func=cmd_start)

    p_stop = sub.add_parser("stop", help="뷰어 창 + 컨테이너 노드 모두 종료")
    p_stop.set_defaults(func=cmd_stop)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
