"""카메라 캘리브 안내문을 RViz 옆에 크게 띄우는 창 (2026-08-13 추가).

## 왜 별도 창인가

`camera_tf_tuner` 의 안내는 지금까지 **RViz 3D 뷰에 떠다니는 텍스트 마커** 하나뿐이었다.
그런데 그 마커는:

- **작다.** 크기가 월드 좌표(`scale.z=0.035`)라 카메라를 줌아웃하면 같이 작아진다.
- **가려지고 뒤집힌다.** 포인트클라우드/로봇 모델 뒤로 들어가고, 시점에 따라 겹친다.
- **한글을 못 쓴다.** RViz(Ogre) 기본 폰트에 한글 글리프가 없어 전부 두부(□)로 나온다 —
  그래서 마커 문구가 전부 영어이고, 그게 "글자가 떠 있는데 뭘 하라는 건지 모르겠다" 의
  직접적인 원인이다.

이 노드는 같은 내용의 **한국어 판**(`/perception/calib_guide`, `std_msgs/String`)을 받아
Tk 창에 큰 글씨로 그린다. 현재 단계·경고·"박스가 지금 보이나" 가 색으로 구분된다.
읽기 전용이라 캘리브 동작에는 관여하지 않는다(발행 토픽 0개).

여기에 더해 **현재 카메라 자세를 TF 에서 직접 읽어** 맨 위에 띄운다 — 드래그하는 동안
숫자가 실시간으로 바뀌고, `[값 복사]` 를 누르면 `cam_x:=...` 한 줄이 클립보드로 간다
(그대로 `camera_tf.launch.py` 기본값에 붙여넣는 게 이 캘리브의 출구다).

맨 아래에는 **검출 목록**을 띄운다. 3D 뷰에 떠다니는 또 하나의 글자 뭉치가
`detection_markers` 의 라벨(`클래스 확신도 [PICK] / 거리cm`)인데, 그건 물체 옆에 붙어
있어서 물체가 겹치거나 멀면 서로 포개져 읽을 수 없다. 같은 내용을 목록으로 보여준다.

## 화면

흰 배경 + **UOS Blue**(서울시립대 UI 전용색상, Pantone 300C / `#005EB8`) 조합이다. 정보를
카드 세 장(자세 / 안내 / 검출)으로 나누고, 각 카드에 파란 제목 띠를 둔다. 자세는 여섯 개의
작은 상자(앞·좌우·높이·roll·pitch·yaw)로 쪼개 놓았다 — 한 줄 문자열로 두면 드래그하는 동안
어느 숫자가 움직였는지 눈이 못 따라간다.

색은 `COLORS` 한 곳에서만 정의한다. 흰 바탕이라 경고/위험/정상 색은 어두운 쪽으로 잡았다
(밝은 형광색은 흰 배경에서 대비가 안 나온다).

## 폰트

컨테이너 이미지에는 한글 폰트가 **하나도 없다**. `docker-compose.yml` 이 호스트
`/usr/share/fonts` 를 읽기 전용으로 얹어 해결한다. 그래도 한글 폰트를 못 찾으면 이 창은
**영어 마커 토픽으로 자동 폴백**하고 그 사실을 상단에 띄운다 — 두부만 잔뜩 나오는 것보다
낫다(`lang` 파라미터로 강제할 수도 있다).

## 사용

    ros2 launch robot_arm_perception camera_calib.launch.py   # 기본으로 같이 뜬다
    ros2 launch robot_arm_perception camera_calib.launch.py status_view:=false
    ros2 run robot_arm_perception calib_status_view --ros-args -p font_size:=18
"""
import math
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time

from std_msgs.msg import String
from visualization_msgs.msg import Marker

import tf2_ros

from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
from robot_arm_perception.camera_tf_tuner import (
    TOPIC_GUIDE, TOPIC_PICK, TOPIC_STATUS, rpy_from_quat,
)

TOPIC_OBJECTS = "/detected_objects"

#: 검출이 많아도 창을 밀어내지 않게 상단 몇 개만 보여준다(확신도 순 아님, 발행 순서 그대로 —
#: `detection_markers` 의 id 와 같은 순서라 3D 라벨과 대조하기 쉽다).
MAX_OBJECT_ROWS = 6

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

#: 한글 글리프가 있는 폰트 후보 — 앞에서부터 먼저 찾은 걸 쓴다. 고정폭을 앞에 두는 이유는
#: 안내문의 체크박스([x][ ][ ])와 단계 들여쓰기가 세로로 맞아야 읽히기 때문이다.
KOREAN_FONTS = ("Noto Sans Mono CJK KR", "NanumGothicCoding", "Noto Sans CJK KR",
                "NanumGothic", "Malgun Gothic", "UnDotum")
FALLBACK_FONT = "DejaVu Sans Mono"

#: 튜너 상태 수신이 이보다 오래 끊기면 "튜너가 죽었나" 를 의심해야 한다.
STALE_S = 3.0

#: 서울시립대 UI 전용색상 "UOS Blue" = Pantone 300C / RGB(0,94,184). 나머지는 그 파랑을
#: 기준으로 맞춘 밝은 배경용 팔레트다(경고/위험/정상은 흰 바탕에서 대비가 나오게 어둡게).
UOS_BLUE = "#005EB8"
UOS_BLUE_DEEP = "#00396E"

COLORS = {
    "bg": "#ffffff",
    "fg": "#1c2430",
    "panel": "#f3f8fd",        # 파랑 기가 아주 옅게 도는 카드 배경
    "border": "#c9dcf0",
    "brand": UOS_BLUE,
    "brand_deep": UOS_BLUE_DEEP,
    "title": UOS_BLUE_DEEP,
    "active": UOS_BLUE,        # 지금 단계
    "active_bg": "#e2eefb",
    "warn": "#b45309",
    "bad": "#c62828",
    "good": "#1b7f4b",
    "dim": "#6b7a8c",
}


class CalibStatusView(Node):
    """ROS 쪽 절반 — 최신 문자열/자세만 락 뒤에 들고 있고, 그리기는 Tk 스레드가 한다."""

    def __init__(self):
        super().__init__("calib_status_view")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("font_size", 14)
        # auto → 한글 폰트가 있으면 한국어(String), 없으면 영어(Marker)
        self.declare_parameter("lang", "auto")
        self.declare_parameter("pose_rate_hz", 10.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self._lock = threading.Lock()
        self._text = ""
        self._text_stamp = None
        self._pose = None          # (x, y, z, roll, pitch, yaw)
        self._tf_error = "TF 대기 중"
        self._objects = []         # [(표시줄, pick 여부)]
        self._pick_key = None      # (class_name, x, y, z) — detection_markers 와 같은 판정

        self.create_subscription(String, TOPIC_GUIDE, self._on_guide, LATCHED)
        self.create_subscription(Marker, TOPIC_STATUS, self._on_marker, LATCHED)
        self.create_subscription(DetectedObjectArray, TOPIC_OBJECTS, self._on_objects, 10)
        self.create_subscription(DetectedObject, TOPIC_PICK, self._on_pick, LATCHED)

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        rate = float(self.get_parameter("pose_rate_hz").value)
        self.create_timer(1.0 / rate, self._read_pose)

        self.use_korean = True     # Tk 쪽이 폰트를 보고 확정한다

    # ── 구독 ───────────────────────────────────

    def _on_guide(self, msg):
        if self.use_korean:
            self._store(msg.data)

    def _on_marker(self, msg):
        if not self.use_korean:
            self._store(msg.text)

    def _store(self, text):
        with self._lock:
            self._text = text
            self._text_stamp = self._now()

    def _on_pick(self, msg):
        p = msg.pose.position
        with self._lock:
            self._pick_key = (msg.class_name, p.x, p.y, p.z)

    def _is_pick(self, obj) -> bool:
        """`detection_markers._is_pick` 와 같은 규칙(이름 일치 + 5cm 이내).

        `/pick_target` 은 `/detected_objects` 와 별개로 발행되므로 같은 물체라도 좌표가
        조금 다르다 — 그래서 동일성 판정이 필요하다.
        """
        if self._pick_key is None:
            return False
        name, px, py, pz = self._pick_key
        if obj.class_name != name:
            return False
        p = obj.pose.position
        return math.dist((p.x, p.y, p.z), (px, py, pz)) < 0.05

    def _on_objects(self, msg):
        rows = []
        for obj in msg.objects:
            p = obj.pose.position
            distance = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
            pick = self._is_pick(obj)
            depth = f"{distance * 100:5.0f}cm" if distance > 1e-6 else "  거리없음"
            rows.append((f"{obj.class_name:<22.22} {obj.confidence:.2f} {depth}"
                         f"{'  ← 집을 대상' if pick else ''}", pick))
        with self._lock:
            self._objects = rows

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── 현재 자세 ───────────────────────────────

    def _read_pose(self):
        """튜너가 30Hz 로 발행하는 TF 를 그대로 읽는다 — 파라미터를 따로 물을 필요가 없고,
        `camera_tf.launch.py`(확정값 static)로 띄운 경우에도 같은 화면이 동작한다."""
        try:
            tf = self._buffer.lookup_transform(
                self.base_frame, self.camera_frame, Time())
        except tf2_ros.TransformException as e:
            with self._lock:
                self._pose = None
                self._tf_error = f"{self.base_frame}→{self.camera_frame} 없음 ({type(e).__name__})"
            return
        t = tf.transform.translation
        roll, pitch, yaw = rpy_from_quat(tf.transform.rotation)
        with self._lock:
            self._pose = (t.x, t.y, t.z, roll, pitch, yaw)
            self._tf_error = None

    # ── Tk 가 읽어가는 스냅샷 ────────────────────

    def snapshot(self):
        with self._lock:
            age = None if self._text_stamp is None else self._now() - self._text_stamp
            return self._text, age, self._pose, self._tf_error, list(self._objects)


def format_values(pose) -> str:
    """`camera_tf.launch.py` 에 그대로 붙여넣는 한 줄 (튜너 로그와 같은 형식)."""
    x, y, z, roll, pitch, yaw = pose
    return (f"cam_x:={x:.4f} cam_y:={y:.4f} cam_z:={z:.4f} "
            f"cam_roll:={roll:.4f} cam_pitch:={pitch:.4f} cam_yaw:={yaw:.4f}")


def line_tag(line: str) -> str:
    """안내문 한 줄 → 색 태그. 문구가 아니라 **접두어/기호**로 판정해 한/영 양쪽에 걸린다."""
    stripped = line.strip()
    if stripped.startswith("==="):
        return "title"
    if stripped.startswith("⚠️"):
        return "warn"
    if line.startswith(">>"):
        return "active"
    if stripped.startswith(("지금:", "NOW:")):
        return "now"
    if "있음" in stripped or "YES" in stripped:
        return "good"
    if ("없음" in stripped or ": NO" in stripped
            or "실패" in stripped or "FAILED" in stripped):
        return "bad"
    if set(stripped) == {"-"}:
        return "dim"
    return "body"


class StatusWindow:
    """Tk 쪽 절반. ROS 콜백에서 위젯을 만지지 않는다(Tk 는 스레드 안전하지 않다)."""

    def __init__(self, node: CalibStatusView):
        self.node = node
        self.size = int(node.get_parameter("font_size").value)
        self._flash_until = 0.0

        self.root = tk.Tk()
        self.root.title("카메라 캘리브 안내")
        self.root.configure(bg=COLORS["bg"])
        self.root.geometry("620x820")
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        family, self.korean_ok = self._pick_font()
        lang = node.get_parameter("lang").value
        node.use_korean = self.korean_ok if lang == "auto" else (lang == "ko")

        self.mono = tkfont.Font(family=family, size=self.size)
        self.big = tkfont.Font(family=family, size=self.size + 4, weight="bold")
        self.small = tkfont.Font(family=family, size=max(8, self.size - 3))
        self.small_bold = tkfont.Font(family=family, size=max(8, self.size - 3),
                                      weight="bold")

        self._build()
        self._tick()

    # ── 폰트 ───────────────────────────────────

    def _pick_font(self):
        available = set(tkfont.families(self.root))
        for name in KOREAN_FONTS:
            if name in available:
                return name, True
        return FALLBACK_FONT, False

    # ── 위젯 ───────────────────────────────────

    def _button(self, parent, text, command, primary=False):
        """Tk 기본 버튼은 회색 3D 테두리라 흰 배경에서 튄다 — 평평한 UOS Blue 로 통일."""
        return tk.Button(
            parent, text=text, command=command,
            font=self.small_bold if primary else self.small,
            relief="flat", bd=0, padx=12, pady=4, cursor="hand2",
            highlightthickness=0,
            bg=COLORS["brand"] if primary else COLORS["panel"],
            fg="#ffffff" if primary else COLORS["brand_deep"],
            activebackground=COLORS["brand_deep"] if primary else COLORS["active_bg"],
            activeforeground="#ffffff" if primary else COLORS["brand_deep"])

    def _card(self, title, subtitle=""):
        """제목 띠(UOS Blue)가 붙은 흰 카드 한 장. 내용을 담을 프레임을 돌려준다."""
        card = tk.Frame(self.root, bg=COLORS["bg"],
                        highlightbackground=COLORS["border"], highlightthickness=1)
        bar = tk.Frame(card, bg=COLORS["brand"])
        bar.pack(fill="x")
        tk.Label(bar, text=title, font=self.small_bold,
                 bg=COLORS["brand"], fg="#ffffff").pack(side="left", padx=10, pady=4)
        if subtitle:
            tk.Label(bar, text=subtitle, font=self.small,
                     bg=COLORS["brand"], fg="#d6e6f7").pack(side="left", pady=4)
        body = tk.Frame(card, bg=COLORS["bg"])
        body.pack(fill="both", expand=True)
        return card, body

    def _metric(self, parent, title, row, column):
        """값 하나짜리 작은 상자 — 숫자를 나란히 놓아야 좌우/높이를 한눈에 비교한다."""
        cell = tk.Frame(parent, bg=COLORS["panel"],
                        highlightbackground=COLORS["border"], highlightthickness=1)
        cell.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        tk.Label(cell, text=title, font=self.small,
                 bg=COLORS["panel"], fg=COLORS["dim"]).pack(anchor="w", padx=8, pady=(3, 0))
        value = tk.Label(cell, text="—", font=self.big,
                         bg=COLORS["panel"], fg=COLORS["brand"])
        value.pack(anchor="w", padx=8, pady=(0, 4))
        return value

    def _build(self):
        card, head = self._card("현재 카메라 자세", "  base_link → camera_link")
        card.pack(fill="x", padx=10, pady=(10, 6))

        grid = tk.Frame(head, bg=COLORS["bg"])
        grid.pack(fill="x", padx=6, pady=(6, 2))
        for column in range(3):
            grid.grid_columnconfigure(column, weight=1, uniform="metric")
        self.metrics = {
            "앞": self._metric(grid, "앞 (x)", 0, 0),
            "좌우": self._metric(grid, "좌우 (y)", 0, 1),
            "높이": self._metric(grid, "높이 (z)", 0, 2),
            "roll": self._metric(grid, "roll", 1, 0),
            "pitch": self._metric(grid, "pitch", 1, 1),
            "yaw": self._metric(grid, "yaw", 1, 2),
        }
        # TF 가 없을 때는 상자 여섯 개 대신 이유 한 줄을 띄운다.
        self.pose_error = tk.Label(head, text="", font=self.small, anchor="w",
                                   bg=COLORS["bg"], fg=COLORS["bad"])

        self.values_label = tk.Label(head, text="", font=self.small, justify="left",
                                     bg=COLORS["panel"], fg=COLORS["fg"], wraplength=540,
                                     anchor="w", padx=8, pady=4,
                                     highlightbackground=COLORS["border"],
                                     highlightthickness=1)
        self.values_label.pack(fill="x", padx=9, pady=(4, 0))

        buttons = tk.Frame(head, bg=COLORS["bg"])
        buttons.pack(anchor="w", padx=9, pady=8)
        self._button(buttons, "값 복사", self._copy, primary=True).pack(side="left")
        self._button(buttons, "글자 +", lambda: self._resize(+2)).pack(side="left", padx=(6, 0))
        self._button(buttons, "글자 −", lambda: self._resize(-2)).pack(side="left", padx=(4, 0))

        # ⚠️ 아래쪽 패널을 **먼저** pack 해야 한다. Tk 의 pack 은 호출 순서대로 남은 공간을
        # 잘라 주는데, 늘어나는(expand) 본문을 먼저 붙이면 그게 공간을 다 가져가고 뒤에
        # 붙는 위젯은 창 밖으로 밀려 **조용히 사라진다**(2026-08-13 실제로 그렇게 됐다).
        self.foot = tk.Label(self.root, text="", font=self.small, anchor="w",
                             bg=COLORS["bg"], fg=COLORS["dim"], wraplength=580,
                             justify="left")
        self.foot.pack(side="bottom", fill="x", padx=12, pady=(4, 8))

        # 3D 뷰의 물체 라벨은 물체 옆에 붙어 서로 겹친다 — 같은 내용을 목록으로도 준다.
        card, objects = self._card("카메라에 보이는 물체", "  /detected_objects")
        card.pack(side="bottom", fill="x", padx=10, pady=(6, 0))
        # 높이는 내용에 맞춰 매번 줄인다 — 고정으로 두면 물체가 없을 때도 자리를 차지해
        # 정작 중요한 안내문이 잘린다.
        self.objects = tk.Text(objects, font=self.mono, height=1,
                               bg=COLORS["bg"], fg=COLORS["fg"], bd=0,
                               padx=10, pady=6, wrap="none",
                               insertbackground=COLORS["bg"], highlightthickness=0)
        self.objects.pack(fill="x")
        self.objects.configure(state="disabled")
        self.objects.tag_configure("good", foreground=COLORS["good"])
        self.objects.tag_configure("body", foreground=COLORS["fg"])
        self.objects.tag_configure("dim", foreground=COLORS["dim"])

        card, guide = self._card("캘리브레이션 안내", "  /perception/calib_guide")
        card.pack(side="top", fill="both", expand=True, padx=10)
        self.text = tk.Text(guide, font=self.mono, height=12,
                            bg=COLORS["bg"], fg=COLORS["fg"], bd=0,
                            padx=12, pady=10, wrap="word",
                            insertbackground=COLORS["bg"], highlightthickness=0)
        self.text.pack(fill="both", expand=True)
        self.text.configure(state="disabled")
        for tag, colour in (("title", COLORS["title"]), ("active", COLORS["active"]),
                            ("warn", COLORS["warn"]), ("bad", COLORS["bad"]),
                            ("good", COLORS["good"]), ("dim", COLORS["dim"]),
                            ("now", COLORS["brand_deep"]), ("body", COLORS["fg"])):
            self.text.tag_configure(tag, foreground=colour)
        self.text.tag_configure("title", font=self.big, foreground=COLORS["title"])
        self.text.tag_configure("active", background=COLORS["active_bg"])

        if not self.korean_ok:
            self.foot.configure(
                text="한글 폰트를 못 찾아 영어 안내로 폴백했습니다 "
                     "(docker-compose.yml 의 /usr/share/fonts 마운트 확인)",
                fg=COLORS["warn"])

    def _resize(self, delta):
        self.size = max(8, self.size + delta)
        self.mono.configure(size=self.size)
        self.big.configure(size=self.size + 4)
        self.small.configure(size=max(8, self.size - 3))
        self.small_bold.configure(size=max(8, self.size - 3))

    def _copy(self):
        pose = self.node.snapshot()[2]
        if pose is None:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(format_values(pose))
        # 하단 줄은 200ms 마다 다시 그려지므로, 잠깐 붙잡아 두지 않으면 눌러도 아무 일이
        # 안 일어난 것처럼 보인다.
        self._flash_until = time.monotonic() + 3.0
        self.foot.configure(text="클립보드에 복사됨 — camera_tf.launch.py 기본값에 붙여넣기",
                            fg=COLORS["good"])

    # ── 주기 갱신 ───────────────────────────────

    def _tick(self):
        # ⚠️ 이게 없으면 `pkill`/Ctrl-C 로 안 죽는다. rclpy 의 시그널 핸들러는 **컨텍스트만**
        # 내리고 프로세스를 끝내지 않는데, 여기서는 Tk mainloop 가 메인 스레드를 잡고 있어
        # 창이 그대로 남는다 — launch 를 내려도 살아남아 옛 값을 계속 띄우는 유령이 된다
        # (2026-08-13 실제로 SIGTERM 을 무시하고 남았다).
        if not rclpy.ok():
            self.root.quit()
            return

        text, age, pose, tf_error, objects = self.node.snapshot()

        self._render_pose(pose, tf_error)

        self._render(text, age)
        self._render_objects(objects)
        self.root.after(200, self._tick)

    def _render_pose(self, pose, tf_error):
        if pose is None:
            for label in self.metrics.values():
                label.configure(text="—", fg=COLORS["dim"])
            self.values_label.configure(
                text="TF 가 들어오면 복사용 값이 여기 표시됩니다", fg=COLORS["dim"])
            self.pose_error.configure(text=tf_error or "TF 대기 중")
            self.pose_error.pack(fill="x", padx=9, pady=(2, 0))
            return

        self.pose_error.pack_forget()
        x, y, z, roll, pitch, yaw = pose
        values = {
            "앞": f"{x * 100:+.1f} cm",
            "좌우": f"{y * 100:+.1f} cm",
            "높이": f"{z * 100:+.1f} cm",
            "roll": f"{math.degrees(roll):+.1f}°",
            "pitch": f"{math.degrees(pitch):+.1f}°",
            "yaw": f"{math.degrees(yaw):+.1f}°",
        }
        for name, label in self.metrics.items():
            label.configure(text=values[name], fg=COLORS["brand"])
        self.values_label.configure(text=format_values(pose), fg=COLORS["fg"])

    def _render_objects(self, objects):
        self.objects.configure(state="normal")
        self.objects.delete("1.0", "end")
        if not objects:
            self.objects.insert("end", "인식된 물체 없음\n", "dim")
        for row, pick in objects[:MAX_OBJECT_ROWS]:
            self.objects.insert("end", row + "\n", "good" if pick else "body")
        extra = max(0, len(objects) - MAX_OBJECT_ROWS)
        if extra:
            self.objects.insert("end", f"... 외 {extra}개\n", "dim")
        self.objects.configure(
            state="disabled",
            height=max(1, min(len(objects), MAX_OBJECT_ROWS) + (1 if extra else 0)))

    def _render(self, text, age):
        if not text:
            body = ("camera_tf_tuner 를 기다리는 중입니다.\n\n"
                    "이 창은 튜너가 발행하는 안내문을 그대로 띄웁니다.\n"
                    "튜너가 떠 있는데도 비어 있으면:\n"
                    "  ros2 topic echo /perception/calib_guide\n"
                    "로 발행 자체를 먼저 확인하세요.")
        else:
            body = text

        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        for line in body.split("\n"):
            tag = line_tag(line)
            if tag == "title":
                continue          # 카드 제목 띠가 같은 말을 하고 있다
            self.text.insert("end", line + "\n", tag)
        self.text.configure(state="disabled")

        if time.monotonic() < self._flash_until:
            return                                  # 방금 띄운 안내를 덮지 않는다
        if age is None:
            self.foot.configure(text="튜너 상태: 수신 없음", fg=COLORS["bad"])
        elif age > STALE_S:
            self.foot.configure(text=f"튜너 상태: {age:.0f}초째 갱신 없음 (노드가 죽었는지 확인)",
                                fg=COLORS["warn"])
        elif self.korean_ok:
            self.foot.configure(text=f"튜너 상태: 정상 ({age:.1f}초 전 갱신)", fg=COLORS["dim"])

    def _close(self):
        self.root.quit()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CalibStatusView()

    # ROS 는 배경 스레드에서 돌리고 Tk 가 메인 스레드를 쓴다 — 반대로 하면 macOS/Tk 제약과
    # 별개로 `mainloop()` 가 블로킹이라 콜백이 아예 안 돈다.
    spin = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin.start()
    try:
        window = StatusWindow(node)
    except tk.TclError as e:
        node.get_logger().error(
            f"창을 열 수 없습니다: {e}\n"
            "DISPLAY 가 컨테이너에 전달됐는지 확인하세요 — 호스트에서 "
            "`xhost +local:docker` 후 `docker compose up -d`.")
        rclpy.shutdown()
        return
    try:
        window.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
