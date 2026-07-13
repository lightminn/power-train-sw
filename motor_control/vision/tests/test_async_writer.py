"""AsyncWriter — 인코더 파이프 write 를 검출 루프에서 분리해 저사양 보드에서도
캡처+추론 fps 가 인코딩 속도에 발목잡히지 않게 하는 백프레셔 동작을 검증.
인코딩이 느려도(파이프 write 블록) 최신 프레임만 유지하고 밀린 프레임은 버려야 한다.
"""
import queue
import threading
import time

from yolo_depth_3d import AsyncWriter


class FakeStdin:
    """write() 호출마다 게이트(Event)를 큐에 올리고 그 게이트가 열릴 때까지 블록.

    테스트가 pending 큐에서 (buf, gate) 를 꺼내 원하는 타이밍에 gate.set() 하므로,
    real-time sleep 기반 경합 없이 "이번 write 가 어떤 프레임인지" 결정적으로 확인 가능.
    """

    def __init__(self):
        self.writes: list[bytes] = []
        self.closed = False
        self.pending: "queue.Queue[tuple[bytes, threading.Event]]" = queue.Queue()

    def write(self, buf: bytes) -> None:
        gate = threading.Event()
        self.pending.put((buf, gate))
        gate.wait(timeout=2.0)
        self.writes.append(buf)

    def close(self) -> None:
        self.closed = True


class FakeProc:
    def __init__(self):
        self.stdin = FakeStdin()
        self.waited = False

    def wait(self, timeout=None):
        self.waited = True

    def terminate(self):
        pass


def test_busy_encoder_drops_stale_frames_keeps_latest():
    proc = FakeProc()
    writer = AsyncWriter(proc)
    try:
        writer.submit(b"frame1")
        buf1, gate1 = proc.stdin.pending.get(timeout=1.0)  # 워커가 frame1 write 중(블록)
        assert buf1 == b"frame1"

        # 인코더가 frame1 을 처리하는 동안 프레임이 두 번 더 들어옴 — frame2 는 밀려서 버려짐.
        writer.submit(b"frame2")
        writer.submit(b"frame3")

        gate1.set()  # frame1 write 완료 허용
        buf2, gate2 = proc.stdin.pending.get(timeout=1.0)  # 워커가 다음으로 집는 프레임
        assert buf2 == b"frame3"  # frame2 는 버려짐 — 최신 프레임만 유지
        gate2.set()

        for _ in range(50):
            if proc.stdin.writes == [b"frame1", b"frame3"]:
                break
            time.sleep(0.02)
        assert proc.stdin.writes == [b"frame1", b"frame3"]
    finally:
        writer.close()


def test_close_closes_stdin_and_waits_for_process():
    proc = FakeProc()
    writer = AsyncWriter(proc)
    writer.close()
    assert proc.stdin.closed
    assert proc.waited


def test_broken_pipe_marks_writer_not_alive():
    class BrokenStdin(FakeStdin):
        def write(self, buf):
            raise BrokenPipeError()

    proc = FakeProc()
    proc.stdin = BrokenStdin()
    writer = AsyncWriter(proc)
    writer.submit(b"frame1")

    for _ in range(50):
        if not writer.alive:
            break
        time.sleep(0.02)
    assert not writer.alive  # 파이프 끊김을 메인 루프가 감지해 종료할 수 있어야 함
