"""Optional pydualsense output isolated from the pygame input path."""

import sys
import threading


OUTPUT_HZ = 20.0
_TELEOP_COLOR = (255, 255, 255)
_UNSET = object()


class _PyDualSenseBackend:
    """Small adapter around the third-party pydualsense API."""

    def __init__(self, controller, trigger_modes):
        self._controller = controller
        self._trigger_modes = trigger_modes

    def rumble(self, low, high, duration_ms):
        del duration_ms
        self._controller.setLeftMotor(
            int(round(max(0.0, min(1.0, float(low))) * 255.0))
        )
        self._controller.setRightMotor(
            int(round(max(0.0, min(1.0, float(high))) * 255.0))
        )

    def lightbar(self, color):
        neutral = (0, 0, 0) if color is None else tuple(color)
        self._controller.light.setColorT(neutral)

    def trigger_lock(self, locked):
        if not locked:
            self._controller.triggerL.setMode(self._trigger_modes.Off)
            self._controller.triggerR.setMode(self._trigger_modes.Off)
            return

        # Strong resistance plus a pulse cue, capped below full stiffness.
        self._controller.triggerL.setMode(self._trigger_modes.Rigid)
        self._controller.triggerL.setForce(0, 180)
        self._controller.triggerR.setMode(self._trigger_modes.Pulse)
        self._controller.triggerR.setForce(0, 160)
        self._controller.triggerR.setForce(1, 110)

    def close(self):
        self._controller.setLeftMotor(0)
        self._controller.setRightMotor(0)
        self._controller.triggerL.setMode(self._trigger_modes.Off)
        self._controller.triggerR.setMode(self._trigger_modes.Off)
        self._controller.light.setColorT((0, 0, 0))
        self._controller.close()


def _default_backend_factory():
    """Import pydualsense only when haptic output is actually requested."""
    try:
        from pydualsense import TriggerModes, pydualsense
    except ImportError:
        return None
    controller = pydualsense()
    controller.init()
    return _PyDualSenseBackend(controller, TriggerModes)


class DualSenseOutput:
    """Drive optional feedback at 20 Hz without affecting controller input."""

    def __init__(
        self,
        arbiter,
        *,
        backend_factory=None,
        clock,
        trigger_fx=False,
    ):
        self._arbiter = arbiter
        self._backend_factory = (
            _default_backend_factory
            if backend_factory is None
            else backend_factory
        )
        self._clock = clock
        self._trigger_fx = bool(trigger_fx)
        self._backend = None
        self._backend_attempted = False
        self._disabled = False
        self._warned = False
        self._rumble_until_s = None
        self._last_lightbar = _UNSET
        self._last_trigger_lock = _UNSET
        self._stop = threading.Event()
        self._thread = None

    def _close_backend(self):
        backend = self._backend
        self._backend = None
        if backend is None:
            return
        try:
            backend.close()
        except:  # Output teardown must never affect input or process lifetime.
            pass

    def _disable(self, *, warn):
        self._disabled = True
        self._close_backend()
        if warn and not self._warned:
            self._warned = True
            print(
                "warning: DualSense haptics disabled after backend failure",
                file=sys.stderr,
            )

    def _ensure_backend(self):
        if self._disabled:
            return False
        if self._backend is not None:
            return True
        if self._backend_attempted:
            return False
        self._backend_attempted = True
        try:
            backend = self._backend_factory()
        except:  # HID/library failures disable output only.
            self._disable(warn=True)
            return False
        if backend is None:
            self._disable(warn=False)
            return False
        self._backend = backend
        return True

    def run_once(self):
        """Execute one deterministic 20 Hz output tick."""
        if not self._ensure_backend():
            return False
        try:
            now_s = float(self._clock())
            decision = self._arbiter.decide()
            if decision is not None:
                self._backend.rumble(
                    decision.low,
                    decision.high,
                    decision.duration_ms,
                )
                self._rumble_until_s = (
                    now_s + float(decision.duration_ms) / 1000.0
                )
            elif (
                self._rumble_until_s is not None
                and now_s >= self._rumble_until_s
            ):
                self._backend.rumble(0.0, 0.0, 0)
                self._rumble_until_s = None

            color = self._arbiter.lightbar()
            if color != self._last_lightbar:
                self._backend.lightbar(color)
                self._last_lightbar = color

            if self._trigger_fx:
                locked = color != _TELEOP_COLOR
                if locked != self._last_trigger_lock:
                    self._backend.trigger_lock(locked)
                    self._last_trigger_lock = locked
            # Slip-flutter intentionally remains disabled until C1 validation.
        except:  # A single backend failure permanently isolates this feature.
            self._disable(warn=True)
            return False
        return True

    def _run(self):
        interval_s = 1.0 / OUTPUT_HZ
        while not self._stop.is_set():
            if not self.run_once() and self._disabled:
                break
            self._stop.wait(interval_s)

    def start(self):
        if self._thread is not None or self._disabled:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="dualsense-output",
            daemon=True,
        )
        self._thread.start()

    def close(self):
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None
        self._close_backend()
