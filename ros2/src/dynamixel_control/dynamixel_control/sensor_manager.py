"""Fail-closed sensor abstraction for the arm cleaning FSM."""

import math

from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, Float64


class SensorManager:
    """Own sensor subscriptions and expose backend-independent safety states.

    A real backend is usable only after a fresh, finite sample has arrived. Mock
    mode bypasses subscriptions deliberately and is intended for simulation/tests.
    """

    def __init__(self, node):
        self._node = node
        self._declare_parameters()
        g = node.get_parameter

        self.mock_mode = bool(g('sensor_mock_mode').value)
        self.timeout = float(g('sensor_timeout').value)
        self.contact_enabled = bool(g('contact_sensor_enabled').value)
        self.force_enabled = bool(g('force_sensor_enabled').value)
        self.distance_enabled = bool(g('distance_sensor_enabled').value)
        self.lock_enabled = bool(g('lock_sensor_enabled').value)
        self.joint_effort_enabled = bool(g('joint_effort_sensor_enabled').value)
        self.force_threshold = float(g('force_contact_threshold').value)
        self.distance_threshold = float(g('obstacle_distance_threshold').value)
        self.joint_effort_threshold = float(g('joint_effort_contact_threshold').value)
        self.joint_effort_name = str(g('joint_effort_sensor_joint').value)

        self._samples = {}
        if not self.mock_mode:
            self._subscribe(Bool, g('contact_sensor_topic').value,
                            'contact', lambda msg: bool(msg.data))
            self._subscribe(Float64, g('force_sensor_topic').value,
                            'force', lambda msg: float(msg.data))
            self._subscribe(Range, g('distance_sensor_topic').value,
                            'distance', lambda msg: float(msg.range))
            self._subscribe(Bool, g('lock_sensor_topic').value,
                            'lock', lambda msg: bool(msg.data))
            self._subscribe(JointState, g('joint_effort_sensor_topic').value,
                            'joint_effort', self._joint_effort_value)

    def _declare_parameters(self):
        declare = self._node.declare_parameter
        declare('sensor_mock_mode', False)
        declare('sensor_timeout', 0.5)

        declare('contact_sensor_enabled', False)
        declare('contact_sensor_topic', '/sensors/contact')
        declare('force_sensor_enabled', False)
        declare('force_sensor_topic', '/sensors/force')
        declare('force_contact_threshold', 0.0)
        declare('distance_sensor_enabled', True)
        declare('distance_sensor_topic', '/sensors/distance')
        declare('obstacle_distance_threshold', 0.10)
        declare('lock_sensor_enabled', True)
        declare('lock_sensor_topic', '/sensors/lock_confirmed')
        declare('joint_effort_sensor_enabled', True)
        declare('joint_effort_sensor_topic', '/joint_states')
        declare('joint_effort_sensor_joint', '')
        declare('joint_effort_contact_threshold', 0.0)

        declare('mock_contact', False)
        declare('mock_force', 0.0)
        declare('mock_distance', 0.0)
        declare('mock_lock_confirmed', False)

    def _subscribe(self, msg_type, topic, key, converter):
        if not topic:
            self._node.get_logger().warn(
                f'{key} sensor topic is empty; backend remains fail-closed')
            return

        def callback(msg):
            value = converter(msg)
            if value is not None:
                self._samples[key] = (value, self._node.get_clock().now())

        self._node.create_subscription(msg_type, topic, callback, 10)

    def _joint_effort_value(self, msg):
        if not self.joint_effort_name:
            return None
        try:
            index = msg.name.index(self.joint_effort_name)
        except ValueError:
            return None
        if index >= len(msg.effort):
            return None
        return abs(float(msg.effort[index]))

    def _fresh_value(self, key):
        sample = self._samples.get(key)
        if sample is None:
            return None
        value, stamp = sample
        age = (self._node.get_clock().now() - stamp).nanoseconds * 1e-9
        if age < 0.0 or age > self.timeout:
            return None
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    def contact_confirmed(self):
        if self.mock_mode:
            direct = bool(self._node.get_parameter('mock_contact').value)
            force = abs(float(self._node.get_parameter('mock_force').value))
            return direct or (
                self.force_threshold > 0.0 and force >= self.force_threshold)

        results = []
        if self.contact_enabled:
            value = self._fresh_value('contact')
            results.append(value is not None and bool(value))
        if self.force_enabled and self.force_threshold > 0.0:
            value = self._fresh_value('force')
            results.append(value is not None and abs(value) >= self.force_threshold)
        if self.joint_effort_enabled and self.joint_effort_threshold > 0.0:
            value = self._fresh_value('joint_effort')
            results.append(
                value is not None and value >= self.joint_effort_threshold)
        return bool(results) and any(results)

    def obstacle_clear(self):
        if self.mock_mode:
            distance = float(self._node.get_parameter('mock_distance').value)
        else:
            if not self.distance_enabled:
                return False
            distance = self._fresh_value('distance')
            if distance is None:
                return False
        return math.isfinite(distance) and distance > self.distance_threshold

    def lock_confirmed(self):
        if self.mock_mode:
            return bool(self._node.get_parameter('mock_lock_confirmed').value)
        if not self.lock_enabled:
            return False
        value = self._fresh_value('lock')
        return value is not None and bool(value)
