"""Read-only PDIST80B RS485 monitor ROS node."""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from powertrain_ros.pdist80b import bms_monitor_request, parse_bms_monitor_response


class Pdist80bMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("pdist80b_monitor")
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 57600)
        self.declare_parameter("device_id", 1)
        self.declare_parameter("poll_hz", 2.0)
        port = str(self.get_parameter("port").value)
        if not port:
            raise ValueError("port is required; never share the US-100 UART")
        if port == "/dev/ttyTHS1":
            raise ValueError("/dev/ttyTHS1 is reserved for US-100 safety")
        baud = int(self.get_parameter("baud").value)
        device_id = int(self.get_parameter("device_id").value)
        poll_hz = float(self.get_parameter("poll_hz").value)
        if baud != 57600 or not 0.2 <= poll_hz <= 5.0:
            raise ValueError("PDIST80B requires 57600 baud and poll_hz within 0.2..5.0")
        import serial  # pyserial is installed in the Jetson ROS image.

        self._device_id = device_id
        self._serial = serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N", stopbits=1,
                                     timeout=0.15, write_timeout=0.15)
        self._publisher = self.create_publisher(String, "/power/pdist80b", 10)
        self.create_timer(1.0 / poll_hz, self._poll)
        self.get_logger().info("PDIST80B read-only monitor active on %s", port)

    def _poll(self) -> None:
        try:
            self._serial.reset_input_buffer()
            self._serial.write(bms_monitor_request(self._device_id))
            self._serial.flush()
            response = self._serial.read(18)
            status = parse_bms_monitor_response(response, self._device_id)
        except (OSError, ValueError) as exc:
            self.get_logger().warning("PDIST80B read failed: %s", exc, throttle_duration_sec=5.0)
            return
        message = String()
        message.data = json.dumps({
            "voltage_v": status.voltage_v,
            "discharge_current_a": status.discharge_current_a,
            "charge_current_a": status.charge_current_a,
            "soc_percent": status.soc_percent,
            "battery_flags": status.battery_flags,
            "protection_flags": status.protection_flags,
        }, separators=(",", ":"))
        self._publisher.publish(message)

    def destroy_node(self):
        serial_port = getattr(self, "_serial", None)
        if serial_port is not None:
            serial_port.close()
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = Pdist80bMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
