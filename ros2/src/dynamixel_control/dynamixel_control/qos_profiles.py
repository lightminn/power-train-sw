"""계약이 요구하는 QoS 프로파일 (계약 §5.1).

값 어휘는 `contract.py`(ROS 비의존)에, rclpy 가 필요한 QoS 는 여기에 둔다.

QoS 를 임의로 바꾸지 말 것 — depth 를 키우면 낡은 샘플이 큐에 쌓여 파워트레인의
age(신선도) 판정이 어긋나고, Reliability 가 어긋나면 토픽이 아예 연결되지 않는다.
"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


#: ArmStatus·ChassisMode heartbeat — Reliable / KeepLast **1** / Volatile.
#: 최신 한 건만 의미가 있다. 쌓아두면 stale 판정이 틀어진다.
HEARTBEAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

#: ArrivalStatus — Reliable / KeepLast 10 / Volatile.
#: 파워트레인이 2 Hz 로 최대 2초간 재발행하므로 몇 건은 버퍼링한다.
ARRIVAL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
