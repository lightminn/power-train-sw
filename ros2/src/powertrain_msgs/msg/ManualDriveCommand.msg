# Manual 4WS input. Speed is longitudinal at the midpoint of the fixed middle axle.
# Positive speed is forward; steering is normalized [-1, 1], positive to body left.
# A steering value selects the same wheel angles at zero, forward and reverse speed.
# This message never represents yaw rate or an in-place pivot command.
float64 speed_mps
float64 steering
# Original accepted TCP frame receipt on the robot CLOCK_MONOTONIC clock.
# Preserved on repeated publications. Zero/absent provenance forbids auto resume.
float64 source_received_s
# Server-generated UUID for this accepted TCP connection, bound to these values.
string connection_session_id
