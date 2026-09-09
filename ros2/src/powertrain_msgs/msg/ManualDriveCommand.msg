# Manual 4WS input. Speed is longitudinal at the midpoint of the fixed middle axle.
# Positive speed is forward; steering is normalized [-1, 1], positive to body left.
# A steering value selects the same wheel angles at zero, forward and reverse speed.
# This message never represents yaw rate or an in-place pivot command.
float64 speed_mps
float64 steering
