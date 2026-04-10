# Battery data from an external source (e.g. flight controller BATTERY_STATUS over MAVLink).
# Distinct from battery_status, which reflects this board's local battery driver.
# NOTE: We try to keep this topic as small as possible, so we can log it at max rate.
# therefore, a lot of the information from BATTERY_STATUS is discarded

uint64 timestamp

float32 voltage_v
float32 current_a
