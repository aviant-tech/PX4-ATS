uint64 timestamp

uint32 time_boot_ms		# Timestamp (time since system boot) in ms
uint64 time_unix_usec		# Timestamp (UNIX epoch time) in us
uint8 armed			# Is the aircraft armed? (MAV_BOOL)
uint8 vtol_state		# MAV_VTOL_STATE enum
uint8 system_status		# MAV_STATE enum, equivalent to HEARTBEAT.system_status

uint8 SYSTEM_STATUS_UNINIT = 0
uint8 SYSTEM_STATUS_BOOT = 1
uint8 SYSTEM_STATUS_CALIBRATING = 2
uint8 SYSTEM_STATUS_STANDBY = 3
uint8 SYSTEM_STATUS_ACTIVE = 4
uint8 SYSTEM_STATUS_CRITICAL = 5
uint8 SYSTEM_STATUS_EMERGENCY = 6
uint8 SYSTEM_STATUS_POWEROFF = 7
uint8 SYSTEM_STATUS_FLIGHT_TERMINATION = 8
