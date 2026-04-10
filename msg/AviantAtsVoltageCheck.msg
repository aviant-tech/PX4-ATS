float32 main_power1_v
float32 main_power2_v
float32 ups_v
float32 parachute_v
float32 fc_battery_v
bool main_voltage_fail

uint8 ups_status_flags

uint8 UPS_STATUS_LOWV = 1 # (1 << 0)
uint8 UPS_STATUS_FC_MISMATCH = 2 # (1 << 1)
uint8 UPS_STATUS_MP_MISMATCH = 4 # (1 << 2)
