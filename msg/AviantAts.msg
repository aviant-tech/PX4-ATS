uint64 timestamp

uint8 fc_state
uint8 FC_STATE_DISARMED   = 0
uint8 FC_STATE_ARMED      = 1
uint8 FC_STATE_TERMINATED = 2
bool accel_norm_fail # free fall
bool roll_fail
bool pitch_fail
bool fc_timeout
bool fc_rebooted_while_armed
bool parachute_deploy
