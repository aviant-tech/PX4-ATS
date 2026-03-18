uint64 timestamp

uint8 fc_state
uint8 FC_STATE_DISARMED   = 0
uint8 FC_STATE_ARMED      = 1
uint8 FC_STATE_TERMINATED = 2

float32 main_power1_v
float32 main_power2_v
float32 ups_v

bool accel_norm_fail # free fall
bool roll_fail
bool pitch_fail
bool fc_timeout
bool fc_rebooted_while_armed
bool ups_healthy
bool main_voltage_fail

bool power_loss_trigger_enabled
bool ats_enabled

bool maybe_parachute_deploy
bool parachute_deploy
