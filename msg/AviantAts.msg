uint64 timestamp

bool fc_armed
bool fc_flight_termination

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
bool parachute_deploy # latching

bool flighttermination_acked
bool parachute_acked
