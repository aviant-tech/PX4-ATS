# WARNING: this message is logged at a high rate
# make sure to verify the expected log size if you change this

uint64 timestamp

AviantAtsFcCheck fc
bool fc_timeout
bool accel_norm_fail
AviantAtsAttitudeCheck attitude
AviantAtsVoltageCheck voltage

bool power_loss_trigger_enabled
bool ats_enabled

bool inflight_control_failure
bool inflight_power_failure
bool parachute_deploy

uint8 received_acks

uint8 RECEIVED_ACK_FLIGHTTERMINATION = 1 # (1 << 0)
uint8 RECEIVED_ACK_PARACHUTE = 2 # (1 << 1)

uint8 internal_failure_flags

uint8 IFAIL_DETAILED_FC_STATE = 1 # (1 << 0)
uint8 IFAIL_VEHICLE_ACCELERATION = 2 # (1 << 1)
uint8 IFAIL_VEHICLE_ATTITUDE = 4 # (1 << 2)
uint8 IFAIL_ADC_REPORT = 8 # (1 << 3)
uint8 IFAIL_NO_FC_BATTERY_STATUS = 16 # (1 << 4)
