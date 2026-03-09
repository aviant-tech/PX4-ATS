#include "ATS.hpp"
#include <math.h>

using namespace time_literals;

ATS::ATS() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ATS::~ATS()
{
	ScheduleClear();
}

bool
ATS::init()
{
	bool success = true;

	_last_fc_timestamp = hrt_absolute_time();

	_ext_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_ext_vehicle_status.failsafe = false;

	ScheduleOnInterval(1_ms);

	return success;
}

void
ATS::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_ext_vehicle_status_sub.update(&_ext_vehicle_status)) {

		if (_ext_vehicle_status.failsafe) {
			_fc_state = FC_STATE::TERMINATED;

		} else if (_ext_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			_fc_state = FC_STATE::ARMED;

		} else if (_ext_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
			_fc_state = FC_STATE::DISARMED;
		}

		_aviant_ats.fc_state = static_cast<uint8_t>(_fc_state);
	}

	vehicle_attitude_s external_attitude{};
	vehicle_attitude_s vehicle_attitude{};

	if (_external_attitude_sub.update(&external_attitude)) {

		matrix::Quatf q(external_attitude.q);
		matrix::Eulerf euler(q);

		_fc_roll  = math::degrees(euler.phi());
		_fc_pitch = math::degrees(euler.theta());

		_last_fc_timestamp = external_attitude.timestamp;
	}

	if (hrt_elapsed_time(&_last_fc_timestamp) > (_params_av_ats_timeout.get() * 1000ULL)) {
		_aviant_ats.fc_timeout = true;

	} else {
		_aviant_ats.fc_timeout = false;
	}

	vehicle_acceleration_s vehicle_acceleration;

	if (_vehicle_acceleration_sub.update(&vehicle_acceleration)) {

		float accel_norm = sqrtf(
					   vehicle_acceleration.xyz[0] * vehicle_acceleration.xyz[0] +
					   vehicle_acceleration.xyz[1] * vehicle_acceleration.xyz[1] +
					   vehicle_acceleration.xyz[2] * vehicle_acceleration.xyz[2]
				   );

		if (accel_norm < _params_av_ats_acc_norm.get()) {
			_aviant_ats.accel_norm_fail = true;

		} else {
			_aviant_ats.accel_norm_fail = false;
		}
	}

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {

		matrix::Quatf q(vehicle_attitude.q);
		matrix::Eulerf euler(q);

		_ats_roll  = math::degrees(euler.phi());
		_ats_pitch = math::degrees(euler.theta());

		if (fabsf(_ats_roll) > _params_av_ats_roll_ang.get()) {
			_aviant_ats.roll_fail = true;

		} else {
			_aviant_ats.roll_fail = false;
		}

		if (fabsf(_ats_pitch) > _params_av_ats_pitch_ang.get()) {
			_aviant_ats.pitch_fail = true;

		} else {
			_aviant_ats.pitch_fail = false;
		}
	}

	const bool ats_active = static_cast<bool>(_params_av_ats_active.get());

	if (ats_active) {
		switch (_fc_state) {
		case FC_STATE::ARMED:

			if (_aviant_ats.fc_timeout &&
			    (_aviant_ats.accel_norm_fail || _aviant_ats.roll_fail || _aviant_ats.pitch_fail)
			   ) {
				_aviant_ats.parachute_deploy = true;
			}

			break;

		case FC_STATE::TERMINATED:
			_aviant_ats.parachute_deploy = true;
			break;

		case FC_STATE::DISARMED:
			// Do nothing. No recovery is available at this time. A reboot is required.
			break;

		default:
			break;
		}
	}

	if (!_publish_vehicle_command_once && _aviant_ats.parachute_deploy) {
		send_parachute_command();
		send_flighttermination_command();
		_publish_vehicle_command_once = true;
	}

	_aviant_ats.timestamp = hrt_absolute_time();
	_aviant_ats_pub.publish(_aviant_ats);
}

void ATS::send_parachute_command()
{
	vehicle_command_s vcmd{};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
	vcmd.param1 = static_cast<float>(vehicle_command_s::PARACHUTE_ACTION_RELEASE);

	vcmd.source_system = _param_mav_sys_id.get();
	// It's safe to assume that the ATS and FC have the same system ID,
	// since they're part of the same aircraft
	vcmd.target_system = vcmd.source_system;
	vcmd.source_component = _param_mav_comp_id.get();
	vcmd.target_component = 161; // MAV_COMP_ID_PARACHUTE

	uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vehicle_command_pub.publish(vcmd);

	PX4_INFO("Send PARACHUTE CMD");
}

void ATS::send_flighttermination_command()
{
	vehicle_command_s vcmd{};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION;
	vcmd.param1 = 1.0f; //terminate

	vcmd.source_system = _param_mav_sys_id.get();
	// It's safe to assume that the ATS and FC have the same system ID,
	// since they're part of the same aircraft
	vcmd.target_system = vcmd.source_system;
	vcmd.source_component = _param_mav_comp_id.get();
	vcmd.target_component = 1;

	uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vehicle_command_pub.publish(vcmd);

	PX4_INFO("Send FLIGHTTERMINATION CMD");
}

int ATS::task_spawn(int argc, char *argv[])
{
	ATS *instance = new ATS();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ATS::print_status()
{
	PX4_INFO("Running\n");

	printf("FC State: %s\n", fcStateToString(_fc_state));
	printf("FC Timestamp: %lld\n", _last_fc_timestamp);
	printf("FC  Roll: %.2f°, Pitch: %.2f°\n", (double)_fc_roll, (double)_fc_pitch);
	printf("ATS Roll: %.2f°, Pitch: %.2f°\n", (double)_ats_roll, (double)_ats_pitch);
	return 0;
}

int ATS::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ATS::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aviant_ats", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int aviant_ats_main(int argc, char *argv[]);

int aviant_ats_main(int argc, char *argv[])
{
	return ATS::main(argc, argv);
}