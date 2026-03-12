#include "ATS.hpp"
#include "drivers/drv_hrt.h"
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

	_last_sign_of_life_from_fc = hrt_absolute_time();

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

	external_aviant_detailed_fc_state_s ext_fc_state{};

	if (_ext_detailed_fc_state_sub.update(&ext_fc_state)) {

		_last_sign_of_life_from_fc = ext_fc_state.timestamp;

		const int64_t fc_timestamp = static_cast<int64_t>(ext_fc_state.time_boot_ms * 1000);
		const int64_t timestamp = static_cast<int64_t>(hrt_absolute_time());
		const int64_t measured_fc_boot_time = timestamp - fc_timestamp;

		const int64_t change_in_boot_time = math::abs_t(measured_fc_boot_time - _fc_boot_time);

		constexpr int64_t reboot_change_threshold = 10_s;

		if (change_in_boot_time >= reboot_change_threshold) {
			if (static_cast<FC_STATE>(_aviant_ats.fc_state) == FC_STATE::ARMED) {
				PX4_WARN("Reboot and armed (dt = %ds)", static_cast<int>(change_in_boot_time / 1_s));
				_aviant_ats.fc_rebooted_while_armed = true;

			} else {
				PX4_WARN("Reboot but not armed (dt = %ds)", static_cast<int>(change_in_boot_time / 1_s));
			}

			// Never reset, this will only be true for one sample, but we want it to latch
		}

		_fc_boot_time = measured_fc_boot_time;

		if (ext_fc_state.system_status == external_aviant_detailed_fc_state_s::SYSTEM_STATUS_FLIGHT_TERMINATION) {
			_fc_state = FC_STATE::TERMINATED;

		} else if (ext_fc_state.armed) {
			_fc_state = FC_STATE::ARMED;

		} else {
			_fc_state = FC_STATE::DISARMED;
		}

		_aviant_ats.fc_state = static_cast<uint8_t>(_fc_state);
	}

	if (hrt_elapsed_time(&_last_sign_of_life_from_fc) > (_params_av_ats_timeout.get() * 1000ULL)) {
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

	vehicle_attitude_s vehicle_attitude{};

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {

		matrix::Quatf q(vehicle_attitude.q);
		matrix::Eulerf euler(q);

		_ats_roll  = math::degrees(euler.phi());
		_ats_pitch = math::degrees(euler.theta());

		_aviant_ats.roll_fail = (fabsf(_ats_roll) > _params_av_ats_roll_ang.get());
		_aviant_ats.pitch_fail = (fabsf(_ats_pitch) > _params_av_ats_pitch_ang.get());
	}

	const bool ats_active = static_cast<bool>(_params_av_ats_active.get());

	if (ats_active) {
		const bool failure_detected = (
						      _aviant_ats.roll_fail
						      || _aviant_ats.pitch_fail
						      || _aviant_ats.accel_norm_fail
					      );

		const bool should_trigger_on_failure = (
				(_fc_state == FC_STATE::ARMED && _aviant_ats.fc_timeout)
				|| _aviant_ats.fc_rebooted_while_armed
						       );

		if (
			(should_trigger_on_failure && failure_detected)
			|| _fc_state == FC_STATE::TERMINATED
		) {
			_aviant_ats.parachute_deploy = true;
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
	printf("FC Timestamp: %lld\n", _last_sign_of_life_from_fc);
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
