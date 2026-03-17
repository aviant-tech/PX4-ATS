#include "ATS.hpp"
#include "drivers/drv_hrt.h"
#include "uORB/topics/aviant_ats.h"
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

	ScheduleOnInterval(5_ms);

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


		const int64_t fc_timestamp = static_cast<int64_t>(ext_fc_state.time_boot_ms * 1000);
		const int64_t ats_timestamp = static_cast<int64_t>(ext_fc_state.timestamp);
		const int64_t fc_boot_timestamp = ats_timestamp - fc_timestamp;

		constexpr int64_t reboot_detection_threshold = 10_s;

		if (fc_boot_timestamp > _last_fc_boot_timestamp + reboot_detection_threshold) {
			if (_last_fc_state == aviant_ats_s::FC_STATE_ARMED) {
				PX4_WARN("Reboot detected while armed!");
				_aviant_ats.fc_rebooted_while_armed = true;

			} else {
				PX4_INFO("Reboot detected, but not armed");
			}

			// Never reset, this will only be true for one sample, but we want it to latch
		}

		if (ext_fc_state.system_status == external_aviant_detailed_fc_state_s::SYSTEM_STATUS_FLIGHT_TERMINATION) {
			_aviant_ats.fc_state = aviant_ats_s::FC_STATE_TERMINATED;

		} else if (ext_fc_state.armed) {
			_aviant_ats.fc_state = aviant_ats_s::FC_STATE_ARMED;

		} else {
			_aviant_ats.fc_state = aviant_ats_s::FC_STATE_DISARMED;
		}

		_last_fc_boot_timestamp = fc_boot_timestamp;
		_last_fc_state = _aviant_ats.fc_state;
		_last_sign_of_life_from_fc = ext_fc_state.timestamp;
	}

	if (hrt_elapsed_time(&_last_sign_of_life_from_fc) > (_params_av_ats_timeout.get() * 1000ULL)) {
		_aviant_ats.fc_timeout = true;

	} else {
		_aviant_ats.fc_timeout = false;
	}

	vehicle_acceleration_s vehicle_acceleration;

	if (_vehicle_acceleration_sub.update(&vehicle_acceleration)) {

		const float accel_norm = sqrtf(
						 vehicle_acceleration.xyz[0] * vehicle_acceleration.xyz[0] +
						 vehicle_acceleration.xyz[1] * vehicle_acceleration.xyz[1] +
						 vehicle_acceleration.xyz[2] * vehicle_acceleration.xyz[2]
					 );


		_aviant_ats.accel_norm_fail = accel_norm < _params_av_ats_acc_norm.get();
	}

	vehicle_attitude_s vehicle_attitude{};

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {

		const matrix::Quatf q(vehicle_attitude.q);
		const matrix::Eulerf euler(q);

		const float ats_roll  = math::degrees(euler.phi());
		const float ats_pitch = math::degrees(euler.theta());

		_aviant_ats.roll_fail = (fabsf(ats_roll) > _params_av_ats_roll_ang.get());
		_aviant_ats.pitch_fail = (fabsf(ats_pitch) > _params_av_ats_pitch_ang.get());
	}

	ats_voltage_measurements_s voltage{};

	if (_ats_voltage_sub.update(&voltage)) {
		_aviant_ats.main_voltage_fail = (voltage.main_power1_v < _params_av_ats_mp_lowv.get())
						&& (voltage.main_power2_v < _params_av_ats_mp_lowv.get());
		_aviant_ats.ups_healthy = voltage.ats_ups_v > _params_av_ats_ups_lowv.get();
	}

	// Note: Please use only variables from the _aviant_ats message to deploy the parachute,
	// Being able to read the entire state from one ulog sample makes troubleshooting easier

	const bool control_failure = (
					     _aviant_ats.roll_fail
					     || _aviant_ats.pitch_fail
					     || _aviant_ats.accel_norm_fail
				     );
	const bool fc_is_supposed_to_be_armed_but_is_untrustworthy = (
				(_aviant_ats.fc_state == aviant_ats_s::FC_STATE_ARMED && _aviant_ats.fc_timeout)
				|| _aviant_ats.fc_rebooted_while_armed
			);

	_aviant_ats.maybe_parachute_deploy = (
			(control_failure && fc_is_supposed_to_be_armed_but_is_untrustworthy)
			|| _aviant_ats.fc_state == aviant_ats_s::FC_STATE_TERMINATED
					     );


	_deployment_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _params_av_ats_ttri.get()));
	_deployment_hysteresis.set_state_and_update(_aviant_ats.maybe_parachute_deploy, hrt_absolute_time());
	_aviant_ats.parachute_deploy = _deployment_hysteresis.get_state();

	// We don't have time to wait for the hysteresis in a power loss scenario,
	// since the parachute capacitor can discharge in as little as 30ms.
	// See: https://aviant.atlassian.net/wiki/x/AQDdew
	if (
		(
			_params_av_ats_v_en.get()
			&& _aviant_ats.ups_healthy
			&& _aviant_ats.main_voltage_fail
		)
		&& (
			_aviant_ats.fc_state != aviant_ats_s::FC_STATE_DISARMED
			|| _aviant_ats.fc_rebooted_while_armed
		)
	) {
		if (!_parachute_command_sent) {
			PX4_WARN("Power loss detected, deploying immediately!");
		}

		_aviant_ats.parachute_deploy = true;
	}

	if (_aviant_ats.parachute_deploy) {
		if (!_parachute_command_sent) {
			if (_params_av_ats_active.get()) {
				// Send multiple messages in case the link is bad.
				// We have experienced corrupted messages before
				for (int i = 0; i < 5; i++) {
					send_parachute_command();
					send_flighttermination_command();
				}

			} else {
				PX4_WARN("Would have deployed, but is inactive");
			}

			_parachute_command_sent = true;
		}

	} else {
		_parachute_command_sent = false;
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

	printf("FC State: %d\n", _last_fc_state);
	printf("FC Timestamp: %lld\n", _last_sign_of_life_from_fc);
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
