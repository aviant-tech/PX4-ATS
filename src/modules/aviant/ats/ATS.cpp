#include "ATS.hpp"
#include "drivers/drv_hrt.h"
#pragma GCC diagnostic push
// MAVLink intentionally ignores alignment in some places
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink/aviant/mavlink.h"
#pragma GCC diagnostic pop
#include "uORB/topics/vehicle_command.h"
#include "uORB/topics/vehicle_command_ack.h"
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

	// 200hz is twice as fast as adc_report is expected, this is the minimum rate we should run at
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
			if (_last_fc_armed) {
				PX4_WARN("Reboot detected while armed!");
				_aviant_ats.fc_rebooted_while_armed = true;

			} else {
				PX4_INFO("Reboot detected, but not armed");
			}

			// Never reset, this will only be true for one sample, but we want it to latch
		}

		_aviant_ats.fc_armed = ext_fc_state.armed;
		_aviant_ats.fc_flight_termination = ext_fc_state.flight_termination;

		_last_fc_boot_timestamp = fc_boot_timestamp;
		_last_fc_armed = _aviant_ats.fc_armed;
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

	adc_report_s adc{};

	if (_adc_report_sub.update(&adc)) {
		_aviant_ats.main_power1_v = channel_voltage(adc, _param_mp1_ch.get(), _param_mp1_div.get());
		_aviant_ats.main_power2_v = channel_voltage(adc, _param_mp2_ch.get(), _param_mp2_div.get());
		_aviant_ats.ups_v         = channel_voltage(adc, _param_ups_ch.get(), _param_ups_div.get());

		_aviant_ats.main_voltage_fail = (_aviant_ats.main_power1_v < _params_av_ats_mp_lowv.get())
						&& (_aviant_ats.main_power2_v < _params_av_ats_mp_lowv.get());

		// The UPS has somewhat noisy measurements, use hysteresis to avoid unnecessary latching during boot
		// This is acceptable because the UPS is expected to fail (drain) slowly (it's a capacitor bank)
		_ups_healthy_hysteresis.set_hysteresis_time_from(false, 100_ms);
		_ups_healthy_hysteresis.set_hysteresis_time_from(true, 100_ms);
		_ups_healthy_hysteresis.set_state_and_update(
			_aviant_ats.ups_v > _params_av_ats_ups_lowv.get(),
			hrt_absolute_time()
		);

		if (!_ups_healthy_hysteresis.get_state() && _ups_has_been_healthy && !_latch_ups_unhealthy) {
			PX4_WARN("UPS unhealthy! (latching)");
			_latch_ups_unhealthy = true;
		}

		_aviant_ats.ups_healthy = _latch_ups_unhealthy ? false : _ups_healthy_hysteresis.get_state();
		_ups_has_been_healthy |= _aviant_ats.ups_healthy;
	}

	// Note: Please use only variables from the _aviant_ats message to deploy the parachute,
	// Being able to read the entire state from one ulog sample makes troubleshooting easier

	const bool control_failure = (
					     _aviant_ats.roll_fail
					     || _aviant_ats.pitch_fail
					     || _aviant_ats.accel_norm_fail
				     );
	const bool fc_is_supposed_to_be_armed_but_is_untrustworthy = (
				(_aviant_ats.fc_armed && _aviant_ats.fc_timeout)
				|| _aviant_ats.fc_rebooted_while_armed
			);

	_aviant_ats.maybe_parachute_deploy = (
			control_failure
			&& fc_is_supposed_to_be_armed_but_is_untrustworthy
					     );


	_deployment_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _params_av_ats_ttri.get()));
	_deployment_hysteresis.set_state_and_update(_aviant_ats.maybe_parachute_deploy, hrt_absolute_time());
	_aviant_ats.parachute_deploy = _deployment_hysteresis.get_state();

	_aviant_ats.power_loss_trigger_enabled = static_cast<bool>(_params_av_ats_v_en.get());

	// We don't have time to wait for the hysteresis in a power loss scenario,
	// since the parachute capacitor can discharge in as little as 30ms.
	// See: https://aviant.atlassian.net/wiki/x/AQDdew
	if (
		(
			_aviant_ats.power_loss_trigger_enabled
			&& _aviant_ats.ups_healthy
			&& _aviant_ats.main_voltage_fail
		)
		&& (
			_aviant_ats.fc_armed
			|| _aviant_ats.fc_rebooted_while_armed
		)
	) {
		if (!_parachute_deploy_warned) {
			PX4_WARN("Power loss detected, deploying immediately!");
		}

		_aviant_ats.parachute_deploy = true;
	}

	_aviant_ats.ats_enabled = static_cast<bool>(_params_av_ats_en.get());

	if (_aviant_ats.parachute_deploy) {
		if (_aviant_ats.ats_enabled) {
			if (!_parachute_deploy_warned) {
				PX4_WARN("Deploying parachute!");
				_parachute_deploy_warned = true;
			}

			vehicle_command_ack_s ack;

			// Typically, there are <20 acks per flight,
			// so allowing 20 every loop iteration should never fall behind
			static constexpr uint8_t max_acks_per_iteration = 20;
			uint8_t acks_checked_this_iteration = 0;

			while (_vehicle_command_ack_sub.update(&ack)) {
				if (++acks_checked_this_iteration > max_acks_per_iteration) {
					break;  // Avoid infinite loop in case of ack spam
				}

				if (
					ack.command == vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION
					&& ack.source_system == _param_mav_sys_id.get()
					&& ack.source_component == MAV_COMP_ID_AUTOPILOT1
				) {
					PX4_INFO("Flight termination command acknowledged (%d)", ack.result);

					// We can potentially continue forever if the command is denied, but that is acceptable
					// since the aircraft is assumed in a failure state, we should just keep trying
					if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
						_flighttermination_acked = true;
					}

				} else if (ack.command == vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE
					   && ack.source_system == _param_mav_sys_id.get()
					   && ack.source_component == MAV_COMP_ID_PARACHUTE
					  ) {
					PX4_INFO("Parachute command acknowledged (%d)", ack.result);

					// We can potentially continue forever if the command is denied, but that is acceptable
					// since the aircraft is assumed in a failure state, we should just keep trying
					if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
						_parachute_acked = true;
					}
				}
			}

			if (!_flighttermination_acked && hrt_elapsed_time(&_last_flighttermination_sent) > _flighttermination_interval) {
				send_flighttermination_command();
				_last_flighttermination_sent = hrt_absolute_time();
				_flighttermination_interval = math::min(_flighttermination_interval * 2, (hrt_abstime)500_ms);
			}

			if (!_parachute_acked && hrt_elapsed_time(&_last_parachute_sent) > _parachute_interval) {
				send_parachute_command();
				_last_parachute_sent = hrt_absolute_time();
				_parachute_interval = math::min(_parachute_interval * 2, (hrt_abstime)500_ms);
			}


		} else if (!_parachute_deploy_warned) {
			PX4_WARN("Would have deployed, but is inactive");
			_parachute_deploy_warned = true;
		}

	} else {
		_parachute_deploy_warned = false;
		_flighttermination_acked = false;
		_parachute_acked = false;
		_last_flighttermination_sent = 0;
		_last_parachute_sent = 0;
		_flighttermination_interval = DEFAULT_FLIGHTTERMINATION_INTERVAL;
		_parachute_interval = DEFAULT_PARACHUTE_INTERVAL;
	}

	_aviant_ats.flighttermination_acked = _flighttermination_acked;
	_aviant_ats.parachute_acked = _parachute_acked;

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
	vcmd.target_component = MAV_COMP_ID_PARACHUTE;

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
	vcmd.target_component = MAV_COMP_ID_AUTOPILOT1;

	uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vehicle_command_pub.publish(vcmd);

	PX4_INFO("Send FLIGHTTERMINATION CMD");
}

float ATS::channel_voltage(const adc_report_s &adc, int32_t channel, float divider)
{
	for (unsigned i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); i++) {
		if (adc.channel_id[i] == channel) {
			const float lsb = adc.v_ref / static_cast<float>(adc.resolution);
			const float adc_voltage = static_cast<float>(adc.raw_data[i]) * lsb;
			return adc_voltage * divider;
		}
	}

	return 0.0f;
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

	printf("FC armed: %d\n", _last_fc_armed);
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
