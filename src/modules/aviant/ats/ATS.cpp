#include "ATS.hpp"
#include "drivers/drv_hrt.h"
#include "uORB/topics/aviant_ats.h"
#include <cassert>
#pragma GCC diagnostic push
// MAVLink intentionally ignores alignment in some places
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink/aviant/mavlink.h"
#pragma GCC diagnostic pop
#include "uORB/topics/external_battery_status.h"
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

aviant_ats_fc_check_s
ATS::check_fc_state(uint8_t &internal_failure_flags)
{
	const aviant_ats_fc_check_s &prev = _previous_ats_state.fc;
	external_aviant_detailed_fc_state_s fc{};

	if (!_ext_detailed_fc_state_sub.copy(&fc)) {
		if (prev.ms_from_ats_boot_to_fc_boot != 0) {
			// This should never fail after we have an initial message
			internal_failure_flags |= aviant_ats_s::IFAIL_DETAILED_FC_STATE;
		}

		return prev;
	}

	aviant_ats_fc_check_s result{};

	const int32_t ms_since_fc_boot = static_cast<int32_t>(fc.time_boot_ms);
	const int32_t ms_since_ats_boot = static_cast<int32_t>(fc.timestamp / 1000);
	result.ms_from_ats_boot_to_fc_boot = ms_since_ats_boot - ms_since_fc_boot;

	// Reboot while armed is only observable for one sample, so we need to latch it
	result.rebooted_while_armed = prev.rebooted_while_armed;

	static constexpr int32_t reboot_detection_threshold_ms = 10000;

	if (
		prev.ms_from_ats_boot_to_fc_boot != 0 // initialization is not reboot
		&& result.ms_from_ats_boot_to_fc_boot > prev.ms_from_ats_boot_to_fc_boot +
		reboot_detection_threshold_ms
	) {
		if (prev.armed) {
			PX4_WARN("Reboot detected while armed!");
			result.rebooted_while_armed = true;

		} else {
			PX4_INFO("Reboot detected, but not armed");
		}
	}

	result.armed = fc.armed;
	result.flight_termination = fc.flight_termination;
	result.last_sign_of_life = fc.timestamp;

	return result;
}

bool
ATS::check_acceleration(uint8_t &internal_failure_flags)
{
	vehicle_acceleration_s accel;

	if (!_vehicle_acceleration_sub.copy(&accel)) {
		internal_failure_flags |= aviant_ats_s::IFAIL_VEHICLE_ACCELERATION;
		return _previous_ats_state.accel_norm_fail;
	}

	const float accel_norm = sqrtf(
					 accel.xyz[0] * accel.xyz[0] +
					 accel.xyz[1] * accel.xyz[1] +
					 accel.xyz[2] * accel.xyz[2]
				 );

	return accel_norm < _params_av_ats_acc_norm.get();
}

aviant_ats_attitude_check_s
ATS::check_attitude(uint8_t &internal_failure_flags)
{
	vehicle_attitude_s attitude{};

	if (!_vehicle_attitude_sub.copy(&attitude)) {
		internal_failure_flags |= aviant_ats_s::IFAIL_VEHICLE_ATTITUDE;
		return _previous_ats_state.attitude;
	}

	const matrix::Quatf q(attitude.q);
	const matrix::Eulerf euler(q);

	return {
		.roll_fail = fabsf(math::degrees(euler.phi())) > _params_av_ats_roll_ang.get(),
		.pitch_fail = fabsf(math::degrees(euler.theta())) > _params_av_ats_pitch_ang.get(),
	};
}

aviant_ats_voltage_check_s
ATS::check_voltages(uint8_t &internal_failure_flags)
{
	adc_report_s adc{};

	if (!_adc_report_sub.copy(&adc)) {
		internal_failure_flags |= aviant_ats_s::IFAIL_ADC_REPORT;
		return _previous_ats_state.voltage;
	}

	const float mp1_v = channel_voltage(adc, _param_mp1_ch.get(), _param_mp1_div.get());
	const float mp2_v = channel_voltage(adc, _param_mp2_ch.get(), _param_mp2_div.get());
	const float ups_v = channel_voltage(adc, _param_ups_ch.get(), _param_ups_div.get());
	const float parachute_v = channel_voltage(adc, _param_para_ch.get(), _param_para_div.get());

	aviant_ats_voltage_check_s result{};

	result.main_power1_v = mp1_v;
	result.main_power2_v = mp2_v;
	result.ups_v = ups_v;
	result.parachute_v = parachute_v;

	result.main_voltage_fail = (mp1_v < _params_av_ats_mp_lo_v.get())
				   && (mp2_v < _params_av_ats_mp_lo_v.get());

	if (_param_para_ch.get() >= 0) {
		result.parachute_voltage_fail = parachute_v < _params_av_ats_para_lo_v.get();

		// Only set the flag if we're actually measuring it
		if (result.parachute_voltage_fail) {
			result.ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_PARA_LOWV;
		}

	} else {
		result.parachute_voltage_fail = false;
	}


	result.fc_battery_v = get_fc_battery_voltage(result.ups_status_flags);

	const float mp1_diff_v = result.fc_battery_v - result.main_power1_v;
	const float mp2_diff_v = result.fc_battery_v - result.main_power2_v;

	if (_params_av_ats_ups_lo_v.get() > FLT_EPSILON && ups_v < _params_av_ats_ups_lo_v.get()) {
		result.ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_LOWV;
	}

	if (_param_ups_ch.get() >= 0 && _params_av_ats_ups_hi_v.get() > FLT_EPSILON
	    && ups_v > _params_av_ats_ups_hi_v.get()) {
		result.ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_UPS_HIGHV;
	}

	if (_param_para_ch.get() >= 0 && _params_av_ats_para_hi_v.get() > FLT_EPSILON
	    && parachute_v > _params_av_ats_para_hi_v.get()) {
		result.ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_PARA_HIGHV;
	}

	const float tol = _param_av_ats_bat_v_tol.get();

	if (tol > FLT_EPSILON) {
		if (fabsf(mp1_diff_v) > tol || fabsf(mp2_diff_v) > tol) {
			result.ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_FC_MISMATCH;
		}

		if (fabsf(result.main_power1_v - result.main_power2_v) > tol) {
			result.ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_MP_MISMATCH;
		}
	}

	return result;
}

float
ATS::get_fc_battery_voltage(uint8_t &ups_status_flags)
{
	external_battery_status_s bat{};

	if (!_external_battery_status_sub.copy(&bat)) {
		ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_FC_BATTERY_MISSING;
		return _previous_ats_state.voltage.fc_battery_v;
	}

	const int32_t timeout_ms = _param_av_ats_bat_tout.get();

	if (timeout_ms > 0) {
		const hrt_abstime max_age_us = static_cast<hrt_abstime>(timeout_ms) * 1000ULL;

		if (hrt_elapsed_time(&bat.timestamp) > max_age_us) {
			ups_status_flags |= aviant_ats_voltage_check_s::UPS_STATUS_FC_BATTERY_STALE;
			return _previous_ats_state.voltage.fc_battery_v;
		}
	}

	return bat.voltage_v;
}

uint8_t
ATS::check_for_acks()
{

	// latch these states
	uint8_t received_acks = _previous_ats_state.received_acks;

	if (
		received_acks & aviant_ats_s::RECEIVED_ACK_FLIGHTTERMINATION
		&& received_acks & aviant_ats_s::RECEIVED_ACK_PARACHUTE
	) {
		return received_acks; // Nothing to do
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
				received_acks |= aviant_ats_s::RECEIVED_ACK_FLIGHTTERMINATION;
			}

		} else if (ack.command == vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE
			   && ack.source_system == _param_mav_sys_id.get()
			   && ack.source_component == MAV_COMP_ID_PARACHUTE
			  ) {
			PX4_INFO("Parachute command acknowledged (%d)", ack.result);

			// We can potentially continue forever if the command is denied, but that is acceptable
			// since the aircraft is assumed in a failure state, we should just keep trying
			if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				received_acks |= aviant_ats_s::RECEIVED_ACK_PARACHUTE;
			}
		}
	}

	return received_acks;
}

void
ATS::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	aviant_ats_s ats_state{0};

	ats_state.fc              = check_fc_state(ats_state.internal_failure_flags);
	ats_state.accel_norm_fail = check_acceleration(ats_state.internal_failure_flags);
	ats_state.attitude        = check_attitude(ats_state.internal_failure_flags);
	ats_state.voltage         = check_voltages(ats_state.internal_failure_flags);

	ats_state.fc_timeout      = hrt_elapsed_time(&ats_state.fc.last_sign_of_life) > (_params_av_ats_timeout.get() *
				    1000ULL);

	ats_state.inflight_control_failure =
		(ats_state.attitude.roll_fail || ats_state.attitude.pitch_fail || ats_state.accel_norm_fail)
		&& ((ats_state.fc.armed && ats_state.fc_timeout) || ats_state.fc.rebooted_while_armed);

	ats_state.inflight_power_failure = (
			ats_state.voltage.main_voltage_fail
			&& (ats_state.fc.armed || ats_state.fc.rebooted_while_armed)
			// if parachute voltage is not also failed, we don't have a true power loss
			&& (ats_state.voltage.parachute_voltage_fail || _param_para_ch.get() == -1)
					   );

	_deployment_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _params_av_ats_ttri.get()));
	_deployment_hysteresis.set_state_and_update(ats_state.inflight_control_failure, hrt_absolute_time());

	ats_state.parachute_deploy = _previous_ats_state.parachute_deploy || _deployment_hysteresis.get_state();
	ats_state.power_loss_trigger_enabled = static_cast<bool>(_params_av_ats_v_en.get());

	// In case of power loss, we don't have time to wait for the hysteresis
	if (ats_state.power_loss_trigger_enabled && ats_state.inflight_power_failure) {
		ats_state.parachute_deploy = true;
	}

	ats_state.ats_enabled = static_cast<bool>(_params_av_ats_en.get());

	if (ats_state.parachute_deploy) {
		if (!_previous_ats_state.parachute_deploy) {
			PX4_WARN(ats_state.ats_enabled ? "Deploying parachute!" : "Would have deployed parachute, but is disabled");
		}

		if (ats_state.ats_enabled) {
			ats_state.received_acks = check_for_acks();

			if (!(ats_state.received_acks & aviant_ats_s::RECEIVED_ACK_FLIGHTTERMINATION)
			    && hrt_elapsed_time(&_last_flighttermination_sent) > _flighttermination_interval) {
				send_flighttermination_command();
				_last_flighttermination_sent = hrt_absolute_time();
				_flighttermination_interval = math::min(_flighttermination_interval * 2, (hrt_abstime)500_ms);
			}

			if (!(ats_state.received_acks & aviant_ats_s::RECEIVED_ACK_PARACHUTE)
			    && hrt_elapsed_time(&_last_parachute_sent) > _parachute_interval) {
				send_parachute_command();
				_last_parachute_sent = hrt_absolute_time();
				_parachute_interval = math::min(_parachute_interval * 2, (hrt_abstime)500_ms);
			}
		}

	} else {
		// Reset ack state if we're not deploying
		// so an accidental ack registration early on
		// doesn't latch and block parachute deployment later
		ats_state.received_acks = 0;
	}

	ats_state.timestamp = hrt_absolute_time();
	_aviant_ats_pub.publish(ats_state);
	_previous_ats_state = ats_state;
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
	PX4_INFO("Running, check uorb topic for state\n");
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
