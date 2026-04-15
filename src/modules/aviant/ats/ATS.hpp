#pragma once

#include <cstdint>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/aviant_ats.h>
#include <uORB/topics/external_battery_status.h>
#include <uORB/topics/external_aviant_detailed_fc_state.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

using namespace time_literals;

class ATS : public ModuleBase<ATS>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ATS();
	~ATS() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

	void send_parachute_command();
	void send_flighttermination_command();

private:

	aviant_ats_s _previous_ats_state{0};

	uint8_t check_for_acks();

	aviant_ats_fc_check_s check_fc_state(uint8_t &internal_failure_flags);
	bool check_acceleration(uint8_t &internal_failure_flags);
	aviant_ats_attitude_check_s check_attitude(uint8_t &internal_failure_flags);
	aviant_ats_voltage_check_s check_voltages(uint8_t &internal_failure_flags);
	float get_fc_battery_voltage(uint8_t &ups_status_flags);

	static float channel_voltage(const adc_report_s &adc, int32_t channel, float divider);

	systemlib::Hysteresis _deployment_hysteresis{false};

	hrt_abstime _last_flighttermination_sent{0};
	hrt_abstime _last_parachute_sent{0};

	hrt_abstime _flighttermination_interval{10_ms};
	hrt_abstime _parachute_interval{10_ms};

	uORB::Publication<aviant_ats_s> _aviant_ats_pub{ORB_ID(aviant_ats)};

	uORB::Subscription _adc_report_sub{ORB_ID(adc_report)};
	uORB::Subscription _ext_detailed_fc_state_sub{ORB_ID(external_aviant_detailed_fc_state)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Subscription _external_battery_status_sub{ORB_ID(external_battery_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AV_ATS_TIMEOUT>)     _params_av_ats_timeout,
		(ParamFloat<px4::params::AV_ATS_ACC_NORM>)  _params_av_ats_acc_norm,
		(ParamFloat<px4::params::AV_ATS_ROLL_ANG>)  _params_av_ats_roll_ang,
		(ParamFloat<px4::params::AV_ATS_PITCH_ANG>) _params_av_ats_pitch_ang,
		(ParamInt<px4::params::AV_ATS_EN>)          _params_av_ats_en,
		(ParamFloat<px4::params::AV_ATS_MP_LOWV>)   _params_av_ats_mp_lowv,
		(ParamFloat<px4::params::AV_ATS_PARA_LOWV>) _params_av_ats_para_lowv,
		(ParamFloat<px4::params::AV_ATS_UPS_LOWV>)  _params_av_ats_ups_lowv,
		(ParamInt<px4::params::AV_ATS_V_EN>)        _params_av_ats_v_en,
		(ParamFloat<px4::params::AV_ATS_TTRI>)      _params_av_ats_ttri,
		(ParamInt<px4::params::AV_ATS_MP1_CH>)      _param_mp1_ch,
		(ParamFloat<px4::params::AV_ATS_MP1_DV>)    _param_mp1_div,
		(ParamInt<px4::params::AV_ATS_MP2_CH>)      _param_mp2_ch,
		(ParamFloat<px4::params::AV_ATS_MP2_DV>)    _param_mp2_div,
		(ParamInt<px4::params::AV_ATS_UPS_CH>)      _param_ups_ch,
		(ParamFloat<px4::params::AV_ATS_UPS_DV>)    _param_ups_div,
		(ParamInt<px4::params::AV_ATS_PARA_CH>)       _param_para_ch,
		(ParamFloat<px4::params::AV_ATS_PARA_DV>)     _param_para_div,
		(ParamFloat<px4::params::AV_ATS_BAT_V_TOL>)   _param_av_ats_bat_v_tol,
		(ParamInt<px4::params::AV_ATS_BAT_TOUT>) 	_param_av_ats_bat_tout,
		(ParamInt<px4::params::MAV_SYS_ID>)         _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>)        _param_mav_comp_id
	);
};
