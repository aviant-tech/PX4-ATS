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
#include <uORB/topics/external_aviant_detailed_fc_state.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>

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

	float channel_voltage(const adc_report_s &adc, int32_t channel, float divider);

	aviant_ats_s _aviant_ats{};

	hrt_abstime _last_sign_of_life_from_fc{0};
	bool _last_fc_armed{false};
	bool _ups_has_been_healthy{false};
	bool _latch_ups_unhealthy{false};

	// start at max, so that the first delta is negative.
	// otherwise the first ts may be interpreted as a "reboot" if the time from ats boot to first message is large
	int64_t _last_fc_boot_timestamp{INT64_MAX};

	systemlib::Hysteresis _deployment_hysteresis{false};
	systemlib::Hysteresis _ups_healthy_hysteresis{false};
	bool _parachute_command_sent{false};

	uORB::Publication<aviant_ats_s> _aviant_ats_pub{ORB_ID(aviant_ats)};

	uORB::Subscription _adc_report_sub{ORB_ID(adc_report)};
	uORB::Subscription _ext_detailed_fc_state_sub{ORB_ID(external_aviant_detailed_fc_state)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AV_ATS_TIMEOUT>)     _params_av_ats_timeout,
		(ParamFloat<px4::params::AV_ATS_ACC_NORM>)  _params_av_ats_acc_norm,
		(ParamFloat<px4::params::AV_ATS_ROLL_ANG>)  _params_av_ats_roll_ang,
		(ParamFloat<px4::params::AV_ATS_PITCH_ANG>) _params_av_ats_pitch_ang,
		(ParamInt<px4::params::AV_ATS_EN>)          _params_av_ats_en,
		(ParamFloat<px4::params::AV_ATS_MP_LOWV>)   _params_av_ats_mp_lowv,
		(ParamFloat<px4::params::AV_ATS_UPS_LOWV>)  _params_av_ats_ups_lowv,
		(ParamInt<px4::params::AV_ATS_V_EN>)        _params_av_ats_v_en,
		(ParamFloat<px4::params::AV_ATS_TTRI>)      _params_av_ats_ttri,
		(ParamInt<px4::params::AV_ATS_MP1_CH>)      _param_mp1_ch,
		(ParamFloat<px4::params::AV_ATS_MP1_DV>)    _param_mp1_div,
		(ParamInt<px4::params::AV_ATS_MP2_CH>)      _param_mp2_ch,
		(ParamFloat<px4::params::AV_ATS_MP2_DV>)    _param_mp2_div,
		(ParamInt<px4::params::AV_ATS_UPS_CH>)      _param_ups_ch,
		(ParamFloat<px4::params::AV_ATS_UPS_DV>)    _param_ups_div,
		(ParamInt<px4::params::MAV_SYS_ID>)         _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>)        _param_mav_comp_id
	);
};
