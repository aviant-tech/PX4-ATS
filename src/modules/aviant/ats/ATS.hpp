#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
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

	enum class FC_STATE {
		DISARMED	= 0,
		ARMED		= 1,
		TERMINATED	= 2 //FAILSAFE
	} _fc_state{FC_STATE::DISARMED};

	static inline const char *fcStateToString(FC_STATE fc_state)
	{
		static constexpr const char *strings[] = {
			"DISARMED",
			"ARMED",
			"TERMINATED"
		};

		const uint8_t index = static_cast<uint8_t>(fc_state);

		if (index >= (sizeof(strings) / sizeof(strings[0]))) {
			return "UNKNOWN";
		}

		return strings[index];
	}

	static constexpr float DEG_360 = 360.0f;

	aviant_ats_s _aviant_ats{};

	hrt_abstime _last_sign_of_life_from_fc{0};
	int64_t _fc_boot_time{0};

	float _ats_roll{0.0f};	// degree
	float _ats_pitch{0.0f};	// degree

	bool _publish_vehicle_command_once{false};

	uORB::Publication<aviant_ats_s> _aviant_ats_pub{ORB_ID(aviant_ats)};

	uORB::Subscription _ext_detailed_fc_state_sub{ORB_ID(external_aviant_detailed_fc_state)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AV_ATS_TIMEOUT>)     _params_av_ats_timeout,
		(ParamFloat<px4::params::AV_ATS_ACC_NORM>)  _params_av_ats_acc_norm,
		(ParamFloat<px4::params::AV_ATS_ROLL_ANG>)  _params_av_ats_roll_ang,
		(ParamFloat<px4::params::AV_ATS_PITCH_ANG>) _params_av_ats_pitch_ang,
		(ParamInt<px4::params::AV_ATS_ACTIVE>)      _params_av_ats_active,
		(ParamInt<px4::params::MAV_SYS_ID>)         _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>)        _param_mav_comp_id
	);
};
