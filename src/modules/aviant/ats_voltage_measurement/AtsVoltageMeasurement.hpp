#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/ats_voltage_measurements.h>

#ifndef CONFIG_ARCH_BOARD_PX4_SITL
#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>
#endif

using namespace time_literals;

class AtsVoltageMeasurement : public ModuleBase<AtsVoltageMeasurement>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	AtsVoltageMeasurement();
	~AtsVoltageMeasurement() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;

private:
#ifndef CONFIG_ARCH_BOARD_PX4_SITL
	float channel_voltage(const adc_report_s &adc, int32_t channel, float divider);
	uORB::Subscription _adc_report_sub{ORB_ID(adc_report)};
#endif

	uORB::Publication<ats_voltage_measurements_s> _voltage_pub{ORB_ID(ats_voltage_measurements)};

	DEFINE_PARAMETERS(
#ifdef CONFIG_ARCH_BOARD_PX4_SITL
		(ParamFloat<px4::params::AV_V_MP1_SIM>) _param_mp1_sim,
		(ParamFloat<px4::params::AV_V_MP2_SIM>) _param_mp2_sim,
		(ParamFloat<px4::params::AV_V_UPS_SIM>) _param_ups_sim
#else
		(ParamInt<px4::params::AV_V_MP1_CH>)  _param_mp1_ch,
		(ParamFloat<px4::params::AV_V_MP1_DIV>) _param_mp1_div,
		(ParamInt<px4::params::AV_V_MP2_CH>)  _param_mp2_ch,
		(ParamFloat<px4::params::AV_V_MP2_DIV>) _param_mp2_div,
		(ParamInt<px4::params::AV_V_UPS_CH>)  _param_ups_ch,
		(ParamFloat<px4::params::AV_V_UPS_DIV>) _param_ups_div
#endif
	);
};
