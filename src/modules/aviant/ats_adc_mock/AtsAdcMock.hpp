#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>

using namespace time_literals;

class AtsAdcMock : public ModuleBase<AtsAdcMock>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	AtsAdcMock();
	~AtsAdcMock() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;

private:
	static constexpr float V_REF = 3.3f;
	static constexpr uint32_t RESOLUTION = 4096;

	int32_t raw_from_voltage(float voltage, float divider);

	uORB::Publication<adc_report_s> _adc_pub{ORB_ID(adc_report)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AV_ATS_MP1_SM>) _param_mp1_sim,
		(ParamFloat<px4::params::AV_ATS_MP2_SM>) _param_mp2_sim,
		(ParamFloat<px4::params::AV_ATS_UPS_SM>) _param_ups_sim,
		(ParamFloat<px4::params::AV_ATS_PARA_SM>) _param_para_sim,
		(ParamInt<px4::params::AV_ATS_MP1_CH>)   _param_mp1_ch,
		(ParamFloat<px4::params::AV_ATS_MP1_DV>) _param_mp1_div,
		(ParamInt<px4::params::AV_ATS_MP2_CH>)   _param_mp2_ch,
		(ParamFloat<px4::params::AV_ATS_MP2_DV>) _param_mp2_div,
		(ParamInt<px4::params::AV_ATS_UPS_CH>)   _param_ups_ch,
		(ParamFloat<px4::params::AV_ATS_UPS_DV>) _param_ups_div,
		(ParamInt<px4::params::AV_ATS_PARA_CH>)    _param_para_ch,
		(ParamFloat<px4::params::AV_ATS_PARA_DV>)  _param_para_div
	);
};
