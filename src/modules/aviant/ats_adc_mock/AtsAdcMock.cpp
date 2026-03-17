#include "AtsAdcMock.hpp"
#include "drivers/drv_hrt.h"

AtsAdcMock::AtsAdcMock() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

AtsAdcMock::~AtsAdcMock()
{
	ScheduleClear();
}

bool AtsAdcMock::init()
{
	ScheduleOnInterval(10_ms);
	return true;
}

int32_t AtsAdcMock::raw_from_voltage(float voltage, float divider)
{
	if (divider < 1e-6f) {
		return 0;
	}

	const float lsb = V_REF / static_cast<float>(RESOLUTION);
	return static_cast<int32_t>((voltage / divider) / lsb);
}

void AtsAdcMock::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	updateParams();

	adc_report_s adc{};
	adc.v_ref = V_REF;
	adc.resolution = RESOLUTION;

	for (auto &ch : adc.channel_id) {
		ch = -1;
	}

	const struct {
		int32_t channel;
		float voltage;
		float divider;
	} channels[] = {
		{ _param_mp1_ch.get(), _param_mp1_sim.get(), _param_mp1_div.get() },
		{ _param_mp2_ch.get(), _param_mp2_sim.get(), _param_mp2_div.get() },
		{ _param_ups_ch.get(), _param_ups_sim.get(), _param_ups_div.get() },
	};

	unsigned idx = 0;

	for (const auto &ch : channels) {
		if (ch.channel >= 0 && idx < sizeof(adc.channel_id) / sizeof(adc.channel_id[0])) {
			adc.channel_id[idx] = static_cast<int16_t>(ch.channel);
			adc.raw_data[idx] = raw_from_voltage(ch.voltage, ch.divider);
			idx++;
		}
	}

	adc.timestamp = hrt_absolute_time();
	_adc_pub.publish(adc);
}

int AtsAdcMock::task_spawn(int argc, char *argv[])
{
	AtsAdcMock *instance = new AtsAdcMock();

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

int AtsAdcMock::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AtsAdcMock::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes simulated adc_report messages for SITL testing.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ats_adc_mock", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ats_adc_mock_main(int argc, char *argv[]);

int ats_adc_mock_main(int argc, char *argv[])
{
	return AtsAdcMock::main(argc, argv);
}
