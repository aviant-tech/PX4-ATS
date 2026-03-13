#include "AtsVoltageMeasurement.hpp"

AtsVoltageMeasurement::AtsVoltageMeasurement() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

AtsVoltageMeasurement::~AtsVoltageMeasurement()
{
	ScheduleClear();
}

bool AtsVoltageMeasurement::init()
{
	// Needs to be significantly quicker than the time it takes parachute capacitors to discharge.
	// See https://aviant.atlassian.net/wiki/x/AQDdew
	ScheduleOnInterval(10_ms);
	return true;
}

void AtsVoltageMeasurement::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	ats_voltage_measurements_s msg{};

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	updateParams();
	msg.main_power1_v = _param_mp1_sim.get();
	msg.main_power2_v = _param_mp2_sim.get();
	msg.ats_ups_v     = _param_ups_sim.get();
#else
	adc_report_s adc {};

	if (!_adc_report_sub.update(&adc)) {
		return;
	}

	msg.main_power1_v = channel_voltage(adc, _param_mp1_ch.get(), _param_mp1_div.get());
	msg.main_power2_v = channel_voltage(adc, _param_mp2_ch.get(), _param_mp2_div.get());
	msg.ats_ups_v     = channel_voltage(adc, _param_ups_ch.get(), _param_ups_div.get());
#endif

	msg.timestamp = hrt_absolute_time();
	_voltage_pub.publish(msg);
}

#ifndef CONFIG_ARCH_BOARD_PX4_SITL
float AtsVoltageMeasurement::channel_voltage(const adc_report_s &adc, int32_t channel, float divider)
{
	for (unsigned i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); i++) {
		if (adc.channel_id[i] == channel) {
			const float lsb = adc.v_ref / static_cast<float>(adc.resolution);
			const float adc_voltage = static_cast<float>(adc.raw_data[i]) * lsb;
			const float measured_voltage  = adc_voltage * divider;
			return measured_voltage;
		}
	}

	return 0.0f;
}
#endif

int AtsVoltageMeasurement::task_spawn(int argc, char *argv[])
{
	AtsVoltageMeasurement *instance = new AtsVoltageMeasurement();

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

int AtsVoltageMeasurement::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AtsVoltageMeasurement::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Reads ADC voltages and publishes ats_voltage_measurements.
In SITL, publishes configurable simulated voltages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ats_voltage_measurement", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ats_voltage_measurement_main(int argc, char *argv[]);

int ats_voltage_measurement_main(int argc, char *argv[])
{
	return AtsVoltageMeasurement::main(argc, argv);
}
