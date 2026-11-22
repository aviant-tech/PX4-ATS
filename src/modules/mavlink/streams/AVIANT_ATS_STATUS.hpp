#ifndef AVIANT_ATS_STATUS_HPP
#define AVIANT_ATS_STATUS_HPP

#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/topics/aviant_ats.h>

class MavlinkStreamAviantAtsStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAviantAtsStatus(mavlink); }

	static constexpr const char *get_name_static() { return "AVIANT_ATS_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVIANT_ATS_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AVIANT_ATS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamAviantAtsStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _aviant_ats_sub{ORB_ID::aviant_ats};
	aviant_ats_s _ats{};
	bool _ats_updated{false};
	uint32_t _flags_since_send{0};

	// Called every mavlink loop iteration: collect the flags of every ATS sample, so a flag that is
	// set for a few ms between two 5 Hz messages is still reported
	void update_data() override
	{
		if (!_aviant_ats_sub.update(&_ats)) {
			return;
		}

		uint32_t flags = 0;

		if (_ats.accel_norm_fail)              { flags |= AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL; }

		if (_ats.attitude.roll_fail)           { flags |= AVIANT_ATS_STATUS_FLAG_ROLL_FAIL; }

		if (_ats.attitude.pitch_fail)          { flags |= AVIANT_ATS_STATUS_FLAG_PITCH_FAIL; }

		if (_ats.fc_timeout)                   { flags |= AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT; }

		if (_ats.fc.rebooted_while_armed)      { flags |= AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED; }

		if (_ats.parachute_deploy)             { flags |= AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY; }

		if (_ats.voltage.main_voltage_fail)    { flags |= AVIANT_ATS_STATUS_FLAG_POWER_LOSS; }

		if (_ats.voltage.ups_status_flags != 0)         { flags |= AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY; }

		if (_ats.internal_failure_flags != 0)  { flags |= AVIANT_ATS_STATUS_FLAG_INTERNAL_FAILURE; }

		_flags_since_send |= flags;
		_ats_updated = true;
	}

	bool send() override
	{
		if (_mavlink->get_free_tx_buf() >= get_size() && _ats_updated) {

			mavlink_msg_aviant_ats_status_send(
				_mavlink->get_channel(),
				_ats.timestamp / 1000U,
				_ats.fc.armed,
				_ats.fc.flight_termination,
				_flags_since_send,
				_ats.power_loss_trigger_enabled,
				_ats.ats_enabled,
				_ats.voltage.main_power1_v,
				_ats.voltage.main_power2_v,
				_ats.voltage.ups_v
			);

			_flags_since_send = 0;
			_ats_updated = false;
			return true;
		}

		return false;
	}
};

#endif // AVIANT_ATS_STATUS_HPP
