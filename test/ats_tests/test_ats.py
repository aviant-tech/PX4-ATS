"""SITL integration tests for the ATS module.

Each test starts a fresh PX4 SITL instance.  ATS parameters are passed
via environment variables and applied in the airframe .post script
*before* the module starts (the module does not call updateParams at
runtime).

SIH provides simulated sensor data.  The sensors module derives
vehicle_acceleration (~9.81 m/s^2) and EKF2 publishes vehicle_attitude
(~0 deg roll/pitch) once tilt alignment completes.
Sensor-dependent trigger conditions are exercised by adjusting parameter thresholds
rather than manipulating physics.
"""

import time

import pytest
from ats_tester import FCMock, ParachuteMock, mavlink


def assert_ats_status(fc: FCMock, expected_flags: int,
                      expected_enabled_status: bool,
                      expected_powerloss_enabled_status: bool,
                      expected_fc_armed: bool) -> None:
    """Validate AVIANT_ATS_STATUS flags and fc_state match expectations."""
    status = fc.get_ats_status(timeout_s=3.0)
    assert status is not None, "Did not receive AVIANT_ATS_STATUS message"

    assert expected_enabled_status == status.ats_enabled, f"Expected ats_enabled={expected_enabled_status}"
    assert expected_powerloss_enabled_status == status.power_loss_trigger_enabled, f"Expected power_loss_trigger_enabled={expected_powerloss_enabled_status}"

    actual_flags = status.ats_status_flags
    assert actual_flags == expected_flags, (
        f"ATS flags mismatch: "
        f"expected {fc.flags_str(expected_flags)} ({expected_flags:#x}), "
        f"got {fc.flags_str(actual_flags)} ({actual_flags:#x})"
    )

    assert status.fc_armed == expected_fc_armed, (
        f"fc_armed mismatch: expected {expected_fc_armed}, got {status.fc_armed}"
    )


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_timeout_accel_fail(fc: FCMock, parachute: ParachuteMock):
    """Deploy on ARMED + fc_timeout + accel_norm_fail (threshold > gravity)."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.pause_sending()

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '80.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_timeout_roll_fail(fc: FCMock, parachute: ParachuteMock):
    """Deploy on ARMED + fc_timeout + roll_fail (threshold = 0 deg).

    EKF2 publishes vehicle_attitude at near-level orientation in SIH.
    A 0-degree threshold means any non-zero roll triggers roll_fail.
    """

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.pause_sending()

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_timeout_pitch_fail(fc: FCMock, parachute: ParachuteMock):
    """Deploy on ARMED + fc_timeout + pitch_fail (threshold = 0 deg).

    EKF2 publishes vehicle_attitude at near-level orientation in SIH.
    A 0-degree threshold means any non-zero pitch triggers pitch_fail.
    """

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.pause_sending()

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_reboot_armed_sensor_fail(fc: FCMock, parachute: ParachuteMock):
    """Deploy when FC reboots while armed and a sensor-fail condition is met."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.simulate_fc_reboot()

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=False)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_voltage_main_low_ups_healthy(fc: FCMock, parachute: ParachuteMock):
    """Deploy when both main powers drop below threshold while UPS stays healthy."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.set_ats_param('AV_ATS_MP1_SM', 10.0)
        fc.set_ats_param('AV_ATS_MP2_SM', 10.0)

    assert_ats_status(fc,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY
                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS,
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '0',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '10.0',
    'PARAM_AV_ATS_MP2_SM':     '10.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_disabled(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC is DISABLED, even with all failures set."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        pass

    # All sensor fails + voltage fail -> parachute_deploy is set in the uORB
    # message, but no command is sent because ATS is disabled.
    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_enabled_status=False,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '10.0',
    'PARAM_AV_ATS_MP2_SM':     '10.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_disarmed(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC is DISARMED, even with all fail flags set."""

    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        pass

    # Sensor fail flags are set, but disarmed blocks both the control-failure
    # path and the voltage path -> no PARACHUTE_DEPLOY.
    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=False)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_timeout_no_sensor_fail(fc: FCMock, parachute: ParachuteMock):
    """No deploy on ARMED + fc_timeout when no sensor-fail condition is met."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        fc.pause_sending()

    assert_ats_status(fc,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT,
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '30000',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_sensor_fail_no_timeout(fc: FCMock, parachute: ParachuteMock):
    """No deploy on ARMED + sensor-fail when fc_timeout has not fired.

    Sensor thresholds are set so all sensor-fail flags are true,
    but fc_timeout is set very high (30 s),
    so the timeout condition is never met.
    """

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        fc.pause_sending()
        pass

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_reboot_armed_no_sensor_fail(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC reboots while armed but no sensor-fail condition is met."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        fc.simulate_fc_reboot()

    assert_ats_status(fc,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED,
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=False)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_reboot_disarmed(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC reboots while disarmed, even with failure conditions.
    """

    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        fc.simulate_fc_reboot()

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=False)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_reboot_small_time_drop(fc: FCMock, parachute: ParachuteMock):
    """No deploy when time_boot_ms drops by less than the 10 s threshold."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        fc.boot_timestamp_s += 5

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_voltage_main_low_ups_unhealthy(fc: FCMock, parachute: ParachuteMock):
    """No deploy when all voltages (including UPS) drop simultaneously.

    If the UPS voltage also drops below its threshold (<=4V), the voltage
    readings are considered unreliable (e.g. sensor/ADC issue) and the
    parachute must not fire.
    """

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        # Set UPS low first to avoid a race where main powers are seen as low
        # while UPS is still healthy (which would trigger voltage_fail).
        # This is realistic, we will have a margin on the voltage threshold
        # ensuring measurements for a short while after the failure is detected
        fc.set_ats_param('AV_ATS_UPS_SM', 3.0)
        time.sleep(0.2)  # must be more than 0.1, otherwise test becomes flaky
        fc.set_ats_param('AV_ATS_MP1_SM', 10.0)
        fc.set_ats_param('AV_ATS_MP2_SM', 10.0)

    # Power loss should be flagged, but not deploy parachute since UPS is unhealthy
    assert_ats_status(fc,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY
                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS,
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_proxy_ack_flighttermination(fc: FCMock, parachute: ParachuteMock):
    """Proxy-ack DO_FLIGHTTERMINATION targeting compid 1 after parachute deploy."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.pause_sending()

    with parachute.expect_command_ack(
        command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        result=mavlink.MAV_RESULT_ACCEPTED,
        source_component=1,
        timeout_s=3.0,
    ), fc.expect_flighttermination(timeout_s=3.0):
        parachute.send_flighttermination_command(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_proxy_ack_force_disarm(fc: FCMock, parachute: ParachuteMock):
    """Proxy-ack force disarm targeting compid 1 after parachute deploy."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.pause_sending()

    with parachute.expect_command_ack(
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_ACCEPTED,
        source_component=1,
        timeout_s=3.0,
    ), fc.expect_force_disarm(timeout_s=3.0):
        parachute.send_force_disarm(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_flighttermination_before_deploy(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack for DO_FLIGHTTERMINATION when parachute has not deployed."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=2.0,
    ), fc.expect_flighttermination(timeout_s=2.0):
        parachute.send_flighttermination_command(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_force_disarm_before_deploy(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack for force disarm when parachute has not deployed."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=2.0,
    ), fc.expect_force_disarm(timeout_s=2.0):
        parachute.send_force_disarm(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_flighttermination_wrong_target(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack for DO_FLIGHTTERMINATION targeting something other than the FC"""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.pause_sending()

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=2.0,
    ):
        parachute.send_flighttermination_command(target_component=2)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_regular_disarm(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack for a regular (non-force) disarm after parachute deploy.
    """

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.pause_sending()

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=2.0,
    ):
        parachute.send_disarm(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '0',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '10.0',
    'PARAM_AV_ATS_MP2_SM':    '10.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_disabled(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack when ATS is disabled, even though deploy condition is met internally."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=3.0):
        pass

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=2.0,
    ):
        parachute.send_flighttermination_command(target_component=1)

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=2.0,
    ):
        parachute.send_force_disarm(target_component=1)


@pytest.mark.slow
@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '600000',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LOWV':   '15.0',
    'PARAM_AV_ATS_UPS_LOWV':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
    'PX4_SIM_SPEED_FACTOR':   '1000',
}], indirect=True)
def test_armed_12h(fc: FCMock, parachute: ParachuteMock):
    """No false-positive deploy after 12 simulated hours armed.

    Inject failures in acc/roll/pitch, but it should not deploy since there is no timeout.
    Timeout is adjusted for sim speed.

    Runs PX4 at 1000x real-time so 12 h of simulated flight time
    passes in ~90 s of wall-clock time.  Catches overflow bugs in
    timers or counters that could cause spurious parachute deployment.

    After the  12h, a genuine fault (power loss) is injected to
    confirm ATS still responds correctly.
    """

    fc.set_armed(True)
    time.sleep(0.1)

    wait_time_s = 12 * 60 * 60 / 1000  # 1000 is from PX4_SIM_SPEED_FACTOR
    with parachute.expect_no_deploy(duration_s=wait_time_s):
        pass

    assert_ats_status(fc,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                      expected_enabled_status=True,
                      expected_powerloss_enabled_status=True,
                      expected_fc_armed=True)

    with parachute.expect_deploy(timeout_s=5.0), fc.expect_flighttermination(timeout_s=5.0):
        fc.set_ats_param('AV_ATS_MP1_SM', 10.0)
        fc.set_ats_param('AV_ATS_MP2_SM', 10.0)
