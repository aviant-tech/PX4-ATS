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


def wait_for_ats_status(fc: FCMock, expected_flags: int,
                        expected_enabled_status: bool,
                        expected_powerloss_enabled_status: bool,
                        expected_fc_armed: bool,
                        timeout_s: float,
                        drain_first: bool = True) -> None:
    """Poll AVIANT_ATS_STATUS until it matches expectations, or assert on the last one after `timeout_s`.

    Buffered messages are discarded once before polling if `drain_first`, never between polls.
    """
    deadline = time.monotonic() + timeout_s
    status = None
    drain = drain_first
    while (remaining := deadline - time.monotonic()) > 0:
        msg = fc.get_ats_status(timeout_s=remaining, drain=drain)
        drain = False
        if msg is None:
            break
        status = msg
        if (status.ats_status_flags == expected_flags
                and status.ats_enabled == expected_enabled_status
                and status.power_loss_trigger_enabled == expected_powerloss_enabled_status
                and status.fc_armed == expected_fc_armed):
            return

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
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_timeout_accel_fail(fc: FCMock, parachute: ParachuteMock):
    """Deploy on ARMED + fc_timeout + accel_norm_fail (threshold > gravity)."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY  # FC BATTERY_STATUS missing/stale
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=2.5)  # FC BATTERY_STATUS must time out first


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '80.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
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
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY  # FC BATTERY_STATUS missing/stale
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=2.5)  # FC BATTERY_STATUS must time out first


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
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
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY  # FC BATTERY_STATUS missing/stale
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=2.5)  # FC BATTERY_STATUS must time out first


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_reboot_armed_sensor_fail(fc: FCMock, parachute: ParachuteMock):
    """Deploy when FC reboots while armed and a sensor-fail condition is met."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.simulate_fc_reboot()

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=False,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_short_ups_glitch_is_reported(fc: FCMock):
    """A UPS glitch much shorter than the AVIANT_ATS_STATUS period is still reported."""

    fc.set_armed(True)

    wait_for_ats_status(fc,
                        expected_flags=0,
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)

    # Each glitch is 10ms, and the mavlink stream has a 200ms period.
    # so each one has about 1/20 chance of being detected "naturally".
    # The probability of 10 repetitions being detected naturally is basically 0
    for i in range(10):
        fc.mav.drain()
        fc.set_ats_param_int('AV_ATS_UPS_GLT', 1)

        # Don't drain here, it could discard the single message carrying the glitch
        wait_for_ats_status(fc,
                            expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0,
                            drain_first=False)

        # Power test.
        # If the status flag is still active for another 500ms then it's not a short glitch,
        # and this test is ineffective.
        wait_for_ats_status(fc,
                            expected_flags=0,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=0.5)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_deploy_voltage_main_low_and_ups_unhealthy(fc: FCMock, parachute: ParachuteMock):
    """Deploy on dual main low even if UPS is unhealthy."""

    fc.set_armed(True)
    time.sleep(0.1)

    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):

        # Low UPS voltage should trigger unhealthy
        fc.set_ats_param('AV_ATS_UPS_SM', 3.0)
        wait_for_ats_status(fc,
                            expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0
                            )

        # Back to healthy
        fc.set_ats_param('AV_ATS_UPS_SM', 5.0)
        wait_for_ats_status(fc,
                            expected_flags=0,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0
                            )


        # 30v battery is mismatch, UPS should be flagged unhealthy
        fc._BATTERY_VOLTAGE_MV = 30000
        wait_for_ats_status(fc,
                            expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.5  # BATTERY_STATUS is only 2hz
                            )

        # Back to healthy
        fc._BATTERY_VOLTAGE_MV = 50000
        wait_for_ats_status(fc,
                            expected_flags=0,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.5  # BATTERY_STATUS is only 2hz
                            )

        # Set parachute voltage low so it doesn't block deployment
        fc.set_ats_param('AV_ATS_PARA_SM', 18.0)

        # don't deploy when only one is low
        fc.set_ats_param('AV_ATS_MP1_SM', 10.0)

        # now, UPS is unhealthy since main power measurements disagree
        wait_for_ats_status(fc,
                            expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0
                            )

    # when the other goes low too, we deploy
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.set_ats_param('AV_ATS_MP2_SM', 10.0)

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_UPS_HI_V': '6.0',
    'PARAM_AV_ATS_PARA_HI_V': '25.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
    'PARAM_AV_ATS_PARA_SM':    '24.0',
}], indirect=True)
def test_ups_high_voltage_flags_only(fc: FCMock, parachute: ParachuteMock):
    """UPS / parachute over-voltage sets UPS_UNHEALTHY"""

    fc.set_armed(True)
    time.sleep(0.1)

    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        fc.set_ats_param('AV_ATS_UPS_SM', 10.0)
        wait_for_ats_status(fc,
                            expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0)

        fc.set_ats_param('AV_ATS_UPS_SM', 5.0)
        wait_for_ats_status(fc,
                            expected_flags=0,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0)

        fc.set_ats_param('AV_ATS_PARA_SM', 30.0)
        wait_for_ats_status(fc,
                            expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0)

        fc.set_ats_param('AV_ATS_PARA_SM', 24.0)
        wait_for_ats_status(fc,
                            expected_flags=0,
                            expected_enabled_status=True,
                            expected_powerloss_enabled_status=True,
                            expected_fc_armed=True,
                            timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_powerloss_deploy_requires_parachute_capacitor_low(fc: FCMock, parachute: ParachuteMock):
    """Mains below AV_ATS_MP_LO_V only triggers voltage deploy when parachute capacitor voltage is below AV_ATS_PARA_LO_V."""

    fc.set_armed(True)
    time.sleep(0.1)


    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        fc.set_ats_param('AV_ATS_MP1_SM', 10.0)
        fc.set_ats_param('AV_ATS_MP2_SM', 10.0)

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)

    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.set_ats_param('AV_ATS_PARA_SM', 18.0)

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '0',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '10.0',
    'PARAM_AV_ATS_MP2_SM':     '10.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
    'PARAM_AV_ATS_PARA_SM':    '18.0',
}], indirect=True)
def test_nodeploy_disabled(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC is DISABLED, even with all failures set."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        pass

    # All sensor fails + voltage fail -> parachute_deploy is set in the uORB
    # message, but no command is sent because ATS is disabled.
    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                        expected_enabled_status=False,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '10.0',
    'PARAM_AV_ATS_MP2_SM':     '10.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_disarmed(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC is DISARMED, even with all fail flags set."""

    fc._BATTERY_VOLTAGE_MV = 10000  # match measurements to avoid unhealthy UPS
    time.sleep(0.1)

    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        pass

    # Sensor fail + main low, but disarmed blocks control-failure and power-loss deploy.
    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=False,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_timeout_no_sensor_fail(fc: FCMock, parachute: ParachuteMock):
    """No deploy on ARMED + fc_timeout when no sensor-fail condition is met."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        fc.pause_sending()

    wait_for_ats_status(fc,
                        expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                        | mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,  # FC BATTERY_STATUS missing/stale
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=2.5)  # FC BATTERY_STATUS must time out first


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '30000',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
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
    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        fc.pause_sending()

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY  # FC BATTERY_STATUS missing/stale
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=2.5)  # FC BATTERY_STATUS must time out first


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_reboot_armed_no_sensor_fail(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC reboots while armed but no sensor-fail condition is met."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        fc.simulate_fc_reboot()

    wait_for_ats_status(fc,
                        expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED,
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=False,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_reboot_disarmed(fc: FCMock, parachute: ParachuteMock):
    """No deploy when FC reboots while disarmed, even with failure conditions.
    """
    time.sleep(0.1)

    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        fc.simulate_fc_reboot()

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=False,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
}], indirect=True)
def test_nodeploy_reboot_small_time_drop(fc: FCMock, parachute: ParachuteMock):
    """No deploy when time_boot_ms drops by less than the 10 s threshold."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        fc.boot_timestamp_s += 5

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_stops_repeating_deploy_commands_after_acks(fc: FCMock, parachute: ParachuteMock):
    """Acking DO_PARACHUTE and DO_FLIGHTTERMINATION stops their respective retransmission."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()
        pass

    # must be outside the following context, so the buffer is cleared properly
    fc.send_flighttermination_ack_accepted()
    time.sleep(1.0)  # might be one last retransmit before it's stopped, wait it out
    with fc.expect_no_flighttermination(duration_s=1.0), parachute.expect_deploy(timeout_s=1):
        # parachute deploy should still be sent, we just acked flighttermination
        pass

    parachute.send_do_parachute_ack_accepted()
    time.sleep(3.0)  # for some reason this needs to be 3s, otherwise it's flaky
    # I don't know why it takes that long, but it's an acceptable timeframe for stopping resending
    with parachute.expect_no_deploy(duration_s=1.0):
        pass


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_proxy_ack_flighttermination(fc: FCMock, parachute: ParachuteMock):
    """Proxy-ack DO_FLIGHTTERMINATION targeting compid 1 after parachute deploy."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()

    with parachute.expect_command_ack(
        command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        result=mavlink.MAV_RESULT_ACCEPTED,
        source_component=1,
        timeout_s=0.5,
    ), fc.expect_flighttermination(timeout_s=0.5):
        parachute.send_flighttermination_command(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_proxy_ack_force_disarm(fc: FCMock, parachute: ParachuteMock):
    """Proxy-ack force disarm targeting compid 1 after parachute deploy."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()

    with parachute.expect_command_ack(
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_ACCEPTED,
        source_component=1,
        timeout_s=0.5,
    ), fc.expect_force_disarm(timeout_s=0.5):
        parachute.send_force_disarm(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
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
        duration_s=1.5,
    ), fc.expect_flighttermination(timeout_s=0.5):
        parachute.send_flighttermination_command(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
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
        duration_s=1.5,
    ), fc.expect_force_disarm(timeout_s=0.5):
        parachute.send_force_disarm(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_flighttermination_wrong_target(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack for DO_FLIGHTTERMINATION targeting something other than the FC"""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=1.5,
    ):
        parachute.send_flighttermination_command(target_component=2)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_regular_disarm(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack for a regular (non-force) disarm after parachute deploy.
    """

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.pause_sending()

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=1.5,
    ):
        parachute.send_disarm(target_component=1)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '0',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '10.0',
    'PARAM_AV_ATS_MP2_SM':    '10.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
}], indirect=True)
def test_no_proxy_ack_disabled(fc: FCMock, parachute: ParachuteMock):
    """No proxy-ack when ATS is disabled, even though deploy condition is met internally."""

    fc.set_armed(True)
    time.sleep(0.1)
    with parachute.expect_no_deploy(duration_s=1.5), fc.expect_no_flighttermination(duration_s=1.5):
        pass

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=1.5,
    ):
        parachute.send_flighttermination_command(target_component=1)

    with parachute.expect_no_command_ack(
        command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavlink.MAV_RESULT_ACCEPTED,
        duration_s=1.5,
    ):
        parachute.send_force_disarm(target_component=1)


_ARMED_12H_SIM_SPEED = 1000

@pytest.mark.slow
@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':        '1',
    'PARAM_AV_ATS_TIMEOUT':   str(150 * _ARMED_12H_SIM_SPEED * 1.25),
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':    '50.0',
    'PARAM_AV_ATS_MP2_SM':    '50.0',
    'PARAM_AV_ATS_UPS_SM':    '5.0',
    'PARAM_AV_ATS_BAT_TOUT':  str(150 * _ARMED_12H_SIM_SPEED * 1.25),
    'PX4_SIM_SPEED_FACTOR':   str(_ARMED_12H_SIM_SPEED),
}], indirect=True)
def test_armed_12h(fc: FCMock, parachute: ParachuteMock):
    """No false-positive deploy after 12 simulated hours armed.

    Inject failures in acc/roll/pitch, but it should not deploy since there is no timeout.
    AV_ATS_TIMEOUT and AV_ATS_BAT_TOUT are scaled with the sim speed factor, with a 1.25
    margin to account for scheduling issues at high sim speed

    Runs PX4 at 1000x real-time so 12 h of simulated flight time
    passes in ~90 s of wall-clock time.  Catches overflow bugs in
    timers or counters that could cause spurious parachute deployment.

    After the  12h, a genuine fault (power loss) is injected to
    confirm ATS still responds correctly.
    """

    fc.set_armed(True)
    time.sleep(0.1)

    wait_time_s = 12 * 60 * 60 / _ARMED_12H_SIM_SPEED
    with parachute.expect_no_deploy(duration_s=wait_time_s), fc.expect_no_flighttermination(duration_s=wait_time_s):
        pass

    wait_for_ats_status(fc,
                        expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                        | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)

    with parachute.expect_deploy(timeout_s=0.5), fc.expect_flighttermination(timeout_s=0.5):
        fc.set_ats_param('AV_ATS_PARA_SM', 18.0)
        fc.set_ats_param('AV_ATS_MP1_SM', 10.0)
        fc.set_ats_param('AV_ATS_MP2_SM', 10.0)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_EN':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
    'PARAM_AV_ATS_V_EN':      '1',
    'PARAM_AV_ATS_MP_LO_V':   '15.0',
    'PARAM_AV_ATS_UPS_LO_V':  '4.0',
    'PARAM_AV_ATS_MP1_SM':     '50.0',
    'PARAM_AV_ATS_MP2_SM':     '50.0',
    'PARAM_AV_ATS_UPS_SM':     '5.0',
    'PARAM_AV_ATS_BAT_V_TAU':  '2.0',
}], indirect=True)
def test_fc_mismatch_uses_averaged_main_power(fc: FCMock, parachute: ParachuteMock):
    """FC battery voltage is compared against main power averaged over AV_ATS_BAT_V_TAU"""

    fc.set_armed(True)

    wait_for_ats_status(fc,
                        expected_flags=0,
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=1.0)

    # MP1 and MP2 drop 15 V (above the 10 V tolerance)
    # Lower them in two steps, so they are never more than 10v from each other,
    # which would raise the MP_MISMATCH flag and fail the test
    for v in (42.5, 35.0):
        fc.set_ats_param('AV_ATS_MP1_SM', v)
        fc.set_ats_param('AV_ATS_MP2_SM', v)

    # The averaged values lag, so no mismatch expected for ~2.2 s (2 s time constant).
    # Every status in the first 0.5 s must be clean.
    status = fc.mav.recv_until(
        lambda m: m.get_type() == 'AVIANT_ATS_STATUS' and m.ats_status_flags != 0,
        timeout_s=0.5,
    )
    assert status is None, f"Unexpected flags {fc.flags_str(status.ats_status_flags)}"

    # Then we will get a failure before 3 more seconds have passed
    wait_for_ats_status(fc,
                        expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY,
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=3.0)

    # 0 compares against unfiltered main power: recovery is immediate
    fc.set_ats_param('AV_ATS_BAT_V_TAU', 0.0)
    fc.set_ats_param('AV_ATS_MP1_SM', 50.0)
    fc.set_ats_param('AV_ATS_MP2_SM', 50.0)
    wait_for_ats_status(fc,
                        expected_flags=0,
                        expected_enabled_status=True,
                        expected_powerloss_enabled_status=True,
                        expected_fc_armed=True,
                        timeout_s=0.5)
