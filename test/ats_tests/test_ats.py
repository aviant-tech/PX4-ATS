"""SITL integration tests for the ATS module.

Each test starts a fresh PX4 SITL instance.  ATS parameters are passed
via environment variables and applied in the airframe .post script
*before* the module starts (the module does not call updateParams at
runtime).

The ATSTester background thread continuously sends
AVIANT_DETAILED_FC_STATE at ~20 Hz, emulating a flight controller.

Deploy detection listens for MAV_CMD_DO_PARACHUTE COMMAND_LONG messages
forwarded by the MAVLink module from the ATS vehicle_command publications.

SIH provides simulated sensor data.  The sensors module derives
vehicle_acceleration (~9.81 m/s^2) and EKF2 publishes vehicle_attitude
(~0 deg roll/pitch) once tilt alignment completes.
Sensor-dependent trigger conditions are exercised by adjusting parameter thresholds
rather than manipulating physics.
"""

import time

import pytest
from ats_tester import ATSTester, mavlink


def assert_ats_status(tester: ATSTester, expected_flags: int,
                      expected_active_status: bool,
                      expected_powerloss_active_status: bool,
                      expected_fc_state: int) -> None:
    """Validate AVIANT_ATS_STATUS flags and fc_state match expectations."""
    status = tester.get_ats_status(timeout_s=3.0)
    assert status is not None, "Did not receive AVIANT_ATS_STATUS message"

    assert expected_active_status == status.ats_active, f"Expected ats_active={expected_active_status}"
    assert expected_powerloss_active_status == status.power_loss_trigger_enabled, f"Expected power_loss_trigger_enabled={expected_powerloss_active_status}"

    actual_flags = status.ats_status_flags
    assert actual_flags == expected_flags, (
        f"ATS flags mismatch: "
        f"expected {tester.flags_str(expected_flags)} ({expected_flags:#x}), "
        f"got {tester.flags_str(actual_flags)} ({actual_flags:#x})"
    )

    assert status.fc_state == expected_fc_state, (
        f"fc_state mismatch: expected {expected_fc_state}, "
        f"got {status.fc_state}"
    )


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
}], indirect=True)
def test_deploy_armed_terminated(tester: ATSTester):
    """MAV_STATE_FLIGHT_TERMINATION causes parachute when armed """

    tester.set_armed(True)
    time.sleep(1.0)
    tester.set_system_status(mavlink.MAV_STATE_FLIGHT_TERMINATION)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Expected deploy when FC is MAV_STATE_FLIGHT_TERMINATION while armed"

    assert_ats_status(tester,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY,
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_TERMINATED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
}], indirect=True)
def test_deploy_disarmed_terminated(tester: ATSTester):
    """MAV_STATE_FLIGHT_TERMINATION causes parachute when disarmed """

    time.sleep(1.0)
    tester.set_system_status(mavlink.MAV_STATE_FLIGHT_TERMINATION)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Expected deploy when FC is MAV_STATE_FLIGHT_TERMINATION while disarmed"

    assert_ats_status(tester,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY,
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_TERMINATED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
}], indirect=True)
def test_deploy_timeout_accel_fail(tester: ATSTester):
    """Deploy on ARMED + fc_timeout + accel_norm_fail (threshold > gravity)."""

    tester.set_armed(True)
    time.sleep(1.0)
    tester.pause_sending()

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Expected deploy on ARMED + timeout + accel-norm fail"

    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '80.0',
}], indirect=True)
def test_deploy_timeout_roll_fail(tester: ATSTester):
    """Deploy on ARMED + fc_timeout + roll_fail (threshold = 0 deg).

    EKF2 publishes vehicle_attitude at near-level orientation in SIH.
    A 0-degree threshold means any non-zero roll triggers roll_fail.
    """

    tester.set_armed(True)
    time.sleep(1.0)
    tester.pause_sending()

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Expected deploy on ARMED + timeout + roll fail"

    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
}], indirect=True)
def test_deploy_timeout_pitch_fail(tester: ATSTester):
    """Deploy on ARMED + fc_timeout + pitch_fail (threshold = 0 deg).

    EKF2 publishes vehicle_attitude at near-level orientation in SIH.
    A 0-degree threshold means any non-zero pitch triggers pitch_fail.
    """

    tester.set_armed(True)
    time.sleep(1.0)
    tester.pause_sending()

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Expected deploy on ARMED + timeout + pitch fail"

    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
}], indirect=True)
def test_deploy_reboot_armed_sensor_fail(tester: ATSTester):
    """Deploy when FC reboots while armed and a sensor-fail condition is met."""

    tester.set_armed(True)
    time.sleep(1.0)
    tester.simulate_fc_reboot()

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Expected deploy on FC reboot while armed with sensor failure"

    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_DISARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
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
def test_deploy_voltage_main_low_ups_healthy(tester: ATSTester):
    """Deploy when both main powers drop below threshold while UPS stays healthy."""

    tester.set_armed(True)
    time.sleep(1.0)

    tester.set_param('AV_ATS_MP1_SM', 10.0)
    tester.set_param('AV_ATS_MP2_SM', 10.0)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Expected deploy when both main powers low and UPS healthy"

    assert_ats_status(tester,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY
                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS,
                      expected_active_status=True,
                      expected_powerloss_active_status=True,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '0',
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
def test_nodeploy_inactive(tester: ATSTester):
    """No deploy when FC is INACTIVE, even with all failures set."""

    tester.set_armed(True)
    time.sleep(0.2)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy when ATS is inactive"

    # All sensor fails + voltage fail → parachute_deploy is set in the uORB
    # message, but no command is sent because ATS is inactive.
    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
                      expected_active_status=False,
                      expected_powerloss_active_status=True,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
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
def test_nodeploy_disarmed(tester: ATSTester):
    """No deploy when FC is DISARMED, even with all fail flags set."""

    time.sleep(0.2)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy while FC is disarmed"

    # Sensor fail flags are set, but disarmed blocks both the control-failure
    # path and the voltage path → no PARACHUTE_DEPLOY.
    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS),
                      expected_active_status=True,
                      expected_powerloss_active_status=True,
                      expected_fc_state=ATSTester.FC_STATE_DISARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '0',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
}], indirect=True)
def test_nodeploy_terminated_inactive(tester: ATSTester):
    """MAV_STATE_FLIGHT_TERMINATION causes NO parachute when armed and inactive """

    tester.set_armed(True)
    time.sleep(1.0)
    tester.set_system_status(mavlink.MAV_STATE_FLIGHT_TERMINATION)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy on MAV_STATE_FLIGHT_TERMINATION when ATS is inactive"

    # Terminated path fires → parachute_deploy true in uORB, but no
    # command because ATS is inactive.
    assert_ats_status(tester,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY,
                      expected_active_status=False,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_TERMINATED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
}], indirect=True)
def test_nodeploy_timeout_no_sensor_fail(tester: ATSTester):
    """No deploy on ARMED + fc_timeout when no sensor-fail condition is met."""

    tester.set_armed(True)
    time.sleep(1.0)
    tester.pause_sending()

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy on timeout alone without sensor fail"

    assert_ats_status(tester,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT,
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '30000',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
}], indirect=True)
def test_nodeploy_sensor_fail_no_timeout(tester: ATSTester):
    """No deploy on ARMED + sensor-fail when fc_timeout has not fired.

    Sensor thresholds are set so all sensor-fail flags are true,
    but fc_timeout is set very high (30 s),
    so the timeout condition is never met.
    """

    tester.set_armed(True)
    time.sleep(4.0)

    assert tester.verify_no_deploy(duration_s=0.5), \
        "Expected no deploy on sensor fail alone without timeout"

    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '5.0',
    'PARAM_AV_ATS_ROLL_ANG':  '80.0',
    'PARAM_AV_ATS_PITCH_ANG': '60.0',
}], indirect=True)
def test_nodeploy_reboot_armed_no_sensor_fail(tester: ATSTester):
    """No deploy when FC reboots while armed but no sensor-fail condition is met."""

    tester.set_armed(True)
    time.sleep(1.0)
    tester.simulate_fc_reboot()

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy on FC reboot while armed without sensor failure"

    assert_ats_status(tester,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED,
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_DISARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
}], indirect=True)
def test_nodeploy_reboot_disarmed(tester: ATSTester):
    """No deploy when FC reboots while disarmed, even with failure conditions.
    """

    time.sleep(1.0)
    tester.simulate_fc_reboot()

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy on FC reboot while disarmed"

    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_DISARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
    'PARAM_AV_ATS_TIMEOUT':   '150',
    'PARAM_AV_ATS_ACC_NORM':  '20.0',
    'PARAM_AV_ATS_ROLL_ANG':  '0.0',
    'PARAM_AV_ATS_PITCH_ANG': '0.0',
}], indirect=True)
def test_nodeploy_reboot_small_time_drop(tester: ATSTester):
    """No deploy when time_boot_ms drops by less than the 10 s threshold."""

    tester.set_armed(True)
    time.sleep(1.0)
    tester.boot_timestamp_s += 5

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy on time_boot_ms drop smaller than 10 s threshold"

    assert_ats_status(tester,
                      expected_flags=(mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL
                                      | mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
                      expected_active_status=True,
                      expected_powerloss_active_status=False,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)


@pytest.mark.parametrize('px4', [{
    'PARAM_AV_ATS_ACTIVE':    '1',
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
def test_nodeploy_voltage_main_low_ups_unhealthy(tester: ATSTester):
    """No deploy when all voltages (including UPS) drop simultaneously.

    If the UPS voltage also drops below its threshold (<=4V), the voltage
    readings are considered unreliable (e.g. sensor/ADC issue) and the
    parachute must not fire.
    """

    tester.set_armed(True)
    time.sleep(1.0)

    # Set UPS low first to avoid a race where main powers are seen as low
    # while UPS is still healthy (which would trigger voltage_fail).
    # This is realistic, we will have a margin on the voltage threshold
    # ensuring measurements for a short while after the failure is detected
    tester.set_param('AV_ATS_UPS_SM', 3.0)
    time.sleep(0.3)
    tester.set_param('AV_ATS_MP1_SM', 10.0)
    tester.set_param('AV_ATS_MP2_SM', 10.0)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Expected no deploy when all voltages low (UPS unreliable)"

    # Power loss should be flagged, but not deploy parachute since UPS is unhealthy
    assert_ats_status(tester,
                      expected_flags=mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY
                      | mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS,
                      expected_active_status=True,
                      expected_powerloss_active_status=True,
                      expected_fc_state=ATSTester.FC_STATE_ARMED)
