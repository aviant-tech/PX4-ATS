"""SITL integration tests for the ATS module.

Each test starts a fresh PX4 SITL instance.  ATS parameters are passed
via environment variables and applied in the airframe .post script
*before* the module starts (the module does not call updateParams at
runtime).

The test script drives the ATS through its MAVLink inputs:

  MAVLink HEARTBEAT       -> external_vehicle_status  (arm / disarm)
  MAVLink ATTITUDE        -> external_ins_attitude    (FC-timeout clock)
  MAVLink COMMAND_LONG    -> external_vehicle_status  (DO_PARACHUTE -> TERMINATED)

Deploy detection listens for MAV_CMD_DO_PARACHUTE COMMAND_LONG messages
forwarded by the MAVLink module from the ATS vehicle_command publications.

SIH provides simulated sensor data.  The sensors module derives
vehicle_acceleration (~9.81 m/s^2) and EKF2 publishes vehicle_attitude
(~0 deg roll/pitch) once tilt alignment completes (~30 s in SIH due to
the high simulated IMU noise).  Sensor-dependent trigger conditions are
exercised by adjusting parameter thresholds rather than manipulating
physics.
"""

import time

import pytest


def _ats_env(active=1, timeout=150, acc_norm=5.0, roll_ang=80.0,
             pitch_ang=60.0):
    """Build env-var dict consumed by the .post airframe script."""
    return {
        'ATS_PARAM_ACTIVE':    str(active),
        'ATS_PARAM_TIMEOUT':   str(timeout),
        'ATS_PARAM_ACC_NORM':  str(acc_norm),
        'ATS_PARAM_ROLL_ANG':  str(roll_ang),
        'ATS_PARAM_PITCH_ANG': str(pitch_ang),
    }


@pytest.mark.parametrize('px4', [
    _ats_env(active=0, acc_norm=20.0, roll_ang=0.0, pitch_ang=0.0),
], indirect=True)
def test_no_trigger_when_inactive(tester):
    """No deploy when ATS is inactive (AV_ATS_ACTIVE=0), regardless of conditions."""

    tester.send_fc_heartbeat(armed=True)
    time.sleep(0.2)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered even though ATS is inactive"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0, roll_ang=0.0, pitch_ang=0.0),
], indirect=True)
def test_no_trigger_when_disarmed(tester):
    """No deploy when FC is DISARMED, even with all fail flags set."""

    tester.send_fc_heartbeat(armed=False)
    time.sleep(0.2)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered while FC is DISARMED"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1),
], indirect=True)
def test_do_parachute_forwarding_while_armed(tester):
    """MAV_CMD_DO_PARACHUTE is forwarded when armed"""

    tester.keep_alive(duration_s=1.0, armed=True)
    tester.send_parachute_command()

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered when receiving MAV_CMD_DO_PARACHUTE while armed"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1),
], indirect=True)
def test_do_parachute_forwarding_while_disarmed(tester):
    """MAV_CMD_DO_PARACHUTE is forwarded when disarmed"""

    tester.keep_alive(duration_s=1.0, armed=False)
    tester.send_parachute_command()

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered when receiving MAV_CMD_DO_PARACHUTE while disarmed"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0, timeout=150),
], indirect=True)
def test_armed_timeout_plus_accel_fail(tester):
    """Deploy on ARMED + fc_timeout + accel_norm_fail (threshold > gravity)."""

    tester.keep_alive(duration_s=1.0, armed=True)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered on ARMED + timeout + accel-norm fail"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=5.0, roll_ang=0.0, pitch_ang=80.0,
             timeout=150),
], indirect=True)
def test_armed_timeout_plus_roll_fail(tester):
    """Deploy on ARMED + fc_timeout + roll_fail (threshold = 0 deg).

    EKF2 publishes vehicle_attitude at near-level orientation in SIH.
    A 0-degree threshold means any non-zero roll triggers roll_fail.
    """

    tester.keep_alive(duration_s=1.0, armed=True)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered on ARMED + timeout + roll fail"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=5.0, roll_ang=80.0, pitch_ang=0.0,
             timeout=150),
], indirect=True)
def test_armed_timeout_plus_pitch_fail(tester):
    """Deploy on ARMED + fc_timeout + pitch_fail (threshold = 0 deg).

    EKF2 publishes vehicle_attitude at near-level orientation in SIH.
    A 0-degree threshold means any non-zero pitch triggers pitch_fail.
    """

    tester.keep_alive(duration_s=1.0, armed=True)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered on ARMED + timeout + pitch fail"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=5.0, roll_ang=80.0, pitch_ang=60.0,
             timeout=150),
], indirect=True)
def test_armed_timeout_only_no_deploy(tester):
    """No deploy on ARMED + fc_timeout when no sensor-fail condition is met.

    Thresholds: accel_norm=5 (gravity 9.81 > 5 -> pass),
    roll=80 deg (0 < 80 -> pass), pitch=60 deg (0 < 60 -> pass).
    """

    tester.keep_alive(duration_s=1.0, armed=True)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered with timeout only (no sensor fail)"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0, roll_ang=0.0, pitch_ang=0.0,
             timeout=30000),
], indirect=True)
def test_armed_sensor_fail_only_no_deploy(tester):
    """No deploy on ARMED + sensor-fail when fc_timeout has not fired.

    Sensor thresholds are set so all sensor-fail flags are true,
    but fc_timeout is set very high (30 s) and we keep sending ATTITUDE,
    so the timeout condition is never met.
    """

    tester.keep_alive(duration_s=4.0, armed=True, interval_s=0.02)

    assert tester.verify_no_deploy(duration_s=0.5), \
        "Deploy triggered with sensor fail only (no timeout)"
