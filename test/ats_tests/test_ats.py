"""SITL integration tests for the ATS module.

Each test starts a fresh PX4 SITL instance.  ATS parameters are passed
via environment variables and applied in the airframe .post script
*before* the module starts (the module does not call updateParams at
runtime).

The test script drives the ATS through its MAVLink inputs:

  MAVLink AVIANT_DETAILED_FC_STATE -> external_aviant_detailed_fc_state
      (arm/disarm, system_status, FC-timeout clock)
  MAVLink COMMAND_LONG -> vehicle_command  (DO_PARACHUTE forwarding)

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
from ats_tester import ATSTester


def _ats_env(active=1, timeout=150, acc_norm=5.0, roll_ang=80.0,
             pitch_ang=60.0, v_en=None, mp_lowv=None, ups_lowv=None,
             v_mp1_sim=None, v_mp2_sim=None, v_ups_sim=None):
    """Build env-var dict consumed by the .post airframe script."""
    env = {
        'ATS_PARAM_ACTIVE':    str(active),
        'ATS_PARAM_TIMEOUT':   str(timeout),
        'ATS_PARAM_ACC_NORM':  str(acc_norm),
        'ATS_PARAM_ROLL_ANG':  str(roll_ang),
        'ATS_PARAM_PITCH_ANG': str(pitch_ang),
    }
    if v_en is not None:
        env['ATS_PARAM_V_EN'] = str(v_en)
    if mp_lowv is not None:
        env['ATS_PARAM_MP_LOWV'] = str(mp_lowv)
    if ups_lowv is not None:
        env['ATS_PARAM_UPS_LOWV'] = str(ups_lowv)
    if v_mp1_sim is not None:
        env['ATS_PARAM_V_MP1_SIM'] = str(v_mp1_sim)
    if v_mp2_sim is not None:
        env['ATS_PARAM_V_MP2_SIM'] = str(v_mp2_sim)
    if v_ups_sim is not None:
        env['ATS_PARAM_V_UPS_SIM'] = str(v_ups_sim)
    return env


# ── deploy ────────────────────────────────────────────────────────────


@pytest.mark.parametrize('px4', [
    _ats_env(active=1),
], indirect=True)
def test_deploy_armed_terminated(tester: ATSTester):
    """MAV_STATE_FLIGHT_TERMINATION causes parachute when armed """

    tester.keep_alive(duration_s=1.0, armed=True)
    tester.send_fc_state(armed=True, system_status=ATSTester.MAV_STATE_FLIGHT_TERMINATION)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered when FC is MAV_STATE_FLIGHT_TERMINATION while armed"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1),
], indirect=True)
def test_deploy_disarmed_terminated(tester: ATSTester):
    """MAV_STATE_FLIGHT_TERMINATION causes parachute when disarmed """

    tester.keep_alive(duration_s=1.0, armed=False)
    tester.send_fc_state(armed=False, system_status=ATSTester.MAV_STATE_FLIGHT_TERMINATION)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered when FC is MAV_STATE_FLIGHT_TERMINATION while disarmed"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0, timeout=150),
], indirect=True)
def test_deploy_timeout_accel_fail(tester: ATSTester):
    """Deploy on ARMED + fc_timeout + accel_norm_fail (threshold > gravity)."""

    tester.keep_alive(duration_s=1.0, armed=True)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered on ARMED + timeout + accel-norm fail"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=5.0, roll_ang=0.0, pitch_ang=80.0,
             timeout=150),
], indirect=True)
def test_deploy_timeout_roll_fail(tester: ATSTester):
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
def test_deploy_timeout_pitch_fail(tester: ATSTester):
    """Deploy on ARMED + fc_timeout + pitch_fail (threshold = 0 deg).

    EKF2 publishes vehicle_attitude at near-level orientation in SIH.
    A 0-degree threshold means any non-zero pitch triggers pitch_fail.
    """

    tester.keep_alive(duration_s=1.0, armed=True)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered on ARMED + timeout + pitch fail"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0),
], indirect=True)
def test_deploy_reboot_armed_sensor_fail(tester: ATSTester):
    """Deploy when FC reboots while armed and a sensor-fail condition is met.

    Reboot is detected when time_boot_ms drops by more than 10 s.
    keep_alive runs for 1 s with boot_timestamp_s already 15 s in the
    past, so time_boot_ms ≈ 16 000 ms.  simulate_fc_reboot resets the
    boot origin and sends time_boot_ms ≈ 0, a drop of ~16 s (> 10 s
    threshold).  Combined with acc_norm=20 (gravity ~9.81 < 20 → fail),
    the deploy condition is met.
    """

    tester.keep_alive(duration_s=1.0, armed=True)
    tester.simulate_fc_reboot()

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered on FC reboot while armed with failure"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, v_en=1, mp_lowv=15.0, ups_lowv=4.0,
             v_mp1_sim=50.0, v_mp2_sim=50.0, v_ups_sim=5.0),
], indirect=True)
def test_deploy_voltage_main_low_ups_healthy(tester: ATSTester):
    """Deploy when both main powers drop below threshold while UPS stays healthy.

    Initial state: MP1=50V, MP2=50V, UPS=5V (all normal).
    Then both main powers drop to 10V while UPS remains at 5V (> 4V threshold).
    This indicates a real main power loss with the ATS still on backup power.
    """

    tester.keep_alive(duration_s=1.0, armed=True, interval_s=0.05)
    tester._drain_command_long()

    tester.set_param('AV_V_MP1_SIM', 10.0)
    tester.set_param('AV_V_MP2_SIM', 10.0)

    assert tester.wait_for_deploy(timeout_s=5.0), \
        "Deploy NOT triggered when both main powers low and UPS healthy"


# ── nodeploy ──────────────────────────────────────────────────────────


@pytest.mark.parametrize('px4', [
    _ats_env(active=0, acc_norm=20.0, roll_ang=0.0, pitch_ang=0.0,
             v_en=1, mp_lowv=15.0, ups_lowv=4.0,
             v_mp1_sim=10.0, v_mp2_sim=10.0, v_ups_sim=5.0),
], indirect=True)
def test_nodeploy_inactive(tester: ATSTester):
    """No deploy when ATS is inactive (AV_ATS_ACTIVE=0), regardless of conditions.

    All failure conditions are met: sensor thresholds exceeded, voltage
    enabled with both main powers low and UPS healthy.
    """

    tester.send_fc_state(armed=True)
    time.sleep(0.2)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered even though ATS is inactive"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0, roll_ang=0.0, pitch_ang=0.0,
             v_en=1, mp_lowv=15.0, ups_lowv=4.0,
             v_mp1_sim=10.0, v_mp2_sim=10.0, v_ups_sim=5.0),
], indirect=True)
def test_nodeploy_disarmed(tester: ATSTester):
    """No deploy when FC is DISARMED, even with all fail flags set.

    All failure conditions are met: sensor thresholds exceeded, voltage
    enabled with both main powers low and UPS healthy.
    """

    tester.send_fc_state(armed=False)
    time.sleep(0.2)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered while FC is DISARMED"


@pytest.mark.parametrize('px4', [
    _ats_env(active=0),
], indirect=True)
def test_nodeploy_terminated_inactive(tester: ATSTester):
    """MAV_STATE_FLIGHT_TERMINATION causes NO parachute when armed and inactive """

    tester.keep_alive(duration_s=1.0, armed=True)
    tester.send_fc_state(armed=True, system_status=ATSTester.MAV_STATE_FLIGHT_TERMINATION)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered when FC is MAV_STATE_FLIGHT_TERMINATION while armed, even though ATS is inactive"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=5.0, roll_ang=80.0, pitch_ang=60.0,
             timeout=150),
], indirect=True)
def test_nodeploy_timeout_no_sensor_fail(tester: ATSTester):
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
def test_nodeploy_sensor_fail_no_timeout(tester: ATSTester):
    """No deploy on ARMED + sensor-fail when fc_timeout has not fired.

    Sensor thresholds are set so all sensor-fail flags are true,
    but fc_timeout is set very high (30 s) and we keep sending
    AVIANT_DETAILED_FC_STATE, so the timeout condition is never met.
    """

    tester.keep_alive(duration_s=4.0, armed=True, interval_s=0.02)

    assert tester.verify_no_deploy(duration_s=0.5), \
        "Deploy triggered with sensor fail only (no timeout)"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1),
], indirect=True)
def test_nodeploy_reboot_armed_no_sensor_fail(tester: ATSTester):
    """No deploy when FC reboots while armed but no sensor-fail condition is met.

    Reboot is detected (time_boot_ms drops > 10 s), but default sensor
    thresholds are lenient (acc_norm=5, roll=80, pitch=60) so no sensor
    fail flag is set.  fc_rebooted_while_armed alone is not enough.
    """

    tester.keep_alive(duration_s=1.0, armed=True)
    tester.simulate_fc_reboot()

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered on FC reboot while armed with no failure"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0),
], indirect=True)
def test_nodeploy_reboot_disarmed(tester: ATSTester):
    """No deploy when FC reboots while disarmed, even with a sensor failure.

    time_boot_ms drops > 10 s so the reboot is detected, but the FC was
    disarmed before the reboot so fc_rebooted_while_armed stays false.
    """

    tester.keep_alive(duration_s=1.0, armed=False)
    tester.simulate_fc_reboot()

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered on FC reboot while disarmed"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, acc_norm=20.0),
], indirect=True)
def test_nodeploy_reboot_small_time_drop(tester: ATSTester):
    """No deploy when time_boot_ms drops by less than the 10 s threshold.

    A 5 s backward jump in time_boot_ms must NOT be treated as a reboot.
    Even with a sensor failure and the FC armed, the deploy must not fire
    because fc_rebooted_while_armed should not be set.
    """

    tester.keep_alive(duration_s=1.0, armed=True)

    current_boot_ms = int((time.monotonic() - tester.boot_timestamp_s) * 1000)
    tester.send_fc_state(armed=False, time_boot_ms=current_boot_ms - 5000)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered on time_boot_ms drop smaller than 10 s threshold"


@pytest.mark.parametrize('px4', [
    _ats_env(active=1, v_en=1, mp_lowv=15.0, ups_lowv=4.0,
             v_mp1_sim=50.0, v_mp2_sim=50.0, v_ups_sim=5.0),
], indirect=True)
def test_nodeploy_voltage_main_low_ups_unhealthy(tester: ATSTester):
    """No deploy when all voltages (including UPS) drop simultaneously.

    If the UPS voltage also drops below its threshold (<=4V), the voltage
    readings are considered unreliable (e.g. sensor/ADC issue) and the
    parachute must not fire.
    """

    tester.keep_alive(duration_s=1.0, armed=True, interval_s=0.05)
    tester._drain_command_long()

    # Set UPS low first to avoid a race where main powers are seen as low
    # while UPS is still healthy (which would trigger voltage_fail).
    # This is realistic, because the UPS will remain alive and provide valid
    # measurements for a short while after its power supply fails.
    tester.set_param('AV_V_UPS_SIM', 3.0)
    time.sleep(0.3)
    tester.set_param('AV_V_MP1_SIM', 10.0)
    tester.set_param('AV_V_MP2_SIM', 10.0)

    assert tester.verify_no_deploy(duration_s=3.0), \
        "Deploy triggered when all voltages low (UPS unreliable)"
