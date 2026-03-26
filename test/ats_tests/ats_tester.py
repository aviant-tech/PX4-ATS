"""Helper class wrapping pymavlink with ATS-specific MAVLink interactions.

A background thread continuously sends AVIANT_DETAILED_FC_STATE at ~30 Hz
to emulate the flight controller.  Tests control the emulated FC through
set_armed() and set_system_status(), and can simulate FC silence via
pause_sending().

Requires the ``aviant`` MAVLink dialect (set MAVLINK_DIALECT=aviant and
MDEF pointing at the in-tree message_definitions before importing
pymavlink).  See conftest.py.
"""

from __future__ import annotations

import threading
import time

from pymavlink import mavutil

mavlink = mavutil.mavlink


class ATSTester:
    """Drives ATS test scenarios over MAVLink and verifies deploy commands.

    A background thread continuously sends AVIANT_DETAILED_FC_STATE at
    ~30 Hz.  Tests mutate the emulated FC state through set_armed() and
    set_system_status(); the background thread picks up the new values on
    the next iteration.  pause_sending() / resume_sending() simulate the
    FC going silent or coming back.
    """

    # fc_state values from AviantAts.msg (not in the MAVLink dialect)
    FC_STATE_DISARMED   = 0
    FC_STATE_ARMED      = 1
    FC_STATE_TERMINATED = 2

    _SEND_INTERVAL_S = 1.0/30.0

    def __init__(self, connection: mavutil.mavlink_connection):
        self.conn = connection
        # We act like the FC booted a while ago, this is necessary for reboot detection
        self.boot_timestamp_s = time.monotonic() - 15.0

        self._lock = threading.Lock()
        self._armed = False
        self._system_status = mavlink.MAV_STATE_ACTIVE
        self._sending = True

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

    def _send_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                if self._sending:
                    self._send_fc_state(
                    )
            self._stop_event.wait(timeout=self._SEND_INTERVAL_S)

    def _time_boot_ms(self) -> int:
        return int((time.monotonic() - self.boot_timestamp_s) * 1000)


    def _send_fc_state(self) -> None:
        """Build and send one FC state message.  Caller must hold _lock."""
        msg = mavlink.MAVLink_aviant_detailed_fc_state_message(
            self._time_boot_ms(),  # time_boot_ms
            int(time.time() * 1e6),  # time_unix_usec
            1 if self._armed else 0,  # fc_armed
            0,  # fc_flight_termination, not used in tests
        )
        self.conn.mav.send(msg)

    def set_armed(self, armed: bool) -> None:
        with self._lock:
            self._armed = armed

    def set_system_status(self, status: int) -> None:
        with self._lock:
            self._system_status = status

    def pause_sending(self) -> None:
        """Stop the background FC state stream (simulates FC going silent)."""
        with self._lock:
            self._sending = False

    def resume_sending(self) -> None:
        """Resume the background FC state stream."""
        with self._lock:
            self._sending = True

    def send_parachute_command(self) -> None:
        """Send MAV_CMD_DO_PARACHUTE."""
        with self._lock:
            self.conn.mav.command_long_send(
                target_system=1,
                target_component=1,
                command=mavlink.MAV_CMD_DO_PARACHUTE,
                confirmation=0,
                param1=float(mavlink.PARACHUTE_ACTION_RELEASE),
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
            )

    def simulate_fc_reboot(self) -> None:
        """Simulate an FC reboot by resetting time_boot_ms to a small value.

        The ATS detects a reboot when time_boot_ms drops by more than 10 s
        compared to the previous message.  Resetting boot_timestamp_s makes
        subsequent messages produce small time_boot_ms values.  The
        background thread continues sending with the new epoch.
        """
        with self._lock:
            self.boot_timestamp_s = time.monotonic()
            self._armed = False

    def set_param(self, param_id: str, value: float) -> None:
        """Set a PX4 parameter via MAVLink PARAM_SET."""
        with self._lock:
            self.conn.mav.param_set_send(
                target_system=1,
                target_component=1,
                param_id=param_id.encode('utf-8'),
                param_value=value,
                param_type=mavlink.MAV_PARAM_TYPE_REAL32,
            )

    def _drain_command_long(self) -> None:
        """Discard any stale COMMAND_LONG messages in the receive buffer."""
        while True:
            msg = self.conn.recv_match(type='COMMAND_LONG', blocking=False)
            if msg is None:
                break

    def wait_for_deploy(self, timeout_s: float = 10.0) -> bool:
        """Wait until a DO_PARACHUTE COMMAND_LONG is received.

        Returns True if the command was received within timeout.
        """
        deadline = time.monotonic() + timeout_s
        # Receive in loop since we may get COMMAND_LONG that is not MAV_CMD_DO_PARACHUTE
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='COMMAND_LONG', blocking=True,
                                       timeout=0.5)
            if msg is None:
                continue
            if msg.command == mavlink.MAV_CMD_DO_PARACHUTE:
                return True
        return False

    def verify_no_deploy(self, duration_s: float = 3.0) -> bool:
        """Listen for *duration_s* and verify NO deploy commands appear.

        Returns True if no deploy commands were received (= expected for
        negative tests).
        """
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='COMMAND_LONG', blocking=True,
                                       timeout=0.5)
            if msg is None:
                continue
            if msg.command == mavlink.MAV_CMD_DO_PARACHUTE:
                return False
        return True

    def get_ats_status(self, timeout_s: float = 3.0):
        """Receive a fresh AVIANT_ATS_STATUS message.

        Drains any stale status messages from the buffer, then blocks
        until a new one arrives (or *timeout_s* elapses).
        """
        while self.conn.recv_match(type='AVIANT_ATS_STATUS',
                                   blocking=False) is not None:
            pass
        return self.conn.recv_match(type='AVIANT_ATS_STATUS',
                                    blocking=True, timeout=timeout_s)

    @staticmethod
    def flags_str(flags: int) -> str:
        """Human-readable representation of ATS status flags."""
        _FLAG_NAMES = [
            ('ACCEL_NORM_FAIL',     mavlink.AVIANT_ATS_STATUS_FLAG_ACCEL_NORM_FAIL),
            ('ROLL_FAIL',           mavlink.AVIANT_ATS_STATUS_FLAG_ROLL_FAIL),
            ('PITCH_FAIL',          mavlink.AVIANT_ATS_STATUS_FLAG_PITCH_FAIL),
            ('FC_TIMEOUT',          mavlink.AVIANT_ATS_STATUS_FLAG_FC_TIMEOUT),
            ('POWER_LOSS',          mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS),
            ('UPS_UNHEALTHY',       mavlink.AVIANT_ATS_STATUS_FLAG_UPS_UNHEALTHY),
            ('POWER_LOSS',          mavlink.AVIANT_ATS_STATUS_FLAG_POWER_LOSS),
            ('REBOOTED_WHILE_ARMED',mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED),
            ('PARACHUTE_DEPLOY',    mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
        ]
        names = [n for n, v in _FLAG_NAMES if flags & v]
        return ' | '.join(names) if names else '(none)'

    def shutdown(self) -> None:
        """Stop the background sender thread."""
        self._stop_event.set()
        self._thread.join(timeout=3)
