"""Helper class wrapping pymavlink with ATS-specific MAVLink interactions.

A background thread continuously sends AVIANT_DETAILED_FC_STATE at ~20 Hz
to emulate the flight controller.  Tests control the emulated FC through
set_armed() and set_system_status(), and can simulate FC silence via
pause_sending().

Deploy detection listens for COMMAND_LONG messages on the MAVLink
connection.  The ATS module publishes vehicle_command to uORB which the
MAVLink module forwards as MAV_CMD_DO_PARACHUTE (target component 161)
and MAV_CMD_DO_FLIGHTTERMINATION (target component 1).

The ATS module receives FC state via the AVIANT_DETAILED_FC_STATE custom
MAVLink message (id 59025), which populates the
external_aviant_detailed_fc_state uORB topic.
"""

from __future__ import annotations

import struct
import threading
import time

from pymavlink import mavutil

_mavlink_mod = mavutil.mavlink


class MAVLink_aviant_detailed_fc_state_message(_mavlink_mod.MAVLink_message):
    """AVIANT_DETAILED_FC_STATE (msg id 59025) – not in the standard dialect."""

    msgId = 59025
    id = 59025
    name = 'AVIANT_DETAILED_FC_STATE'
    fieldnames = ['time_boot_ms', 'time_unix_usec', 'armed',
                  'vtol_state', 'system_status']
    ordered_fieldnames = ['time_unix_usec', 'time_boot_ms', 'armed',
                          'vtol_state', 'system_status']
    fieldtypes = ['uint32_t', 'uint64_t', 'uint8_t', 'uint8_t', 'uint8_t']
    fielddisplays_by_name = {}
    fieldenums_by_name = {}
    fieldunits_by_name = {}
    format = '<QIBBB'
    native_format = bytearray(b'<QIBBB')
    orders = [1, 0, 2, 3, 4]
    lengths = [1, 1, 1, 1, 1]
    array_lengths = [0, 0, 0, 0, 0]
    crc_extra = 173
    unpacker = struct.Struct('<QIBBB')
    instance_field = None
    instance_offset = -1

    def __init__(self, time_boot_ms, time_unix_usec, armed,
                 vtol_state, system_status):
        _mavlink_mod.MAVLink_message.__init__(self, self.msgId, self.name)
        self._fieldnames = self.fieldnames
        self.time_boot_ms = time_boot_ms
        self.time_unix_usec = time_unix_usec
        self.armed = armed
        self.vtol_state = vtol_state
        self.system_status = system_status

    def pack(self, mav, force_mavlink1=False):
        return self._pack(
            mav, self.crc_extra,
            struct.pack('<QIBBB',
                        self.time_unix_usec, self.time_boot_ms,
                        self.armed, self.vtol_state, self.system_status),
            force_mavlink1=force_mavlink1,
        )


class ATSTester:
    """Drives ATS test scenarios over MAVLink and verifies deploy commands.

    A background thread continuously sends AVIANT_DETAILED_FC_STATE at
    ~20 Hz.  Tests mutate the emulated FC state through set_armed() and
    set_system_status(); the background thread picks up the new values on
    the next iteration.  pause_sending() / resume_sending() simulate the
    FC going silent or coming back.
    """

    MAV_STATE_ACTIVE = 4
    MAV_STATE_CRITICAL = 5
    MAV_STATE_EMERGENCY = 6
    MAV_STATE_FLIGHT_TERMINATION = 8
    MAV_CMD_DO_PARACHUTE = 208
    PARACHUTE_ACTION_RELEASE = 2

    _SEND_INTERVAL_S = 0.05

    def __init__(self, connection: mavutil.mavlink_connection):
        self.conn = connection
        # We act like the FC booted a while ago, this is necessary for reboot detection
        self.boot_timestamp_s = time.monotonic() - 15.0

        self._lock = threading.Lock()
        self._armed = False
        self._system_status = self.MAV_STATE_ACTIVE
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
        msg = MAVLink_aviant_detailed_fc_state_message(
            time_boot_ms=self._time_boot_ms(),
            time_unix_usec=int(time.time() * 1e6),
            armed=1 if self._armed else 0,
            vtol_state=0,
            system_status=self._system_status,
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
                command=self.MAV_CMD_DO_PARACHUTE,
                confirmation=0,
                param1=float(self.PARACHUTE_ACTION_RELEASE),
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
                param_type=_mavlink_mod.MAV_PARAM_TYPE_REAL32,
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
            if msg.command == self.MAV_CMD_DO_PARACHUTE:
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
            if msg.command == self.MAV_CMD_DO_PARACHUTE:
                return False
        return True

    def shutdown(self) -> None:
        """Stop the background sender thread."""
        self._stop_event.set()
        self._thread.join(timeout=3)
