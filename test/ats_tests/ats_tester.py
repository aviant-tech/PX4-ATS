"""Helper class wrapping pymavlink with ATS-specific MAVLink interactions.

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
    """Drives ATS test scenarios over MAVLink and verifies deploy commands."""

    MAV_STATE_ACTIVE = 4
    MAV_STATE_CRITICAL = 5
    MAV_STATE_EMERGENCY = 6
    MAV_STATE_FLIGHT_TERMINATION = 8
    MAV_CMD_DO_PARACHUTE = 208
    PARACHUTE_ACTION_RELEASE = 2

    def __init__(self, connection: mavutil.mavlink_connection):
        self.conn = connection
        # Act like it has been on a while, this is necessary for reboot detection
        self.boot_timestamp_s = time.monotonic() - 15.0

    def send_fc_state(self, armed: bool = False,
                      system_status: int = MAV_STATE_ACTIVE,
                      time_boot_ms: int | None = None) -> None:
        """Send AVIANT_DETAILED_FC_STATE to update the ATS FC state."""
        if time_boot_ms is None:
            time_boot_ms = int((time.monotonic() - self.boot_timestamp_s) * 1000)
        msg = MAVLink_aviant_detailed_fc_state_message(
            time_boot_ms=time_boot_ms,
            time_unix_usec=int(time.time() * 1e6),
            armed=1 if armed else 0,
            vtol_state=0,
            system_status=system_status,
        )
        self.conn.mav.send(msg)

    def send_parachute_command(self) -> None:
        """Send MAV_CMD_DO_PARACHUTE"""
        self.conn.mav.command_long_send(
            target_system=1,
            target_component=1,
            command=self.MAV_CMD_DO_PARACHUTE,
            confirmation=0,
            param1=float(self.PARACHUTE_ACTION_RELEASE),
            param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
        )

    def keep_alive(self, duration_s: float, armed: bool = True,
                   interval_s: float = 0.05) -> None:
        """Send AVIANT_DETAILED_FC_STATE at *interval_s* for *duration_s*."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.send_fc_state(armed=armed)
            time.sleep(interval_s)

    def simulate_fc_reboot(self) -> None:
        """Simulate an FC reboot by resetting time_boot_ms to a small value.

        The ATS detects a reboot when time_boot_ms drops by more than 10 s
        compared to the previous message.  Resetting boot_timestamp_s makes
        subsequent send_fc_state() calls produce small time_boot_ms values
        automatically.
        """
        self.boot_timestamp_s = time.monotonic()
        self.send_fc_state(armed=False)

    def set_param(self, param_id: str, value: float) -> None:
        """Set a PX4 parameter via MAVLink PARAM_SET."""
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
