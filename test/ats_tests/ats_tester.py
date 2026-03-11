"""Helper class wrapping pymavlink with ATS-specific MAVLink interactions.

Deploy detection listens for COMMAND_LONG messages on the MAVLink
connection.  The ATS module publishes vehicle_command to uORB which the
MAVLink module forwards as MAV_CMD_DO_PARACHUTE (target component 161)
and MAV_CMD_DO_FLIGHTTERMINATION (target component 1).
"""

from __future__ import annotations

import time

from pymavlink import mavutil


class ATSTester:
    """Drives ATS test scenarios over MAVLink and verifies deploy commands."""

    MAV_AUTOPILOT_PX4 = 12
    MAV_TYPE_GENERIC = 0
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_CMD_DO_PARACHUTE = 208
    MAV_CMD_DO_FLIGHTTERMINATION = 185
    PARACHUTE_ACTION_RELEASE = 2

    def __init__(self, connection: mavutil.mavlink_connection):
        self.conn = connection

    def send_fc_heartbeat(self, armed: bool) -> None:
        """Send a HEARTBEAT that looks like it comes from the main FC."""
        base_mode = self.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        self.conn.mav.heartbeat_send(
            type=self.MAV_TYPE_GENERIC,
            autopilot=self.MAV_AUTOPILOT_PX4,
            base_mode=base_mode,
            custom_mode=0,
            system_status=4,  # MAV_STATE_ACTIVE
        )

    def send_fc_attitude(self, roll: float = 0.0, pitch: float = 0.0,
                         yaw: float = 0.0) -> None:
        """Send an ATTITUDE message (populates external_ins_attitude uORB)."""
        self.conn.mav.attitude_send(
            time_boot_ms=int(time.monotonic() * 1000) & 0xFFFFFFFF,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
        )

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
        """Send attitude + heartbeat at *interval_s* for *duration_s*.

        Attitude is sent BEFORE heartbeat so that fc_timeout is cleared
        before any arm-state transition is processed by the ATS.
        """
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.send_fc_attitude()
            self.send_fc_heartbeat(armed)
            time.sleep(interval_s)

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
