"""Mock components for ATS SITL testing over MAVLink.

* **FCMock** (component 1 / MAV_COMP_ID_AUTOPILOT1) – flight controller.
  A background thread sends AVIANT_DETAILED_FC_STATE at ~30 Hz.  Tests
  control the emulated FC through set_armed() and set_system_status(),
  and can simulate FC silence via pause_sending().  Carries observation
  helpers for FC-targeted traffic (expect_flighttermination,
  get_ats_status, …).

* **ParachuteMock** (component 161 / MAV_COMP_ID_PARACHUTE) – parachute.
  A background thread publishes heartbeats at 1 Hz.  Can send
  DO_FLIGHTTERMINATION and force-disarm commands.  Carries deploy
  observation helpers (expect_deploy, expect_no_deploy) and command-ack
  helpers (expect_command_ack, expect_no_command_ack).

Requires the ``aviant`` MAVLink dialect (set MAVLINK_DIALECT=aviant and
MDEF pointing at the in-tree message_definitions before importing
pymavlink).  See conftest.py.
"""

from __future__ import annotations

from contextlib import contextmanager
import threading
import time
from pymavlink import mavutil

mavlink = mavutil.mavlink


class MAVLinkInterface:
    """Owns a ``udpin`` MAVLink connection and the machinery shared by
    every mock: a thread-safe lock and receive helpers (drain /
    recv_until).
    """

    def __init__(self, port: int, component_id: int):
        self.conn = mavutil.mavlink_connection(
            f'udpin:0.0.0.0:{port}',
            source_system=1,
            source_component=component_id,
        )
        self.conn.wait_heartbeat(timeout=15)
        self.lock = threading.Lock()

    def drain(self) -> None:
        """Discard all buffered messages."""
        while self.conn.recv_match(blocking=False) is not None:
            pass

    def recv_until(self, predicate, timeout_s: float = 5.0):
        """Read messages until *predicate(msg)* returns True or timeout.

        Returns the matching message, or None.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg is not None and predicate(msg):
                return msg
        return None

    def close(self) -> None:
        self.conn.close()


class FCMock:
    """Mock flight controller (MAV_COMP_ID_AUTOPILOT1, component 1)."""

    SYS_ID = 1
    COMP_ID = 1
    ATS_COMP_ID = 60

    _SEND_INTERVAL_S = 1.0 / 30.0

    def __init__(self, port: int):
        self.mav = MAVLinkInterface(port, self.COMP_ID)

        # Pretend the FC booted a while ago (needed for reboot detection)
        self.boot_timestamp_s = time.monotonic() - 15.0

        self._armed = False
        self._system_status = mavlink.MAV_STATE_ACTIVE
        self._sending = True

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

    @property
    def conn(self):
        return self.mav.conn

    def _send_loop(self) -> None:
        while not self._stop_event.is_set():
            with self.mav.lock:
                if self._sending:
                    self._send_fc_state()
            self._stop_event.wait(timeout=self._SEND_INTERVAL_S)

    def _time_boot_ms(self) -> int:
        return int((time.monotonic() - self.boot_timestamp_s) * 1000)

    def _send_fc_state(self) -> None:
        """Build and send one FC state message.  Caller must hold lock."""
        msg = mavlink.MAVLink_aviant_detailed_fc_state_message(
            self._time_boot_ms(),  # time_boot_ms
            int(time.time() * 1e6),  # time_unix_usec
            1 if self._armed else 0,  # fc_armed
            0,  # fc_flight_termination
        )
        self.mav.conn.mav.send(msg)

    def set_armed(self, armed: bool) -> None:
        with self.mav.lock:
            self._armed = armed

    def set_system_status(self, status: int) -> None:
        with self.mav.lock:
            self._system_status = status

    def pause_sending(self) -> None:
        """Stop the background FC state stream (simulates FC going silent)."""
        with self.mav.lock:
            self._sending = False

    def resume_sending(self) -> None:
        """Resume the background FC state stream."""
        with self.mav.lock:
            self._sending = True

    def simulate_fc_reboot(self) -> None:
        """Simulate an FC reboot by resetting time_boot_ms to a small value.

        The ATS detects a reboot when time_boot_ms drops by more than 10 s
        compared to the previous message.  Resetting boot_timestamp_s makes
        subsequent messages produce small time_boot_ms values.  The
        background thread continues sending with the new epoch.
        """
        with self.mav.lock:
            self.boot_timestamp_s = time.monotonic()
            self._armed = False

    def set_ats_param(self, param_id: str, value: float) -> None:
        """Set a PX4 parameter via MAVLink PARAM_SET."""
        with self.mav.lock:
            self.mav.conn.mav.param_set_send(
                target_system=self.SYS_ID,
                target_component=self.ATS_COMP_ID,
                param_id=param_id.encode('utf-8'),
                param_value=value,
                param_type=mavlink.MAV_PARAM_TYPE_REAL32,
            )

    @contextmanager
    def expect_flighttermination(self, timeout_s: float = 10.0):
        """Context: drain buffer, yield, then assert DO_FLIGHTTERMINATION arrived."""
        self.mav.drain()
        yield
        msg = self.mav.recv_until(
            lambda m: (m.get_type() == 'COMMAND_LONG'
                       and m.command == mavlink.MAV_CMD_DO_FLIGHTTERMINATION),
            timeout_s=timeout_s,
        )
        assert msg is not None, \
            f"Expected DO_FLIGHTTERMINATION COMMAND_LONG within {timeout_s}s"

    @contextmanager
    def expect_force_disarm(self, timeout_s: float = 3.0):
        """Context: drain buffer, yield, then assert force-disarm COMPONENT_ARM_DISARM arrived."""
        self.mav.drain()
        yield
        msg = self.mav.recv_until(
            lambda m: (m.get_type() == 'COMMAND_LONG'
                       and m.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                       and m.param2 == 21196.0),
            timeout_s=timeout_s,
        )
        assert msg is not None, \
            f"Expected force-disarm COMPONENT_ARM_DISARM COMMAND_LONG within {timeout_s}s"

    def get_ats_status(self, timeout_s: float = 3.0):
        """Receive the next AVIANT_ATS_STATUS message."""
        self.mav.drain()
        return self.mav.recv_until(
            lambda m: m.get_type() == 'AVIANT_ATS_STATUS',
            timeout_s=timeout_s,
        )

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
            ('REBOOTED_WHILE_ARMED',mavlink.AVIANT_ATS_STATUS_FLAG_REBOOTED_WHILE_ARMED),
            ('PARACHUTE_DEPLOY',    mavlink.AVIANT_ATS_STATUS_FLAG_PARACHUTE_DEPLOY),
            ('INTERNAL_FAILURE',    mavlink.AVIANT_ATS_STATUS_FLAG_INTERNAL_FAILURE),
        ]
        names = [n for n, v in _FLAG_NAMES if flags & v]
        return ' | '.join(names) if names else '(none)'

    def shutdown(self) -> None:
        time.sleep(0.5)
        self._stop_event.set()
        self._thread.join(timeout=3)
        self.mav.close()


class ParachuteMock:
    """Mock parachute (MAV_COMP_ID_PARACHUTE, component 161)."""

    SYS_ID = 1
    COMP_ID = 161
    _HEARTBEAT_INTERVAL_S = 1.0

    def __init__(self, port: int):
        self.mav = MAVLinkInterface(port, self.COMP_ID)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._thread.start()

    @property
    def conn(self):
        return self.mav.conn

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.is_set():
            with self.mav.lock:
                self.mav.conn.mav.heartbeat_send(
                    mavlink.MAV_TYPE_GENERIC,
                    mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0,
                    mavlink.MAV_STATE_ACTIVE,
                )
            self._stop_event.wait(timeout=self._HEARTBEAT_INTERVAL_S)

    def send_flighttermination_command(self, target_component: int = 1) -> None:
        """Send MAV_CMD_DO_FLIGHTTERMINATION."""
        with self.mav.lock:
            self.mav.conn.mav.command_long_send(
                target_system=self.SYS_ID,
                target_component=target_component,
                command=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                confirmation=0,
                param1=1.0,
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
            )

    def send_force_disarm(self, target_component: int = 1) -> None:
        """Send MAV_CMD_COMPONENT_ARM_DISARM with force-disarm parameters."""
        with self.mav.lock:
            self.mav.conn.mav.command_long_send(
                target_system=self.SYS_ID,
                target_component=target_component,
                command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                confirmation=0,
                param1=0.0,
                param2=21196.0,
                param3=0, param4=0, param5=0, param6=0, param7=0,
            )

    def send_disarm(self, target_component: int = 1) -> None:
        """Send MAV_CMD_COMPONENT_ARM_DISARM without force flag (param2=0)."""
        with self.mav.lock:
            self.mav.conn.mav.command_long_send(
                target_system=self.SYS_ID,
                target_component=target_component,
                command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                confirmation=0,
                param1=0.0,
                param2=0.0,
                param3=0, param4=0, param5=0, param6=0, param7=0,
            )

    @contextmanager
    def expect_deploy(self, timeout_s: float = 10.0):
        """Context: drain buffer, yield, then assert DO_PARACHUTE arrived."""
        self.mav.drain()
        yield
        msg = self.mav.recv_until(
            lambda m: (m.get_type() == 'COMMAND_LONG'
                       and m.command == mavlink.MAV_CMD_DO_PARACHUTE),
            timeout_s=timeout_s,
        )
        assert msg is not None, \
            f"Expected DO_PARACHUTE COMMAND_LONG within {timeout_s}s"

    @contextmanager
    def expect_no_deploy(self, duration_s: float = 3.0):
        """Context: drain buffer, yield, then assert NO DO_PARACHUTE arrived."""
        self.mav.drain()
        yield
        msg = self.mav.recv_until(
            lambda m: (m.get_type() == 'COMMAND_LONG'
                       and m.command == mavlink.MAV_CMD_DO_PARACHUTE),
            timeout_s=duration_s,
        )
        assert msg is None, "Unexpected DO_PARACHUTE COMMAND_LONG received"

    @contextmanager
    def expect_command_ack(self, command: int, result: int | None = None,
                           source_component: int | None = None,
                           timeout_s: float = 5.0):
        """Context: drain buffer, yield, then assert matching COMMAND_ACK."""
        self.mav.drain()
        yield
        def _match(m):
            if m.get_type() != 'COMMAND_ACK' or m.command != command:
                return False
            if result is not None and m.result != result:
                return False
            if source_component is not None and m.get_srcComponent() != source_component:
                return False
            return True
        msg = self.mav.recv_until(_match, timeout_s=timeout_s)
        assert msg is not None, \
            f"Expected COMMAND_ACK(cmd={command}, result={result}, srcComp={source_component}) within {timeout_s}s"

    @contextmanager
    def expect_no_command_ack(self, command: int, result: int | None = None,
                              duration_s: float = 2.0):
        """Context: drain buffer, yield, then assert NO matching COMMAND_ACK."""
        self.mav.drain()
        yield
        def _match(m):
            if m.get_type() != 'COMMAND_ACK' or m.command != command:
                return False
            return result is None or m.result == result
        msg = self.mav.recv_until(_match, timeout_s=duration_s)
        assert msg is None, \
            f"Unexpected COMMAND_ACK(cmd={command}, result={result})"

    def shutdown(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=3)
        self.mav.close()
