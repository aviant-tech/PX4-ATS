"""pytest fixtures for ATS SITL tests.

Each test gets a fresh PX4 SITL instance so that reboot-required
parameters and post-deploy lockdown state do not leak between tests.

ATS parameters are passed as environment variables to the PX4 process.
The airframe .post script reads them and applies ``param set`` before
starting the aviant_ats module.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import threading
import time

import psutil
import pytest
from pymavlink import mavutil

from ats_tester import ATSTester


WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DEFAULT_BUILD_DIR = os.path.join(WORKSPACE, 'build', 'px4_sitl_ats')

MAV_PORT = 14540


def pytest_addoption(parser):
    parser.addoption(
        '--build-dir', action='store', default=DEFAULT_BUILD_DIR,
        help='Path to the px4_sitl_ats build directory',
    )


# ------------------------------------------------------------------
# PX4 lifecycle helpers
# ------------------------------------------------------------------

def _kill_existing_px4():
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == 'px4':
            proc.kill()
            proc.wait(timeout=5)


class PX4Instance:
    """Wraps a running PX4 SITL process with stdout-based boot detection."""

    def __init__(self, proc: subprocess.Popen):
        self.proc = proc
        self._lines: list[str] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        assert self.proc.stdout is not None
        while not self._stop.is_set():
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    break
                time.sleep(0.01)
                continue
            with self._lock:
                self._lines.append(line)

    def wait_for_line(self, needle: str, timeout_s: float = 30.0) -> bool:
        """Wait until a line containing *needle* (case-insensitive) appears."""
        deadline = time.monotonic() + timeout_s
        seen = 0
        while time.monotonic() < deadline:
            with self._lock:
                while seen < len(self._lines):
                    if needle.lower() in self._lines[seen].lower():
                        return True
                    seen += 1
            time.sleep(0.05)
        return False

    def stop(self):
        self._stop.set()
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3)
        self._reader.join(timeout=3)


def _start_px4(build_dir: str,
               ats_params: dict[str, str] | None = None) -> PX4Instance:
    rootfs = os.path.join(build_dir, 'tmp_ats_tests', 'rootfs')

    if os.path.isdir(rootfs):
        for item in os.listdir(rootfs):
            if item == 'log':
                continue
            path = os.path.join(rootfs, item)
            if os.path.isfile(path) or os.path.islink(path):
                os.remove(path)
            else:
                shutil.rmtree(path)
    os.makedirs(rootfs, exist_ok=True)

    env = os.environ.copy()
    env['PX4_SIM_MODEL'] = 'sihsim_aviant_ats'
    env['PX4_SIM_SPEED_FACTOR'] = '1'

    if ats_params:
        env.update(ats_params)

    px4_bin = os.path.join(build_dir, 'bin', 'px4')
    etc_dir = os.path.join(build_dir, 'etc')

    proc = subprocess.Popen(
        [
            px4_bin,
            etc_dir,
            '-s', 'etc/init.d-posix/rcS',
            '-d',
        ],
        cwd=rootfs,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )

    instance = PX4Instance(proc)

    if not instance.wait_for_line('startup script returned', timeout_s=30):
        instance.stop()
        raise TimeoutError('PX4 did not complete boot within timeout')

    return instance


# ------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------

@pytest.fixture()
def px4(request):
    """Start a fresh PX4 SITL for each test, with optional ATS params.

    Tests provide ``ats_params`` via ``pytest.mark.parametrize`` on the
    indirect ``px4`` fixture, or via a helper ``ats_params`` fixture.
    """
    build_dir = os.path.abspath(request.config.getoption('--build-dir'))

    if not os.path.isfile(os.path.join(build_dir, 'bin', 'px4')):
        pytest.skip(
            f'PX4 binary not found at {build_dir}/bin/px4 – '
            'run `DONT_RUN=1 make px4_sitl_ats` first')

    params = getattr(request, 'param', None)

    _kill_existing_px4()
    instance = _start_px4(build_dir, ats_params=params)

    yield instance

    instance.stop()
    _kill_existing_px4()


@pytest.fixture()
def tester(px4):
    """Provide an ATSTester connected to the running PX4 instance.

    Blocks until EKF2 achieves tilt alignment (publishes vehicle_attitude)
    so that sensor-dependent ATS triggers work reliably.  In SIH the high
    simulated IMU noise makes this take ~30 s.
    """
    time.sleep(1)

    conn = mavutil.mavlink_connection(
        f'udpin:0.0.0.0:{MAV_PORT}',
        source_system=1,
        source_component=2,
    )

    conn.wait_heartbeat(timeout=15)

    # Wait for EKF2 tilt alignment.  The ATTITUDE MAVLink message is only
    # sent once EKF2 publishes vehicle_attitude (requires tilt_align=true).
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        msg = conn.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg is not None:
            break
    else:
        raise TimeoutError('EKF2 did not achieve tilt alignment within 60 s')

    t = ATSTester(conn)

    # Let the system settle and establish baseline state.
    t.keep_alive(duration_s=5.0, armed=False)

    # Drain any stale COMMAND_LONG messages from the boot period.
    t._drain_command_long()

    yield t

    conn.close()
