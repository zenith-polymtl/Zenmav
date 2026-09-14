"""ArduCopter SITL and fake ground station used by the SITL scenarios."""

import json
import os
import subprocess
import threading
import time

os.environ["MAVLINK20"] = "1"  # before any pymavlink import, also when testing older Zenmav sources
from pymavlink import mavutil  # noqa: E402

ml = mavutil.mavlink

ARDUPILOT = os.environ.get("ARDUPILOT_DIR", os.path.expanduser("~/ardupilot"))
ARDUCOPTER = os.path.join(ARDUPILOT, "build", "sitl", "bin", "arducopter")
REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "src"))
HOME = (45.5048, -73.6135, 50.0)


def record(name, **values):
    """Prints a metric and appends it to $ZENMAV_SITL_RESULTS (JSON lines) to compare runs."""
    print(f"[metrics] {name}: {values}")
    path = os.environ.get("ZENMAV_SITL_RESULTS")
    if path:
        with open(path, "a") as f:
            f.write(json.dumps({"test": name, "src": os.environ.get("ZENMAV_SRC", "repo"), **values}) + "\n")


def wait_until(predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return bool(predicate())


class Sitl:
    """ArduCopter SITL. serial0 = tcp 5760 (link under test), serial1 = tcp 5762 (test probe)."""

    PROBE_PORT = 5762

    def __init__(self, workdir, serial2="tcp:3", params=None):
        cmd = [
            ARDUCOPTER, "--model", "+", "--speedup", "1", "-w",
            "--home", f"{HOME[0]},{HOME[1]},{HOME[2]},0",
            "--defaults", os.path.join(ARDUPILOT, "Tools", "autotest", "default_params", "copter.parm"),
            "--serial0", "tcp:0", "--serial1", "tcp:2", "--serial2", serial2,
        ]
        self._log = open(os.path.join(workdir, "arducopter.log"), "w")
        self.proc = subprocess.Popen(cmd, cwd=workdir, stdout=self._log, stderr=subprocess.STDOUT)
        self.statustexts = []
        self.attitude_samples = []  # (monotonic time, time_boot_ms) received directly, without relay
        self._stop = threading.Event()
        self.probe = mavutil.mavlink_connection(
            f"tcp:127.0.0.1:{self.PROBE_PORT}", source_system=254, source_component=250, retries=30
        )
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        assert wait_until(lambda: self.probe.sysid != 0, 60), "no heartbeat from SITL"
        for name, value in (params or {}).items():
            self.set_param(name, value)
        self.wait_ready()

    def _pump(self):
        while not self._stop.is_set():
            try:
                m = self.probe.recv_match(blocking=True, timeout=0.2)
            except Exception:
                time.sleep(0.1)
                continue
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                self.statustexts.append(m.text)
            elif m.get_type() == "ATTITUDE":
                self.attitude_samples.append((time.monotonic(), m.time_boot_ms))

    def request(self, msg_id, hz):
        self.probe.mav.command_long_send(
            1, 1, ml.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0
        )

    def param(self, name, default=None):
        state = self.probe.param_state.get((1, 1))
        return state.params.get(name, default) if state else default

    def set_param(self, name, value):
        for _ in range(10):
            self.probe.param_set_send(name, value)
            if wait_until(lambda: abs(self.param(name, 1e9) - value) < 1e-4, 1.0):
                return
        raise RuntimeError(f"could not set {name}")

    def wait_ready(self, timeout=180):
        for msg_id in (ml.MAVLINK_MSG_ID_SYS_STATUS, ml.MAVLINK_MSG_ID_GPS_RAW_INT):
            self.request(msg_id, 2)

        def ready():
            status = self.probe.messages.get("SYS_STATUS")
            gps = self.probe.messages.get("GPS_RAW_INT")
            return (
                status is not None and gps is not None and gps.fix_type >= 3
                and status.onboard_control_sensors_health & ml.MAV_SYS_STATUS_PREARM_CHECK
            )

        assert wait_until(ready, timeout), "SITL is not ready to arm"

    def mode(self):
        return self.probe.flightmode

    def armed(self):
        return bool(self.probe.motors_armed())

    def close(self):
        self._stop.set()
        self._thread.join(2)
        try:
            self.probe.close()
        except Exception:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self._log.close()


class FakeGcs:
    """Stands for Mission Planner: sends GCS heartbeats and records every message it receives."""

    def __init__(self, url, sysid=255, hz=1.0):
        self.conn = mavutil.mavlink_connection(url, source_system=sysid, source_component=190, retries=20)
        self.period = 1.0 / hz
        self.heartbeats = threading.Event()
        self.heartbeats.set()
        self.received = []  # (monotonic time, message)
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=f, daemon=True) for f in (self._send, self._recv)]
        for t in self._threads:
            t.start()

    def _send(self):
        while not self._stop.is_set():
            if self.heartbeats.is_set():
                try:
                    self.conn.mav.heartbeat_send(ml.MAV_TYPE_GCS, ml.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                except Exception:
                    pass
            self._stop.wait(self.period)

    def _recv(self):
        while not self._stop.is_set():
            try:
                m = self.conn.recv_match(blocking=True, timeout=0.2)
            except Exception:
                time.sleep(0.05)
                continue
            if m is not None and m.get_type() != "BAD_DATA":
                self.received.append((time.monotonic(), m))

    def messages(self, msg_type, since=0.0):
        return [(t, m) for t, m in list(self.received) if m.get_type() == msg_type and t >= since]

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(1)
        try:
            self.conn.close()
        except Exception:
            pass
