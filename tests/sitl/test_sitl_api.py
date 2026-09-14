"""User-facing API against ArduCopter SITL, following the README examples.

Run it on the current sources and on a previous version (ZENMAV_SRC) to check that scripts behave the same:
    ARDUPILOT_DIR=~/ardupilot python3 -m pytest tests/sitl/test_sitl_api.py -v
Step durations are recorded with ZENMAV_SITL_RESULTS.
"""

import csv
import os
import socket
import time

import pytest

from sitl_helpers import ARDUCOPTER, HOME, FakeGcs, record, wait_until

if not os.path.exists(ARDUCOPTER):
    pytest.skip(f"ArduCopter SITL not built ({ARDUCOPTER})", allow_module_level=True)

pytestmark = pytest.mark.timeout(600)

from zenmav.core import Zenmav  # noqa: E402


class Steps:
    """Runs API calls and keeps their duration."""

    def __init__(self):
        self.durations = {}

    def run(self, name, function, *args, **kwargs):
        start = time.monotonic()
        result = function(*args, **kwargs)
        self.durations[name] = round(time.monotonic() - start, 2)
        return result


def test_readme_flight(sitl, tmp_path):
    steps = Steps()
    drone = None
    try:
        drone = steps.run("init", Zenmav, "tcp:127.0.0.1:5760", gps_thresh=3)
        assert drone.param_naming in ("si", "legacy")
        assert abs(drone.home.lat - HOME[0]) < 1e-3 and abs(drone.home.lon - HOME[1]) < 1e-3

        steps.run("set_mode", drone.set_mode, "GUIDED")
        assert wait_until(lambda: sitl.mode() == "GUIDED", 5)
        steps.run("arm", drone.arm)
        steps.run("takeoff", drone.takeoff, altitude=10)
        pos = steps.run("get_global_pos", drone.get_global_pos, heading=True)
        assert 8.5 < pos.alt < 11.5
        assert pos.hdg is not None

        steps.run("local_target", drone.local_target, [15, 10, -10])
        local = drone.get_local_pos()
        # local_target returns within its acceptance_radius (5 m by default)
        assert ((local.N - 15) ** 2 + (local.E - 10) ** 2) ** 0.5 < 5.5

        steps.run("global_target", drone.global_target, (pos.lat, pos.lon, 12))
        assert abs(drone.get_global_pos().alt - 12) < 2

        def cruise():
            for _ in range(20):
                drone.speed_target([3, 0, 0])
                time.sleep(0.1)
            drone.speed_target([0, 0, 0])

        steps.run("speed_target", cruise)
        steps.run("yaw_target", drone.yaw_target, 90)
        assert wait_until(lambda: abs(drone.get_attitude()[2] - 90) < 10, 20)

        assert steps.run("get_battery", drone.get_battery) is not None
        assert 800 <= steps.run("get_rc_value", drone.get_rc_value, 3) <= 2200

        speed = drone.get_param("WP_SPD")
        assert steps.run("set_param", drone.set_param, "WP_SPD", 3.0)
        assert abs(drone.get_param("WPNAV_SPEED") - 300) < 1e-3
        assert drone.set_param("WP_SPD", speed)

        pos.name = "Take-off point"
        drone.insert_coordinates_to_csv(tmp_path / "waypoints.csv", pos)
        with open(tmp_path / "waypoints.csv") as f:
            assert len(list(csv.reader(f))) == 2
        assert drone.gimbal.set_angle(pitch=-45, yaw=0) is True

        steps.run("rectilinear_scan", drone.rectilinear_scan, detection_width=10, altitude=10, scan_radius=15)
        steps.run("RTL", drone.RTL)
        assert wait_until(lambda: not sitl.armed(), 10)
        record("readme_flight", **steps.durations)
    finally:
        if drone is not None:
            try:
                drone.close_all_connections()
            except Exception as e:
                print(f"close failed: {e!r}")


def test_gcs_mode_for_mission_planner(sitl, tmp_path):
    steps = Steps()
    drone = mp = None
    try:
        drone = steps.run("init_gcs", Zenmav, "tcp:127.0.0.1:5760", GCS=True)
        mp = FakeGcs("tcp:127.0.0.1:14551")
        drone.get_global_pos()
        assert wait_until(lambda: mp.messages("GLOBAL_POSITION_INT"), 5)

        path = tmp_path / "params.param"
        steps.run("download_all_params", drone.download_all_params, str(path))
        count = len(path.read_text().splitlines())
        assert count > 500

        steps.run("close_all_connections", drone.close_all_connections)
        drone = None
        time.sleep(1)
        with pytest.raises(OSError):
            socket.create_connection(("127.0.0.1", 14551), timeout=2).close()
        record("gcs_mode_api", params=count, **steps.durations)
    finally:
        if drone is not None:
            try:
                drone.close_all_connections()
            except Exception as e:
                print(f"close failed: {e!r}")
        if mp is not None:
            mp.close()
