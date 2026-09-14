"""Network scenarios against ArduCopter SITL: several Zenmav instances, GCS mode, failsafe, faults, load.

Run from Linux/WSL with ArduPilot built (./waf copter):
    ARDUPILOT_DIR=~/ardupilot python3 -m pytest tests/sitl -v
ZENMAV_SRC selects the Zenmav sources under test, ZENMAV_SITL_RESULTS appends metrics as JSON lines.
"""

import os
import socket
import subprocess
import sys
import threading
import time

import pytest

from sitl_helpers import ARDUCOPTER, HOME, REPO_SRC, FakeGcs, ml, record, wait_until

if not os.path.exists(ARDUCOPTER):
    pytest.skip(f"ArduCopter SITL not built ({ARDUCOPTER})", allow_module_level=True)

pytestmark = pytest.mark.timeout(420)

from zenmav.core import Zenmav  # noqa: E402
from zenmav.zenpoint import wp  # noqa: E402

UDP_LINK_PORT = 14560


def close_all(*drones):
    for drone in drones:
        if drone is not None:
            try:
                drone.close_all_connections()
            except Exception as e:
                print(f"close failed: {e!r}")


def land_and_disarm(drone, sitl):
    drone.set_mode("LAND")
    assert wait_until(lambda: not sitl.armed(), 90), "vehicle did not land"


def test_zenmav_locks_onto_the_vehicle_when_gcs_heartbeats_arrive_first(sitl):
    """Historical bug: Zenmav mistook another ground station's heartbeat for the vehicle."""
    relay = subprocess.Popen([
        sys.executable, os.path.join(os.path.dirname(__file__), "relay_proc.py"),
        REPO_SRC, "tcp:127.0.0.1:5760", "14571,14572",
    ])
    gcs = drone = None
    try:
        gcs = FakeGcs("tcp:127.0.0.1:14571", hz=20)  # 20 GCS heartbeats for every vehicle heartbeat
        time.sleep(1.0)
        drone = Zenmav("tcp:127.0.0.1:14572")
        mode_map = drone.connection.mode_mapping() or {}
        record("gcs_heartbeat_first", target_system=drone.connection.target_system, guided_known="GUIDED" in mode_map)
        assert drone.connection.target_system == 1
        assert "GUIDED" in mode_map
        drone.set_mode("GUIDED")
        assert wait_until(lambda: sitl.mode() == "GUIDED", 5)
    finally:
        close_all(drone)
        if gcs is not None:
            gcs.close()
        relay.terminate()
        relay.wait(5)


def test_several_instances_in_gcs_mode(sitl, tmp_path):
    fence = tmp_path / "fence.toml"
    lat, lon, _ = HOME
    fence.write_text(
        'version = 1\nname = "sitl"\nframe = "global"\nmargin = 0.0\naction = "brake"\n'
        '[[regions]]\nkind = "include-polygon"\n'
        f"latlon = [[{lat - 0.002},{lon - 0.003}],[{lat + 0.002},{lon - 0.003}],"
        f"[{lat + 0.002},{lon + 0.003}],[{lat - 0.002},{lon + 0.003}]]\n"
    )
    a = b = c = mp = None
    stop = threading.Event()
    try:
        # 14552 % 255 == 14807 % 255 == 17: b and c start with the same system ID
        a = Zenmav("tcp:127.0.0.1:5760", GCS=True, tcp_ports=[14551, 14552, 14807], boundary_path=str(fence))
        mp = FakeGcs("tcp:127.0.0.1:14551")
        b = Zenmav("tcp:127.0.0.1:14552")
        c = Zenmav("tcp:127.0.0.1:14807")
        ids = [d.connection.mav.srcSystem for d in (a, b, c)]
        record("system_ids", ids=ids)
        assert len(set(ids)) == 3, f"system IDs are not unique: {ids}"

        positions = []

        def watch():
            while not stop.is_set():
                positions.append(b.get_global_pos())

        watcher = threading.Thread(target=watch, daemon=True)
        watcher.start()

        speed = c.get_param("WP_SPD")
        assert c.set_param("WP_SPD", 7.0)
        assert abs(a.get_param("WP_SPD") - 7.0) < 1e-3
        assert c.set_param("WP_SPD", speed)

        flight_start = time.monotonic()
        a.guided_arm_takeoff(5)
        a.local_target(wp(8, 0, -5, frame="local"), acceptance_radius=1.0)
        land_and_disarm(a, sitl)
        stop.set()
        watcher.join(5)

        telemetry = [t for t, _ in mp.messages("GLOBAL_POSITION_INT", since=flight_start)]
        max_gap = max((t2 - t1 for t1, t2 in zip(telemetry, telemetry[1:])), default=float("inf"))
        foreign_acks = [
            m.target_system for _, m in mp.messages("COMMAND_ACK") if m.target_system not in (0, 255)
        ]
        record(
            "several_instances",
            watcher_positions=len(positions),
            mp_position_rate=len(telemetry) / max(time.monotonic() - flight_start, 1e-3),
            mp_max_gap_s=max_gap,
            acks_for_other_systems=len(foreign_acks),
        )
        assert len(positions) > 50
        assert max_gap < 1.0
        assert foreign_acks == [], "Mission Planner received command acknowledgements meant for other instances"
    finally:
        stop.set()
        close_all(c, b, a)
        if mp is not None:
            mp.close()


def test_gcs_failsafe_through_the_relay(sitl):
    """ArduPilot only arms its GCS failsafe after it has seen a heartbeat from MAV_GCS_SYSID (255)."""
    sitl.set_param("FS_GCS_ENABLE", 1)
    a = mp = None
    try:
        a = Zenmav("tcp:127.0.0.1:5760", GCS=True, tcp_ports=[14551])
        mp = FakeGcs("tcp:127.0.0.1:14551", sysid=255, hz=1)
        time.sleep(3)
        a.guided_arm_takeoff(5)
        mp.heartbeats.clear()  # Mission Planner goes silent
        lost = time.monotonic()
        triggered = wait_until(lambda: sitl.mode() in ("RTL", "LAND", "SMART_RTL"), 20)
        record("gcs_failsafe", triggered=triggered, delay_s=time.monotonic() - lost, mode=sitl.mode())
        assert triggered, "GCS failsafe did not trigger"
        assert wait_until(lambda: not sitl.armed(), 120)
    finally:
        close_all(a)
        if mp is not None:
            mp.close()


@pytest.mark.sitl_options(serial2=f"udpclient:127.0.0.1:{UDP_LINK_PORT}")
def test_relay_on_udp_link_survives_client_churn(sitl):
    a = mp1 = mp2 = mp3 = None
    try:
        a = Zenmav(f"udpin:0.0.0.0:{UDP_LINK_PORT}", GCS=True, tcp_ports=[14551])
        a.get_global_pos()
        mp1 = FakeGcs("tcp:127.0.0.1:14551")
        mp2 = FakeGcs("tcp:127.0.0.1:14551")
        both = wait_until(lambda: mp1.messages("GLOBAL_POSITION_INT") and mp2.messages("GLOBAL_POSITION_INT"), 5)
        record("two_clients_same_port", both_receive=bool(both))
        assert both, "two ground stations on the same port do not both receive telemetry"

        mp1.close()
        mp1 = None
        time.sleep(1)
        mp3 = FakeGcs("tcp:127.0.0.1:14551")
        assert wait_until(lambda: mp3.messages("GLOBAL_POSITION_INT"), 5), "reconnected client gets no telemetry"

        ghost = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # a UDP peer that disappears
        ghost.sendto(bytes(ml.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3).pack(ml.MAVLink(None, 200, 0))),
                     ("127.0.0.1", UDP_LINK_PORT))
        time.sleep(0.3)
        ghost.close()
        for _ in range(5):  # the relay now also writes to the vanished peer
            mp3.conn.mav.command_long_send(1, 1, ml.MAV_CMD_REQUEST_MESSAGE, 0, ml.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
            time.sleep(0.1)
        count = len(mp3.messages("GLOBAL_POSITION_INT"))
        time.sleep(2)
        after = len(mp3.messages("GLOBAL_POSITION_INT"))
        record("udp_peer_vanished", positions_in_2s=after - count)
        assert after - count > 20
        assert a.get_global_pos() is not None
    finally:
        close_all(a)
        for mp in (mp1, mp2, mp3):
            if mp is not None:
                mp.close()


@pytest.mark.sitl_options(serial2=f"udpclient:127.0.0.1:{UDP_LINK_PORT}")
def test_telemetry_rate_and_lag_through_the_relay_on_udp(sitl):
    """ArduPilot sends several messages per UDP datagram: the old relay fell behind above ~215 msg/s."""
    a = mp = None
    try:
        a = Zenmav(f"udpin:0.0.0.0:{UDP_LINK_PORT}", GCS=True, tcp_ports=[14551])
        a.get_attitude()  # 100 Hz
        a.get_global_pos()  # 60 Hz
        a.get_local_pos()  # 60 Hz
        a.get_rc_value(3)  # 60 Hz
        for msg_id in (
            ml.MAVLINK_MSG_ID_RAW_IMU, ml.MAVLINK_MSG_ID_SCALED_IMU2, ml.MAVLINK_MSG_ID_SCALED_PRESSURE,
            ml.MAVLINK_MSG_ID_VFR_HUD, ml.MAVLINK_MSG_ID_GPS_RAW_INT, ml.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        ):
            a.message_request(msg_id, 50)
        sitl.request(ml.MAVLINK_MSG_ID_ATTITUDE, 10)  # reference measured without the relay
        mp = FakeGcs("tcp:127.0.0.1:14551")
        time.sleep(3)
        start, duration = time.monotonic(), 20.0
        time.sleep(duration)

        def lag_growth(samples):
            if len(samples) < 2:
                return float("inf")
            t0, b0 = samples[0]
            lags = [(t - t0) - (b - b0) / 1000.0 for t, b in samples]
            return max(lags) - min(lags)

        through_relay = [(t, m.time_boot_ms) for t, m in mp.messages("ATTITUDE", since=start)]
        direct = [(t, b) for t, b in sitl.attitude_samples if t >= start]
        total = [m for t, m in list(mp.received) if t >= start]
        relay_lag, direct_lag = lag_growth(through_relay), lag_growth(direct)
        record(
            "udp_load",
            msgs_per_s=len(total) / duration,
            attitude_hz=len(through_relay) / duration,
            lag_growth_relay_s=relay_lag,
            lag_growth_direct_s=direct_lag,
        )
        assert relay_lag - direct_lag < 0.25, "telemetry falls behind through the relay"
    finally:
        close_all(a)
        if mp is not None:
            mp.close()
