"""SharedReader and system ID tests. Fast, no simulator needed."""

import socket
import threading
import time

import pytest

from zenmav.zenlink import SharedReader, default_system_id, free_system_id
from zenmav.zenrelay import MavRelay, mavutil

from relay_helpers import UdpDrone, free_port, ml


@pytest.fixture
def link():
    vehicle_port = free_port(socket.SOCK_DGRAM)
    relay = MavRelay(mavutil.mavlink_connection(f"udpin:127.0.0.1:{vehicle_port}"), tcp_ports=[], bind="127.0.0.1")
    connection = relay.add_internal_link(15)
    reader = SharedReader(connection)
    drone = UdpDrone(vehicle_port)
    yield connection, reader, drone
    connection.close()
    relay.close()
    drone.close()


def attitude(drone):
    return drone.enc.pack(ml.MAVLink_attitude_message(0, 0, 0, 0, 0, 0, 0))


def sys_status(drone):
    return drone.enc.pack(ml.MAVLink_sys_status_message(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))


def test_threads_do_not_steal_each_other_messages(link):
    """The fence thread and the main thread used to swallow each other's messages."""
    connection, _, drone = link
    counts = {"ATTITUDE": 0, "SYS_STATUS": 0}
    registered = threading.Barrier(3)
    done = threading.Event()

    def consumer(msg_type):
        connection.recv_match(type=msg_type, blocking=False)  # starts this thread's stream
        registered.wait()
        while not done.is_set():
            if connection.recv_match(type=msg_type, blocking=True, timeout=0.2):
                counts[msg_type] += 1

    threads = [threading.Thread(target=consumer, args=(t,)) for t in counts]
    for t in threads:
        t.start()
    registered.wait()
    for _ in range(100):
        drone.send(attitude(drone) + sys_status(drone))
        time.sleep(0.005)
    time.sleep(0.5)
    done.set()
    for t in threads:
        t.join()
    assert counts == {"ATTITUDE": 100, "SYS_STATUS": 100}


def test_reply_received_before_first_read_is_kept(link):
    connection, _, drone = link
    result = []

    def late_reader():
        time.sleep(0.2)  # the reply arrives before this thread ever reads
        result.append(connection.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0))

    t = threading.Thread(target=late_reader)
    t.start()
    drone.send(sys_status(drone))
    t.join()
    assert result[0] is not None


def test_flush_discards_old_answers(link):
    """An idle thread must not read an old PARAM_VALUE as the answer to its new request."""
    connection, reader, drone = link
    connection.recv_match(blocking=False)  # starts the main thread's stream
    drone.send(sys_status(drone))
    assert connection.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0) is not None
    for _ in range(3):
        drone.send(sys_status(drone))
    time.sleep(0.3)
    reader.flush()
    assert connection.recv_match(type="SYS_STATUS", blocking=False) is None
    drone.send(sys_status(drone))
    assert connection.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0) is not None


def test_timeout_and_close(link):
    connection, reader, _ = link
    start = time.monotonic()
    assert connection.recv_match(type="ATTITUDE", blocking=True, timeout=0.3) is None
    assert 0.25 <= time.monotonic() - start < 1.0
    connection.close()
    assert reader.closed
    start = time.monotonic()
    assert connection.recv_match(blocking=True) is None
    assert time.monotonic() - start < 0.5


def test_message_state_is_updated_by_the_reader(link):
    connection, _, drone = link
    drone.send(drone.enc.heartbeat())
    assert connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0) is not None
    assert "HEARTBEAT" in connection.sysid_state[1].messages


def test_system_ids():
    assert default_system_id("tcp:127.0.0.1:14550") == 15
    assert default_system_id("udpout:192.168.144.12:19856") == 19856 % 255
    assert default_system_id("COM3") is None
    assert default_system_id("/dev/ttyACM0") is None
    assert free_system_id(set(), preferred=16) == 16
    assert free_system_id({16}, preferred=16) == 200
    assert free_system_id({16, 200}, preferred=16) == 201
    assert free_system_id(set(), preferred=255) == 200
    assert free_system_id(set(), preferred=None, salt=3) == 203
