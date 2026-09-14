"""Relay tests with fake MAVLink systems. Fast, no simulator needed."""

import socket
import statistics
import time
import types

import pytest

from zenmav import zenrelay
from zenmav.zenrelay import MavRelay, mavutil

from relay_helpers import Encoder, TcpClient, UdpDrone, free_port, ml, tcp_clients, wait_until


@pytest.fixture
def udp_relay():
    vehicle_port = free_port(socket.SOCK_DGRAM)
    ports = (free_port(), free_port())
    relay = MavRelay(
        mavutil.mavlink_connection(f"udpin:127.0.0.1:{vehicle_port}"), tcp_ports=ports, bind="127.0.0.1"
    )
    drone = UdpDrone(vehicle_port)
    yield types.SimpleNamespace(relay=relay, ports=ports, drone=drone, vehicle_port=vehicle_port)
    relay.close()
    drone.close()


def connect(relay, *clients_args):
    clients = [TcpClient(*args) if isinstance(args, tuple) else TcpClient(args) for args in clients_args]
    assert wait_until(lambda: tcp_clients(relay) >= len(clients))
    return clients


def test_batched_udp_telemetry_is_delivered_without_delay(udp_relay):
    """Several messages per datagram (ArduPilot, MAVProxy) used to cap the relay near 215 msg/s."""
    (client,) = connect(udp_relay.relay, udp_relay.ports[1])
    sent = 0
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        udp_relay.drone.send(b"".join(udp_relay.drone.enc.system_time() for _ in range(10)))
        sent += 10
        time.sleep(0.01)
    assert wait_until(lambda: client.count("SYSTEM_TIME") >= sent)
    assert statistics.median(client.latencies_ms) < 20
    client.close()


def test_relay_survives_udp_connection_reset(udp_relay):
    """Windows raises ConnectionResetError (10054) after writing to a closed UDP port."""
    (client,) = connect(udp_relay.relay, udp_relay.ports[0])
    gone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gone.sendto(Encoder(1).heartbeat(), ("127.0.0.1", udp_relay.vehicle_port))
    time.sleep(0.2)
    gone.close()
    command = Encoder(250, 190).pack(
        ml.MAVLink_command_long_message(1, 1, ml.MAV_CMD_REQUEST_MESSAGE, 0, 0, 0, 0, 0, 0, 0, 0)
    )
    for _ in range(5):  # the relay writes these to the closed UDP port
        client.send(command)
        time.sleep(0.05)
    time.sleep(0.3)
    for _ in range(50):
        udp_relay.drone.send(udp_relay.drone.enc.system_time())
        time.sleep(0.005)
    assert wait_until(lambda: client.count("SYSTEM_TIME") >= 50)
    assert all(t.is_alive() for t in udp_relay.relay._threads)
    client.close()


def test_several_clients_per_port_and_reconnection(udp_relay):
    relay, port, drone = udp_relay.relay, udp_relay.ports[1], udp_relay.drone
    first, second = connect(relay, port, port)
    for _ in range(20):
        drone.send(drone.enc.system_time())
    assert wait_until(lambda: first.count("SYSTEM_TIME") == 20 and second.count("SYSTEM_TIME") == 20)

    first.close()
    assert wait_until(lambda: tcp_clients(relay) == 1)
    third = TcpClient(port)
    assert wait_until(lambda: tcp_clients(relay) == 2)
    for _ in range(20):
        drone.send(drone.enc.system_time())
    assert wait_until(lambda: third.count("SYSTEM_TIME") == 20 and second.count("SYSTEM_TIME") == 40)
    second.close()
    third.close()


def test_gcs_heartbeats_reach_the_vehicle_but_not_other_clients(udp_relay):
    relay, drone = udp_relay.relay, udp_relay.drone
    drone.send(drone.enc.heartbeat())  # the relay learns the vehicle address
    gcs, other = connect(relay, udp_relay.ports[0], udp_relay.ports[1])
    assert wait_until(lambda: other.count("HEARTBEAT", type=ml.MAV_TYPE_QUADROTOR) >= 0)

    for _ in range(3):
        gcs.send(Encoder(255, 190).gcs_heartbeat())
    received = drone.receive(0.5)
    assert any(m.get_type() == "HEARTBEAT" and m.type == ml.MAV_TYPE_GCS for m in received)
    assert other.count("HEARTBEAT", type=ml.MAV_TYPE_GCS) == 0

    drone.send(drone.enc.heartbeat())
    assert wait_until(lambda: other.count("HEARTBEAT", type=ml.MAV_TYPE_QUADROTOR) >= 1)
    assert wait_until(lambda: gcs.count("HEARTBEAT", type=ml.MAV_TYPE_QUADROTOR) >= 1)

    relay.forward_gcs_heartbeats = True
    gcs.send(Encoder(255, 190).gcs_heartbeat())
    assert wait_until(lambda: other.count("HEARTBEAT", type=ml.MAV_TYPE_GCS) == 1)
    gcs.close()
    other.close()


def test_targeted_messages_only_reach_their_target(udp_relay):
    relay, drone = udp_relay.relay, udp_relay.drone
    drone.send(drone.enc.heartbeat())
    a, b = connect(relay, udp_relay.ports[0], udp_relay.ports[1])
    a.send(Encoder(250, 190).gcs_heartbeat())
    b.send(Encoder(251, 190).gcs_heartbeat())
    assert wait_until(lambda: len(drone.receive(0.1)) >= 0 and all(
        any(sysid in e.sysids for e in relay._endpoints) for sysid in (250, 251)
    ))

    drone.send(Encoder(1).pack(ml.MAVLink_command_ack_message(ml.MAV_CMD_REQUEST_MESSAGE, 0, 0, 0, 250, 190)))
    drone.send(drone.enc.system_time())  # broadcast
    assert wait_until(lambda: a.count("COMMAND_ACK") == 1 and b.count("SYSTEM_TIME") == 1)
    time.sleep(0.2)
    assert b.count("COMMAND_ACK") == 0
    a.close()
    b.close()


def test_client_is_told_when_its_system_id_is_taken(udp_relay):
    relay = udp_relay.relay
    first, second = connect(relay, udp_relay.ports[0], udp_relay.ports[1])
    first.send(Encoder(17, 0).gcs_heartbeat())
    assert wait_until(lambda: any(17 in e.sysids for e in relay._endpoints))
    second.send(Encoder(17, 0).gcs_heartbeat())

    def notices(client):
        return [str(m.text) for m in list(client.msgs) if m.get_type() == "STATUSTEXT"]

    assert wait_until(lambda: f"{zenrelay.SYSID_IN_USE_PREFIX}17" in notices(second))
    time.sleep(0.2)
    assert notices(first) == []
    first.close()
    second.close()


def test_slow_client_does_not_affect_others(udp_relay, monkeypatch):
    monkeypatch.setattr(zenrelay, "MAX_CLIENT_BACKLOG", 16 * 1024)
    relay, drone = udp_relay.relay, udp_relay.drone
    fast, slow = connect(relay, udp_relay.ports[0], (udp_relay.ports[1], False, 4096))
    slow_name = f"127.0.0.1:{slow.sock.getsockname()[1]} "
    for endpoint in relay._endpoints:  # small kernel buffers, so the relay backlog fills up
        if slow_name in endpoint.name:
            endpoint.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
    sent = 0
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        drone.send(b"".join(drone.enc.system_time() for _ in range(30)))
        sent += 30
        time.sleep(0.01)
    assert wait_until(lambda: fast.count("SYSTEM_TIME") >= sent)
    assert statistics.median(fast.latencies_ms) < 20
    assert any(v["dropped"] > 0 for k, v in relay.stats().items() if k.startswith("tcp client"))

    slow.resume()
    time.sleep(0.5)
    assert slow.bad == 0  # only whole messages are dropped, the stream stays aligned
    assert slow.count("SYSTEM_TIME") > 0
    fast.close()
    slow.close()


def test_vehicle_tcp_link_reconnects():
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    server.settimeout(10)
    port = free_port()
    relay = MavRelay(
        mavutil.mavlink_connection(f"tcp:127.0.0.1:{server.getsockname()[1]}", autoreconnect=True),
        tcp_ports=[port], bind="127.0.0.1",
    )
    try:
        first, _ = server.accept()
        (client,) = connect(relay, port)
        enc = Encoder(1)
        first.sendall(enc.system_time())
        assert wait_until(lambda: client.count("SYSTEM_TIME") == 1)

        first.close()
        second, _ = server.accept()
        for _ in range(5):
            second.sendall(enc.system_time())
        assert wait_until(lambda: client.count("SYSTEM_TIME") == 6, timeout=5)
        second.close()
        client.close()
    finally:
        relay.close()
        server.close()
