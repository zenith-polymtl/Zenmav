"""Fake MAVLink systems for the relay tests (no simulator needed)."""

import socket
import threading
import time

from zenmav.zenrelay import mavutil

ml = mavutil.mavlink


def free_port(kind=socket.SOCK_STREAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_until(predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class Encoder:
    """Packs messages as a given MAVLink system."""

    def __init__(self, sysid, compid=1):
        self.mav = ml.MAVLink(None, srcSystem=sysid, srcComponent=compid)

    def pack(self, msg):
        return bytes(msg.pack(self.mav))

    def heartbeat(self, mav_type=ml.MAV_TYPE_QUADROTOR, autopilot=ml.MAV_AUTOPILOT_ARDUPILOTMEGA):
        return self.pack(ml.MAVLink_heartbeat_message(mav_type, autopilot, 0, 0, 0, 3))

    def gcs_heartbeat(self):
        return self.heartbeat(ml.MAV_TYPE_GCS, ml.MAV_AUTOPILOT_INVALID)

    def system_time(self):
        return self.pack(ml.MAVLink_system_time_message(time.time_ns() // 1000, 0))


class TcpClient:
    """TCP client of the relay recording every message it receives."""

    def __init__(self, port, read=True, rcvbuf=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if rcvbuf:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        self.sock.connect(("127.0.0.1", port))
        self.sock.settimeout(0.1)
        self.mav = ml.MAVLink(None)
        self.msgs = []
        self.latencies_ms = []
        self.bad = 0
        self._stop = threading.Event()
        self._paused = threading.Event()
        if not read:
            self._paused.set()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self._stop.is_set():
            if self._paused.is_set():
                time.sleep(0.01)
                continue
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            now = time.time_ns() // 1000
            for m in self.mav.parse_buffer(data) or []:
                if m.get_type() == "BAD_DATA":
                    self.bad += 1
                    continue
                self.msgs.append(m)
                if m.get_type() == "SYSTEM_TIME":
                    self.latencies_ms.append((now - m.time_unix_usec) / 1000.0)

    def count(self, msg_type, **fields):
        return sum(
            1 for m in list(self.msgs)
            if m.get_type() == msg_type and all(getattr(m, k) == v for k, v in fields.items())
        )

    def send(self, data):
        self.sock.sendall(data)

    def resume(self):
        self._paused.clear()

    def close(self):
        self._stop.set()
        self.sock.close()
        self.thread.join(1.0)


class UdpDrone:
    """Fake autopilot talking UDP to the relay."""

    def __init__(self, relay_port, sysid=1):
        self.addr = ("127.0.0.1", relay_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.enc = Encoder(sysid)
        self.mav = ml.MAVLink(None)

    def send(self, data):
        self.sock.sendto(data, self.addr)

    def receive(self, duration=0.5):
        msgs = []
        end = time.monotonic() + duration
        while time.monotonic() < end:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                continue
            msgs += [m for m in (self.mav.parse_buffer(data) or []) if m.get_type() != "BAD_DATA"]
        return msgs

    def close(self):
        self.sock.close()


def tcp_clients(relay):
    return sum(1 for name in relay.stats() if name.startswith("tcp client"))
