"""
MAVLink relay used by Zenmav in GCS mode.

One vehicle link (serial, UDP or TCP) is shared between:

- in-process clients, such as the Zenmav instance that owns the relay (no socket involved),
- TCP clients (Mission Planner, QGroundControl, other Zenmav scripts), several per port.

Messages are routed the way ArduPilot routes them: the relay learns on which link each
(sysid, compid) was seen, sends a targeted message only to the link(s) of its target and
broadcasts everything else.

GCS-like heartbeats (GCS, ADS-B, onboard controller) always reach the vehicle, which needs
them for its GCS failsafe. Between clients they are only forwarded when
``forward_gcs_heartbeats`` is True: old Zenmav versions locked onto the first heartbeat they
received and mistook another ground station for the vehicle.
"""

import os
import select
import socket
import threading
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

if mavutil.mavlink.WIRE_PROTOCOL_VERSION != "2.0":
    # pymavlink was imported before Zenmav: switch the dialect to MAVLink 2
    mavutil.set_dialect(mavutil.current_dialect)

MAX_CLIENT_BACKLOG = 512 * 1024  # bytes queued for a slow TCP client before its messages are dropped
MAX_INTERNAL_BACKLOG = 8 * 1024 * 1024  # bytes queued for an in-process client that stopped reading
VEHICLE_WRITE_TIMEOUT = 0.5  # seconds a write to a congested vehicle TCP link may block
SYSID_IN_USE_PREFIX = "zenmav-relay:sysid-in-use:"  # STATUSTEXT sent to a client whose system ID is taken


def _mav_types(*names):
    return {getattr(mavutil.mavlink, n) for n in names if hasattr(mavutil.mavlink, n)}


GCS_LIKE_TYPES = _mav_types("MAV_TYPE_GCS", "MAV_TYPE_ADSB", "MAV_TYPE_ONBOARD_CONTROLLER")
NON_VEHICLE_TYPES = GCS_LIKE_TYPES | _mav_types(
    "MAV_TYPE_GIMBAL", "MAV_TYPE_CAMERA", "MAV_TYPE_FLARM", "MAV_TYPE_SERVO", "MAV_TYPE_ODID",
    "MAV_TYPE_BATTERY", "MAV_TYPE_CHARGING_STATION", "MAV_TYPE_LOG", "MAV_TYPE_OSD", "MAV_TYPE_IMU",
    "MAV_TYPE_GPS", "MAV_TYPE_WINCH", "MAV_TYPE_GENERIC", "MAV_TYPE_ANTENNA_TRACKER",
)


def is_gcs_like_heartbeat(msg):
    """True for heartbeats of ground stations, ADS-B receivers and onboard controllers."""
    return msg.get_type() == "HEARTBEAT" and msg.type in GCS_LIKE_TYPES


def is_vehicle_heartbeat(msg):
    """True for the heartbeat of an autopilot (not a GCS, gimbal, camera, companion computer...)."""
    return (
        msg.get_type() == "HEARTBEAT"
        and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        and msg.type not in NON_VEHICLE_TYPES
        and msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_GIMBAL
    )


def target_of(msg):
    """(target_system, target_component) of a message, (0, 0) when it is a broadcast."""
    target_system = getattr(msg, "target_system", None)
    if target_system is None and msg.get_type() == "MANUAL_CONTROL":
        target_system = msg.target
    if not target_system:
        return 0, 0
    return target_system, getattr(msg, "target_component", 0) or 0


class _RateLimitedLog:
    def __init__(self, period=10.0):
        self.period = period
        self._last = {}

    def __call__(self, key, text):
        now = time.monotonic()
        if now - self._last.get(key, -1e9) >= self.period:
            self._last[key] = now
            print(f"[relay] {text}")


class _Endpoint:
    """One side of the relay: the vehicle link, a TCP client or an in-process client."""

    is_vehicle = False

    def __init__(self, name):
        self.name = name
        self.mav = mavutil.mavlink.MAVLink(None)
        self.mav.robust_parsing = True
        self.parse_lock = threading.Lock()
        self.routes = set()  # (sysid, compid) seen on this link
        self.sysids = set()
        self.rx = 0
        self.tx = 0
        self.dropped = 0
        self.closed = False

    def parse(self, data):
        with self.parse_lock:
            msgs = self.mav.parse_buffer(data) or []
        return [m for m in msgs if m.get_type() != "BAD_DATA"]

    def send(self, data, count):
        raise NotImplementedError


class _VehicleEndpoint(_Endpoint):
    is_vehicle = True

    def __init__(self, conn):
        super().__init__(f"vehicle {conn.address}")
        self.conn = conn
        self.write_lock = threading.Lock()

    def send(self, data, count):
        with self.write_lock:
            if isinstance(self.conn, mavutil.mavtcp):
                sent = _send_all(self.conn.port, data, VEHICLE_WRITE_TIMEOUT)
            else:
                sent = self.conn.write(data) != -1
        if sent:
            self.tx += count
        else:
            self.dropped += count


class _TcpClient(_Endpoint):
    def __init__(self, relay, sock, addr, port):
        super().__init__(f"tcp client {addr[0]}:{addr[1]} on port {port}")
        self.relay = relay
        self.sock = sock
        self.out = bytearray()
        self.lock = threading.Lock()

    def send(self, data, count):
        with self.lock:
            if self.closed:
                return
            if len(self.out) + len(data) > MAX_CLIENT_BACKLOG:
                self.dropped += count  # whole messages only, the stream stays aligned
                return
            self.out += data
            self.tx += count
            self.flush_locked()
            pending = bool(self.out)
        if pending:
            self.relay._wake()

    def flush_locked(self):
        if not self.out or self.closed:
            return
        try:
            sent = self.sock.send(self.out)
            del self.out[:sent]
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            self.closed = True


class _InternalEndpoint(_Endpoint):
    def __init__(self, link):
        super().__init__(f"in-process client {link.source_system}")
        self.link = link

    def send(self, data, count):
        if self.link._deliver(data):
            self.tx += count
        else:
            self.dropped += count


class RelayLink(mavutil.mavfile):
    """pymavlink connection object backed by the relay: usable like any mavlink_connection()."""

    def __init__(self, relay, source_system, source_component=0):
        self._relay = relay
        self._inbuf = bytearray()
        self._inlock = threading.Lock()
        self._event = threading.Event()
        mavutil.mavfile.__init__(
            self, None, "zenmav-relay", source_system=source_system, source_component=source_component
        )
        self._endpoint = _InternalEndpoint(self)

    def recv(self, n=None):
        with self._inlock:
            if not self._inbuf:
                self._event.clear()
                return b""
            data = bytes(self._inbuf)
            self._inbuf.clear()
            return data

    def _deliver(self, data):
        with self._inlock:
            if len(self._inbuf) + len(data) > MAX_INTERNAL_BACKLOG:
                return False
            self._inbuf += data
            self._event.set()
        return True

    def write(self, buf):
        if self._endpoint.closed:
            return -1
        self._relay._handle(self._endpoint, bytes(buf))
        return len(buf)

    def select(self, timeout):
        return self._event.wait(timeout)

    def close(self, n=None):
        self._relay._remove_endpoint(self._endpoint)


def _send_all(sock, data, timeout):
    if sock is None:
        return False
    view = memoryview(data)
    deadline = time.monotonic() + timeout
    while view:
        try:
            sent = sock.send(view)
            view = view[sent:]
        except (BlockingIOError, InterruptedError):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            select.select([], [sock], [], remaining)
        except OSError:
            return False
    return True


class MavRelay:
    """Shares one vehicle link between in-process clients and TCP clients.

    Args:
        vehicle_conn: pymavlink connection to the vehicle (serial, UDP or TCP).
        tcp_ports: ports of the TCP servers. Each accepts several clients.
        bind: address the TCP servers listen on.
        forward_gcs_heartbeats: forward GCS-like heartbeats between clients (they always reach the vehicle).
    """

    def __init__(self, vehicle_conn, tcp_ports=(14550, 14551), bind="0.0.0.0", forward_gcs_heartbeats=False):
        self.forward_gcs_heartbeats = forward_gcs_heartbeats
        self.vehicle = _VehicleEndpoint(vehicle_conn)
        self._endpoints = [self.vehicle]
        self._endpoints_lock = threading.Lock()
        self._stop = threading.Event()
        self._log = _RateLimitedLog()
        self._warned_duplicates = set()
        self._notice_mav = mavutil.mavlink.MAVLink(None, srcSystem=0, srcComponent=0)

        self._listeners = {}
        try:
            for port in tcp_ports:
                listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((bind, int(port)))
                listener.listen(8)
                listener.setblocking(False)
                self._listeners[listener] = int(port)
        except OSError as e:
            for listener in self._listeners:
                listener.close()
            raise OSError(f"Zenmav relay cannot listen on TCP port {port}: {e}") from e

        self._wake_r, self._wake_w = socket.socketpair()
        self._wake_r.setblocking(False)
        self._wake_w.setblocking(False)

        self._threads = [
            threading.Thread(target=self._supervise, args=(self._vehicle_loop,), name="zenmav-relay-vehicle", daemon=True),
            threading.Thread(target=self._supervise, args=(self._server_loop,), name="zenmav-relay-tcp", daemon=True),
        ]
        for t in self._threads:
            t.start()
        print(f"Relay started: vehicle {vehicle_conn.address}, TCP ports {', '.join(map(str, tcp_ports))}")

    # ------------------------------------------------------------------ public API

    def add_internal_link(self, source_system, source_component=0):
        """Returns a pymavlink connection object for an in-process client."""
        link = RelayLink(self, source_system, source_component)
        with self._endpoints_lock:
            self._endpoints.append(link._endpoint)
        return link

    def stats(self):
        """Per link counters: received, sent and dropped messages."""
        with self._endpoints_lock:
            endpoints = list(self._endpoints)
        return {e.name: {"rx": e.rx, "tx": e.tx, "dropped": e.dropped} for e in endpoints}

    def close(self):
        self._stop.set()
        self._wake()
        for t in self._threads:
            if t is not threading.current_thread():
                t.join(timeout=2.0)
        with self._endpoints_lock:
            endpoints, self._endpoints = list(self._endpoints), []
        for e in endpoints:
            e.closed = True
            if isinstance(e, _TcpClient):
                e.sock.close()
        for listener in self._listeners:
            listener.close()
        self._wake_r.close()
        self._wake_w.close()
        try:
            self.vehicle.conn.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ routing

    def _handle(self, src, data):
        msgs = src.parse(data)
        if not msgs:
            return
        src.rx += len(msgs)
        with self._endpoints_lock:
            endpoints = [e for e in self._endpoints if e is not src and not e.closed]
            taken = [sysid for sysid in (self._learn(src, m) for m in msgs) if sysid is not None]
            batches = {}
            for m in msgs:
                for dst in self._destinations(src, m, endpoints):
                    batches.setdefault(dst, []).append(m.get_msgbuf())
        for dst, bufs in batches.items():
            dst.send(b"".join(bufs), len(bufs))
        for sysid in taken:
            # Tells this client only, so that Zenmav picks another system ID (see Zenmav.connect)
            text = f"{SYSID_IN_USE_PREFIX}{sysid}".encode()
            notice = mavutil.mavlink.MAVLink_statustext_message(mavutil.mavlink.MAV_SEVERITY_NOTICE, text)
            src.send(bytes(notice.pack(self._notice_mav)), 1)

    def _learn(self, src, m):
        """Records the route of the message source. Returns its sysid if another link already uses it."""
        sysid, compid = m.get_srcSystem(), m.get_srcComponent()
        if sysid == 0 or (sysid, compid) in src.routes:
            return None
        src.routes.add((sysid, compid))
        src.sysids.add(sysid)
        if src.is_vehicle or sysid == 255:
            return None  # 255 is shared by every Mission Planner / QGroundControl, Zenmav never uses it
        # Same sysid with another compid is legitimate (e.g. a companion computer on the vehicle system)
        other = next((e for e in self._endpoints if e is not src and not e.closed and (sysid, compid) in e.routes), None)
        if other is None:
            return None
        key = (sysid, src.name)
        if key not in self._warned_duplicates:
            self._warned_duplicates.add(key)
            print(f"[relay] WARNING : MAVLink system ID {sysid} of {src.name} is already used by {other.name}")
        return sysid

    def _destinations(self, src, m, endpoints):
        if is_gcs_like_heartbeat(m) and not self.forward_gcs_heartbeats:
            # The vehicle needs GCS heartbeats (GCS failsafe), clients must not mistake them for the vehicle
            return [] if src.is_vehicle else [e for e in endpoints if e.is_vehicle]
        target_system, target_component = target_of(m)
        if target_system == 0:
            return endpoints
        if target_component:
            routed = [e for e in endpoints if (target_system, target_component) in e.routes]
            if routed:
                return routed
        routed = [e for e in endpoints if target_system in e.sysids]
        # Unknown target (route not learned yet): broadcast rather than drop
        return routed or endpoints

    def _remove_endpoint(self, endpoint):
        endpoint.closed = True
        with self._endpoints_lock:
            if endpoint in self._endpoints:
                self._endpoints.remove(endpoint)

    # ------------------------------------------------------------------ threads

    def _supervise(self, loop):
        while not self._stop.is_set():
            try:
                loop()
            except Exception as e:  # never let the relay die silently
                self._log(f"crash:{loop.__name__}", f"ERROR in {loop.__name__}: {e!r}, restarting")
                time.sleep(0.2)

    def _wake(self):
        try:
            self._wake_w.send(b"\0")
        except OSError:
            pass

    def _vehicle_loop(self):
        conn = self.vehicle.conn
        while not self._stop.is_set():
            try:
                data = self._read_vehicle(conn)
            except Exception as e:
                self._log(f"vehicle:{type(e).__name__}", f"vehicle link error: {e!r}")
                self._recover_vehicle(conn)
                continue
            if data:
                self._handle(self.vehicle, data)

    def _read_vehicle(self, conn):
        if isinstance(conn, mavutil.mavtcp):
            if conn.port is None:
                raise ConnectionError("vehicle TCP link is closed")
            if not select.select([conn.port], [], [], 0.1)[0]:
                return b""
            data = conn.port.recv(65536)
            if not data:
                raise ConnectionError("vehicle TCP link closed by peer")
            return data
        if isinstance(conn, mavutil.mavudp):
            if not select.select([conn.port], [], [], 0.1)[0]:
                return b""
            chunks = []
            for _ in range(256):  # drain every waiting datagram
                try:
                    chunk = conn.recv()
                except OSError as e:  # e.g. Windows ConnectionResetError 10054 after an ICMP port unreachable
                    self._log(f"udp:{type(e).__name__}", f"vehicle UDP link: {e!r} (ignored)")
                    continue
                if not chunk:
                    break
                chunks.append(chunk)
            return b"".join(chunks)
        if conn.fd is not None:
            if not select.select([conn.fd], [], [], 0.1)[0]:
                return b""
            return conn.recv(65536)
        data = conn.recv(65536)  # serial port without file descriptor (Windows)
        if not data:
            time.sleep(0.002)
        return data

    def _recover_vehicle(self, conn):
        if self._stop.wait(1.0):
            return
        try:
            if isinstance(conn, mavutil.mavtcp):
                if conn.port is not None:
                    conn.port.close()
                    conn.port = None
                conn.do_connect()
                print("[relay] vehicle TCP link restored")
            elif isinstance(conn, mavutil.mavserial):
                if conn.reset():
                    print("[relay] vehicle serial link restored")
        except Exception:
            pass

    def _server_loop(self):
        while not self._stop.is_set():
            with self._endpoints_lock:
                clients = [e for e in self._endpoints if isinstance(e, _TcpClient)]
            for c in [c for c in clients if c.closed]:
                self._drop_client(c)
            clients = [c for c in clients if not c.closed]
            readers = [self._wake_r, *self._listeners, *(c.sock for c in clients)]
            writers = [c.sock for c in clients if c.out]
            try:
                readable, writable, _ = select.select(readers, writers, [], 0.5)
            except (OSError, ValueError):
                time.sleep(0.01)
                continue
            by_sock = {c.sock: c for c in clients}
            for s in writable:
                c = by_sock.get(s)
                if c is not None:
                    with c.lock:
                        c.flush_locked()
            for s in readable:
                if s is self._wake_r:
                    try:
                        while self._wake_r.recv(4096):
                            pass
                    except OSError:
                        pass
                elif s in self._listeners:
                    self._accept(s)
                elif s in by_sock:
                    self._read_client(by_sock[s])

    def _accept(self, listener):
        while True:
            try:
                sock, addr = listener.accept()
            except OSError:
                return
            sock.setblocking(False)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client = _TcpClient(self, sock, addr, self._listeners[listener])
            with self._endpoints_lock:
                self._endpoints.append(client)
            print(f"[relay] {client.name} connected")

    def _read_client(self, client):
        chunks = []
        while True:
            try:
                chunk = client.sock.recv(65536)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                client.closed = True
                break
            if not chunk:
                client.closed = True
                break
            chunks.append(chunk)
        if chunks:
            self._handle(client, b"".join(chunks))
        if client.closed:
            self._drop_client(client)

    def _drop_client(self, client):
        self._remove_endpoint(client)
        try:
            client.sock.close()
        except OSError:
            pass
        print(f"[relay] {client.name} disconnected")
