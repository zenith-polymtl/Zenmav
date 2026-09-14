"""
Connection helpers for Zenmav.

``SharedReader`` makes a pymavlink connection safe to read from several threads (main script,
fence monitor, user callbacks). A single thread reads and parses the link; every thread calling
``connection.recv_match()`` gets its own copy of the message stream, so no thread steals the
messages another one is waiting for.
"""

import threading
import time
from collections import deque

from .zenrelay import mavutil

HISTORY_S = 0.5  # a thread reading for the first time also gets the messages of the last HISTORY_S seconds
QUEUE_LEN = 5000  # messages kept per thread (a full parameter download fits)
SYSID_RANGE = range(200, 255)  # automatic system IDs picked when the default one is already in use


class SharedReader:
    def __init__(self, conn):
        self.conn = conn
        self.closed = False
        self._cond = threading.Condition()
        self._queues = {}
        self._history = deque(maxlen=2000)
        self._stop = threading.Event()
        self._last_log = {}
        self._orig_close = conn.close
        conn.recv_match = self.recv_match
        conn.close = self.close
        self._queue()  # the thread that opened the connection reads from now on
        self._thread = threading.Thread(target=self._run, name="zenmav-reader", daemon=True)
        self._thread.start()

    def _queue(self):
        thread = threading.current_thread()
        with self._cond:
            q = self._queues.get(thread)
            if q is None:
                now = time.monotonic()
                q = deque((m for t, m in self._history if now - t <= HISTORY_S), maxlen=QUEUE_LEN)
                self._queues[thread] = q
            return q

    def _log(self, key, text, period=10.0):
        now = time.monotonic()
        if now - self._last_log.get(key, -1e9) >= period:
            self._last_log[key] = now
            print(text)

    def _run(self):
        conn = self.conn
        # A serial port without file descriptor cannot wait for data: poll it
        poll = 0.002 if conn.fd is None and not hasattr(conn, "_relay") else 0.1
        last_prune = time.monotonic()
        while not self._stop.is_set():
            try:
                m = conn.recv_msg()
            except Exception as e:
                self._log(type(e).__name__, f"WARNING : MAVLink link error: {e!r}")
                self._stop.wait(0.05)
                continue
            if m is None:
                try:
                    conn.select(poll)
                except Exception:
                    time.sleep(poll)
                continue
            if (
                m.get_type() == "HEARTBEAT"
                and m.get_srcSystem() == conn.mav.srcSystem
                and m.get_srcComponent() == conn.mav.srcComponent
            ):
                self._log("sysid", f"WARNING : another MAVLink system also uses system ID {conn.mav.srcSystem}")
            now = time.monotonic()
            with self._cond:
                self._history.append((now, m))
                if now - last_prune > 1.0:
                    last_prune = now
                    for t in [t for t in self._queues if not t.is_alive()]:
                        del self._queues[t]
                for q in self._queues.values():
                    q.append(m)
                self._cond.notify_all()

    def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
        """Same contract as pymavlink's recv_match, on the calling thread's own message stream."""
        if type is not None and not isinstance(type, (list, set, tuple)):
            type = [type]
        q = self._queue()
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            with self._cond:
                while not q:
                    if not blocking or self._stop.is_set():
                        return None
                    if deadline is None:
                        self._cond.wait(0.5)
                        continue
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self._cond.wait(remaining)
                m = q.popleft()
            if type is not None and m.get_type() not in type:
                if deadline is not None and blocking and time.monotonic() > deadline:
                    return None
                continue
            if condition is not None and not mavutil.evaluate_condition(condition, self.conn.messages):
                continue
            return m

    def flush(self):
        """Discards the messages already received by the calling thread.

        Call it before sending a request, so that the answer read afterwards is not an old
        copy that was waiting in this thread's stream (PARAM_VALUE, HEARTBEAT...).
        """
        q = self._queue()
        with self._cond:
            q.clear()

    def close(self, *args):
        if self.closed:
            return
        self.closed = True
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._orig_close()
        except Exception:
            pass


def lock_sends(conn):
    """Serialises packet sending (sequence number and write) between threads."""
    lock = threading.RLock()
    send = conn.mav.send

    def locked_send(*args, **kwargs):
        with lock:
            return send(*args, **kwargs)

    conn.mav.send = locked_send


def default_system_id(connection_string):
    """System ID derived from the connection port, as in previous Zenmav versions (None for serial links)."""
    try:
        return int(connection_string.rsplit(":", 1)[1]) % 255 or None
    except (IndexError, ValueError):
        return None


def free_system_id(used, preferred=None, salt=0):
    """Returns ``preferred`` when it is free, otherwise a free ID in SYSID_RANGE."""
    if preferred and preferred not in used and preferred != 255:
        return preferred
    candidates = [i for i in SYSID_RANGE if i not in used]
    if not candidates:
        raise RuntimeError("No free MAVLink system ID left")
    return candidates[salt % len(candidates)]
