"""Standalone relay forwarding GCS heartbeats between clients, used to stress Zenmav's vehicle detection.

usage: python relay_proc.py <src dir> <vehicle connection> <port,port,...>
"""
import sys
import time

sys.path.insert(0, sys.argv[1])
from zenmav.zenrelay import MavRelay, mavutil  # noqa: E402

relay = MavRelay(
    mavutil.mavlink_connection(sys.argv[2], autoreconnect=True),
    tcp_ports=[int(p) for p in sys.argv[3].split(",")],
    bind="127.0.0.1",
    forward_gcs_heartbeats=True,
)
while True:
    time.sleep(1)
