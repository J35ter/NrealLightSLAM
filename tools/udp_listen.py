#!/usr/bin/env python3
"""UDP listener that mimics Opentrack's "UDP over network" tracker reader.

Parses incoming datagrams the way opentrack's tracker-udp does:
readDatagram(buf, 48) — first 6×f64, native (little) endian, rotation in
degrees, rejecting NaN/Inf. Prints each packet's parsed values plus a total.

Use it to isolate whether the tracker's UDP output works on its own
(before involving Opentrack):

  terminal 1:  python3 tools/udp_listen.py [seconds]
  terminal 2:  ./start-tracker.sh            (or with glasses: default run)

If packets print here, the tracker side is fine and the problem is
Opentrack's config (tracker enabled? port? sandbox?).

Usage:  python3 tools/udp_listen.py [timeout_seconds] [--extended]
"""
import math
import socket
import struct
import sys

HOST = "127.0.0.1"
PORT = 4242

timeout = 10.0
extended = False
for arg in sys.argv[1:]:
    if arg == "--extended":
        extended = True
    else:
        try:
            timeout = float(arg)
        except ValueError:
            print(f"ignoring unknown arg: {arg}", file=sys.stderr)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
sock.settimeout(timeout)
print(f"listening on {HOST}:{PORT} for {timeout}s (extended={'yes' if extended else 'no'}) ...",
      flush=True)

count = 0
try:
    while True:
        data, _ = sock.recvfrom(1024)
        n = len(data) // 8
        if n < 6:
            print(f"packet {count + 1}: too small ({len(data)}B)", flush=True)
            continue
        vals = struct.unpack(f"<{n}d", data[: n * 8])
        if any(math.isnan(v) or math.isinf(v) for v in vals[:6]):
            print(f"packet {count + 1}: NaN/Inf — opentrack would REJECT", flush=True)
            continue
        is_ext = n >= 10
        if is_ext and not extended:
            print(f"packet {count + 1}: extended ({len(data)}B) — add --extended to count these",
                  flush=True)
            continue
        count += 1
        extra = f"  [ext fields: {vals[6]:.1f},{vals[7]:.1f},{vals[8]:.1f},{vals[9]:.1f}]" if is_ext else ""
        if count <= 5 or count % 50 == 0:
            print(f"packet {count}: {len(data)}B  TX,TY,TZ={vals[0]:.1f},{vals[1]:.1f},{vals[2]:.1f}"
                  f"  yaw={vals[3]:7.2f}  pitch={vals[4]:7.2f}  roll={vals[5]:7.2f}{extra}",
                  flush=True)
except socket.timeout:
    pass
print(f"TOTAL packets accepted: {count}", flush=True)
