#!/usr/bin/env python3
"""Analyze a --log-pose JSONL file (spec §4.4 pose format).

Each line: {"t": <s>, "yaw": <deg>, "pitch": <deg>, "roll": <deg>,
            "x": <m>, "y": <m>, "z": <m>}
Position is in meters; ypr in wire units (deg unless --units rad was used).

Prints the drift summary used by test-drift.sh / the M9 acceptance bar:
  path length, endpoint displacement, per-step deltas (median/max),
  per-step dz bias (mean + t-stat), yaw wander.

Usage:
    python3 tools/analyze_pose.py POSES.jsonl [--units deg|rad]
"""
import json
import math
import statistics
import sys


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    path = args[0]
    units = "deg"
    if "--units" in args:
        i = args.index("--units")
        units = args[i + 1] if i + 1 < len(args) else "deg"
    yaw_scale = 180.0 / math.pi if units == "rad" else 1.0

    poses = []  # (t, yaw, x, y, z)
    n_skipped = 0
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
                poses.append((o["t"], o["yaw"] * yaw_scale, o["x"], o["y"], o["z"]))
            except (json.JSONDecodeError, KeyError, TypeError):
                n_skipped += 1

    if len(poses) < 2:
        print(f"error: only {len(poses)} valid pose(s) in {path} — is the "
              "log empty (SIGTERM killed it before flush?) or did every "
              "frame fail the inlier gate?", file=sys.stderr)
        return 1

    t0 = poses[0][0]
    dur = poses[-1][0] - t0
    pos = [(p[2], p[3], p[4]) for p in poses]
    yaw = [p[1] for p in poses]

    def dist(a, b):
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

    steps = [dist(pos[i], pos[i + 1]) for i in range(len(pos) - 1)]
    total = sum(steps)
    endpoint = dist(pos[0], pos[-1])
    dz = [pos[i + 1][2] - pos[i][2] for i in range(len(pos) - 1)]

    # Per-step dz bias: mean and t-stat (|t| >= 2 => systematic bias).
    dz_mean = statistics.mean(dz)
    if len(dz) > 1 and statistics.pstdev(dz) > 0:
        t_stat = dz_mean / (statistics.pstdev(dz) / math.sqrt(len(dz)))
    else:
        t_stat = 0.0

    rate = len(poses) / max(dur, 1e-9)
    print(f"file:        {path}")
    print(f"poses:       {len(poses)}  ({n_skipped} unparsed lines skipped)")
    print(f"duration:    {dur:.1f} s")
    print(f"path:        {total:.3f} m")
    print(f"endpoint:    {endpoint:.3f} m")
    print(f"per-step:    median {statistics.median(steps):.4f} m, "
          f"max {max(steps):.4f} m")
    print(f"dz bias:     mean {dz_mean:+.4f} m/step, t={t_stat:+.1f} "
          f"({'bias' if abs(t_stat) >= 2 else 'no bias'})")
    print(f"yaw:         range {max(yaw) - min(yaw):.1f} deg, "
          f"drift {yaw[-1] - yaw[0]:+.1f} deg")

    print("---")
    # M9 acceptance bar (measured at 20 s on a still headset). The VO-only
    # path reference assumes the ~1-2 Hz VO pose rate of test-drift.sh; a
    # fused log runs at IMU rate (~1 kHz), where the accumulated path over
    # ~20k sub-mm steps is large by construction and not comparable.
    if rate < 50:
        print("still-headset reference (M9, 20 s): path < 1.46 m, "
              "endpoint < 0.21 m, median < 0.042 m, |t| < 2")
        ok = (
            total < 1.46 * max(dur / 20.0, 0.25)
            and endpoint < 0.21 * max(dur / 20.0, 0.25)
            and abs(t_stat) < 2
        )
    else:
        # Fused (IMU-rate) reference: endpoint drift, per-step median, and
        # yaw stability dominate.
        print("fused still-headset reference (P2b, 20 s): endpoint < 0.21 m, "
              "per-step median < 0.042 m, |t| < 2, yaw drift < 5°")
        ok = (
            endpoint < 0.21 * max(dur / 20.0, 0.25)
            and statistics.median(steps) < 0.042
            and abs(t_stat) < 2
            and abs(yaw[-1] - yaw[0]) < 5.0
        )
    print("RESULT:", "PASS" if ok else "FAIL (drift — see D.9 / P2b IMU fusion)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
