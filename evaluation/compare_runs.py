#!/usr/bin/env python3
"""Compares two (or more) runs of the same protocol, segment by segment.

    ./compare_runs.py sim_A sim_B [sim_C ...]

Only runs that declare the SAME protocol are compared: otherwise it stops, because the numbers would
not be comparable. It prints the deltas with respect to the first run, and at the end the list of
regressions beyond threshold -- the quickest way to see whether a change broke something that
worked before.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

# metric -> (label, better direction, threshold beyond which a change is reported)
FIELDS = [
    ("gain_vx",   "gain vx",  +1, 0.10),
    ("gain_vy",   "gain vy",  +1, 0.10),
    ("gain_wz",   "gain wz",  +1, 0.10),
    ("h_ctrl",    "h ctrl",    0, 0.005),   # height actually commanded: must match between runs
    ("h_err",     "h err",     0, 0.008),   # true height tracking error of the robot
    ("sway",      "sway",     -1, 0.010),
    ("roll_sd",   "rollSD",   -1, 0.100),
    ("gyro",      "gyro",     -1, 0.002),
    ("dact",      "Δaction",  -1, 0.005),
    ("tau_ankle", "tau ankle", 0, 3.0),
]


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    runs = [Path(a).resolve() for a in sys.argv[1:]]
    data, names = [], []
    proto = None
    for r in runs:
        met = json.loads((r / "metrics.json").read_text())
        meta = json.loads((r / "run_meta.json").read_text()) if (r / "run_meta.json").exists() else {}
        if proto is None:
            proto = meta.get("protocol")
        elif meta.get("protocol") != proto:
            print(f"ERROR: {r.name} uses protocol '{meta.get('protocol')}', expected '{proto}'. "
                  f"The runs are not comparable.")
            return 1
        data.append(met)
        names.append(meta.get("label") or meta.get("policy") or r.name)

    print(f"protocol: {proto}")
    for i, n in enumerate(names):
        print(f"  [{i}] {n}   (rtf {data[i]['rtf']:.4f}, {len(data[i]['checks'])} failed checks)")
    print()

    base = {(s["t0"]): s for s in data[0]["segments"]}
    regressions = []
    hdr = f"{'t0':>6}{'h':>6}{'stick':>16}  " + "  ".join(f"{lab:>10}" for _, lab, _, _ in FIELDS)
    print(hdr); print("-" * len(hdr))
    for s0 in data[0]["segments"]:
        st = ",".join(f"{v:g}" for v in s0["stick"])
        print(f"{s0['t0']:6.0f}{s0['h_cmd']:6.2f}{st:>16}  " +
              "  ".join("        —" if s0[k] is None else f"{s0[k]:10.3f}" for k, _, _, _ in FIELDS))
        for j in range(1, len(data)):
            sj = next((s for s in data[j]["segments"] if s["t0"] == s0["t0"]), None)
            if sj is None:
                print(f"{'':6}{'':6}{'[' + str(j) + '] missing':>16}")
                continue
            cells = []
            for k, lab, better, thr in FIELDS:
                a, b = s0[k], sj[k]
                if a is None or b is None:
                    cells.append("        —"); continue
                dl = b - a
                cells.append(f"{dl:+10.3f}")
                if better != 0 and dl * better < -thr:
                    regressions.append((names[j], s0["t0"], s0["h_cmd"], st, lab, a, b))
                elif better == 0 and abs(dl) > thr:
                    regressions.append((names[j], s0["t0"], s0["h_cmd"], st, lab, a, b))
            print(f"{'':6}{'':6}{'[' + str(j) + '] Δ':>16}  " + "  ".join(cells))

    for j in range(1, len(data)):
        y0, yj = data[0]["yaw_drift"], data[j]["yaw_drift"]
        if y0 and yj:
            print(f"\nyaw drift         [0] {y0['bias_deg_s']:+.2f} {y0['per_vx_deg_s']:+.2f}*vx {y0['per_vy_deg_s']:+.2f}*vy "
                  f"(res {y0['resid_rms_deg_s']:.2f})")
            print(f"                  [{j}] {yj['bias_deg_s']:+.2f} {yj['per_vx_deg_s']:+.2f}*vx {yj['per_vy_deg_s']:+.2f}*vy "
                  f"(res {yj['resid_rms_deg_s']:.2f})")
        t0 = {t["t0"]: t for t in data[0]["turns"]}
        for t in data[j]["turns"]:
            if t["t0"] in t0:
                print(f"turn in place t={t['t0']:.0f} h={t['h']:.2f}: net displacement "
                      f"{t0[t['t0']]['net_disp_m']:.3f} -> {t['net_disp_m']:.3f} m")

    if regressions:
        print(f"\n=== REGRESSIONS beyond threshold ({len(regressions)}) ===")
        for n, t0, h, st, lab, a, b in regressions:
            print(f"  {n}: t={t0:.0f} h={h:.2f} [{st}]  {lab}: {a:.3f} -> {b:.3f}")
    else:
        print("\nNo regression beyond threshold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
