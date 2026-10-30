#!/usr/bin/env python3
"""Response of the robot to the MANUAL pushes given from the keyboard in the MuJoCo window.

The simulator applies the pushes (simulator/src/push_disturbance.h, presets in
simulator/disturbances.yaml) during a normal sim2sim session with the physical joystick, and logs
them to sim_logs/sim_<ts>/disturbance_events.csv (with the ground-truth log active). This script
reads them with ground_truth.csv and obs.csv and, for every push, measures: change of the CoM
velocity (against the one expected from the impulse: checks that the force really arrived), peak
pitch/roll, maximum and final displacement, yaw rotation, settling time, fall. Writes pushes.json.

    ./report_pushes.py sim_logs/sim_<ts>

Not part of the automated protocols (run_protocol.py): it is a tool for manual tests.
"""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np, pandas as pd

PRE = 0.5      # [s] window before the push, for the baseline
POST = 4.0     # [s] observation window after the push
V_SETTLE = 0.05    # [m/s]
TILT_SETTLE = 1.0  # [deg] with respect to the baseline
HOLD = 0.5         # [s] to declare it settled
FALL_Z = 0.50      # [m]
FALL_TILT = 30.0   # [deg]


def main() -> int:
    run = Path(sys.argv[1]).resolve()
    ev = pd.read_csv(run / "disturbance_events.csv")
    gt = pd.read_csv(run / "ground_truth.csv")
    obs = pd.read_csv(run / "obs.csv")
    t = gt.sim_t.to_numpy()
    q = gt[["qw", "qx", "qy", "qz"]].to_numpy()
    # roll/pitch from the base quaternion (negative pitch = forward, as in the other analyses)
    pitch = -np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
    roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
    v = gt[["vx", "vy"]].to_numpy(); z = gt.base_z.to_numpy(); xy = gt[["base_x", "base_y"]].to_numpy()
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))))
    # commanded height on the time axis of the ground truth
    osim = np.interp(obs.wall_t.to_numpy(), gt.wall_t.to_numpy(), t)
    hcmd = np.interp(t, osim, obs.base_height_command_0.to_numpy() + 0.98)

    starts = ev[ev.event == "start"].reset_index(drop=True)
    rows = []
    for _, e in starts.iterrows():
        t0 = float(e.sim_t); dur = float(e.dur_planned)
        F = np.array([e.fx, e.fy, e.fz]); T = np.array([e.tx, e.ty, e.tz])
        imp = np.linalg.norm(F) * max(dur, 0.005)
        pre = (t >= t0 - PRE) & (t < t0); post = (t >= t0) & (t < t0 + POST)
        v0 = v[pre].mean(0); p0 = pitch[pre].mean(); r0 = roll[pre].mean(); xy0 = xy[pre].mean(0)
        dv = np.linalg.norm(v[post] - v0, axis=1)
        dp = pitch[post] - p0; dr = roll[post] - r0
        disp = np.linalg.norm(xy[post] - xy0, axis=1)
        dyaw = yaw[post] - yaw[pre].mean()
        fell = bool((z[post] < FALL_Z).any() or (np.abs(dp) > FALL_TILT).any() or (np.abs(dr) > FALL_TILT).any())
        # settling: first instant from which, for HOLD s, |dv|<V_SETTLE and |tilt|<TILT_SETTLE
        tp = t[post]; ok = (dv < V_SETTLE) & (np.abs(dp) < TILT_SETTLE) & (np.abs(dr) < TILT_SETTLE)
        settle = None
        n_hold = int(HOLD / np.median(np.diff(tp)))
        for i in range(len(ok) - n_hold):
            if ok[i:i + n_hold].all(): settle = float(tp[i] - t0); break
        rows.append({
            "t": round(t0, 2), "body": e.body, "h_cmd": round(float(hcmd[pre].mean()), 3),
            "force": [float(x) for x in F], "torque": [float(x) for x in T], "dur": dur, "impulse_Ns": round(float(imp), 1),
            "dv_peak": round(float(dv.max()), 3), "dv_expected": round(float(imp / 79.5), 3),
            "dpitch_peak": round(float(dp[np.argmax(np.abs(dp))]), 2), "droll_peak": round(float(dr[np.argmax(np.abs(dr))]), 2),
            "disp_max": round(float(disp.max()), 3), "disp_end": round(float(disp[-1]), 3),
            "dyaw_end": round(float(dyaw[-1]), 2),
            "settle_s": None if settle is None else round(settle, 2), "fell": fell,
        })
    (run / "pushes.json").write_text(json.dumps(rows, indent=2))
    print(f"{'t':>6}{'h':>6}{'body':>12}{'F/T':>22}{'N.s':>6}{'dv_pk':>7}{'(exp)':>7}{'dpitch':>8}{'droll':>7}{'disp':>7}{'end':>7}{'dyaw':>7}{'settle':>8}{'fell':>8}")
    for r in rows:
        ft = f"F{tuple(int(x) for x in r['force'])}" if any(r["force"]) else f"T{tuple(int(x) for x in r['torque'])}"
        st = "—" if r["settle_s"] is None else "%.2f" % r["settle_s"]
        print(f"{r['t']:6.1f}{r['h_cmd']:6.2f}{r['body']:>12}{ft:>22}{r['impulse_Ns']:6.0f}{r['dv_peak']:7.3f}{r['dv_expected']:7.3f}"
              f"{r['dpitch_peak']:+8.2f}{r['droll_peak']:+7.2f}{r['disp_max']:7.3f}{r['disp_end']:7.3f}{r['dyaw_end']:+7.1f}"
              f"{st:>8}{('YES' if r['fell'] else 'no'):>8}")
    print(f"\nwritten {run / 'pushes.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
