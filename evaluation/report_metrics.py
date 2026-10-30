#!/usr/bin/env python3
"""FIXED metrics of a protocol run, to compare different policies on the same commands.

The advantage of a scripted protocol is that the segmentation does not have to be guessed from the
data: protocol.yaml says which manoeuvre was running at which simulation time. Two runs of the same
protocol therefore produce the same rows, comparable one by one.

    ./report_metrics.py <run_folder>

Writes metrics.json (for compare_runs.py) and report.md to the run folder.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

SETTLE = 2.0      # [s] dropped at the start of every interval (command transient)
MIN_DUR = 3.0     # [s] minimum duration for an interval to be measured
NOMINAL = 0.98


def load(run: Path):
    obs = pd.read_csv(run / "obs.csv")
    err = pd.read_csv(run / "errors.csv")
    cmd = pd.read_csv(run / "commands.csv")
    gt = pd.read_csv(run / "ground_truth.csv")
    return obs, err, cmd, gt


def build(run: Path):
    obs, err, cmd, gt = load(run)
    t = obs["ep_t"].to_numpy()

    # --- alignment: protocol and ground_truth live on SIMULATION time, obs on the controller's time.
    # The two files start at different instants (the simulator starts first), so they are aligned on
    # wall_t, the only common clock.
    gt_wall = gt["wall_t"].to_numpy()
    gt_sim = gt["sim_t"].to_numpy()
    obs_wall = obs["wall_t"].to_numpy()
    sim_of_ep = np.interp(obs_wall, gt_wall, gt_sim)        # simulation time at every obs row
    rs = lambda c: np.interp(obs_wall, gt_wall, gt[c].to_numpy())

    d = {}
    d["t"] = t
    d["sim_t"] = sim_of_ep
    d["vc"] = obs[["velocity_commands_0", "velocity_commands_1", "velocity_commands_2"]].to_numpy()
    d["hcmd"] = obs["base_height_command_0"].to_numpy() + NOMINAL
    pg = obs[["projected_gravity_0", "projected_gravity_1", "projected_gravity_2"]].to_numpy()
    d["pitch"] = np.degrees(np.arctan2(-pg[:, 0], -pg[:, 2]))
    d["roll"] = np.degrees(np.arctan2(pg[:, 1], -pg[:, 2]))
    d["av"] = obs[["base_ang_vel_0", "base_ang_vel_1", "base_ang_vel_2"]].to_numpy()
    gp = obs[["gait_phase_0", "gait_phase_1"]].to_numpy()
    d["gait_on"] = (np.hypot(gp[:, 0], gp[:, 1]) > 0.5).astype(float)

    d["bz"], d["bx"], d["by"] = rs("base_z"), rs("base_x"), rs("base_y")
    vx, vy, d["wz"] = rs("vx"), rs("vy"), rs("wz")
    q = np.stack([rs("qw"), rs("qx"), rs("qy"), rs("qz")], 1)
    yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                               1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
    d["yaw"] = yaw
    d["vxb"] = np.cos(yaw) * vx + np.sin(yaw) * vy
    d["vyb"] = -np.sin(yaw) * vx + np.cos(yaw) * vy

    tau = lambda j: cmd[f"{j}_kp"].to_numpy() * err[f"{j}_q_err"].to_numpy() - cmd[f"{j}_kd"].to_numpy() * obs[f"{j}_vel"].to_numpy()
    d["tau_ap"] = (tau("left_ankle_pitch") + tau("right_ankle_pitch")) / 2
    d["knee_v"] = (obs["left_knee_vel"].to_numpy() + obs["right_knee_vel"].to_numpy()) / 2
    act = obs[[c for c in obs.columns if c.endswith("_act") and "hm" not in c]].to_numpy()
    d["dact"] = np.append(np.linalg.norm(np.diff(act, axis=0), axis=1), 0.0)

    # real-time factor: the protocol runs on simulation time, the controller on its 50 Hz clock. If
    # they diverge, the commanded heights are not those written in the protocol.
    span_sim = gt_sim[-1] - gt_sim[0]
    span_wall = gt_wall[-1] - gt_wall[0]
    d["rtf"] = span_sim / span_wall if span_wall > 0 else float("nan")
    return d


def intervals(proto):
    """Intervals (sim time) with the sticky state resolved, as in the scripted joystick."""
    out, cur = [], {"vx": 0.0, "vy": 0.0, "wz": 0.0, "height": float(proto.get("nominal_height", NOMINAL)), "do": None}
    tl = proto["timeline"]
    for i, n in enumerate(tl):
        s = dict(cur)
        s["do"] = n.get("do")
        for k in ("vx", "vy", "wz", "height"):
            if k in n:
                s[k] = float(n[k])
        s["t0"] = float(n["t"])
        s["t1"] = float(tl[i + 1]["t"]) if i + 1 < len(tl) else float(proto["stop_at"])
        out.append(s)
        cur = {k: s[k] for k in ("vx", "vy", "wz", "height")}
        cur["do"] = None
    return out


def measure(run: Path):
    proto = yaml.safe_load((run / "protocol.yaml").read_text())
    d = build(run)
    rows = []
    for s in intervals(proto):
        if s["do"] is not None:
            continue
        a_sim, b_sim = s["t0"] + SETTLE, s["t1"]
        if b_sim - a_sim < MIN_DUR:
            continue
        m = (d["sim_t"] >= a_sim) & (d["sim_t"] < b_sim)
        if m.sum() < 50:
            continue
        # Reference height: the target ACTUALLY received by the controller, not the one written in
        # the protocol. The two can differ (the scripted joystick integrates its ramp on simulation
        # time, the controller once per policy step on its own clock), and measuring against the
        # protocol would attribute the bench offset to the robot.
        # Mean over the SECOND HALF of the segment: with 0.05 m/s ramps a rise from 0.60 takes 7.6 s
        # and fits entirely in the segment, so the mean over all of it would be biased.
        idx_m = np.where(m)[0]; m2 = np.zeros_like(m); m2[idx_m[len(idx_m) // 2:]] = True
        h_ctrl = float(d["hcmd"][m2].mean())
        cx, cy, cw = s["vx"], s["vy"], s["wz"]
        # command ACTUALLY seen by the policy (after the controller's depth scaling or cutoff)
        sc = d["vc"][m].mean(axis=0)
        ax, ay, aw = d["vxb"][m].mean(), d["vyb"][m].mean(), d["wz"][m].mean()
        g = lambda a, c: (a / c) if abs(c) > 0.02 else None
        rows.append({
            "t0": round(s["t0"], 1), "dur": round(b_sim - a_sim, 1),
            "h_cmd": s["height"],                      # height requested by the protocol
            "h_ctrl": round(h_ctrl, 4),                # height actually commanded to the robot
            "h_act": round(float(d["bz"][m2].mean()), 4),                 # same window as h_ctrl
            "h_err": round(float(d["bz"][m2].mean() - h_ctrl), 4),      # TRUE tracking error
            "h_drift": round(h_ctrl - s["height"], 4),                 # bench offset
            "stick": [cx, cy, cw],
            "cmd_scaled": [round(float(v), 3) for v in sc],
            "achieved": [round(float(ax), 3), round(float(ay), 3), round(float(aw), 3)],
            "gain_vx": g(ax, sc[0]), "gain_vy": g(ay, sc[1]), "gain_wz": g(aw, sc[2]),
            "sway": round(float(np.hypot(d["vxb"][m] - ax, d["vyb"][m] - ay).mean()), 4),
            "pitch": round(float(d["pitch"][m].mean()), 2),
            "roll_sd": round(float(d["roll"][m].std()), 3),
            "gyro": round(float(np.linalg.norm(d["av"][m], axis=1).mean()), 4),
            "gait_pct": round(float(100 * d["gait_on"][m].mean()), 0),
            "tau_ankle": round(float(d["tau_ap"][m].mean()), 2),
            "knee_vel": round(float(np.abs(d["knee_v"][m]).mean()), 4),
            "dact": round(float(d["dact"][m].mean()), 4),
        })
        r = rows[-1]
        for k in ("gain_vx", "gain_vy", "gain_wz"):
            if r[k] is not None:
                r[k] = round(r[k], 3)

    # --- yaw drift: omega = a + b*vx + c*vy over the segments WITHOUT yaw command ----------------
    lin = [r for r in rows if abs(r["stick"][2]) < 0.05 and (abs(r["stick"][0]) > 0.02 or abs(r["stick"][1]) > 0.02)]
    drift = None
    if len(lin) >= 3:
        A, y = [], []
        for r in lin:
            m = (d["sim_t"] >= r["t0"] + SETTLE) & (d["sim_t"] < r["t0"] + SETTLE + r["dur"])
            idx = np.where(m)[0]
            rate = (d["yaw"][idx[-1]] - d["yaw"][idx[0]]) / (d["t"][idx[-1]] - d["t"][idx[0]])
            A.append([1.0, r["cmd_scaled"][0], r["cmd_scaled"][1]])
            y.append(rate)
        A, y = np.array(A), np.array(y)
        c, *_ = np.linalg.lstsq(A, y, rcond=None)
        drift = {"bias_deg_s": round(float(np.degrees(c[0])), 2),
                 "per_vx_deg_s": round(float(np.degrees(c[1])), 2),
                 "per_vy_deg_s": round(float(np.degrees(c[2])), 2),
                 "resid_rms_deg_s": round(float(np.degrees((y - A @ c).std())), 2),
                 "n_bouts": len(lin)}

    # --- straightness: deviation from the commanded line, and translation during pure yaw -------
    straight, turns = [], []
    for r in rows:
        m = np.where((d["sim_t"] >= r["t0"] + SETTLE) & (d["sim_t"] < r["t0"] + SETTLE + r["dur"]))[0]
        if len(m) < 50:
            continue
        a, b = m[0], m[-1]
        x, yv = d["bx"][a:b] - d["bx"][a], d["by"][a:b] - d["by"][a]
        y0 = d["yaw"][a]
        u = np.cos(y0) * x + np.sin(y0) * yv
        v = -np.sin(y0) * x + np.cos(y0) * yv
        dyaw = np.degrees(d["yaw"][b] - d["yaw"][a])
        cx, cy, cw = r["cmd_scaled"]
        if abs(cw) > 0.05 and abs(cx) < 0.02 and abs(cy) < 0.02:
            turns.append({"t0": r["t0"], "h": r["h_cmd"], "wz": round(cw, 3),
                          "dyaw_deg": round(dyaw, 1), "net_disp_m": round(float(np.hypot(u[-1], v[-1])), 3)})
        elif (abs(cx) > 0.02 or abs(cy) > 0.02) and abs(cw) < 0.05:
            cd = np.array([cx, cy]) / max(np.hypot(cx, cy), 1e-9)
            along = u * cd[0] + v * cd[1]
            perp = -u * cd[1] + v * cd[0]
            straight.append({"t0": r["t0"], "h": r["h_cmd"], "cmd": [round(cx, 3), round(cy, 3)],
                             "along_m": round(float(along[-1]), 3), "dev_m": round(float(perp[-1]), 3),
                             "dev_pct": round(float(100 * abs(perp[-1]) / max(abs(along[-1]), 1e-6)), 1),
                             "dyaw_deg": round(dyaw, 1)})

    # --- validity checks of the run ------------------------------------------------------------------
    checks = []
    if not (0.97 <= d["rtf"] <= 1.03):
        checks.append(f"real-time factor {d['rtf']:.3f} outside [0.97, 1.03]: the scripted joystick "
                      f"runs on simulation time and the controller on its own clock, so the commanded "
                      f"heights may differ from the protocol")
    for r in rows:
        # robot tracking: must be tight, the target is a slow ramp that has already settled
        if abs(r["h_err"]) > 0.01:
            checks.append(f"t={r['t0']}: controller target {r['h_ctrl']:.3f} but measured "
                          f"{r['h_act']:.3f} (error {r['h_err']:+.3f} m)")
    # Bench offset: the controller did not receive the height written in the protocol. It does not
    # invalidate a comparison between policies (it is the same in every run), but it must be VISIBLE.
    # Grouped by height, otherwise the same line repeats for every segment held at that depth.
    # Not named `drift`: that name is already the yaw drift regression above.
    h_groups: dict = {}
    for r in rows:
        if abs(r["h_drift"]) > 0.01:
            h_groups.setdefault((r["h_cmd"], r["h_ctrl"], r["h_drift"]), []).append(r["t0"])
    for (hc, hk, hd), ts in sorted(h_groups.items()):
        checks.append(f"the protocol asks for {hc:.3f} but the controller received {hk:.3f} "
                      f"({hd:+.3f} m bench offset) — segments t={', '.join(f'{t:g}' for t in ts)}")
    return {"rtf": round(float(d["rtf"]), 4), "segments": rows, "yaw_drift": drift,
            "straight": straight, "turns": turns, "checks": checks}


def render(run: Path, met: dict) -> str:
    meta = json.loads((run / "run_meta.json").read_text()) if (run / "run_meta.json").exists() else {}
    L = [f"# Protocol report — {run.name}", ""]
    L += [f"- protocol: `{meta.get('protocol', '?')}`",
          f"- policy: `{meta.get('policy', '?')}`",
          f"- label: {meta.get('label') or '—'}",
          f"- commit: `{meta.get('git_commit', '?')}`" + (" **(dirty tree)**" if meta.get("git_dirty") else ""),
          f"- real-time factor: {met['rtf']:.4f}", ""]
    if met["checks"]:
        L += ["## ⚠ Failed checks", ""] + [f"- {c}" for c in met["checks"]] + [""]
    else:
        L += ["All validity checks passed.", ""]

    L += ["## Segments", "",
          "| t0 | dur | h prot | h ctrl | h meas | err | stick | cmd scaled | achieved | gain | sway | pitch | rollSD | gyro | gait% | tau ankle | knee | Δact |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in met["segments"]:
        gains = " / ".join("—" if r[k] is None else f"{r[k]:.2f}" for k in ("gain_vx", "gain_vy", "gain_wz"))
        st = ",".join(f"{v:g}" for v in r["stick"])
        cs = ",".join(f"{v:g}" for v in r["cmd_scaled"])
        ac = ",".join(f"{v:g}" for v in r["achieved"])
        L.append(f"| {r['t0']} | {r['dur']} | {r['h_cmd']:.2f} | {r['h_ctrl']:.3f} | {r['h_act']:.3f} | {r['h_err']:+.3f} | {st} | {cs} | {ac} | {gains} | "
                 f"{r['sway']:.4f} | {r['pitch']:.2f} | {r['roll_sd']:.3f} | {r['gyro']:.4f} | {r['gait_pct']:.0f} | "
                 f"{r['tau_ankle']:.1f} | {r['knee_vel']:.4f} | {r['dact']:.4f} |")

    if met["yaw_drift"]:
        y = met["yaw_drift"]
        L += ["", "## Yaw drift (segments without yaw command)", "",
              f"`omega [deg/s] = {y['bias_deg_s']:+.2f} {y['per_vx_deg_s']:+.2f}*vx {y['per_vy_deg_s']:+.2f}*vy` "
              f"— residual RMS {y['resid_rms_deg_s']:.2f}, {y['n_bouts']} segments"]

    if met["straight"]:
        L += ["", "## Straightness (linear segments)", "",
              "| t0 | h | command | travelled | deviation | % | Δyaw |", "|---|---|---|---|---|---|---|"]
        for s in met["straight"]:
            L.append(f"| {s['t0']} | {s['h']:.2f} | {s['cmd'][0]:+.2f},{s['cmd'][1]:+.2f} | {s['along_m']:+.2f} m | "
                     f"{s['dev_m']:+.3f} m | {s['dev_pct']:.0f}% | {s['dyaw_deg']:+.1f}° |")

    if met["turns"]:
        L += ["", "## Turning in place", "", "| t0 | h | wz | Δyaw | net displacement |", "|---|---|---|---|---|"]
        for s in met["turns"]:
            L.append(f"| {s['t0']} | {s['h']:.2f} | {s['wz']:+.3f} | {s['dyaw_deg']:+.1f}° | {s['net_disp_m']:.3f} m |")
    return "\n".join(L) + "\n"


def main() -> int:
    run = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
    if run is None or not (run / "protocol.yaml").exists():
        print("usage: report_metrics.py <run_folder>   (must contain protocol.yaml)")
        return 1
    met = measure(run)
    (run / "metrics.json").write_text(json.dumps(met, indent=2))
    (run / "report.md").write_text(render(run, met))
    print(f"written {run/'metrics.json'} and {run/'report.md'}")
    for c in met["checks"]:
        print("  ⚠ " + c)
    return 0


if __name__ == "__main__":
    sys.exit(main())
