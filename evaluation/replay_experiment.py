#!/usr/bin/env python3
"""Replays in MuJoCo an experiment made on the real robot, with the SAME commands.

    ./replay_experiment.py sim_logs/exp_2026-09-22_14-45-16
    ./replay_experiment.py sim_logs/exp_*          # all of them

For every experiment it produces a folder `sim_logs/sim_<ts>_replay_<exp>/` with the same files as
any sim2sim run (obs.csv, commands.csv, errors.csv, ground_truth.csv, plots/) plus:
    replay.yaml          the joystick timeline rebuilt from the experiment
    source_run_info.md   the run_info.md of the real experiment
    replay_fidelity.md   how closely the commands seen by the policy in sim match the real ones

METHOD. Nothing is forced inside the controller: the simulation runs the SAME controller as the robot
(same code, same policy, same kp/kd, same deploy.yaml) and only the joystick source changes, the
simulator's scripted joystick instead of the remote. The joystick timeline is rebuilt from obs.csv
of the experiment, which records EXACTLY the commands seen by the policy at every step:

  - velocity (velocity_commands): in the controller it goes through an EMA filter (x <- 0.97x + 0.03u
    at 1 kHz) before being recorded. Sending it back as is would filter it twice (~33 ms of delay);
    here the filter is PRE-COMPENSATED: for every step k, u_k is sent such that, after the 20 filter
    updates between two steps, the output is exactly the recorded value:
        u_k = (y_k - 0.97^20 * y_{k-1}) / (1 - 0.97^20)
    and the simulator-side filter is disabled (stick_smoothing: false).
  - height (base_height_command): the controller integrates it by 1 mm per step while RT (down) or B
    (up) is pressed. The timeline carries the recorded height step by step, and the scripted joystick
    holds RT/B for exactly those steps. RT also has an EMA filter on the controller side (~23 ms to
    pass the threshold), B does not: the descent entries are anticipated accordingly.
  - timing: the controller times its steps on the WALL clock, while the simulation runs at a real-time
    factor slightly below 1. The replay therefore follows the wall clock from the RL entry on
    (clock: wall_after_rl), otherwise the commands would slip by several steps over a long replay.
  - start: FixStand, then the elastic band is released BEFORE entering RL (on the real robot the
    rope was slack: the policy starts with the robot fully on its feet), then RL.

LIMIT. The operator's COMMANDS are replayed, not the external disturbances: pushes by hand, contacts
with the rope or with the operator are not measured and do not appear in the replay.
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

HERE = Path(__file__).resolve().parent
POLICIES = HERE.parent / "policies" / "h1_2_squat"

# --- constants of the controller and of the scripted joystick (they must match) ----------------------
STEP = 0.02                 # policy step_dt
EMA_A = 0.97                # Axis::smooth = 0.03 -> x <- 0.97 x + 0.03 u, at 1 kHz (FSM loop)
EMA_N = 20                  # filter updates between two policy steps
RT_DELAY = 0.023            # time for the EMA filter to bring RT above 0.5 in the controller: ln(.5)/ln(.97) ms
MARGIN = 0.010              # timing margin of the ramps (half a step)
CTRL_DELAY = 0.003          # RL entry and first step: a few ms after the X edge
MODIFIER_LEAD = 0.15        # = kModifierLead in scripted_joystick.h (X edge after RB)

T_FIXSTAND = 2.0            # LT + up (3 s FixStand ramp)
T_RL = 8.0                  # RB + X
# Band release 0.3 s BEFORE the RL entry. On the real robot the rope was slack: the policy starts with
# the robot fully on its feet. In simulation, however, the FixStand pose without the band is NOT stable
# for long (released 1.5 s earlier, the robot collapses before RL). 0.3 s before the RL entry the robot
# is already fully loaded and still standing.
BAND_LEAD = 0.30
T_BAND = T_RL + MODIFIER_LEAD - BAND_LEAD


def build_timeline(exp: Path, nominal: float, rate: float) -> dict:
    obs = pd.read_csv(exp / "obs.csv")
    y = obs[["velocity_commands_0", "velocity_commands_1", "velocity_commands_2"]].to_numpy(float)
    h = obs["base_height_command_0"].to_numpy(float) + nominal
    n = len(obs)

    # pre-compensation of the controller's EMA filter
    a = EMA_A ** EMA_N
    yprev = np.vstack([np.zeros((1, 3)), y[:-1]])
    u = (y - a * yprev) / (1.0 - a)
    u[np.abs(u) < 1e-6] = 0.0

    # height: step by step, the sign of the change selects RT (down) or B (up)
    dh = np.diff(np.concatenate([[nominal], h]))
    anchor = T_RL + MODIFIER_LEAD + CTRL_DELAY        # time of step k = 0
    tl = [{"t": T_FIXSTAND, "do": "fixstand"}, {"t": T_RL, "do": "rl"},
          {"t": round(T_RL + 0.5, 4), "vx": 0.0, "vy": 0.0, "wz": 0.0, "height": nominal}]

    last = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    events = []   # (t, dict)
    for k in range(n):
        s_k = anchor + STEP * k
        # sticks: the pre-compensated value must act in the 20 ms before step k
        cur = {"vx": float(u[k, 0]), "vy": float(u[k, 1]), "wz": float(u[k, 2])}
        if any(abs(cur[c] - last[c]) > 1e-6 for c in cur):
            events.append((s_k - STEP, {c: round(v, 6) for c, v in cur.items()}))
            last = cur
        # height: the entry carries the target of step k; different lead for RT (filter) and B
        if abs(dh[k]) > 1e-7:
            lead = (RT_DELAY + MARGIN) if dh[k] < 0 else MARGIN
            events.append((s_k - lead, {"height": round(float(h[k]), 5)}))

    # the timeline must be sorted; entries at the same time are merged
    events.sort(key=lambda e: e[0])
    merged = []
    for t, d in events:
        t = round(t, 4)
        if merged and abs(merged[-1]["t"] - t) < 1e-9:
            merged[-1].update(d)
        else:
            merged.append({"t": t, **d})
    tl += merged

    stop_at = anchor + STEP * n + 2.0
    return {
        "name": f"replay_{exp.name}",
        "replay_of": str(exp),
        "nominal_height": nominal,
        "deepest_height": 0.60,
        "descend_rate": rate,
        "rise_rate": rate,
        "band_release_at": T_BAND,
        "overhold": 0.2,
        "stick_smoothing": False,
        "clock": "wall_after_rl",
        "stop_at": round(stop_at, 2),
        "timeline": tl,
    }


def fidelity(real: Path, sim: Path, nominal: float) -> str:
    r = pd.read_csv(real / "obs.csv"); s = pd.read_csv(sim / "obs.csv")
    cols = ["velocity_commands_0", "velocity_commands_1", "velocity_commands_2"]
    yr = r[cols].to_numpy(float); ys = s[cols].to_numpy(float)
    hr = r.base_height_command_0.to_numpy(float) + nominal; hs = s.base_height_command_0.to_numpy(float) + nominal
    n = min(len(r), len(s))
    if n < 50:
        return (f"# Replay fidelity — {real.name}\n\n**REPLAY FAILED**: the simulation recorded "
                f"only {len(s)} policy steps against {len(r)} on the real robot (robot fallen or controller "
                f"out of RL). Check ground_truth.csv.\n")

    def err_at(lag):
        a, b = (slice(0, n - lag), slice(lag, n)) if lag >= 0 else (slice(-lag, n), slice(0, n + lag))
        return yr[a] - ys[b], hr[a] - hs[b]

    lags = range(-5, 6)
    cost = {L: np.sqrt(np.mean(err_at(L)[0] ** 2)) + np.sqrt(np.mean(err_at(L)[1] ** 2)) for L in lags}
    # with constant commands (e.g. only standing) every lag gives the same error: not measurable
    if max(cost.values()) - min(cost.values()) < 1e-9:
        lag_txt = "not measurable (constant commands)"
    else:
        lag_txt = f"**{min(cost, key=cost.get)} steps** (0 = aligned)"
    ev, eh = err_at(0)
    active = np.linalg.norm(yr[:n], axis=1) > 0.02
    L = []
    L.append(f"# Replay fidelity — {real.name}\n")
    L.append(f"- real steps {len(r)}, sim steps {len(s)} (compared on the first {n})")
    L.append(f"- lag that minimizes the error: {lag_txt}\n")
    L.append("| command | RMS error | max error | RMS while commanding | real amplitude |")
    L.append("|---|---|---|---|---|")
    for i, c in enumerate(["vx [m/s]", "vy [m/s]", "wz [rad/s]"]):
        e = ev[:, i]
        ea = e[active] if active.any() else np.array([0.0])
        L.append(f"| {c} | {np.sqrt(np.mean(e**2)):.4f} | {np.abs(e).max():.4f} | {np.sqrt(np.mean(ea**2)):.4f} | {np.abs(yr[:n, i]).max():.3f} |")
    L.append(f"| height [m] | {np.sqrt(np.mean(eh**2)):.4f} | {np.abs(eh).max():.4f} | — | {hr.min():.3f}–{hr.max():.3f} |")
    # initial state at the RL entry: how closely the sim starts from the same condition as the real robot
    pg = ["projected_gravity_0", "projected_gravity_1", "projected_gravity_2"]
    def tilt(df):
        g = df[pg].to_numpy(float)[:25]
        return np.degrees(np.arctan2(-g[:, 0], -g[:, 2])).mean(), np.degrees(np.arctan2(g[:, 1], -g[:, 2])).mean()
    jp = [c for c in r.columns if c.endswith("_pos") and "hm" not in c]
    dj = np.abs(r[jp].to_numpy(float)[0] - s[jp].to_numpy(float)[0])
    (pr, rr), (ps, rs) = tilt(r), tilt(s)
    L.append("")
    L.append("**State at the RL entry** (first 0.5 s): "
             f"pitch real {pr:+.2f}° / sim {ps:+.2f}°, roll real {rr:+.2f}° / sim {rs:+.2f}°; "
             f"joint positions at the first step: mean difference {np.degrees(dj.mean()):.2f}°, max {np.degrees(dj.max()):.2f}° "
             f"({jp[int(dj.argmax())].replace('_pos', '')}).")
    L.append("")
    L.append("The comparison is step by step (same ep_t in the two logs): every obs.csv row in sim is the")
    L.append("command the policy saw at the same step after the RL entry on the real robot.")
    return "\n".join(L) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiments", nargs="+", help="exp_<ts> folders of the real robot")
    ap.add_argument("--policy", default=None, help="policy folder (default: the one named in run_info.md, from policies/h1_2_squat)")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--python", default=sys.executable)
    args = ap.parse_args()

    rc_all = 0
    for e in args.experiments:
        exp = Path(e).resolve()
        if not (exp / "obs.csv").exists():
            print(f"SKIP {exp}: obs.csv missing"); rc_all = 1; continue

        info = (exp / "run_info.md").read_text() if (exp / "run_info.md").exists() else ""
        m = re.search(r"\*\*policy_name\*\*:\s*(\S+)", info)
        policy = Path(args.policy).resolve() if args.policy else (POLICIES / m.group(1) if m else None)
        if policy is None or not (policy / "exported" / "policy.onnx").exists():
            print(f"SKIP {exp.name}: policy not found ({policy})"); rc_all = 1; continue

        dep = yaml.safe_load((policy / "params" / "deploy.yaml").read_text())
        nominal = float(dep["observations"]["base_height_command"]["params"]["nominal_height"])
        # controller ramp: measured from the experiment itself (1 mm/step -> 0.05 m/s)
        h = pd.read_csv(exp / "obs.csv").base_height_command_0.to_numpy(float)
        steps = np.abs(np.diff(h)); steps = steps[steps > 1e-7]
        rate = float(np.median(steps) / STEP) if len(steps) else 0.05
        rate = round(rate, 4)

        proto = build_timeline(exp, nominal, rate)
        tmp = Path("/tmp") / f"replay_{exp.name}.yaml"
        tmp.write_text(yaml.safe_dump(proto, sort_keys=False, default_flow_style=None, width=200))
        print(f"\n=== {exp.name}: {len(proto['timeline'])} entries, ramp {rate} m/s, duration {proto['stop_at']:.0f} s, policy {policy.name}")

        cmd = [args.python, str(HERE / "run_protocol.py"), "--protocol", str(tmp), "--policy", str(policy),
               "--label", f"replay_{exp.name}", "--no-metrics", "--meta", f"replay_of={exp}"]
        if args.no_plots:
            cmd.append("--no-plots")
        out = subprocess.run(cmd, capture_output=True, text=True)
        sys.stdout.write("\n".join(l for l in out.stdout.splitlines() if not l.startswith("RUN_DIR=")) + "\n")
        mm = re.search(r"^RUN_DIR=(.+)$", out.stdout, re.M)
        if out.returncode != 0 or not mm:
            print(out.stderr[-2000:]); print(f"ERROR in the replay of {exp.name}"); rc_all = 1; continue
        run = Path(mm.group(1))
        dest = run.with_name(f"{run.name}_replay_{exp.name}")
        run.rename(dest)
        shutil.copy(tmp, dest / "replay.yaml")
        if (exp / "run_info.md").exists():
            shutil.copy(exp / "run_info.md", dest / "source_run_info.md")
        rep = fidelity(exp, dest, nominal)
        (dest / "replay_fidelity.md").write_text(rep)
        print(rep)
        print(f"folder: {dest}")
    return rc_all


if __name__ == "__main__":
    sys.exit(main())
