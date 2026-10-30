#!/usr/bin/env python3
"""Runs a sim2sim test protocol automatically and repeatably.

Replaces the manual sequence (start the controller, start MuJoCo, LT+up, RB+X, key 9, joystick,
close, analyze_sim.py) with a single command. The joystick is played INSIDE the simulator against
simulation time (see simulator/src/scripted_joystick.h), because the controller reads the joystick
from lowstate->joystick, which the simulator publishes: there is no way to inject it from outside
via DDS.

    ./run_protocol.py                                  # standard protocol, final policy
    ./run_protocol.py --protocol protocols/smoke.yaml --label quick
    ./run_protocol.py --policy /path/to/<run> --no-plots

At the end, inside the sim_<timestamp>/ folder of the run (under --log-dir, default <repo>/sim_logs):
    protocol.yaml   exact copy of the protocol used (the run is self-contained)
    run_meta.json   policy, commit, run information
    metrics.json    the fixed metrics, for compare_runs.py
    report.md       the same metrics in readable form
    plots/          the plots of analyze_sim.py
    controller.log, simulator.log   terminal output of the two processes
"""
from __future__ import annotations

import argparse
import json
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
SIM_DIR = REPO / "simulator"
SIM_BIN = SIM_DIR / "build" / "unitree_mujoco"
CTRL_DIR = REPO / "deploy" / "robots" / "h1_2_squat"
CTRL_BIN = CTRL_DIR / "build" / "h1_2_ctrl_squat"
POLICIES = REPO / "policies" / "h1_2_squat"


def newest(pattern_dir: Path, pattern: str) -> Path | None:
    items = sorted(pattern_dir.glob(pattern))
    return items[-1] if items else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--protocol", default=str(Path(__file__).parent / "protocols" / "standard_v3.yaml"))
    ap.add_argument("--policy", default=None, help="policy folder (default: the newest in policies/h1_2_squat)")
    ap.add_argument("--log-dir", default=str(REPO / "sim_logs"), help="folder of the runs (default: <repo>/sim_logs)")
    ap.add_argument("--label", default=None, help="run label, written to run_meta.json")
    ap.add_argument("--no-plots", action="store_true", help="skip analyze_sim.py")
    ap.add_argument("--no-metrics", action="store_true",
                    help="skip report_metrics.py (for replays: the per-segment protocol metrics do not apply)")
    ap.add_argument("--meta", action="append", default=[], metavar="KEY=VALUE",
                    help="extra fields for run_meta.json (repeatable)")
    ap.add_argument("--python", default=sys.executable, help="interpreter for the analysis scripts")
    ap.add_argument("--keep-config", action="store_true", help="keep the generated temporary configs")
    args = ap.parse_args()

    protocol = Path(args.protocol).resolve()
    logs = Path(args.log_dir).resolve()
    for path, what in [(SIM_BIN, "simulator"), (CTRL_BIN, "controller"), (protocol, "protocol")]:
        if not path.exists():
            print(f"ERROR: {what} not found: {path}")
            return 1

    proto = yaml.safe_load(protocol.read_text())
    stop_at = float(proto["stop_at"])

    policy = Path(args.policy).resolve() if args.policy else newest(POLICIES, "*")
    if policy is None or not policy.exists():
        print(f"ERROR: no policy found in {POLICIES}")
        return 1

    # --- height stop: the protocol and the controller must see it at the same height -----------------
    # The controller stops R2 at cutoff_height + 1 cm (from the policy's deploy.yaml); the scripted
    # joystick models it with `stop_height`. If they differ, the heights below the stop are commanded
    # wrongly without any visible error. A protocol WITHOUT `stop_height` holds A with RT and does not
    # depend on the stop: it works with any policy.
    if "stop_height" in proto:
        dep = yaml.safe_load((policy / "params" / "deploy.yaml").read_text())
        cutoff = (dep.get("commands", {}).get("base_velocity") or {}).get("cutoff_height")
        if cutoff is None or abs(float(cutoff) + 0.01 - float(proto["stop_height"])) > 1e-6:
            print(f"ERROR: stop_height {proto['stop_height']} in the protocol, but the policy has "
                  f"cutoff_height {cutoff} (controller stop = cutoff_height + 0.01)")
            return 1

    # --- THROWAWAY configs, generated here and passed with --config --------------------------------
    # The config.yaml files are never touched: the controller's one is the SAME file used on the real
    # robot, and an interrupted test must not leave it pointing to the policy under test.
    tmp = Path(tempfile.mkdtemp(prefix="h1_2_protocol_"))

    sim_cfg = yaml.safe_load((SIM_DIR / "config.yaml").read_text())
    sim_cfg["use_joystick"] = 1
    sim_cfg["joystick_type"] = "script"
    sim_cfg["joystick_script"] = str(protocol)
    sim_cfg["enable_elastic_band"] = 1   # needed during FixStand; the release is in the protocol
    sim_cfg["disturbance_script"] = ""   # never keyboard pushes in automated runs
    sim_cfg["log_ground_truth"] = 1
    sim_cfg["log_dir"] = str(logs)
    sim_cfg_path = tmp / "sim_config.yaml"
    sim_cfg_path.write_text(yaml.safe_dump(sim_cfg, sort_keys=False))

    # `robot_scene` is relative and both programs resolve it against the folder of the BINARY, not
    # of the config: it stays valid from here.
    ctrl_cfg = yaml.safe_load((CTRL_DIR / "config" / "config.yaml").read_text())
    # The policy under test is set explicitly: otherwise the controller takes "the newest one",
    # which could change between two tests and silently invalidate the comparison.
    ctrl_cfg["FSM"]["Squat"]["policy_dir"] = str(policy)
    ctrl_cfg_path = tmp / "ctrl_config.yaml"
    ctrl_cfg_path.write_text(yaml.safe_dump(ctrl_cfg, sort_keys=False))

    logs.mkdir(parents=True, exist_ok=True)
    before = {p.name for p in logs.glob("sim_*")}
    procs: list[tuple[str, subprocess.Popen]] = []
    # Output of the two processes goes to FILES, then copied into the run. With a PIPE that is never
    # read, a process blocks on writing once the 64 kB buffer is full: the controller prints the
    # height during every ramp, and with its policy thread blocked the simulated robot would get no
    # commands. A folder separate from `tmp`, which the finally block deletes before the run folder is
    # known.
    out_dir = Path(tempfile.mkdtemp(prefix="h1_2_protocol_out_"))
    ctrl_log = open(out_dir / "controller.log", "w")
    sim_log = open(out_dir / "simulator.log", "w")
    t0 = time.time()
    try:
        print(f"protocol   : {protocol.name}  (stop_at {stop_at:.0f} s)")
        print(f"policy     : {policy.name}")
        print("starting the controller...")
        ctrl = subprocess.Popen([str(CTRL_BIN), "--network", "lo", "--config", str(ctrl_cfg_path),
                                 "--log", "--log-dir", str(logs)],
                                cwd=CTRL_BIN.parent,
                                stdout=ctrl_log, stderr=subprocess.STDOUT, text=True)
        procs.append(("controller", ctrl))
        time.sleep(2.0)
        if ctrl.poll() is not None:
            ctrl_log.flush()
            print("ERROR: the controller exited immediately:\n" + (out_dir / "controller.log").read_text())
            return 1

        print("starting the simulator...")
        sim = subprocess.Popen([str(SIM_BIN), "--config", str(sim_cfg_path)], cwd=SIM_DIR,
                               stdout=sim_log, stderr=subprocess.STDOUT, text=True)
        procs.append(("simulator", sim))
        t_sim = time.time()   # the protocol starts here, not when the controller started

        # the protocol runs on simulation time: if MuJoCo slows down, wait longer
        budget = stop_at * 1.6 + 30.0
        print(f"running (waiting up to {budget:.0f} s)...")
        deadline = time.time() + budget
        while time.time() < deadline:
            if sim.poll() is not None or ctrl.poll() is not None:
                print("WARNING: a process terminated by itself")
                break
            time.sleep(1.0)
            if time.time() - t_sim > stop_at + 8.0:
                break
        wall = time.time() - t0
    finally:
        for name, p in reversed(procs):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        time.sleep(1.5)
        for name, p in reversed(procs):
            if p.poll() is None:
                p.terminate()
        time.sleep(1.0)
        for name, p in reversed(procs):
            if p.poll() is None:
                p.kill()
        ctrl_log.close()
        sim_log.close()
        if not args.keep_config:
            shutil.rmtree(tmp, ignore_errors=True)
        else:
            print(f"temporary configs kept in {tmp}")

    # --- find the run folder ------------------------------------------------------------------------
    after = {p.name for p in logs.glob("sim_*")}
    created = sorted(after - before)
    if not created:
        print("ERROR: no sim_* folder created; the simulator logged nothing")
        print(f"output of controller and simulator in {out_dir}")
        return 1
    run_dir = logs / created[-1]
    print(f"run        : {run_dir}")

    shutil.copy(protocol, run_dir / "protocol.yaml")
    for f in ("controller.log", "simulator.log"):
        shutil.move(str(out_dir / f), str(run_dir / f))
    shutil.rmtree(out_dir, ignore_errors=True)

    def git(*a):
        try:
            return subprocess.check_output(["git", *a], cwd=REPO, text=True).strip()
        except Exception:
            return None

    meta = {
        "protocol": proto.get("name", protocol.stem),
        "protocol_file": protocol.name,
        "policy": policy.name,
        "policy_path": str(policy),
        "label": args.label,
        "wall_seconds": round(wall, 1),
        "stop_at": stop_at,
        "git_commit": git("rev-parse", "--short", "HEAD"),
        "git_dirty": bool(git("status", "--porcelain")),
    }
    for kv in args.meta:
        k, _, v = kv.partition("=")
        meta[k] = v
    (run_dir / "run_meta.json").write_text(json.dumps(meta, indent=2))

    # --- metrics + report -----------------------------------------------------------------------------
    if not args.no_metrics:
        rc = subprocess.call([args.python, str(Path(__file__).parent / "report_metrics.py"), str(run_dir)])
        if rc != 0:
            print("WARNING: report_metrics.py failed")

    if not args.no_plots:
        subprocess.call([args.python, str(Path(__file__).parent / "analyze_sim.py"), str(run_dir)])

    print(f"\ndone. report: {run_dir / 'report.md'}")
    print(f"RUN_DIR={run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
