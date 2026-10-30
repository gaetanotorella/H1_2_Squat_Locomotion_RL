#!/usr/bin/env python3
"""Quantitative analysis of a run of the controller (MuJoCo sim2sim or real robot).

Two kinds of run, both under sim_logs/:
  - sim_<ts>/  MuJoCo simulation -> includes ground_truth.csv
  - exp_<ts>/  test on the REAL ROBOT -> NO ground_truth.csv

CSV files read (each on its OWN time axis):
  - obs.csv          : observations given to the policy (ep_t, 50 Hz)       [always]
  - commands.csv     : q_target commands sent to the robot (ep_t)            [always]
  - errors.csv       : joint tracking error q_target - q_measured            [always]
  - ground_truth.csv : TRUE state of the base (sim_t, 100 Hz)                [simulation ONLY]

On the real robot the absolute base position, the quaternion and the linear velocity are NOT
measurable: plots and metrics using them are skipped automatically. Roll and pitch are estimated
from projected_gravity (IMU) instead, which is already an observation of the policy.

Joints are in policy order and are split into LEGS (hip/knee/ankle/torso) and ARMS
(shoulder/elbow/wrist). Every plot uses the time of its own file.

Usage:
    python3 analyze_sim.py [run_folder]     # default: the newest sim_*/exp_* in <repo>/sim_logs
"""
import os
import re
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- policy order of the 27 H1_2 joints (i = index in joint_pos_rel/q_target/q_err) ---
JOINT_NAMES = [
    "left_hip_yaw", "right_hip_yaw", "torso", "left_hip_pitch", "right_hip_pitch",
    "left_shoulder_pitch", "right_shoulder_pitch", "left_hip_roll", "right_hip_roll",
    "left_shoulder_roll", "right_shoulder_roll", "left_knee", "right_knee",
    "left_shoulder_yaw", "right_shoulder_yaw", "left_ankle_pitch", "right_ankle_pitch",
    "left_elbow", "right_elbow", "left_ankle_roll", "right_ankle_roll",
    "left_wrist_roll", "right_wrist_roll", "left_wrist_pitch", "right_wrist_pitch",
    "left_wrist_yaw", "right_wrist_yaw",
]
DEFAULT_JOINT_POS = np.array([
    0.0, 0.0, 0.0, -0.3, -0.3, 0.35, 0.35, 0.0, 0.0, 0.25, -0.25, 0.5, 0.5,
    -0.1, -0.1, -0.2, -0.2, 0.65, 0.65, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
])
ARM_KEYS = ("shoulder", "elbow", "wrist")


def is_arm(name):
    return any(k in name for k in ARM_KEYS)


def joint_type(name):
    """strip left_/right_ to pair L/R in the same subplot."""
    return name.replace("left_", "").replace("right_", "")


def side(name):
    return "L" if name.startswith("left_") else ("R" if name.startswith("right_") else "")


def groups(indices):
    """ordered dict type -> [(idx, side)], keeping the order of appearance."""
    g = {}
    for i in indices:
        g.setdefault(joint_type(JOINT_NAMES[i]), []).append((i, side(JOINT_NAMES[i])))
    return g


def latest_run():
    base = os.path.join(REPO, "sim_logs")
    runs = sorted(glob.glob(os.path.join(base, "sim_*")) + glob.glob(os.path.join(base, "exp_*")),
                  key=lambda p: os.path.basename(p).split("_", 1)[-1])
    if runs:
        return runs[-1]
    sys.exit(f"No run found in {base}")


MJCF_CANDIDATES = [
    os.environ.get("H1_2_MJCF", ""),
    os.path.join(REPO, "assets", "h1_2", "h1_2_handless.xml"),
]


def estimate_base_height_fk(jpos, roll, pitch):
    """Estimate the base height by FORWARD KINEMATICS -- usable on the REAL ROBOT.

    Inputs: joint angles (encoders) + IMU orientation, nothing else. No absolute position sensor, so
    it is reproducible on board exactly as for the policy, which infers its own height from joint_pos
    in the same way.

    Method: the MuJoCo model is put in the measured pose, with the base at the origin rotated by the
    IMU roll/pitch (YAW does not change a vertical distance), forward kinematics is run and the lowest
    point of the two feet is taken. Assuming that point is on the ground, the base height is its
    height with opposite sign. The minimum over BOTH feet automatically uses the stance foot.

    Accuracy against the simulation ground truth (5486 samples): bias +0.33 cm, std 0.23 cm,
    corr 0.9995 (standing: std 0.07-0.12 cm; walking: 0.26 cm).

    ASSUMPTIONS: flat ground and at least one foot in contact. Degrades on sloped or rough ground.
    Returns None if mujoco or the model are not available.
    """
    try:
        import mujoco
    except ImportError:
        print("  [FK height] mujoco not installed -> estimate not available")
        return None
    path = next((p for p in MJCF_CANDIDATES if p and os.path.exists(p)), None)
    if path is None:
        print("  [FK height] h1_2 MJCF not found (set $H1_2_MJCF) -> estimate not available")
        return None

    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n + "_joint")]
            for n in JOINT_NAMES]
    feet = []
    for side in ("left", "right"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_roll_link")
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == bid and model.geom_contype[g] != 0:
                mid = model.geom_dataid[g]
                a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
                feet.append((g, model.mesh_vert[a:a + n].astype(np.float64)))

    out = np.empty(len(jpos))
    for i in range(len(jpos)):
        cr, sr = np.cos(roll[i] / 2), np.sin(roll[i] / 2)
        cp, sp = np.cos(pitch[i] / 2), np.sin(pitch[i] / 2)
        data.qpos[:] = 0
        data.qpos[3:7] = [cr * cp, sr * cp, cr * sp, -sr * sp]  # (w,x,y,z), yaw=0
        data.qpos[qadr] = jpos[i]
        mujoco.mj_kinematics(model, data)
        lowest = min((V @ data.geom_xmat[g].reshape(3, 3)[2] + data.geom_xpos[g][2]).min()
                     for g, V in feet)
        out[i] = -lowest
    return out


def tilt_from_gravity(obs, lay):
    """Roll/pitch [rad] from projected_gravity (IMU) -- also available on the REAL ROBOT.

    projected_gravity is already an observation of the policy, so nothing more is needed on board.
    Validated against the ground truth of a simulation: RMS 0.06 deg (roll) and 0.03 deg (pitch).
    YAW cannot be obtained from the gravity vector (it is not observable).
    """
    pg = oget(obs, lay, "projected_gravity")
    gx, gy, gz = pg[:, 0], pg[:, 1], pg[:, 2]
    roll = np.arctan2(-gy, -gz)
    pitch = np.arctan2(gx, np.sqrt(gy**2 + gz**2))
    return roll, pitch


def quat_to_rpy(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def jmat(df, suffix, fb_prefix, fb_off):
    """(N,27) matrix in policy order: columns <joint>_<suffix>, with a positional fallback
    <fb_prefix><fb_off+i> for older CSVs.

    For commands.csv / errors.csv, whose columns are always named. For obs.csv use obs_layout()/oget()
    instead: there the positional fallback depends on the history."""
    named = [f"{n}_{suffix}" for n in JOINT_NAMES]
    if all(c in df.columns for c in named):
        return df[named].to_numpy()
    return df[[f"{fb_prefix}{fb_off + i}" for i in range(27)]].to_numpy()


# ---------------------------------------------------------------------------
# Layout of obs.csv
#
# obs.csv has NAMED columns (written by the run logger) or only obs_0..obs_N-1. In the positional
# case the vector is NOT a single frame: with history_length=H the concatenation is PER TERM
# (Isaac Lab, and observation_manager.h with use_gym_history=false), i.e.
#     [ term_1 x H | term_2 x H | ... ]
# and within every term the frames go from the OLDEST to the most RECENT. The real layout is
# rebuilt from params/deploy.yaml of the policy given in run_info.md.
# ---------------------------------------------------------------------------

# default order (h1_2 squat) when deploy.yaml cannot be reached
OBS_TERMS_FALLBACK = [
    ("base_ang_vel", 3), ("projected_gravity", 3), ("velocity_commands", 3),
    ("joint_pos_rel", 27), ("joint_vel_rel", 27), ("last_action", 27),
    ("gait_phase", 2), ("base_height_command", 1),
]

# term -> named columns written by the controller (most recent frame)
NAMED_COLS = {
    "joint_pos_rel": [f"{n}_pos" for n in JOINT_NAMES],
    "joint_vel_rel": [f"{n}_vel" for n in JOINT_NAMES],
    "last_action": [f"{n}_act" for n in JOINT_NAMES],
}


def parse_deploy_obs(path):
    """[(term, dim, history_length)] in concatenation order, from deploy.yaml.

    Minimal regex parser, so that this script does not need PyYAML."""
    m = re.search(r"^observations:\n(.*?)(?=\n\S|\Z)", open(path).read(), re.S | re.M)
    if not m:
        return None
    terms = []
    for block in re.split(r"\n  (?=\S)", "\n" + m.group(1)):
        mn = re.match(r"\s*([A-Za-z_]\w*):", block)
        if not mn or mn.group(1) in ("scale_first", "use_gym_history"):
            continue
        mh = re.search(r"history_length:\s*(\d+)", block)
        ms = re.search(r"scale:\s*(.*?)(?=\n\s{4}\w+:|\Z)", block, re.S)
        dim = None
        if ms and "null" not in ms.group(1).lower():
            dim = len(re.findall(r"-?\d+\.?\d*(?:[eE][-+]?\d+)?", ms.group(1)))
        terms.append((mn.group(1), dim, int(mh.group(1)) if mh else 1))
    return terms or None


def obs_layout(obs, run):
    """dict term -> obs.csv columns of the MOST RECENT frame (+ a note on where the layout comes from)."""
    lay, hist = {}, 1

    # 1) named columns: the controller already resolved the layout
    named_ok = [t for t, cols in NAMED_COLS.items() if all(c in obs.columns for c in cols)]
    for t in named_ok:
        lay[t] = NAMED_COLS[t]
    for t, d in OBS_TERMS_FALLBACK:
        if t in lay:
            continue
        cols = [f"{t}_{k}" for k in range(d)]
        if all(c in obs.columns for c in cols):
            lay[t] = cols
    if len(lay) >= 4:
        return lay, "named columns in obs.csv"

    # 2) positional: rebuild the layout (terms + history) from deploy.yaml
    n_obs = sum(c.startswith("obs_") for c in obs.columns)
    terms, src = None, None
    info_path = os.path.join(run, "run_info.md")
    if os.path.exists(info_path):
        m = re.search(r"-\s+\*\*policy_dir\*\*:\s*(.+)", open(info_path).read())
        if m:
            y = os.path.join(m.group(1).strip(), "params", "deploy.yaml")
            if os.path.exists(y):
                cand = parse_deploy_obs(y)
                if cand and all(d for _, d, _ in cand) and \
                        sum(d * h for _, d, h in cand) == n_obs:
                    terms, src = cand, "deploy.yaml of the policy"
    if terms is None:
        # deploy.yaml missing or not matching: assume the standard order and infer the history
        base = sum(d for _, d in OBS_TERMS_FALLBACK)
        for cut in (len(OBS_TERMS_FALLBACK), len(OBS_TERMS_FALLBACK) - 1):  # with/without squat
            b = sum(d for _, d in OBS_TERMS_FALLBACK[:cut])
            if n_obs % b == 0:
                h = n_obs // b
                terms = [(t, d, h) for t, d in OBS_TERMS_FALLBACK[:cut]]
                src = f"default order (history_length={h} inferred from {n_obs} obs)"
                break
        if terms is None:
            sys.exit(f"Layout of obs.csv not recognised: {n_obs} obs_* columns "
                     f"(expected a multiple of {base} or {base - 1})")

    off = 0
    for name, dim, h in terms:
        hist = max(hist, h)
        # the current frame is the LAST block of the term's history
        lay[name] = [f"obs_{i}" for i in range(off + (h - 1) * dim, off + h * dim)]
        off += dim * h
    return lay, f"{src}, current frame of history_length={hist}"


def oget(obs, lay, term):
    """(N, dim) matrix of an observation term (most recent frame)."""
    return obs[lay[term]].to_numpy()


# families per row (torso excluded: it is the only trunk joint)
FAMILIES = {"legs": ["hip", "knee", "ankle"], "arms": ["shoulder", "elbow", "wrist"]}


def dof_label(name, family):
    s = name.replace("left_", "").replace("right_", "").replace(family, "").strip("_")
    return s if s else family


def plot_part(t, mat, part, title, ylabel, path):
    """Grid of 3 rows (families) x 2 columns (left/right). mat: (N,27)."""
    fams = FAMILIES[part]
    sides = [("left", "LEFT"), ("right", "RIGHT")]
    fig, axes = plt.subplots(len(fams), 2, figsize=(11, 8.5), sharex=True, squeeze=False)
    for r, fam in enumerate(fams):
        for c, (pref, slab) in enumerate(sides):
            ax = axes[r][c]
            for i in range(27):
                nm = JOINT_NAMES[i]
                if nm.startswith(pref) and fam in nm:
                    ax.plot(t, mat[:, i], lw=1.0, label=dof_label(nm, fam))
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.legend(fontsize=8, loc="upper right")
            if r == 0:
                ax.set_title(slab, fontsize=12, fontweight="bold")
            if c == 0:
                ax.set_ylabel(f"{fam}\n{ylabel}", fontsize=9)
    fig.suptitle(title, fontsize=13)
    fig.supxlabel("t [s]", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    run = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else latest_run()
    print(f"== Run: {run}")
    outdir = os.path.join(run, "plots")
    os.makedirs(outdir, exist_ok=True)

    obs = pd.read_csv(os.path.join(run, "obs.csv"))
    cmd = pd.read_csv(os.path.join(run, "commands.csv"))
    err = pd.read_csv(os.path.join(run, "errors.csv"))

    # ground_truth.csv is written ONLY by the MuJoCo simulator: absolute base position, quaternion and
    # linear velocity are NOT measurable on the real robot. In exp_* runs (real robot) the file does not
    # exist, and only the data actually available on board are shown.
    gt_path = os.path.join(run, "ground_truth.csv")
    gt = pd.read_csv(gt_path) if os.path.exists(gt_path) else None
    real_robot = gt is None
    print(f"== Mode: {'REAL ROBOT (no ground truth)' if real_robot else 'MuJoCo simulation'}")

    # run_info.md: written by the controller, tells WHICH POLICY produced this log
    info_path = os.path.join(run, "run_info.md")
    if os.path.exists(info_path):
        info = dict(re.findall(r"-\s+\*\*(\w+)\*\*:\s*(.+)", open(info_path).read()))
        print(f"== Policy: {info.get('policy_name', '?')}   ({info.get('controller', '?')}, "
              f"{info.get('n_obs', '?')} obs, squat {info.get('base_height_range', '?')})")
        print(f"   {info.get('policy_dir', '')}")
    else:
        print("== Policy: UNKNOWN (no run_info.md)")

    leg_idx = [i for i in range(27) if not is_arm(JOINT_NAMES[i])]
    arm_idx = [i for i in range(27) if is_arm(JOINT_NAMES[i])]

    # layout of obs.csv: column names, or indices rebuilt taking the history into account
    lay, lay_src = obs_layout(obs, run)
    print(f"== Layout obs: {lay_src}  ({', '.join(sorted(lay))})")

    # ---- TRUE joint position = joint_pos_rel + default
    t_obs = obs["ep_t"].to_numpy()
    jpos = oget(obs, lay, "joint_pos_rel") + DEFAULT_JOINT_POS  # broadcast over 27 columns
    plot_part(t_obs, jpos, "legs", "Joint position - LEGS", "q [rad]",
              os.path.join(outdir, "01_joint_pos_legs.png"))
    plot_part(t_obs, jpos, "arms", "Joint position - ARMS", "q [rad]",
              os.path.join(outdir, "02_joint_pos_arms.png"))

    # ---- position error (q_target - q_measured) from errors.csv
    t_err = err["ep_t"].to_numpy()
    qerr = jmat(err, "q_err", "q_err_", 0)
    plot_part(t_err, qerr, "legs", "Joint position error - LEGS", "err [rad]",
              os.path.join(outdir, "03_joint_err_legs.png"))
    plot_part(t_err, qerr, "arms", "Joint position error - ARMS", "err [rad]",
              os.path.join(outdir, "04_joint_err_arms.png"))

    # ---- policy commands q_target from commands.csv
    t_cmd = cmd["ep_t"].to_numpy()
    qtgt = jmat(cmd, "q_target", "q_target_", 0)
    plot_part(t_cmd, qtgt, "legs", "Policy commands q_target - LEGS", "q_target [rad]",
              os.path.join(outdir, "06_cmd_legs.png"))
    plot_part(t_cmd, qtgt, "arms", "Policy commands q_target - ARMS", "q_target [rad]",
              os.path.join(outdir, "07_cmd_arms.png"))

    # ---- base pose
    if real_robot:
        # Real robot: absolute position and yaw are NOT measurable; roll and pitch come from
        # projected_gravity (IMU), already an observation of the policy.
        roll_i, pitch_i = tilt_from_gravity(obs, lay)
        fig, ax = plt.subplots(1, 1, figsize=(11, 4.5))
        ax.plot(t_obs, np.degrees(roll_i), label="roll (IMU)")
        ax.plot(t_obs, np.degrees(pitch_i), label="pitch (IMU)")
        ax.axhline(0, ls="--", lw=0.8, color="gray")
        ax.set_ylabel("tilt [deg]"); ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend()
        ax.set_title("Base tilt from projected_gravity (IMU)\n"
                     "yaw and XYZ position are not observable on board")
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "05_base_tilt_imu.png"), dpi=110); plt.close(fig)
    else:
        t_gt = gt["sim_t"].to_numpy()
        q = gt[["qw", "qx", "qy", "qz"]].to_numpy()
        roll, pitch, yaw = quat_to_rpy(q)
        fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        ax[0].plot(t_gt, gt["base_x"], label="x")
        ax[0].plot(t_gt, gt["base_y"], label="y")
        ax[0].plot(t_gt, gt["base_z"], label="z")
        ax[0].set_ylabel("position [m]"); ax[0].grid(alpha=0.3); ax[0].legend()
        ax[0].set_title("XYZ position of the base")
        ax[1].plot(t_gt, np.degrees(roll), label="roll")
        ax[1].plot(t_gt, np.degrees(pitch), label="pitch")
        ax[1].plot(t_gt, np.degrees(yaw), label="yaw")
        ax[1].set_ylabel("orientation [deg]"); ax[1].set_xlabel("t [s]"); ax[1].grid(alpha=0.3); ax[1].legend()
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "05_base_pose.png"), dpi=110); plt.close(fig)

    # ---- SQUAT: commanded vs true height
    # base_height_command (obs) = target - nominal (0 = standing, <0 = squat). The TRUE height is in
    # ground_truth.base_z, on another clock: it is aligned to obs through the shared wall_t.
    if "base_height_command" in lay:
        NOMINAL_H = 0.98
        bh_cmd = oget(obs, lay, "base_height_command")[:, 0]
        target_h = NOMINAL_H + bh_cmd
        squatting = bh_cmd < -0.03
        roll_e, pitch_e = tilt_from_gravity(obs, lay)
        h_fk = estimate_base_height_fk(jpos, roll_e, pitch_e)  # None if mujoco or the MJCF are missing
        if real_robot:
            # The TRUE height is not measurable on board, but it can be ESTIMATED by forward kinematics
            # (encoders + IMU): the same information the policy infers its own height from.
            fig, axs = plt.subplots(2 if h_fk is not None else 1, 1, figsize=(11, 7 if h_fk is not None else 4.5),
                                    sharex=True, squeeze=False)
            a0 = axs[0][0]
            a0.plot(t_obs, target_h, label="commanded height (target)", lw=1.6, color="tab:orange")
            if h_fk is not None:
                a0.plot(t_obs, h_fk, label="ESTIMATED height (FK: encoders + IMU)", lw=1.2, color="tab:green")
            a0.axhline(NOMINAL_H, ls="--", lw=0.8, color="gray", label="nominal 0.98")
            a0.set_ylabel("height [m]"); a0.grid(alpha=0.3); a0.legend(fontsize=9)
            a0.set_title("Squat: commanded vs estimated height (no direct measurement on board)")
            if h_fk is not None:
                herr = h_fk - target_h
                a1 = axs[1][0]
                a1.plot(t_obs, herr * 100, lw=1.0, color="tab:red")
                a1.axhline(0, ls="--", lw=0.8, color="gray")
                a1.set_ylabel("estimated error [cm]"); a1.grid(alpha=0.3)
                if squatting.any():
                    a1.set_title(f"estimated |err| in squat: mean={np.abs(herr[squatting]).mean()*100:.2f}cm  "
                                 f"max={np.abs(herr[squatting]).max()*100:.2f}cm   "
                                 f"(FK estimate: bias +0.3cm, std 0.2cm vs simulation ground truth)")
            axs[-1][0].set_xlabel("t [s]")
        else:
            base_z_on_obs = np.interp(obs["wall_t"].to_numpy(), gt["wall_t"].to_numpy(), gt["base_z"].to_numpy())
            herr = base_z_on_obs - target_h
            fig, axs2 = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
            axs2[0].plot(t_obs, target_h, label="commanded height (target)", lw=1.6, color="tab:orange")
            axs2[0].plot(t_obs, base_z_on_obs, label="true height base_z (gt)", lw=1.2, color="tab:blue")
            if h_fk is not None:
                # cross-check: the same FK estimate used on the real robot, to see its accuracy
                axs2[0].plot(t_obs, h_fk, label="FK estimate (as on the real robot)", lw=1.0,
                             ls="--", color="tab:green", alpha=0.8)
            axs2[0].axhline(NOMINAL_H, ls="--", lw=0.8, color="gray", label="nominal 0.98")
            axs2[0].set_ylabel("height [m]"); axs2[0].grid(alpha=0.3); axs2[0].legend(fontsize=9)
            axs2[0].set_title("Squat: commanded vs true height")
            axs2[1].plot(t_obs, herr * 100, lw=1.0, color="tab:red")
            axs2[1].axhline(0, ls="--", lw=0.8, color="gray")
            axs2[1].set_ylabel("height error [cm]"); axs2[1].set_xlabel("t [s]"); axs2[1].grid(alpha=0.3)
            if squatting.any():
                axs2[1].set_title(f"|err| in squat: mean={np.abs(herr[squatting]).mean()*100:.2f}cm  max={np.abs(herr[squatting]).max()*100:.2f}cm")
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "09_squat_height.png"), dpi=110); plt.close(fig)

    # ---- all the other observations
    has_squat = "base_height_command" in lay
    nrows = 4 if has_squat else 3
    fig, ax = plt.subplots(nrows, 2, figsize=(13, 3 * nrows))
    ax = ax.ravel()
    bav = oget(obs, lay, "base_ang_vel")
    pg = oget(obs, lay, "projected_gravity")
    vc = oget(obs, lay, "velocity_commands")
    gp = oget(obs, lay, "gait_phase")
    jv = oget(obs, lay, "joint_vel_rel")
    la = oget(obs, lay, "last_action")
    for k, lab in zip(range(3), ["wx", "wy", "wz"]):
        ax[0].plot(t_obs, bav[:, k], label=lab)
    ax[0].set_title("base_ang_vel (scaled 0.2)"); ax[0].legend(fontsize=7)
    for k, lab in zip(range(3), ["gx", "gy", "gz"]):
        ax[1].plot(t_obs, pg[:, k], label=lab)
    ax[1].set_title("projected_gravity"); ax[1].legend(fontsize=7)
    for k, lab in zip(range(3), ["vx", "vy", "wz"]):
        ax[2].plot(t_obs, vc[:, k], label=lab)
    ax[2].set_title("velocity_commands"); ax[2].legend(fontsize=7)
    ax[3].plot(t_obs, gp); ax[3].set_title("gait_phase (sin, cos)")
    ax[4].plot(t_obs, jv, lw=0.6); ax[4].set_title("joint_vel_rel (27, scaled 0.05)")
    ax[5].plot(t_obs, la, lw=0.6); ax[5].set_title("last_action (27)")
    if has_squat:
        bhc = oget(obs, lay, "base_height_command")[:, 0]
        ax[6].plot(t_obs, bhc, color="tab:orange"); ax[6].axhline(0, ls="--", lw=0.8, color="gray")
        ax[6].set_title("base_height_command (target - nominal) [m]")
        # panel 8: absolute height command, for readability
        ax[7].plot(t_obs, 0.98 + bhc, color="tab:orange")
        ax[7].axhline(0.98, ls="--", lw=0.8, color="gray")
        ax[7].set_title("absolute commanded height [m]")
    for a in ax:
        a.grid(alpha=0.3); a.tick_params(labelsize=7); a.set_xlabel("t [s]", fontsize=8)
    fig.suptitle("Other observations", fontsize=12)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "08_other_obs.png"), dpi=110); plt.close(fig)

    # ---- metrics on screen
    if real_robot:
        roll_i, pitch_i = tilt_from_gravity(obs, lay)
        print("\n== Stability (from IMU: projected_gravity)")
        print(f"  |roll| max = {np.max(np.abs(np.degrees(roll_i))):.1f} deg  |pitch| max = {np.max(np.abs(np.degrees(pitch_i))):.1f} deg")
        print(f"  roll std = {np.degrees(roll_i.std()):.2f} deg   pitch std = {np.degrees(pitch_i.std()):.2f} deg")
        print("  (base height, xy drift and yaw are not measurable on the real robot)")
    else:
        print("\n== Stability (ground_truth)")
        print(f"  mean base height = {gt['base_z'].mean():.3f} m (std {gt['base_z'].std():.4f})")
        print(f"  |roll| max = {np.max(np.abs(np.degrees(roll))):.1f} deg  |pitch| max = {np.max(np.abs(np.degrees(pitch))):.1f} deg")
        print(f"  xy drift = {np.hypot(gt['base_x'].iloc[-1]-gt['base_x'].iloc[0], gt['base_y'].iloc[-1]-gt['base_y'].iloc[0]):.2f} m")
    print("== Joint tracking (q_target - q_measured)")
    print(f"  LEGS : |err| mean={np.abs(qerr[:, leg_idx]).mean():.4f}  max={np.abs(qerr[:, leg_idx]).max():.4f} rad")
    print(f"  ARMS : |err| mean={np.abs(qerr[:, arm_idx]).mean():.4f}  max={np.abs(qerr[:, arm_idx]).max():.4f} rad")

    # ---- SQUAT + command tracking (obs aligned to gt through wall_t)
    if "base_height_command" in lay:
        bh_cmd = oget(obs, lay, "base_height_command")[:, 0]
        vcmd = oget(obs, lay, "velocity_commands")
        walking = np.linalg.norm(vcmd, axis=1) > 0.1
        squatting = bh_cmd < -0.03
        standing = (~walking) & (~squatting)
        target_h = 0.98 + bh_cmd
        print("== Squat / height")
        print(f"  min commanded height = {target_h.min():.3f} m  |  phases: walking {100*walking.mean():.0f}% / squat {100*squatting.mean():.0f}% / standing {100*standing.mean():.0f}%")
        if real_robot:
            roll_m, pitch_m = tilt_from_gravity(obs, lay)
            h_est = estimate_base_height_fk(jpos, roll_m, pitch_m)
            if h_est is None:
                print("  (actual height not available: no direct measurement and no FK estimate)")
            else:
                herr_e = h_est - target_h
                print("  height ESTIMATED by forward kinematics (encoders + IMU; no direct measurement on board)")
                print(f"  [accuracy of the estimate against the simulation ground truth: bias +0.33 cm, std 0.23 cm]")
                for lbl, mk in [("standing", standing), ("squat", squatting)]:
                    if mk.any():
                        print(f"  {lbl}: |height err| mean={np.abs(herr_e[mk]).mean()*100:.2f}cm  max={np.abs(herr_e[mk]).max()*100:.2f}cm")
        else:
            base_z_on_obs = np.interp(obs["wall_t"].to_numpy(), gt["wall_t"].to_numpy(), gt["base_z"].to_numpy())
            herr = base_z_on_obs - target_h
            for lbl, m in [("standing", standing), ("squat", squatting)]:
                if m.any():
                    print(f"  {lbl}: |height err| mean={np.abs(herr[m]).mean()*100:.2f}cm  max={np.abs(herr[m]).max()*100:.2f}cm")
        # sway per phase (base_ang_vel wx,wy)
        bav = oget(obs, lay, "base_ang_vel")
        print("== Sway base_ang_vel [rad/s] per phase")
        for lbl, m in [("walking", walking), ("standing", standing), ("squat", squatting)]:
            if m.sum() > 20:
                print(f"  {lbl}: wx std={bav[m,0].std():.3f}  wy std={bav[m,1].std():.3f}  |wxy| max={np.hypot(bav[m,0],bav[m,1]).max():.3f}")

    print(f"\n== {len([f for f in os.listdir(outdir) if f.endswith('.png')])} plots saved in {outdir}")


if __name__ == "__main__":
    main()
