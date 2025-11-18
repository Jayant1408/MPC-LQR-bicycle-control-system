#!/usr/bin/env python3
"""
compare_rmse.py
Compute RMSE of tracking for LQR and/or MPC against the same reference.

Inputs:
  --ref  : output_ref.csv   (t, x_r, y_r, yaw_r, v_r)
  --lqr  : output_lqr.csv   (t, x, y, yaw, v, a, delta)   [optional]
  --mpc  : output_mpc.csv   (t, x, y, yaw, v, a, delta)   [optional]

Outputs:
  - Prints an RMSE table to console
  - Writes metrics to metrics_rmse.csv and metrics_rmse.md
"""

import argparse, os, math
import numpy as np

def load_csv(path):
    data = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line: continue
            data.append([float(x) for x in line.split(",")])
    return np.asarray(data, dtype = float)

def wrap_to_pi(a):
    while a > math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

def compute_errors(ref, traj):
    """
    Compute errors in the reference frame for better interpretability:
    e_lon, e_lat, e_yaw, ev
    """
    n = min(len(ref), len(traj))
    ref = ref[:n]; traj = traj[:n]
    t = ref[:, 0]
    xr, yr, yawr, vr = ref[:,1], ref[:,2], ref[:,3], ref[:,4]
    x, y, yaw, v = traj[:,1], traj[:,2], traj[:,3], traj[:,4]

    # rotate position error into reference frame
    dx = x - xr
    dy = y - yr
    cosr = np.cos(yawr)
    sinr = np.sin(yawr)
    e_lon = cosr * dx + sinr * dy
    e_lat = -sinr * dx + cosr * dy
    e_yaw = np.array([wrap_to_pi(yaw[i] - yawr[i]) for i in range(n)])
    ev = v - vr
    return t, e_lon, e_lat, e_yaw,ev

def rmse(z): return float(np.sqrt(np.mean(np.asarray(z)**2)))

def summarize(name, ref, traj):
    t, e_lon, e_lat, e_yaw, ev = compute_errors(ref, traj)
    pos = np.sqrt(e_lon**2 + e_lat**2)
    return {
        "controller": name, 
        "samples": len(t),
        "rmse_e_lon_m": rmse(e_lon),
        "rmse_e_lat_m": rmse(e_lat),
        "rmse_pos_m":   rmse(pos),
        "rmse_e_yaw_rad": rmse(e_yaw),
        "rmse_ev_mps": rmse(ev),
        "mean_speed_mps": float(np.mean(traj[:len(t),4])),
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="output_ref.csv")
    ap.add_argument("--lqr", default="output_lqr.csv")
    ap.add_argument("--mpc", default="output_mpc.csv")
    ap.add_argument("--out_csv", default="metrics_rmse.csv")
    ap.add_argument("--out_md",  default="metrics_rmse.md")
    args = ap.parse_args()

    ref = load_csv(args.ref)

    rows = []
    if os.path.exists(args.lqr):
        rows.append(summarize("LQR", ref, load_csv(args.lqr)))
    if os.path.exists(args.mpc):
        rows.append(summarize("MPC", ref, load_csv(args.mpc)))

    if not rows:
        raise SystemExit("No controller trajectories found (expected output_lqr.csv and/or output_mpc.csv).")

    # Print nice table
    headers = ["controller","samples","rmse_e_lon_m","rmse_e_lat_m","rmse_pos_m","rmse_e_yaw_rad","rmse_ev_mps","mean_speed_mps"]
    col_w = {h:max(len(h), max(len(f"{r[h]:.4f}") if isinstance(r[h], float) else len(str(r[h])) for r in rows)) for h in headers}
    line = " | ".join(h.ljust(col_w[h]) for h in headers)
    sep  = "-+-".join("-"*col_w[h] for h in headers)
    print(line); print(sep)
    for r in rows:
        def fmt(v): return f"{v:.4f}" if isinstance(v, float) else str(v)
        print(" | ".join(fmt(r[h]).ljust(col_w[h]) for h in headers))

    # Save CSV
    with open(args.out_csv,"w") as f:
        f.write(",".join(headers)+"\n")
        for r in rows:
            vals = [str(r[h]) if not isinstance(r[h], float) else f"{r[h]:.6f}" for h in headers]
            f.write(",".join(vals)+"\n")

    # Save Markdown
    with open(args.out_md,"w") as f:
        f.write("| " + " | ".join(headers) + " |\n")
        f.write("|" + "|".join("---" for _ in headers) + "|\n")
        for r in rows:
            def fmt(v): return f"{v:.6f}" if isinstance(v, float) else str(v)
            f.write("| " + " | ".join(fmt(r[h]) for h in headers) + " |\n")

    print(f"\nWrote {args.out_csv} and {args.out_md}")

if __name__ == "__main__":
    main()