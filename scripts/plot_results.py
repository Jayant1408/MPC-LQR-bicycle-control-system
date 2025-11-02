import argparse, os, math
import matplotlib.pyplot as plt
import numpy as np

def load_csv(path):
    data = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            data.append([float(x) for x in line.split(',')])
    return np.array(data, dtype=float)

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def rmse(z):
    return float(np.sqrt(np.mean(z**2)))

def main():
    p = argparse.ArgumentParser(description='Plot results of the simulation')
    p.add_argument("--ref", default="output_ref.csv", help='Path to the reference trajectory CSV file')
    p.add_argument("--traj", default="output_lqr.csv", help='Path to the LQR trajectory CSV file')
    p.add_argument("--outdir", default = "plots", help='Plot the results and save in the directory')
    args = p.parse_args()

    ref = load_csv(args.ref) #[t,x_r,y_r,yaw_r, v_r]
    traj = load_csv(args.traj) #[t,x,y,yaw, v, a, delta]
    if ref.shape[1] < 5 or traj.shape[1] < 7:
        raise ValueError("Unexpected CSV Format for ref/traj-check column counts.")

    n =  min(len(ref), len(traj))
    ref, traj  = ref[:n], traj[:n]

    t = ref[:, 0]
    x_r, y_r, yaw_r, v_r = ref[:, 1], ref[:, 2], ref[:, 3], ref[:, 4]
    x, y, yaw, v, a, delta = traj[:, 1], traj[:, 2], traj[:, 3], traj[:, 4], traj[:, 5], traj[:, 6]

    ex = x_r - x
    ey = y_r - y
    e_yaw = np.array([wrap_to_pi(yaw_r[i] - yaw[i]) for i in range(n)])
    ev = v_r - v

    metrics = {
        "rmse_ex": rmse(ex),
        "rmse_ey": rmse(ey),
        "rmse_pos": rmse(np.sqrt(ex**2 + ey**2)),
        "rmse_e_yaw_rad": rmse(e_yaw),
        "rmse_ev": rmse(ev),
        "mean_speed": float(np.mean(v)),
        "mean_accel": float(np.mean(a)),
        "mean_steer_rad": float(np.mean(delta)),
    }

    os.makedirs(args.outdir, exist_ok=True)

     # 1) XY trajectory
    plt.figure()
    plt.plot(x_r, y_r, label="reference")
    plt.plot(x, y, label="actual")
    plt.axis("equal")
    plt.xlabel("x [m]"); plt.ylabel("y [m]"); plt.title("Trajectory (XY)"); plt.legend()
    plt.savefig(os.path.join(args.outdir, "trajectory_xy.png"), dpi=160, bbox_inches="tight")

    # 2) Speed vs time
    plt.figure()
    plt.plot(t, v_r, label="v_ref")
    plt.plot(t, v, label="v")
    plt.xlabel("time [s]"); plt.ylabel("speed [m/s]"); plt.title("Speed vs Time"); plt.legend()
    plt.savefig(os.path.join(args.outdir, "speed_time.png"), dpi=160, bbox_inches="tight")

    # 3) Acceleration vs time
    plt.figure()
    plt.plot(t, a, label="a")
    plt.xlabel("time [s]"); plt.ylabel("accel [m/s^2]"); plt.title("Acceleration vs Time"); plt.legend()
    plt.savefig(os.path.join(args.outdir, "accel_time.png"), dpi=160, bbox_inches="tight")

    # 4) Steering vs time
    plt.figure()
    plt.plot(t, delta, label="delta")
    plt.xlabel("time [s]"); plt.ylabel("steer [rad]"); plt.title("Steering vs Time"); plt.legend()
    plt.savefig(os.path.join(args.outdir, "steering_time.png"), dpi=160, bbox_inches="tight")

    # 5) Errors vs time
    plt.figure()
    plt.plot(t, ex, label="e_x")
    plt.plot(t, ey, label="e_y")
    plt.plot(t, e_yaw, label="e_yaw [rad]")
    plt.plot(t, ev, label="e_v")
    plt.xlabel("time [s]"); plt.ylabel("error"); plt.title("Tracking Errors vs Time"); plt.legend()
    plt.savefig(os.path.join(args.outdir, "errors_time.png"), dpi=160, bbox_inches="tight")


    with open(os.path.join(args.outdir, "metrics.txt"), "w") as f:
        for k, v_ in metrics.items():
            f.write(f"{k}: {v_}\n")

    print("Saved plots + rmse_summary.txt in", args.outdir)

if __name__ == "__main__":
    main()

