#!/usr/bin/env python3
"""
mpc_cvxpy.py  —  Closed-loop MPC prototype for the kinematic bicycle project.

Key points:
- Error state is in the reference (path-tangent) frame: e = [e_lon, e_lat, e_yaw, ev].
- Discrete linear model around v_ref:
    e_lon+  = e_lon + ev*dt
    e_lat+  = e_lat + v_ref*e_yaw*dt
    e_yaw+  = e_yaw + (v_ref/L)*delta*dt
    ev+     = ev + a*dt
- Feed-forward steering from path curvature is INCLUDED INSIDE THE QP as U_ff,
  and the optimizer penalizes (U_total − U_ff) and its rate. Apply U_total directly.
- Simulates the nonlinear plant for ground truth, writes output_mpc.csv,
  and makes basic plots (optionally overlays output_lqr.csv if present).

Install:
  pip install cvxpy osqp numpy matplotlib

Run:
  python scripts/mpc_cvxpy.py --ref output_ref.csv --out output_mpc.csv --plots plots_mpc
"""

import argparse, os, math
import numpy as np
import matplotlib.pyplot as plt

try:
    import cvxpy as cp
except Exception:
    print("cvxpy not available. Install with: pip install cvxpy osqp")
    raise

# --------------------------- helpers ---------------------------

def wrap_to_pi(a: float) -> float:
    while a > math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

def load_csv(path):
    data = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line: continue
            data.append([float(x) for x in line.split(",")])
    return np.asarray(data, dtype=float)

def bicycle_step(state, u, L, dt, limits):
    """Nonlinear kinematic plant step used for closed-loop simulation."""
    x, y, yaw, v = state
    a, delta = u

    # clamp inputs
    delta = max(-limits["max_steer"], min(limits["max_steer"], delta))
    a     = max(-limits["max_accel"], min(limits["max_accel"], a))

    x   += v * math.cos(yaw) * dt
    y   += v * math.sin(yaw) * dt
    yaw += (v / L) * math.tan(delta) * dt
    v   += a * dt

    v = max(-limits["max_speed"], min(limits["max_speed"], v))
    return np.array([x, y, yaw, v], dtype=float), np.array([a, delta], dtype=float)

def ref_frame_error(state, ref):
    """Compute e = [e_lon, e_lat, e_yaw, ev] in the reference frame at 'ref'."""
    x, y, yaw, v = state
    xr, yr, yawr, vr = ref
    dx = x - xr
    dy = y - yr
    cosr = math.cos(yawr); sinr = math.sin(yawr)
    e_lon =  cosr*dx + sinr*dy
    e_lat = -sinr*dx + cosr*dy
    e_yaw = wrap_to_pi(yaw - yawr)
    ev    = v - vr
    return np.array([e_lon, e_lat, e_yaw, ev], dtype=float)

def curvature_ff(yaw_now, yaw_next, v_ref, L, dt):
    """Feed-forward steering from curvature κ ≈ Δψ / (v_ref*dt)."""
    dyaw = wrap_to_pi(yaw_next - yaw_now)
    ds = max(1e-6, v_ref * dt)
    kappa = dyaw / ds
    return math.atan(L * kappa)

def build_AB(dt, L, v_ref):
    """Linearized discrete model (reference-frame errors)."""
    A = np.eye(4)
    A[0,3] = dt           # e_lon <- ev
    A[1,2] = v_ref * dt   # e_lat <- e_yaw
    B = np.zeros((4,2))
    B[2,1] = (v_ref / L) * dt  # e_yaw <- delta
    B[3,0] = dt                # ev <- a
    return A, B

# --------------------------- MPC runner ---------------------------

def run_mpc(args):
    # Load reference: [t, x_r, y_r, yaw_r, v_r]
    ref = load_csv(args.ref)
    t = ref[:,0]
    xr, yr, yawr, vr = ref[:,1], ref[:,2], ref[:,3], ref[:,4]
    n_steps = len(t)

    # Inherit dt from ref if not given
    dt = args.dt if args.dt is not None else (float(t[1]-t[0]) if n_steps > 1 else 0.05)
    L  = args.L
    v_ref = args.v_ref if args.v_ref is not None else float(np.mean(vr))
    N = args.N

    limits = dict(max_steer=args.max_steer, max_accel=args.max_accel, max_speed=args.max_speed)
    A, B = build_AB(dt, L, v_ref)

    # Costs
    Q  = np.diag([args.Qx, args.Qy, args.Qyaw, args.Qv])
    R  = np.diag([args.Ra, args.Rdelta])
    Rd = np.diag([args.Rda, args.Rdd])

    # Start on reference
    s = np.array([xr[0], yr[0], yawr[0], v_ref], dtype=float)
    u_prev = np.zeros(2)
    out = []  # rows: [t, x, y, yaw, v, a, delta]

    # Optional LQR overlay
    traj_lqr = load_csv(args.traj_lqr) if os.path.exists(args.traj_lqr) else None

    for k in range(n_steps - 1):
        rk = np.array([xr[k], yr[k], yawr[k], vr[k]])
        ek = ref_frame_error(s, rk)

        # ---- Precompute feed-forward steering along the horizon ----
        Uff = np.zeros((2, N))  # [a_ff, delta_ff]
        for i in range(N):
            j = min(k + i, n_steps - 1)
            yaw_now  = yawr[j]
            yaw_next = yawr[min(j + 1, n_steps - 1)]
            Uff[0, i] = 0.0  # accel feed-forward (keep 0 unless using a varying v_ref profile)
            Uff[1, i] = curvature_ff(yaw_now, yaw_next, v_ref, L, dt)

        # Decision variables are TOTAL inputs that will hit the plant
        X = cp.Variable((4, N+1))
        U = cp.Variable((2, N))  # total inputs

        cost = 0
        constr = [X[:, 0] == ek]

        for i in range(N):
            # Track around feed-forward: penalize (U - Uff) and its rate
            cost += cp.quad_form(X[:, i], Q) + cp.quad_form(U[:, i] - Uff[:, i], R)

            if i == 0:
                du = (U[:, 0] - Uff[:, 0]) - (u_prev - Uff[:, 0])
            else:
                du = (U[:, i] - Uff[:, i]) - (U[:, i-1] - Uff[:, i-1])
            cost += cp.quad_form(du, Rd)

            # Linear error dynamics with the total input U
            constr += [X[:, i+1] == A @ X[:, i] + B @ U[:, i]]

            # Input limits and (optional) rate limits
            constr += [
                -limits["max_accel"] <= U[0, i], U[0, i] <= limits["max_accel"],
                -limits["max_steer"] <= U[1, i], U[1, i] <= limits["max_steer"],
            ]
            if args.max_da > 0:
                prev_a = u_prev[0] if i == 0 else U[0, i-1]
                constr += [ -args.max_da <= U[0, i] - prev_a, U[0, i] - prev_a <= args.max_da ]
            if args.max_dd > 0:
                prev_d = u_prev[1] if i == 0 else U[1, i-1]
                constr += [ -args.max_dd <= U[1, i] - prev_d, U[1, i] - prev_d <= args.max_dd ]

        # Terminal cost
        cost += cp.quad_form(X[:, N], Q)

        prob = cp.Problem(cp.Minimize(cost), constr)
        prob.solve(solver=cp.OSQP, eps_abs=1e-4, eps_rel=1e-4, polish=True,
                   max_iter=10000, verbose=False)
        if U.value is None:
            raise RuntimeError(f"MPC failed at step {k}")

        # Apply first total input directly (no extra +delta_ff afterward)
        u_cmd = np.array(U.value[:, 0]).reshape(2)
        s, u_sat = bicycle_step(s, u_cmd, L, dt, limits)

        out.append([t[k], s[0], s[1], s[2], s[3], u_sat[0], u_sat[1]])
        u_prev = u_sat

    out = np.asarray(out, dtype=float)
    np.savetxt(args.out, out, fmt="%.6f", delimiter=",")
    print("Saved MPC trajectory to", args.out)

    # --------------------------- Plots ---------------------------
    os.makedirs(args.plots, exist_ok=True)

    # XY
    plt.figure()
    plt.plot(xr, yr, label="reference")
    plt.plot(out[:,1], out[:,2], label="mpc")
    if traj_lqr is not None:
        plt.plot(traj_lqr[:len(out),1], traj_lqr[:len(out),2], label="lqr")
    plt.axis("equal"); plt.xlabel("x [m]"); plt.ylabel("y [m]"); plt.title("XY Trajectory"); plt.legend()
    plt.savefig(os.path.join(args.plots, "xy_mpc.png"), dpi=160, bbox_inches="tight")

    # Speed
    plt.figure()
    plt.plot(t[:len(out)], vr[:len(out)], label="v_ref")
    plt.plot(t[:len(out)], out[:,4], label="v_mpc")
    if traj_lqr is not None:
        plt.plot(traj_lqr[:len(out),0], traj_lqr[:len(out),4], label="v_lqr")
    plt.xlabel("time [s]"); plt.ylabel("speed [m/s]"); plt.title("Speed"); plt.legend()
    plt.savefig(os.path.join(args.plots, "speed_mpc.png"), dpi=160, bbox_inches="tight")

    # Steering
    plt.figure()
    plt.plot(t[:len(out)], out[:,6], label="delta_mpc")
    if traj_lqr is not None:
        plt.plot(traj_lqr[:len(out),0], traj_lqr[:len(out),6], label="delta_lqr")
    plt.xlabel("time [s]"); plt.ylabel("steer [rad]"); plt.title("Steering"); plt.legend()
    plt.savefig(os.path.join(args.plots, "steer_mpc.png"), dpi=160, bbox_inches="tight")

    # Acceleration
    plt.figure()
    plt.plot(t[:len(out)], out[:,5], label="a_mpc")
    if traj_lqr is not None:
        plt.plot(traj_lqr[:len(out),0], traj_lqr[:len(out),5], label="a_lqr")
    plt.xlabel("time [s]"); plt.ylabel("accel [m/s^2]"); plt.title("Acceleration"); plt.legend()
    plt.savefig(os.path.join(args.plots, "accel_mpc.png"), dpi=160, bbox_inches="tight")

    print("Saved plots in", args.plots)

# --------------------------- CLI ---------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="output_ref.csv", help="reference CSV path")
    ap.add_argument("--traj_lqr", default="output_lqr.csv", help="optional LQR traj for overlay")
    ap.add_argument("--out", default="output_mpc.csv", help="output MPC CSV path")
    ap.add_argument("--plots", default="plots_mpc", help="plots output directory")

    # Vehicle/sim
    ap.add_argument("--dt", type=float, default=None, help="dt [s] (default: infer from ref)")
    ap.add_argument("--L", type=float, default=2.7, help="wheelbase [m]")
    ap.add_argument("--v_ref", type=float, default=None, help="ref speed [m/s] (default: mean of ref v)")

    # Limits
    ap.add_argument("--max_steer", type=float, default=0.6, help="max steer [rad]")
    ap.add_argument("--max_accel", type=float, default=2.5, help="max accel [m/s^2]")
    ap.add_argument("--max_speed", type=float, default=20.0, help="max speed [m/s]")

    # Rate limits (per step); 0 disables
    ap.add_argument("--max_dd", type=float, default=0.2, help="max steer rate [rad/step]")
    ap.add_argument("--max_da", type=float, default=0.5, help="max accel rate [m/s^2/step]")

    # Costs
    ap.add_argument("--Qx", type=float, default=6.0)
    ap.add_argument("--Qy", type=float, default=8.0)   # a bit higher lateral weight for tight tracking
    ap.add_argument("--Qyaw", type=float, default=2.0)
    ap.add_argument("--Qv", type=float, default=0.5)
    ap.add_argument("--Ra", type=float, default=0.5)
    ap.add_argument("--Rdelta", type=float, default=0.2)
    ap.add_argument("--Rda", type=float, default=0.05, help="rate penalty accel")
    ap.add_argument("--Rdd", type=float, default=0.10, help="rate penalty steer")

    ap.add_argument("--N", type=int, default=25, help="MPC horizon")

    args = ap.parse_args()
    run_mpc(args)
