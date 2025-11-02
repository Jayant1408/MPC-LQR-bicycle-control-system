#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include "bicycle_model.hpp"
#include "lqr.hpp"
#include "mpc.hpp"
#include "trajectory.hpp"
#include "util.hpp"

/**
 * Helper function to normalize an angle to the range [-pi, pi]
 * @param a The angle to normalize
 * @return The normalized angle
 */
static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/**
 * Main function to run the simulation
 * @param argc The number of command line arguments
 * @param argv The command line arguments
 * @return The exit status
 */
int main(int argc, char** argv)
{
    std::string cfg_path = (argc > 1) ? std::string(argv[1]) : std::string("");

    // Load configuration from file
    SimConfig cfg;
    if(!loadConfig(cfg_path, cfg))
    {
        // If the configuration file is not found, use default values
        std::cerr << "Failed to load config. Using defaults.\n";
    }

    // Build the vehicle model
    BicycleModel model(cfg.veh, cfg.dt);

    // Reference trajectory
    // Generate the reference trajectory based on the configuration
    std::vector<RefPoint> ref;
    // Generate a circle trajectory
    if(cfg.traj_type == "circle") ref = Trajectory::circle(cfg.radius, cfg.v_ref, cfg.dt, cfg.steps);
    // Generate a lane change trajectory
    else if (cfg.traj_type == "lanechange") ref = Trajectory::laneChange(cfg.length, cfg.v_ref, cfg.dt, cfg.steps);
    // Generate a figure 8 trajectory
    else ref = Trajectory::figure8(cfg.radius, cfg.v_ref, cfg.dt, cfg.steps);

    // Controllers
    LQRController lqr;
    // Build the linear model for the LQR controller
    lqr.buildLinearModel(cfg.dt, cfg.veh.L, cfg.v_ref);
    // Set the weights for the LQR controller
    Eigen::Matrix4d Ql = Eigen::Matrix4d::Zero();
    Ql(0,0) = cfg.Qx; Ql(1,1) = cfg.Qy; Ql(2,2) = cfg.Qyaw; Ql(3,3) = cfg.Qv;  // states weights
    Eigen::Matrix2d Rl = Eigen::Matrix2d::Zero();
    Rl(0,0) = cfg.Ra; Rl(1,1) = cfg.Rdelta;  // inputs weights
    lqr.setWeights(Ql, Rl);
    // Solve the LQR Riccati equation
    bool ok = lqr.solve();
    // If the LQR Riccati equation did not converge, print a warning
    if (!ok) std::cerr << "[WARN] LQR Riccati did not converge; using zeros.\n";

    // Build the MPC controller
    MPCController mpc;
    // Configure the MPC controller
    MPCConfig mc;
    // Set the horizon for the MPC controller
    mc.N = cfg.N;
    // Set the state weights for the MPC controller
    Eigen::Matrix4d Qm = Eigen::Matrix4d::Zero();
    Qm(0,0) = cfg.Qmx; Qm(1,1) = cfg.Qmy; Qm(2,2) = cfg.Qmyaw; Qm(3,3) = cfg.Qmv;
    mc.Q = Qm;
    // Set the control weights for the MPC controller
    Eigen::Matrix2d Rm = Eigen::Matrix2d::Zero();
    Rm(0,0) = cfg.Rma; Rm(1,1) = cfg.Rmdelta;
    mc.R = Rm;
    // Set the rate penalty for the MPC controller
    mc.Rd = Eigen::Matrix2d::Identity() * 0.0;
    mc.v_ref = cfg.v_ref; mc.L = cfg.veh.L; mc.dt = cfg.dt;
    mpc.configure(mc);

    // Initial State
    // State s; s.x = ref[0].x; s.y = ref[0].y - 1.0; s.yaw = ref[0].yaw; s.v = cfg.v_ref * 0.5; // Start with offset + slower
    // Initialize exactly on the reference to avoid initial bias
    State s; s.x = ref[0].x; s.y = ref[0].y;
    s.yaw = ref[0].yaw; s.v = cfg.v_ref;

    std::vector<std::vector<double>> log_ref;
    std::vector<std::vector<double>> log_lqr;
    for(int k = 0; k < cfg.steps; k++)
    {
        const RefPoint& r = ref[k];

        /*
        // Compute tracking error in global frame (simple):
        double ex = r.x - s.x;
        double ey = r.y - s.y;
        double eyaw = normalizeAngle(r.yaw - s.yaw);
        double ev = r.v - s.v;
        */

        // Compute tracking error in the REFERENCE (path-tangent) frame for better tracking stability:
        double dx = s.x - r.x;
        double dy = s.y - r.y;
        double cosr = std::cos(r.yaw);
        double sinr = std::sin(r.yaw);

        double e_lon = cosr * dx + sinr * dy;
        double e_lat = -sinr * dx + cosr * dy;

        double e_yaw = normalizeAngle(s.yaw - r.yaw);
        double ev = s.v - r.v;

        Eigen::Vector4d e; e << e_lon, e_lat, e_yaw, ev;

        Control u;
        if(cfg.ctrl_mode == "MPC")
        {
            Eigen::Vector2d uu = mpc.solve(e);
            u.a = uu(0);
            u.delta = uu(1);
        }
        else
        {
            Eigen::Vector2d uu = lqr.control(e);
            u.a = uu(0);
            u.delta = uu(1);
        }

        double yaw_next = ref[std::min(k + 1, cfg.steps - 1)].yaw;
        double dyaw = normalizeAngle(yaw_next - r.yaw);
        double ds = std::max(1e-6, r.v * cfg.dt);
        double kappa = dyaw / ds;
        double delta_ff = std::atan(cfg.veh.L * kappa);
        u.delta += delta_ff;

        s = model.step(s, u);

        if((k % cfg.log_every) == 0)
        {
            std::vector<double> row_r; row_r.push_back((double)k * cfg.dt); row_r.push_back(r.x); row_r.push_back(r.y); row_r.push_back(r.yaw); row_r.push_back(r.v);
            log_ref.push_back(row_r);
            std::vector<double> row_s; row_s.push_back((double)k * cfg.dt); row_s.push_back(s.x); row_s.push_back(s.y); row_s.push_back(s.yaw); row_s.push_back(s.v); row_s.push_back(u.a); row_s.push_back(u.delta);
            log_lqr.push_back(row_s);
        }
    }

    saveCSV("output_ref.csv", log_ref);
    saveCSV("output_lqr.csv", log_lqr);

    std::cout << "Simulation complete. Wrote output_ref.csv and output_lqr.csv" << std::endl;
    return 0;
}