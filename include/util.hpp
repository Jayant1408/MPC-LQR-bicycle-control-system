#ifndef UTIL_HPP
#define UTIL_HPP

#include <string>
#include <vector>
#include "bicycle_model.hpp"
#include "trajectory.hpp"

struct SimConfig {
    // Simulation parameters
    double dt;
    int steps;
    int log_every;
    VehicleParams veh;
    std::string ctrl_mode;  // "LQR" or "MPC"
    double v_ref;
    
    // LQR weights
    double Qx, Qy, Qyaw, Qv;    // State weights
    double Ra, Rdelta;          // Control weights
    
    // MPC parameters
    int N;                      // Prediction horizon
    double Qmx, Qmy, Qmyaw, Qmv; // State weights
    double Rma, Rmdelta;        // Control weights
    double Rd_a, Rd_delta;      // Rate weights
    
    // Trajectory parameters
    std::string traj_type;      // "circle", "figure8", "lanechange"
    double radius;
    double length;
};

bool loadConfig(const std::string& path, SimConfig& cfg);

// helpers
void saveCSV(const std::string& path, const std::vector<std::vector<double>>& rows);

#endif // UTIL_HPP