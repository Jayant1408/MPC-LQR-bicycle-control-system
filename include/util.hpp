#pragma once

#include <string>
#include <vector>
#include "bicycle_model.hpp"

// Global simulation + controller configuration
struct SimConfig
{
    // Simulation
    double dt;
    int    steps;
    int    log_every;

    // Vehicle
    VehicleParams veh;

    // Controller mode & reference speed
    std::string ctrl_mode;  // "LQR", "MPC", "MPC_OSQP"
    double      v_ref;

    // LQR weights
    double Qx;
    double Qy;
    double Qyaw;
    double Qv;
    double Ra;
    double Rdelta;

    // MPC horizon and weights
    int    N;
    double Qmx;
    double Qmy;
    double Qmyaw;
    double Qmv;
    double Rma;
    double Rmdelta;
    double Rd_a;
    double Rd_delta;

    // Trajectory parameters
    std::string traj_type;   // "straight", "circle", "figure8", "lanechange"
    double      radius;      // circle / figure8 radius
    double      length;      // lane-change longitudinal length
    double      lane_width;  // lane-change lateral width
};

// Load configuration from YAML (or use defaults if YAML fails)
bool loadConfig(const std::string& path, SimConfig& cfg);

// Save table of doubles to CSV
void saveCSV(const std::string& path,
             const std::vector<std::vector<double>>& rows);
