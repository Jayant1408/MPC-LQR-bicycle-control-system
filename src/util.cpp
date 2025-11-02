#include "util.hpp"
#include <fstream>
#include <sstream>

#ifdef HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

bool loadConfig(const std::string& path, SimConfig& cfg) {
    // defaults
    cfg.dt = 0.05;
    cfg.steps = 1200;
    cfg.log_every = 1;
    cfg.veh = {2.7, 0.6, 2.5, 20.0};
    cfg.ctrl_mode = "LQR";
    cfg.v_ref = 8.0;
    cfg.Qx = 4.0; cfg.Qy = 4.0; cfg.Qyaw = 1.0; cfg.Qv = 0.5;
    cfg.Ra = 0.5; cfg.Rdelta = 0.2;
    cfg.N = 15;
    cfg.Qmx = 6.0; cfg.Qmy = 6.0; cfg.Qmyaw = 1.5; cfg.Qmv = 0.5;
    cfg.Rma = 0.5; cfg.Rmdelta = 0.2; cfg.Rd_a = 0.1; cfg.Rd_delta = 0.1;
    cfg.traj_type = "figure8"; cfg.radius = 20.0; cfg.length = 200.0;

#ifdef HAVE_YAML
    try {
        YAML::Node root = YAML::LoadFile(path);
        if (root["sim"]) {
            YAML::Node s = root["sim"];
            if (s["dt"]) cfg.dt = s["dt"].as<double>();
            if (s["steps"]) cfg.steps = s["steps"].as<int>();
            if (s["log_every"]) cfg.log_every = s["log_every"].as<int>();
        }
        if (root["vehicle"]) {
            YAML::Node v = root["vehicle"];
            if (v["L"]) cfg.veh.L = v["L"].as<double>();
            if (v["max_steer"]) cfg.veh.max_steer = v["max_steer"].as<double>();
            if (v["max_accel"]) cfg.veh.max_accel = v["max_accel"].as<double>();
            if (v["max_speed"]) cfg.veh.max_speed = v["max_speed"].as<double>();
        }
        if (root["controller"]) {
            YAML::Node c = root["controller"];
            if (c["mode"]) cfg.ctrl_mode = c["mode"].as<std::string>();
            if (c["v_ref"]) cfg.v_ref = c["v_ref"].as<double>();
        }
        if (root["lqr"]) {
            YAML::Node l = root["lqr"];
            if (l["Q"]) {
                cfg.Qx = l["Q"][0].as<double>();
                cfg.Qy = l["Q"][1].as<double>();
                cfg.Qyaw = l["Q"][2].as<double>();
                cfg.Qv = l["Q"][3].as<double>();
            }
            if (l["R"]) {
                cfg.Ra = l["R"][0].as<double>();
                cfg.Rdelta = l["R"][1].as<double>();
            }
        }
        if (root["mpc"]) {
            YAML::Node m = root["mpc"];
            if (m["N"]) cfg.N = m["N"].as<int>();
            if (m["Q"]) {
                cfg.Qmx = m["Q"][0].as<double>();
                cfg.Qmy = m["Q"][1].as<double>();
                cfg.Qmyaw = m["Q"][2].as<double>();
                cfg.Qmv = m["Q"][3].as<double>();
            }
            if (m["R"]) {
                cfg.Rma = m["R"][0].as<double>();
                cfg.Rmdelta = m["R"][1].as<double>();
            }
            if (m["Rd"]) {
                cfg.Rd_a = m["Rd"][0].as<double>();
                cfg.Rd_delta = m["Rd"][1].as<double>();
            }
        }
        if (root["trajectory"]) {
            YAML::Node t = root["trajectory"];
            if (t["type"]) cfg.traj_type = t["type"].as<std::string>();
            if (t["radius"]) cfg.radius = t["radius"].as<double>();
            if (t["length"]) cfg.length = t["length"].as<double>();
        }
    } catch (...) {
        // If load fails, keep defaults
    }
#else
    (void)path; // suppress unused warning
#endif
    return true;
}

void saveCSV(const std::string& path, const std::vector<std::vector<double>>& rows) {
    std::ofstream f(path.c_str());
    for (size_t i = 0; i < rows.size(); i++) {
        const std::vector<double>& r = rows[i];
        for (size_t j = 0; j < r.size(); j++) {
            f << r[j];
            if (j + 1 < r.size()) f << ",";
        }
        f << "\n";
    }
}