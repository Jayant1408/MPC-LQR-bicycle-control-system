#include "bicycle_model.hpp"
#include <cmath>

/**
 * Constructor for the BicycleModel class
 * @param p The vehicle parameters
 * @param dt The time step
 */
BicycleModel::BicycleModel(const VehicleParams& p, double dt) : params_(p), dt_(dt) {}

/**
 * Step the bicycle model forward
 * @param s The current state
 * @param u_in The input control
 * @return The next state
 */
State BicycleModel::step(const State& s, const Control& u_in) const {
    // Clamp the input control to the physical limits
    Control u = clamp(u_in);
    State sn = s; // next state
    double beta = 0.0; // simple kinematic model (no slip)
    // Update the state of the bicycle model
    // x = x + v * cos(yaw) * dt
    sn.x += s.v * std::cos(s.yaw) * dt_;
    // y = y + v * sin(yaw) * dt
    sn.y += s.v * std::sin(s.yaw) * dt_;
    // yaw = yaw + (v / L) * tan(delta) * dt
    sn.yaw += (s.v / params_.L) * std::tan(u.delta) * dt_;
    // v = v + a * dt
    sn.v += u.a * dt_;


    // speed clamp
    if(sn.v > params_.max_speed) sn.v = params_.max_speed;
    if(sn.v < -params_.max_speed) sn.v = -params_.max_speed;

    return sn;

}

// Clamp the input control to the physical limits
Control BicycleModel::clamp(const Control& u) const {
    Control uc = u;
    // Clamp the steering angle to the physical limits
    if(uc.delta > params_.max_steer) uc.delta = params_.max_steer;
    if(uc.delta < -params_.max_steer) uc.delta = -params_.max_steer;
    // Clamp the acceleration to the physical limits
    if(uc.a > params_.max_accel) uc.a = params_.max_accel;
    if(uc.a < -params_.max_accel) uc.a = -params_.max_accel;
    return uc;
}


double BicycleModel::dt() const {return dt_; }
const VehicleParams& BicycleModel::params() const {return params_; }



