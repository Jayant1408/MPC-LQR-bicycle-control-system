#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include <vector>

struct State {
    double x;      // Position X (m)
    double y;      // Position Y (m) 
    double yaw;    // Heading angle (rad)
    double v;      // Velocity (m/s)
};

struct Control {
    double a;      // Acceleration (m/s²)
    double delta;  // Steering angle (rad)
};

struct VehicleParams {
    double L;          // Wheelbase (m)
    double max_steer;  // Max steering angle (rad)
    double max_accel;  // Max acceleration (m/s²)
    double max_speed;  // Max speed (m/s)
};

class BicycleModel{
    public:
        BicycleModel(const VehicleParams& p, double dt);
        State step(const State& s, const Control& u) const; // one step forward
        
        // Clamp controls to physical limits;
        Control clamp(const Control& u) const;

        double dt() const;
        const VehicleParams& params() const;

    private:
        VehicleParams params_;
        double dt_;
};

#endif // BICYCLE_MODEL_HPP