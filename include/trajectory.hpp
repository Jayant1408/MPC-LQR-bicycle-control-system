#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <vector>
#include <string>
#include "bicycle_model.hpp"

struct RefPoint {
    double x;    // Reference X position
    double y;    // Reference Y position
    double yaw;  // Reference heading
    double v;    // Reference velocity
};

class Trajectory {
    public:
        static std::vector<RefPoint> circle(double radius, double v_ref, double dt, int steps);
        static std::vector<RefPoint> figure8(double radius, double v_ref, double dt, int steps);
        static std::vector<RefPoint> laneChange(double length, double v_ref, double dt, int steps);
};

#endif // TRAJECTORY_HPP