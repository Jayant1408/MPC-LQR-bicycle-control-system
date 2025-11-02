#include "trajectory.hpp"
#include <cmath>

std::vector<RefPoint> Trajectory::circle(double radius, double v_ref, double dt, int steps){
    std::vector<RefPoint> ref;
    ref.reserve(steps);
    double omega = v_ref / radius;
    double theta = 0.0;
    double x0 = radius;  // start at (R,0)
    double y0 = 0.0;
    for(int k = 0; k < steps; k++)
    {
        RefPoint p;
        p.x = radius * std::cos(theta);
        p.y = radius * std::sin(theta);
        p.yaw = theta + M_PI/2.0;  // tangent direction
        p.v = v_ref;
        ref.push_back(p);
        theta += omega * dt;
    }

    (void)x0; (void)y0;  // suppress unused warnings
    return ref;
}

std::vector<RefPoint> Trajectory::figure8(double radius, double v_ref, double dt, int steps)
{
    std::vector<RefPoint> ref;
    ref.reserve(steps);
    double t = 0.0;
    for(int k = 0; k < steps; k++)
    {
        double a = std::sin(t);
        double b = std::sin(t) * std::cos(t);  // 
        RefPoint p;
        p.x = radius * a;
        p.y = radius * b;
        p.yaw = std::atan2(radius * (std::cos(t) * std::cos(t) - std::sin(t) * std::sin(t)), radius * std::cos(t));
        p.v = v_ref;
        ref.push_back(p);
        t += v_ref * dt/radius;
    }
    return ref;
}

std::vector<RefPoint> Trajectory::laneChange(double length, double v_ref, double dt, int steps)
{
    std::vector<RefPoint> ref;
    ref.reserve(steps);
    double s = 0.0;
    for(int k = 0; k < steps; k++)
    {
        RefPoint p;
        double x = (double)k * (length / (double) steps);
        double y = 0.5 * std::sin(2.0 * M_PI * x / length);
        p.x = x;
        p.y = y;
        p.yaw = std::atan2(0.5 * (2.0 * M_PI / length) * std::cos(2.0 * M_PI * x / length), 1.0);
        p.v = v_ref;
        ref.push_back(p);
        s += v_ref * dt;
    }
    return ref;

}

