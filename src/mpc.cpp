#include "mpc.hpp"

MPCController::MPCController() : ready_(false) {
    A_.setZero();
    B_.setZero();
}

void MPCController::configure(const MPCConfig& cfg)
{
    cfg_ = cfg;
    buildLinearModel();
    ready_ = true;
}

void MPCController::buildLinearModel() {
    A_.setIdentity();
    A_(0,3) = cfg_.dt;                          // x depends on v
    A_(1,2) = cfg_.v_ref * cfg_.dt;             // y depends on yaw

    B_.setZero();                   
    B_(3,0) = cfg_.dt;                          // dv/da
    B_(2,1) = (cfg_.v_ref / cfg_.L) * cfg_.dt;  // dyaw/d(delta)
} 

Eigen::Vector2d MPCController::solve(const Eigen::Vector4d& e_k)
{
    // Stub: return DLQR-like action for now (acts as a stabilizing controller)
    // You can replace this with a QP solve over horizon N.
    Eigen::Matrix4d Q = cfg_.Q;
    Eigen::Matrix2d R = cfg_.R;

    // One-step Ricatti equivalent (not true MPC). Use same interaction as LQR for a quick stabilizer.
    Eigen::Matrix4d P = Q;
    for(int i = 0; i < 100; i++)
    {
        Eigen::Matrix2d S = R + B_.transpose() * P * B_;
        Eigen::Matrix<double,2,4> K = S.ldlt().solve(B_.transpose() * P * A_);
        Eigen::Matrix4d Pn = Q + A_.transpose() * P * (A_ - B_ * K);
        if((Pn - P).norm() < 1e-8) {P = Pn; break;}
        P = Pn;
    }

    Eigen::Matrix<double,2,4> K = (R + B_.transpose() * P * B_).ldlt().solve(B_.transpose() * P * A_);
    Eigen::Vector2d u = -K * e_k;
    return u;

}