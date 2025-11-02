#include "lqr.hpp"
#include <cmath>

/**
 * Constructor for the LQRController class
 */
LQRController::LQRController() : ready_(false){
    // Initialize the state transition matrix A to zero
    A_.setZero();
    // Initialize the input matrix B to zero
    B_.setZero();
    // Initialize the cost-to-go matrix Q to the identity matrix
    Q_.setIdentity();
    // Initialize the input matrix R to the identity matrix
    R_.setIdentity();
    // Initialize the gain matrix K to zero
    K_.K.setZero();
}

/*
    Build the linear model
    @param dt The time step
    @param L The wheelbase
    @param v_ref The reference velocity
*/
void LQRController::buildLinearModel(double dt, double L, double v_ref){
    // Linearized discrete model around small angles (ψ≈0, δ≈0) and v≈v_ref
    // State e = [e_lon, e_lat, e_yaw, ev]^T
    Eigen::Matrix4d Ad = Eigen::Matrix4d::Identity();
    Ad(0,3) = dt;            // e_lon depends on ev
    Ad(1,2) = v_ref * dt;    // e_lat depends on e_yaw

    Eigen::Matrix<double,4,2> Bd; Bd.setZero();
    Bd(2,1) = (v_ref / L) * dt; // e_yaw depends on δ
    Bd(3,0) = dt;               // ev depends on a

    A_ = Ad;
    B_ = Bd;
}
// // void LQRController::buildLinearModel(double dt, double L, double v_ref){
//     // States: [x,y,yaw,v]
//     // Inputs: [a, delta]
//     // Discrete model around small angles (cos ~ 1, sin ~ 0, tan(delta) ~ delta)
//     Eigen::Matrix4d Ad = Eigen::Matrix4d::Identity();
//     Ad(0,2) = -v_ref * dt * std::sin(0.0);  // ~0
//     Ad(0,3) = dt;
//     Ad(1,2) = v_ref * dt * std::cos(0.0);  // ~v_ref*dt
//     Ad(2,3) = dt * (1.0 / L) * 0.0; // ~0

//     // Improve: use a more faithful discretization
//     Ad(1,2) = v_ref * dt; // coupling yaw to y

//     Eigen::Matrix<double,4, 2> Bd;
//     Bd.setZero();
//     Bd(3,0) = dt;                   // dv/da
//     Bd(2,1) = (v_ref / L) * dt;     // dyaw / d(delta)

//     A_ = Ad;
//     B_ = Bd;
// }

void LQRController::setWeights(const Eigen::Matrix4d& Q, const Eigen::Matrix2d& R){
    Q_ = Q; R_ = R;
}


bool LQRController::solve() {
    // Discrete Ricatti iteration
    Eigen::Matrix4d P = Q_;
    const int max_iter = 200;
    const double eps = 1e-8;

    for(int i = 0; i < max_iter; i++)
    {
        // Compute the cost-to-go matrix S
        Eigen::Matrix2d S = R_ + B_.transpose() * P * B_;
        // Compute the gain matrix K
        Eigen::Matrix<double, 2, 4> K = S.ldlt().solve(B_.transpose() * P * A_);
        // Compute the next cost-to-go matrix Pn
        Eigen::Matrix4d Pn = Q_ + A_.transpose() * P * (A_ - B_ * K);
        // Compute the difference between the current and next cost-to-go matrices
        double diff = (Pn - P).norm();
        // Update the cost-to-go matrix
        P = Pn;
        // Check if the difference is less than the tolerance
        if(diff < eps)
        {
            K_.K = K;
            // Set the ready flag to true
            ready_ = true;
            return true;
        }
    }
    // If the solution is not found, set the ready flag to false    
    K_.K.setZero();
    ready_ = false;
    return false;
}

Eigen::Vector2d LQRController::control(const Eigen::Vector4d& e) const {
    Eigen::Vector2d u;
    if(!ready_) {u.setZero(); return u;}
    u = -K_.K * e;
    return u;
}

const LQRGains& LQRController::gains() const {return K_;}


