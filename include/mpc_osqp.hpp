#ifndef MPC_OSQP_HPP
#define MPC_OSQP_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

struct MPCOSQPConfig {
    int N;                  // horizon 
    double dt, L, v_ref;    // state cost
    Eigen::Matrix4d Q;      // input cost (around feed-forward)
    Eigen::Matrix2d R;      // rate cost (around feed-forward rate)
    Eigen::Matrix2d Rd;
    double max_accel, max_steer;
    double max_da, max_dd; // per-step rate limits (0 disables)
};

class MPCControllerOSQP{
public:
    MPCControllerOSQP();
    void configure(const MPCOSQPConfig& cfg);


    // Inputs:
    // e_k = [e_lon, e_lat, e_yaw, ev] (error at step k)
    // uff = 2xN feed-forward over horizon (rows: [a_ff, delta_ff])
    // u_prev= previous applied control [a, delta] (for rate costs/limits)
    // Output: u0 = first optimal control [a, delta]
    Eigen::Vector2d solve(const Eigen::Vector4d& e_k,
                          const Eigen::Matrix<double, 2, Eigen::Dynamic>& Uff,
                          const Eigen::Vector2d& u_prev);

private:
    MPCOSQPConfig cfg_;
    Eigen::Matrix4d A_;
    Eigen::Matrix<double, 4, 2> B_;

    // Helpers to build condensed QP (H, f, Aineq, l, u)
    void buildLinearModel();
    void buildQP(const Eigen::Vector4d& e0,
                  const Eigen::Matrix<double, 2, Eigen::Dynamic>& Uff,
                  const Eigen::Vector2d& u_prev,
                  Eigen::SparseMatrix<double>& H,
                  Eigen::VectorXd& f,
                  Eigen::SparseMatrix<double>& Aineq,
                  Eigen::VectorXd& l,
                  Eigen::VectorXd& u);

    // Solve with OSQP (returns first control)
    Eigen::Vector2d solveOSQP(const Eigen::SparseMatrix<double>& H,
        const Eigen::VectorXd& f,
        const Eigen::SparseMatrix<double>& Aineq,
        const Eigen::VectorXd& l,
        const Eigen::VectorXd& u);

};

#endif // MPC_OSQP_HPP
