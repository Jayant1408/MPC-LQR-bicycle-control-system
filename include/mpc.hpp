#ifndef MPC_HPP
#define MPC_HPP

#include <Eigen/Dense>
#include <vector>

struct MPCConfig {
    int N;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix2d Rd; // rate penalty
    double v_ref;
    double L;
    double dt;
};

class MPCController {
    public:
        
        MPCController();
        void configure(const MPCConfig& cfg);

        // Simple linearized model per-step (same as LQR Linearization)
        // Returns u_k = [a, delta] fpr current error e_k

        Eigen::Vector2d solve(const Eigen::Vector4d& e_k);

    private:
        MPCConfig cfg_;
        bool ready_;
        Eigen::Matrix4d A_;
        Eigen::Matrix<double,4,2> B_;

        void buildLinearModel();
};

#endif // MPC_HPP

