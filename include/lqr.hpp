#ifndef LQR_HPP
#define LQR_HPP

#include <Eigen/Dense>

struct LQRGains {
    Eigen::Matrix<double,2,4> K; // u = -K * e
};


class LQRController {
    public:
        LQRController();

        // Build linearized discrete-time model around small yaw and constance v_ref
        void buildLinearModel(double dt, double L, double v_ref);

        // Set cost weights
        void setWeights(const Eigen::Matrix4d& Q, const Eigen::Matrix2d& R);

        // Solve discrete-time Algebraic Ricatti Equation via iteration
        bool solve();

        // Compute control give error state e = [ex, ey, e_yaw, ev]
        Eigen::Vector2d control(const Eigen::Vector4d& e) const;

        const LQRGains& gains() const;

    private:
        Eigen::Matrix4d A_;
        Eigen::Matrix<double,4,2> B_;
        Eigen::Matrix4d Q_;
        Eigen::Matrix2d R_;
        LQRGains K_;
        bool ready_;
};


#endif // LQR_HPP


