#include "mpc_osqp.hpp"
#ifdef HAVE_OSQP
    #include "osqp.h"
#endif
#include <Eigen/Sparse>
#include <cassert>

static inline Eigen::SparseMatrix<double> sparse_from_triplets(
    int rows, int cols, const std::vector<Eigen::Triplet<double>>& T){
        Eigen::SparseMatrix<double> S(rows, cols);
        S.setFromTriplets(T.begin(), T.end());
        S.makeCompressed();
        return S;
    }

MPCControllerOSQP::MPCControllerOSQP() {A_.setZero(); B_.setZero();}

void MPCControllerOSQP::configure(const MPCOSQPConfig& cfg)
{
    cfg_ = cfg;
    buildLinearModel();
}

void MPCControllerOSQP::buildLinearModel() {
    A_.setIdentity();
    A_(0,3) = cfg_.dt;               // e_lon <- ev
    A_(1,2) = cfg_.v_ref * cfg_.dt;  // e_lat <- e_yaw
    B_.setZero();
    B_(2,1) = (cfg_.v_ref / cfg_.L) * cfg_.dt;  // e_yaw <- delta
    B_(3,0) = cfg_.dt;                          // ev <- accel 
}

// Build condensed QP in control-only form:
// Decision z = [u_0; u_1; ...; u_{N-1}] in R^{2N} (each u_i=[a,delta])
// Cost ~ sum_i (x_i^T Q x_i) + (u_i - Uff_i)^T R (u_i - Uff_i) + rate terms around Uff
// Subject to input bounds and (optional) rate bounds.

void MPCControllerOSQP::buildQP(const Eigen::Vector4d& e0,
                                const Eigen::Matrix<double, 2, Eigen::Dynamic>& Uff,
                                const Eigen::Vector2d& u_prev,
                                Eigen::SparseMatrix<double>& H,
                                Eigen::VectorXd& f,
                                Eigen::SparseMatrix<double>& Aineq,
                                Eigen::VectorXd& l,
                                Eigen::VectorXd& u)
{
    const int nx = 4, nu = 2, N = cfg_.N;
    const int Nz = nu * N;

    // Precompute powers of A and lifted dynamics to express x_i = A^i e0 + sum_j A^{i-1-j} B u_j
    std::vector<Eigen::Matrix4d> Ap(N + 1);
    Ap[0] = Eigen::Matrix4d::Identity();
    for(int i = 1; i <= N; ++i) Ap[i] = A_ * Ap[i - 1];

    // Build big S matrix mapping stacked U -> stacked X (excluding x0), and term for x from e0
    // X_stack = S * U_stack + T * e0
    // Where X_stack = [x1; x2; ...; xN]
    const int Nxstack = nx * N;
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(Nxstack, nx);
    for(int i = 0; i < N; ++i)
    {
        T.block(i * nx, 0, nx, nx) = Ap[i + 1];
    }
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(Nxstack, Nz);
    for(int i = 0; i < N; ++i)  
    {                                   // row block for x_{i+1}
        for(int j = 0; j <= i; ++j)     // col block for u_j
        {
            Eigen::Matrix4d A_pow = Ap[i - j];
            S.block(i * nx, j * nu, nx, nu) = A_pow * B_;
        }
    }

    // Quadratic cost terms
    // J = sum_{i=0..N-1} (x_i^T Q x_i) + (u_i - uff_i)^T R (u_i - uff_i) + Δu_i^T Rd Δu_i
    // with x_0 = e0, and for i >= 1 use X_stack.
    // Build H and f over z = U_stack.

    Eigen::MatrixXd H_dense = Eigen::MatrixXd::Zero(Nz, Nz);
    Eigen::VectorXd f_dense = Eigen::VectorXd::Zero(Nz);


    // State cost: includes x1..xN (x0 handled outside as constant)
    // X = S z + T e0
    Eigen::MatrixXd Qblk = Eigen::MatrixXd::Zero(Nxstack, Nxstack);
    for(int i = 0; i < N; i++) Qblk.block(i * nx, i * nx, nx, nx) = cfg_.Q;
    H_dense += S.transpose() * Qblk * S;
    f_dense += (S.transpose() * Qblk * T) * e0; // linear term 2*? We'll put factor 1/2 later in OSQP

    // Input cost around Uff: sum(u_i - uff_i) ^ T R (u_i - uff_i)
    // Expand: z^T (I ⊗ R) z - 2 (Uff)^T (I⊗R) z + const
    Eigen::MatrixXd Rblk = Eigen::MatrixXd::Zero(Nz, Nz);
    for(int i = 0; i < N; ++i) Rblk.block(i * nu, i * nu, nu, nu) = cfg_.R;
    Eigen::VectorXd Uff_stack(Nz);
    for(int i = 0; i < N; ++i) Uff_stack.segment(i * nu, nu) = Uff.col(i);
    H_dense += Rblk;
    f_dense += -2.0 * Rblk * Uff_stack;

    // Rate penalty around Uff rates: Δu_i - Δu_ff_i
    // Build D such that D*z = [u0 - u_prev; u1 - u0; ...]

    Eigen::MatrixXd D  = Eigen::MatrixXd::Zero(Nz, Nz);
    for(int i = 0; i < N; ++i)
    {
        if(i == 0) D.block(0,0,nu,nu) = Eigen::Matrix2d::Identity();
        else
        {
            D.block(i*nu, i*nu, nu, nu) =  Eigen::Matrix2d::Identity();
            D.block(i*nu, (i-1)*nu, nu, nu) =  -Eigen::Matrix2d::Identity();
        }
    }

    // Δu_ff stack
    Eigen::VectorXd dUff(Nz);
    for(int i = 0; i < N; ++i) {
        Eigen::Vector2d prev = (i==0) ? u_prev : Uff.col(i - 1);
        dUff.segment(i * nu, nu) = Uff.col(i) - prev;
    }

     // Add D^T (I⊗Rd) D
     Eigen::MatrixXd Rdblk = Eigen::MatrixXd::Zero(Nz, Nz);
     for(int i = 0; i < N; ++i) Rdblk.block(i*nu, i*nu, nu, nu) = cfg_.Rd;
     H_dense += D.transpose() * Rdblk * D;
     f_dense += -2.0 * D.transpose() * Rdblk * dUff;

     // Inequality constraints: input bounds and (optional) rate bounds
     // Stacked bounds:
     std::vector<Eigen::Triplet<double>> Atrip;
     std::vector<double> Lb, Ub;

     auto push_box = [&](int row_base, int col, double l0, double hi) {
          // rows row_base..row_base for each scalar bound, but we'll add row per scalar
          const int r = (int)Lb.size(); (void)row_base;
          Atrip.emplace_back((int)Lb.size(), col, 1.0);
          Lb.push_back(l0); Ub.push_back(hi);
     };
    // |a|, |delta| per step
    for (int i = 0; i < N; ++i) {
        int idx_a = i*nu + 0;
        int idx_d = i*nu + 1;
        // a bounds
        Atrip.emplace_back((int)Lb.size(), idx_a, 1.0); Lb.push_back(-cfg_.max_accel); Ub.push_back(cfg_.max_accel);
        // delta bounds
        Atrip.emplace_back((int)Lb.size(), idx_d, 1.0); Lb.push_back(-cfg_.max_steer); Ub.push_back(cfg_.max_steer);
    }
    // Rate bounds (if any)
    if (cfg_.max_da > 0.0 || cfg_.max_dd > 0.0) {
        for (int i = 0; i < N; ++i) {
            // a rate
            {
                int row = (int)Lb.size();
                for (int j = 0; j < Nz; ++j) {
                    // D block row for accel
                    if (j == i*nu+0)        Atrip.emplace_back(row, j,  1.0);
                    if (i>0 && j == (i-1)*nu+0) Atrip.emplace_back(row, j, -1.0);
                }
                double lo = (i==0) ? -cfg_.max_da + u_prev(0) : -cfg_.max_da;
                double hi = (i==0) ?  cfg_.max_da + u_prev(0) :  cfg_.max_da;
                Lb.push_back(lo); Ub.push_back(hi);
            }
            // delta rate
            {
                int row = (int)Lb.size();
                for (int j = 0; j < Nz; ++j) {
                    if (j == i*nu+1)        Atrip.emplace_back(row, j,  1.0);
                    if (i>0 && j == (i-1)*nu+1) Atrip.emplace_back(row, j, -1.0);
                }
                double lo = (i==0) ? -cfg_.max_dd + u_prev(1) : -cfg_.max_dd;
                double hi = (i==0) ?  cfg_.max_dd + u_prev(1) :  cfg_.max_dd;
                Lb.push_back(lo); Ub.push_back(hi);
            }
        }
    }

    // Convert dense matrices to sparse
    // Ensure H is symmetric for OSQP (OSQP expects symmetric Hessian)
    H_dense = 0.5 * (H_dense + H_dense.transpose());
    Eigen::SparseMatrix<double> H_sparse = H_dense.sparseView();
    H = H_sparse;
    f = f_dense;          // OSQP uses (1/2) z^T H z + f^T z
    Aineq = sparse_from_triplets((int)Lb.size(), Nz, Atrip);
    l = Eigen::Map<Eigen::VectorXd>(Lb.data(), (int)Lb.size());
    u = Eigen::Map<Eigen::VectorXd>(Ub.data(), (int)Ub.size());

}

Eigen::Vector2d MPCControllerOSQP::solveOSQP(const Eigen::SparseMatrix<double>& H,
    const Eigen::VectorXd& f,
    const Eigen::SparseMatrix<double>& Aineq,
    const Eigen::VectorXd& l,
    const Eigen::VectorXd& u)
{
#ifndef HAVE_OSQP
(void)H; (void)f; (void)Aineq; (void)l; (void)u;
return Eigen::Vector2d::Zero();
#else
// Convert Eigen sparse to OSQP csc
c_int n = (c_int)H.cols();
c_int m = (c_int)Aineq.rows();

// Helper to wrap Eigen::Sparse into OSQP csc without copying too much
auto to_csc = [](const Eigen::SparseMatrix<double>& S) {
Eigen::SparseMatrix<double> Sc = S; Sc.makeCompressed();
c_int*    indptr = (c_int*)Sc.outerIndexPtr();
c_int*    indices= (c_int*)Sc.innerIndexPtr();
c_float*  data   = (c_float*)Sc.valuePtr();
return std::tuple<c_int*, c_int*, c_float*, Eigen::SparseMatrix<double>>(indptr, indices, data, Sc);
};

auto [P_indptr, P_indices, P_data, P_hold] = to_csc(H.selfadjointView<Eigen::Upper>());
auto [A_indptr, A_indices, A_data, A_hold] = to_csc(Aineq);

OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
osqp_set_default_settings(settings);
settings->alpha = 1.6;
settings->eps_abs = 1e-4;
settings->eps_rel = 1e-4;
settings->verbose = 0;
settings->polish  = 1;

OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));
data->n = n;
data->m = m;
data->P = csc_matrix(n, n, (c_int)P_hold.nonZeros(), P_data, P_indices, P_indptr);
data->q = (c_float*)f.data();
data->A = csc_matrix(m, n, (c_int)A_hold.nonZeros(), A_data, A_indices, A_indptr);
data->l = (c_float*)l.data();
data->u = (c_float*)u.data();

OSQPWorkspace* work = osqp_setup(data, settings);
osqp_solve(work);

Eigen::Vector2d u0(0.0, 0.0);
if (work->info->status_val >= 1 && work->solution && work->solution->x) {
// First control is first two elements
u0(0) = work->solution->x[0];
u0(1) = work->solution->x[1];
}

osqp_cleanup(work);
c_free(data->P); c_free(data->A);
c_free(data); c_free(settings);
return u0;
#endif
}

Eigen::Vector2d MPCControllerOSQP::solve(const Eigen::Vector4d& e_k,
const Eigen::Matrix<double,2,Eigen::Dynamic>& Uff,
const Eigen::Vector2d& u_prev)
{
const int nu = 2, N = cfg_.N;
assert(Uff.cols() == N);

Eigen::SparseMatrix<double> H, Aineq;
Eigen::VectorXd f, l, u;
buildQP(e_k, Uff, u_prev, H, f, Aineq, l, u);
return solveOSQP(H, f, Aineq, l, u);
}