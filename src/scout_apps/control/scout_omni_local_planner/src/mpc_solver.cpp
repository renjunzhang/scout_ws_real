/**
 * @file mpc_solver.cpp
 * @brief 全向轮 MPC 求解器实现
 * 
 * 决策变量 z = [x₀, u₀, x₁, u₁, ..., x_N]
 *   x ∈ R⁹ (9维增广状态), u ∈ R³ (3维控制)
 *   nz = N * (9 + 3) + 9 = 12N + 9
 * 
 * 适配 OSQP 旧版 API (osqp-vendor in ROS Noetic)
 */

#include "scout_omni_local_planner/mpc_solver.h"
#include "scout_omni_local_planner/omni_drive_model.h"

#include <chrono>
#include <ros/ros.h>

namespace scout_omni_local_planner {

MPCSolver::MPCSolver() = default;

MPCSolver::~MPCSolver() {
    cleanupOSQP();
}

void MPCSolver::cleanupOSQP() {
    if (osqp_work_) {
        osqp_cleanup(osqp_work_);
        osqp_work_ = nullptr;
    }
    if (osqp_data_) {
        if (osqp_data_->P) c_free(osqp_data_->P);
        if (osqp_data_->A) c_free(osqp_data_->A);
        c_free(osqp_data_);
        osqp_data_ = nullptr;
    }
    if (osqp_settings_) {
        c_free(osqp_settings_);
        osqp_settings_ = nullptr;
    }
    osqp_initialized_ = false;
}

bool MPCSolver::initialize(const MPCParams& mpc_params, 
                            const VehicleParams& vehicle_params) {
    mpc_params_ = mpc_params;
    vehicle_params_ = vehicle_params;
    
    // 默认全向轮模型
    dynamics_model_ = std::make_shared<OmniDriveModel>(vehicle_params);
    
    cost_function_.initialize(mpc_params);
    constraint_manager_.initialize(vehicle_params);
    
    const int nx = StateIndex::TOTAL_DIM;  // 9
    const int nu = ControlIndex::DIM;       // 3
    const int N = mpc_params_.N;
    
    nz_ = N * (nx + nu) + nx;
    
    initialized_ = true;
    ROS_INFO("[MPCSolver] Omni MPC initialized: N=%d, dt=%.3f, nx=%d, nu=%d, nz=%d", 
             N, mpc_params_.dt, nx, nu, nz_);
    
    return true;
}

void MPCSolver::setDynamicsModel(DynamicsModelPtr model) {
    dynamics_model_ = model;
}

void MPCSolver::addConstraint(ConstraintPtr constraint) {
    constraint_manager_.addConstraint(constraint);
}

void MPCSolver::removeConstraint(const std::string& name) {
    constraint_manager_.removeConstraint(name);
}

void MPCSolver::addCostTerm(CostTermPtr term) {
    cost_function_.addCostTerm(term);
}

void MPCSolver::setPreviousControl(const ControlVector& u_prev) {
    u_prev_ = u_prev;
    cost_function_.setPreviousControl(u_prev);
}

void MPCSolver::setMPCParams(const MPCParams& params) {
    mpc_params_ = params;
    cost_function_.initialize(params);
}

MPCSolution MPCSolver::solve(
    const StateVector& current_state,
    const std::vector<ReferencePoint>& reference_path) {
    
    MPCSolution solution;
    solution.success = false;
    
    if (!initialized_) {
        solution.status_msg = "Solver not initialized";
        return solution;
    }
    
    if (reference_path.size() < static_cast<size_t>(mpc_params_.N)) {
        solution.status_msg = "Not enough reference points";
        return solution;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (!buildQP(current_state, reference_path)) {
        solution.status_msg = "Failed to build QP";
        return solution;
    }
    
    if (!updateOSQP()) {
        solution.status_msg = "Failed to update OSQP";
        return solution;
    }
    
    warmStart();
    
    c_int exit_flag = osqp_solve(osqp_work_);
    
    if (exit_flag != 0) {
        solution.status_msg = "OSQP solve failed with exit_flag=" + std::to_string(exit_flag);
        return solution;
    }
    
    if (osqp_work_->info->status_val != OSQP_SOLVED &&
        osqp_work_->info->status_val != OSQP_SOLVED_INACCURATE) {
        solution.status_msg = "OSQP status: " + std::to_string(osqp_work_->info->status_val);
        return solution;
    }
    
    extractSolution(solution);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    solution.solve_time_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    
    solution.success = true;
    solution.status_msg = "Solved";
    
    return solution;
}

bool MPCSolver::buildQP(
    const StateVector& x0,
    const std::vector<ReferencePoint>& refs) {
    
    const int nx = StateIndex::TOTAL_DIM;   // 9
    const int nu = ControlIndex::DIM;        // 3
    const int N = mpc_params_.N;
    const double dt = mpc_params_.dt;
    
    // 1. 代价函数
    cost_function_.buildQPCost(N, refs, P_, q_);
    
    // 2. 约束
    int num_dynamics_constraints = N * nx;
    int num_initial_constraints = nx;
    
    constraint_manager_.setControlRateConstraints(
        mpc_params_.constrain_omega_rate,
        mpc_params_.constrain_accel_rate,
        dt,
        u_prev_);

    Eigen::SparseMatrix<double> A_bounds;
    Eigen::VectorXd l_bounds, u_bounds;
    constraint_manager_.buildQPConstraints(N, A_bounds, l_bounds, u_bounds);
    int num_bounds_constraints = l_bounds.size();
    
    nc_ = num_initial_constraints + num_dynamics_constraints + num_bounds_constraints;
    
    std::vector<Eigen::Triplet<double>> triplets;
    l_ = Eigen::VectorXd::Zero(nc_);
    u_ = Eigen::VectorXd::Zero(nc_);
    
    int row = 0;
    
    // 初始条件
    for (int i = 0; i < nx; ++i) {
        triplets.emplace_back(row + i, i, 1.0);
        l_(row + i) = x0(i);
        u_(row + i) = x0(i);
    }
    row += nx;
    
    // 动力学约束：从上一周期解恢复控制序列
    std::vector<ControlVector> u_prev_seq(static_cast<size_t>(N), u_prev_);
    if (z_prev_.size() == nz_) {
        for (int k = 0; k < N; ++k) {
            int u_idx = k * (nx + nu) + nx;
            for (int d = 0; d < nu; ++d) {
                u_prev_seq[k](d) = z_prev_(u_idx + d);
            }
        }
    }

    StateVector x_lin = x0;

    for (int k = 0; k < N; ++k) {
        int x_k_idx = k * (nx + nu);
        int u_k_idx = x_k_idx + nx;
        int x_k1_idx = (k + 1) * (nx + nu);
        
        Eigen::MatrixXd A_dyn, B_dyn;
        Eigen::VectorXd c_dyn;
        const ControlVector& u_lin = u_prev_seq[k];
        dynamics_model_->linearize(x_lin, u_lin, refs[k], dt, A_dyn, B_dyn, c_dyn);
        
        // x[k+1]
        for (int i = 0; i < nx; ++i) {
            triplets.emplace_back(row + i, x_k1_idx + i, 1.0);
        }
        
        // -A * x[k]
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                triplets.emplace_back(row + i, x_k_idx + j, -A_dyn(i, j));
            }
        }
        
        // -B * u[k]
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                triplets.emplace_back(row + i, u_k_idx + j, -B_dyn(i, j));
            }
        }
        
        for (int i = 0; i < nx; ++i) {
            l_(row + i) = c_dyn(i);
            u_(row + i) = c_dyn(i);
        }
        
        row += nx;
        x_lin = dynamics_model_->predict(x_lin, u_lin, refs[k], dt);
    }
    
    // 边界约束
    for (int k = 0; k < A_bounds.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_bounds, k); it; ++it) {
            triplets.emplace_back(row + it.row(), it.col(), it.value());
        }
    }
    l_.segment(row, num_bounds_constraints) = l_bounds;
    u_.segment(row, num_bounds_constraints) = u_bounds;
    
    A_.resize(nc_, nz_);
    A_.setFromTriplets(triplets.begin(), triplets.end());
    
    return true;
}

bool MPCSolver::updateOSQP() {
    Eigen::SparseMatrix<double> P_upper = P_.triangularView<Eigen::Upper>();
    P_upper.makeCompressed();
    A_.makeCompressed();
    
    P_p_.assign(P_upper.outerIndexPtr(), P_upper.outerIndexPtr() + P_upper.cols() + 1);
    P_i_.assign(P_upper.innerIndexPtr(), P_upper.innerIndexPtr() + P_upper.nonZeros());
    P_x_.assign(P_upper.valuePtr(), P_upper.valuePtr() + P_upper.nonZeros());
    
    A_p_.assign(A_.outerIndexPtr(), A_.outerIndexPtr() + A_.cols() + 1);
    A_i_.assign(A_.innerIndexPtr(), A_.innerIndexPtr() + A_.nonZeros());
    A_x_.assign(A_.valuePtr(), A_.valuePtr() + A_.nonZeros());
    
    q_data_.resize(q_.size());
    l_data_.resize(l_.size());
    u_data_.resize(u_.size());
    for (int i = 0; i < q_.size(); ++i) q_data_[i] = q_(i);
    for (int i = 0; i < l_.size(); ++i) l_data_[i] = l_(i);
    for (int i = 0; i < u_.size(); ++i) u_data_[i] = u_(i);
    
    if (!osqp_initialized_) {
        cleanupOSQP();
        
        osqp_settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
        osqp_set_default_settings(osqp_settings_);
        osqp_settings_->verbose = false;
        osqp_settings_->warm_start = true;
        osqp_settings_->polish = true;
        osqp_settings_->eps_abs = 1e-4;
        osqp_settings_->eps_rel = 1e-4;
        osqp_settings_->max_iter = 4000;
        
        osqp_data_ = (OSQPData*)c_malloc(sizeof(OSQPData));
        osqp_data_->n = nz_;
        osqp_data_->m = nc_;
        
        osqp_data_->P = csc_matrix(nz_, nz_, P_x_.size(),
                                    P_x_.data(), P_i_.data(), P_p_.data());
        osqp_data_->A = csc_matrix(nc_, nz_, A_x_.size(),
                                    A_x_.data(), A_i_.data(), A_p_.data());
        
        osqp_data_->q = q_data_.data();
        osqp_data_->l = l_data_.data();
        osqp_data_->u = u_data_.data();
        
        c_int exit_flag = osqp_setup(&osqp_work_, osqp_data_, osqp_settings_);
        
        if (exit_flag != 0) {
            ROS_ERROR("[MPCSolver] OSQP setup failed: %d", static_cast<int>(exit_flag));
            return false;
        }
        
        osqp_initialized_ = true;
    } else {
        c_int P_nnz = static_cast<c_int>(P_x_.size());
        c_int A_nnz = static_cast<c_int>(A_x_.size());
        
        if (P_nnz != osqp_data_->P->nzmax || A_nnz != osqp_data_->A->nzmax) {
            ROS_WARN_THROTTLE(1.0, "[MPCSolver] Matrix structure changed, reinitializing");
            cleanupOSQP();
            osqp_initialized_ = false;
            return updateOSQP();
        }
        
        osqp_update_P(osqp_work_, P_x_.data(), OSQP_NULL, P_nnz);
        osqp_update_A(osqp_work_, A_x_.data(), OSQP_NULL, A_nnz);
        osqp_update_lin_cost(osqp_work_, q_data_.data());
        osqp_update_bounds(osqp_work_, l_data_.data(), u_data_.data());
    }
    
    return true;
}

void MPCSolver::extractSolution(MPCSolution& solution) {
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    const int N = mpc_params_.N;
    
    const c_float* z = osqp_work_->solution->x;
    
    z_prev_ = Eigen::Map<const Eigen::VectorXd>(z, nz_);
    
    // 控制序列
    solution.u_optimal.resize(N);
    for (int k = 0; k < N; ++k) {
        int u_idx = k * (nx + nu) + nx;
        for (int d = 0; d < nu; ++d) {
            solution.u_optimal[k](d) = z[u_idx + d];
        }
    }
    
    // 状态轨迹
    solution.x_predicted.resize(N + 1);
    for (int k = 0; k <= N; ++k) {
        int x_idx = k * (nx + nu);
        for (int i = 0; i < nx; ++i) {
            solution.x_predicted[k](i) = z[x_idx + i];
        }
    }
    
    solution.u_first = solution.u_optimal[0];
    
    // 全向轮输出：v_x, v_y 从预测状态读取，ω 从控制量读取
    solution.vx_cmd = solution.x_predicted[1](StateIndex::V_X);
    solution.vy_cmd = solution.x_predicted[1](StateIndex::V_Y);
    solution.omega_cmd = solution.u_first(ControlIndex::OMEGA);
}

void MPCSolver::warmStart() {
    if (z_prev_.size() == nz_ && osqp_work_) {
        const int nx = StateIndex::TOTAL_DIM;
        const int nu = ControlIndex::DIM;
        const int N = mpc_params_.N;
        
        Eigen::VectorXd z_init(nz_);
        
        for (int k = 0; k < N - 1; ++k) {
            int src_idx = (k + 1) * (nx + nu);
            int dst_idx = k * (nx + nu);
            z_init.segment(dst_idx, nx + nu) = z_prev_.segment(src_idx, nx + nu);
        }
        
        int last_idx = (N - 1) * (nx + nu);
        z_init.segment(last_idx, nx + nu) = z_prev_.segment(last_idx, nx + nu);
        z_init.segment(N * (nx + nu), nx) = z_prev_.segment(N * (nx + nu), nx);
        
        std::vector<c_float> z_init_data(nz_);
        for (int i = 0; i < nz_; ++i) z_init_data[i] = z_init(i);
        
        osqp_warm_start_x(osqp_work_, z_init_data.data());
    }
}

void MPCSolver::resetWarmStart(bool keep_u_prev) {
    z_prev_.resize(0);
    if (!keep_u_prev) {
        u_prev_.setZero();
    }
    cost_function_.setPreviousControl(u_prev_);
}

}  // namespace scout_omni_local_planner
