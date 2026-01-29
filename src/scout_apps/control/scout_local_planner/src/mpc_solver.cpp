/**
 * @file mpc_solver.cpp
 * @brief MPC 求解器实现
 * 
 * 适配 OSQP 旧版 API (osqp-vendor in ROS Noetic)
 */

#include "scout_local_planner/mpc_solver.h"
#include "scout_local_planner/diff_drive_model.h"

#include <chrono>
#include <ros/ros.h>

namespace scout_local_planner {

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
    
    // 初始化动力学模型（默认差速）
    dynamics_model_ = std::make_shared<DiffDriveModel>(vehicle_params);
    
    // 初始化代价函数
    cost_function_.initialize(mpc_params);
    
    // 初始化约束管理器
    constraint_manager_.initialize(vehicle_params);
    
    // 计算变量维度
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    const int N = mpc_params_.N;
    
    // z = [x_0, u_0, x_1, u_1, ..., x_{N-1}, u_{N-1}, x_N]
    nz_ = N * (nx + nu) + nx;
    
    initialized_ = true;
    ROS_INFO("[MPCSolver] Initialized with N=%d, dt=%.3f, nz=%d", 
             N, mpc_params_.dt, nz_);
    
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
    
    // 1. 构建 QP 问题
    if (!buildQP(current_state, reference_path)) {
        solution.status_msg = "Failed to build QP";
        return solution;
    }
    
    // 2. 更新/初始化 OSQP
    if (!updateOSQP()) {
        solution.status_msg = "Failed to update OSQP";
        return solution;
    }
    
    // 3. 热启动
    warmStart();
    
    // 4. 求解
    c_int exit_flag = osqp_solve(osqp_work_);
    
    if (exit_flag != 0) {
        solution.status_msg = "OSQP solve failed with exit_flag=" + std::to_string(exit_flag);
        return solution;
    }
    
    // 检查求解状态
    if (osqp_work_->info->status_val != OSQP_SOLVED &&
        osqp_work_->info->status_val != OSQP_SOLVED_INACCURATE) {
        solution.status_msg = "OSQP status: " + std::to_string(osqp_work_->info->status_val);
        return solution;
    }
    
    // 5. 提取解
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
    
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    const int N = mpc_params_.N;
    const double dt = mpc_params_.dt;
    
    // ========== 1. 构建代价函数 ==========
    cost_function_.buildQPCost(N, refs, P_, q_);
    
    // ========== 2. 构建约束 ==========
    // 约束包括：
    // - 动力学约束：x[k+1] = A*x[k] + B*u[k] + c
    // - 边界约束：状态和控制的上下界
    // - 初始条件：x[0] = x0
    
    // 计算动力学约束数量
    int num_dynamics_constraints = N * nx;  // 每步 nx 个等式约束
    int num_initial_constraints = nx;        // 初始状态
    
    // 获取边界约束
    Eigen::SparseMatrix<double> A_bounds;
    Eigen::VectorXd l_bounds, u_bounds;
    constraint_manager_.buildQPConstraints(N, A_bounds, l_bounds, u_bounds);
    int num_bounds_constraints = l_bounds.size();
    
    nc_ = num_initial_constraints + num_dynamics_constraints + num_bounds_constraints;
    
    // 构建完整约束矩阵
    std::vector<Eigen::Triplet<double>> triplets;
    l_ = Eigen::VectorXd::Zero(nc_);
    u_ = Eigen::VectorXd::Zero(nc_);
    
    int row = 0;
    
    // ----- 初始条件约束：x[0] = x0 -----
    for (int i = 0; i < nx; ++i) {
        triplets.emplace_back(row + i, i, 1.0);
        l_(row + i) = x0(i);
        u_(row + i) = x0(i);
    }
    row += nx;
    
    // ----- 动力学约束：x[k+1] = A*x[k] + B*u[k] + c -----
    // 改写为：x[k+1] - A*x[k] - B*u[k] = c
    StateVector x_lin = x0;
    ControlVector u_lin = u_prev_;

    for (int k = 0; k < N; ++k) {
        int x_k_idx = k * (nx + nu);
        int u_k_idx = x_k_idx + nx;
        int x_k1_idx = (k + 1) * (nx + nu);
        
        Eigen::MatrixXd A_dyn, B_dyn;
        Eigen::VectorXd c_dyn;
        dynamics_model_->linearize(x_lin, u_lin, refs[k], dt, A_dyn, B_dyn, c_dyn);
        
        // x[k+1] 的系数：I
        for (int i = 0; i < nx; ++i) {
            triplets.emplace_back(row + i, x_k1_idx + i, 1.0);
        }
        
        // x[k] 的系数：-A（固定稀疏结构，避免 OSQP 反复重建）
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                triplets.emplace_back(row + i, x_k_idx + j, -A_dyn(i, j));
            }
        }
        
        // u[k] 的系数：-B（固定稀疏结构）
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                triplets.emplace_back(row + i, u_k_idx + j, -B_dyn(i, j));
            }
        }
        
        // 等式约束：值为 c
        for (int i = 0; i < nx; ++i) {
            l_(row + i) = c_dyn(i);
            u_(row + i) = c_dyn(i);
        }
        
        row += nx;

        // 更新名义轨迹用于下一步线性化
        x_lin = dynamics_model_->predict(x_lin, u_lin, refs[k], dt);
        u_lin.setZero();
    }
    
    // ----- 边界约束 -----
    // 复制边界约束矩阵
    for (int k = 0; k < A_bounds.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_bounds, k); it; ++it) {
            triplets.emplace_back(row + it.row(), it.col(), it.value());
        }
    }
    l_.segment(row, num_bounds_constraints) = l_bounds;
    u_.segment(row, num_bounds_constraints) = u_bounds;
    
    // 构建稀疏矩阵
    A_.resize(nc_, nz_);
    A_.setFromTriplets(triplets.begin(), triplets.end());
    
    return true;
}

bool MPCSolver::updateOSQP() {
    // 确保矩阵是压缩格式
    Eigen::SparseMatrix<double> P_upper = P_.triangularView<Eigen::Upper>();
    P_upper.makeCompressed();
    A_.makeCompressed();
    
    // 转换为 C 数组格式
    // P 矩阵
    P_p_.assign(P_upper.outerIndexPtr(), P_upper.outerIndexPtr() + P_upper.cols() + 1);
    P_i_.assign(P_upper.innerIndexPtr(), P_upper.innerIndexPtr() + P_upper.nonZeros());
    P_x_.assign(P_upper.valuePtr(), P_upper.valuePtr() + P_upper.nonZeros());
    
    // A 矩阵
    A_p_.assign(A_.outerIndexPtr(), A_.outerIndexPtr() + A_.cols() + 1);
    A_i_.assign(A_.innerIndexPtr(), A_.innerIndexPtr() + A_.nonZeros());
    A_x_.assign(A_.valuePtr(), A_.valuePtr() + A_.nonZeros());
    
    // 向量
    q_data_.resize(q_.size());
    l_data_.resize(l_.size());
    u_data_.resize(u_.size());
    for (int i = 0; i < q_.size(); ++i) q_data_[i] = q_(i);
    for (int i = 0; i < l_.size(); ++i) l_data_[i] = l_(i);
    for (int i = 0; i < u_.size(); ++i) u_data_[i] = u_(i);
    
    if (!osqp_initialized_) {
        // 首次初始化
        cleanupOSQP();
        
        // 分配设置
        osqp_settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
        osqp_set_default_settings(osqp_settings_);
        osqp_settings_->verbose = false;
        osqp_settings_->warm_start = true;
        osqp_settings_->polish = true;
        osqp_settings_->eps_abs = 1e-4;
        osqp_settings_->eps_rel = 1e-4;
        osqp_settings_->max_iter = 4000;
        
        // 分配数据
        osqp_data_ = (OSQPData*)c_malloc(sizeof(OSQPData));
        osqp_data_->n = nz_;
        osqp_data_->m = nc_;
        
        // P 矩阵（CSC 格式）
        osqp_data_->P = csc_matrix(nz_, nz_, P_x_.size(),
                                    P_x_.data(), P_i_.data(), P_p_.data());
        
        // A 矩阵（CSC 格式）
        osqp_data_->A = csc_matrix(nc_, nz_, A_x_.size(),
                                    A_x_.data(), A_i_.data(), A_p_.data());
        
        osqp_data_->q = q_data_.data();
        osqp_data_->l = l_data_.data();
        osqp_data_->u = u_data_.data();
        
        // 设置求解器
        c_int exit_flag = osqp_setup(&osqp_work_, osqp_data_, osqp_settings_);
        
        if (exit_flag != 0) {
            ROS_ERROR("[MPCSolver] OSQP setup failed: %d", static_cast<int>(exit_flag));
            return false;
        }
        
        osqp_initialized_ = true;
    } else {
        // 更新现有问题 - 每个周期都需要更新 P, A, q, l, u
        // 因为线性化和参考点每周期都变化
        
        // 更新 P 矩阵（仅更新数值，结构不变）
        // 注意：如果稀疏结构变化，需要重新 setup
        c_int P_nnz = static_cast<c_int>(P_x_.size());
        c_int A_nnz = static_cast<c_int>(A_x_.size());
        
        // 检查结构是否变化
        if (P_nnz != osqp_data_->P->nzmax || A_nnz != osqp_data_->A->nzmax) {
            // 结构变化，需要重新初始化
            ROS_WARN_THROTTLE(1.0, "[MPCSolver] Matrix structure changed, reinitializing OSQP");
            cleanupOSQP();
            osqp_initialized_ = false;
            return updateOSQP();  // 递归调用重新初始化
        }
        
        // 更新 P 矩阵数值
        osqp_update_P(osqp_work_, P_x_.data(), OSQP_NULL, P_nnz);
        
        // 更新 A 矩阵数值
        osqp_update_A(osqp_work_, A_x_.data(), OSQP_NULL, A_nnz);
        
        // 更新线性项和边界
        osqp_update_lin_cost(osqp_work_, q_data_.data());
        osqp_update_bounds(osqp_work_, l_data_.data(), u_data_.data());
    }
    
    return true;
}

void MPCSolver::extractSolution(MPCSolution& solution) {
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    const int N = mpc_params_.N;
    
    // 获取解向量
    const c_float* z = osqp_work_->solution->x;
    
    // 保存用于热启动
    z_prev_ = Eigen::Map<const Eigen::VectorXd>(z, nz_);
    
    // 提取控制序列
    solution.u_optimal.resize(N);
    for (int k = 0; k < N; ++k) {
        int u_idx = k * (nx + nu) + nx;
        solution.u_optimal[k](ControlIndex::A) = z[u_idx];
        solution.u_optimal[k](ControlIndex::ANG_ACC) = z[u_idx + 1];
    }
    
    // 提取状态轨迹
    solution.x_predicted.resize(N + 1);
    for (int k = 0; k <= N; ++k) {
        int x_idx = k * (nx + nu);
        for (int i = 0; i < nx; ++i) {
            solution.x_predicted[k](i) = z[x_idx + i];
        }
    }
    
    // 第一个控制量
    solution.u_first = solution.u_optimal[0];
    
    // 输出速度（从预测的下一步状态读取）
    // 注意：cmd_vel = [v, ω]，不是 [a, α]！
    solution.v_cmd = solution.x_predicted[1](StateIndex::V);
    solution.omega_cmd = solution.x_predicted[1](StateIndex::OMEGA);
}

void MPCSolver::warmStart() {
    if (z_prev_.size() == nz_ && osqp_work_) {
        // 将上一次解向前移动一步作为初始猜测
        const int nx = StateIndex::TOTAL_DIM;
        const int nu = ControlIndex::DIM;
        const int N = mpc_params_.N;
        
        Eigen::VectorXd z_init(nz_);
        
        // 移动：z_init[k] = z_prev[k+1]
        for (int k = 0; k < N - 1; ++k) {
            int src_idx = (k + 1) * (nx + nu);
            int dst_idx = k * (nx + nu);
            z_init.segment(dst_idx, nx + nu) = z_prev_.segment(src_idx, nx + nu);
        }
        
        // 最后一步用上一次的最后一步
        int last_idx = (N - 1) * (nx + nu);
        z_init.segment(last_idx, nx + nu) = z_prev_.segment(last_idx, nx + nu);
        z_init.segment(N * (nx + nu), nx) = z_prev_.segment(N * (nx + nu), nx);
        
        std::vector<c_float> z_init_data(nz_);
        for (int i = 0; i < nz_; ++i) z_init_data[i] = z_init(i);
        
        osqp_warm_start_x(osqp_work_, z_init_data.data());
    }
}

}  // namespace scout_local_planner
