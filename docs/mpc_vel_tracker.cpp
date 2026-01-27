#include "communication_rs485/mpc_vel_tracker.h"
#include "communication_rs485/liquid_slosh_model.h"

#include <ros/ros.h>
#include <algorithm>
#include <chrono>

namespace communication_rs485 {

namespace {
inline Eigen::Matrix2d makeDiag(const Eigen::Vector2d& v) {
  Eigen::Matrix2d m = Eigen::Matrix2d::Zero();
  m(0, 0) = v.x();
  m(1, 1) = v.y();
  return m;
}

inline Eigen::SparseMatrix<double> denseToSparse(const Eigen::MatrixXd& dense) {
  Eigen::SparseMatrix<double> sparse = dense.sparseView(1e-12, 0);
  sparse.makeCompressed();
  return sparse;
}
}  // namespace

MPCVelTracker::MPCVelTracker() = default;

bool MPCVelTracker::configure(const Params& params) {
  if (params.horizon <= 0) {
    ROS_ERROR("[MPCVelTracker] horizon must be positive");
    return false;
  }
  if (params.dt <= 1e-4) {
    ROS_ERROR("[MPCVelTracker] dt must be larger than 1e-4");
    return false;
  }

  params_ = params;
  dim_u_ = 2 * params_.horizon;  // 速度决策变量维度
  dim_x_ = 4 * params_.horizon;  // 晃动状态变量维度
  
  // 决定总优化变量维度
  if (params_.enable_slosh) {
    dim_total_ = dim_u_ + dim_x_;  // 增广状态空间
    ROS_INFO("[MPCVelTracker] 配置增广状态空间MPC (速度 %d + 晃动状态 %d = 总维度 %d)",
             dim_u_, dim_x_, dim_total_);
  } else {
    dim_total_ = dim_u_;  // 仅速度优化
    ROS_INFO("[MPCVelTracker] 配置标准MPC (仅速度优化，维度 %d)", dim_total_);
  }

  buildConstantMatrices();

  H_reg_ = H_base_;
  H_reg_ += params_.regularization * Eigen::MatrixXd::Identity(dim_u_, dim_u_);

  last_solution_ = Eigen::VectorXd::Zero(dim_total_);
  e_prev_vec_ = Eigen::VectorXd::Zero(dim_u_ + 2);  // 包含 u_{-1}，维度为 dim_u_ + 2
  u_prev_.setZero();
  slosh_state_.setZero();
  omega_z_prev_ = 0.0;

  // 如果启用晃动抑制但还未绑定模型，延迟初始化QP求解器
  if (params_.enable_slosh && !slosh_model_) {
    ROS_WARN("[MPCVelTracker] enable_slosh=true 但液体晃动模型未绑定，等待 setSloshModel()");
    configured_ = false;
    return true;  // 允许稍后绑定模型
  }

  // 构建QP问题矩阵
  if (params_.enable_slosh && slosh_model_) {
    buildAugmentedMatrices();
  } else {
    H_sparse_ = denseToSparse(H_reg_);
    constraint_mat_ = Eigen::SparseMatrix<double>(dim_total_, dim_total_);
    constraint_mat_.setIdentity();
    constraint_mat_.makeCompressed();

    double vmax = std::max(params_.max_speed, 1e-6);
    lower_bound_ = Eigen::VectorXd::Constant(dim_total_, -vmax);
    upper_bound_ = Eigen::VectorXd::Constant(dim_total_, vmax);
  }

  gradient_vec_ = Eigen::VectorXd::Zero(dim_total_);

  // 初始化QP求解器
  qp_solver_.clearSolver();
  qp_solver_.settings()->setWarmStart(params_.warm_start);
  qp_solver_.settings()->setVerbosity(false);

  qp_solver_.data()->setNumberOfVariables(dim_total_);
  qp_solver_.data()->setNumberOfConstraints(constraint_mat_.rows());

  if (!qp_solver_.data()->setHessianMatrix(H_sparse_)) {
    ROS_ERROR("[MPCVelTracker] Failed to set Hessian matrix");
    configured_ = false;
    return false;
  }
  if (!qp_solver_.data()->setGradient(gradient_vec_)) {
    ROS_ERROR("[MPCVelTracker] Failed to set gradient vector");
    configured_ = false;
    return false;
  }
  if (!qp_solver_.data()->setLinearConstraintsMatrix(constraint_mat_)) {
    ROS_ERROR("[MPCVelTracker] Failed to set constraint matrix");
    configured_ = false;
    return false;
  }
  if (!qp_solver_.data()->setLowerBound(lower_bound_)) {
    ROS_ERROR("[MPCVelTracker] Failed to set lower bounds");
    configured_ = false;
    return false;
  }
  if (!qp_solver_.data()->setUpperBound(upper_bound_)) {
    ROS_ERROR("[MPCVelTracker] Failed to set upper bounds");
    configured_ = false;
    return false;
  }

  if (!qp_solver_.initSolver()) {
    ROS_ERROR("[MPCVelTracker] Failed to initialize OSQP solver");
    configured_ = false;
    return false;
  }

  configured_ = true;
  return true;
}

void MPCVelTracker::reset(const Eigen::Vector2d& last_cmd) {
  u_prev_ = last_cmd;
  if (dim_total_ > 0) {
    last_solution_.setZero(dim_total_);
  }
  slosh_state_.setZero();
  omega_z_prev_ = 0.0;
  if (slosh_model_) {
    slosh_model_->reset();
  }
}

bool MPCVelTracker::setSloshModel(std::shared_ptr<LiquidSloshModel> slosh_model) {
  if (!slosh_model || !slosh_model->isConfigured()) {
    ROS_ERROR("[MPCVelTracker] Invalid or unconfigured slosh model");
    return false;
  }
  
  slosh_model_ = slosh_model;
  slosh_state_ = slosh_model_->getState();
  
  ROS_INFO("[MPCVelTracker] 晃动模型已绑定 (模态 %d, ωn=%.3f rad/s)",
           slosh_model_->getParams().mode_index,
           slosh_model_->getModalParams().omega_n);
  
  // 如果启用晃动抑制且已配置基础矩阵，构建增广状态空间
  if (params_.enable_slosh && !configured_) {
    // 首次绑定模型，完成配置
    buildAugmentedMatrices();
    
    gradient_vec_ = Eigen::VectorXd::Zero(dim_total_);
    
    // 初始化QP求解器
    qp_solver_.clearSolver();
    qp_solver_.settings()->setWarmStart(params_.warm_start);
    qp_solver_.settings()->setVerbosity(false);

    qp_solver_.data()->setNumberOfVariables(dim_total_);
    qp_solver_.data()->setNumberOfConstraints(constraint_mat_.rows());

    if (!qp_solver_.data()->setHessianMatrix(H_sparse_) ||
        !qp_solver_.data()->setGradient(gradient_vec_) ||
        !qp_solver_.data()->setLinearConstraintsMatrix(constraint_mat_) ||
        !qp_solver_.data()->setLowerBound(lower_bound_) ||
        !qp_solver_.data()->setUpperBound(upper_bound_)) {
      ROS_ERROR("[MPCVelTracker] Failed to setup augmented QP problem");
      return false;
    }

    if (!qp_solver_.initSolver()) {
      ROS_ERROR("[MPCVelTracker] Failed to initialize augmented OSQP solver");
      return false;
    }
    
    configured_ = true;
    ROS_INFO("[MPCVelTracker] ✓ 增广状态空间MPC初始化完成");
  }
  
  return true;
}

MPCResult MPCVelTracker::compute(const Eigen::Vector2d& pos_error,
                                 const std::vector<Eigen::Vector2d>& ref_vel,
                                 double omega_z,
                                 double alpha_z) {
  MPCResult result;
  if (!configured_) {
    ROS_ERROR_THROTTLE(1.0, "[MPCVelTracker] compute() called before configure()");
    return result;
  }
  if (static_cast<int>(ref_vel.size()) < params_.horizon) {
    ROS_ERROR_THROTTLE(1.0, "[MPCVelTracker] reference velocity horizon too short (%zu)",
                       ref_vel.size());
    return result;
  }

  auto tic = std::chrono::steady_clock::now();

  // ========== 增广状态空间法：同时优化速度和晃动状态 ==========
  if (params_.enable_slosh && slosh_model_) {
    // 决策变量 z = [u_0...u_{N-1}, x_0...x_{N-1}]
    // 其中 u_j ∈ R^2 (速度), x_j ∈ R^4 (晃动状态)
    
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(dim_total_);
    const Eigen::Matrix2d Q = makeDiag(params_.q_pos);
    
    // 位置误差梯度贡献（作用于速度变量）
    Eigen::Vector2d accum_ref = Eigen::Vector2d::Zero();
    for (int j = 0; j < params_.horizon; ++j) {
      accum_ref += ref_vel[j] * params_.dt;
      const Eigen::Vector2d target = pos_error + accum_ref;
      // C_mats_[j].transpose() * Q * target 是 dim_u_ 维向量
      // 累加到速度部分的梯度
      grad.segment(0, dim_u_) += -2.0 * C_mats_[j].transpose() * Q * target;
    }
    
    // 差分项线性部分（仅作用于速度）
    e_prev_vec_.setZero();
    e_prev_vec_.segment<2>(0) = u_prev_;
    grad.segment(0, dim_u_) += -2.0 * E_mat_.transpose() * S_bar_ * e_prev_vec_;
    
    // 晃动高度惩罚梯度（作用于晃动状态变量）
    const double q_slosh = params_.q_slosh;
    const auto& modal = slosh_model_->getModalParams();
    const double h_coeff = modal.height_coeff;
    
    for (int j = 0; j < params_.horizon; ++j) {
      // 晃动状态变量位于 [dim_u_ + 4*j, dim_u_ + 4*j + 4)
      // 梯度: ∂(q_slosh * η²)/∂x = 2*q_slosh*η * ∂η/∂x
      // 其中 η = h_coeff * sqrt(xn² + yn²)
      // 从当前解提取晃动状态（warm start）
      Eigen::Vector4d x_j;
      if (params_.warm_start && last_solution_.size() == dim_total_) {
        x_j = last_solution_.segment<4>(dim_u_ + 4 * j);
      } else {
        x_j = slosh_state_;  // 初始化为当前状态
      }
      
      double xn = x_j(0);
      double yn = x_j(2);
      double r_slosh = std::sqrt(xn*xn + yn*yn + 1e-12);
      double eta = h_coeff * r_slosh;
      
      // ∂η/∂xn = h_coeff * xn / r_slosh, ∂η/∂yn = h_coeff * yn / r_slosh
      // 梯度贡献到 xn 和 yn 分量
      grad(dim_u_ + 4*j + 0) += 2.0 * q_slosh * eta * h_coeff * xn / r_slosh;  // ∂/∂xn
      grad(dim_u_ + 4*j + 2) += 2.0 * q_slosh * eta * h_coeff * yn / r_slosh;  // ∂/∂yn
    }
    
    gradient_vec_ = grad;
    
    // 更新等式约束右端（晃动动态约束）
    // 约束: x_{j+1} = Ad*x_j + Bd*(u_j - u_{j-1})/dt + Cd*omega_z
    Eigen::Matrix4d Ad;
    Eigen::Matrix<double, 4, 2> Bd;
    slosh_model_->getDiscreteMatrices(Ad, Bd);
    
    // 更新 b_eq（包含当前状态和预测角速度）
    // 假设角速度恒定（简化模型）: omega_z[j] = omega_z
    b_eq_.segment<4>(0) = Ad * slosh_state_ + Bd * (-u_prev_ / params_.dt);
    
    // 预测角速度变化（恒定角速度模型）
    for (int j = 1; j < params_.horizon; ++j) {
      b_eq_.segment<4>(4 * j).setZero();  // 其他项已在A_eq中编码
    }
    
    // 将更新后的等式约束同步到完整的约束边界向量
    const int num_eq = dim_x_;  // 60个等式约束
    lower_bound_.segment(0, num_eq) = b_eq_;
    upper_bound_.segment(0, num_eq) = b_eq_;
    
    // 更新约束右端
    if (!qp_solver_.updateBounds(lower_bound_, upper_bound_)) {
      ROS_ERROR_THROTTLE(1.0, "[MPCVelTracker] Failed to update bounds");
      return result;
    }
    
  } else {
    // ========== 标准MPC：仅优化速度 ==========
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(dim_total_);
    const Eigen::Matrix2d Q = makeDiag(params_.q_pos);

    Eigen::Vector2d accum_ref = Eigen::Vector2d::Zero();
    for (int j = 0; j < params_.horizon; ++j) {
      accum_ref += ref_vel[j] * params_.dt;
      const Eigen::Vector2d target = pos_error + accum_ref;
      // 累加位置误差梯度贡献
      grad += -2.0 * C_mats_[j].transpose() * Q * target;
    }

    // 差分项线性部分（作用于整个速度向量）
    e_prev_vec_.setZero();
    e_prev_vec_.segment<2>(0) = u_prev_;
    grad += -2.0 * E_mat_.transpose() * S_bar_ * e_prev_vec_;
    
    gradient_vec_ = grad;
  }

  // 求解QP问题
  if (!qp_solver_.updateGradient(gradient_vec_)) {
    ROS_ERROR_THROTTLE(1.0, "[MPCVelTracker] Failed to update QP gradient");
    return result;
  }

  OsqpEigen::ErrorExitFlag flag = qp_solver_.solveProblem();
  if (flag != OsqpEigen::ErrorExitFlag::NoError) {
    ROS_WARN_THROTTLE(1.0, "[MPCVelTracker] QP solver failed (flag %d)", static_cast<int>(flag));
    return result;
  }

  Eigen::VectorXd sol = qp_solver_.getSolution();
  Eigen::Vector2d cmd = sol.segment<2>(0);

  // 约束：最大/最小速度幅值
  double max_v = std::max(params_.max_speed, 1e-6);
  double norm = cmd.norm();
  if (norm > max_v) {
    cmd *= (max_v / norm);
  }
  if (params_.enforce_min_speed && norm > 1e-6 && norm < params_.min_speed) {
    cmd *= (params_.min_speed / norm);
  }

  // 生成预测误差轨迹和液体晃动轨迹
  result.predicted_errors.reserve(params_.horizon);
  result.predicted_slosh_heights.reserve(params_.horizon);
  
  Eigen::Vector2d err = pos_error;
  
  for (int j = 0; j < params_.horizon; ++j) {
    Eigen::Vector2d uj = sol.segment<2>(2 * j);
    double n = uj.norm();
    if (n > max_v) {
      uj *= (max_v / n);
    }
    if (params_.enforce_min_speed && n > 1e-6 && n < params_.min_speed) {
      uj *= (params_.min_speed / n);
    }
    err = err + params_.dt * (ref_vel[j] - uj);
    result.predicted_errors.push_back(err);
    
    // 提取预测的晃动高度（从增广状态变量中）
    if (params_.enable_slosh && slosh_model_) {
      Eigen::Vector4d x_j = sol.segment<4>(dim_u_ + 4 * j);
      double xn = x_j(0);
      double yn = x_j(2);
      double eta = slosh_model_->getModalParams().height_coeff * std::sqrt(xn*xn + yn*yn);
      result.predicted_slosh_heights.push_back(eta);
    }
  }

  // 更新液体晃动状态（使用增广状态空间的第一步预测值）
  if (params_.enable_slosh && slosh_model_) {
    slosh_state_ = sol.segment<4>(dim_u_);  // 提取第一步的晃动状态预测
    result.current_slosh_height = slosh_model_->getModalParams().height_coeff * 
                                  std::sqrt(slosh_state_(0)*slosh_state_(0) + 
                                           slosh_state_(2)*slosh_state_(2));
  }

  // Warm start：保存当前解
  if (params_.warm_start) {
    last_solution_ = sol;
  }
  
  u_prev_ = cmd;  // 更新上一次速度
  omega_z_prev_ = omega_z;  // 更新上一次角速度
  
  auto toc = std::chrono::steady_clock::now();
  result.solve_time_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(toc - tic).count();
  result.command = cmd;
  result.success = true;
  return result;
}

void MPCVelTracker::buildConstantMatrices() {
  const Eigen::Matrix2d Q = makeDiag(params_.q_pos);
  const Eigen::Matrix2d R = makeDiag(params_.r_vel);
  const Eigen::Matrix2d S = makeDiag(params_.s_delta);

  C_mats_.clear();
  C_mats_.resize(params_.horizon);

  for (int j = 0; j < params_.horizon; ++j) {
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(2, dim_u_);
    for (int i = 0; i <= j; ++i) {
      C.block<2, 2>(0, 2 * i) = params_.dt * Eigen::Matrix2d::Identity();
    }
    C_mats_[j] = C;
  }

  // 差分矩阵 E (维度: (dim_u_ + 2) × dim_u_，包含 u_{-1} 项)
  E_mat_ = Eigen::MatrixXd::Zero(dim_u_ + 2, dim_u_);
  for (int j = 0; j < params_.horizon; ++j) {
    if (j == 0) {
      E_mat_.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
    } else {
      E_mat_.block<2, 2>(2 * j, 2 * j) = Eigen::Matrix2d::Identity();
      E_mat_.block<2, 2>(2 * j, 2 * (j - 1)) = -Eigen::Matrix2d::Identity();
    }
  }

  // S_bar：block diagonal of S (维度: (dim_u_ + 2) × (dim_u_ + 2))
  S_bar_ = Eigen::MatrixXd::Zero(dim_u_ + 2, dim_u_ + 2);
  for (int j = 0; j < params_.horizon; ++j) {
    S_bar_.block<2, 2>(2 * j, 2 * j) = S;
  }

  H_base_ = Eigen::MatrixXd::Zero(dim_u_, dim_u_);

  for (int j = 0; j < params_.horizon; ++j) {
    const Eigen::MatrixXd& C = C_mats_[j];
    H_base_ += 2.0 * (C.transpose() * Q * C);
  }

  for (int j = 0; j < params_.horizon; ++j) {
    H_base_.block<2, 2>(2 * j, 2 * j) += 2.0 * R;
  }

  Eigen::MatrixXd Hs = E_mat_.transpose() * S_bar_ * E_mat_;
  H_base_ += 2.0 * Hs;
}

void MPCVelTracker::buildAugmentedMatrices() {
  if (!params_.enable_slosh || !slosh_model_) {
    ROS_ERROR("[MPCVelTracker] buildAugmentedMatrices called without slosh model");
    return;
  }
  
  // 增广状态空间维度: z = [u_0...u_{N-1}, x_0...x_{N-1}]
  // dim_total_ = dim_u_ + dim_x_ = 2*N + 4*N = 6*N
  
  const Eigen::Matrix2d Q = makeDiag(params_.q_pos);
  const Eigen::Matrix2d R = makeDiag(params_.r_vel);
  const Eigen::Matrix2d S = makeDiag(params_.s_delta);
  const double q_slosh = params_.q_slosh;
  
  // 获取离散晃动动态矩阵
  Eigen::Matrix4d Ad;
  Eigen::Matrix<double, 4, 2> Bd;
  slosh_model_->getDiscreteMatrices(Ad, Bd);
  
  // ========== 构建增广Hessian矩阵 ==========
  // H = [ H_uu  H_ux ]  (dim_total_ x dim_total_)
  //     [ H_xu  H_xx ]
  
  H_augmented_ = Eigen::MatrixXd::Zero(dim_total_, dim_total_);
  
  // H_uu: 速度变量的Hessian（与标准MPC相同）
  H_augmented_.block(0, 0, dim_u_, dim_u_) = H_base_;
  
  // H_xx: 晃动状态变量的Hessian（晃动高度惩罚）
  // 成本: Σ q_slosh * η_j² = Σ q_slosh * h_coeff² * (xn_j² + yn_j²)
  // Hessian对角块: ∂²/∂x_j² = 2*q_slosh*diag([h_coeff², 0, h_coeff², 0])
  const double h_coeff = slosh_model_->getModalParams().height_coeff;
  Eigen::Matrix4d H_slosh_diag = Eigen::Matrix4d::Zero();
  H_slosh_diag(0, 0) = 2.0 * q_slosh * h_coeff * h_coeff;  // xn
  H_slosh_diag(2, 2) = 2.0 * q_slosh * h_coeff * h_coeff;  // yn
  
  for (int j = 0; j < params_.horizon; ++j) {
    H_augmented_.block<4, 4>(dim_u_ + 4*j, dim_u_ + 4*j) = H_slosh_diag;
  }
  
  // H_ux 和 H_xu: 速度-晃动耦合项（线性模型下为零，这里保持为零）
  // （如果需要考虑加速度-晃动的二阶耦合，可以在此添加）
  
  // 正则化
  H_augmented_ += params_.regularization * Eigen::MatrixXd::Identity(dim_total_, dim_total_);
  H_sparse_ = denseToSparse(H_augmented_);
  
  // ========== 构建等式约束矩阵（晃动动态约束）==========
  // 约束: x_{j+1} = Ad*x_j + Bd*(u_j - u_{j-1})/dt, j=0...N-1
  // 形式: A_eq * z = b_eq
  // 其中 z = [u, x], A_eq的结构为:
  // [-Bd/dt,  Ad,  -I,   0,  0, ...]  第0行: x_0 = Ad*x_{-1} + Bd*(-u_{-1}/dt) + Bd*(u_0/dt)
  // [Bd/dt, -Bd/dt,  0,  Ad, -I, ...]  第1行: x_1 = Ad*x_0 + Bd*(u_1 - u_0)/dt
  // ...
  
  const int num_eq = 4 * params_.horizon;  // 每一步4个等式约束
  A_eq_ = Eigen::MatrixXd::Zero(num_eq, dim_total_);
  b_eq_ = Eigen::VectorXd::Zero(num_eq);
  
  Eigen::Matrix<double, 4, 2> Bd_dt = Bd / params_.dt;
  
  for (int j = 0; j < params_.horizon; ++j) {
    // 第 j 步约束: x_j = Ad*x_{j-1} + Bd*(u_j - u_{j-1})/dt
    // 重新排列: -Bd/dt * u_j + Bd/dt * u_{j-1} + Ad * x_{j-1} - I * x_j = 0
    
    // 速度项
    A_eq_.block(4*j, 2*j, 4, 2) = -Bd_dt;  // -Bd/dt * u_j
    if (j > 0) {
      A_eq_.block(4*j, 2*(j-1), 4, 2) = Bd_dt;  // Bd/dt * u_{j-1}
    }
    
    // 晃动状态项
    if (j > 0) {
      A_eq_.block(4*j, dim_u_ + 4*(j-1), 4, 4) = Ad;  // Ad * x_{j-1}
    }
    A_eq_.block(4*j, dim_u_ + 4*j, 4, 4) = -Eigen::Matrix4d::Identity();  // -I * x_j
  }
  
  // 右端向量 b_eq 将在 compute() 中动态更新（包含当前状态和上一次速度）
  
  // ========== 构建不等式约束（速度界和晃动状态界）==========
  // 使用简单的盒约束: lb <= z <= ub
  double vmax = std::max(params_.max_speed, 1e-6);
  lower_bound_ = Eigen::VectorXd::Constant(dim_total_, -vmax);
  upper_bound_ = Eigen::VectorXd::Constant(dim_total_, vmax);
  
  // 晃动状态约束（设置为较大范围，软约束通过Hessian实现）
  const double x_max = 0.5;  // 最大晃动位移 [m]
  const double v_max_slosh = 2.0;  // 最大晃动速度 [m/s]
  for (int j = 0; j < params_.horizon; ++j) {
    lower_bound_(dim_u_ + 4*j + 0) = -x_max;    // xn
    upper_bound_(dim_u_ + 4*j + 0) = x_max;
    lower_bound_(dim_u_ + 4*j + 1) = -v_max_slosh;  // vxn
    upper_bound_(dim_u_ + 4*j + 1) = v_max_slosh;
    lower_bound_(dim_u_ + 4*j + 2) = -x_max;    // yn
    upper_bound_(dim_u_ + 4*j + 2) = x_max;
    lower_bound_(dim_u_ + 4*j + 3) = -v_max_slosh;  // vyn
    upper_bound_(dim_u_ + 4*j + 3) = v_max_slosh;
  }
  
  // ========== 合并等式约束和不等式约束 ==========
  // OSQP格式: lb <= A * z <= ub
  // 等式约束: A_eq * z = b_eq  =>  b_eq <= A_eq * z <= b_eq
  // 不等式约束: lb <= I * z <= ub
  
  const int num_constraints = num_eq + dim_total_;
  Eigen::MatrixXd A_combined = Eigen::MatrixXd::Zero(num_constraints, dim_total_);
  A_combined.block(0, 0, num_eq, dim_total_) = A_eq_;
  A_combined.block(num_eq, 0, dim_total_, dim_total_) = Eigen::MatrixXd::Identity(dim_total_, dim_total_);
  
  constraint_mat_ = denseToSparse(A_combined);
  
  // 边界向量（等式约束部分在 compute() 中更新）
  Eigen::VectorXd lb_combined(num_constraints);
  Eigen::VectorXd ub_combined(num_constraints);
  lb_combined.segment(0, num_eq) = b_eq_;
  ub_combined.segment(0, num_eq) = b_eq_;
  lb_combined.segment(num_eq, dim_total_) = lower_bound_;
  ub_combined.segment(num_eq, dim_total_) = upper_bound_;
  
  lower_bound_ = lb_combined;
  upper_bound_ = ub_combined;
  
  ROS_INFO("[MPCVelTracker] 增广状态空间矩阵构建完成:");
  ROS_INFO("  - Hessian: %dx%d", dim_total_, dim_total_);
  ROS_INFO("  - 等式约束(晃动动态): %d", num_eq);
  ROS_INFO("  - 不等式约束(速度界+状态界): %d", dim_total_);
  ROS_INFO("  - 总约束: %d", num_constraints);
}

}  // namespace communication_rs485
