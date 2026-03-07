#include "slosh_models/liquid_slosh_model.h"
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace slosh_models {

namespace {
// 贝塞尔函数导数的根 J'₁(ξ₁ₙ) = 0
// 来源: 数值表格 (Abramowitz & Stegun)
const double MODAL_ROOTS[] = {
  1.8412,  // n=1 (第一阶模态)
  5.3314,  // n=2 (第二阶模态)
  8.5363,  // n=3 (第三阶模态)
  11.7060, // n=4
  14.8636  // n=5
};
const int MAX_MODE_INDEX = 5;
}  // namespace

LiquidSloshModel::LiquidSloshModel()
  : configured_(false) {
  state_.setZero();
}

bool LiquidSloshModel::configure(const Params& params) {
  params_ = params;

  // 参数验证
  if (params_.R <= 0.0 || params_.h <= 0.0 || params_.rho <= 0.0) {
    ROS_ERROR("[LiquidSloshModel] Invalid geometry: R=%.3f, h=%.3f, rho=%.1f",
              params_.R, params_.h, params_.rho);
    return false;
  }

  if (params_.dt <= 1e-4) {
    ROS_ERROR("[LiquidSloshModel] dt too small: %.6f", params_.dt);
    return false;
  }

  if (params_.mode_index < 1 || params_.mode_index > MAX_MODE_INDEX) {
    ROS_ERROR("[LiquidSloshModel] mode_index out of range [1,%d]: %d",
              MAX_MODE_INDEX, params_.mode_index);
    return false;
  }

  if (params_.zeta < 0.0 || params_.zeta > 1.0) {
    ROS_WARN("[LiquidSloshModel] Unusual damping ratio: %.3f (typical 0.01~0.1)",
             params_.zeta);
  }

  // === 1. 计算模态根 ξ₁ₙ ===
  modal_.xi_1n = computeModalRoot(params_.mode_index);

  // === 2. 计算液体总质量 m_F ===
  modal_.m_F = params_.rho * M_PI * params_.R * params_.R * params_.h;

  // === 3. 计算模态固有频率 ωₙ (式3) ===
  modal_.omega_n = computeNaturalFrequency(modal_.xi_1n, params_.R, params_.h, params_.g);

  // === 4. 计算模态质量 mₙ (式4) ===
  modal_.m_n = computeModalMass(modal_.xi_1n, modal_.m_F, params_.R, params_.h);

  // === 5. 计算弹簧刚度 kₙ = mₙ·ωₙ² ===
  modal_.k_n = modal_.m_n * modal_.omega_n * modal_.omega_n;

  // === 6. 计算阻尼系数 cₙ = 2·ζₙ·√(kₙ·mₙ) ===
  modal_.c_n = 2.0 * params_.zeta * std::sqrt(modal_.k_n * modal_.m_n);

  // === 7. 计算液面高度系数 ===
  if (params_.use_linear_model) {
    modal_.height_coeff = computeHeightCoeffLinear(params_.h, modal_.m_n,
                                                   modal_.m_F, params_.R);
  } else {
    modal_.height_coeff = computeHeightCoeffNonlinear(modal_.xi_1n, params_.h,
                                                      modal_.m_n, modal_.m_F, params_.R);
  }

  // === 8. 构建连续状态空间矩阵 ===
  buildContinuousMatrices();

  // === 9. 离散化 (ZOH) ===
  discretize();

  // === 10. 重置状态 ===
  reset();

  configured_ = true;

  ROS_INFO("[LiquidSloshModel] 配置成功:");
  ROS_INFO("  容器参数: R=%.3f m, h=%.3f m, rho=%.1f kg/m^3",
           params_.R, params_.h, params_.rho);
  ROS_INFO("  液体质量: m_F=%.3f kg", modal_.m_F);
  ROS_INFO("  模态 %d: xi_1n=%.4f, omega_n=%.3f rad/s (f=%.2f Hz)",
           params_.mode_index, modal_.xi_1n, modal_.omega_n, modal_.omega_n/(2*M_PI));
  ROS_INFO("  模态参数: m_n=%.4f kg, k_n=%.2f N/m, c_n=%.4f N*s/m",
           modal_.m_n, modal_.k_n, modal_.c_n);
  ROS_INFO("  阻尼比: zeta=%.3f", params_.zeta);
  ROS_INFO("  高度系数: %.4f (%s)", modal_.height_coeff,
           params_.use_linear_model ? "线性模型" : "非线性模型");

  return true;
}

void LiquidSloshModel::reset() {
  state_.setZero();
  last_omega_z_ = 0.0;
}

void LiquidSloshModel::setState(const Eigen::Vector4d& state) {
  if (!state.allFinite()) {
    ROS_WARN_THROTTLE(1.0, "[LiquidSloshModel] Ignore non-finite state injection");
    return;
  }
  state_ = state;
}

void LiquidSloshModel::update(const Eigen::Vector2d& accel_base,
                              double omega_z,
                              double alpha_z) {
  if (!configured_) {
    ROS_ERROR_THROTTLE(1.0, "[LiquidSloshModel] update() called before configure()");
    return;
  }

  // 记录最近一次的绕 z 轴角速度（用于 Lp 抛物面高度计算）
  last_omega_z_ = omega_z;

  // === 旋转修正 (式12下方说明) ===
  // 容器处的真实加速度考虑旋转效应:
  //   a_cx = a_x - α_z·r_y - ω_z²·r_x
  //   a_cy = a_y + α_z·r_x - ω_z²·r_y
  double a_cx = accel_base.x() - alpha_z * params_.r_y - omega_z * omega_z * params_.r_x;
  double a_cy = accel_base.y() + alpha_z * params_.r_x - omega_z * omega_z * params_.r_y;

  Eigen::Vector2d accel_container(a_cx, a_cy);

  // === 离散状态更新 ===
  // x_{k+1} = A_d·x_k + B_d·u_k
  state_ = A_discrete_ * state_ + B_discrete_ * accel_container;
}

/**
 * @brief 计算液面晃动高度 (Lp 模型：线性 MSD + 旋转抛物面)
 *
 * 实现论文中的 Lp 模型：
 *   η_total = η_slosh (线性 MSD 晃动) + η_parabola (旋转抛物面静态抬升)
 *
 * 其中：
 *   η_slosh    = height_coeff * sqrt(xₙ² + yₙ²)  [MSD 模态位移 → 高度]
 *   η_parabola = R² * ω_z² / (4g)                [旋转强迫涡的抛物面形状]
 *
 * @return η_total [m] 液面相对静止位置的最大抬升高度
 */
double LiquidSloshModel::getSloshHeight() const {
  if (!configured_) return 0.0;

  // 1. 线性 MSD 模型的晃动高度（论文里的 η_L 或 η_NL 的第一项）
  double x_n = state_(0);
  double y_n = state_(2);
  double r_modal = std::sqrt(x_n * x_n + y_n * y_n);
  double eta_slosh = modal_.height_coeff * r_modal;

  // 2. Lp 模型中的抛物面高度项：η_p = R² * ω² / (4g)
  //    物理意义：容器旋转时，液体表面形成抛物面（强迫涡）
  //    边缘处的静态抬升高度（相对于中心）
  double eta_parabola = 0.0;
  if (params_.use_parabola_term) {
    double R = params_.R;           // 容器半径 [m]
    double g = params_.g;           // 重力加速度 [m/s²]
    double omega = last_omega_z_;   // 最近一次角速度 [rad/s]

    eta_parabola = (R * R * omega * omega) / (4.0 * g);
  }

  // 3. 总高度：Lp = 线性MSD晃动 + 旋转抛物面
  //    直线运动时 ω ≈ 0，抛物面项自动为 0
  //    转弯时随角速度增大，给出额外的升高量
  return eta_slosh + eta_parabola;
}

Eigen::Vector2d LiquidSloshModel::getModalDisplacement() const {
  return Eigen::Vector2d(state_(0), state_(2));
}

Eigen::Vector2d LiquidSloshModel::getModalVelocity() const {
  return Eigen::Vector2d(state_(1), state_(3));
}

// ============ 静态辅助函数 ============

double LiquidSloshModel::computeModalRoot(int n) {
  if (n < 1 || n > MAX_MODE_INDEX) {
    ROS_WARN("[LiquidSloshModel] Mode index %d out of range, using n=1", n);
    return MODAL_ROOTS[0];
  }
  return MODAL_ROOTS[n - 1];
}

double LiquidSloshModel::computeNaturalFrequency(double xi_1n, double R, double h, double g) {
  // 式(3): ωₙ² = g·(ξ₁ₙ/R)·tanh(ξ₁ₙ·h/R)
  double arg = xi_1n * h / R;
  double omega_sq = g * (xi_1n / R) * std::tanh(arg);
  return std::sqrt(std::max(omega_sq, 0.0));
}

double LiquidSloshModel::computeModalMass(double xi_1n, double m_F, double R, double h) {
  // 式(4): mₙ = m_F · [2R / (ξ₁ₙ·h·(ξ₁ₙ² - 1))] · tanh(ξ₁ₙ·h/R)
  double arg = xi_1n * h / R;
  double numerator = 2.0 * R * std::tanh(arg);
  double denominator = xi_1n * h * (xi_1n * xi_1n - 1.0);

  if (std::abs(denominator) < 1e-9) {
    ROS_WARN("[LiquidSloshModel] Near-singular modal mass calculation");
    return 0.0;
  }

  return m_F * numerator / denominator;
}

double LiquidSloshModel::computeHeightCoeffLinear(double h, double m_n, double m_F, double R) {
  // 式(23) L模型: η = (4h·mₙ)/(m_F·R) · √(xₙ² + yₙ²)
  return (4.0 * h * m_n) / (m_F * R);
}

double LiquidSloshModel::computeHeightCoeffNonlinear(double xi_1n, double h,
                                                     double m_n, double m_F, double R) {
  // 式(24) NL模型: η = (ξ₁ₙ²·h·mₙ)/(m_F·R) · √(xₙ² + yₙ²)
  return (xi_1n * xi_1n * h * m_n) / (m_F * R);
}

void LiquidSloshModel::buildContinuousMatrices() {
  // 状态向量: x_f = [xₙ, ẋₙ, yₙ, ẏₙ]^T
  // 输入向量: u_f = [aₓ, aᵧ]^T
  //
  // 状态方程 (式12):
  //   ẍₙ + 2ζₙωₙẋₙ + ωₙ²xₙ = -aₓ
  //   ÿₙ + 2ζₙωₙẏₙ + ωₙ²yₙ = -aᵧ
  //
  // 状态空间形式:
  //   ẋf = Af·xf + Bf·uf
  //
  // Af = [  0          1          0          0      ]
  //      [ -ωₙ²  -2ζₙωₙ       0          0      ]
  //      [  0          0          0          1      ]
  //      [  0          0        -ωₙ²  -2ζₙωₙ  ]
  //
  // Bf = [  0    0 ]
  //      [ -1    0 ]
  //      [  0    0 ]
  //      [  0   -1 ]

  const double omega_n = modal_.omega_n;
  const double two_zeta_omega = 2.0 * params_.zeta * omega_n;
  const double omega_sq = omega_n * omega_n;

  A_continuous_.setZero();
  A_continuous_(0, 1) = 1.0;
  A_continuous_(1, 0) = -omega_sq;
  A_continuous_(1, 1) = -two_zeta_omega;
  A_continuous_(2, 3) = 1.0;
  A_continuous_(3, 2) = -omega_sq;
  A_continuous_(3, 3) = -two_zeta_omega;

  B_continuous_.setZero();
  B_continuous_(1, 0) = -1.0;
  B_continuous_(3, 1) = -1.0;
}

void LiquidSloshModel::discretize() {
  // Zero-Order Hold (ZOH) 离散化:
  //   A_d = e^(A·Ts)
  //   B_d = ∫₀^Ts e^(A·τ) dτ · B
  //
  // 使用 Eigen 的矩阵指数函数

  const double Ts = params_.dt;

  // 方法1: 使用矩阵指数 (精确)
  // 构造增广矩阵 M = [A  B]
  //                  [0  0]
  Eigen::Matrix<double, 6, 6> M;
  M.setZero();
  M.block<4, 4>(0, 0) = A_continuous_ * Ts;
  M.block<4, 2>(0, 4) = B_continuous_ * Ts;

  // 计算 e^M
  Eigen::Matrix<double, 6, 6> EM = M.exp();

  A_discrete_ = EM.block<4, 4>(0, 0);
  B_discrete_ = EM.block<4, 2>(0, 4);

  // 验证数值稳定性
  if (!A_discrete_.allFinite() || !B_discrete_.allFinite()) {
    ROS_ERROR("[LiquidSloshModel] Discretization failed (non-finite matrices)");
    A_discrete_.setIdentity();
    B_discrete_.setZero();
  }
}

}  // namespace slosh_models
