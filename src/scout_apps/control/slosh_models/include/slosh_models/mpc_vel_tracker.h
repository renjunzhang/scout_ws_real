#ifndef SLOSH_MODELS_MPC_VEL_TRACKER_H
#define SLOSH_MODELS_MPC_VEL_TRACKER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <memory>

namespace slosh_models {

// 前向声明
class LiquidSloshModel;

struct MPCResult {
  Eigen::Vector2d command = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector2d> predicted_errors;
  std::vector<double> predicted_slosh_heights;  // 预测的液面晃动高度序列
  double current_slosh_height = 0.0;            // 当前液面晃动高度
  double solve_time_ms = 0.0;
  bool success = false;
};

class MPCVelTracker {
public:
  struct Params {
    double dt = 0.05;            // 采样周期 (s)
    int horizon = 15;            // 预测步数
    Eigen::Vector2d q_pos{4.0, 4.0};   // 位置误差权重 [ex, ey]
    Eigen::Vector2d r_vel{1.0, 1.0};   // 速度输出权重 [vx, vy]
    Eigen::Vector2d s_delta{0.5, 0.5}; // 速度增量权重 [dvx, dvy]
    double max_speed = 0.7;      // 最大速度幅值 (m/s)
    double min_speed = 0.0;      // 最小速度幅值 (m/s)
    bool enforce_min_speed = false; // 是否强制速度下限
    double regularization = 1e-6;   // Hessian 正则化，保持可逆
    bool warm_start = true;      // 是否利用上一周期解作为增量惩罚基准 (速度+晃动状态)

    // === 液体晃动约束参数 ===
    bool enable_slosh = false;   // 是否启用液体晃动约束 (增广状态空间法)
    double q_slosh = 10.0;       // 液面高度惩罚权重 (越大越抑制晃动) - 软约束
    double max_slosh_height = 0.05; // 最大允许液面晃动高度 [m] (参考值，仅用于可视化)
  };

  MPCVelTracker();

  bool configure(const Params& params);
  void reset(const Eigen::Vector2d& last_cmd = Eigen::Vector2d::Zero());

  MPCResult compute(const Eigen::Vector2d& pos_error,
                    const std::vector<Eigen::Vector2d>& ref_vel,
                    double omega_z = 0.0,
                    double alpha_z = 0.0);

  /**
   * @brief 配置液体晃动模型 (必须在 configure() 之后调用)
   * @param slosh_model 外部创建并配置好的液体晃动模型 (共享所有权)
   * @return true 成功, false 失败
   */
  bool setSloshModel(std::shared_ptr<LiquidSloshModel> slosh_model);

  /**
   * @brief 获取当前液体晃动模型
   */
  std::shared_ptr<LiquidSloshModel> getSloshModel() const { return slosh_model_; }

  const Params& params() const { return params_; }
  const Eigen::Vector2d& lastCommand() const { return u_prev_; }

private:
  Params params_;
  bool configured_ = false;

  // === 增广状态空间维度 ===
  int dim_u_ = 0;       // 2 * horizon (速度决策变量)
  int dim_x_ = 0;       // 4 * horizon (晃动状态变量，仅enable_slosh时使用)
  int dim_total_ = 0;   // dim_u_ + dim_x_ (总优化变量维度)

  // === 标准MPC矩阵（仅速度） ===
  std::vector<Eigen::MatrixXd> C_mats_; // 2 x dim_u_
  Eigen::MatrixXd E_mat_;               // 差分矩阵 (dim_u_ x dim_u_)
  Eigen::MatrixXd S_bar_;               // (dim_u_ x dim_u_) block-diagonal of s_delta
  Eigen::MatrixXd H_base_;              // 常量 Hessian (含 2 系数)
  Eigen::MatrixXd H_reg_;               // 正则化后的 Hessian

  // === 增广状态空间矩阵（速度+晃动） ===
  Eigen::MatrixXd H_augmented_;         // (dim_total_ x dim_total_) 增广Hessian
  Eigen::MatrixXd A_eq_;                // 等式约束矩阵 (晃动动态约束)
  Eigen::VectorXd b_eq_;                // 等式约束右端

  Eigen::SparseMatrix<double> H_sparse_;
  Eigen::SparseMatrix<double> constraint_mat_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;

  OsqpEigen::Solver qp_solver_;

  Eigen::VectorXd e_prev_vec_;          // 差分偏移向量
  Eigen::VectorXd last_solution_;       // 上一次求解结果 (warm start)
  Eigen::VectorXd gradient_vec_;
  Eigen::Vector2d u_prev_ = Eigen::Vector2d::Zero();

  // === 液体晃动相关 ===
  std::shared_ptr<LiquidSloshModel> slosh_model_;
  Eigen::Vector4d slosh_state_;         // 当前液体晃动状态 [xn, vxn, yn, vyn]
  double omega_z_prev_ = 0.0;           // 上一时刻角速度

  void buildConstantMatrices();
  void buildAugmentedMatrices();        // 构建增广状态空间矩阵 (速度+晃动)
};

}  // namespace slosh_models

#endif  // SLOSH_MODELS_MPC_VEL_TRACKER_H
