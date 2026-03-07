#ifndef SLOSH_MODELS_LIQUID_SLOSH_MODEL_H
#define SLOSH_MODELS_LIQUID_SLOSH_MODEL_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace slosh_models {

/**
 * @brief 液体晃动动力学模型 (基于2D Mass-Spring-Damper等效模型)
 *
 * 参考论文: "Sloshing Dynamics Estimation for Liquid-filled Containers
 *           under 2-Dimensional Excitation" (ECCOMAS 2021)
 *
 * 建模思路:
 * - 液体被分解为刚体质量m0和振动模态质量mn
 * - 每个模态n对应一个2D弹簧-阻尼系统
 * - 输入: 容器加速度 [ax, ay]
 * - 状态: [xn, vxn, yn, vyn] (模态位移和速度)
 * - 输出: 液面晃动高度 ηn
 *
 * 动力学方程 (连续时间):
 *   ẍn + 2ζnωnẋn + ωn²xn = -ax
 *   ÿn + 2ζnωnẏn + ωn²yn = -ay
 *
 * 状态空间形式:
 *   ẋf = Af·xf + Bf·uf
 *   其中 xf = [xn, ẋn, yn, ẏn]^T
 *        uf = [ax, ay]^T
 */
class LiquidSloshModel {
public:
  /**
   * @brief 液体和容器参数
   */
  struct Params {
    // === 几何参数 (必须提供) ===
    double R = 0.15;        ///< 容器内半径 [m]
    double h = 0.20;        ///< 液体静液高度 [m]
    double rho = 1000.0;    ///< 液体密度 [kg/m³] (水=1000)

    // === 控制参数 ===
    double dt = 0.05;       ///< 离散化采样周期 [s]
    int mode_index = 1;     ///< 使用的模态阶数 (1=第一阶, 2=第二阶...)
    double zeta = 0.05;     ///< 阻尼比 (经验值 0.01~0.1, 水通常0.05)

    // === 旋转修正参数 ===
    double r_x = 0.0;       ///< 容器相对底盘旋转中心的偏心距 x [m]
    double r_y = 0.0;       ///< 容器相对底盘旋转中心的偏心距 y [m]

    // === 物理常量 ===
    double g = 9.81;        ///< 重力加速度 [m/s²]

    // === 输出选项 ===
    bool use_linear_model = true;  ///< true=线性模型(L), false=非线性模型(NL)

    // === Lp 模型：旋转抛物面高度项 ===
    bool use_parabola_term = true;  ///< 是否启用旋转抛物面高度修正 (Lp 模型)
    // 注：抛物面高度计算需要 R 和 g，已在上方定义
  };

  /**
   * @brief 模态参数 (由configure()自动计算)
   */
  struct ModalParams {
    double xi_1n = 0.0;      ///< 贝塞尔函数根 ξ1n
    double omega_n = 0.0;    ///< 模态固有频率 [rad/s]
    double m_n = 0.0;        ///< 模态质量 [kg]
    double m_F = 0.0;        ///< 液体总质量 [kg]
    double k_n = 0.0;        ///< 等效弹簧刚度 [N/m]
    double c_n = 0.0;        ///< 等效阻尼系数 [N·s/m]
    double height_coeff = 0.0; ///< 液面高度系数 (用于η计算)
  };

  LiquidSloshModel();

  /**
   * @brief 配置液体晃动模型
   * @param params 液体和容器参数
   * @return true 成功, false 失败
   */
  bool configure(const Params& params);

  /**
   * @brief 重置状态为零
   */
  void reset();

  /**
   * @brief 更新液体晃动状态 (离散时间步进)
   * @param accel_base 底盘加速度 [ax, ay] [m/s²]
   * @param omega_z 底盘角速度 [rad/s] (用于旋转修正)
   * @param alpha_z 底盘角加速度 [rad/s²] (用于旋转修正)
   */
  void update(const Eigen::Vector2d& accel_base,
              double omega_z = 0.0,
              double alpha_z = 0.0);

  /**
   * @brief 获取当前液面晃动高度
   * @return η [m] (液面相对静止位置的最大抬升高度)
   */
  double getSloshHeight() const;

  /**
   * @brief 获取当前模态位移
   * @return [xn, yn] [m]
   */
  Eigen::Vector2d getModalDisplacement() const;

  /**
   * @brief 获取当前模态速度
   * @return [vxn, vyn] [m/s]
   */
  Eigen::Vector2d getModalVelocity() const;

  /**
   * @brief 获取完整状态向量
   * @return [xn, vxn, yn, vyn]
   */
  Eigen::Vector4d getState() const { return state_; }

  /**
   * @brief 直接设置完整状态向量
   * @param state [xn, vxn, yn, vyn]
   *
   * 用于与外部估计器/MPC 增广状态做一致化同步。
   */
  void setState(const Eigen::Vector4d& state);

  /**
   * @brief 获取离散状态矩阵 (用于外部MPC集成)
   * @param Ad 输出: 状态转移矩阵 (4x4)
   * @param Bd 输出: 输入矩阵 (4x2)
   */
  void getDiscreteMatrices(Eigen::Matrix4d& Ad, Eigen::Matrix<double,4,2>& Bd) const {
    Ad = A_discrete_;
    Bd = B_discrete_;
  }

  /**
   * @brief 获取模态参数
   */
  const ModalParams& getModalParams() const { return modal_; }

  /**
   * @brief 获取配置参数
   */
  const Params& getParams() const { return params_; }

  /**
   * @brief 检查模型是否已配置
   */
  bool isConfigured() const { return configured_; }

private:
  Params params_;
  ModalParams modal_;
  bool configured_ = false;

  // 连续状态空间矩阵
  Eigen::Matrix4d A_continuous_;
  Eigen::Matrix<double,4,2> B_continuous_;

  // 离散状态空间矩阵 (ZOH离散化)
  Eigen::Matrix4d A_discrete_;
  Eigen::Matrix<double,4,2> B_discrete_;

  // 当前状态: [xn, vxn, yn, vyn]
  Eigen::Vector4d state_;

  // 记录最近一次 update() 时的底盘绕 z 轴角速度 [rad/s]
  // 用于计算旋转抛物面高度项 (Lp 模型)
  double last_omega_z_ = 0.0;

  /**
   * @brief 计算模态根 ξ1n (贝塞尔函数导数的根)
   * @param n 模态阶数 (1,2,3...)
   * @return ξ1n
   */
  static double computeModalRoot(int n);

  /**
   * @brief 计算模态固有频率 ωn
   * @param xi_1n 模态根
   * @param R 容器半径
   * @param h 液面高度
   * @param g 重力加速度
   * @return ωn [rad/s]
   */
  static double computeNaturalFrequency(double xi_1n, double R, double h, double g);

  /**
   * @brief 计算模态质量 mn
   * @param xi_1n 模态根
   * @param m_F 液体总质量
   * @param R 容器半径
   * @param h 液面高度
   * @return mn [kg]
   */
  static double computeModalMass(double xi_1n, double m_F, double R, double h);

  /**
   * @brief 计算液面高度系数 (线性模型)
   * @return 4*h*mn / (m_F*R)
   */
  static double computeHeightCoeffLinear(double h, double m_n, double m_F, double R);

  /**
   * @brief 计算液面高度系数 (非线性模型)
   * @return xi_1n² * h * mn / (m_F * R)
   */
  static double computeHeightCoeffNonlinear(double xi_1n, double h, double m_n, double m_F, double R);

  /**
   * @brief 构建连续状态空间矩阵
   */
  void buildContinuousMatrices();

  /**
   * @brief 离散化 (Zero-Order Hold)
   */
  void discretize();
};

}  // namespace slosh_models

#endif  // SLOSH_MODELS_LIQUID_SLOSH_MODEL_H
