#ifndef TRACK_SIMPLE_H
#define TRACK_SIMPLE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <ctime>
#include <sys/types.h>
#include <Eigen/Dense>

#include "communication_rs485/mpc_vel_tracker.h"
#include "communication_rs485/liquid_slosh_model.h"

// ============= 小工具：C++11 兼容 clamp =============
template <typename T>
inline T clampv(T v, T lo, T hi){ return (v<lo)?lo:((v>hi)?hi:v); }

// ============= 一维 PID（四向 1m 用） =============
class PIDController {
public:
  PIDController(double kp=0, double ki=0, double kd=0,
                double min_out=-1, double max_out=1, double i_lim=1)
    : kp_(kp),ki_(ki),kd_(kd),min_(min_out),max_(max_out),i_lim_(i_lim),
      e_prev_(0),i_(0),d_prev_(0),t_prev_(ros::Time::now()){}
  void setParams(double kp,double ki,double kd){kp_=kp;ki_=ki;kd_=kd;}
  void setOutputLimits(double mn,double mx){min_=mn;max_=mx;}
  void setIntegralLimit(double l){i_lim_=l;}
  void reset(){e_prev_=0;i_=0;d_prev_=0;t_prev_=ros::Time::now();}
  double compute(double target,double current){
    ros::Time now=ros::Time::now(); double dt=(now-t_prev_).toSec(); t_prev_=now; if(dt<=1e-3) dt=0.01;
    double e=target-current;
    double p=kp_*e; i_+=ki_*e*dt; i_=clampv(i_,-i_lim_,i_lim_);
    double d=kd_*(e-e_prev_)/dt; d=0.7*d+0.3*d_prev_; d_prev_=d; e_prev_=e;
    return clampv(p+i_+d,min_,max_);
  }
private:
  double kp_,ki_,kd_,min_,max_,i_lim_,e_prev_,i_,d_prev_; ros::Time t_prev_;
};

// ================== 模式枚举 ==================
enum Mode { MODE_IDLE=0, MODE_LINE_1M, MODE_TIME_TRAJ, MODE_NEW_TEST_LINE };

// ================== 通用参考接口 ==================
struct IRef {
  virtual ~IRef() = default;
  virtual bool   closed()  const = 0;
  virtual double period()  const = 0;           // T
  virtual void eval(double t, double& x, double& y,
                    double& vx, double& vy) const = 0;  // 局部系
};

// ================== 傅里叶轨迹参考 ==================
struct RefFourier : public IRef {
  double T = 20.0;                 // 周期
  double w = 2.0 * M_PI / 20.0;    // ω = 2π/T
  std::vector<double> a, b;        // x: a0..aK (cos), b0..bK (sin)
  std::vector<double> c, d;        // y: c0..cK (cos), d0..dK (sin)

  bool closed() const override { return true; }
  double period() const override { return T; }

  void eval(double t, double& x, double& y, double& vx, double& vy) const override {
    x = (a.empty()?0.0:a[0]);
    y = (c.empty()?0.0:c[0]);
    vx = 0.0; vy = 0.0;
    int K = std::max(std::max((int)a.size(), (int)b.size()),
                     std::max((int)c.size(), (int)d.size())) - 1;
    for (int k=1; k<=K; ++k) {
      const double ak = (k<(int)a.size()? a[k]:0.0);
      const double bk = (k<(int)b.size()? b[k]:0.0);
      const double ck = (k<(int)c.size()? c[k]:0.0);
      const double dk = (k<(int)d.size()? d[k]:0.0);

      const double th = k * w * t;
      const double ct = std::cos(th), st = std::sin(th);

      x  += ak * ct + bk * st;
      y  += ck * ct + dk * st;

      const double kw = k * w;
      vx += -ak * kw * st + bk * kw * ct;  // d/dt[cos]= -sin*kω, d/dt[sin]=cos*kω
      vy += -ck * kw * st + dk * kw * ct;
    }
  }
};

// ================== 测试直线轨迹参考 ==================
struct RefTestLine : public IRef {
  double duration = 10.0;          // 轨迹持续时间 [s]
  double velocity = 0.3;           // 匀速运动速度 [m/s]
  int direction = 0;               // 方向: 0=前(+x), 1=后(-x), 2=左(+y), 3=右(-y)
  
  bool closed() const override { return false; }
  double period() const override { return duration; }
  
  void eval(double t, double& x, double& y, double& vx, double& vy) const override {
    // 限制时间在 [0, duration] 范围内
    t = std::max(0.0, std::min(t, duration));
    
    x = 0.0; y = 0.0;
    vx = 0.0; vy = 0.0;
    
    switch(direction) {
      case 0:  // 前 (+x)
        x = velocity * t;
        vx = velocity;
        break;
      case 1:  // 后 (-x)
        x = -velocity * t;
        vx = -velocity;
        break;
      case 2:  // 左 (+y)
        y = velocity * t;
        vy = velocity;
        break;
      case 3:  // 右 (-y)
        y = -velocity * t;
        vy = -velocity;
        break;
      default:
        break;
    }
  }
};

// ================== TF 坐标系直线轨迹参考（NEW_TEST_LINE 用）==================
// 时间参数化轨迹：从当前 TF 位置 (dx0, dy0) 线性插值到目标 (0, 0)
struct TfRefLine {
  double dx0 = 0.0;           // 起始 TF dx
  double dy0 = 0.0;           // 起始 TF dy
  double total_time = 10.0;   // 总时间
  
  double duration() const { return total_time; }
  
  // 在 TF 坐标系下：从 (dx0, dy0) 线性插值到 (0, 0)
  void eval(double t, double& dx_ref, double& dy_ref, 
            double& vx_ref, double& vy_ref) const {
    double s = (total_time > 1e-6) ? std::min(t / total_time, 1.0) : 1.0;
    
    // 位置：线性插值
    dx_ref = dx0 * (1.0 - s);
    dy_ref = dy0 * (1.0 - s);
    
    // 速度：恒定（t < T 时）
    // dx0 > 0 表示基点在前方 → 需前进 → vx_ref > 0
    if (t < total_time && total_time > 1e-6) {
      vx_ref = dx0 / total_time;
      vy_ref = dy0 / total_time;
    } else {
      vx_ref = 0.0;
      vy_ref = 0.0;
    }
  }
};

// ================== 主类 ==================
class TrackSimple {
public:
  TrackSimple();
  ~TrackSimple() = default;

private:
  // 回调
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void tfFlashCallback(const geometry_msgs::Pose2D::ConstPtr& pose);  // NEW_TEST_LINE: TF Flash 回调
  void controlLoop(const ros::TimerEvent&);

  // 工具
  static double yawFromQuat(double x,double y,double z,double w);
  static double wrapToPi(double a);
  void   shapeAndPublish(double vx_w,double vy_w);
  void   shapeAndPublish(double vx_w,double vy_w,double wz);
  void   publishErrors(double t,double ex,double ey);
  void   startBagRecording();
  void   stopBagRecording();

  // ROS I/O
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_, odom_sub_;
  ros::Subscriber tf_flash_sub_;          // NEW_TEST_LINE: TF Flash 订阅
  ros::Publisher  vel_pub_, err_xy_pub_, err_norm_pub_;
  ros::Publisher  ref_path_pub_, odom_path_pub_, err_time_path_pub_;
  ros::Publisher  mpc_pred_pub_;
  ros::Timer      control_timer_;

  // 话题名参数
  std::string odom_topic_ = "/odom";  // 可配置里程计话题

  // 手柄
  int enable_button_=5;
  int track_button_=2;         // B
  int axis_dir_lat_=4;         // axes[4] 左/右
  int axis_dir_long_=5;        // axes[5] 前/后

  // 四向 1m
  PIDController linear_pid_;
  double target_distance_=1.0;
  double max_linear_vel_=0.5, min_linear_vel_=0.25, ki_max_=0.2;
  int    dir_mode_=0;          // 1前 -1后 2左 -2右
  double start_x_=0, start_y_=0, current_distance_=0;

  // B键：傅里叶通用轨迹
  std::unique_ptr<IRef> ref_;   // 指向 RefFourier 或 RefTestLine
  double traj_total_time_=20.0; // 周期 T
  bool   traj_closed_=true;     // 默认闭合
  double kp_pos_=1.4, kd_damp_=0.4;
  bool   open_loop_ = false;        // 开环：仅前馈，不加P/D
  double v_min_=0.25, v_max_=0.5;
  double start_boost_vel_=0.22, start_boost_time_=0.6;
  bool   use_world_frame_=false;
  
  // === 测试直线轨迹 ===
  bool   test_line_enabled_ = false;     // 测试直线轨迹开关
  double test_line_duration_ = 10.0;     // 直线轨迹持续时间
  double test_line_velocity_ = 0.3;      // 直线运动速度
  int    test_line_direction_ = 0;       // 方向: 0=前, 1=后, 2=左, 3=右
  bool   test_line_enable_slosh_ = true; // 测试直线模式下是否启用液体晃动抑制

  // test_line 终点驻留收敛（仅非闭合轨迹使用）
  bool   test_line_terminal_hold_ = true;
  double test_line_terminal_hold_timeout_ = 5.0; // s
  double test_line_terminal_pos_tol_ = 0.01;     // m
  double test_line_terminal_vel_tol_ = 0.02;     // m/s
  bool   terminal_hold_active_ = false;
  ros::Time terminal_hold_start_time_;
  double terminal_hold_xd_ = 0.0;
  double terminal_hold_yd_ = 0.0;

  // ============ NEW_TEST_LINE 相关（TF 高精度时间参数化直线）============
  bool   ntl_enabled_ = true;            // NEW_TEST_LINE 总开关
  
  // TF 坐标系正负号配置（方便调整，修改此处即可）
  // TF坐标系定义：+X=目标前方, +Y=目标左侧, yaw>0=CW
  // dx>0 表示车在目标前方, dy>0 表示车在目标左侧
  double ntl_tf_x_sign_ = 1.0;            // dx 正负号：1.0 或 -1.0
  double ntl_tf_y_sign_ = 1.0;            // dy 正负号：1.0 或 -1.0
  double ntl_tf_yaw_sign_ = -1.0;         // yaw 正负号：-1.0 表示 CW->CCW
  
  // TF Flash 数据（已乘正负号）
  double tf_dx_ = 0.0;
  double tf_dy_ = 0.0;
  double tf_yaw_ = 0.0;
  bool   tf_valid_ = false;
  ros::Time tf_flash_stamp_;
  
  // TF 差分速度（目标坐标系）
  double tf_dx_prev_ = 0.0;
  double tf_dy_prev_ = 0.0;
  double tf_vx_ = 0.0;
  double tf_vy_ = 0.0;
  ros::Time tf_vel_stamp_;
  bool   tf_vel_init_ = false;
  
  // NEW_TEST_LINE 轨迹
  std::unique_ptr<TfRefLine> tf_ref_;
  ros::Time ntl_start_time_;
  
  // NEW_TEST_LINE 控制参数
  double ntl_kp_pos_ = 1.5;               // 位置 P 增益
  double ntl_kd_vel_ = 0.5;               // 速度阻尼 D 增益
  double ntl_kp_yaw_ = 2.0;               // 航向 P 增益
  double ntl_kd_yaw_ = 0.3;               // 航向 D 增益
  double ntl_v_max_ = 0.3;                // 最大速度
  double ntl_v_approach_ = 0.1;           // 接近速度（近距离时）
  double ntl_tf_timeout_ = 0.3;           // TF 数据超时阈值
  double ntl_vel_alpha_ = 0.3;            // TF 差分速度低通滤波系数
  double ntl_default_vel_ = 0.15;         // 默认速度（用于计算总时间）
  
  // 收敛阈值
  double ntl_pos_tol_ = 0.008;            // 位置收敛阈值 8mm
  double ntl_yaw_tol_ = 0.035;            // 航向收敛阈值 ~2°
  double ntl_vel_tol_ = 0.02;             // 速度收敛阈值 2cm/s
  double ntl_timeout_ = 30.0;             // 总超时

  // 航向控制（仅在 B 键闭环模式下生效）：使车头对齐参考轨迹切线方向
  bool   align_tangent_=false;
  double kp_yaw_=2.0, kd_yaw_=0.3, max_wz_=1.0;
  std::string bag_dir_="/tmp/track_simple_bags";
  std::string bag_path_;
  pid_t  bag_pid_=-1;
  bool   record_bag_on_b_mode_= true; // 显式开关：B键模式是否自动录包

  // MPC 控制
  bool mpc_enabled_=false;
  communication_rs485::MPCVelTracker::Params mpc_params_;
  std::unique_ptr<communication_rs485::MPCVelTracker> mpc_;
  
  // 液体晃动控制
  bool slosh_enabled_=false;
  std::shared_ptr<communication_rs485::LiquidSloshModel> slosh_model_;
  communication_rs485::LiquidSloshModel::Params slosh_params_;
  ros::Publisher slosh_height_pub_;  // 发布当前液面晃动高度

  // 状态/里程计
  bool  is_enabled_=false;
  Mode  mode_=MODE_IDLE;
  ros::Time mode_start_time_;
  double x0_=0,y0_=0,yaw0_=0;   // 进入轨迹时的锚点
  double odom_x_=0,odom_y_=0,odom_yaw_=0;
  double odom_vx_=0, odom_vy_=0;
  double odom_wz_=0;

  // 可视化
  bool   plot_enabled_=true;
  double path_pub_hz_=20.0;
  size_t path_max_pts_=2000;
  ros::Time last_path_pub_;
  nav_msgs::Path ref_path_, odom_path_, err_time_path_;
  nav_msgs::Path mpc_pred_path_;
};

#endif // TRACK_SIMPLE_H
