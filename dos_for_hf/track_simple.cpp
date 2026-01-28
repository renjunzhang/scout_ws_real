#include "track_simple.h"
#include <algorithm>
#include <sstream>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <cerrno>
#include <cstdio>
#include <ctime>

// ============ 小工具 ============
static inline double clampNormAndScale(double& x,double& y,double vmin,double vmax){
  double n=std::hypot(x,y); if(n<1e-9) return 0.0;
  double s=1.0; if(n<vmin) s=vmin/n; if(n>vmax) s=vmax/n; x*=s; y*=s; return n*s;
}

double TrackSimple::yawFromQuat(double x,double y,double z,double w){
  tf::Quaternion q(x,y,z,w); double r,p,ya; tf::Matrix3x3(q).getRPY(r,p,ya); return ya;
}
double TrackSimple::wrapToPi(double a){ while(a<=-M_PI)a+=2*M_PI; while(a> M_PI)a-=2*M_PI; return a; }

// ============ 构造 ============
TrackSimple::TrackSimple() : linear_pid_(0.8,0.05,0.2, 0.05,0.6,0.2) {
  // 订阅/发布
  // 里程计话题可配置，默认 "/odom"
  nh_.param<std::string>("track_simple/odom_topic", odom_topic_, std::string("/odom"));
  joy_sub_  = nh_.subscribe<sensor_msgs::Joy>("/hf_platform/joy",10,&TrackSimple::joyCallback,this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_,10,&TrackSimple::odomCallback,this);
  vel_pub_  = nh_.advertise<geometry_msgs::Twist>("/hf_platform/joy_vel",10);
  err_xy_pub_   = nh_.advertise<geometry_msgs::PointStamped>("/track_simple/error_xy",10);
  err_norm_pub_ = nh_.advertise<std_msgs::Float64>("/track_simple/error_norm",10);
  ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/track_simple/ref_path",1,true);
  odom_path_pub_= nh_.advertise<nav_msgs::Path>("/track_simple/odom_path",1,true);
  err_time_path_pub_ = nh_.advertise<nav_msgs::Path>("/track_simple/error_time",1,true);
  mpc_pred_pub_ = nh_.advertise<nav_msgs::Path>("/track_simple/mpc_predicted_path",1,true);

  // 手柄/参数（与工程键保持一致）
  nh_.param<int>("joy_to_twist/enable_button", enable_button_, 5);
  nh_.param<int>("joy_to_twist/track_button",  track_button_,  2);
  nh_.param<int>("track_simple/axis_dir_lat",  axis_dir_lat_,  4);
  nh_.param<int>("track_simple/axis_dir_long", axis_dir_long_, 5);

  nh_.param<double>("track_simple/target_distance", target_distance_, 1.0);
  nh_.param<double>("track_simple/max_linear_vel",  max_linear_vel_, 1);
  nh_.param<double>("track_simple/min_linear_vel",  min_linear_vel_, 0.25);
  nh_.param<double>("track_simple/ki_max", ki_max_, 0.2);
  linear_pid_.setOutputLimits(min_linear_vel_, max_linear_vel_);
  linear_pid_.setIntegralLimit(ki_max_);

  // ===== 傅里叶轨迹参数 =====
  nh_.param<double>("track_simple/trajectory/total_time", traj_total_time_, 20.0);
  traj_closed_ = false; // Fourier 闭合
  std::vector<double> A,B,C,D;
  nh_.param<std::vector<double>>("track_simple/trajectory/a", A, std::vector<double>{-1.0, 1.0}); // 圆: x=-R+Rcos
  nh_.param<std::vector<double>>("track_simple/trajectory/b", B, std::vector<double>{0.0});
  nh_.param<std::vector<double>>("track_simple/trajectory/c", C, std::vector<double>{0.0});
  nh_.param<std::vector<double>>("track_simple/trajectory/d", D, std::vector<double>{0.0, 1.0}); // 圆: y=Rsin

  nh_.param<double>("track_simple/kp_pos",  kp_pos_,  1.4);
  nh_.param<double>("track_simple/kd_damp", kd_damp_, 0.4);
  nh_.param<bool>(  "track_simple/control/open_loop", open_loop_, false);
  if(open_loop_){
    ROS_WARN("[track_simple] 开环控制已启用：仅发布前馈 v(t)，不加P/D纠偏");
  }
  nh_.param<double>("track_simple/min_linear_vel", v_min_, 0.25);
  nh_.param<double>("track_simple/max_linear_vel", v_max_, 0.50);
  nh_.param<double>("track_simple/start_boost_vel",  start_boost_vel_,  0.22);
  nh_.param<double>("track_simple/start_boost_time", start_boost_time_, 0.6);
  nh_.param<bool>("track_simple/use_world_frame", use_world_frame_, false);
  // 航向（偏航）控制参数：使车头对齐参考轨迹切线（仅闭环 B 键模式）
  nh_.param<bool>("track_simple/yaw_control/align_tangent", align_tangent_, false);
  nh_.param<double>("track_simple/yaw_control/kp_yaw", kp_yaw_, 2.0);
  nh_.param<double>("track_simple/yaw_control/kd_yaw", kd_yaw_, 0.3);
  nh_.param<double>("track_simple/yaw_control/max_wz", max_wz_, 1.0);
  // MPC 参数
  mpc_enabled_ = nh_.param<bool>("track_simple/mpc/enabled", true);
  if (mpc_enabled_) {
    mpc_params_.dt = nh_.param("track_simple/mpc/dt", 0.05);
    mpc_params_.horizon = nh_.param("track_simple/mpc/horizon_steps", 15);

    auto loadVec2 = [this](const std::string& key, const Eigen::Vector2d& defaults) {
      std::vector<double> tmp;
      if (!nh_.getParam(key, tmp) || tmp.size() != 2) {
        nh_.setParam(key, std::vector<double>{defaults.x(), defaults.y()});
        return defaults;
      }
      return Eigen::Vector2d(tmp[0], tmp[1]);
    };

    mpc_params_.q_pos = loadVec2("track_simple/mpc/q_pos", Eigen::Vector2d(10.0, 10.0));
    mpc_params_.r_vel = loadVec2("track_simple/mpc/r_vel", Eigen::Vector2d(0.2, 0.2));
    mpc_params_.s_delta = loadVec2("track_simple/mpc/s_delta", Eigen::Vector2d(0.05, 0.05));
    mpc_params_.max_speed = nh_.param("track_simple/mpc/max_speed", v_max_);
    mpc_params_.min_speed = nh_.param("track_simple/mpc/min_speed", v_min_);
    mpc_params_.enforce_min_speed = nh_.param("track_simple/mpc/enforce_min_speed", false);
    mpc_params_.regularization = nh_.param("track_simple/mpc/regularization", 1e-6);
    mpc_params_.warm_start = nh_.param("track_simple/mpc/warm_start", true);
    
    // === 液体晃动MPC参数 ===
    mpc_params_.enable_slosh = nh_.param("track_simple/mpc/enable_slosh", true);
    mpc_params_.q_slosh = nh_.param("track_simple/mpc/q_slosh", 10.0);
    mpc_params_.max_slosh_height = nh_.param("track_simple/mpc/max_slosh_height", 0.01);

    mpc_.reset(new communication_rs485::MPCVelTracker());
    if (!mpc_->configure(mpc_params_)) {
      ROS_ERROR("[track_simple] MPC configure failed, fallback to PD");
      mpc_.reset();
      mpc_enabled_ = false;
    } else {
      ROS_INFO("[track_simple] MPC 已启用 (N=%d, dt=%.3f s)", mpc_params_.horizon, mpc_params_.dt);
    }
  }
  
  // === 液体晃动模型参数 ===
  slosh_enabled_ = nh_.param("track_simple/slosh/enabled", true);
  if (slosh_enabled_) {
    slosh_params_.R = nh_.param("track_simple/slosh/container_radius", 0.0136);
    slosh_params_.h = nh_.param("track_simple/slosh/liquid_height", 0.055);
    slosh_params_.rho = nh_.param("track_simple/slosh/liquid_density", 1000.0);
    slosh_params_.dt = mpc_enabled_ ? mpc_params_.dt : 0.05;
    slosh_params_.mode_index = nh_.param("track_simple/slosh/mode_index", 1);
    slosh_params_.zeta = nh_.param("track_simple/slosh/damping_ratio", 0.05);
    slosh_params_.r_x = nh_.param("track_simple/slosh/offset_x", 0.0);
    slosh_params_.r_y = nh_.param("track_simple/slosh/offset_y", 0.0);
    slosh_params_.g = nh_.param("track_simple/slosh/gravity", 9.81);
    slosh_params_.use_linear_model = nh_.param("track_simple/slosh/use_linear_model", true);
    
        
    // === 新增：Lp 模型的旋转抛物面项 ===
    slosh_params_.use_parabola_term = nh_.param("track_simple/slosh/use_parabola_term", true);

    slosh_model_.reset(new communication_rs485::LiquidSloshModel());
    if (!slosh_model_->configure(slosh_params_)) {
      ROS_ERROR("[track_simple] Slosh model configure failed, disabling slosh");
      slosh_model_.reset();
      slosh_enabled_ = false;
    } else {
      const char* model_name = slosh_params_.use_parabola_term ? "Lp (线性MSD + 旋转抛物面)" : "L (线性MSD)";
      ROS_INFO("[track_simple] 液体晃动模型已配置：%s (按B键查看详情)", model_name);
      
      // 将晃动模型绑定到MPC
      if (mpc_enabled_ && mpc_ && mpc_params_.enable_slosh) {
        if (mpc_->setSloshModel(slosh_model_)) {
          ROS_INFO("[track_simple] 液体晃动已绑定到MPC (按B键启动)");
        } else {
          ROS_WARN("[track_simple] ✗ 液体晃动模型绑定到MPC失败");
        }
      }
      
      // 发布液面晃动高度话题
      slosh_height_pub_ = nh_.advertise<std_msgs::Float64>("/track_simple/slosh_height", 10);
    }
  }
  
  nh_.param<std::string>("track_simple/bag_directory", bag_dir_, std::string("/tmp/track_simple_bags"));
  nh_.param<bool>("track_simple/bag_record_on_b_mode", record_bag_on_b_mode_, false);
  if(::mkdir(bag_dir_.c_str(), 0755)!=0 && errno!=EEXIST){
    ROS_WARN_STREAM("[track_simple] 创建 bag 目录失败: " << bag_dir_ << ", errno=" << errno);
  }

  // === NEW_TEST_LINE 模式参数（TF 高精度时间参数化直线）===
  ntl_enabled_ = nh_.param("track_simple/new_test_line/enabled", true);
  if (ntl_enabled_) {
    // TF 坐标系正负号配置（方便调整）
    ntl_tf_x_sign_   = nh_.param("track_simple/new_test_line/tf_x_sign", 1.0);
    ntl_tf_y_sign_   = nh_.param("track_simple/new_test_line/tf_y_sign", 1.0);
    ntl_tf_yaw_sign_ = nh_.param("track_simple/new_test_line/tf_yaw_sign", -1.0);
    
    // 控制参数
    ntl_kp_pos_      = nh_.param("track_simple/new_test_line/kp_pos", 1.5);
    ntl_kd_vel_      = nh_.param("track_simple/new_test_line/kd_vel", 0.5);
    ntl_kp_yaw_      = nh_.param("track_simple/new_test_line/kp_yaw", 2.0);
    ntl_kd_yaw_      = nh_.param("track_simple/new_test_line/kd_yaw", 0.3);
    ntl_v_max_       = nh_.param("track_simple/new_test_line/v_max", 0.3);
    ntl_v_approach_  = nh_.param("track_simple/new_test_line/v_approach", 0.1);
    ntl_tf_timeout_  = nh_.param("track_simple/new_test_line/tf_timeout", 0.3);
    ntl_vel_alpha_   = nh_.param("track_simple/new_test_line/vel_alpha", 0.3);
    ntl_default_vel_ = nh_.param("track_simple/new_test_line/default_velocity", 0.15);
    
    // 收敛阈值
    ntl_pos_tol_     = nh_.param("track_simple/new_test_line/pos_tol", 0.008);
    ntl_yaw_tol_     = nh_.param("track_simple/new_test_line/yaw_tol", 0.035);
    ntl_vel_tol_     = nh_.param("track_simple/new_test_line/vel_tol", 0.02);
    ntl_timeout_     = nh_.param("track_simple/new_test_line/timeout", 30.0);
    
    // 订阅 TF Flash
    tf_flash_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(
        "/tf_flash", 10, &TrackSimple::tfFlashCallback, this);
    
    ROS_WARN("[track_simple] ========================================");
    ROS_WARN("[track_simple] NEW_TEST_LINE 模式已启用（TF 时间参数化直线）");
    ROS_WARN("[track_simple]   TF 正负号: x=%.0f, y=%.0f, yaw=%.0f", 
             ntl_tf_x_sign_, ntl_tf_y_sign_, ntl_tf_yaw_sign_);
    ROS_WARN("[track_simple]   目标: TF (dx=0, dy=0, yaw=0)");
    ROS_WARN("[track_simple]   精度: ~%.0fmm", ntl_pos_tol_ * 1000);
    ROS_WARN("[track_simple]   优先级: NEW_TEST_LINE > test_line > Fourier");
    ROS_WARN("[track_simple]   按 B 键启动");
    ROS_WARN("[track_simple] ========================================");
  }

  // === 测试直线轨迹参数 ===
  test_line_enabled_ = nh_.param("track_simple/test_line/enabled", true);
  if (test_line_enabled_) {
    test_line_duration_ = nh_.param("track_simple/test_line/duration", 10.0);
    test_line_velocity_ = nh_.param("track_simple/test_line/velocity", 0.2);
    test_line_direction_ = nh_.param("track_simple/test_line/direction", 3);
    test_line_enable_slosh_ = nh_.param("track_simple/test_line/enable_slosh_in_mpc",false);

    // 终点驻留收敛参数（仅对 test_line 生效）
    test_line_terminal_hold_ = nh_.param("track_simple/test_line/terminal_hold", true);
    test_line_terminal_hold_timeout_ = nh_.param("track_simple/test_line/terminal_hold_timeout", 5.0);
    test_line_terminal_pos_tol_ = nh_.param("track_simple/test_line/terminal_pos_tol", 0.01);
    test_line_terminal_vel_tol_ = nh_.param("track_simple/test_line/terminal_vel_tol", 0.02);
    
    // 构造 RefTestLine
    std::unique_ptr<RefTestLine> rtl(new RefTestLine());
    rtl->duration = test_line_duration_;
    rtl->velocity = test_line_velocity_;
    rtl->direction = test_line_direction_;
    ref_ = std::move(rtl);
    
    // === 测试直线模式：动态配置 MPC 液体晃动 ===
    if (mpc_enabled_ && mpc_) {
      if (test_line_enable_slosh_ && slosh_enabled_ && slosh_model_) {
        // 启用液体晃动抑制
        mpc_params_.enable_slosh = true;
        if (mpc_->setSloshModel(slosh_model_)) {
          ROS_INFO("[track_simple] 测试直线模式: 液体晃动抑制已启用");
        } else {
          ROS_WARN("[track_simple] 测试直线模式: 液体晃动模型绑定失败");
          mpc_params_.enable_slosh = false;
        }
      } else {
        // 禁用液体晃动抑制（用于对比实验）
        mpc_params_.enable_slosh = false;
        ROS_INFO("[track_simple] 测试直线模式: 液体晃动抑制已禁用 (对比实验)");
      }
    }
    
    const char* dir_names[] = {"前(+X)", "后(-X)", "左(+Y)", "右(-Y)"};
    int dir_idx = std::max(0, std::min(3, test_line_direction_));
    ROS_WARN("[track_simple] ========================================");
    ROS_WARN("[track_simple] 测试直线轨迹已启用！");
    ROS_WARN("[track_simple]   方向: %s", dir_names[dir_idx]);
    ROS_WARN("[track_simple]   参考速度: %.2f m/s (实际速度由MPC计算)", test_line_velocity_);
    ROS_WARN("[track_simple]   持续时间: %.1f s", test_line_duration_);
    ROS_WARN("[track_simple]   预期距离: %.2f m", test_line_velocity_ * test_line_duration_);
    ROS_WARN("[track_simple]   MPC晃动抑制: %s", test_line_enable_slosh_ ? "✓ 启用" : "✗ 禁用(对比实验)");
    ROS_WARN("[track_simple] 按B键启动测试直线轨迹跟踪");
    ROS_WARN("[track_simple] ========================================");
  } else {
    // 构造 RefFourier（原有逻辑）
    std::unique_ptr<RefFourier> rf(new RefFourier());
    rf->T = traj_total_time_;
    rf->w = 2.0 * M_PI / std::max(1e-3, traj_total_time_);
    rf->a = A; rf->b = B; rf->c = C; rf->d = D;
    ref_ = std::move(rf);
    
    // === 傅里叶模式：使用全局配置 ===
    if (mpc_enabled_ && mpc_ && slosh_enabled_ && slosh_model_) {
      bool global_enable_slosh = nh_.param("track_simple/mpc/enable_slosh", true);
      mpc_params_.enable_slosh = global_enable_slosh;
      if (global_enable_slosh) {
        if (mpc_->setSloshModel(slosh_model_)) {
          ROS_INFO("[track_simple] 傅里叶模式: 液体晃动抑制已启用 (全局配置)");
        }
      }
    }
    
    ROS_WARN("[track_simple] 测试直线轨迹未开启 (使用傅里叶轨迹)");
  }

  // 可视化
  nh_.param<bool>("track_simple/plot/enabled", plot_enabled_, true);
  int mp=2000; nh_.param<int>("track_simple/plot/max_points", mp, 2000); path_max_pts_=(size_t)std::max(100,mp);
  nh_.param<double>("track_simple/plot/pub_hz", path_pub_hz_, 20.0);
  last_path_pub_=ros::Time::now();
  ref_path_.header.frame_id="odom";
  odom_path_.header.frame_id="odom";
  err_time_path_.header.frame_id="error"; // 以时间为X轴，误差为Y轴

  // 定时器
  control_timer_ = nh_.createTimer(ros::Duration(0.001), &TrackSimple::controlLoop, this);

  if (!test_line_enabled_) {
    ROS_INFO("[track_simple] 四向1m + B键=傅里叶轨迹; T=%.2fs, a=%zu, b=%zu, c=%zu, d=%zu",
             traj_total_time_, A.size(), B.size(), C.size(), D.size());
  }
}

// ============ Joy 回调 ============
void TrackSimple::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  is_enabled_ = (enable_button_<(int)joy->buttons.size())? joy->buttons[enable_button_] : 0;

  // 四向1m（空闲时，方向轴触发）
  if(is_enabled_ && mode_==MODE_IDLE){
    double ax_lat  = (axis_dir_lat_  < (int)joy->axes.size())  ? joy->axes[axis_dir_lat_]  : 0.0;
    double ax_long = (axis_dir_long_ < (int)joy->axes.size())  ? joy->axes[axis_dir_long_] : 0.0;
    int dm=0;
    if(ax_long>=0.5) dm= 1; else if(ax_long<=-0.5) dm=-1;
    else if(ax_lat>=0.5) dm=2; else if(ax_lat<=-0.5) dm=-2;
    if(dm!=0){
      mode_=MODE_LINE_1M; dir_mode_=dm; start_x_=odom_x_; start_y_=odom_y_;
      current_distance_=0;
      // MODE_LINE_1M 需要允许速度为 0/负值以便越过目标后回拉修正
      linear_pid_.setOutputLimits(-max_linear_vel_, max_linear_vel_);
      linear_pid_.reset();
      mode_start_time_=ros::Time::now();
      ROS_WARN("[track_simple] 直线1m启动: %s",(dm==1?"前":dm==-1?"后":dm==2?"左":"右"));
    }
  }

  // B 键沿边沿触发
  static int b_prev=0;
  int b=(track_button_<(int)joy->buttons.size())? joy->buttons[track_button_] : 0;
  bool b_pressed=(b==1 && b_prev==0); b_prev=b;
  if(is_enabled_ && b_pressed && mode_==MODE_IDLE){
    
    // ★ 优先级 1：NEW_TEST_LINE（TF 高精度时间参数化直线）
    if (ntl_enabled_) {
      // ★ 修改 1：启动阶段只检查"是否收到过 TF"，不检查新鲜度
      if (!tf_valid_) {
        ROS_ERROR("[track_simple] NEW_TEST_LINE 启动失败: 尚未收到任何 TF Flash 数据");
        ROS_ERROR("[track_simple]   请确保 /tf_flash 话题在发布");
        // 不 return，继续尝试其他模式
      } else {
        // 创建 TF 直线轨迹
        tf_ref_.reset(new TfRefLine());
        tf_ref_->dx0 = tf_dx_;
        tf_ref_->dy0 = tf_dy_;
        
        // 根据距离计算总时间
        double dist = std::hypot(tf_dx_, tf_dy_);
        tf_ref_->total_time = std::max(dist / ntl_default_vel_, 2.0);  // 最少 2 秒
        
        mode_ = MODE_NEW_TEST_LINE;
        ntl_start_time_ = ros::Time::now();
        tf_vel_init_ = false;
        
        ROS_WARN("[track_simple] ========================================");
        ROS_WARN("[track_simple] NEW_TEST_LINE 模式启动");
        ROS_WARN("[track_simple]   起点(TF): dx=%.3fm, dy=%.3fm, yaw=%.1f°",
                 tf_dx_, tf_dy_, tf_yaw_ * 180.0 / M_PI);
        ROS_WARN("[track_simple]   终点(TF): dx=0, dy=0, yaw=0");
        ROS_WARN("[track_simple]   直线距离: %.3fm, 预计时间: %.1fs",
                 dist, tf_ref_->total_time);
        ROS_WARN("[track_simple]   收敛阈值: %.0fmm / %.1f°",
                 ntl_pos_tol_*1000, ntl_yaw_tol_*180/M_PI);
        ROS_WARN("[track_simple] ========================================");
        
        if(record_bag_on_b_mode_) startBagRecording();
        return;
      }
    }
    
    // ★ 优先级 2 & 3：test_line 或傅里叶（原有逻辑）
    x0_=odom_x_; y0_=odom_y_; yaw0_=odom_yaw_;
    mode_=MODE_TIME_TRAJ; mode_start_time_=ros::Time::now();
    ref_path_.poses.clear(); odom_path_.poses.clear();
    if (mpc_) {
      mpc_->reset();
    }
    
    ROS_WARN("[track_simple] ========================================");
    if (test_line_enabled_) {
      const char* dir_names[] = {"前(+X)", "后(-X)", "左(+Y)", "右(-Y)"};
      int dir_idx = std::max(0, std::min(3, test_line_direction_));
      ROS_WARN("[track_simple] 测试直线轨迹模式开始");
      ROS_WARN("[track_simple]   方向: %s", dir_names[dir_idx]);
      ROS_WARN("[track_simple]   参考速度: %.2f m/s (实际由MPC计算)", test_line_velocity_);
      ROS_WARN("[track_simple]   持续时间: %.1f s", test_line_duration_);
      ROS_WARN("[track_simple]   预期距离: %.2f m", test_line_velocity_ * test_line_duration_);
    } else {
      ROS_WARN("[track_simple] 傅里叶轨迹模式开始");
    }
    
    if(open_loop_){
      ROS_WARN("[track_simple] 开环模式：v = 前馈 (x'(t), y'(t))，不加P/D");
    }
    if(mpc_enabled_ && mpc_ && !open_loop_){
      ROS_WARN("[track_simple] 当前使用 MPC 闭环控制");
      
      // 根据模式显示不同的晃动抑制状态
      if (test_line_enabled_) {
        // 测试直线模式：显示测试配置
        if(test_line_enable_slosh_ && slosh_enabled_ && mpc_params_.enable_slosh){
          ROS_WARN("[track_simple] ✓ MPC配置: 液体晃动抑制 已启用");
          ROS_WARN("[track_simple]   容器: R=%.3fm, h=%.3fm, 固有频率=%.2fHz",
                   slosh_params_.R, slosh_params_.h, 
                   slosh_model_->getModalParams().omega_n/(2*M_PI));
          ROS_WARN("[track_simple]   晃动权重 q_slosh=%.1f, 监控: /track_simple/slosh_height",
                   mpc_params_.q_slosh);
        } else {
          ROS_WARN("[track_simple] ✗ MPC配置: 液体晃动抑制 已禁用 (对比实验)");
          ROS_WARN("[track_simple]   标准MPC控制 - 无晃动约束");
        }
      } else {
        // 傅里叶模式：显示全局配置
        if(slosh_enabled_ && mpc_params_.enable_slosh){
          ROS_WARN("[track_simple] ✓ 液体晃动抑制已激活!");
          ROS_WARN("[track_simple]   容器: R=%.3fm, h=%.3fm, 固有频率=%.2fHz",
                   slosh_params_.R, slosh_params_.h, 
                   slosh_model_->getModalParams().omega_n/(2*M_PI));
          ROS_WARN("[track_simple]   晃动权重 q_slosh=%.1f, 监控话题: /track_simple/slosh_height",
                   mpc_params_.q_slosh);
        } else {
          ROS_INFO("[track_simple] 液体晃动抑制未启用 (标准MPC控制)");
        }
      }
    }
    ROS_WARN("[track_simple] ========================================");
    if(record_bag_on_b_mode_){
      startBagRecording();
    } else {
      ROS_INFO_THROTTLE(5.0, "[track_simple] 已禁用B键自动录包 (track_simple/bag_record_on_b_mode=false)");
    }
  }

  // 松开使能：立即停
  if(!is_enabled_ && mode_!=MODE_IDLE){
    mode_=MODE_IDLE; geometry_msgs::Twist zero; vel_pub_.publish(zero);
    terminal_hold_active_ = false;  // 重置终点驻留状态
    tf_ref_.reset();                // 重置 TF 轨迹
    ROS_WARN_THROTTLE(1.0,"[track_simple] 未使能：退出模式");
    stopBagRecording();
  }
}

// ============ TF Flash 回调（NEW_TEST_LINE 用）============
void TrackSimple::tfFlashCallback(const geometry_msgs::Pose2D::ConstPtr& pose)
{
    // 直接从 Pose2D 提取数据
    double dx_raw = pose->x;
    double dy_raw = pose->y;
    double yaw_raw_rad = pose->theta;  // ← /tf_flash 输出的是弧度（radian）
    double dist = std::hypot(dx_raw, dy_raw);
    
    ros::Time now = ros::Time::now();
    
    // /tf_flash 定义：后(+)、右(+)、顺时针(+)
    // 控制希望：前(+)、左(+)、逆时针(+) → 取反后再乘可配置正负号
    double dx = -dx_raw * ntl_tf_x_sign_;
    double dy = -dy_raw * ntl_tf_y_sign_;
    double yaw = -yaw_raw_rad * ntl_tf_yaw_sign_;
    yaw = wrapToPi(yaw);  // 保护：限制到 [-π, π]

      // 位置可选低通，抑制抖动（复用速度滤波系数）
      if (tf_valid_) {
        const double alpha_p = ntl_vel_alpha_;
        tf_dx_ = alpha_p * dx + (1.0 - alpha_p) * tf_dx_;
        tf_dy_ = alpha_p * dy + (1.0 - alpha_p) * tf_dy_;
      } else {
        tf_dx_ = dx;
        tf_dy_ = dy;
      }
      tf_yaw_ = yaw;

      // 差分速度（仍在车体系），并低通
      if (tf_vel_init_) {
        double dt = (now - tf_vel_stamp_).toSec();
        if (dt > 0.005 && dt < 0.5) {
          double vx_raw = (dx - tf_dx_prev_) / dt;
          double vy_raw = (dy - tf_dy_prev_) / dt;
          tf_vx_ = ntl_vel_alpha_ * vx_raw + (1.0 - ntl_vel_alpha_) * tf_vx_;
          tf_vy_ = ntl_vel_alpha_ * vy_raw + (1.0 - ntl_vel_alpha_) * tf_vy_;
        }
      } else {
        tf_vel_init_ = true;
        tf_vx_ = tf_vy_ = 0.0;
      }

      tf_dx_prev_ = dx;
      tf_dy_prev_ = dy;
      tf_vel_stamp_ = now;
      tf_flash_stamp_ = now;
      tf_valid_ = true;
      
      // ★ 调试信息：打印接收到的 TF 数据（限制频率：1Hz）
      ROS_INFO_THROTTLE(1.0, "[track_simple] TF 数据更新：dx=%.3f m, dy=%.3f m, yaw=%.3f rad (%.1f°), dist=%.3f m", 
                        dx, dy, yaw, yaw * 180.0 / M_PI, dist);
}

// ============ Odom 回调 ============
void TrackSimple::odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
  odom_x_=odom->pose.pose.position.x;
  odom_y_=odom->pose.pose.position.y;
  odom_yaw_=yawFromQuat(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,
                        odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
  // 世界系速度
  double bvx=odom->twist.twist.linear.x, bvy=odom->twist.twist.linear.y;
  odom_wz_ = odom->twist.twist.angular.z;
  double c=std::cos(odom_yaw_), s=std::sin(odom_yaw_);
  odom_vx_= c*bvx - s*bvy;
  odom_vy_= s*bvx + c*bvy;

  // 1m 距离 & 完成/超时
  if(mode_==MODE_LINE_1M){
    current_distance_ = std::hypot(odom_x_-start_x_, odom_y_-start_y_);
    // 收敛条件：位置到位 + 速度停稳（防止刚进入阈值就切 IDLE 导致惯性冲过头）
    const double pos_tol = 0.005;  // m
    const double vel_tol = 0.01;  // m/s
    const double vel_norm = std::hypot(odom_vx_, odom_vy_);
    const bool reached = (std::fabs(target_distance_ - current_distance_) < pos_tol);
    const bool stopped = (vel_norm < vel_tol);

    if(reached && stopped){
      mode_=MODE_IDLE; geometry_msgs::Twist zero; vel_pub_.publish(zero);
      ROS_WARN("[track_simple] 直线%.2fm收敛完成 (|e|<%.3fm 且 |v|<%.3fm/s)",
               target_distance_, pos_tol, vel_tol);
      stopBagRecording();
    }else if((ros::Time::now()-mode_start_time_).toSec()>20.0){
      mode_=MODE_IDLE; geometry_msgs::Twist zero; vel_pub_.publish(zero);
      ROS_ERROR("[track_simple] 直线%.2fm超时退出", target_distance_);
      stopBagRecording();
    }
  }
}

// ============ 控制循环 ============
void TrackSimple::controlLoop(const ros::TimerEvent&){
  if(!is_enabled_ || mode_==MODE_IDLE) return;

  if(mode_==MODE_LINE_1M){
    // 允许 v 为 0 或负值：越过目标后可倒车回拉，避免“强制最小速度”造成过冲
    double v = linear_pid_.compute(target_distance_, current_distance_);

    // 可选：仅在误差较大且 PID 输出过小的时候，给一个起步克服静摩擦的速度
    const double err = target_distance_ - current_distance_;
    const double start_err_thresh = 0.10;  // m
    const double start_kick = 0.10;        // m/s
    if(std::fabs(err) > start_err_thresh && std::fabs(v) < start_kick){
      v = (err >= 0.0) ? start_kick : -start_kick;
    }

    // 仅限制最大速度，不设最小速度下限
    v = clampv(v, -max_linear_vel_, max_linear_vel_);

    double bx=0,by=0;             // 机体系
    if(dir_mode_== 1) bx=+v; if(dir_mode_==-1) bx=-v;
    if(dir_mode_== 2) by=+v; if(dir_mode_==-2) by=-v;

    // 旋到世界
    double c=std::cos(odom_yaw_), s=std::sin(odom_yaw_);
    double vx_w= c*bx - s*by, vy_w= s*bx + c*by;
    shapeAndPublish(vx_w,vy_w);
    return;
  }

  // ================================================================
  // MODE_NEW_TEST_LINE: TF 高精度时间参数化直线控制
  // ================================================================
  if (mode_ == MODE_NEW_TEST_LINE) {
    ros::Time now = ros::Time::now();
    double t = (now - ntl_start_time_).toSec();
    
    // 1) 超时检查
    if (t > ntl_timeout_) {
      mode_ = MODE_IDLE;
      geometry_msgs::Twist zero;
      vel_pub_.publish(zero);
      tf_ref_.reset();
      ROS_ERROR("[track_simple] NEW_TEST_LINE 超时退出 (%.1fs)", ntl_timeout_);
      stopBagRecording();
      return;
    }

    // 2) TF 数据有效性检查
    bool tf_fresh = tf_valid_ && (now - tf_flash_stamp_).toSec() < ntl_tf_timeout_;
    
    double cmd_vx = 0, cmd_vy = 0, cmd_wz = 0;
    double ex_tf = 0, ey_tf = 0;
    
    if (tf_fresh && tf_ref_) {
      // ========================================================
      // 3) 获取参考轨迹（车体系，dx/dy 朝向目标的直线轨迹）
      // ========================================================
      double dx_ref, dy_ref, vx_ref, vy_ref;
      tf_ref_->eval(t, dx_ref, dy_ref, vx_ref, vy_ref);
      
      // ========================================================
      // 4) 计算误差（视觉伺服：误差 = 实测 - 参考）
      //    例：实测 tf_dx_=1.0, 参考 dx_ref=0.9
      //        误差 ex_tf = +0.1（车落后于参考，需加速追上）
      // ========================================================
      ex_tf = tf_dx_ - dx_ref;
      ey_tf = tf_dy_ - dy_ref;
      
      // ========================================================
      // 5) PD 控制律（视觉伺服，车体系直接闭环）
      //    - 前馈：vx_ref > 0（车需前进）
      //    - P项：ex_tf > 0 时需加速 → +Kp * ex_tf
      //    - D项：tf_vx_ < 0（车前进时 tf_dx_ 减小）
      //           需要 +Kd * tf_vx_ 提供阻尼（负值=减速）
      // ========================================================
      cmd_vx = vx_ref + ntl_kp_pos_ * ex_tf + ntl_kd_vel_ * tf_vx_;
      cmd_vy = vy_ref + ntl_kp_pos_ * ey_tf + ntl_kd_vel_ * tf_vy_;
      
      // 航向控制：tf_yaw_ > 0 表示需顺时针（负角速度）
      cmd_wz = -ntl_kp_yaw_ * tf_yaw_ - ntl_kd_yaw_ * odom_wz_;
      
      // ========================================================
      // 7) 速度限幅（近距离减速）
      // ========================================================
      double dist = std::hypot(ex_tf, ey_tf);
      double v_limit = (dist < 0.1) ? ntl_v_approach_ : ntl_v_max_;
      double v_norm = std::hypot(cmd_vx, cmd_vy);
      if (v_norm > v_limit) {
        cmd_vx *= v_limit / v_norm;
        cmd_vy *= v_limit / v_norm;
      }
      cmd_wz = clampv(cmd_wz, -max_wz_, max_wz_);
      
      // ========================================================
      // 8) 收敛判定（轨迹结束后）
      // ========================================================
      if (t >= tf_ref_->duration()) {
        double vel_norm = std::hypot(tf_vx_, tf_vy_);
        if (dist < ntl_pos_tol_ && 
            std::fabs(tf_yaw_) < ntl_yaw_tol_ && 
            vel_norm < ntl_vel_tol_) {
          mode_ = MODE_IDLE;
          geometry_msgs::Twist zero;
          vel_pub_.publish(zero);
          tf_ref_.reset();
          ROS_WARN("[track_simple] ========================================");
          ROS_WARN("[track_simple] NEW_TEST_LINE 完成!");
          ROS_WARN("[track_simple]   最终误差: dx=%.1fmm, dy=%.1fmm, yaw=%.2f°",
                   tf_dx_ * 1000, tf_dy_ * 1000, tf_yaw_ * 180.0 / M_PI);
          ROS_WARN("[track_simple]   最终速度: %.1fmm/s", vel_norm * 1000);
          ROS_WARN("[track_simple]   总耗时: %.1fs", t);
          ROS_WARN("[track_simple] ========================================");
          stopBagRecording();
          return;
        }
      }
      
      // 发布误差（用于调试/可视化）
      publishErrors(t, ex_tf, ey_tf);
      
      // ★ 轨迹跟踪误差日志（1Hz）
      double vel_norm = std::hypot(tf_vx_, tf_vy_);
      ROS_INFO_THROTTLE(1.0, 
        "[track_simple] NEW_TEST_LINE 进度 | "
        "时间: %.1f/%.1f s | "
        "位置误差: dx=%.1f mm, dy=%.1f mm, dist=%.1f mm | "
        "姿态误差: yaw=%.2f° | "
        "速度: vx=%.2f m/s, vy=%.2f m/s, |v|=%.2f m/s",
        t, tf_ref_->duration(),
        ex_tf * 1000.0, ey_tf * 1000.0, dist * 1000.0,
        tf_yaw_ * 180.0 / M_PI,
        tf_vx_, tf_vy_, vel_norm);
      
    } else {
      // ========================================================
      // TF 丢失：阻尼刹车
      // ========================================================
      double c = std::cos(odom_yaw_);
      double s = std::sin(odom_yaw_);
      double vx_body =  c * odom_vx_ + s * odom_vy_;
      double vy_body = -s * odom_vx_ + c * odom_vy_;
      
      cmd_vx = -0.6 * vx_body;
      cmd_vy = -0.6 * vy_body;
      cmd_wz = -0.3 * odom_wz_;
      
      ROS_WARN_THROTTLE(0.5, "[track_simple] TF 丢失，阻尼刹车中...");
    }
    
    // ========================================================
    // 9) 发布车体系速度
    // ========================================================
    geometry_msgs::Twist cmd;
    cmd.linear.x = clampv(cmd_vx, -ntl_v_max_, ntl_v_max_);
    cmd.linear.y = clampv(cmd_vy, -ntl_v_max_, ntl_v_max_);
    cmd.angular.z = clampv(cmd_wz, -max_wz_, max_wz_);
    vel_pub_.publish(cmd);
    
    return;
  }

  if(mode_==MODE_TIME_TRAJ){
    // 1) 取时间
    double t=(ros::Time::now()-mode_start_time_).toSec();
    const double T = ref_? ref_->period() : traj_total_time_;
    
    // 检查是否是闭合轨迹；test_line(非闭合) 支持终点驻留收敛
    bool is_closed = ref_ ? ref_->closed() : traj_closed_;
    const bool terminal_hold_enabled = (!is_closed) && test_line_enabled_ && test_line_terminal_hold_ && (!open_loop_);
    if (!is_closed && t > T) {
      if (terminal_hold_enabled) {
        if (!terminal_hold_active_) {
          terminal_hold_active_ = true;
          terminal_hold_start_time_ = ros::Time::now();

          // 锁存终点位置（t=T）
          double xrT=0.0, yrT=0.0, vxrT=0.0, vyrT=0.0;
          if(ref_) ref_->eval(T, xrT, yrT, vxrT, vyrT);
          double c0=std::cos(yaw0_), s0=std::sin(yaw0_);
          terminal_hold_xd_ = x0_ + c0*xrT - s0*yrT;
          terminal_hold_yd_ = y0_ + s0*xrT + c0*yrT;

          ROS_WARN("[track_simple] test_line 进入终点驻留：收敛到(%.3f, %.3f), timeout=%.1fs",
                   terminal_hold_xd_, terminal_hold_yd_, test_line_terminal_hold_timeout_);
        }
        // 终点驻留阶段：参考点冻结在 T
        t = T;
      } else {
        // 非闭合轨迹结束，停止运动
        mode_ = MODE_IDLE;
        geometry_msgs::Twist zero;
        vel_pub_.publish(zero);
        ROS_WARN("[track_simple] 轨迹完成 (t=%.2f s, T=%.2f s)，已停止", t, T);
        stopBagRecording();
        return;
      }
    }
    
    // 闭合轨迹才使用 fmod 循环
    if(is_closed && T>1e-3){ t=fmod(t,T); if(t<0) t+=T; }

    // 2) 局部参考 -> 世界
    double xr,yr,vxr,vyr; xr=yr=vxr=vyr=0.0;
    if(ref_) ref_->eval(t, xr, yr, vxr, vyr);

    double c0=std::cos(yaw0_), s0=std::sin(yaw0_);
    double xd = x0_ + c0*xr - s0*yr;
    double yd = y0_ + s0*xr + c0*yr;
    double vxd=       c0*vxr - s0*vyr;
    double vyd=       s0*vxr + c0*vyr;

    // 终点驻留阶段：位置固定在终点，参考速度为 0，让控制器做刹车/回拉收敛
    if(terminal_hold_active_){
      xd = terminal_hold_xd_;
      yd = terminal_hold_yd_;
      vxd = 0.0;
      vyd = 0.0;
    }

    // 3) 前馈 + 闭环调节（世界系）
    double ex=xd-odom_x_, ey=yd-odom_y_;
    double ux=0.0, uy=0.0;
    bool mpc_used=false;

    const bool use_mpc = (!open_loop_) && mpc_enabled_ && mpc_;
    if(use_mpc){
      std::vector<Eigen::Vector2d> ref_vel_seq;
      ref_vel_seq.reserve(mpc_params_.horizon);
      for(int j=0; j<mpc_params_.horizon; ++j){
        double tj = t + j * mpc_params_.dt;
        
        // 只对闭合轨迹使用 fmod 循环
        if(is_closed && T>1e-3){
          tj = std::fmod(tj, T);
          if(tj<0) tj+=T;
        }
        // 非闭合轨迹：终点后冻结在 T
        else if (!is_closed) {
          tj = std::min(tj, T);
        }
        
        double xj=0.0, yj=0.0, vxj=0.0, vyj=0.0;
        if(ref_) ref_->eval(tj, xj, yj, vxj, vyj);
        // 终点后参考速度置零（终点驻留）：让 MPC 预见到停车
        if(!is_closed && terminal_hold_active_ && tj >= T){
          vxj = 0.0;
          vyj = 0.0;
        }
        Eigen::Vector2d vel_world(c0*vxj - s0*vyj,
                                  s0*vxj + c0*vyj);
        ref_vel_seq.push_back(vel_world);
      }
      Eigen::Vector2d pos_err(ex, ey);
      // 传递角速度和角加速度给MPC（用于液体晃动计算）
      // 注意：当前简化实现中角加速度设为0（因为未跟踪角加速度）
      auto mpc_res = mpc_->compute(pos_err, ref_vel_seq, odom_wz_, 0.0);
      if(mpc_res.success){
        ux = mpc_res.command.x();
        uy = mpc_res.command.y();
        mpc_used = true;
        
        // 注意：液面晃动高度现在在速度指令发布后统一更新和发布
        // 这样可以支持"有/无晃动抑制"的对比实验
        
        // 发布 MPC 预测轨迹（世界坐标，odom系）以便可视化
        if(ref_) {
          nav_msgs::Path pred_path;
          pred_path.header.stamp = ros::Time::now();
          pred_path.header.frame_id = "odom";
          pred_path.poses.reserve(mpc_params_.horizon);
          for (int j = 0; j < mpc_params_.horizon; ++j) {
            double tj = t + j * mpc_params_.dt;
            
            // 只对闭合轨迹使用 fmod 循环
            if (is_closed && T > 1e-3) {
              tj = std::fmod(tj, T);
              if (tj < 0) tj += T;
            }
            // 非闭合轨迹：限制在 [0, T] 范围内
            else if (!is_closed) {
              tj = std::min(tj, T);
            }
            
            double xj = 0.0, yj = 0.0, vxj = 0.0, vyj = 0.0;
            ref_->eval(tj, xj, yj, vxj, vyj);
            double wx = x0_ + c0 * xj - s0 * yj;
            double wy = y0_ + s0 * xj + c0 * yj;
            Eigen::Vector2d errj = (j < (int)mpc_res.predicted_errors.size()) ? mpc_res.predicted_errors[j] : Eigen::Vector2d::Zero();
            geometry_msgs::PoseStamped ps;
            ps.header.stamp = ros::Time::now();
            ps.header.frame_id = "odom";
            ps.pose.position.x = wx - errj.x();
            ps.pose.position.y = wy - errj.y();
            ps.pose.orientation.w = 1.0;
            pred_path.poses.push_back(ps);
          }
          mpc_pred_pub_.publish(pred_path);
        }
      }else{
        ROS_WARN_THROTTLE(1.0, "[track_simple] MPC solve failed, fallback to PD");
      }
    }

    if(!mpc_used){
      if(open_loop_){
        ux = vxd; uy = vyd;            // 开环：仅发布前馈速度
      }else{
        ux = vxd + kp_pos_*ex - kd_damp_*odom_vx_;
        uy = vyd + kp_pos_*ey - kd_damp_*odom_vy_;
      }
    }

    // 4) 起步/限幅/平滑
    const double ts=(ros::Time::now()-mode_start_time_).toSec();
    const double vmin_dyn = (ts<start_boost_time_) ? std::max(v_min_,start_boost_vel_) : v_min_;
    double clamp_min = terminal_hold_active_ ? 0.0 : vmin_dyn;
    if(mpc_used && !mpc_params_.enforce_min_speed){
      clamp_min = 0.0;
    }
    clampNormAndScale(ux,uy,clamp_min,v_max_);
    static double lvx=0, lvy=0; const double alpha=0.9;
    if(mpc_used){
      lvx = ux; lvy = uy;
    }else{
      ux=alpha*ux+(1-alpha)*lvx; uy=alpha*uy+(1-alpha)*lvy; lvx=ux; lvy=uy;
    }

    // 5) 航向对齐（仅闭环且启用）：使车头朝向参考切线方向
    double wz_cmd = 0.0;
    if(!open_loop_ && align_tangent_){
      // 参考切线方向（世界系），由参考速度方向给出
      double yaw_ref = std::atan2(vyd, vxd);
      double eyaw = wrapToPi(yaw_ref - odom_yaw_);
      wz_cmd = kp_yaw_ * eyaw - kd_yaw_ * odom_wz_;
      wz_cmd = clampv(wz_cmd, -max_wz_, max_wz_);
    }

    // 发布 & 误差
    shapeAndPublish(ux,uy, wz_cmd);
    publishErrors(t,ex,ey);

    // 终点驻留退出条件：到位且停稳，或驻留超时
    if(terminal_hold_active_){
      const double epos = std::hypot(ex, ey);
      const double evel = std::hypot(odom_vx_, odom_vy_);
      const bool reached = (epos < test_line_terminal_pos_tol_);
      const bool stopped = (evel < test_line_terminal_vel_tol_);
      const bool timeout = ((ros::Time::now() - terminal_hold_start_time_).toSec() > test_line_terminal_hold_timeout_);

      if((reached && stopped) || timeout){
        mode_ = MODE_IDLE;
        geometry_msgs::Twist zero;
        vel_pub_.publish(zero);
        ROS_WARN("[track_simple] test_line 终点驻留结束: |e|=%.3fm |v|=%.3fm/s%s",
                 epos, evel, timeout ? " (timeout)" : "");
        stopBagRecording();
        terminal_hold_active_ = false;
        return;
      }
    }
    
    // === 独立维护液体晃动状态（无论MPC是否启用晃动抑制）===
    // 用于对比实验：即使MPC不优化晃动，也观察实际晃动情况
    if (slosh_enabled_ && slosh_model_) {
      static Eigen::Vector2d last_vel = Eigen::Vector2d::Zero();
      static ros::Time last_update_time = ros::Time(0);  // 初始化为零时刻
      static bool first_update = true;
      
      auto current_time = ros::Time::now();
      
      // 首次更新，仅初始化
      if (first_update) {
        last_vel = Eigen::Vector2d(ux, uy);
        last_update_time = current_time;
        first_update = false;
      } else {
        double dt_slosh = (current_time - last_update_time).toSec();
        
        // 使用固定的时间步长（与MPC的dt一致），避免数值不稳定
        const double dt_model = slosh_params_.dt;  // 0.05s (20Hz)
        
        // 只在时间步累积到模型时间步长时更新
        if (dt_slosh >= dt_model) {
          Eigen::Vector2d current_vel(ux, uy);
          Eigen::Vector2d accel = (current_vel - last_vel) / dt_slosh;
          
          // 更新晃动模型（使用实际发送的速度指令）
          slosh_model_->update(accel, odom_wz_, 0.0);
          
          // 发布液面晃动高度（单位：米）
          std_msgs::Float64 slosh_msg;
          slosh_msg.data = slosh_model_->getSloshHeight();
          slosh_height_pub_.publish(slosh_msg);
          
          last_vel = current_vel;
          last_update_time = current_time;
        }
      }
    }

    // 6) 可视化（低频）
    if(plot_enabled_){
      const ros::Time now=ros::Time::now();
      const double period=(path_pub_hz_>1e-3)?(1.0/path_pub_hz_):0.05;
      if((now-last_path_pub_).toSec()+1e-9>=period){
        last_path_pub_=now;
        geometry_msgs::PoseStamped pr; pr.header.stamp=now; pr.header.frame_id="odom";
        pr.pose.position.x=xd; pr.pose.position.y=yd; pr.pose.orientation.w=1.0;
        ref_path_.header.stamp=now; ref_path_.poses.push_back(pr);
        if(ref_path_.poses.size()>path_max_pts_) ref_path_.poses.erase(ref_path_.poses.begin());
        ref_path_pub_.publish(ref_path_);

        geometry_msgs::PoseStamped po; po.header.stamp=now; po.header.frame_id="odom";
        po.pose.position.x=odom_x_; po.pose.position.y=odom_y_; po.pose.orientation.w=1.0;
        odom_path_.header.stamp=now; odom_path_.poses.push_back(po);
        if(odom_path_.poses.size()>path_max_pts_) odom_path_.poses.erase(odom_path_.poses.begin());
        odom_path_pub_.publish(odom_path_);

        // 误差-时间曲线：x=从起始累计时间，y=|e|
        geometry_msgs::PoseStamped pe; pe.header.stamp=now; pe.header.frame_id="error";
        pe.pose.position.x = ts;                         // 时间轴（秒）
        pe.pose.position.y = std::hypot(ex,ey);          // 误差模长
        pe.pose.orientation.w = 1.0;
        err_time_path_.header.stamp=now; err_time_path_.poses.push_back(pe);
        if(err_time_path_.poses.size()>path_max_pts_) err_time_path_.poses.erase(err_time_path_.poses.begin());
        err_time_path_pub_.publish(err_time_path_);
      }
    }
  }
}

// ============ 发布 ============
void TrackSimple::shapeAndPublish(double vx_w,double vy_w){
  // 向后兼容：不指定角速度时为 0
  shapeAndPublish(vx_w, vy_w, 0.0);
}

void TrackSimple::shapeAndPublish(double vx_w,double vy_w,double wz){
  geometry_msgs::Twist cmd;
  if(use_world_frame_){
    cmd.linear.x = clampv(vx_w,-v_max_,v_max_);
    cmd.linear.y = clampv(vy_w,-v_max_,v_max_);
  }else{
    double c=std::cos(odom_yaw_), s=std::sin(odom_yaw_);
    double vx_b= c*vx_w + s*vy_w;
    double vy_b=-s*vx_w + c*vy_w;
    cmd.linear.x = clampv(vx_b,-v_max_,v_max_);
    cmd.linear.y = clampv(vy_b,-v_max_,v_max_);
  }
  cmd.angular.z = clampv(wz, -max_wz_, max_wz_);
  vel_pub_.publish(cmd);
}

void TrackSimple::publishErrors(double /*t*/,double ex,double ey){
  geometry_msgs::PointStamped p; p.header.stamp=ros::Time::now(); p.header.frame_id="odom";
  p.point.x=ex; p.point.y=ey; err_xy_pub_.publish(p);
  std_msgs::Float64 n; n.data=std::hypot(ex,ey); err_norm_pub_.publish(n);
}

void TrackSimple::startBagRecording(){
  if(bag_pid_>0){
    ROS_WARN_THROTTLE(5.0,"[track_simple] rosbag 正在记录，忽略新的启动请求");
    return;
  }

  std::time_t now = std::time(nullptr);
  std::tm tm_buf;
  localtime_r(&now,&tm_buf);
  char ts[32];
  if(std::strftime(ts,sizeof(ts),"%Y%m%d_%H%M%S",&tm_buf)==0){
    std::snprintf(ts,sizeof(ts),"%ld", static_cast<long>(now));
  }

  std::ostringstream oss;
  oss << bag_dir_;
  if(!bag_dir_.empty() && bag_dir_.back()!='/') oss << '/';
  oss << "track_simple_" << ts << ".bag";
  bag_path_ = oss.str();

  pid_t pid = fork();
  if(pid==0){
    execlp("rosbag","rosbag","record","-a","-O",bag_path_.c_str(),static_cast<char*>(nullptr));
    _exit(127);
  }
  if(pid>0){
    bag_pid_=pid;
    ROS_INFO_STREAM("[track_simple] 开始记录 rosbag: " << bag_path_);
    return;
  }

  ROS_ERROR("[track_simple] 启动 rosbag 失败");
  bag_path_.clear();
}

void TrackSimple::stopBagRecording(){
  if(bag_pid_<=0) return;

  if(kill(bag_pid_,SIGINT)!=0 && errno!=ESRCH){
    ROS_WARN_STREAM("[track_simple] 无法结束 rosbag (pid=" << bag_pid_ << "): errno=" << errno);
  }

  int status=0;
  if(waitpid(bag_pid_,&status,0)<0){
    ROS_WARN_STREAM("[track_simple] 等待 rosbag 进程结束失败: errno=" << errno);
  }

  if(!bag_path_.empty()){
    ROS_INFO_STREAM("[track_simple] rosbag 已保存: " << bag_path_);
  }
  bag_pid_=-1;
  bag_path_.clear();
}
