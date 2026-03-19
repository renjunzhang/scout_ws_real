# 启动前的注意工作

## ⚠️ 编译注意事项

### catkin_make 白名单问题
如果之前使用过白名单编译单个包：
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=scout_global_planner
```

**白名单会被缓存！** 后续 `catkin_make` 只会编译白名单中的包，其他包的修改不会生效。

**清除白名单**（恢复编译所有包）：
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

### 验证包是否被编译
```bash
catkin_make 2>&1 | grep "processing catkin package"
# 应看到所有需要的包，如 scout_local_planner、scout_global_planner 等
```

---

## 实物流程
### 1. 将底盘和工控机连接并建立 CAN 通信
    sudo modprobe gs_usb   # 若需要
    sudo ip link set can0 down 2>/dev/null || true
    sudo ip link set can0 up type can bitrate 500000
    candump can0          # 如需监听
    roslaunch scout_bringup scout_mini_robot_base.launch
#### 监听状态的脚本
    roslaunch scout_bringup bms_status_monitor.launch     topic:=/BMS_status     period:=60.0
### 2. 启动键盘控制
    roslaunch scout_bringup scout_teleop_keyboard.launch
### 3. 启动激光雷达（真实）
    工作区内 nanoscan3_bringup 包：
    roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
### 4. 建图（真实）
    这个也是在sick_ws里面，有一个nanoscan3_mapping包：
    roslaunch nanoscan3_mapping scout_nanoscan3_gmapping.launch fake_odom_tf:=false use_rviz:=true
#### Cartographer 建图（直接打开新终端，不要手动 source）
    roslaunch nanoscan3_mapping scout_nanoscan3_cartographer.launch
### 5. 定位（真实）
    新开一个终端，运行：
    roslaunch nanoscan3_localization scout_nanoscan3_amcl.launch use_rviz:=true
    或者使用cartographer定位
    roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
### 5.5 启动 IMU（真实，做 IMU / anti-slosh 实验时建议单独启动）
    # 推荐使用独立 launch，不要复用厂家自带的 rviz_and_imu.launch
    # 原因：工控机上通常不需要 RViz，而且需要单独控制 port / topic / frame_id
    roslaunch scout_bringup scout_imu.launch

    # 如果 udev 绑定还没配好，可先临时直连串口
    roslaunch scout_bringup scout_imu.launch port:=/dev/ttyUSB0

    # 启动后先检查
    rostopic list | grep imu
    rostopic echo -n1 /imu/data
    rostopic hz /imu/data

    # 说明：
    # 1. 这一步对普通导航不是硬依赖，但对阶段 7 的 IMU 接入验证是必做步骤
    # 2. 建议放在定位之后、全局规划之前，先把 /imu/data 独立确认好，再继续后面的 planner 链路
### 6. 全局规划
    # 实物对齐：global_planner.yaml 已加入 transform_tolerance（global_costmap/local_costmap）
    # 默认改为旁路速度话题：move_base 输出 /scout/move_base_cmd_vel，不抢占 /cmd_vel
    roslaunch scout_global_planner move_base_global.launch mode:=mpc
    roslaunch scout_global_planner move_base_global.launch mode:=teb
    或者启动自己的简单全局规划器
    roslaunch scout_global_planner simple_global_planner.launch
    或者启动MBF全局规划器
    roslaunch scout_global_planner mbf_global.launch
### 7. MPC 局部规划
    # 普通启动（Q_slosh=0，无晃动抑制）
    roslaunch scout_local_planner test_mpc.launch

    # 带晃动抑制的启动（论文实验用）
    roslaunch scout_local_planner test_mpc.launch Q_slosh:=10.0

    # 或使用实验专用 launch（等价，参数更集中）
    roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=10

    # 液体晃动实验
    roslaunch scout_local_planner slosh_experiment.launch \
    Q_slosh:=5 \
    enable_slosh_box_constraint:=true \
    slosh_speed_governor_enable:=true \
    slosh_speed_governor_k_eta:=2.5 \
    slosh_speed_governor_eta_deadband:=0.3 \
    slosh_speed_governor_eta_exit_ratio:=0.2 \
    slosh_speed_governor_min_active_steps:=10 \
    slosh_speed_governor_ay_max_base:=0.6 \
    slosh_speed_governor_v_des_min:=0.2 \
    slosh_speed_governor_preview_distance:=1.0

### 8. 录制实验数据（与第 7 步同时，另开终端）
    cd $(rospack find scout_local_planner)
    ./scripts/record_slosh_experiment.sh 10           # 参数 = 当前 Q_slosh 值
    # bag 默认输出到 /data/$USER/slosh_bags；若无 /data 则回退到 ~/slosh_bags
    # Ctrl+C 停止录制

### 🔧 关键参数速查（mpc_params.yaml）
| 参数 | 当前值 | 说明 | 实验时是否需要改 |
|------|--------|------|:---:|
| `mpc/Q_slosh` | 0.0 | 晃动抑制权重（消融变量：0/5/10/20） | ✅ 通过 launch arg 切换 |
| `mpc/slosh_height_max` | 0.05 | 液面高度阈值 (m) | ✅ 做峰值约束实验时关注 |
| `mpc/enable_slosh_box_constraint` | false | 第一版液面盒约束代理开关 | ✅ 约束实验时切换 |
| `mpc/Q_ec` | 30.0 | 横向误差权重 | 一般不改 |
| `mpc/Q_v` | 8.0 | 速度跟踪权重 | 一般不改 |
| `vehicle/v_max` | 1.0 | 最大线速度 (m/s) | 按实验场景调 |
| `vehicle/omega_max` | 1.0 | 最大角速度 (rad/s) | 按实验场景调 |
| `path_handler/goal_capture_distance` | 0.50 | 终点捕获区距离 (m) | 终点不收敛时重点看 |
| `path_handler/goal_capture_min_speed` | 0.08 | 捕获区最低参考速度 (m/s) | 终点不收敛时重点看 |
| `slosh/container_radius` | 0.014 | 28 mm 试管内半径 (m) | ✅ 按实物量测 |
| `slosh/liquid_height` | 0.055 | 当前试管液面高度 (m) | ✅ 按实物量测 |
| `slosh/liquid_density` | 1000.0 | 液体密度 (kg/m³) | 水=1000 |
| `slosh/damping_ratio` | 0.12 | 28 mm 水试管初始阻尼比 | 按辨识实验继续修正 |
| `slosh_estimator/accel_filter_alpha` | 0.3 | EMA 滤波系数 (0,1] | 实物抖动大可降低 |
| `slosh_speed_governor/enable` | false | 残余晃动感知速度治理开关 | ✅ 阶段 4 实验切换 |
| `slosh_speed_governor/k_eta` | 2.5 | 液面高度比例缩放系数 | governor 调参核心项 |
| `slosh_speed_governor/eta_deadband` | 0.3 | 死区阈值 | governor 调参核心项 |
| `slosh_speed_governor/eta_exit_ratio` | 0.2 | 退出阈值（滞回） | governor 稳定性关键项 |
| `slosh_speed_governor/preview_distance` | 1.0 | 前方曲率预览长度 (m) | governor 调参核心项 |
| `slosh_speed_governor/min_active_steps` | 10 | 最少保持周期数 | governor 稳定性关键项 |

说明：
当前 `scout_local_planner` 实验入口实际读取的 slosh 参数真源是 [mpc_params.yaml](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params.yaml)；`slosh_models/config/slosh_params.yaml` 仅保留为建模参考示例，不会被 `slosh_experiment.launch` 自动加载。

---

## 仿真流程
### 1. 启动仿真环境
    roslaunch scout_description scout_mini_gazebo.launch use_rviz:=false
### 2. 启动键盘控制（仿真）
    roslaunch scout_bringup scout_teleop_keyboard.launch
### 3. 启动激光雷达（仿真，可选）
    工作区内 nanoscan3_bringup 包：
    roslaunch nanoscan3_bringup nanoscan3_front_sim.launch use_rviz:=false
    说明：
    - 该 launch 会把 `/scan` relay 成 `/scan_front`，供旧的建图/定位流程使用。
    - 当前仿真已统一使用 `/scan`，此步可省略。
    - 如需启动该 launch，保持 `publish_static_tf:=false`，避免与 URDF 中雷达 TF 冲突。
### 4. 建图（仿真）
    方式一：Gmapping（默认）
    roslaunch nanoscan3_mapping scout_nanoscan3_gmapping_sim.launch use_rviz:=true
    
    方式二：Cartographer（推荐，精度更高）
    # 需先 source Cartographer 工作空间
    source ~/scout_ws/devel/setup.bash
    source ~/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash
    roslaunch nanoscan3_mapping scout_nanoscan3_cartographer_sim.launch
    
    说明：
    - Cartographer 需从源码编译，安装步骤见 sensors/说明.md
    - Cartographer 使用在线相关性扫描匹配，对仿真里程计噪声更鲁棒
    - 建图完成后保存：rosrun map_server map_saver -f ~/scout_ws/src/scout_apps/scout_maps/maps/map_carto
### 5. 定位（仿真）
    roslaunch nanoscan3_localization scout_nanoscan3_amcl_sim.launch use_rviz:=true
    不能source，source会覆盖cartographer的环境
    roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization_sim.launch
### 6. 全局规划
    roslaunch scout_global_planner move_base_global_sim.launch mode:=mpc
    roslaunch scout_global_planner move_base_global_sim.launch mode:=teb
    或者启动自己的简单全局规划器
    roslaunch scout_global_planner simple_global_planner_sim.launch
    或者启动MBF全局规划器
    roslaunch scout_global_planner mbf_global_sim.launch
### 7. MPC 局部规划
    # 普通启动（Q_slosh=0，无晃动抑制）
    roslaunch scout_local_planner test_mpc_sim.launch

    # 带晃动抑制的启动（论文实验用）
    roslaunch scout_local_planner test_mpc_sim.launch Q_slosh:=10.0

    # 或使用实验专用 launch（自动选 sim YAML）
    roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=10 sim:=true

    # TEB 对比实验
    roslaunch teb_local_planner test_teb_sim.launch

    # 液体晃动实验
    roslaunch scout_local_planner slosh_experiment.launch \
    sim:=true \
    Q_slosh:=5 \
    enable_slosh_box_constraint:=true \
    slosh_speed_governor_enable:=true \
    slosh_speed_governor_k_eta:=2.5 \
    slosh_speed_governor_eta_deadband:=0.3 \
    slosh_speed_governor_eta_exit_ratio:=0.2 \
    slosh_speed_governor_min_active_steps:=10 \
    slosh_speed_governor_ay_max_base:=0.6 \
    slosh_speed_governor_v_des_min:=0.2 \
    slosh_speed_governor_preview_distance:=1.0

### 8. 录制实验数据（与第 7 步同时，另开终端）
    cd $(rospack find scout_local_planner)
    ./scripts/record_slosh_experiment.sh 10           # 参数 = 当前 Q_slosh 值
#### 消融实验
    rosrun scout_local_planner send_fixed_goal.py \
    --goal-topic /scout/goal \
    --frame map \
    --x 1.00 \
    --y 1.12 \
    --yaw 0.0

    rosrun scout_local_planner send_fixed_goal.py \
    --goal-topic /scout/goal \
    --frame map \
    --x 0.5 \
    --y 1.12 \
    --yaw 0.0

### 🔧 关键参数速查（mpc_params_sim.yaml）
| 参数 | 当前值 | 说明 | 实验时是否需要改 |
|------|--------|------|:---:|
| `mpc/Q_slosh` | 0.0 | 晃动抑制权重（消融变量：0/5/10/20） | ✅ 通过 launch arg 切换 |
| `mpc/slosh_height_max` | 0.05 | 液面高度阈值 (m) | ✅ 做峰值约束实验时关注 |
| `mpc/enable_slosh_box_constraint` | false | 第一版液面盒约束代理开关 | ✅ 约束实验时切换 |
| `vehicle/v_max` | 2.0 | 最大线速度 (m/s) | 仿真可保持较大 |
| `vehicle/omega_max` | 3.5 | 最大角速度 (rad/s) | 仿真可保持较大 |
| `path_handler/goal_capture_distance` | 0.45 | 终点捕获区距离 (m) | 终点不收敛时重点看 |
| `path_handler/goal_capture_min_speed` | 0.10 | 捕获区最低参考速度 (m/s) | 终点不收敛时重点看 |
| `slosh/container_radius` | 0.014 | 28 mm 试管内半径 (m) | ✅ 按实验设定 |
| `slosh/liquid_height` | 0.055 | 当前试管液面高度 (m) | ✅ 按实验设定 |
| `slosh/liquid_density` | 1000.0 | 液体密度 (kg/m³) | 水=1000 |
| `slosh/damping_ratio` | 0.12 | 28 mm 水试管初始阻尼比 | 按辨识实验继续修正 |
| `slosh_speed_governor/enable` | false | 残余晃动感知速度治理开关 | ✅ 阶段 4 实验切换 |
| `slosh_speed_governor/k_eta` | 2.5 | 液面高度比例缩放系数 | governor 调参核心项 |
| `slosh_speed_governor/eta_deadband` | 0.3 | 死区阈值 | governor 调参核心项 |
| `slosh_speed_governor/eta_exit_ratio` | 0.2 | 退出阈值（滞回） | governor 稳定性关键项 |
| `slosh_speed_governor/preview_distance` | 1.0 | 前方曲率预览长度 (m) | governor 调参核心项 |
| `slosh_speed_governor/min_active_steps` | 10 | 最少保持周期数 | governor 稳定性关键项 |

### 9.实验数据分析
  结束实验后，使用 extract_slosh_metrics.py 提取晃动指标（需安装 rosbag；脚本本身只依赖 Python 标准库）
  只提取单个 bag：
  python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
    /data/a/slosh_bags/slosh_Q5_20260307_160207.bag \
    /data/a/slosh_bags/slosh_Q10_20260307_160423.bag \
    --per-episode \
    --csv /tmp/slosh_metrics.csv
  如果要扫整个目录：
  python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
    /data/a/slosh_bags/0308 \
    --per-episode \
    --csv /data/a/slosh_bags/0308_data_processing/slosh_metrics.csv

    






# 修改记录（scout_ws）
## 2026-01-26
- 调整 BMS 监控默认输出周期：`bms_status_monitor.py` 与 `bms_status_monitor.launch` 的 `period` 默认由 180 秒改为 60 秒，便于更快看到电池状态。
- 运行方式：
  1. 每个终端先 `source ~/scout_ws/devel/setup.bash`
  2. 启动监控：
     ```bash
     roslaunch scout_bringup bms_status_monitor.launch topic:=/BMS_status period:=60.0
     ```
  3. 需要其他频率可调整 `period` 参数（单位秒）。

## 2026-01-27
- Gmapping 无地图显示的问题确认：RViz 需添加 `Map` 显示并选择 `/map`，且 `Fixed Frame` 设为 `map`。
- 将 `nanoscan3_bringup` 与 `nanoscan3_mapping` 从 sick_ws 复制到本工作空间：`src/scout_apps/sensors/nanoscan3_bringup`、`src/scout_apps/sensors/nanoscan3_mapping`。
- 地图改为独立 ROS 包：新增 `src/scout_apps/scout_maps`，地图文件移动到 `scout_maps/maps/`，AMCL 默认地图路径改为 `$(find scout_maps)/maps/map122.yaml`。
- 修正地图配置：`map122.yaml` 的 `image` 改为相对路径 `map122.pgm`，避免硬编码绝对路径。
- 更新文档说明：补充 `sensors/说明.md` 三个功能包说明，并同步 `nanoscan3_localization/README.md` 的地图保存与启动示例。

## 2026-01-28
- 当前 TF 链路确认：
  - `scout_mini_robot_base.launch` 发布 `odom -> base_link`。
  - `nanoscan3_front.launch` 发布 `base_link -> nanoscan3_front`，并提供 `/scan_front_filtered`。
  - `scout_nanoscan3_gmapping.launch` 发布 `map -> odom`，形成 `map -> odom -> base_link -> nanoscan3_front`。
- 计划：将建图/定位算法从 Gmapping 切换为 Cartographer。

- **新建 `control` 模块包结构**：
  - 创建 `slosh_models` 包：液体晃动建模库
    - 从 `/home/a/scout_ws/docs` 迁移 `liquid_slosh_model.cpp/h` 和 `mpc_vel_tracker.cpp/h`
    - 将命名空间从 `communication_rs485` 改为 `slosh_models`
    - 添加标准 ROS 包结构（CMakeLists.txt, package.xml）
    - 包含配置文件 `slosh_params.yaml`
  - 创建 `scout_local_planner` 包：MPC局部规划器
    - 集成 `slosh_models` 库
    - 提供局部路径跟踪功能，支持液体晃动抑制软约束
    - 添加 ROS 节点、配置文件和 launch 文件
    - 话题：订阅 `/odom`, `/global_path`，发布 `/cmd_vel`, `/slosh_height`

- **新建 `navigation` 模块包结构**：
  - 创建 `scout_global_planner` 包：全局路径规划器框架
    - 预留自定义规划器接口
    - 添加全局规划参数配置 `global_planner.yaml`
    - 支持与 ROS 导航栈集成（move_base）

- **更新说明文档**：
  - 更新 `control/说明.md`：详细说明控制模块的包结构、使用方法和修改记录
  - 更新 `navigation/说明.md`：说明导航模块的功能和集成方式

- **代码迁移说明**：
  - 原始液体建模代码位于 `/home/a/scout_ws/docs`，已迁移至 `control/slosh_models`
  - MPC控制器已封装为 ROS 节点，可独立运行
  - 液体晃动约束使用增广状态空间法，通过 OSQP 求解器实现

- **全局规划对接与测试步骤**：
  - 启动：`roslaunch scout_global_planner move_base_global.launch`
  - RViz 中设置 Fixed Frame 为 `map`，并将 "2D Nav Goal" 话题改为 `/scout/goal`
  - 全局路径话题：`/scout/global_path`（类型 `nav_msgs/Path`）
  - 在 RViz 中添加 `Path` 显示并选择该话题即可查看规划路径

- **全局路径话题 remap 修复**：
  - 将 move_base 私有话题 `~GlobalPlanner/plan` 与 `~NavfnROS/plan` 重映射为 `/scout/global_path`
  - 修正 `/scout/global_path` 无输出问题，路径可在 RViz 显示

- **OSQP 和依赖安装（新工控机配置参考）**：
  > 注意：Ubuntu 20.04 apt 源中没有 libosqp-dev，需要从源码安装。
  
  1. **安装 ROS 导航相关依赖**：
    ```bash
     sudo apt-get install ros-noetic-ros-base ros-noetic-roscpp \
       ros-noetic-nav-core ros-noetic-costmap-2d ros-noetic-base-local-planner \
       ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf \
       ros-noetic-laser-geometry ros-noetic-laser-filters \
       ros-noetic-amcl ros-noetic-gmapping ros-noetic-eigen-conversions \
       ros-noetic-sick-safetyscanners ros-noetic-rviz \
       ros-noetic-teleop-twist-keyboard ros-noetic-map-server \
       ros-noetic-move-base ros-noetic-global-planner ros-noetic-dwa-local-planner
     ```
  
  2. **下载本地 CMake 3.18+**（用于编译 OSQP，不替换系统 CMake）：
     ```bash
     cd ~
     wget https://github.com/Kitware/CMake/releases/download/v3.18.6/cmake-3.18.6-Linux-x86_64.tar.gz
     tar -xzf cmake-3.18.6-Linux-x86_64.tar.gz
     export PATH=~/cmake-3.18.6-Linux-x86_64/bin:$PATH
     ```
  
  3. **从源码安装 OSQP v0.6.2**（注意使用 v0.6.2 分支，最新版需要 CMake 3.18+）：
     ```bash
     cd ~
     git clone --recursive -b v0.6.2 https://github.com/osqp/osqp
     cd osqp && mkdir build && cd build
     cmake ..
     make -j4
     sudo make install
     ```
  
  4. **安装 osqp-eigen**：
     ```bash
     cd ~
     git clone https://github.com/robotology/osqp-eigen.git
     cd osqp-eigen && mkdir build && cd build
     cmake ..
     make -j4
     sudo make install
     sudo ldconfig
     ```
  
  5. **编译工作空间**：
     ```bash
     source /opt/ros/noetic/setup.bash
     cd ~/scout_ws
     catkin_make -j4
     source devel/setup.bash
     ```

- **创建 scout_global_planner 的 include 目录**（避免 CMake 报错）：
  ```bash
  mkdir -p ~/scout_ws/src/scout_apps/navigation/scout_global_planner/include/scout_global_planner
  ```

- **编译成功的包**：
  - `slosh_models` - 液体晃动模型库（已集成 OSQP 求解器）
  - `scout_local_planner` - MPC 局部规划器
  - `scout_global_planner` - 全局规划器框架
  - `nanoscan3_bringup/mapping/localization` - 激光雷达相关
  - `scout_base/bringup/msgs` - 底盘驱动相关


## 2026-01-29（实际修改）
- 修正 MPC 控制变化率代价：加入 `(u_k - u_{k-1})^2` 跨步耦合项，避免被当作额外 `u^2`。
- 参考点生成前更新 `current_s_`，并让 `lookahead_distance` 生效。
- 线性化名义轨迹随步推进（不再固定 x0）。
- 路径持续使用时刷新时间戳，避免静态路径 5s 超时停住。
- 更新 `docs/topic_list.md`：补充 `/rosout`、`/rosout_agg` 等观测话题并校对命名空间。
- 启动 test_mpc.launch 则对接 `/scout/global_path`，稳定输出 `/cmd_vel`

### 2026-01-29（文档更新）
- 将启动流程拆分为“实物流程”和“仿真流程”，明确各步骤对应的启动命令。
- 仿真流程补充说明：`nanoscan3_front_sim.launch` 仅用于将 `/scan` relay 为 `/scan_front`，当前仿真统一使用 `/scan` 可省略；如需启用需保持 `publish_static_tf:=false` 以避免 TF 冲突。

## 2026-01-30（Cartographer 修复与环境配置）

### 问题 1：libglog.so.1 找不到
- **现象**：运行时报错 `libglog.so.1: cannot open shared object file`
- **根因**：`~/.bashrc` 中 Conan 版本的 glog (0.6.0) 依赖 `libglog.so.1`，而系统只有 `libglog.so.0`
- **解决**：注释掉 `~/.bashrc` 中冲突的 Conan 路径（gflags、glog、ceres-solver），保留 osqp 和 libunwind

### 问题 2：cartographer_ros 包找到错误路径
- **现象**：`rospack find cartographer_ros` 返回 `chemist_robot3.0` 工作空间中的源码目录
- **根因**：`~/.bashrc` 中 source 了多个工作空间，`chemist_robot3.0` 含有未编译的 cartographer_ros 源码
- **解决**：从 `~/.bashrc` 中删除 `source /home/a/chemist_robot3.0/devel/setup.bash`

### 问题 3：手动 source 覆盖环境
- **现象**：打开终端后手动执行 `source ~/scout_ws/devel/setup.bash`，导致 cartographer_ws 路径丢失
- **根因**：`~/.bashrc` 已按正确顺序配置了 source，手动 source 会覆盖后续配置
- **解决**：**不要手动 source**，直接打开新终端即可

### 最终 ~/.bashrc 配置（关键部分）
```bash
# scout_ws 主工作空间
source ~/scout_ws/devel/setup.bash

# Cartographer 工作空间（必须在主工作空间之后）
export CATKIN_SETUP_UTIL_ARGS="--extend"
source ~/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash
export ROS_PACKAGE_PATH=/home/a/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/share:$ROS_PACKAGE_PATH
```

### Cartographer 仿真建图启动命令
```bash
# 终端 1：启动仿真环境
roslaunch scout_description scout_mini_gazebo.launch use_rviz:=false

# 终端 2：键盘控制
roslaunch scout_bringup scout_teleop_keyboard.launch

# 终端 3：激光雷达（仿真）
roslaunch nanoscan3_bringup nanoscan3_front_sim.launch use_rviz:=false

# 终端 4：Cartographer 建图（直接打开新终端，不要手动 source）
roslaunch nanoscan3_mapping scout_nanoscan3_cartographer_sim.launch
```

### 保存地图
```bash
# Cartographer 需要先调用服务完成轨迹，再保存
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '/home/geist/scout_ws/src/scout_apps/scout_maps/maps/map_carto122_3.pbstream'}"

# 转换为标准地图格式
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=/home/geist/scout_ws/src/scout_apps/scout_maps/maps/map_carto122_3.pbstream \
  -map_filestem=/home/geist/scout_ws/src/scout_apps/scout_maps/maps/map_carto122_3 \
  -resolution=0.02
```

### 验证环境配置
```bash
# 新终端中执行
rospack find cartographer_ros
# 应输出：/home/a/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/share/cartographer_ros

rospack find nanoscan3_mapping
# 应输出：/home/a/scout_ws/src/scout_apps/sensors/nanoscan3_mapping

ldd ~/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/lib/cartographer_ros/cartographer_node | grep "not found"
# 应无输出
```
### 对于仿真的全局规划的修改
- 仿真全局规划：新增 `scout_global_planner/config/global_planner_sim.yaml`，降低静态层阈值并启用未知区域跟踪，避免 Cartographer 地图代价（~99）导致路径穿墙。
- `move_base_global_sim.launch` 改为加载仿真配置，补充 `remap_goal` 参数，默认将 `cmd_vel` 输出到 `/cmd_vel`，并保持根命名空间启动。
- 更新 Cartographer 定位 RViz 配置：`nanoscan3_localization/rviz/cartographer_localization.rviz` 增加机器人模型/TF/坐标轴显示，便于仿真定位调试。

## 2026-01-31（计划）
- 首先进行 **简单 MPC**（不含液体晃动），参照 `docs/change_plan.md`：
  - 路径处理与 Frenet 转换：`path_handler.h/.cpp`
  - 差速动力学模型：`diff_drive_model.h/.cpp`
  - MPC 求解器框架：`mpc_solver.h/.cpp`
  - 配置文件与启动入口：`config/mpc_params.yaml`、`launch/test_mpc.launch`
- 验证：对接 `/scout/global_path`，稳定输出 `/cmd_vel
- 进行 **液体晃动模型集成（第 2 步）**，参照 `docs/change_plan.md`：
  - 晃动动力学扩展：`diff_drive_slosh_model.h/.cpp`
  - 晃动约束模块：`slosh_models/slosh_constraint.h/.cpp`
  - 状态扩展与代价/约束调整（`types.h` / `cost_function.*` / `constraint_manager.*`）
  - 输出晃动调试话题（如 `slosh_height`）

## 2026-01-31（实改：MPC 平滑路径与仿真/实物对齐）
- **MPC 新增平滑路径可选发布**  
  - 内部仍基于“局部三次样条”生成参考点，MPC 控制逻辑不变。  
  - 新增参数：`path_handler.publish_smoothed_path`、`path_handler.smoothed_path_topic`、`path_handler.smoothed_path_points`。  
  - 仿真默认开启：`config/mpc_params_sim.yaml` → `/scout/global_path_smooth`。  
  - 平滑路径 frame_id 为 `base_link`（局部样条在 base_link 坐标系）。

- **仿真 vs 实物：MPC 输入/输出/命名空间/TF 对齐（便于移植）**  
  - **实物启动**：`scout_local_planner/launch/test_mpc.launch`  
  - **仿真启动**：`scout_local_planner/launch/test_mpc_sim.launch`  
  - **输入话题（两者一致）**：  
    - 全局路径：`/scout/global_path`（`nav_msgs/Path`）  
    - 里程计：`/odom`（`nav_msgs/Odometry`）  
    - TF：`map -> odom -> base_link`（PathHandler 内部使用 TF 将路径转到 base_link）  
  - **输出话题（两者一致）**：  
    - 速度指令：`/cmd_vel`（`geometry_msgs/Twist`）  
    - 预测轨迹：`/local_path`（`nav_msgs/Path`）  
    - 状态信息：`/mpc_status`  
  - **仿真特有**：  
    - 使用 `mpc_params_sim.yaml`，默认发布 `/scout/global_path_smooth`（仅可视化，不参与控制）。  
    - 仿真有 `/clock`，use_sim_time 生效。  
  - **实物特有**：  
    - 使用 `mpc_params.yaml`，默认不发布平滑路径话题。
  
- **路径处理增强**  
  - 新增参数：`window_back` / `window_forward`（样条窗口）、`resample_spacing`（路径重采样）、`max_lat_accel`（曲率限速）、`min_ref_speed`（参考速度下限）。  
  - 路径可按固定间距重采样，减少折线噪声；参考速度根据曲率自动限速。
- **终点航向判定修复**  
  - `isGoalReached()` 使用路径末端**切线方向**作为目标航向，避免终点姿态默认值导致不收敛。
- **仿真参数调优**  
  - 提升线速度与转弯能力：`v_max`、`a_max`、`omega_max`、`alpha_max`、`Q_v`、`lookahead_distance`、`max_lat_accel` 等在 `mpc_params_sim.yaml` 调整。

## 2026-01-31（计划：借鉴 mpc_planner 的改进思路）
- **目标**：在现有 `scout_local_planner` 上逐步引入更强的轨迹跟踪与速度规划能力，不替换为 move_base 插件。  
- **改进思路（按顺序实施）**：
  1. **时间化速度规划 v(s)**  
     - 基于路径弧长生成速度曲线 `v(s)`  
     - 前向/后向遍历约束加减速，得到可行速度分布  
     - MPC 按时间采样参考点（真正时间化，而非固定 v_ref）  
  2. **contour + lag 误差结构**  
     - 强化横向/纵向误差分离，提升转弯稳定性  
     - 与现有 Frenet 误差对齐，优化权重结构  
  3. **第三步：安全退化策略说明**  
     - 求解失败时执行“制动/降速”逻辑  
     - 防止指令突变或停滞，提升鲁棒性  

## 2026-01-31（计划：路径平滑方案）
- **A. 安全版（推荐）**  
  只对局部窗口做 B‑spline 平滑（base_link 坐标系），不改变全局路径，避免穿墙。  
  → 仅影响 MPC 内部参考点。  

## 2026-01-31（仿真：MPC 速度与响应调优）
- `mpc_params_sim.yaml` 调整以提升速度响应与转弯执行：
  - 降低 `Q_ec/Q_etheta`，提高 `Q_v`
  - 降低 `R_a/R_alpha/R_da/R_dalpha`
  - 提高 `v_max/a_max/alpha_max`
  - 放宽 `max_lat_accel`，提高 `min_ref_speed`
  - 提高 `max_tan_accel/max_tan_decel`，将 `goal_speed` 调到 0.4
  - 增大 `omega_max/alpha_max` 并降低 `R_alpha`，提升转向能力

## 2026-01-31（仿真：原地对齐模式）
- 新增 Heading Align：航向误差过大时 `v=0` 原地转向
- 参数：`heading_align/enable`、`enter_angle`、`exit_angle`、`omega_gain`、`max_omega`
- 仿真默认开启，实物默认关闭

## 2026-02-05（Cartographer 定位启动问题修复）

### 日常启动 Cartographer 定位（实物）

> ⚠️ **重要**：打开新终端后**不要手动 source**，直接运行即可。手动 `source devel/setup.bash` 会覆盖 `~/.bashrc` 中的 cartographer 环境配置。

```bash
# 终端 1：底盘
roslaunch scout_bringup scout_mini_robot_base.launch

# 终端 2：激光雷达
roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false

# 终端 3：Cartographer 定位（直接打开新终端运行）
roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
```

### 遇到的问题：Cannot locate node of type [cartographer_node]

**错误信息**：
```
ERROR: cannot launch node of type [cartographer_ros/cartographer_node]: Cannot locate node of type [cartographer_node] in package [cartographer_ros]. Make sure file exists in package path and permission is set to executable (chmod +x)
```

**根本原因**：
- `catkin_make_isolated --install` 编译方式将可执行文件安装到 `install_isolated/lib/cartographer_ros/` 目录
- 但 `rosrun`/`roslaunch` 只在 `install_isolated/share/cartographer_ros/` 目录下查找可执行文件
- 两个目录之间没有自动创建链接

**解决方案**：在 `share/cartographer_ros/` 目录下创建到 `lib/cartographer_ros/` 可执行文件的符号链接

```bash
CARTO_INSTALL=~/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated
cd ${CARTO_INSTALL}/share/cartographer_ros
ln -sf ../../lib/cartographer_ros/cartographer_node .
ln -sf ../../lib/cartographer_ros/cartographer_occupancy_grid_node .
ln -sf ../../lib/cartographer_ros/cartographer_offline_node .
ln -sf ../../lib/cartographer_ros/cartographer_assets_writer .
ln -sf ../../lib/cartographer_ros/cartographer_pbstream_to_ros_map .
ln -sf ../../lib/cartographer_ros/cartographer_pbstream_map_publisher .
```

**验证修复**：
```bash
# 新终端中执行
rosrun cartographer_ros cartographer_node --help
# 应显示帮助信息，不报错
```

**备注**：此问题在重新编译 Cartographer 后可能需要再次创建符号链接。




## 2026-02-05
- 新增 MBF 全局规划 launch（实物/仿真）：
  - `scout_global_planner/launch/mbf_global.launch`
  - `scout_global_planner/launch/mbf_global_sim.launch`
  - 仅维护 costmap + GetPath，不控制底盘，`cmd_vel` 旁路到 `/scout/move_base_cmd_vel`。
- 新增 MBF GetPath 路径发布节点：
  - `scout_global_planner/src/mbf_path_publisher_node.cpp`
  - 订阅 `goal`，调用 `mbf_costmap_nav/get_path`，发布到 `/scout/global_path`。
- 修复 MBF planners 参数替换问题：
  - `mbf_global*.launch` 中 `planners` 增加 `subst_value="true"`。
- `scout_global_planner` 增加依赖与编译目标：
  - CMake 增加 `actionlib`、`mbf_msgs`，新增可执行 `mbf_path_publisher_node`。
  - `package.xml` 增加依赖 `actionlib`、`mbf_msgs`。
- simple_global_planner 增加硬膨胀：
  - `simple_global_planner_node.cpp` 新增 `inflation_radius`，构建膨胀栅格用于 A* 与平滑。
  - `simple_global_planner*.launch` 新增参数 `inflation_radius`（默认 0.35m）。
- simple_global_planner 改为每周期重算路径（方案 A）。
- MPC 本地轨迹可视化起点对齐：
  - `scout_local_planner/src/local_planner_ros.cpp` 的 `publishLocalPath()` 先插入 base_link 原点，确保 `/local_path` 从车体起点显示。




## 2026-02-06（实物定位优化与 MPC 参数调整）

### 问题：刹车和大角速度时定位漂移
- **根本原因**：差速底盘在急刹车时轮子打滑、大角速度时里程计累积误差

### 修复 1：MPC 参数调整（减少极端运动）
- `mpc_params.yaml` 调整：
  | 参数 | 原值 | 新值 | 原因 |
  |-----|-----|-----|------|
  | `v_max` | 1.5 | 1.0 | 降低最大速度 |
  | `omega_max` | 1.5 | 1.0 | 降低最大角速度 |
  | `a_max` | 2.0 | 1.5 | 减少刹车打滑 |
  | `alpha_max` | 4.0 | 2.5 | 平滑转向 |
  | `max_tan_decel` | 2.0 | 1.2 | 减少急刹车 |
  | `max_lat_accel` | 2.0 | 1.5 | 减少激进转弯 |
  | `s_jump_threshold` | 0.5 | 0.8 | 容忍定位抖动 |

### 修复 2：AMCL 参数优化
- `scout_nanoscan3_amcl.launch` 调整：
  - 粒子数：500-5000 → 800-8000（提高鲁棒性）
  - 更新频率：0.2m/0.5rad → 0.1m/0.2rad（更频繁更新）
  - 里程计噪声：alpha1-4 从 0.2 增大到 0.3-0.5（容忍漂移）
  - 新增 `transform_tolerance: 0.5`

### 修复 3：Cartographer 定位参数优化
- `scout_2d_localization.lua` 调整：
  - 搜索窗口：0.15m/30° → 0.25m/45°（容忍更大漂移）
  - 增加运动滤波器配置，更频繁更新
  - 增大 `rotation_weight` 减少大角速度时的漂移

### 修复 4：TF 时间戳问题
- `path_handler.cpp::isGoalReached()` 修复：
  - 原问题：使用 `global_path_.header.stamp` 导致 TF 查询过期
  - 修复：改用 `ros::Time(0)` 获取最新变换

---

## 2026-03-06 P0-A：液体晃动状态注入（slosh integration wiring）

### 目的
将 `SloshIntegration` 晃动模型实际接入 MPC 控制回路，使 x0 的晃动维度 [η_x, η̇_x, η_y, η̇_y] 反映真实液体状态，不再永远为零。

**关键约束**：`Q_slosh` 保持 0.0，本步仅验证"状态真的活了"，不改 QP 目标函数。

### 修改文件

#### 1. `local_planner_ros.h`
- 新增 `#include "slosh_integration.h"` 及 `std_msgs/Float32.h`, `Float32MultiArray.h`, `Int32.h`
- 新增成员：
  - `SloshIntegration slosh_integration_` — 晃动模型实例
  - `bool slosh_enabled_` — 运行时开关
  - `SloshParams slosh_params_` — 从 YAML 加载的晃动参数
  - 加速度估计状态：`prev_v_`, `prev_omega_`, `has_prev_odom_`, `ax_filtered_`, `ay_filtered_`, `alpha_filtered_`, `accel_filter_alpha_`
  - 调试发布者：`slosh_state_pub_`, `slosh_height_pub_`, `mpc_solve_ms_pub_`, `mpc_status_val_pub_`
- 新增方法：`publishSloshDebug(double solve_time_ms, bool solve_ok)`

#### 2. `local_planner_ros.cpp`
- **loadParameters()**：新增读取 `slosh/*`（container_radius, liquid_height, liquid_density, damping_ratio, mode_index, offset_x, offset_y, use_parabola_term, use_linear_model）、`slosh_estimator/accel_filter_alpha`、`mpc/slosh_height_max`
- **initialize()**：调用 `slosh_integration_.configure(slosh_params_)`，通过 `dynamic_pointer_cast<DiffDriveModel>` 获取模型并调用 `setSloshIntegration()`，创建 4 个调试发布者
- **controlLoop()**：
  - 加速度估计：odom 差分 → EMA 低通滤波（`ax`, `ay≈v*ω`, `alpha`）
  - 调用 `slosh_integration_.update(ax, ay, omega, alpha)` 推进模型
  - 调用 `slosh_integration_.writeToAugmentedState(x0)` 将 [η_x, η̇_x, η_y, η̇_y] 写入 MPC 初始状态
  - MPC 求解后调用 `publishSloshDebug()`
- **resetWarmStart()**：重置 `slosh_integration_.reset()` 和加速度滤波状态
- **publishSloshDebug()**：发布 `/slosh/state`（Float32MultiArray[4]）、`/slosh/height`（Float32）、`/mpc/solve_ms`（Float32）、`/mpc/status_val`（Int32）

#### 3. `mpc_solver.h`
- 新增 `getDynamicsModel()` getter 方法

#### 4. `mpc_params.yaml`（实物）& `mpc_params_sim.yaml`（仿真）
- 新增 `slosh:` 参数块（当前运行时真源；现按 28 mm 试管参数维护）
- 新增 `slosh_estimator:` 参数块（accel_filter_alpha=0.3）

### 验证方法
```bash
# 运行后检查 slosh 话题
rostopic echo /slosh/state     # 应看到非零、随运动变化的 [η_x, η̇_x, η_y, η̇_y]
rostopic echo /slosh/height    # 应看到随加减速/转弯波动的液面高度值
rostopic hz /mpc/solve_ms      # 确认发布频率 ~20Hz
```

### 后续（P0-B）
在确认状态注入正确后，将 `Q_slosh` 设为非零值以在 QP 代价函数中惩罚晃动。

---

## P0-B：晃动软代价闭环 + 动力学/估计一致性修复

### 目的
让 `Q_slosh` 真正进入 QP 目标函数，使优化器能感知并抑制液体晃动；同时修复 P0-A 遗留的 4 个一致性问题。

### 修复清单

#### Fix #1: linearize() 缺失的 ∂ay/∂v 耦合项
- **文件**: `diff_drive_model.cpp` linearize()
- **问题**: `ay = v * omega` 在 predict() 中被正确使用，但 linearize() 只写了 `∂ay/∂omega = v`（→ B 矩阵），遗漏了 `∂ay/∂v = omega`（→ A 矩阵）。导致速度变化对横向晃动激励的灵敏度被低估，转弯时优化器判断偏乐观。
- **修复**: 新增 `A(ETA_*, V) += B_slosh[:,1] * omega_lin`，从线性化点的 omega 提取偏导

#### Fix #2: 加速度估计改用真实 odom 时间戳差分
- **文件**: `local_planner_ros.h` + `local_planner_ros.cpp`
- **问题**: 原实现用固定 `1/control_rate_` 做差分，实物 odom 频率抖动时 ax/alpha 会被系统性算错，直接污染晃动注入状态
- **修复**:
  - `odomCallback()` 缓存 `current_odom_time_`
  - `controlLoop()` 用 `(current_odom_time_ - prev_odom_time_).toSec()` 做差分
  - 增加合理性保护 `1e-4 < dt_odom < 1.0`

#### Fix #3: DiffDriveModel 注入失败时关闭 slosh
- **文件**: `local_planner_ros.cpp` initialize()
- **问题**: `dynamic_pointer_cast<DiffDriveModel>` 失败时，`slosh_enabled_` 仍为 true，会继续更新 slosh 并写入 x0，但求解器模型不预测这些状态 → 静默失配
- **修复**: cast 失败时 `slosh_enabled_ = false`

#### Fix #4: `getSloshCostMatrix()` 乘 height_coeff²
- **文件**: `slosh_integration.cpp`
- **问题**: 原实现返回 `diag([Q_slosh, 0, Q_slosh, 0])`，缺少 height_coeff² 缩放，与"惩罚液面高度平方"的物理语义不一致
- **修复**: `Q_eta = Q_slosh * h_coeff²`，返回 `diag([Q_eta, 0, Q_eta, 0])`

#### Fix #5: cost_function 加 Q_slosh 软代价
- **文件**: `types.h` + `cost_function.cpp` + `local_planner_ros.cpp`
- **机制**:
  1. `types.h::MPCParams` 新增 `Q_slosh_eta` 字段（运行时计算）
  2. `local_planner_ros.cpp` initialize() 在 slosh 配置成功后计算 `Q_slosh_eta = Q_slosh * height_coeff²`，通过 `setMPCParams()` 同步到求解器
  3. `cost_function.cpp::StateTrackingCost::getQuadraticCost()` 读取 `Q_slosh_eta`，写入 `Q_total(ETA_X, ETA_X)` 和 `Q_total(ETA_Y, ETA_Y)`
  4. 经由 `buildQPCost()` 统一乘 2.0 进入 OSQP P 矩阵（与 Change #11 一致）
- **安全性**: `Q_slosh=0.0` 时 `Q_slosh_eta=0.0`，QP 完全不受影响（向后兼容）

### 当前 Q_slosh 仍为 0.0
代码路径已完备，但 YAML 配置保持 `Q_slosh: 0.0`。验证方法：

```bash
# 设为非零值测试晃动抑制效果
rosparam set /local_planner_node/mpc/Q_slosh 5.0
# 对比 /slosh/height 的 RMS
```

### 修改文件汇总
| 文件 | 修改内容 |
|------|---------|
| `diff_drive_model.cpp` | linearize() 补 A[:, V] 晃动耦合 |
| `local_planner_ros.h` | 新增 `prev_odom_time_`, `current_odom_time_` |
| `local_planner_ros.cpp` | odom 时间戳缓存; 真实 dt 差分; cast 失败关闭 slosh; 计算 Q_slosh_eta; resetWarmStart 重置时间戳 |
| `slosh_integration.cpp` | getSloshCostMatrix 乘 height_coeff² |
| `types.h` | MPCParams 新增 Q_slosh_eta |
| `cost_function.cpp` | StateTrackingCost 加 ETA_X/ETA_Y 软代价 |

---

## 方案 2：论文实验基础设施

### 目的
在 P0-B（Solution 1 闭环修复）基础上，添加论文消融实验所需的完整观测、参数切换和数据录制能力。

### 改动清单

#### 1. 加速度估计调试话题（3 个新 publisher）
- **文件**: `local_planner_ros.h` + `local_planner_ros.cpp`
- **新增话题**:
  - `/slosh/ax_est` (Float32) — EMA 滤波后的纵向加速度估计
  - `/slosh/ay_est` (Float32) — EMA 滤波后的横向加速度估计（v·ω）
  - `/slosh/alpha_est` (Float32) — EMA 滤波后的角加速度估计
- **用途**: 论文图表中展示加速度激励与晃动响应的因果关系

#### 2. launch 文件 Q_slosh 参数覆盖
- **文件**: `test_mpc.launch`, `test_mpc_sim.launch`
- **改动**: 新增 `<arg name="Q_slosh" default="0.0"/>`，在 YAML 加载后通过 `<param name="mpc/Q_slosh">` 覆盖
- **用法**:
  ```bash
  # 基线 (Q_slosh=0, 与原始行为完全一致)
  roslaunch scout_local_planner test_mpc.launch

  # 消融实验
  roslaunch scout_local_planner test_mpc.launch Q_slosh:=10.0
  ```

#### 3. 消融实验专用 launch
- **文件**: `launch/slosh_experiment.launch`（新建）
- **功能**: 统一入口，支持 `Q_slosh`, `sim`, `verbose` 三个参数
- **用法**:
  ```bash
  roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=10 sim:=true
  ```

#### 4. rosbag 录制脚本
- **文件**: `scripts/record_slosh_experiment.sh`（新建，已 chmod +x）
- **录制话题**: /slosh/*, /mpc/*, /cmd_vel, /odom, /scout/global_path, /local_path, /tf, /tf_static
- **输出**: `~/slosh_bags/slosh_Q{value}_{date}.bag`
- **用法**:
  ```bash
  ./scripts/record_slosh_experiment.sh 10           # Q=10
  ./scripts/record_slosh_experiment.sh 10 trial_3   # 自定义后缀
  ```

### 论文消融实验 SOP（标准操作流程）

```bash
# ① 启动底盘 + 雷达 + 定位 + 全局规划（见实物流程）

# ② 在不同终端分别运行：
# 终端 A：MPC 规划器（切换 Q_slosh）
roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=0    # 基线
# 或
roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=10   # 消融

# 终端 B：rosbag 录制
cd $(rospack find scout_local_planner)
./scripts/record_slosh_experiment.sh 0     # 与 Q_slosh 值对应
# 或
./scripts/record_slosh_experiment.sh 10

# ③ 跑相同路径 → Ctrl+C 停止录制 → 切换 Q_slosh → 重复

# ④ 数据分析
# bag 文件在 ~/slosh_bags/，可用 plotjuggler 或 Python 分析
```

### 全部调试话题汇总（7 个）

| 话题 | 类型 | 内容 |
|------|------|------|
| `/slosh/state` | Float32MultiArray | [η_x, η̇_x, η_y, η̇_y] |
| `/slosh/height` | Float32 | 液面波高 (m) |
| `/slosh/ax_est` | Float32 | 纵向加速度估计 (m/s²) |
| `/slosh/ay_est` | Float32 | 横向加速度估计 (m/s²) |
| `/slosh/alpha_est` | Float32 | 角加速度估计 (rad/s²) |
| `/mpc/solve_ms` | Float32 | QP 求解耗时 (ms) |
| `/mpc/status_val` | Int32 | 求解状态 (1=ok, 0=fail) |

### 修改文件汇总
| 文件 | 修改内容 |
|------|---------|
| `local_planner_ros.h` | 新增 3 个 accel est publisher 声明 |
| `local_planner_ros.cpp` | 注册 3 个 publisher; publishSloshDebug 发布 ax/ay/alpha |
| `test_mpc.launch` | 新增 Q_slosh arg + param 覆盖 |
| `test_mpc_sim.launch` | 同上 |
| `slosh_experiment.launch` | 新建：消融实验统一入口 |
| `record_slosh_experiment.sh` | 新建：rosbag 录制脚本 |
