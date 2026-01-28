# 启动前的注意工作
## 1.将底盘和工控机连接并建立 CAN 通信
    sudo modprobe gs_usb   # 若需要
    sudo ip link set can0 down 2>/dev/null || true
    sudo ip link set can0 up type can bitrate 500000
    candump can0          # 如需监听
    roslaunch scout_bringup scout_mini_robot_base.launch
### 监听状态的脚本
    roslaunch scout_bringup bms_status_monitor.launch     topic:=/BMS_status     period:=60.0
## 2.启动键盘控制
    roslaunch scout_bringup scout_teleop_keyboard.launch
## 3.启动激光雷达
    这个在sick_ws里面，有一个nanoscan3_bringup包：
    roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
## 4.建图
    这个也是在sick_ws里面，有一个nanoscan3_mapping包：
    roslaunch nanoscan3_mapping scout_nanoscan3_gmapping.launch fake_odom_tf:=false use_rviz:=true
## 5.定位
    新开一个终端，运行：
    roslaunch nanoscan3_localization scout_nanoscan3_amcl.launch use_rviz:=true
## 6.全局规划
    roslaunch scout_global_planner move_base_global.launch
    

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
  - 启动：`roslaunch scout_global_planner move_base_global.launch use_rviz:=true`
  - RViz 中设置 Fixed Frame 为 `map`，并将 “2D Nav Goal” 话题改为 `/scout/goal`
  - 观察 `/scout/global_path` 是否发布 `nav_msgs/Path`

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
       ros-noetic-teleop-twist-keyboard ros-noetic-map-server
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
