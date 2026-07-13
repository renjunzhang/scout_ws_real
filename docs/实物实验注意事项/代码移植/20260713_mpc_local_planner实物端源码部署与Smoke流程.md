# 20260713 mpc_local_planner 实物端源码部署与 Smoke 流程

> 适用机器：Scout 实物工控机 `/home/geist/scout_ws`
>
> 适用分支：`diag/lt-dwa-collision-tracking`，或后续包含同等改动的正式实验分支
>
> 背景：2026-07-12 实物端没有可用的 `mpc_local_planner` 源码和 isolated
> 编译产物，`rospack find mpc_local_planner` 失败；现场临时从 GitHub 初始化
> submodule 又因网络超时中断。因此当天没有进入 MPC 算法实物运行阶段。

## 0. 先看结论

实物端需要完成的顺序固定为：

```text
同步主仓库
  -> 精确初始化 control_box_rst 和 mpc_local_planner 两个子模块
  -> isolated 编译
  -> source 主工作区和 isolated overlay
  -> rospack / pluginlib 检查
  -> real/no-obstacle 配置检查
  -> shadow
  -> 20s 低速 actuated smoke
  -> 60s N=1 gate
  -> 参数冻结后 formal
```

必须区分：

```text
包不存在 / 插件加载失败：部署问题，不是 MPC 算法失败；
插件已加载但持续 NO_VALID_CMD：才进入规划器配置或算法排查；
机器人运动异常：先急停，再依据 raw/final cmd、tracking 和日志分析。
```

当前仓库已经有：

```text
src/scout_apps/control/baseline_local_planner_runner/
src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml
```

截至本文创建时，下面的实物专用 planner YAML **尚未入库**：

```text
src/scout_apps/control/spmpc_experiments/config/baselines/
mpc_local_planner_fixed_path_real_noobs.yaml
```

因此当前可以先完成源码、编译和插件可见性验证；在该 YAML 由开发端创建、仿真检查、
commit/push 并由实物端 pull 之前，**禁止直接运行 actuated**。也不要让实物脚本回退到
`mpc_local_planner_fixed_path_tuned_sim.yaml`。

---

## 1. 需要同步的代码范围

### 1.1 主仓库

实物端至少应拉到：

```text
branch: diag/lt-dwa-collision-tracking
superproject commit: a8c6358 或更新提交
```

主仓库包含 runner、实验脚本、costmap 和实验文档。不能只复制
`src/scout_apps/control/mpc_local_planner` 目录，因为 fixed-path 到点、命令限幅、
tracking diagnostics 和录包逻辑都在主仓库其他包中。

### 1.2 只需要两个 MPC 子模块

本次 `mpc_local_planner` baseline 需要：

```text
src/scout_apps/control/control_box_rst
  expected commit: 59b2028335c7a050b00a09841089bd0405707775

src/scout_apps/control/mpc_local_planner
  expected commit: 3885d64dd9484cc0199b4a05ae644cd3b64ff705
  patch: 适配 Noetic 下 mpc_local_planner isolated 构建
```

本次不需要初始化：

```text
src/mpc_planner
```

`src/mpc_planner` 是另一套通用 MPC 框架，不是这里运行的
`mpc_local_planner/MpcLocalPlannerROS` 插件。不要执行无选择的全量
`git submodule update --init --recursive`，否则会增加下载量，并可能让普通
`catkin_make` 遇到 non-catkin package。

---

## 2. 路线 A：实物端网络可用时

以下命令全部在实物工控机执行。

### 2.1 同步主仓库

先确认实物端没有需要保留的未提交修改：

```bash
cd /home/geist/scout_ws
git status --short --branch
```

工作区可安全更新后：

```bash
source /opt/ros/noetic/setup.bash
cd /home/geist/scout_ws

git fetch origin
git checkout diag/lt-dwa-collision-tracking
git pull --ff-only
git log -1 --oneline
```

不要使用 `git reset --hard` 清理实物端文件。若有现场修改，先保存或提交，再更新。

### 2.2 精确初始化两个子模块

```bash
cd /home/geist/scout_ws
git submodule sync --recursive

git submodule update --init \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner
```

检查：

```bash
git submodule status -- \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner

git -C src/scout_apps/control/control_box_rst rev-parse HEAD
git -C src/scout_apps/control/mpc_local_planner rev-parse HEAD
```

期望：

```text
59b2028335c7a050b00a09841089bd0405707775
3885d64dd9484cc0199b4a05ae644cd3b64ff705
```

`git submodule status` 前缀含义：

```text
空格：已初始化，且 commit 与主仓库一致；
-：未初始化；
+：当前 checkout 与主仓库记录不一致；
U：存在冲突。
```

只有空格前缀可以进入正式构建。

### 2.3 昨天失败目录仍残留时

如果提示目标目录已存在、不是空目录或不是有效 Git 工作树，不要覆盖。先备份失败目录：

```bash
cd /home/geist/scout_ws

git submodule deinit -f src/scout_apps/control/control_box_rst || true
git submodule deinit -f src/scout_apps/control/mpc_local_planner || true

mv src/scout_apps/control/control_box_rst \
  /home/geist/control_box_rst.failed_20260712
mv src/scout_apps/control/mpc_local_planner \
  /home/geist/mpc_local_planner.failed_20260712

git submodule update --init \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner
```

只有确实出现残留目录冲突时才执行本节；正常初始化不需要移动任何目录。

---

## 3. 路线 B：实物端 GitHub 网络仍不稳定时

推荐传输 Git bundle，而不是 `scp -r` 覆盖源码目录。bundle 会保留 commit，便于确认
实物端使用的是开发端已经验证的同一版本。

### 3.1 开发端生成 bundle

在开发机 `/home/a/scout_ws` 执行：

```bash
cd /home/a/scout_ws

git -C src/scout_apps/control/control_box_rst \
  bundle create /tmp/control_box_rst_59b2028.bundle --all

git -C src/scout_apps/control/mpc_local_planner \
  bundle create /tmp/mpc_local_planner_3885d64.bundle --all

git -C src/scout_apps/control/control_box_rst \
  bundle verify /tmp/control_box_rst_59b2028.bundle
git -C src/scout_apps/control/mpc_local_planner \
  bundle verify /tmp/mpc_local_planner_3885d64.bundle
```

将两个 bundle 通过 U 盘或局域网复制到实物端，例如：

```text
/home/geist/offline_bundles/control_box_rst_59b2028.bundle
/home/geist/offline_bundles/mpc_local_planner_3885d64.bundle
```

主仓库本身仍应通过 `git pull`、主仓库 bundle 或 `git format-patch/git am` 同步；
两个子模块 bundle 不能替代主仓库的 runner 和实验配置。

### 3.2 实物端从 bundle 初始化子模块

```bash
cd /home/geist/scout_ws

git submodule init \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner

git config 'submodule.src/scout_apps/control/control_box_rst.url' \
  /home/geist/offline_bundles/control_box_rst_59b2028.bundle

git config 'submodule.src/scout_apps/control/mpc_local_planner.url' \
  /home/geist/offline_bundles/mpc_local_planner_3885d64.bundle

git -c protocol.file.allow=always submodule update --init \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner
```

再执行 §2.2 的 commit 检查。后续网络恢复后，可恢复 `.gitmodules` 中记录的在线 URL：

```bash
git submodule sync -- \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner
```

---

## 4. 实物端 isolated 构建

### 4.1 系统依赖

首次构建先确认 IPOPT：

```bash
sudo apt-get update
sudo apt-get install -y coinor-libipopt-dev
```

如果机器已经安装，apt 会直接跳过。缺少该依赖时常见症状为：

```text
SolverIpopt cannot be selected since it is not installed properly
```

### 4.2 不要混入普通 catkin_make

`control_box_rst` 的 `build_type` 是 `cmake`，因此 MPC 必须走 isolated 构建。
不要执行：

```text
catkin_make --pkg mpc_local_planner
```

也不要为了 MPC 删除或重建现有 SPMPC 的 `build/`、`devel/`。MPC 使用独立目录：

```text
build_isolated_mpc
devel_isolated_mpc
install_isolated_mpc
```

### 4.3 构建命令

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

catkin_make_isolated --install --force-cmake \
  --only-pkg-with-deps mpc_local_planner \
  --install-space install_isolated_mpc \
  --devel devel_isolated_mpc \
  --build build_isolated_mpc
```

若昨天留下了不完整的 isolated 构建目录，先保留备份再重新构建：

```bash
cd /home/geist/scout_ws
mv build_isolated_mpc build_isolated_mpc.failed_20260712
mv devel_isolated_mpc devel_isolated_mpc.failed_20260712
mv install_isolated_mpc install_isolated_mpc.failed_20260712
```

目录不存在时 `mv` 会报错；只移动实际存在且确认是失败产物的目录。

构建完成后必须存在：

```bash
test -r /home/geist/scout_ws/install_isolated_mpc/setup.bash
find /home/geist/scout_ws/install_isolated_mpc \
  -name 'libmpc_local_planner.so' -print
```

---

## 5. 每个 MPC 终端的 source 顺序

新开终端，按以下顺序 source：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
cd /home/geist/scout_ws
```

最后 source `install_isolated_mpc`，确保 `mpc_local_planner` 插件优先可见，同时主工作区的
runner、实验脚本和配置仍然可见。

不要在已经 source 过旧 `install_isolated`、仿真 overlay 或其他工作区的终端继续测试。
环境混乱时直接开新终端，不要只反复 source。

检查环境：

```bash
echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | head -n 10
rospack find baseline_local_planner_runner
rospack find spmpc_experiments
rospack find mpc_local_planner
```

`rospack find mpc_local_planner` 应指向：

```text
/home/geist/scout_ws/install_isolated_mpc/share/mpc_local_planner
```

---

## 6. 插件加载前检查

### 6.1 pluginlib 可见性

```bash
rospack plugins --attrib=plugin nav_core | grep mpc_local_planner
```

期望先看到 plugin XML 路径，例如：

```text
mpc_local_planner /home/geist/scout_ws/install_isolated_mpc/share/mpc_local_planner/mpc_local_planner_plugin.xml
```

再检查 XML 中的 nav_core 类型：

```bash
MPC_PKG=$(rospack find mpc_local_planner)
grep -n 'mpc_local_planner/MpcLocalPlannerROS' \
  "$MPC_PKG/mpc_local_planner_plugin.xml"
```

期望看到：

```text
mpc_local_planner/MpcLocalPlannerROS
```

继续检查库依赖：

```bash
MPC_LIB=$(find /home/geist/scout_ws/install_isolated_mpc \
  -name 'libmpc_local_planner.so' -print -quit)
test -n "$MPC_LIB"
ldd "$MPC_LIB" | grep 'not found' || true
```

如果 `ldd` 出现 `not found`，不能进入 shadow。

### 6.2 实物配置 NO-GO 检查

```bash
REAL_MPC_YAML=/home/geist/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_real_noobs.yaml
REAL_NOOBS_COSTMAP=/home/geist/scout_ws/src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml

test -r "$REAL_MPC_YAML"
test -r "$REAL_NOOBS_COSTMAP"
```

任何一个失败都停止。不要使用 sim YAML 替代。

real YAML 至少必须满足：

```text
MpcLocalPlannerROS/robot/unicycle/max_vel_x=0.30
MpcLocalPlannerROS/robot/unicycle/max_vel_x_backwards=0.0
MpcLocalPlannerROS/robot/unicycle/max_vel_theta=1.20
MpcLocalPlannerROS/robot/unicycle/acc_lim_x=0.60
MpcLocalPlannerROS/robot/unicycle/dec_lim_x=0.60
MpcLocalPlannerROS/robot/unicycle/acc_lim_theta=1.20
MpcLocalPlannerROS/collision_avoidance/include_costmap_obstacles=false
MpcLocalPlannerROS/collision_avoidance/enable_dynamic_obstacles=false
```

注意：runner 对 MPC 只做最终命令安全限幅，不会像 TEB 一样覆盖 MPC 内部速度参数。
所以只设置 `MAX_V=0.30` 不够，YAML 内部 `max_vel_x` 也必须是 `0.30`。

### 6.3 实物基础链路

传感器、定位和底盘栈启动后检查：

```bash
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /scan_front
rosrun tf tf_echo map base_link
rostopic info /cmd_vel
```

进入 shadow 前必须满足：

```text
/odom、/map 和 TF 连续有效；
机器人静止时 odom 没有明显漂移或跳变；
/cmd_vel 的现有发布者已经明确；
急停可用；
场地为空旷 fixed-path 区域。
```

---

## 7. 第一次只跑 shadow

只有 §2～§6 全部通过，且 real YAML 已经入库后才能执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
cd /home/geist/scout_ws

DATE=20260713 \
METHOD=mpc_local_planner \
STAGE=shadow \
RUN_LABEL=MPC_local_planner_real_noobs_v1_shadow01 \
PLANNER_CONFIG=/home/geist/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_real_noobs.yaml \
COSTMAP_CONFIG=/home/geist/scout_ws/src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml \
MAX_V=0.30 \
MAX_W=1.20 \
MAX_ACC=0.60 \
MAX_ANGULAR_ACC=1.20 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=false \
START_STANDALONE_SLOSH=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_TOPIC_INFO=true \
RECORD_SEC=30 \
MAX_RECORD_SEC=30 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

shadow 通过标准：

```text
/baseline/mpc_local_planner/status 进入 TRACKING；
/baseline/mpc_local_planner/global_plan 是当前 fixed S-curve；
/baseline/mpc_local_planner/raw_cmd_vel 连续且全部有限；
/baseline/mpc_local_planner/tracking_error 连续发布；
/spmpc_shadow_cmd_vel 有合理命令；
无持续 SET_PLAN_FAILED / NO_VALID_CMD；
MPC 不向 /cmd_vel 发布，机器人保持静止。
```

建议另开终端观察：

```bash
rostopic echo /baseline/mpc_local_planner/status
rostopic echo /baseline/mpc_local_planner/raw_cmd_vel
rostopic echo /baseline/mpc_local_planner/tracking_error
rostopic info /cmd_vel
```

任何一项失败，都先保存 planner log 和 rosparam，不进入 actuated。

---

## 8. 20 秒低速 actuated smoke

shadow 通过后，操作者站在急停旁，只跑 20 秒：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
cd /home/geist/scout_ws

DATE=20260713 \
METHOD=mpc_local_planner \
STAGE=actuated \
RUN_LABEL=MPC_local_planner_real_noobs_v1_short01 \
PLANNER_CONFIG=/home/geist/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_real_noobs.yaml \
COSTMAP_CONFIG=/home/geist/scout_ws/src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml \
MAX_V=0.30 \
MAX_W=1.20 \
MAX_ACC=0.60 \
MAX_ANGULAR_ACC=1.20 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=true \
START_STANDALONE_SLOSH=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_TOPIC_INFO=true \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=20 \
MAX_RECORD_SEC=25 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

20 秒通过标准：

```text
机器人持续沿 fixed path 正向推进；
20 秒内 path progress 至少增加 0.15；
tracking p95 < 0.30m；
无左右高频摆头、原地高速旋转、倒车或切弯失控；
raw cmd 不长期贴 MAX_V/MAX_W；
command_intervention linear/angular limited fraction 建议 <1%；
无持续 NO_VALID_CMD；
无人工接管或 E-stop。
```

出现下列任意情况立即急停：

```text
机器人向路径反方向运动；
角速度持续贴 1.20rad/s；
tracking error 持续增大；
定位跳变；
控制命令在正负角速度之间高频切换；
底盘不响应 zero command。
```

---

## 9. 60 秒 N=1 与 formal

只有 20 秒 smoke 通过后才允许 60 秒 N=1。最低 gate：

```text
GOAL_REACHED；
goal time <=57s；
tracking p95 <=0.30m；
无人工接管、定位跳变或持续 NO_VALID_CMD；
无长期后级速度 clamp；
RGB、odom、TF、fixed path、raw/final cmd、tracking diagnostics 完整；
rosbag 无 buffer exceeded。
```

调参一次只改一个变量。第一优先级是确认 `MAX_V=0.30` 的稳定性，不要同时修改
tracking 权重、lookahead、速度和角速度上限。

N=1 通过后：

1. 将最终 YAML 另存为 frozen/final 文件；
2. 记录主仓库 commit 和两个子模块 commit；
3. push 后由实物端重新 pull；
4. formal N=3 期间禁止继续修改参数；
5. 与同日 governor-off `B_ours` bridge 交错运行。

---

## 10. 常见错误与判定

### 10.1 `package 'mpc_local_planner' not found`

原因优先级：

```text
子模块未初始化；
isolated 构建未完成；
当前终端没有 source install_isolated_mpc；
source 顺序错误或终端被其他 overlay 污染。
```

检查：

```bash
git submodule status -- src/scout_apps/control/mpc_local_planner
test -r /home/geist/scout_ws/install_isolated_mpc/setup.bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
rospack find mpc_local_planner
```

### 10.2 `Could not find library corresponding to plugin`

说明 package.xml/plugin XML 可见，但共享库不可加载。检查：

```bash
find /home/geist/scout_ws/install_isolated_mpc \
  -name 'libmpc_local_planner.so' -print
ldd /home/geist/scout_ws/install_isolated_mpc/lib/libmpc_local_planner.so \
  | grep 'not found' || true
```

如果实际库路径不同，以 `find` 输出为准。

### 10.3 普通 `catkin_make` 报 non-homogeneous workspace

这是因为 `control_box_rst` 是纯 CMake 包。SPMPC 主线仍使用普通 `catkin_make`；
MPC 单独使用本文 `build_isolated_mpc/devel_isolated_mpc/install_isolated_mpc`。
不要为了处理该报错破坏现有 SPMPC `devel/`。

### 10.4 插件加载成功但持续 `NO_VALID_CMD`

此时才开始检查 MPC 配置：

```text
global plan frame 与 map/base_link TF；
odom topic；
initial pose 与 fixed path 起点；
内部 max_vel/acc limit；
lookahead 和 grid；
solver 日志；
是否误用了 sim YAML 或 obstacle costmap。
```

先在 shadow 中解决，不能用 actuated 反复试错。

### 10.5 设置了 `MAX_V=0.30`，raw cmd 仍按高速规划

原因是 MPC 内部上限仍来自 planner YAML。runner 的 `MAX_V` 只是最终安全限幅。
检查：

```bash
rosparam get /baseline_local_planner_runner/MpcLocalPlannerROS/robot/unicycle/max_vel_x
rosparam get /baseline_local_planner_runner/max_cmd_vel_x
```

两者都应为 `0.30`。如果内部仍是 `0.8`，说明误用了 tuned-sim YAML，立即停止。

---

## 11. 每次实验必须保存的部署证据

正式实验前，把以下输出保存到当天记录：

```bash
cd /home/geist/scout_ws
git rev-parse HEAD
git status --short --branch
git submodule status -- \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner

rospack find mpc_local_planner
rospack plugins --attrib=plugin nav_core | grep mpc_local_planner
rosparam get /baseline_local_planner_runner/MpcLocalPlannerROS
rostopic info /cmd_vel
```

实验记录至少写明：

```text
主仓库 commit；
control_box_rst commit；
mpc_local_planner commit；
planner YAML 路径和文件 hash；
costmap YAML 路径；
shadow / actuated 阶段；
MAX_V/MAX_W/MAX_ACC/MAX_ANGULAR_ACC；
是否人工接管；
是否出现 buffer exceeded、NO_VALID_CMD 或 plugin load error。
```

---

## 12. 关联文档

```text
docs/实物实验注意事项/代码移植/20260602_实物端代码拉取与子模块注意事项.md
docs/实物实验注意事项/对比试验/实物对比试验分析/20260712_mpc-local-planner实物接入问题记录.md
docs/实物实验注意事项/对比试验/实物对比实验/0710_TEB实物fixed_path正式化方案.md
docs/实物实验注意事项/对比试验/仿真对比试验/仿真对比试验启动指南.md
```

一句话原则：

```text
先把开发端已验证的两个子模块精确同步并 isolated 构建，通过插件检查和 shadow；
实物 real/no-obstacle YAML 未入库、未检查之前，绝不使用 sim 配置直接 actuated。
```
