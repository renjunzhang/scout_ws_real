# S-MPCC 正式实物实验启动与录制命令

> 日期：2026-07-17
>
> 适用范围：[0717_S-MPCC正式实物实验矩阵_先40后88.md](./0717_S-MPCC正式实物实验矩阵_先40后88.md) 的 S1、S2A、S2B 正式 trial。
>
> 本文件只规定现场命令。实验次数、随机顺序、统计口径、失败规则和 recorder 设置以矩阵文件为准。

---

## 1. 正式入口与当前限制

正式 trial 必须 replay 已冻结的 H1/L1 JSON，禁止根据每次实际起点重新生成路径。

当前下面两个脚本仍会调用 `template_fixed_path_generator.py`：

```text
run_spmpc_real_fixed_path_trial.sh
run_continuous_real.sh
```

所以它们只用于 smoke，不用于本轮 40/88 次正式采集。正式采集使用本文的多终端流程：

```text
实物基础栈
  + standalone slosh monitor
  + fixed_global_path_runner.py --mode replay
  + spmpc_experiment.launch
  + record_spmpc_full_rgb_bag.sh
```

在 H1 JSON、Smooth-match `v_ref` 或 C2 配置仍为占位值时，对应正式实验为 NO-GO。

---

## 2. 每个终端的公共环境

每个新终端都先执行：

```bash
export SCOUT_WS="${SCOUT_WS:-${HOME}/scout_ws}"
source /opt/ros/noetic/setup.bash
source "${SCOUT_WS}/devel/setup.bash"
cd "${SCOUT_WS}"
```

确认当前代码和构建：

```bash
git status --short
git rev-parse HEAD
rospack find spmpc_local_planner
rospack find scout_local_planner
```

正式 revision 必须与冻结记录一致。工作区存在未归档修改时，不开始正式采集。

---

## 3. 实物基础系统

若实物基础栈尚未启动，分别在独立终端启动：

```bash
roslaunch scout_bringup scout_mini_robot_base.launch
```

```bash
roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
```

```bash
roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
```

```bash
roslaunch scout_bringup scout_imu_with_tf.launch
```

```bash
roslaunch realsense2_camera rs_camera.launch
```

基础检查：

```bash
timeout 10s rostopic hz /odom
timeout 10s rostopic hz /imu/data
timeout 10s rostopic hz /camera/color/image_raw
timeout 5s rosrun tf tf_echo map base_link
rostopic info /cmd_vel
df -h "${HOME}/slosh_bags"
```

开始 planner 前，`/cmd_vel` 不得存在旧 planner publisher。RealSense 必须已经按冻结值设置为 `1920x1080@30 Hz` 和手动曝光/增益/白平衡。

---

## 4. 为本次 trial 生成公共运行环境

先按随机表填写以下字段。示例是 S1/E2、H1、C1、B0、block 01：

```bash
export EXP_DATE=20260717
export STAGE=S1
export GROUP=E2
export PATH_ID=H1
export CONTAINER=C1
export METHOD=B0
export BLOCK=01
export REPEAT=01
```

方法名与代码映射：

| `METHOD` | `VARIANT` | `V_REF` |
| --- | --- | ---: |
| `B0` | `B0` | `0.20` |
| `Bsmooth` | `B_smooth` | `0.20` |
| `Bslosh` | `B_slosh` | `0.20` |
| `SmoothMatch` | `B_smooth` | 独立 pilot 冻结值 |

执行映射：

```bash
case "${METHOD}" in
  B0)          export VARIANT=B0;       export V_REF=0.20 ;;
  Bsmooth)     export VARIANT=B_smooth; export V_REF=0.20 ;;
  Bslosh)      export VARIANT=B_slosh;  export V_REF=0.20 ;;
  SmoothMatch)
    : "${SMOOTH_MATCH_V_REF:?先填写独立 pilot 冻结的 SMOOTH_MATCH_V_REF}"
    export VARIANT=B_smooth
    export V_REF="${SMOOTH_MATCH_V_REF}"
    ;;
  *) echo "未知 METHOD=${METHOD}" >&2; exit 2 ;;
esac
```

路径和容器映射：

```bash
FREEZE_ROOT="${SCOUT_WS}/docs/实物实验注意事项/对比试验/实物对比实验/freeze"

case "${PATH_ID}" in
  H1) export PATH_JSON="${FREEZE_ROOT}/paths/H1_P2_s_curve.json" ;;
  L1) export PATH_JSON="${FREEZE_ROOT}/paths/L1_gentle.json" ;;
  *) echo "未知 PATH_ID=${PATH_ID}" >&2; exit 2 ;;
esac

case "${CONTAINER}" in
  C1)
    export CONTAINER_CONFIG=tube_default
    export CONTAINER_RADIUS=0.0185
    export LIQUID_HEIGHT=0.058
    export DAMPING_RATIO=0.05
    ;;
  C2)
    : "${C2_CONTAINER_CONFIG:?填写冻结的 C2 配置名，不带 .yaml}"
    : "${C2_CONTAINER_RADIUS:?填写冻结的 C2 内半径}"
    : "${C2_LIQUID_HEIGHT:?填写冻结的 C2 液深}"
    export CONTAINER_CONFIG="${C2_CONTAINER_CONFIG}"
    export CONTAINER_RADIUS="${C2_CONTAINER_RADIUS}"
    export LIQUID_HEIGHT="${C2_LIQUID_HEIGHT}"
    export DAMPING_RATIO=0.05
    ;;
  *) echo "未知 CONTAINER=${CONTAINER}" >&2; exit 2 ;;
esac
```

生成 run 名称和目录：

```bash
export RUN_LABEL="${STAGE}_${GROUP}_${PATH_ID}_${CONTAINER}_${METHOD}_b${BLOCK}_r${REPEAT}"
export OUT_DIR="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_formal/${STAGE}/${GROUP}/${PATH_ID}_${CONTAINER}/${METHOD}"
export NAME="${RUN_LABEL}"
export CONTAINER_YAML="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/containers/${CONTAINER_CONFIG}.yaml"

test -s "${PATH_JSON}"
test -s "${CONTAINER_YAML}"
mkdir -p "${OUT_DIR}"
test ! -e "${OUT_DIR}/${NAME}.bag"

echo "RUN_LABEL=${RUN_LABEL}"
echo "PATH_JSON=${PATH_JSON}"
echo "CONTAINER_CONFIG=${CONTAINER_CONFIG}"
echo "VARIANT=${VARIANT} V_REF=${V_REF}"
echo "OUT_DIR=${OUT_DIR}"
```

把本次公共变量保存到临时文件，随后每个 trial 终端都 source 它：

```bash
export RUN_ENV="/tmp/${RUN_LABEL}.env"
declare -px SCOUT_WS EXP_DATE STAGE GROUP PATH_ID CONTAINER METHOD BLOCK REPEAT \
  VARIANT V_REF PATH_JSON CONTAINER_CONFIG CONTAINER_RADIUS LIQUID_HEIGHT \
  DAMPING_RATIO RUN_LABEL OUT_DIR NAME CONTAINER_YAML RUN_ENV > "${RUN_ENV}"
```

同时保存路径、配置和 revision 证据：

```bash
sha256sum "${PATH_JSON}" > "${OUT_DIR}/${NAME}_path_sha256.txt"
sha256sum \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/planner/common.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/planner/variants.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/platforms/scout_mini.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/experiments/fixed_path.yaml" \
  "${CONTAINER_YAML}" \
  > "${OUT_DIR}/${NAME}_config_sha256.txt"

git rev-parse HEAD > "${OUT_DIR}/${NAME}_git_revision.txt"
git status --short > "${OUT_DIR}/${NAME}_git_status.txt"
cp "${RUN_ENV}" "${OUT_DIR}/${NAME}_formal_run.env"
```

---

## 5. 终端 A：standalone slosh monitor

每次切换容器时重启 monitor。同一容器连续 block 可以保持进程运行，但每个 trial 运动前必须 reset。

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

roslaunch slosh_models slosh_monitor.launch \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel \
  output_namespace:=/slosh \
  container_radius:="${CONTAINER_RADIUS}" \
  liquid_height:="${LIQUID_HEIGHT}" \
  damping_ratio:="${DAMPING_RATIO}" \
  use_parabola_term:=false \
  model_dt:=0.02 \
  accel_filter_alpha:=0.3 \
  min_dt:=0.001 \
  max_dt:=0.1
```

另一个终端确认并 reset：

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
timeout 5s rostopic echo -n 1 /slosh/height
rosservice call /slosh/reset
```

`/slosh/*` 只用于统一的模型监测和离线评价，不作为 B0/B_smooth 的控制输入。

---

## 6. 终端 B：装载冻结路径并停在人工开始门

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

rosrun scout_local_planner fixed_global_path_runner.py \
  --mode replay \
  --path-file "${PATH_JSON}" \
  --output-topic /scout/global_path_fixed \
  --base-frame base_link \
  --manual-start \
  --start-pos-tol 0.05 \
  --start-yaw-tol 0.10 \
  --start-hold-sec 0.5 \
  --publish-rate 2.0 \
  --publish-count 0
```

看到下面提示后先不要按 Enter：

```text
Press Enter to continue fixed-path replay...
```

机器人必须先回到冻结路径起点标记，液体达到静稳门槛，planner 和 recorder 也必须已经启动。

---

## 7. 终端 C：启动正式 recorder

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"
cd "${SCOUT_WS}"

DATE="${EXP_DATE}" \
VARIANT="${VARIANT}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${NAME}" \
OUT_DIR="${OUT_DIR}" \
RECORD_SEC=90 \
RECORD_RGB=true \
RECORD_CAMERA_COMPRESSED=false \
RECORD_DEPTH=false \
RECORD_SCAN=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_ONLINE_LIQUID=false \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=true \
ROSBAG_BUFFER_SIZE_MB=4096 \
SOLVER_BACKEND=continuous_mpcc_acados \
V_REF="${V_REF}" \
W_SLOSH=-1.0 \
SLOSH_HEIGHT_MAX=-1.0 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
CONTROL_FREQUENCY=30 \
OPERATOR_NOTE="formal ${STAGE}/${GROUP} ${PATH_ID}/${CONTAINER} block ${BLOCK} ${METHOD}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

等待 recorder 打印输出路径且没有立即退出，再启动 planner。90 s 是防遗忘的最长录制保护；trial 提前结束时仍需保证到达/停止后继续录制 5 s，再用 Ctrl+C 正常关闭 recorder。

---

## 8. 终端 D：启动本次 planner

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

roslaunch spmpc_local_planner spmpc_experiment.launch \
  planner_variant:="${VARIANT}" \
  experiment_mode:=fixed_path \
  experiment_config:=fixed_path \
  platform_config:=scout_mini \
  container_config:="${CONTAINER_CONFIG}" \
  reference_path_topic:=/scout/global_path_fixed \
  reference_target_frame:=map \
  costmap_topic:=/map \
  cmd_vel_topic:=/cmd_vel \
  publish_cmd_vel:=true \
  solver_backend:=continuous_mpcc_acados \
  v_ref:="${V_REF}" \
  w_slosh:=-1.0 \
  slosh_height_max:=-1.0 \
  alpha_max:=1.2 \
  shared_linear_accel_limit_enable:=true \
  shared_linear_accel_max:=0.6 \
  shared_angular_limit_enable:=true \
  shared_angular_rate_max:=1.2 \
  shared_angular_accel_max:=1.2 \
  delay_phase_mode:=fixed_closed_loop \
  delay_phase_linear_delay_sec:=0.15 \
  delay_phase_angular_delay_sec:=0.22
```

路径尚未发布时 planner 应保持等待，不得自行运动。在第五个观察终端检查并补存 planner 参数：

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

rostopic echo -n 1 /spmpc/solver_backend
rostopic echo -n 1 /spmpc/controller_variant
rostopic echo -n 1 /spmpc/debug/effective_config \
  > "${OUT_DIR}/${NAME}_effective_config_before_start.txt"
rosparam get /spmpc_local_planner \
  > "${OUT_DIR}/${NAME}_planner_rosparam_before_start.yaml"
rostopic info /cmd_vel
```

`/cmd_vel` 必须只有本次 S-MPCC planner 一个有效控制 publisher。

---

## 9. 正式开始运动

planner 和 recorder 均正常后：

1. 再次调用 `rosservice call /slosh/reset`；
2. 确认液面连续至少 2 s 低于冻结静止阈值；
3. 确认安全员、急停和走廊；
4. 回到终端 B，按 Enter；
5. 等待起点门控输出 `Start pose aligned`，随后路径开始发布，车辆开始运动。

运动期间不修改参数、不切换节点、不手动重发路径。发生 safety/solver/tracking failure 时安全停止，但保留本次 bag 作为方法失败。

---

## 10. 到达与停止顺序

到达、方法失败或安全停止后：

1. 保持 recorder 继续运行 5 s；
2. Ctrl+C 停止终端 C recorder，使 bag 正常闭合；
3. Ctrl+C 停止终端 D planner；
4. Ctrl+C 停止终端 B path replay；
5. 需要时发布一次零速度；
6. 同一容器继续下一 trial 时保留 monitor，但下一次必须再次 `/slosh/reset`。

零速度命令：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

---

## 11. 每次 run 的即时验收

```bash
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"
export BAG="${OUT_DIR}/${NAME}.bag"

test -s "${BAG}"
rosbag info "${BAG}" | tee "${OUT_DIR}/${NAME}_manual_bag_info.txt"
```

核对核心话题：

```bash
for topic in \
  /cmd_vel \
  /odom \
  /tf \
  /scout/global_path_fixed \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /spmpc/status \
  /spmpc/solver_backend \
  /spmpc/controller_variant \
  /spmpc/debug/effective_config \
  /spmpc/debug/predicted_horizon \
  /spmpc/debug/pre_solve_snapshot; do
  grep -Fxq "${topic}" "${OUT_DIR}/${NAME}_recorded_topics.txt" \
    || echo "MISSING_REQUIRED_TOPIC ${topic}"
done
```

检查 replay 消息：

```bash
rostopic echo -b "${BAG}" -n 1 /spmpc/debug/predicted_horizon \
  | rg 'valid:|backend:|variant:|horizon_steps:'

rostopic echo -b "${BAG}" -n 1 /spmpc/debug/pre_solve_snapshot \
  | rg 'valid:|backend:|variant:|primal_guess_only:|horizon_steps:|state_width:|control_width:|parameter_width:'
```

预期：

```text
valid=true
backend=continuous_mpcc_acados
horizon_steps=60
预测状态/控制=61/60
state_width=10
control_width=3
B0/B_smooth parameter_width=23
B_slosh parameter_width=32
primal_guess_only=true
```

若缺少 RGB、odom、TF、冻结路径或两个 replay 话题，本次按第 12 节判断为采集故障并保留原始文件；不得删除后静默重跑。若 solver/tracking/safety 本身失败，则作为方法失败进入 success-rate 分母。

---

## 12. 下一次 trial

1. 按矩阵随机顺序修改 `METHOD` 和 `BLOCK`；
2. 重新执行第 4 节，生成新的 `RUN_LABEL` 和 `RUN_ENV`；
3. 每个 trial 重新启动 planner；
4. 路径仍从同一冻结 JSON replay；
5. monitor reset，液体重新静稳；
6. recorder 产生新的独立 bag；
7. 不根据已看到的 RGB 排名修改后续顺序、权重或 `v_ref`。
