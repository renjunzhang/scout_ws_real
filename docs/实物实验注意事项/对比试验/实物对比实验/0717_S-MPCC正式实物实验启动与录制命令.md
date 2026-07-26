# S-MPCC 正式实物实验启动与录制命令

> 协议 ID：`SMPCC-REAL-40-88-v1.0`
>
> 版本日期：2026-07-25
>
> 当前状态：**参数/工具准备可执行；正式 S1/S2 采集 NO-GO。** 第 13 节矩阵冻结清单和 K6-FID-v1.0 第 12 节全部通过，并生成只读 `FREEZE_ID`/manifest 后，才允许开始第一条正式 trial。
>
> 适用范围：[0717_S-MPCC正式实物实验矩阵_先40后88.md](./0717_S-MPCC正式实物实验矩阵_先40后88.md) 的参数冻结 PF pilot，以及 S1、S2A、S2B 正式 trial。
>
> 本文件只规定现场命令。实验次数、随机顺序、统计口径、失败规则和 recorder 设置以矩阵文件为准。

---

## 1. 当前脚本入口、执行顺序与边界

除一次不计入矩阵的 H0 path-freeze smoke 外，参数 pilot 和正式 trial 都必须 replay 已冻结的 H0/H1/L1 JSON，禁止根据每次实际起点重新生成路径。

当前入口状态：

```text
run_spmpc_real_fixed_path_trial.sh：已支持 generate/replay，C1 参数 PF pilot 的主入口
run_continuous_real.sh：仍按当前起点生成路径，只用于 smoke
```

`run_spmpc_real_fixed_path_trial.sh` 是“单次 trial 一键脚本”，不会重复启动底盘、定位、传感器或 standalone slosh monitor；运行它之前先按第 3 节启动基础栈一键脚本。trial 脚本内部实际调用：

```text
spmpc_fixed_path.launch
record_spmpc_full_rgb_bag.sh
template_fixed_path_generator.py 或 fixed_global_path_runner.py
```

当前 `spmpc_fixed_path.launch` 固定使用 `scout_mini + tube_default + fixed_path`，一键脚本没有 `container_config` 参数。因此：

- `C1/tube_default` 的 PF 权重筛选使用一键脚本；
- `C2`、正式 40/88、需要人工 Enter 门或需要单独核查 recorder/planner 时，使用本文第 5 节之后的多终端流程；
- 不得在 C2 run 中仅修改标签却继续调用当前一键脚本，否则实际容器模型仍是 `tube_default`。

一键 replay 的真实执行顺序为：

```text
检查冻结 JSON
→ 启动 fixed_global_path_runner.py
→ 自动等待起点门控（无 --manual-start、无需按 Enter，最长默认 120 s）
→ 路径开始持续发布
→ 启动 recorder
→ 等待 recorder 默认 8 s
→ 启动 spmpc_fixed_path.launch
→ recorder 超时、planner 退出或 Ctrl-C
→ 自动停 planner、发零速度、停 recorder 和 path source
```

这意味着一键 replay 必须在执行命令前就完成“遥控回起点、航向对齐、液体静稳和安全确认”。如果尚未进入门控范围，脚本只等待路径，不会提前启动 recorder 浪费录包时间。

正式 40/88 次仍使用多终端流程，保留路径 Enter 门、recorder、planner 和开始时刻的独立人工检查：

```text
实物基础栈
  + standalone slosh monitor
  + fixed_global_path_runner.py --mode replay
  + spmpc_experiment.launch
  + record_spmpc_full_rgb_bag.sh
```

在 H1/L1 JSON、最终 `w_slosh`、Smooth-match `v_ref`、C2 配置、执行 freeze manifest、actual/zero replay 或 K6-FID-v1.0 准入仍未完成时，全部正式 S1/S2 实验为 NO-GO。当前仓库尚缺 K6 唯一脚本、manifest、独立同步标定和 smoke 报告；旧 0705/0706 的 `w_slosh=5` 只作为历史 pilot 先验，不能直接替代下面的参数冻结步骤。

本轮只允许当前已实现的动力学、状态传播和 slosh cost。rotation-consistent dynamics、相位能量/有符号功率等属于后续独立 release，不得在 40/64/88 阶段之间混入。

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

现场优先使用已有的一键基础栈脚本，不逐个手动启动节点：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

SCOUT_WS=/home/geist/scout_ws \
REALSENSE_COLOR_WIDTH=1920 \
REALSENSE_COLOR_HEIGHT=1080 \
REALSENSE_COLOR_FPS=30 \
REALSENSE_ENABLE_DEPTH=false \
REALSENSE_ENABLE_INFRA=false \
WAIT_FOR_ODOM=true \
WAIT_FOR_LOCALIZATION_MAP=false \
bash src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

该脚本会统一完成：

```text
CAN can0 500000
→ Scout Mini 底盘和 /odom
→ nanoscan3 前雷达和 /scan_front
→ Cartographer localization
→ Scout IMU 和 /imu/data
→ RealSense 1920x1080@30 Hz（depth/infra 关闭）
→ camera_info 与 camera/IMU hz 检查
```

日志默认写入 `/tmp/launch_real_sensors_stack_<时间戳>/`。该终端必须保持运行；按 `Ctrl-C` 会停止它启动的全部基础节点。

无论 PF pilot 是否录 RGB，基础栈都必须在最开始统一启动 RealSense，并在本批实验期间保持运行，不得在 pilot 与正式 trial 之间停止后重新启动。这样可以避免后续再次启动带来的节点状态、相机时间基准、曝光稳定过程和现场操作差异。

PF pilot 的“无 RGB”只表示 `run_spmpc_real_fixed_path_trial.sh` 强制不订阅、不写入 raw RGB、compressed RGB、depth 或 online liquid bag 数据，不表示关闭相机节点，因此不会产生 RGB rosbag 存储占用。正式 trial 开始前只需再次核对冻结的分辨率、帧率、曝光、增益和白平衡，不重新启动 RealSense。

基础栈打印 `Stack is running` 后，另开终端做实验前补充检查：

```bash
timeout 10s rostopic hz /odom
timeout 10s rostopic hz /scan_front
timeout 10s rostopic hz /imu/data
timeout 5s rosrun tf tf_echo map base_link
rostopic info /cmd_vel
df -h "${HOME}/slosh_bags"
```

正式 trial 再核对相机冻结状态：

```bash
timeout 10s rostopic hz /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
```

开始 planner 前，`/cmd_vel` 不得存在旧 planner publisher。若基础栈脚本退出，不得继续启动 path/recorder/planner。

只有基础栈一键脚本本身失败时，才按其 `/tmp/launch_real_sensors_stack_*` 日志定位具体的 base、LiDAR、localization、IMU 或 RealSense 节点，不把逐节点启动作为日常 SOP。

---

## 4. 正式前参数冻结 pilot

### 4.1 去现场前冻结候选和门槛

默认参数候选：

```text
W1: w_slosh=1
W2: w_slosh=2
W5: w_slosh=5
```

现场协议 v1.0 固定只使用 W1/W2/W5，W10 和任意临时候选均不进入本轮 15 次矩阵。若以后确需增加候选，必须在产生任何本轮 pilot 数据前升级协议版本并重新生成完整矩阵，不能现场追加。

除 `w_slosh` 外统一保持：

```text
v_ref=0.20
v_max=0.8
omega_max=1.2
a_max=0.6
alpha_max=1.2
horizon=60
rho_eta_dot=0.3
delay=fixed_closed_loop 0.15/0.22
terminal/gate/limiter/fallback 相同
C1、液深、安装和冻结路径相同
RECORD_RGB=false，不启动在线视觉液面节点
```

在开始候选运行前，必须写明并冻结：tracking p95、completion time、solve-time/inter-arrival、fallback、执行层干预和数据完整性门槛。B0/B_smooth 关闭控制器液体状态，不能把其缺失或零的在线 `H_modal` 用作噪声基线；基础重复性/噪声必须来自全部 15 次中统一启动、统一 reset 的 standalone `/slosh/height`。若在产生任何 P3 数据前另行预注册独立 repeatability smoke，其噪声容差只能按 `max(15次 standalone 重复性, 独立 smoke 重复性)` 补充，不能选择较宽松或较有利的一项。W1/W2/W5 的候选排序才使用各自实际在线 `/spmpc/slosh_height` 和预测 horizon，并按冻结规则计算 `delta_model`。

recorder whitelist 已补入 `/spmpc/debug/warm_start` 和 `/spmpc/debug/warm_start_status`，但计入固定 15 次 P3 之前仍必须用独立 bag smoke 确认两个话题实际落包且 `used_fallback` 可读。smoke 通过前只允许做路径和安全调试，不能开始将用于最终选权的 15 次 PF；否则无法执行本节的 fallback 准入。

固定 15 次 P3 还必须在一键命令之外另开终端启动第 6 节的 standalone slosh monitor，保持 `output_namespace:=/slosh`，并在每个 trial 前调用 `/slosh/reset`。一键脚本中的 `RECORD_STANDALONE_SLOSH=true` 只负责订阅，绝不会代启动或 reset monitor。只有 H0 路径生成、路径 replay 和纯安全 smoke 可以省略 monitor；一旦计入 15 次选权 pilot，缺少 `/slosh/state` 或 `/slosh/height` 即为数据完整性失败。

### 4.2 一键启动单次参数 pilot

#### 4.2.1 当前脚本的 pilot 默认值

| 字段 | `PILOT_MODE=true` 的实际行为 |
| --- | --- |
| `PATH_SOURCE_MODE` | 默认 `replay`；首次冻结 H0 时显式改为 `generate` |
| `PATH_FILE` | `${HOME}/fixed_paths/real/${DATE}_spmpc_parameter_pilot/H0_weight_pilot.json` |
| `RUN_OUT_DIR` | 未指定时为 `${HOME}/slosh_bags/real/${DATE}_spmpc_parameter_pilot/${PILOT_METHOD}` |
| generate 几何 | goal `(-5.424,-4.736,0)`；`s_curve`、spacing `0.05`、amplitude ratio `0.18`、min/max `0.25/1.20`、left、smooth `3` |
| 起点门控 | `START_POS_TOL=0.08`、`START_YAW_TOL=0.15`、`START_HOLD_SEC=0.5`、最长等待 `120 s` |
| 路径发布 | `2 Hz`，持续发布至清理；replay 不发送 `/scout/goal` |
| topics/frames | reference `/scout/global_path_fixed`、goal `/scout/goal`、cmd `/cmd_vel`、costmap `/map`、target frame `map`、base frame `base_link` |
| planner | `spmpc_fixed_path.launch`，默认容器 `tube_default`，仅用于 C1 |
| solver/control | `continuous_mpcc_acados`、`V_REF=0.20`、`fixed_closed_loop 0.15/0.22` |
| 公共限制 | `alpha_max=1.2`、线加速度 `0.6`、角速度/角加速度 `1.2` |
| RGB/视觉 | `PILOT_RECORD_RGB=false` 时强制 raw RGB、compressed RGB、depth、online liquid 全部关闭 |
| 其他录制 | scan 和 `/spmpc/*` 默认记录；P3 固定 15 次必须另行启动/reset monitor，`RECORD_STANDALONE_SLOSH=true` 只表示订阅已有 `/slosh/*` |
| timing | path source 启动等待 `2 s`、recorder 启动等待 `8 s`、planner 启动等待 `2 s` |
| recorder | `RECORD_SEC=60`、`MAX_RECORD_SEC=60`、`RECORD_TOPIC_INFO=false` |
| 退出 | 当 `CMD_TOPIC=/cmd_vel` 时自动发布一次零速度；随后停止 recorder 和 path source |

脚本会验证数值、ROS master、冻结路径是否存在以及 recorder 是否可读。pilot generate 遇到已有 `PATH_FILE` 时默认失败，不会静默覆盖。

脚本代码仍保留旧 `MATRIX_PRESET` 和隐式 `B_ours` 兼容入口，因此本协议的每条 PF 命令都必须先执行 `unset MATRIX_PRESET`，并显式设置非空 `PILOT_METHOD=B0|Bsmooth|W1|W2|W5`。旧 `0706_bsmooth_bours` preset、W10、`W3.5` 等临时候选即使代码能够运行，也不得进入本轮冻结数据。

一键脚本可直接覆盖的是 `V_REF`、`W_SLOSH`、delay、`ALPHA_MAX` 和 shared execution limits；`v_max`、horizon、`rho_eta_dot`、terminal/gate/fallback 仍来自当前加载的 YAML，不是该脚本的环境变量。若修改这些 YAML，必须重新归档配置并重新冻结，不能只改文档中的命令标签。

#### 4.2.2 首次生成并冻结 H0

历史方案确认现场机使用：

```text
工作空间：/home/geist/scout_ws
历史 bag：/home/geist/slosh_bags/real/<DATE>_fixed_path_compare/
历史路径：/home/geist/fixed_paths/real/<DATE>_fixed_path_compare/fixed_s_curve_compare.json
```

历史路径文件由旧一键流程逐 run 重新生成并覆盖，不能直接当作冻结 H0。当前开发机上的 0706 同名镜像文件终点为 `(7, 2)`，也不符合本实验的 `(-5.424, -4.736)` 终点，因此下一次现场先做一次不计入 15 次矩阵的 path-freeze smoke：

`generate` 模式没有起点门控。执行下面命令前必须把机器人精确放在地面起点标记并对齐航向，随后保持静止；生成器会以发送 goal 时的当前 map 位姿和 heading 生成路径。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
unset MATRIX_PRESET

DATE=20260718 \
PILOT_MODE=true \
PILOT_METHOD=B0 \
PATH_SOURCE_MODE=generate \
PATH_FILE=/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json \
RUN_LABEL=PF_PATH_FREEZE_H0_B0_smoke01 \
RUN_OUT_DIR=/home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/PATH/H0_C1/B0 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
OPERATOR_NOTE="C1 H0 path-freeze smoke; no RGB" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

在该 run 成功到点、路径安全且 tracking 合格后，冻结文件为：

```text
/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json
```

保存冻结证据：

```bash
sha256sum /home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json \
  | tee /home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/PATH/H0_C1/B0/PF_PATH_FREEZE_H0_B0_smoke01_path_sha256.txt
```

记录其 SHA-256，并且后续不得再用 `generate` 覆盖。脚本在 pilot generate 时默认拒绝覆盖已有文件；`ALLOW_PILOT_PATH_OVERWRITE=true` 只允许在书面作废旧 H0、准备完整重新冻结时使用。

#### 4.2.3 replay 单次权重候选

执行命令前先把机器人遥控到 H0 起点附近、对齐航向并等待液体静稳。命令启动后没有 Enter 门；一旦位置和航向连续满足门控 `0.5 s`，脚本自动进入 recorder/planner 启动流程。

在另一个终端先按第 6 节启动 C1 standalone monitor。然后按本次 block 和冻结顺序位置设置命名；切换候选时不能只改方法而遗留旧标签：

```bash
set -euo pipefail
export SCOUT_WS=/home/geist/scout_ws
export EXP_DATE=20260718
export PATH_JSON=/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json
: "${H0_EXPECTED_SHA256:?fill the frozen H0 SHA-256 before P3}"
export PILOT_METHOD=W2
export BLOCK=01
export REPEAT=01
export BLOCK_SEGMENT_ID=PF_WS_b01_seg01
export SPLIT_BLOCK=false
export ORDER_POSITION=04  # W2 在 block 01 的冻结位置；按表逐 trial 修改
export ACQUISITION_RETRY=false
export RETRY_REASON_FILE=""
: "${PILOT_T_SETTLE_SEC:?P3/P4 开始前必须 export 预注册的 PILOT_T_SETTLE_SEC}"

unset MATRIX_PRESET
: "${PILOT_METHOD:?PILOT_METHOD must be explicit for protocol v1.0}"
[[ -z "${MATRIX_PRESET:-}" ]] || { echo "MATRIX_PRESET is forbidden" >&2; exit 2; }
case "${PILOT_METHOD}" in B0|Bsmooth|W1|W2|W5) ;; *) echo "forbidden PILOT_METHOD=${PILOT_METHOD}" >&2; exit 2 ;; esac
case "${SPLIT_BLOCK}" in true|false) ;; *) echo "SPLIT_BLOCK must be true|false" >&2; exit 2 ;; esac
[[ "${BLOCK}" =~ ^0[1-3]$ ]] || { echo "P3 BLOCK must be 01..03" >&2; exit 2; }
[[ "${ORDER_POSITION}" =~ ^0[1-5]$ ]] || { echo "ORDER_POSITION must be 01..05" >&2; exit 2; }
case "${REPEAT}" in
  01) [[ "${ACQUISITION_RETRY}" == "false" && -z "${RETRY_REASON_FILE}" ]] || exit 2 ;;
  02) [[ "${ACQUISITION_RETRY}" == "true" ]] && test -s "${RETRY_REASON_FILE}" || exit 2 ;;
  *) echo "P3 REPEAT must be 01 or protocol-authorized 02" >&2; exit 2 ;;
esac

export RUN_LABEL="PF_WS_H0_C1_${PILOT_METHOD}_b${BLOCK}_r${REPEAT}"
export RUN_OUT_DIR="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_parameter_pilot/PF/WS/H0_C1/${PILOT_METHOD}"
mkdir -p "${RUN_OUT_DIR}"
if [[ "${ACQUISITION_RETRY}" == "true" ]]; then
  cp -- "${RETRY_REASON_FILE}" "${RUN_OUT_DIR}/${RUN_LABEL}_retry_reason.txt"
  sha256sum "${RUN_OUT_DIR}/${RUN_LABEL}_retry_reason.txt" \
    > "${RUN_OUT_DIR}/${RUN_LABEL}_retry_reason_sha256.txt"
fi
```

每条计入 15 个矩阵单元的命令前，都必须保存 monitor 配置、reset 成功证据和固定静置计时 sidecar。下面的文件名与本次 `RUN_LABEL` 绑定，不得复用上一 trial 的结果：

```bash
set -euo pipefail
timeout 5s rostopic echo -n 1 /slosh/height >/dev/null
rosparam get /slosh/slosh_monitor \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_standalone_monitor_rosparam.yaml"
sha256sum "${RUN_OUT_DIR}/${RUN_LABEL}_standalone_monitor_rosparam.yaml" \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_standalone_monitor_sha256.txt"
sha256sum -c "${RUN_OUT_DIR}/${RUN_LABEL}_standalone_monitor_sha256.txt"

export RESET_SIDECAR="${RUN_OUT_DIR}/${RUN_LABEL}_slosh_reset.txt"
{
  printf 'reset_start_utc=%s\n' "$(date --utc --iso-8601=ns)"
  rosservice call /slosh/reset
  echo 'height_unit=m'
  timeout 5s rostopic echo -n 1 /slosh/debug
  echo 'pass=true'
} > "${RESET_SIDECAR}" 2>&1
grep -Fxq 'pass=true' "${RESET_SIDECAR}"

export SETTLE_SIDECAR="${RUN_OUT_DIR}/${RUN_LABEL}_t_settle.txt"
python3 - "${PILOT_T_SETTLE_SEC}" "${SETTLE_SIDECAR}" <<'PY'
import datetime
import math
import pathlib
import sys
import time

target = float(sys.argv[1])
if not math.isfinite(target) or target <= 0.0:
    raise SystemExit(f"invalid frozen PILOT_T_SETTLE_SEC={target!r}")
output = pathlib.Path(sys.argv[2])
start_epoch = time.time()
start_mono = time.monotonic()
start_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
time.sleep(target)
elapsed = time.monotonic() - start_mono
if elapsed + 1e-6 < target:
    raise SystemExit(f"P3 T_SETTLE incomplete: target={target}, elapsed={elapsed}")
output.write_text(
    "pass=true\n"
    f"target_sec={target:.12g}\n"
    f"elapsed_monotonic_sec={elapsed:.12g}\n"
    f"start_epoch_sec={start_epoch:.9f}\n"
    f"end_epoch_sec={time.time():.9f}\n"
    f"start_utc={start_utc}\n"
    f"end_utc={datetime.datetime.now(datetime.timezone.utc).isoformat()}\n",
    encoding="utf-8",
)
PY
test -s "${RUN_OUT_DIR}/${RUN_LABEL}_standalone_monitor_sha256.txt"
test -s "${RESET_SIDECAR}"
test -s "${SETTLE_SIDECAR}"
grep -Fxq 'pass=true' "${SETTLE_SIDECAR}"
```

计时通过后不再移动容器或车体，立即启动当次一键 trial：

```bash
PILOT_MODE=true \
PILOT_METHOD="${PILOT_METHOD}" \
DATE="${EXP_DATE}" \
PATH_SOURCE_MODE=replay \
PATH_FILE="${PATH_JSON}" \
REQUIRE_PATH_HASH=true \
PATH_EXPECTED_SHA256="${H0_EXPECTED_SHA256}" \
RUN_LABEL="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID}" \
SPLIT_BLOCK="${SPLIT_BLOCK}" \
ORDER_POSITION="${ORDER_POSITION}" \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
START_POS_TOL=0.08 \
START_YAW_TOL=0.15 \
START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC=120 \
PATH_PUBLISH_RATE=2.0 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_SCAN=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
OPERATOR_NOTE="C1 model-side weight screening ${PILOT_METHOD}; segment=${BLOCK_SEGMENT_ID}; split=${SPLIT_BLOCK}; position=${ORDER_POSITION}; retry=${ACQUISITION_RETRY}; no RGB" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

本协议的 `PILOT_METHOD` 只允许 `B0`、`Bsmooth`、`W1`、`W2`、`W5`。脚本会自动映射 `VARIANT/W_SLOSH`，pilot 默认不录 RGB，并把路径模式、起点容差和实际权重写入 sidecar。机器人用遥控回到起点附近即可；脚本默认允许 `0.08 m / 0.15 rad` 的误差，但仍需以地面起点标记为准，不能继续无上限放宽。

#### 4.2.4 运行中观察、停止与输出

启动横幅至少核对：

```text
pilot=true
variant / PILOT_METHOD 正确
record_rgb=false
path_source=replay
path_file 指向冻结 H0
start_gate=0.08 m / 0.15 rad
v_ref/w_slosh 与候选一致
```

正常日志顺序应为：

```text
[path] starting replay source
[path] waiting ... start gate
[record] starting black-box recorder
[goal] replay mode ... skipped
[launch] starting planner
[run] recording ...
```

到点后继续保留至少 `5 s` 残余晃动。可以等待 recorder 到 `60 s` 自动退出；如果要提前结束，先等满到达后 `5 s`，再在一键终端按 `Ctrl-C`，由脚本统一清理。不要另开终端抢先杀 recorder 或 path runner。

每个 run 的主要输出为：

```text
${RUN_OUT_DIR}/${RUN_LABEL}.bag
${RUN_OUT_DIR}/${RUN_LABEL}_one_click_meta.env
${RUN_OUT_DIR}/${RUN_LABEL}_info.txt
${RUN_OUT_DIR}/${RUN_LABEL}_recorded_topics.txt
${RUN_OUT_DIR}/${RUN_LABEL}_{path_generator,recorder,planner}.log
```

### 4.3 15 次交错顺序

| Pilot block | 顺序 |
| ---: | --- |
| 01 | B0 → W1 → B_smooth → W2 → W5 |
| 02 | W2 → B0 → W5 → W1 → B_smooth |
| 03 | B_smooth → W5 → W2 → B0 → W1 |

每次优先使用第 4.2 节的一键命令；如果需要逐节点排障，也可使用本文后面的多终端 replay/recorder/planner 流程。对应运行环境设为：

```bash
export EXP_DATE=20260718       # 改成实际日期
export STAGE=PF
export GROUP=WS
export PATH_ID=H0             # 权重筛选默认使用独立冻结 H0
export CONTAINER=C1
export METHOD=W1              # B0 / Bsmooth / W1 / W2 / W5
export BLOCK=01
export REPEAT=01

unset MATRIX_PRESET
export PILOT_METHOD="${METHOD}"
: "${PILOT_METHOD:?PILOT_METHOD must be explicit for protocol v1.0}"
[[ -z "${MATRIX_PRESET:-}" ]] || { echo "MATRIX_PRESET is forbidden" >&2; exit 2; }
case "${PILOT_METHOD}" in B0|Bsmooth|W1|W2|W5) ;; *) echo "forbidden PILOT_METHOD=${PILOT_METHOD}" >&2; exit 2 ;; esac
export RUN_LABEL="${STAGE}_${GROUP}_${PATH_ID}_${CONTAINER}_${METHOD}_b${BLOCK}_r${REPEAT}"
export RUN_OUT_DIR="/home/geist/slosh_bags/real/${EXP_DATE}_spmpc_parameter_pilot/${STAGE}/${GROUP}/${PATH_ID}_${CONTAINER}/${METHOD}"
```

对应命名示例：

```text
PF_WS_H0_C1_B0_b01_r01
PF_WS_H0_C1_Bsmooth_b01_r01
PF_WS_H0_C1_W1_b01_r01
PF_WS_H0_C1_W2_b01_r01
PF_WS_H0_C1_W5_b01_r01
```

执行第 4.2.3 节命令时，将示例里的 `PILOT_METHOD=W2`、`RUN_LABEL=...W2...` 和 `RUN_OUT_DIR=.../W2` 分别替换为上述三个已导出的变量，避免方法名、实际权重、标签和目录不一致。

所有 `PF` bag 只用于模型侧参数决策，不进入正式 40/64/88。每次运行后只做数据完整性和安全检查；不得看完一个候选的内部模型排序后改变剩余顺序。

### 4.4 权重选择与 Smooth-match 后置

完成全部预注册候选后统一分析：

1. 先剔除未通过任务、tracking、实时性、fallback、执行层和数据完整性门槛的候选；
2. 检查 Z2/Z3 的完整 horizon、optimized first action 和局部速度/激励分配，排除全程统一降速；
3. 先用统一 `/slosh/height` repeatability 复核跨方法噪声，再只对 W1/W2/W5 比较其在线 `/spmpc/slosh_height`、horizon peak/RMS、post-arrival RMS、完成时间和 tracking；不得把 B0/B_smooth 的零/缺失在线 `H_modal` 当作较优结果；
4. 相邻权重的额外模型侧收益低于冻结 `delta_model` 时选择较小权重；
5. 形成书面权重决策，设置 `FINAL_W_SLOSH=<最终值>`，并明确标记为 `model-side frozen weight`；
6. 严格按矩阵第 5.4 节执行固定 12 次 `PF_SM`：先由 P3 完成时间按冻结公式得到 `v_c`，再一次性冻结 `M-=v_c-0.01`、`M0=v_c`、`M+=v_c+0.01`；三个候选与最终 S-MPCC 各做 3 次，只按配对完成时间误差选择；
7. 归档四方法配置、effective config、路径/config hash、pilot 数据和 Git revision，提交正式 freeze commit。

P4 的固定顺序为：block 01 `S→M-→M0→M+`，block 02 `M0→S→M+→M-`，block 03 `M+→M0→M-→S`。只有候选 `3/3` 完成、全部准入合格且中位配对完成时间误差 `≤5%` 时才冻结；无候选通过则正式实验保持 NO-GO，不得追加速度或重复到满意。

#### 4.4.1 P4 四条件的唯一命名与一键映射

P4 不得把 M-/M0/M+ 都标成同一个 `SmoothMatch`。先从 P3A/P3 报告填写冻结值，并在任何 P4 数据产生前一次性验证三个速度：

```bash
set -euo pipefail
: "${FINAL_W_SLOSH:?fill frozen final weight}"
: "${M_MINUS_V_REF:?fill frozen M- v_ref}"
: "${M0_V_REF:?fill frozen M0 v_ref}"
: "${M_PLUS_V_REF:?fill frozen M+ v_ref}"
: "${P4_SAFE_V_REF_MIN:?fill P3A safe minimum}"
: "${P4_SAFE_V_REF_MAX:?fill P3A safe maximum}"
python3 - \
  "${M_MINUS_V_REF}" "${M0_V_REF}" "${M_PLUS_V_REF}" \
  "${P4_SAFE_V_REF_MIN}" "${P4_SAFE_V_REF_MAX}" <<'PY'
import math
import sys

m_minus, m0, m_plus, safe_min, safe_max = map(float, sys.argv[1:])
if not all(map(math.isfinite, (m_minus, m0, m_plus, safe_min, safe_max))):
    raise SystemExit("P4 v_ref contains non-finite value")
if not safe_min < safe_max:
    raise SystemExit("invalid P4 safe interval")
if not (safe_min <= m_minus < m0 < m_plus <= safe_max):
    raise SystemExit("P4 candidates must be distinct, ordered, and inside the safe interval")
if not math.isclose(m0 - m_minus, 0.01, abs_tol=1e-12):
    raise SystemExit("M0-M- must equal 0.01 m/s")
if not math.isclose(m_plus - m0, 0.01, abs_tol=1e-12):
    raise SystemExit("M+-M0 must equal 0.01 m/s")
print("P4_CANDIDATES=PASS")
PY
```

每个 trial 只选一个 `P4_CONDITION=S|Mminus|M0|Mplus`，并按冻结表填写 `BLOCK` 和 `ORDER_POSITION`：

```bash
set -euo pipefail
export SCOUT_WS=/home/geist/scout_ws
export EXP_DATE=20260718
export PATH_JSON="${SCOUT_WS}/docs/实物实验注意事项/对比试验/实物对比实验/freeze/paths/H1_P2_s_curve.json"
: "${H1_EXPECTED_SHA256:?fill the frozen H1 SHA-256 before P4}"
export P4_CONDITION=Mminus
export BLOCK=01
export REPEAT=01
export ORDER_POSITION=02
export BLOCK_SEGMENT_ID=PF_SM_b01_seg01
export SPLIT_BLOCK=false
export ACQUISITION_RETRY=false
export RETRY_REASON_FILE=""
: "${PILOT_T_SETTLE_SEC:?fill pre-registered pilot settle time}"

case "${FINAL_W_SLOSH}" in
  1|1.0) FINAL_PILOT_METHOD=W1 ;;
  2|2.0) FINAL_PILOT_METHOD=W2 ;;
  5|5.0) FINAL_PILOT_METHOD=W5 ;;
  *) echo "FINAL_W_SLOSH must be 1, 2, or 5" >&2; exit 2 ;;
esac
case "${P4_CONDITION}" in
  S)      export PILOT_METHOD="${FINAL_PILOT_METHOD}"; export P4_V_REF=0.20 ;;
  Mminus) export PILOT_METHOD=Bsmooth; export P4_V_REF="${M_MINUS_V_REF}" ;;
  M0)     export PILOT_METHOD=Bsmooth; export P4_V_REF="${M0_V_REF}" ;;
  Mplus)  export PILOT_METHOD=Bsmooth; export P4_V_REF="${M_PLUS_V_REF}" ;;
  *) echo "P4_CONDITION must be S|Mminus|M0|Mplus" >&2; exit 2 ;;
esac
[[ "${BLOCK}" =~ ^0[1-3]$ ]] || { echo "P4 BLOCK must be 01..03" >&2; exit 2; }
[[ "${ORDER_POSITION}" =~ ^0[1-4]$ ]] || { echo "P4 ORDER_POSITION must be 01..04" >&2; exit 2; }
case "${REPEAT}" in
  01) [[ "${ACQUISITION_RETRY}" == "false" && -z "${RETRY_REASON_FILE}" ]] || exit 2 ;;
  02) [[ "${ACQUISITION_RETRY}" == "true" ]] && test -s "${RETRY_REASON_FILE}" || exit 2 ;;
  *) echo "P4 REPEAT must be 01 or protocol-authorized 02" >&2; exit 2 ;;
esac

export RUN_LABEL="PF_SM_H1_C1_${P4_CONDITION}_b${BLOCK}_r${REPEAT}"
export RUN_OUT_DIR="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_parameter_pilot/PF/SM/H1_C1/${P4_CONDITION}"
mkdir -p "${RUN_OUT_DIR}"
if [[ "${ACQUISITION_RETRY}" == "true" ]]; then
  cp -- "${RETRY_REASON_FILE}" "${RUN_OUT_DIR}/${RUN_LABEL}_retry_reason.txt"
  sha256sum "${RUN_OUT_DIR}/${RUN_LABEL}_retry_reason.txt" \
    > "${RUN_OUT_DIR}/${RUN_LABEL}_retry_reason_sha256.txt"
fi
```

使用上面当次变量执行第 4.2.3 节完全相同的 monitor 配置 hash、reset 和 `PILOT_T_SETTLE_SEC` sidecar 代码块，然后运行：

```bash
PILOT_MODE=true \
PILOT_METHOD="${PILOT_METHOD}" \
PILOT_CONDITION="${P4_CONDITION}" \
DATE="${EXP_DATE}" \
PATH_SOURCE_MODE=replay \
PATH_FILE="${PATH_JSON}" \
REQUIRE_PATH_HASH=true \
PATH_EXPECTED_SHA256="${H1_EXPECTED_SHA256}" \
RUN_LABEL="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID}" \
SPLIT_BLOCK="${SPLIT_BLOCK}" \
ORDER_POSITION="${ORDER_POSITION}" \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
START_POS_TOL=0.05 \
START_YAW_TOL=0.10 \
START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC=120 \
PILOT_RECORD_RGB=false \
RECORD_STANDALONE_SLOSH=true \
RECORD_SCAN=true \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
V_REF="${P4_V_REF}" \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
OPERATOR_NOTE="P4 ${P4_CONDITION}; v_ref=${P4_V_REF}; segment=${BLOCK_SEGMENT_ID}; position=${ORDER_POSITION}; no RGB" \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

`PILOT_METHOD=Bsmooth` 只是代码 variant 映射；论文条件和文件身份由唯一 `PILOT_CONDITION`/`RUN_LABEL` 区分。P4 分析必须同时读取 label 和 metadata 中的实际 `v_ref`，不得仅凭 `B_smooth` variant 合并三个候选。

只有第 7 步以及 K6、replay 和全部 freeze manifest 准入完成后，才允许把 `STAGE` 改为 `S1` 开始正式 40 次。

不录 RGB 的 PF pilot 不能证明真实液面改善。本协议不使用候选权重之间的 RGB 排名选权；P0/P8 的 RGB 只用于视觉链、同步、噪声和激励准入，正式 RGB 结果不得反向修改权重，否则已有正式数据必须隔离并以新 release 重新开始。

---

## 5. 正式 40/88 与排障用多终端环境

本节及第 6–11 节不是一键脚本的内部步骤，而是正式采集/排障时的独立多终端入口。它使用 `spmpc_experiment.launch`，因此可以显式设置 C1/C2 container config、使用人工 Enter 门，并让 recorder 录制 90 s。

代码/工具前置 NO-GO：recorder whitelist 已补入 `/spmpc/debug/warm_start` 和 `/spmpc/debug/warm_start_status`，但仍须在独立 bag smoke 中确认两个话题与 `used_fallback` 实际可读；actual/zero replay 唯一工具和 K6 唯一脚本也必须通过 smoke。只有 `warm_start_head` 或 pre-solve 快照仍不能替代这些验收。

变量名注意：

```text
一键脚本输出变量：RUN_OUT_DIR
直接 recorder 输出变量：OUT_DIR
```

二者不要混用。PF 日常权重筛选优先按第 4 节一键执行；只有节点排障时才把 PF 放到本节的多终端流程。

先按随机表填写以下字段。示例是 S1/E2、H1、C1、B0、block 01：

```bash
export EXP_DATE=20260718
export STAGE=S1
export GROUP=E2
export PATH_ID=H1
export CONTAINER=C1
export METHOD=B0
export BLOCK=01
export REPEAT=01
export BLOCK_SEGMENT_ID=S1_b01_seg01  # 同一连续 5-trial super-block 共享
export SPLIT_BLOCK=false              # 中断后新 segment 改为 true
export ORDER_POSITION=01              # 本 trial 在冻结 super-block 中的位置
export ACQUISITION_RETRY=false        # 只有协议允许的 r02 才改为 true
export RETRY_REASON_FILE=""            # r02 时必须指向已写好的故障记录
```

`BLOCK_SEGMENT_ID` 不能由 `GROUP` 自动生成：S1 中 E2 和 E3 共享同一个连续 5-trial super-block。无中断时同一 block 全部使用 `..._seg01` 且 `SPLIT_BLOCK=false`；若必须跨时段继续，将后续 trial 改为 `..._seg02`（再次中断则 `seg03`）并设 `SPLIT_BLOCK=true`，不得回填为原 segment。`ORDER_POSITION` 按冻结顺序表填写：S1 为 `01–05`，E1/E2-C2 为 `01–03`。

正式阶段还必须从只读执行 manifest 读取 `FREEZE_ID`，不能用日期或操作者临时起名代替。manifest 至少包含 `protocol_id: SMPCC-REAL-40-88-v1.0`、唯一 `freeze_id`、`e4_enabled: false` 以及代码、路径、容器、配置、视觉/K6 和随机表哈希。

正式运行 `METHOD=Bslosh` 前还必须从只读 freeze 配置填写：

```bash
read -rp "FINAL_W_SLOSH（从只读 freeze 配置填写）: " FINAL_W_SLOSH
export FINAL_W_SLOSH
```

正式或 pilot 运行 `METHOD=SmoothMatch` 前填写对应的独立调速候选/最终值：

```bash
read -rp "SMOOTH_MATCH_V_REF（候选或只读 freeze 值）: " SMOOTH_MATCH_V_REF
export SMOOTH_MATCH_V_REF
```

方法名与代码映射：

| `METHOD` | `VARIANT` | `V_REF` | `W_SLOSH` |
| --- | --- | ---: | ---: |
| `B0` | `B0` | `0.20` | `0` |
| `Bsmooth` | `B_smooth` | `0.20` | `0` |
| `W1` | `B_slosh` | `0.20` | `1` |
| `W2` | `B_slosh` | `0.20` | `2` |
| `W5` | `B_slosh` | `0.20` | `5` |
| `Bslosh` | `B_slosh` | `0.20` | `FINAL_W_SLOSH` |
| `SmoothMatch` | `B_smooth` | 独立 pilot 冻结值 | `0` |

执行映射：

```bash
case "${METHOD}" in
  B0)      export VARIANT=B0;       export V_REF=0.20; export W_SLOSH=0.0 ;;
  Bsmooth) export VARIANT=B_smooth; export V_REF=0.20; export W_SLOSH=0.0 ;;
  W1)      export VARIANT=B_slosh;  export V_REF=0.20; export W_SLOSH=1.0 ;;
  W2)      export VARIANT=B_slosh;  export V_REF=0.20; export W_SLOSH=2.0 ;;
  W5)      export VARIANT=B_slosh;  export V_REF=0.20; export W_SLOSH=5.0 ;;
  Bslosh)
    : "${FINAL_W_SLOSH:?先完成参数 pilot 并填写冻结的 FINAL_W_SLOSH}"
    export VARIANT=B_slosh
    export V_REF=0.20
    export W_SLOSH="${FINAL_W_SLOSH}"
    ;;
  SmoothMatch)
    : "${SMOOTH_MATCH_V_REF:?先填写独立 pilot 冻结的 SMOOTH_MATCH_V_REF}"
    export VARIANT=B_smooth
    export V_REF="${SMOOTH_MATCH_V_REF}"
    export W_SLOSH=0.0
    ;;
  *) echo "未知 METHOD=${METHOD}" >&2; exit 2 ;;
esac
```

路径和容器映射：

```bash
export FREEZE_ROOT="${SCOUT_WS}/docs/实物实验注意事项/对比试验/实物对比实验/freeze"
export FREEZE_MANIFEST="${FREEZE_ROOT}/freeze_manifest.yaml"

case "${PATH_ID}" in
  H1) export PATH_JSON="${FREEZE_ROOT}/paths/H1_P2_s_curve.json" ;;
  L1) export PATH_JSON="${FREEZE_ROOT}/paths/L1_gentle.json" ;;
  H0) export PATH_JSON="${FREEZE_ROOT}/paths/H0_weight_pilot.json" ;;
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
    : "${C2_DAMPING_RATIO:?填写冻结的 C2 阻尼比}"
    export CONTAINER_CONFIG="${C2_CONTAINER_CONFIG}"
    export CONTAINER_RADIUS="${C2_CONTAINER_RADIUS}"
    export LIQUID_HEIGHT="${C2_LIQUID_HEIGHT}"
    export DAMPING_RATIO="${C2_DAMPING_RATIO}"
    ;;
  *) echo "未知 CONTAINER=${CONTAINER}" >&2; exit 2 ;;
esac
```

生成 run 名称和目录：

```bash
case "${STAGE}" in
  PF)
    export RUN_CLASS=pilot
    export PILOT_MODE_META=true
    export PILOT_METHOD_META="${METHOD}"
    export RECORD_RGB=false
    export DATASET_ROOT="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_parameter_pilot"
    ;;
  S1|S2A|S2B)
    export RUN_CLASS=formal
    export PILOT_MODE_META=false
    export PILOT_METHOD_META=""
    export RECORD_RGB=true
    export DATASET_ROOT="${HOME}/slosh_bags/real/${EXP_DATE}_spmpc_formal"
    ;;
  *) echo "未知 STAGE=${STAGE}" >&2; exit 2 ;;
esac

export RUN_LABEL="${STAGE}_${GROUP}_${PATH_ID}_${CONTAINER}_${METHOD}_b${BLOCK}_r${REPEAT}"
export OUT_DIR="${DATASET_ROOT}/${STAGE}/${GROUP}/${PATH_ID}_${CONTAINER}/${METHOD}"
export NAME="${RUN_LABEL}"
export CONTAINER_YAML="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/containers/${CONTAINER_CONFIG}.yaml"
export VALIDATOR_SCRIPT="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/validate_spmpc_formal_freeze.py"

set -euo pipefail
: "${BLOCK_SEGMENT_ID:?BLOCK_SEGMENT_ID must be explicit}"
case "${SPLIT_BLOCK}" in true|false) ;; *) echo "SPLIT_BLOCK must be true|false" >&2; exit 2 ;; esac
if [[ "${RUN_CLASS}" == "formal" ]]; then
  [[ "${BLOCK}" =~ ^0[1-8]$ ]] || { echo "formal BLOCK must be 01..08" >&2; exit 2; }
  case "${STAGE}/${GROUP}" in
    S1/E2|S1/E3) [[ "${ORDER_POSITION}" =~ ^0[1-5]$ ]] || { echo "S1 ORDER_POSITION must be 01..05" >&2; exit 2; } ;;
    S2A/E1|S2B/E2) [[ "${ORDER_POSITION}" =~ ^0[1-3]$ ]] || { echo "S2 ORDER_POSITION must be 01..03" >&2; exit 2; } ;;
    *) echo "illegal formal STAGE/GROUP=${STAGE}/${GROUP}" >&2; exit 2 ;;
  esac
else
  [[ "${BLOCK}" =~ ^0[1-3]$ ]] || { echo "pilot BLOCK must be 01..03" >&2; exit 2; }
  case "${GROUP}" in
    WS) [[ "${ORDER_POSITION}" =~ ^0[1-5]$ ]] || { echo "PF/WS ORDER_POSITION must be 01..05" >&2; exit 2; } ;;
    SM) [[ "${ORDER_POSITION}" =~ ^0[1-4]$ ]] || { echo "PF/SM ORDER_POSITION must be 01..04" >&2; exit 2; } ;;
    *) echo "pilot GROUP must be WS or SM" >&2; exit 2 ;;
  esac
fi
case "${REPEAT}" in
  01)
    [[ "${ACQUISITION_RETRY}" == "false" && -z "${RETRY_REASON_FILE}" ]] || {
      echo "r01 must use ACQUISITION_RETRY=false and empty RETRY_REASON_FILE" >&2; exit 2;
    }
    ;;
  02)
    [[ "${ACQUISITION_RETRY}" == "true" ]] || { echo "r02 requires ACQUISITION_RETRY=true" >&2; exit 2; }
    test -s "${RETRY_REASON_FILE}"
    ;;
  *) echo "formal REPEAT must be 01 or protocol-authorized 02" >&2; exit 2 ;;
esac
test -s "${PATH_JSON}"
test -s "${CONTAINER_YAML}"
mkdir -p "${OUT_DIR}"
test ! -e "${OUT_DIR}/${NAME}.bag"

if [[ "${RUN_CLASS}" == "formal" ]]; then
  test -s "${FREEZE_MANIFEST}"
  test -x "${VALIDATOR_SCRIPT}"
  export FREEZE_VALIDATION_REPORT="${OUT_DIR}/${NAME}_freeze_validation_report.txt"
  if ! python3 "${VALIDATOR_SCRIPT}" \
      --manifest "${FREEZE_MANIFEST}" \
      --repo-root "${SCOUT_WS}" \
      --stage "${STAGE}" \
      --group "${GROUP}" \
      --method "${METHOD}" \
      --variant "${VARIANT}" \
      --v-ref "${V_REF}" \
      --w-slosh "${W_SLOSH}" \
      --path-id "${PATH_ID}" \
      --path-file "${PATH_JSON}" \
      --container-id "${CONTAINER}" \
      --container-config "${CONTAINER_CONFIG}" \
      --container-yaml "${CONTAINER_YAML}" \
      --container-radius "${CONTAINER_RADIUS}" \
      --liquid-height "${LIQUID_HEIGHT}" \
      --damping-ratio "${DAMPING_RATIO}" \
      > "${FREEZE_VALIDATION_REPORT}" 2>&1; then
    cat "${FREEZE_VALIDATION_REPORT}" >&2
    echo "FORMAL RUN BLOCKED: freeze validation failed" >&2
    exit 2
  fi
  cat "${FREEZE_VALIDATION_REPORT}"
  grep -Fxq 'FORMAL_FREEZE_VALIDATION=PASS' "${FREEZE_VALIDATION_REPORT}"
  export PROTOCOL_ID="SMPCC-REAL-40-88-v1.0"
  export FREEZE_STATUS="GO"
  export E4_ENABLED="false"
  export FREEZE_ID="$(sed -n 's/^FREEZE_ID=//p' "${FREEZE_VALIDATION_REPORT}")"
  export T_SETTLE="$(sed -n 's/^T_SETTLE=//p' "${FREEZE_VALIDATION_REPORT}")"
  [[ -n "${FREEZE_ID}" && -n "${T_SETTLE}" ]]
else
  export PROTOCOL_ID="SMPCC-REAL-40-88-v1.0"
  export FREEZE_ID="PF_UNFROZEN"
  export FREEZE_STATUS="NO-GO"
  export E4_ENABLED="false"
  export T_SETTLE="${T_SETTLE:-10}"
  export FREEZE_VALIDATION_REPORT=""
fi

echo "RUN_LABEL=${RUN_LABEL}"
echo "PROTOCOL_ID=${PROTOCOL_ID} FREEZE_ID=${FREEZE_ID}"
echo "PATH_JSON=${PATH_JSON}"
echo "CONTAINER_CONFIG=${CONTAINER_CONFIG}"
echo "RUN_CLASS=${RUN_CLASS}"
echo "VARIANT=${VARIANT} V_REF=${V_REF} W_SLOSH=${W_SLOSH}"
echo "OUT_DIR=${OUT_DIR}"
```

把本次公共变量保存到临时文件，随后每个 trial 终端都 source 它：

```bash
export RUN_ENV="/tmp/${RUN_LABEL}.env"
declare -px SCOUT_WS EXP_DATE STAGE GROUP PATH_ID CONTAINER METHOD BLOCK REPEAT \
  BLOCK_SEGMENT_ID SPLIT_BLOCK ORDER_POSITION ACQUISITION_RETRY RETRY_REASON_FILE \
  RUN_CLASS PILOT_MODE_META PILOT_METHOD_META RECORD_RGB DATASET_ROOT \
  PROTOCOL_ID FREEZE_ID FREEZE_STATUS E4_ENABLED FREEZE_ROOT FREEZE_MANIFEST T_SETTLE \
  VALIDATOR_SCRIPT FREEZE_VALIDATION_REPORT \
  VARIANT V_REF W_SLOSH PATH_JSON CONTAINER_CONFIG \
  CONTAINER_RADIUS LIQUID_HEIGHT DAMPING_RATIO RUN_LABEL OUT_DIR NAME \
  CONTAINER_YAML RUN_ENV > "${RUN_ENV}"
```

同时保存路径、配置和 revision 证据：

```bash
sha256sum "${PATH_JSON}" > "${OUT_DIR}/${NAME}_path_sha256.txt"
printf '%s\n' "${FREEZE_ID}" > "${OUT_DIR}/${NAME}_freeze_id.txt"
if [[ "${RUN_CLASS}" == "formal" ]]; then
  sha256sum "${FREEZE_MANIFEST}" > "${OUT_DIR}/${NAME}_freeze_manifest_sha256.txt"
  test -s "${FREEZE_VALIDATION_REPORT}"
fi
if [[ "${ACQUISITION_RETRY}" == "true" ]]; then
  cp -- "${RETRY_REASON_FILE}" "${OUT_DIR}/${NAME}_retry_reason.txt"
  sha256sum "${OUT_DIR}/${NAME}_retry_reason.txt" \
    > "${OUT_DIR}/${NAME}_retry_reason_sha256.txt"
fi
sha256sum \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/planner/common.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/planner/variants.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/platforms/scout_mini.yaml" \
  "${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/config/experiments/fixed_path.yaml" \
  "${CONTAINER_YAML}" \
  > "${OUT_DIR}/${NAME}_config_sha256.txt"

git rev-parse HEAD > "${OUT_DIR}/${NAME}_git_revision.txt"
git status --short > "${OUT_DIR}/${NAME}_git_status.txt"
cp "${RUN_ENV}" "${OUT_DIR}/${NAME}_run_env.env"
```

---

## 6. 终端 A：standalone slosh monitor

每次切换容器时重启 monitor。同一容器连续 block 可以保持进程运行，但每个 trial 运动前必须 reset。

边界说明：一键脚本中的 `RECORD_STANDALONE_SLOSH=true` 只会把已经存在的 `/slosh/*` 加入 recorder whitelist，不会执行下面的 `roslaunch`。计入 P3 固定 15 次的 PF 必须启动本终端，并在每个 trial 前 reset；只有不计入矩阵的路径生成、路径 replay 和纯安全 smoke 可以省略。

```bash
set -euo pipefail
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

## 7. 正式多终端 B：装载冻结路径并停在人工开始门

下面的 `--manual-start + 0.05/0.10` 是正式多终端口径，与一键 pilot 的“无 Enter + 0.08/0.15”不同，不要复制到第 4 节后误以为一键脚本也会等待 Enter。

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

机器人必须先回到冻结路径起点标记，并完成固定 `T_SETTLE`；planner 和 recorder 也必须已经启动。正式 `H_vis` 是 raw RGB 的离线派生量，本终端不会提供在线 `H_vis` 门。

---

## 8. 正式多终端 C：启动 recorder

这里直接调用 recorder，所以可以设置 `RECORD_SEC=90`；一键脚本仍受 `MAX_RECORD_SEC` 约束，默认只录 60 s。

```bash
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"
cd "${SCOUT_WS}"

DATE="${EXP_DATE}" \
VARIANT="${VARIANT}" \
RUN_CLASS="${RUN_CLASS}" \
PILOT_MODE="${PILOT_MODE_META}" \
PILOT_METHOD="${PILOT_METHOD_META}" \
BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID}" \
SPLIT_BLOCK="${SPLIT_BLOCK}" \
ORDER_POSITION="${ORDER_POSITION}" \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${NAME}" \
OUT_DIR="${OUT_DIR}" \
RECORD_SEC=90 \
RECORD_RGB="${RECORD_RGB}" \
RECORD_CAMERA="${RECORD_RGB}" \
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
W_SLOSH="${W_SLOSH}" \
SLOSH_HEIGHT_MAX=-1.0 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
CONTROL_FREQUENCY=30 \
PATH_SOURCE_MODE=replay \
PATH_FILE="${PATH_JSON}" \
START_POS_TOL=0.05 \
START_YAW_TOL=0.10 \
START_HOLD_SEC=0.5 \
OPERATOR_NOTE="freeze_id=${FREEZE_ID} ${RUN_CLASS} ${STAGE}/${GROUP} ${PATH_ID}/${CONTAINER} block ${BLOCK} segment=${BLOCK_SEGMENT_ID} split=${SPLIT_BLOCK} position=${ORDER_POSITION} retry=${ACQUISITION_RETRY} ${METHOD} w_slosh=${W_SLOSH}" \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

等待 recorder 打印输出路径且没有立即退出，再启动 planner。90 s 是防遗忘的最长录制保护；trial 提前结束时仍需保证到达/停止后继续录制 5 s，再用 Ctrl+C 正常关闭 recorder。

---

## 9. 正式多终端 D：启动本次 planner

这里使用 `spmpc_experiment.launch` 并显式传入 `container_config`。一键脚本使用的是 `spmpc_fixed_path.launch`，当前只能采用其默认 `tube_default`。

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
  w_slosh:="${W_SLOSH}" \
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
set -euo pipefail
source /opt/ros/noetic/setup.bash
source "${HOME}/scout_ws/devel/setup.bash"
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"

rostopic echo -n 1 /spmpc/solver_backend
rostopic echo -n 1 /spmpc/controller_variant
rostopic echo -n 1 /spmpc/debug/effective_config \
  > "${OUT_DIR}/${NAME}_effective_config_before_start.txt"

export ACTUAL_W_SLOSH="$(rosparam get "/spmpc_local_planner/variants/${VARIANT}/w_slosh")"
export ACTUAL_V_REF="$(rosparam get "/spmpc_local_planner/variants/${VARIANT}/v_ref")"
export ACTUAL_CONTAINER_RADIUS="$(rosparam get /spmpc_local_planner/slosh/container_radius)"
export ACTUAL_LIQUID_HEIGHT="$(rosparam get /spmpc_local_planner/slosh/liquid_height)"
export ACTUAL_DAMPING_RATIO="$(rosparam get /spmpc_local_planner/slosh/damping_ratio)"
export EFFECTIVE_SCALAR_REPORT="${OUT_DIR}/${NAME}_effective_scalar_check.txt"
python3 - \
  "${W_SLOSH}" "${ACTUAL_W_SLOSH}" \
  "${V_REF}" "${ACTUAL_V_REF}" \
  "${CONTAINER_RADIUS}" "${ACTUAL_CONTAINER_RADIUS}" \
  "${LIQUID_HEIGHT}" "${ACTUAL_LIQUID_HEIGHT}" \
  "${DAMPING_RATIO}" "${ACTUAL_DAMPING_RATIO}" \
  > "${EFFECTIVE_SCALAR_REPORT}" <<'PY'
import math
import sys

labels = ("w_slosh", "v_ref", "container_radius", "liquid_height", "damping_ratio")
values = [float(value) for value in sys.argv[1:]]
if len(values) != 10 or not all(math.isfinite(value) for value in values):
    raise SystemExit("invalid effective scalar inputs")
for index, label in enumerate(labels):
    expected, actual = values[2 * index : 2 * index + 2]
    if not math.isclose(expected, actual, rel_tol=1e-9, abs_tol=1e-12):
        raise SystemExit(f"{label} mismatch: expected={expected}, actual={actual}")
    print(f"{label}_expected={expected:.12g}")
    print(f"{label}_actual={actual:.12g}")
print("pass=true")
PY
grep -Fxq 'pass=true' "${EFFECTIVE_SCALAR_REPORT}"
cat "${EFFECTIVE_SCALAR_REPORT}"

rosparam get /spmpc_local_planner \
  > "${OUT_DIR}/${NAME}_planner_rosparam_before_start.yaml"
rostopic info /cmd_vel
```

只有 `effective_scalar_check.txt` 以 `pass=true` 结尾时，才证明 launch 后实际 `w_slosh`、`v_ref` 和容器半径/液深/阻尼与 validator 已核对的运行值一致；任一不一致都不得按 Enter。`/cmd_vel` 必须只有本次 S-MPCC planner 一个有效控制 publisher。

---

## 10. 正式多终端开始运动

planner 和 recorder 均正常后：

1. 机器人和容器完全停止；
2. 执行下面的 monitor 配置快照、reset 证据和计时门；只有 reset 与 `T_SETTLE` 两个 sidecar 都以 `pass=true` 结尾，才允许继续：

```bash
set -euo pipefail
export MONITOR_PARAM_SIDECAR="${OUT_DIR}/${NAME}_standalone_monitor_rosparam.yaml"
export MONITOR_HASH_SIDECAR="${OUT_DIR}/${NAME}_standalone_monitor_sha256.txt"
rosparam get /slosh/slosh_monitor > "${MONITOR_PARAM_SIDECAR}"
sha256sum "${MONITOR_PARAM_SIDECAR}" > "${MONITOR_HASH_SIDECAR}"
sha256sum -c "${MONITOR_HASH_SIDECAR}"

export RESET_SIDECAR="${OUT_DIR}/${NAME}_slosh_reset.txt"
{
  printf 'reset_start_utc=%s\n' "$(date --utc --iso-8601=ns)"
  rosservice call /slosh/reset
  echo 'height_unit=m'
  timeout 5s rostopic echo -n 1 /slosh/debug
  echo 'pass=true'
} > "${RESET_SIDECAR}" 2>&1
grep -Fxq 'pass=true' "${RESET_SIDECAR}"

export SETTLE_SIDECAR="${OUT_DIR}/${NAME}_t_settle.txt"
python3 - "${T_SETTLE}" "${SETTLE_SIDECAR}" <<'PY'
import datetime
import math
import pathlib
import sys
import time

target = float(sys.argv[1])
if not math.isfinite(target) or target <= 0.0:
    raise SystemExit(f"invalid frozen T_SETTLE={target!r}")
output = pathlib.Path(sys.argv[2])
start_epoch = time.time()
start_mono = time.monotonic()
start_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
time.sleep(target)
end_mono = time.monotonic()
end_epoch = time.time()
end_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
elapsed = end_mono - start_mono
if elapsed + 1e-6 < target:
    raise SystemExit(f"T_SETTLE incomplete: target={target}, elapsed={elapsed}")
output.write_text(
    "pass=true\n"
    f"target_sec={target:.12g}\n"
    f"elapsed_monotonic_sec={elapsed:.12g}\n"
    f"start_epoch_sec={start_epoch:.9f}\n"
    f"end_epoch_sec={end_epoch:.9f}\n"
    f"start_utc={start_utc}\n"
    f"end_utc={end_utc}\n",
    encoding="utf-8",
)
PY
test -s "${SETTLE_SIDECAR}"
test -s "${MONITOR_HASH_SIDECAR}"
test -s "${RESET_SIDECAR}"
grep -Fxq 'pass=true' "${RESET_SIDECAR}"
grep -Fxq 'pass=true' "${SETTLE_SIDECAR}"
```

可选核对 monitor-only `/slosh/height`，但它不是 K6 的 `H_vis`。

3. recorder 已连续记录至少 `2 s` raw RGB，作为覆盖 30 个有效静止帧的名义窗口；有效帧数只能由离线 QC 最终确认；
4. 确认第 9 节 `effective_scalar_check.txt` 仍为 `pass=true`，安全员、急停和走廊均就位；
5. 回到终端 B，按 Enter；
6. 等待起点门控输出 `Start pose aligned`，随后路径开始发布，车辆开始运动。

正式 `H_vis`、clipping、缺帧、30 帧零点和同步质量在 trial 后由冻结离线脚本生成。离线 visual-start QC 必须确认运动前连续 2 s 低于冻结静止阈值；失败时按矩阵的预定义采集故障规则保留原始 bag 并登记，不能伪造在线结果。

运动期间不修改参数、不切换节点、不手动重发路径。发生 safety/solver/tracking failure 时安全停止，但保留本次 bag 作为方法失败。

---

## 11. 正式多终端到达与停止顺序

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

## 12. 每次 run 的即时验收

### 12.1 一键 PF pilot

一键终端结束时会打印 `bag/meta dir`、run meta 和三个 log 路径。按本次命令填写：

```bash
set -euo pipefail
export CHECK_DIR=/home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/WS/H0_C1/W2
export NAME=PF_WS_H0_C1_W2_b01_r01
export BAG="${CHECK_DIR}/${NAME}.bag"

test -s "${BAG}"
test -s "${CHECK_DIR}/${NAME}_one_click_meta.env"
test -s "${CHECK_DIR}/${NAME}_info.txt"
test -s "${CHECK_DIR}/${NAME}_recorded_topics.txt"
test -s "${CHECK_DIR}/${NAME}_path_sha256.txt"
test -s "${CHECK_DIR}/${NAME}_standalone_monitor_sha256.txt"
test -s "${CHECK_DIR}/${NAME}_slosh_reset.txt"
test -s "${CHECK_DIR}/${NAME}_t_settle.txt"
sha256sum -c "${CHECK_DIR}/${NAME}_standalone_monitor_sha256.txt"
grep -Fxq 'pass=true' "${CHECK_DIR}/${NAME}_slosh_reset.txt"
grep -Fxq 'pass=true' "${CHECK_DIR}/${NAME}_t_settle.txt"
grep -Fxq 'pass=true' "${CHECK_DIR}/${NAME}_path_sha256.txt"
if grep -Fxq 'acquisition_retry=true' "${CHECK_DIR}/${NAME}_one_click_meta.env"; then
  test -s "${CHECK_DIR}/${NAME}_retry_reason.txt"
  sha256sum -c "${CHECK_DIR}/${NAME}_retry_reason_sha256.txt"
fi

rg '^(variant|pilot_mode|pilot_method|pilot_condition|block_segment_id|split_block|order_position|acquisition_retry|retry_reason_file|path_source_mode|path_file|start_pos_tol|start_yaw_tol|v_ref|w_slosh|record_rgb)=' \
  "${CHECK_DIR}/${NAME}_one_click_meta.env"

rg '^(run_class|pilot_mode|pilot_method|pilot_condition|block_segment_id|split_block|order_position|acquisition_retry|retry_reason_file|path_source_mode|path_file|start_pos_tol|start_yaw_tol|v_ref|w_slosh|record_rgb|record_camera)=' \
  "${CHECK_DIR}/${NAME}_info.txt"

rosbag info "${BAG}" | tee "${CHECK_DIR}/${NAME}_manual_bag_info.txt"
```

两份 metadata 必须一致满足：`pilot_mode=true`、候选方法/权重正确、segment/顺序/补录字段与本次标签一致、`record_rgb=false`、`record_camera=false`。P3 必须是冻结 H0；P4 必须是冻结 H1，且 `pilot_condition=S|Mminus|M0|Mplus` 与实际 `v_ref/w_slosh` 映射一致。

若 run 失败，先查看：

```bash
tail -n 80 "${CHECK_DIR}/${NAME}_path_generator.log"
tail -n 80 "${CHECK_DIR}/${NAME}_recorder.log"
tail -n 80 "${CHECK_DIR}/${NAME}_planner.log"
```

### 12.2 正式多终端 run

```bash
set -euo pipefail
export RUN_ENV=/tmp/S1_E2_H1_C1_B0_b01_r01.env  # 改成本次 RUN_LABEL
source "${RUN_ENV}"
export BAG="${OUT_DIR}/${NAME}.bag"
export CHECK_DIR="${OUT_DIR}"
export REQUIRE_RGB=true

test -s "${BAG}"
test -s "${CHECK_DIR}/${NAME}_freeze_id.txt"
test -s "${CHECK_DIR}/${NAME}_freeze_manifest_sha256.txt"
test -s "${CHECK_DIR}/${NAME}_freeze_validation_report.txt"
test -s "${CHECK_DIR}/${NAME}_t_settle.txt"
test -s "${CHECK_DIR}/${NAME}_standalone_monitor_sha256.txt"
test -s "${CHECK_DIR}/${NAME}_slosh_reset.txt"
test -s "${CHECK_DIR}/${NAME}_effective_scalar_check.txt"
grep -Fxq "${FREEZE_ID}" "${CHECK_DIR}/${NAME}_freeze_id.txt"
grep -Fxq 'FORMAL_FREEZE_VALIDATION=PASS' "${CHECK_DIR}/${NAME}_freeze_validation_report.txt"
for key in METHOD VARIANT V_REF W_SLOSH SMOOTH_MATCH_V_REF \
    SMOOTH_MATCH_SAFE_V_REF_MIN SMOOTH_MATCH_SAFE_V_REF_MAX; do
  grep -Eq "^${key}=.+$" "${CHECK_DIR}/${NAME}_freeze_validation_report.txt"
done
grep -Fxq 'pass=true' "${CHECK_DIR}/${NAME}_t_settle.txt"
grep -Fxq 'pass=true' "${CHECK_DIR}/${NAME}_slosh_reset.txt"
grep -Fxq 'pass=true' "${CHECK_DIR}/${NAME}_effective_scalar_check.txt"
sha256sum -c "${CHECK_DIR}/${NAME}_standalone_monitor_sha256.txt"
sha256sum -c "${CHECK_DIR}/${NAME}_freeze_manifest_sha256.txt"
if [[ "${ACQUISITION_RETRY}" == "true" ]]; then
  test -s "${CHECK_DIR}/${NAME}_retry_reason.txt"
  sha256sum -c "${CHECK_DIR}/${NAME}_retry_reason_sha256.txt"
fi
rosbag info "${BAG}" | tee "${CHECK_DIR}/${NAME}_manual_bag_info.txt"
```

### 12.3 核对核心话题

PF 和正式 run 的公共必录话题：

```bash
set -euo pipefail
missing_required=0
for topic in \
  /cmd_vel \
  /odom \
  /tf \
  /scout/global_path_fixed \
  /spmpc/status \
  /spmpc/solver_backend \
  /spmpc/solver_time_ms \
  /spmpc/controller_variant \
  /spmpc/debug/effective_config \
  /spmpc/debug/command_intervention \
  /spmpc/debug/warm_start \
  /spmpc/debug/warm_start_status \
  /spmpc/debug/predicted_horizon \
  /spmpc/debug/pre_solve_snapshot \
  /slosh/state \
  /slosh/height; do
  if ! grep -Fxq "${topic}" "${CHECK_DIR}/${NAME}_recorded_topics.txt"; then
    echo "MISSING_REQUIRED_TOPIC ${topic}" >&2
    missing_required=1
  fi
done
(( missing_required == 0 )) || exit 20

check_variant=""
if [[ -s "${CHECK_DIR}/${NAME}_one_click_meta.env" ]]; then
  check_variant="$(sed -n 's/^variant=//p' "${CHECK_DIR}/${NAME}_one_click_meta.env")"
else
  check_variant="${VARIANT:-}"
fi
case "${check_variant}" in
  B0|B_smooth|B_slosh) ;;
  *) echo "UNKNOWN_RECORDED_VARIANT ${check_variant}" >&2; exit 24 ;;
esac
if [[ "${check_variant}" == "B_slosh" ]]; then
  missing_slosh=0
  for topic in /spmpc/slosh_height /spmpc/slosh_horizon_summary /spmpc/debug/slosh_state; do
    if ! grep -Fxq "${topic}" "${CHECK_DIR}/${NAME}_recorded_topics.txt"; then
      echo "MISSING_REQUIRED_SLOSH_TOPIC ${topic}" >&2
      missing_slosh=1
    fi
  done
  (( missing_slosh == 0 )) || exit 23
fi

SUMMARY_SCRIPT="${SCOUT_WS}/src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py"
python3 "${SUMMARY_SCRIPT}" "${BAG}" --out-dir "${CHECK_DIR}"
python3 - "${CHECK_DIR}/${NAME}_summary.json" <<'PY'
import json
import pathlib
import sys

summary = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
warm = summary.get("metrics", {}).get("warm_start", {})
if warm.get("used_fallback_field_readable") is not True:
    raise SystemExit("used_fallback is not readable from /spmpc/debug/warm_start")
print(
    "WARM_START_FALLBACK_READABLE=PASS "
    f"count={warm.get('used_fallback_count')} frac={warm.get('used_fallback_frac')}"
)
PY
```

正式 run 额外核对 RGB；PF 必须确认这两个话题没有被录入：

```bash
if [[ "${REQUIRE_RGB:-false}" == "true" ]]; then
  missing_rgb=0
  for topic in /camera/color/image_raw /camera/color/camera_info; do
    if ! grep -Fxq "${topic}" "${CHECK_DIR}/${NAME}_recorded_topics.txt"; then
      echo "MISSING_REQUIRED_RGB_TOPIC ${topic}" >&2
      missing_rgb=1
    fi
  done
  (( missing_rgb == 0 )) || exit 21
else
  if grep -Eq '^/camera/color/(image_raw|image_raw/compressed)$' \
      "${CHECK_DIR}/${NAME}_recorded_topics.txt"; then
    echo "UNEXPECTED_RGB_TOPIC_IN_PILOT"
    exit 22
  fi
fi
```

正式 RGB 检查只确认 raw RGB 与 camera_info 可离线解码，不在现场伪造 `H_vis`。冻结 K6/RGB 脚本还必须验证：运动前至少 30 个有效帧且覆盖连续 2 s 静止段、30 帧全部早于 `t_move`、到达后 5 s 完整覆盖，以及 clipping、缺帧和固定 `tau_cal` 同步质量。`/spmpc/slosh_height` 的单位是 mm，预测 horizon 的 `h_modal` 是 m，分析时必须显式换算。

实时性也分开验收：solve-budget overrun 从 `/spmpc/solver_time_ms>33.33 ms` 计算；`/spmpc/debug/command_intervention` 的 bag 时间戳只生成 `observed command-intervention inter-arrival gap rate`。后者可能混入 recorder/传输丢样，不能称严格 control-cycle deadline miss。两者使用 freeze manifest 中已冻结的阈值、窗口和分母；v1.0 不要求当前未发布的 `/spmpc/debug/timing_budget`，也不作严格 deadline 保证。

### 12.4 检查 replay 消息

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

正式 run 若缺少 RGB、odom、TF、冻结路径或两个 replay 话题，本次按矩阵文件第 12 节判断为采集故障并保留原始文件。P3 全 15 次不要求 RGB，但都必须包含 odom、TF、冻结路径、standalone `/slosh/state`/`/slosh/height` 和两个 replay 话题；仅 W1/W2/W5（代码 variant=`B_slosh`）额外要求在线 `/spmpc/slosh_height`、`/spmpc/debug/slosh_state` 与 slosh horizon。B0/B_smooth 的在线零值或缺失值不用于候选排序。不得删除后静默重跑；若 solver/tracking/safety 本身失败，则作为方法失败进入 success-rate 分母。

---

## 13. 下一次 trial

一键 PF pilot：

1. 等脚本正常结束，确认 planner、recorder、path source 已清理且 `/cmd_vel` 为零；
2. 按矩阵顺序修改 `METHOD/PILOT_METHOD`、`BLOCK`、`RUN_LABEL`、`RUN_OUT_DIR` 和 `OPERATOR_NOTE`；
3. `PATH_SOURCE_MODE` 始终保持 `replay`，`PATH_FILE` 始终指向同一冻结 H0；
4. 遥控回起点并对齐航向；P3 固定 15 次必须确认 standalone monitor 正常并执行 `/slosh/reset`；
5. 按 PF 冻结的固定静置时间（及可选 monitor 门）确认液体静稳后再运行下一条一键命令；PF 不录 RGB，不能声称通过在线 `H_vis` 门；
6. 每个 run 生成新的独立 bag、metadata 和 log；
7. 不根据已看到的内部模型排序改变后续顺序或临时追加权重。

正式多终端 trial：

1. 按矩阵 v1.0 已冻结的唯一顺序修改 `METHOD` 和 `BLOCK`；
2. 重新执行第 5 节，生成新的 `RUN_LABEL` 和 `RUN_ENV`；
3. 每个 trial 重新启动 planner；
4. 路径仍从同一冻结 JSON replay；
5. monitor reset，等待冻结 `T_SETTLE`，并在按 Enter 前录满至少 2 s 静止 RGB；
6. recorder 产生新的独立 bag；
7. 不根据已看到的 RGB 排名修改后续顺序、权重或 `v_ref`。
