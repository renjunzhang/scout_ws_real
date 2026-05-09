# 2026-05-09 phase3 实物 GeoRef 修复与复录 SOP

## 1. 目的

本文档用于修复并复录 `20260508_phase3` 实物实验中暴露出的流程问题：

```text
1. RAW_REAL 流程正常，可作为 baseline。
2. GEOREF_FIXED_MILD_REAL 已能真正接管 mild candidate，是当前已验证接管的候选基线。
3. GEOREF_TUNED_MILD_REAL 没有接管，selector 仍选 original。
4. GEOREF_OSCRS_ACTIVE_MILD_REAL active 了，但 hard gate 失败，fallback original，并触发 safety_alarm。
```

当前目标是验证 OSCRS 方案，而不是停留在 fixed smoothing。FIXED 不是替代 OSCRS，
而是 OSCRS 论证链里的候选存在性 lemma：

```text
Claim: OSCRS works
  需要：
    (a) candidate set 里存在比 RAW 更低晃的路径      ← FIXED vs RAW 测这个
    (b) OSCRS rollout 与实车执行口径一致             ← prediction_v_max 与真实 v_max 对齐
    (c) OSCRS selector 能系统性挑出更优候选          ← OSCRS_ACTIVE 对比测这个
```

因此现场主线应在同一 session 内推进：

```text
RAW_REAL
GEOREF_FIXED_MILD_REAL
GEOREF_OSCRS_MEDIUM_ACTIVE_REAL
```

其中 `GEOREF_OSCRS_MEDIUM_ACTIVE_REAL` 的第一攻击向量是开放候选上限到 `medium`，
不是先放宽 `eta_lim_mm`。

---

## 1.5 现场快速索引（出问题先看这里）

```text
启动链路全部起齐？                           §6.0
定位是不是真的稳？                           §4
yaw 修复回归 smoke                          §3.1 末段
正式批量录之前要不要做三包 smoke？           §5.1（必做）
RAW 录完怎么验？                            §7.1
FIXED 录完怎么验？                          §7.2
OSCRS_MEDIUM 录完怎么判？                   §7.3
OSCRS smoke selected=mild 算通过吗？        §5.1 B 档 / §7.3 B 档（条件接受）
OSCRS smoke 现场过冲了怎么办？              §5.1 C 档 → §9.4 (调小 medium) → §9.5 (RGB 标定)
validate FAIL 提示"位置 …"是什么？          §7 开头
现场要 takeover OSCRS                       §6.3 OSCRS_MEDIUM smoke
FIXED 包 takeover=0 是不是 bug？            §7.2 注意段（不是）
有效性判据全套                              §11.3
最终能宣称什么？                            §11.4
什么动作绝对不能做                          §10
```

每个录包流程的最小路径：

```text
§6.0 起链路 → §4 定位 smoke → §5.1 三包 smoke + §3.1 yaw smoke
   ├── PASS: §6.1/§6.2/§6.3 正式 RAW×3 + FIXED×3 + OSCRS×3，每包 §7 验收
   └── FAIL: 按 validate 输出的"位置 …"或本节速查改，不要硬录

OSCRS 是目标主线；若 §6.3 takeover smoke 失败，先不做 OSCRS 正式统计。
```

---

## 2. phase3 现象汇总

目录：

```text
/data/a/slosh_bags/real/20260508_phase3
```

行为验收结果：

```text
RAW_REAL run01/run02:
  PASS
  无 /anti_slosh_path/* topic，符合 baseline 口径。

GEOREF_FIXED_MILD_REAL run01/run02:
  PASS
  selected=mild
  safety_alarm=0
  /scout/global_path_anti_slosh 确实为 mild path。

GEOREF_TUNED_MILD_REAL run01/run02:
  FAIL
  selected=original
  mild accepted 但 geometry score 高于 original。
  这两包物理上约等于 RAW，不进入 GeoRef 有效性统计。

GEOREF_OSCRS_ACTIVE_MILD_REAL run01/run02:
  FAIL
  active=1
  fb=2
  selected=original
  safety_alarm=1
  表示几何候选存在，但 OSCRS slosh hard gate 全失败。

  phase3 report 中的 rollout sH（峰值液面，单位 mm）：
    run01: original=32, mild=27, mid=15, medium=6, strong=7（mild/original 均 > eta_lim=25）
    run02: original=60, mild=31（更糟）
  说明：当前路径与 SAFE mild 组合的物理 slosh 就超过当前 eta_lim_mm=25,
  不是参数小调能救的；medium/mid 物理上能过 hard gate 但被 max_candidate_level=mild
  cap 掉，进不了 OSCRS rollout。
```

关键判断：

```text
轨迹跟踪流程没有整体性 bug。
post-processor、candidate_report、metrics、safety_alarm 都按协议工作。

OSCRS fb=2 的物理根因有两层并存：
  (1) max_candidate_level=mild 把 mid/medium 挡在 rollout 外
      （mid sH=15, medium sH=6, 都 << eta_lim=25, 本来能过 hard gate）；
  (2) 候选集仅剩的非 original 是 mild, 而 mild sH=27 > 25, 自身也过不了 hard gate。

第一攻击向量是放 max_candidate_level → medium, 把 (1) 解开,
让物理 sH 远低于默认门的 mid/medium 进入 rollout, 而不是去放宽 eta_lim 救 mild。
TUNED 的 selector 退化是另一回事：SAFE mild 太弱, geometry score 倾向 original。
```

---

## 3. 已修复的代码问题

### 3.1 终点 yaw 被 post-processor 改坏

旧问题：

```text
anti_slosh_path_post_processor.py:path_to_msg()
会把非 original path 的每个 pose.orientation 按候选路径切线重算。

PathHandler::computeGoalInfoLocked()
优先读取 global_path 最后一个 pose.orientation 作为 goal_yaw。

结果：
post-processor 把原始 goal yaw 改成候选末段切线，
terminal recovery 可能对齐错误 yaw，表现为终点附近回头/过冲。
```

修复：

```text
selected/debug candidate path 发布时保留 raw MBF path 的首尾 orientation；
中间点仍按候选路径切线生成 yaw。
```

代码落点（可 grep 核实）：

```text
scripts/anti_slosh_path_post_processor.py:776-777   注入 raw_msg.poses[0]/[-1].pose.orientation
scripts/anti_slosh_path_post_processor.py:100-104   path_to_msg 用注入值覆盖首尾 candidate 切线 yaw
```

复录前编译检查：

```bash
cd /home/geist/scout_ws
source devel/setup.bash
python3 -m py_compile src/scout_apps/control/scout_local_planner/scripts/anti_slosh_path_post_processor.py
```

录完第一包带 post-processor 的包（按 §5.1 顺序即 FIXED smoke）立即跑 yaw 回归 smoke——RAW 不走 path_to_msg，无法验证此修复：

```bash
python3 - <<'PY'
import rosbag, sys
bag = "<FIXED_RUN01_BAG>"
o, a = None, None
with rosbag.Bag(bag) as b:
    for t,m,_ in b.read_messages(["/scout/global_path","/scout/global_path_anti_slosh"]):
        if t == "/scout/global_path" and o is None: o = m.poses[-1].pose.orientation
        if t == "/scout/global_path_anti_slosh" and a is None: a = m.poses[-1].pose.orientation
def eq(p, q): return (p.x,p.y,p.z,p.w) == (q.x,q.y,q.z,q.w)
print("orig last quat:", o); print("anti last quat:", a)
print("EQUAL =", eq(o,a))
sys.exit(0 if eq(o,a) else 1)
PY
```

`EQUAL = True` 才说明 yaw 修复未回归。任一分量不等：停止录后续包，回到 §3.1 排查。

---

## 4. 复录前定位 smoke

实物验证前先确认定位，不通过则不要录防晃实验。

量化判据：

```text
1. RViz 中 /scan_front 与地图边界基本重合（视觉判断，无大段错位）。
2. 静止 10 s：tf_echo map base_link
   - xy 漂移幅度 < 0.05 m
   - yaw 漂移幅度 < 1°
3. 低速直行 10 s：cmd_vel.linear.x = 0.2 m/s
   - 行驶距离与 odom 累计差 < 0.10 m
4. 低速原地转向 10 s：cmd_vel.angular.z = 0.3 rad/s
   - 角度与 odom 累计差 < 3°
```

可用命令：

```bash
# 静止漂移
rosrun tf tf_echo map base_link
# 协方差（amcl 时）
rostopic echo -n 1 /amcl_pose | grep -A6 covariance
# cov[0,0] (x) < 0.05；cov[5,5] (yaw) < 0.005 视为可信
```

若任一项不通过：

```text
不要调 GeoRef/OSCRS。
先处理 localization / TF / 初始位姿 / 地图匹配问题。
该状态下录到的 bag 不进入有效性统计。
```

---

## 5. OSCRS 主线复录矩阵

### 5.1 批量前 smoke（必做）

同一定位、同一终点、同一 session 下先录：

```text
RAW_REAL smoke x1
GEOREF_FIXED_MILD_REAL smoke x1
GEOREF_OSCRS_MEDIUM_ACTIVE_REAL smoke x1
```

三包都必须通过 §7 行为验收。OSCRS smoke 按结果分三档决策：

**A. 理想样本 — 进入正式批量（§5.2）**

```text
selected ∈ {mid, medium}
active=1
fb=0
takeover=1
safety_alarm=0
现场无明显终点过冲/回头
```

medium/mid 物理 sH 远低于默认 25mm 门，takeover 的物理意义最强，论文证据链最干净。

**B. 可接受样本 — 进入正式批量但论文要标注**

```text
selected=mild
active=1
fb=0
takeover ∈ {0,1}
safety_alarm=0
现场无过冲
```

含义：yaw 修复后 mild sH 实测可能低于历史 27mm，过了 hard gate；OSCRS 选了 mild 而非 mid/medium。
处理：仍可进批量录，但论文中明确"OSCRS 选择倾向于 mild，medium/mid 收益空间未被利用"，
不能写"OSCRS 主要利用 medium 候选"。

**C. 不可用样本 — 停止 OSCRS_MEDIUM 批量**

任一发生：

```text
- 现场看到明显终点过冲/回头：
    yaw 修复后 medium 仍不适合实物。
    回退备选见 §9 fallback 顺序：先调小 medium 平滑参数（medium_iters/medium_gain/medium_max_drift）
    重 smoke；仍过冲则 §9.4 RGB 经验标定 eta_lim_mm + 退回 max_candidate_level=mild。
- safety_alarm=1 / fb ∈ {2,3}：
    按 §7.3 分级排查；不要先放 eta_lim_mm。
- 持续 selected=original：
    OSCRS hard gate 全 reject；同 §7.3 fb=2 排查路径。
```

### 5.2 smoke 通过后的正式批量

smoke 通过后录：

```text
RAW_REAL x3
GEOREF_FIXED_MILD_REAL x3
GEOREF_OSCRS_MEDIUM_ACTIVE_REAL x3
```

三组对比的论文含义：

```text
OSCRS_MEDIUM vs RAW:
  总体抑晃证据。

FIXED_MILD vs RAW:
  candidate generator 确实能提供低晃几何候选的证据。

OSCRS_MEDIUM vs FIXED_MILD:
  selector 增量证据；但若 OSCRS 稳定选择 medium，则这不是严格的
  "same candidate strength" 对照。
```

严格 selector ablation：

```text
若 OSCRS_MEDIUM 连续选择 medium，且 medium smoke 无过冲，
建议额外录 GEOREF_FIXED_MEDIUM_REAL x1~3。

目的：
  区分 "OSCRS 选择器有效" 与 "medium smoothing 本身强于 mild"。
若时间不够，至少在论文中明确 OSCRS_MEDIUM vs FIXED_MILD 是工程主对照，
不是完全隔离 candidate strength 的纯 selector ablation。
```

---

## 6. 启动命令

### 6.0 启动前置链路（必做）

启动 §6.1/§6.2/§6.3 前，先按 `docs/重要文档/20260506有效性验证方案.md §5.1-5.6` 起完：

```text
5.1  CAN 与底盘     scout_mini_robot_base.launch
5.2  激光雷达       nanoscan3_front.launch  → rostopic hz /scan_front 应稳定
5.3  定位           scout_nanoscan3_amcl.launch  或  scout_nanoscan3_cartographer_localization.launch
                    → tf_echo map base_link 不漂；过 §4 定位 smoke
5.4  IMU            scout_imu_with_tf.launch  → rostopic hz /imu/data 应稳定
5.5  RealSense RGB  rs_camera.launch align_depth:=true（如需视觉真值）
5.6  MBF 全局规划   mbf_global.launch  → rostopic info /scout/goal、/scout/global_path 存在
```

任一上游链路未起，下面 6.1/6.2/6.3 都不会得到正确数据，不要往下走。

所有 §6.x 终端先执行：

```bash
source /home/geist/scout_ws/devel/setup.bash
```

### 6.1 RAW_REAL

RAW 不启动 `anti_slosh_path_post_processor.launch`。

终端 A：启动 MPC，订阅原始全局路径。

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false
```

终端 B：发布同一终点。

```bash
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x <GOAL_X> \
  --y <GOAL_Y> \
  --yaw <GOAL_YAW> \
  --repeat-count 30 \
  --repeat-rate 5 \
  --wait-subscriber-timeout 20
```

终端 C：录包。

```bash
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
CONDITION=RAW_REAL RUN_ID=run01 ./scripts/record_slosh_experiment.sh 0
```

### 6.2 GEOREF_FIXED_MILD_REAL

终端 A：启动 post-processor，输入原始全局路径，输出 anti-slosh path。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  fixed_candidate_name:=mild \
  max_candidate_level:=mild \
  ay_ratio_limit:=3.0 \
  collision_threshold:=90 \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  oscrs_shadow_enable:=false \
  oscrs_active_enable:=false
```

口径约束：`prediction_v_max` 必须等于 MPC 真实 `cmd_vel.linear.x_max`（当前 2.0）。改 MPC v_max 必须同步本参数；任一改动单边都会让 OSCRS rollout 与实车口径脱节。

终端 B：启动 MPC，订阅 post-processor 输出路径。

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_anti_slosh \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false
```

终端 C：发布同一终点。

```bash
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x <GOAL_X> \
  --y <GOAL_Y> \
  --yaw <GOAL_YAW> \
  --repeat-count 30 \
  --repeat-rate 5 \
  --wait-subscriber-timeout 20
```

终端 D：录包。

```bash
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
CONDITION=GEOREF_FIXED_MILD_REAL RUN_ID=run01 ./scripts/record_slosh_experiment.sh 0
```

### 6.3 GEOREF_OSCRS_MEDIUM_ACTIVE_REAL

该模式是 OSCRS 主线 smoke/正式批量入口。

第一攻击向量：

```text
开放候选上限到 medium；
保持 eta_lim_mm=25 与 residual_ratio=0.2 默认物理门；
prediction_v_max=2.0 必须与实车 v_max 对齐。
```

理由：

```text
phase3 report 中 medium/mid 的预测 sH 已低于 eta_lim=25mm。
之前 OSCRS fb=2 的直接原因是 max_candidate_level=mild 把它们挡掉，
不是必须放宽 eta_lim。
```

终端 A：启动 OSCRS active post-processor。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  fixed_candidate_name:= \
  max_candidate_level:=medium \
  ay_ratio_limit:=3.0 \
  collision_threshold:=90 \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  oscrs_shadow_enable:=true \
  oscrs_active_enable:=true \
  oscrs_eta_lim_mm:=25 \
  oscrs_residual_ratio:=0.2
```

终端 B：MPC，仍只订阅 `/scout/global_path_anti_slosh`。

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_anti_slosh \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false
```

终端 C：发布同一终点。

```bash
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x <GOAL_X> \
  --y <GOAL_Y> \
  --yaw <GOAL_YAW> \
  --repeat-count 30 \
  --repeat-rate 5 \
  --wait-subscriber-timeout 20
```

终端 D：录包。

```bash
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
CONDITION=GEOREF_OSCRS_MEDIUM_ACTIVE_REAL RUN_ID=smoke01 ./scripts/record_slosh_experiment.sh 0
```

---

## 7. 每包录完立即验收

`validate_georef_oscrs_bag.py` 在 FAIL 时会直接列出"调哪个文件哪个 arg / yaml key + 默认值"。按建议中的 `位置 src/.../*.launch <arg name="…">` 直接定位修改，不要再去翻代码。

### 7.1 RAW_REAL

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
  <RAW_BAG> \
  --mode raw
```

必须满足：

```text
VERDICT=PASS
reports=0
global_path_anti_slosh=0
safety_alarm=0
```

### 7.2 GEOREF_FIXED_MILD_REAL

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
  <FIXED_MILD_BAG> \
  --mode fixed \
  --require-non-original
```

必须满足：

```text
VERDICT=PASS
selected=mild
safety_alarm=0
global_path_anti_slosh=1
```

注意：

```text
FIXED 模式下 takeover 字段恒为 0（OSCRS 未 active），不要把 takeover=0 当 FIXED 失败判据。
takeover 只对 OSCRS_ACTIVE 模式有意义。
```

人工检查：

```text
1. 现场没有明显终点过冲/回头；
2. /scan_front 与地图边界没有明显漂；
3. 最终状态最好是 REACHED，而不是 ERROR。
```

若 `VERDICT=PASS` 但现场终点明显回头：

```text
该包不进正式统计。
优先检查定位漂移与 terminal debug（先跑 §3.1 yaw 回归 smoke），
而不是调 slosh 参数。
```

### 7.3 GEOREF_OSCRS_MEDIUM_ACTIVE_REAL

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
  <OSCRS_MEDIUM_BAG> \
  --mode oscrs \
  --require-non-original
```

通路条件分级（与 §5.1 三档决策口径一致）：

**理想 (A 档)**：

```text
selected ∈ {mid, medium}
active=1, fb=0, takeover=1, safety_alarm=0
```

**可接受 (B 档)**：

```text
selected=mild
active=1, fb=0, safety_alarm=0
```

含义：yaw 修复使 mild sH 低于 hard gate；OSCRS 真接管但选择倾向 mild。
处理：检查 candidate_report 中 medium/mid 是否被 collision/ay/residual gate 拒绝。
若是 collision/ay 等几何 gate, 修后重 smoke；若是 residual, 先记录数据再讨论。

**不可用 (C 档)**：

仍 `fb=2` 或 `safety_alarm=1` 或 `selected=original`：

```text
不要先放 eta_lim_mm。
先看 candidate_report：
  - 若 medium/mid 被 level 以外的 gate 拒绝，修对应 gate；
  - 若 medium/mid 通过几何 gate 但 residual 失败，才讨论 residual_ratio；
  - 若所有非 original 都被 collision 拒绝，先修定位/costmap。
```

---

## 8. Fixed Mild 推荐启动参数

在 open 实物场景中，当前建议保持 SAFE mild：

```text
fixed_candidate_name:=mild
max_candidate_level:=mild
ay_ratio_limit:=3.0
collision_threshold:=90
```

如果再次出现：

```text
mild:accepted=0,reason=collision:idx=0:cost=68
```

处理顺序：

```text
1. 确认 costmap 与地图/scan 对齐；
2. 试 collision_threshold:=100；
3. 仅在开阔场地通路 smoke 中可临时 enable_collision_check:=false；
4. 正式安全实验不要关闭 collision check。
```

---

## 9. OSCRS_MEDIUM 参数口径

```text
GEOREF_OSCRS_MEDIUM_ACTIVE_REAL
oscrs_eta_lim_mm:=25
oscrs_residual_ratio:=0.2
max_candidate_level:=medium
ay_ratio_limit:=3.0
collision_threshold:=90
prediction_v_max:=2.0
```

参数原则：

```text
1. 不先放 eta_lim_mm。
   因为 phase3 report 已显示 medium/mid 预测可过 25mm hard gate。

2. 不降低 prediction_v_max。
   prediction_v_max 必须等于真实 MPC/底盘速度上限，否则 hard gate 是假通过。

3. max_candidate_level:=medium 是当前第一攻击向量。
   若 medium yaw 修复后仍过冲，一包看到就停。
```

fallback 顺序：

```text
1. medium/mid 因 collision 拒绝：
   先看定位/costmap；必要时 open 场地 smoke 临时 collision_threshold:=100。

2. medium/mid 因 ay 拒绝：
   先看 odom_ay_p95；若没有实际 ay 反弹，可把 ay_ratio_limit 从 3.0 小幅放到 3.5。

3. medium/mid 因 residual 拒绝：
   记录 candidate_report 后再讨论 residual_ratio，不要盲调。

4. medium/mid 通过 gate 但现场看到终点过冲（§5.1 C 档）：

   先调小 medium 候选强度，让它仍区别于 mild 但更接近 mild 的几何特性：
       medium_iters       由 default 适度下调
       medium_gain        小幅下调
       medium_max_drift   小幅下调
   再录 1 包 OSCRS_MEDIUM smoke。

   仍过冲 → 转 §9.5 RGB 经验标定。

5. RGB 经验标定 eta_lim_mm（终极 fallback）：

   - 容器加注到目标液位；
   - 跑 RAW 一次激进 path（手动加速到 v_max）；
   - 用 RealSense RGB 离线提取真实 η peak（脚本：
     src/scout_apps/sensors/realsense_liquid_measurement/scripts/.../*rgb_heights.csv）；
   - 取多次实验的最大不溢出 η 作为经验上限 eta_lim_mm；
   - 此时回退 max_candidate_level:=mild，用经验 eta_lim 让 mild 通过 hard gate。
   - 论文中明确这是经验标定门，不是理论默认。
```

---

## 10. 不建议的操作

不要做：

```text
1. 不要为了让 TUNED 接管而改 selector 为“禁止选 original”。
   这会把 TUNED 变成 FIXED 的变体，丢失 selector baseline 语义。

2. 不要把放宽 eta_lim_mm 当作 OSCRS 第一攻击向量。
   当前已知 medium/mid 预测能过默认 25mm hard gate，先开放 max_candidate_level。

3. 不要通过降低 prediction_v_max 来让 OSCRS hard gate 通过。
   如果真实 MPC 仍按原速度执行，而 rollout 用更低速度预测，就是模型自欺骗。

4. 不要打开 slosh_speed_governor 来修终点过冲。
   speed governor 是残余晃动/横向激励限速器，不是 terminal 制动模块。

5. 不要在定位漂移时录正式防晃实验。
   map->base_link 错了以后，终点距离、bearing、goal_yaw_err 和 tracking 都不可信。
```

---

## 11. 正式有效性判断口径

### 11.1 单包准入条件（门口）

只有同时满足以下条件的 bag 才能进入统计：

```text
1. validate VERDICT=PASS；
2. RAW 没有 anti_slosh topic；
3. FIXED 包 selected=mild；
4. OSCRS_MEDIUM 包 selected ∈ {mid, medium} 且 active=1/fb=0/takeover=1；
5. safety_alarm=0；
6. 通过 §4 定位 smoke 量化判据（xy<0.05m, yaw<1°）；
7. 终点无明显过冲/回头（对 FIXED 第一包必须跑 §3.1 yaw 回归 smoke EQUAL=True）；
8. solve_success_ratio >= 0.97。
```

### 11.2 数据量门槛

```text
第一阶段：RAW x3 + FIXED_MILD x3 + OSCRS_MEDIUM x3
- 若 3 包内 h_p95 方差 / 均值 < 0.3，3 包足够；
- 若有任一包数据可疑（VERDICT=PASS 但人工怀疑），必须升到 5 包均值；
- 任一包 §11.1 失败，直接重录该 condition 一包，不顶替。
```

### 11.3 效果判据（与 `20260506有效性验证方案.md §3` 对齐）

主效果：`GEOREF_OSCRS_MEDIUM_ACTIVE_REAL` 相对 `RAW_REAL`，必须**全部**满足：

```text
/slosh/height h_p95           下降 >= 10%
modal_energy_rms              下降 >= 10%
eta_dot_rms                   不上升
tracking_time                 <= RAW * 1.15
ay_p95                        不上升
solve_success_ratio           >= 0.97
无碰撞、无人工接管、无定位丢失
```

辅助消融：

```text
GEOREF_FIXED_MILD_REAL vs RAW_REAL:
  证明 candidate generator 里存在可执行的低晃候选。

GEOREF_OSCRS_MEDIUM_ACTIVE_REAL vs GEOREF_FIXED_MILD_REAL:
  证明 selector 相对 fixed mild 的增量。

若 OSCRS 稳定选择 medium，严格 selector 论文表建议补：
  GEOREF_FIXED_MEDIUM_REAL x1~3
```

抽包指标命令：

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py <BAG>
```

### 11.4 可宣称结论

`§11.1 ∩ §11.2 ∩ §11.3` 全部满足时：

```text
"OSCRS 在 Scout Mini 实物上能够从 GeoRef 候选集合中选择低晃参考，并相对 RAW 降低液面晃动。"
```

仅 §11.1 + §11.2 满足、§11.3 部分项不达标：

```text
"OSCRS takeover 行为成立（selector 在实物上能正确选择并发布候选），
 但相对 RAW 抑晃幅度不达 §11.3 阈值。视为 inconclusive 而非 negative：
 可能成因包括路径段不够长 / 起始液位过低 / 容器特性 / IMU 估计偏差等，
 论文中应明确未达阈值的具体维度（h_p95 / energy / eta_dot），
 并保留路径设计 + RGB 真值标定后再次评估的空间。"
```

### 11.5 OSCRS 的地位

```text
OSCRS 是当前目标主线；
FIXED_MILD 是候选生成器 lemma，不是最终替代方案；
OSCRS_MEDIUM takeover smoke 通过后才进入正式有效性矩阵。
```
