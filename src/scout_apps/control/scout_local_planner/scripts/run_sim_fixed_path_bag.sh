#!/usr/bin/env bash
# ============================================================================
# run_sim_fixed_path_bag.sh
# ============================================================================
#
# 功能：
#   一次性跑一个仿真 trial，并自动录 bag。脚本会按 CONDITION 配置 MPC /
#   固定路径/模板路径，并根据 CONDITION 配置 MPC 参数后自动录 bag。
#
# 这个脚本功能很多，最容易混淆的是：
#
#   固定 goal  ≠ 固定轨迹
#
#   1. 固定 goal / MBF 全局规划路径
#      - 你固定的是目标点 `/scout/goal`。
#      - MBF / global planner 根据当前地图、起点、costmap 实时重新生成
#        `/scout/global_path`。
#      - 适合验证全局规划器输出是否能被 MPC 跟踪。
#      - 同一个 goal 不保证每次 global path 字节级完全一致；这是真实在线规划。
#
#   2. 固定轨迹 / JSON replay
#      - 你固定的是一条已经保存好的 path JSON。
#      - 脚本直接把这条 path 发布到 `/scout/global_path_fixed` 或指定话题。
#      - 不经过 MBF 重新规划。
#      - 适合离线消融、重复跑 P2/P3、控制变量最干净。
#
# PATH_MODE 三种模式：
#
#   PATH_MODE=replay
#     固定轨迹 replay。默认模式。
#     输入: PATH_FILE 或 FIXED_PATH_DIR/PATH_ID.json
#     常见用途: P2_s_curve / P3_mixed 等固定路径复现实验。
#
#   PATH_MODE=global_goal
#     固定终点 goal。脚本发布 `/scout/goal`，让 MBF 生成 `/scout/global_path`。
#     输入: TEMPLATE_GOAL_X/Y/QZ/QW 或 SCENARIO。
#     常见用途: open 场地同终点普通 MPC smoke。
#
#   PATH_MODE=template_goal
#     从当前位姿到 goal 生成模板轨迹，如 s_curve / mixed / sharp_turn。
#     输入: TEMPLATE_NAME + goal。
#     注意: 这不是 MBF 全局路径，而是模板生成器输出的固定形状 path。
#
# SCENARIO 快捷模式：
#   如果设置 SCENARIO=<name>，脚本从 config/scenarios.yaml 读取 goal，
#   自动切到 PATH_MODE=global_goal。
#   当前脚本只消费 scenarios.yaml 里的 goal 字段；start/map/notes 只是 checklist。
#
# CONDITION 常用值：
#   CUSTOM / NOM / FAS_* 等
#     固定路径、内部 MPC cost 消融或历史 smoke 入口。
#
#   RETIME_METHOD=toppra / ruckig / biagiotti
#     在 PATH_MODE=template_goal/replay 下为固定路径生成 external_speed_profile_csv，
#     用于 TOPPRA-style / Ruckig-style / Biagiotti-style baseline 仿真 smoke。
#
# 常用命令：固定 goal，MBF 全局规划，普通 MPC
#   source /home/a/scout_ws/devel/setup.bash
#   PATH_MODE=global_goal \
#   PATH_ID=open_custom_goal \
#   CONDITION=CUSTOM \
#   RUN_ID=active01 \
#   START_DELAY=10 \
#   RECORD_DURATION=0 \
#   TEMPLATE_GOAL_X=-3.014343023300171 \
#   TEMPLATE_GOAL_Y=2.987114429473877 \
#   TEMPLATE_GOAL_QZ=0.9999403278718936 \
#   TEMPLATE_GOAL_QW=0.010924316704027428 \
#   rosrun scout_local_planner run_sim_fixed_path_bag.sh
#
# 常用命令：固定轨迹 replay，P2_s_curve
#   source /home/a/scout_ws/devel/setup.bash
#   PATH_MODE=replay \
#   PATH_ID=P2_s_curve \
#   CONDITION=CUSTOM \
#   RUN_ID=active01 \
#   START_DELAY=10 \
#   RECORD_DURATION=0 \
#   rosrun scout_local_planner run_sim_fixed_path_bag.sh
#
# 常用命令：从 scenarios.yaml 读取 goal
#   source /home/a/scout_ws/devel/setup.bash
#   SCENARIO=open_user_goal \
#   CONDITION=CUSTOM \
#   RUN_ID=active01 \
#   START_DELAY=10 \
#   RECORD_DURATION=0 \
#   rosrun scout_local_planner run_sim_fixed_path_bag.sh
#
# 重要参数：
#   RECORD_DURATION=0
#     手动 Ctrl+C 停止录包；非 0 时自动停止。
#
#   TEMPLATE_GOAL_REPEAT_COUNT / TEMPLATE_GOAL_REPEAT_RATE
#     固定 goal 发布次数和频率。默认多发几次，避免 planner 尚未订阅时丢 goal。
#
# 输出：
#   默认 bag:
#     /data/a/slosh_bags/sim/YYYYMMDD/YYYYMMDD_<PATH_ID>_<CONDITION>_run<RUN_ID>_<HHMMSS>.bag
#
# 注意：
#   - 如果你传了 TEMPLATE_GOAL_X/Y/QZ/QW 但没显式设置 PATH_MODE，
#     脚本会自动切到 PATH_MODE=global_goal，避免误跑默认 P2_s_curve replay。
#   - 如果你要测试“真实在线全局规划 + post-processor”，用 global_goal 或 SCENARIO。
#   - 如果你要最干净地重复一条路径，用 replay。
# ============================================================================

set -euo pipefail

PATH_FILE_WAS_SET="${PATH_FILE+x}"
BAG_NAME_WAS_SET="${BAG_NAME+x}"
PATH_MODE_WAS_SET="${PATH_MODE+x}"
TEMPLATE_GOAL_X_WAS_SET="${TEMPLATE_GOAL_X+x}"
TEMPLATE_GOAL_Y_WAS_SET="${TEMPLATE_GOAL_Y+x}"
TEMPLATE_GOAL_QZ_WAS_SET="${TEMPLATE_GOAL_QZ+x}"
TEMPLATE_GOAL_QW_WAS_SET="${TEMPLATE_GOAL_QW+x}"

PATH_ID="${PATH_ID:-P2_s_curve}"
CONDITION="${CONDITION:-FAS_Q5_DOT}"
RUN_ID="${RUN_ID:-01}"
SCENARIO="${SCENARIO:-}"
SCENARIOS_FILE="${SCENARIOS_FILE:-/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/scenarios.yaml}"

FIXED_PATH_DIR="${FIXED_PATH_DIR:-/data/a/fixed_paths/sim}"
PATH_FILE="${PATH_FILE:-${FIXED_PATH_DIR}/${PATH_ID}.json}"
GLOBAL_PATH_TOPIC="${GLOBAL_PATH_TOPIC:-}"
PATH_SOURCE_OUTPUT_TOPIC="${PATH_SOURCE_OUTPUT_TOPIC:-}"
PATH_MODE="${PATH_MODE:-replay}"  # replay / template_goal / global_goal
DEFAULT_GLOBAL_PATH_TOPIC=""

TEMPLATE_NAME="${TEMPLATE_NAME:-}"
TEMPLATE_START_HEADING="${TEMPLATE_START_HEADING:-current}"
TEMPLATE_GOAL_TOPIC="${TEMPLATE_GOAL_TOPIC:-/scout/goal}"
TEMPLATE_GOAL_FRAME="${TEMPLATE_GOAL_FRAME:-map}"
TEMPLATE_GOAL_X="${TEMPLATE_GOAL_X:--3.6119120121002197}"
TEMPLATE_GOAL_Y="${TEMPLATE_GOAL_Y:-3.955589771270752}"
TEMPLATE_GOAL_Z="${TEMPLATE_GOAL_Z:-0.0}"
TEMPLATE_GOAL_QX="${TEMPLATE_GOAL_QX:-0.0}"
TEMPLATE_GOAL_QY="${TEMPLATE_GOAL_QY:-0.0}"
TEMPLATE_GOAL_QZ="${TEMPLATE_GOAL_QZ:-0.9992705515413127}"
TEMPLATE_GOAL_QW="${TEMPLATE_GOAL_QW:-0.03818854307669733}"
TEMPLATE_GOAL_REPEAT_COUNT="${TEMPLATE_GOAL_REPEAT_COUNT:-30}"
TEMPLATE_GOAL_REPEAT_RATE="${TEMPLATE_GOAL_REPEAT_RATE:-5}"
TEMPLATE_GOAL_WAIT_SUBSCRIBER_TIMEOUT="${TEMPLATE_GOAL_WAIT_SUBSCRIBER_TIMEOUT:-20}"
TEMPLATE_GENERATOR_WARMUP_S="${TEMPLATE_GENERATOR_WARMUP_S:-1}"
MPC_WARMUP_S="${MPC_WARMUP_S:-1}"

if [[ -z "${PATH_MODE_WAS_SET}" && -z "${SCENARIO}" &&
      ( -n "${TEMPLATE_GOAL_X_WAS_SET}" || -n "${TEMPLATE_GOAL_Y_WAS_SET}" ||
        -n "${TEMPLATE_GOAL_QZ_WAS_SET}" || -n "${TEMPLATE_GOAL_QW_WAS_SET}" ) ]]; then
    PATH_MODE="global_goal"
    echo "[run_sim_fixed_path_bag] PATH_MODE not set but TEMPLATE_GOAL_* was provided; using PATH_MODE=global_goal"
fi

BAG_DATE="${BAG_DATE:-$(date +%Y%m%d)}"
BAG_TIME="${BAG_TIME:-$(date +%H%M%S)}"
BAG_DIR="${BAG_DIR:-/data/a/slosh_bags/sim/${BAG_DATE}}"
BAG_NAME="${BAG_NAME:-${BAG_DATE}_${PATH_ID}_${CONDITION}_run${RUN_ID}_${BAG_TIME}}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

START_DELAY="${START_DELAY:-10}"
RECORD_DURATION="${RECORD_DURATION:-0}"
RECORD_WARMUP_S="${RECORD_WARMUP_S:-2}"
PATH_WARMUP_S="${PATH_WARMUP_S:-2}"

APPROACH_START_ENABLE="${APPROACH_START_ENABLE:-false}"
APPROACH_GOAL_TOPIC="${APPROACH_GOAL_TOPIC:-/scout/goal}"
APPROACH_GLOBAL_PATH_TOPIC="${APPROACH_GLOBAL_PATH_TOPIC:-/scout/global_path}"
APPROACH_START_TIMEOUT="${APPROACH_START_TIMEOUT:-90}"
APPROACH_START_HOLD_S="${APPROACH_START_HOLD_S:-0.5}"
APPROACH_START_POS_TOL="${APPROACH_START_POS_TOL:-0.25}"
APPROACH_START_YAW_TOL="${APPROACH_START_YAW_TOL:-3.20}"

START_GATE="${START_GATE:-false}"
START_POS_TOL="${START_POS_TOL:-0.10}"
START_YAW_TOL="${START_YAW_TOL:-0.15}"
PATH_PUBLISH_ONCE_KEEPALIVE="${PATH_PUBLISH_ONCE_KEEPALIVE:-true}"

Q_SLOSH="${Q_SLOSH:-}"
Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-}"
MPC_Q_V="${MPC_Q_V:-8.0}"
MPC_R_A="${MPC_R_A:-}"
MPC_R_DA="${MPC_R_DA:-}"
MPC_CMD_VEL_LEAD_TIME="${MPC_CMD_VEL_LEAD_TIME:-}"
TERMINAL_FACTOR_SLOSH_ETA="${TERMINAL_FACTOR_SLOSH_ETA:-0.0}"
TERMINAL_FACTOR_SLOSH_ETA_DOT="${TERMINAL_FACTOR_SLOSH_ETA_DOT:-0.0}"
VEHICLE_V_MAX="${VEHICLE_V_MAX:-}"

EXPERIMENT_GROUP="${EXPERIMENT_GROUP:-LEGACY}"
CONTROLLER_VARIANT="${CONTROLLER_VARIANT:-mpc}"
EXTERNAL_PROFILE_MODE="${EXTERNAL_PROFILE_MODE:-}"
RETIME_METHOD="${RETIME_METHOD:-none}"  # none / toppra / ruckig / biagiotti
EXTERNAL_SPEED_PROFILE_CSV="${EXTERNAL_SPEED_PROFILE_CSV:-}"
RETIME_PROFILE_DIR="${RETIME_PROFILE_DIR:-${FIXED_PATH_DIR}/baseline_profiles}"
RETIME_V_MAX="${RETIME_V_MAX:-0.80}"
RETIME_A_MAX="${RETIME_A_MAX:-0.60}"
RETIME_DECEL_MAX="${RETIME_DECEL_MAX:-0.80}"
RETIME_J_MAX="${RETIME_J_MAX:-1.50}"
RETIME_DS="${RETIME_DS:-0.02}"
RETIME_DELTA_TIME="${RETIME_DELTA_TIME:-0.02}"
BIAGIOTTI_OMEGA_N="${BIAGIOTTI_OMEGA_N:-5.0}"
BIAGIOTTI_DAMPING_RATIO="${BIAGIOTTI_DAMPING_RATIO:-0.05}"
EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE="${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE:-false}"
EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT="${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT:-${RETIME_A_MAX}}"
EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT="${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT:-${RETIME_DECEL_MAX}}"
EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT="${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT:-0.0}"

if [[ -n "${SCENARIO}" ]]; then
    if [[ ! -f "${SCENARIOS_FILE}" ]]; then
        echo "[run_sim_fixed_path_bag] ERROR: SCENARIOS_FILE not found: ${SCENARIOS_FILE}" >&2
        exit 2
    fi
    scenario_env="$(
        python3 - "${SCENARIOS_FILE}" "${SCENARIO}" <<'PY'
import math
import sys

import yaml

path, wanted = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}
for item in data.get("scenarios", []) or []:
    if item.get("name") == wanted:
        goal = item.get("goal", {}) or {}
        yaw = float(goal.get("yaw", 0.0))
        print(f"PATH_ID={wanted}")
        print("PATH_MODE=global_goal")
        print(f"TEMPLATE_GOAL_X={float(goal.get('x', 0.0))}")
        print(f"TEMPLATE_GOAL_Y={float(goal.get('y', 0.0))}")
        print(f"TEMPLATE_GOAL_Z=0.0")
        print(f"TEMPLATE_GOAL_QX=0.0")
        print(f"TEMPLATE_GOAL_QY=0.0")
        print(f"TEMPLATE_GOAL_QZ={math.sin(0.5 * yaw)}")
        print(f"TEMPLATE_GOAL_QW={math.cos(0.5 * yaw)}")
        sys.exit(0)
sys.stderr.write(f"scenario not found: {wanted}\n")
sys.exit(2)
PY
    )" || {
        echo "[run_sim_fixed_path_bag] ERROR: failed to load SCENARIO='${SCENARIO}' from ${SCENARIOS_FILE}" >&2
        exit 2
    }
    eval "${scenario_env}"
    if [[ -z "${PATH_FILE_WAS_SET}" ]]; then
        PATH_FILE="${FIXED_PATH_DIR}/${PATH_ID}.json"
    fi
    if [[ -z "${BAG_NAME_WAS_SET}" ]]; then
        BAG_NAME="${BAG_DATE}_${PATH_ID}_${CONDITION}_run${RUN_ID}_${BAG_TIME}"
        BAG_PATH="${BAG_DIR}/${BAG_NAME}"
    fi
fi

if [[ "${PATH_MODE}" == "template_goal" && -z "${PATH_FILE_WAS_SET}" ]]; then
    PATH_FILE="${FIXED_PATH_DIR}/${BAG_NAME}.json"
fi

case "${CONDITION}" in
    NOM)
        Q_SLOSH="${Q_SLOSH:-0}"
        Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.0}"
        ;;
    FAS_Q5)
        Q_SLOSH="${Q_SLOSH:-5}"
        Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.0}"
        ;;
    FAS_Q5_DOT)
        Q_SLOSH="${Q_SLOSH:-5}"
        Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.05}"
        ;;
    FAS_Q10)
        Q_SLOSH="${Q_SLOSH:-10}"
        Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.0}"
        ;;
    FAS_Q5_TERM)
        # P2: Q_slosh_eta_dot > 0 是 terminal_factor_slosh_eta_dot 生效的前提
        Q_SLOSH="${Q_SLOSH:-5}"
        Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.02}"
        TERMINAL_FACTOR_SLOSH_ETA="${TERMINAL_FACTOR_SLOSH_ETA:-10.0}"
        TERMINAL_FACTOR_SLOSH_ETA_DOT="${TERMINAL_FACTOR_SLOSH_ETA_DOT:-10.0}"
        ;;
    CUSTOM)
        Q_SLOSH="${Q_SLOSH:-0}"
        Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.0}"
        ;;
    RAW_TUNED)
        Q_SLOSH="${Q_SLOSH:-0}"
        Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.0}"
        MPC_R_A="${MPC_R_A:-1.0}"
        MPC_R_DA="${MPC_R_DA:-2.0}"
        MPC_CMD_VEL_LEAD_TIME="${MPC_CMD_VEL_LEAD_TIME:-0.05}"
        VEHICLE_V_MAX="${VEHICLE_V_MAX:-2.0}"
        DEFAULT_GLOBAL_PATH_TOPIC="/scout/global_path"
        ;;
    *)
        echo "[run_sim_fixed_path_bag] ERROR: unsupported CONDITION='${CONDITION}'" >&2
        echo "Use NOM, FAS_Q5, FAS_Q5_DOT, FAS_Q10, FAS_Q5_TERM, CUSTOM, or RAW_TUNED." >&2
        exit 2
        ;;
esac

if [[ "${PATH_MODE}" != "replay" && "${PATH_MODE}" != "template_goal" && "${PATH_MODE}" != "global_goal" ]]; then
    echo "[run_sim_fixed_path_bag] ERROR: unsupported PATH_MODE='${PATH_MODE}' (use replay, template_goal, or global_goal)" >&2
    exit 2
fi

MPC_R_A="${MPC_R_A:-0.4}"
MPC_R_DA="${MPC_R_DA:-0.5}"
MPC_CMD_VEL_LEAD_TIME="${MPC_CMD_VEL_LEAD_TIME:-0.15}"
VEHICLE_V_MAX="${VEHICLE_V_MAX:-3.0}"
if [[ -z "${GLOBAL_PATH_TOPIC}" ]]; then
    if [[ -n "${DEFAULT_GLOBAL_PATH_TOPIC}" ]]; then
        GLOBAL_PATH_TOPIC="${DEFAULT_GLOBAL_PATH_TOPIC}"
    elif [[ "${PATH_MODE}" == "global_goal" ]]; then
        GLOBAL_PATH_TOPIC="/scout/global_path"
    else
        GLOBAL_PATH_TOPIC="/scout/global_path_fixed"
    fi
fi

if [[ -z "${PATH_SOURCE_OUTPUT_TOPIC}" ]]; then
    PATH_SOURCE_OUTPUT_TOPIC="${GLOBAL_PATH_TOPIC}"
fi

if [[ "${PATH_MODE}" == "template_goal" && -z "${TEMPLATE_NAME}" ]]; then
    case "${PATH_ID}" in
        P0_straight) TEMPLATE_NAME="straight" ;;
        P1_single_turn) TEMPLATE_NAME="single_turn" ;;
        P2_s_curve) TEMPLATE_NAME="s_curve" ;;
        P3_mixed) TEMPLATE_NAME="mixed" ;;
        *)
            echo "[run_sim_fixed_path_bag] ERROR: TEMPLATE_NAME is required for PATH_ID='${PATH_ID}'" >&2
            exit 2
            ;;
    esac
fi

if [[ "${PATH_MODE}" == "replay" && ! -f "${PATH_FILE}" ]]; then
    echo "[run_sim_fixed_path_bag] ERROR: path file not found: ${PATH_FILE}" >&2
    exit 2
fi

case "${RETIME_METHOD}" in
    none|toppra|ruckig|biagiotti)
        ;;
    *)
        echo "[run_sim_fixed_path_bag] ERROR: unsupported RETIME_METHOD='${RETIME_METHOD}' (use none, toppra, ruckig, or biagiotti)" >&2
        exit 2
        ;;
esac

if [[ -z "${EXTERNAL_PROFILE_MODE}" ]]; then
    if [[ "${RETIME_METHOD}" != "none" ]]; then
        EXTERNAL_PROFILE_MODE="${RETIME_METHOD}"
    elif [[ -n "${EXTERNAL_SPEED_PROFILE_CSV}" ]]; then
        EXTERNAL_PROFILE_MODE="custom_csv"
    else
        EXTERNAL_PROFILE_MODE="none"
    fi
fi

if [[ "${RETIME_METHOD}" != "none" && "${PATH_MODE}" == "global_goal" ]]; then
    echo "[run_sim_fixed_path_bag] ERROR: RETIME_METHOD requires replay/template_goal with a fixed path JSON, not PATH_MODE=global_goal" >&2
    exit 2
fi

mkdir -p "${BAG_DIR}"
mkdir -p "$(dirname "${PATH_FILE}")"
mkdir -p "${RETIME_PROFILE_DIR}"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi

if [[ -f /home/a/scout_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/a/scout_ws/devel/setup.bash
fi

pids=()
LAST_PID=""

stop_zero_cmd() {
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

cleanup() {
    local status=$?
    trap - INT TERM EXIT

    echo
    echo "[run_sim_fixed_path_bag] Stopping trial processes..."
    stop_zero_cmd
    for pid in "${pids[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
    wait || true
    stop_zero_cmd

    echo "[run_sim_fixed_path_bag] Bag target: ${BAG_PATH}.bag"
    exit "${status}"
}

trap cleanup INT TERM EXIT

start_bg() {
    local label="$1"
    shift
    echo "[run_sim_fixed_path_bag] Starting ${label}: $*"
    "$@" &
    local pid=$!
    LAST_PID="${pid}"
    pids+=("${pid}")
    sleep 1
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[run_sim_fixed_path_bag] ERROR: ${label} exited during startup." >&2
        wait "${pid}" || true
        exit 1
    fi
}

stop_pid() {
    local pid="$1"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    fi
}

wait_for_path_file() {
    local timeout_s="${1:-20}"
    local start_s
    start_s="$(date +%s)"
    while true; do
        if [[ -s "${PATH_FILE}" ]]; then
            return 0
        fi
        if (( $(date +%s) - start_s >= timeout_s )); then
            echo "[run_sim_fixed_path_bag] ERROR: timed out waiting for generated path file: ${PATH_FILE}" >&2
            return 1
        fi
        sleep 0.2
    done
}

prepare_external_speed_profile() {
    if [[ -n "${EXTERNAL_SPEED_PROFILE_CSV}" ]]; then
        if [[ ! -f "${EXTERNAL_SPEED_PROFILE_CSV}" ]]; then
            echo "[run_sim_fixed_path_bag] ERROR: EXTERNAL_SPEED_PROFILE_CSV not found: ${EXTERNAL_SPEED_PROFILE_CSV}" >&2
            exit 2
        fi
        return 0
    fi

    if [[ "${RETIME_METHOD}" == "none" ]]; then
        return 0
    fi

    if [[ ! -f "${PATH_FILE}" ]]; then
        echo "[run_sim_fixed_path_bag] ERROR: cannot retime missing path file: ${PATH_FILE}" >&2
        exit 2
    fi

    EXTERNAL_SPEED_PROFILE_CSV="${RETIME_PROFILE_DIR}/${BAG_NAME}_${RETIME_METHOD}_speed_profile.csv"
    local plot_path="${RETIME_PROFILE_DIR}/${BAG_NAME}_${RETIME_METHOD}_speed_profile.png"

    case "${RETIME_METHOD}" in
        toppra)
            echo "[run_sim_fixed_path_bag] Generating TOPPRA-style speed profile: ${EXTERNAL_SPEED_PROFILE_CSV}"
            rosrun scout_local_planner retime_toppra_style.py \
                --path-file "${PATH_FILE}" \
                --out-csv "${EXTERNAL_SPEED_PROFILE_CSV}" \
                --plot "${plot_path}" \
                --v-max "${RETIME_V_MAX}" \
                --a-max "${RETIME_A_MAX}" \
                --decel-max "${RETIME_DECEL_MAX}" \
                --ds "${RETIME_DS}"
            ;;
        ruckig)
            echo "[run_sim_fixed_path_bag] Generating Ruckig-style speed profile: ${EXTERNAL_SPEED_PROFILE_CSV}"
            rosrun scout_local_planner retime_ruckig_style.py \
                --path-file "${PATH_FILE}" \
                --out-csv "${EXTERNAL_SPEED_PROFILE_CSV}" \
                --plot "${plot_path}" \
                --v-max "${RETIME_V_MAX}" \
                --a-max "${RETIME_A_MAX}" \
                --j-max "${RETIME_J_MAX}" \
                --delta-time "${RETIME_DELTA_TIME}"
            ;;
        biagiotti)
            echo "[run_sim_fixed_path_bag] Generating Biagiotti-style shaped speed profile: ${EXTERNAL_SPEED_PROFILE_CSV}"
            rosrun scout_local_planner shape_biagiotti.py \
                --path-file "${PATH_FILE}" \
                --out-csv "${EXTERNAL_SPEED_PROFILE_CSV}" \
                --plot "${plot_path}" \
                --omega-n "${BIAGIOTTI_OMEGA_N}" \
                --damping-ratio "${BIAGIOTTI_DAMPING_RATIO}" \
                --v-max "${RETIME_V_MAX}" \
                --a-max "${RETIME_A_MAX}" \
                --decel-max "${RETIME_DECEL_MAX}" \
                --ds "${RETIME_DS}" \
                --delta-time "${RETIME_DELTA_TIME}"
            ;;
    esac
}

publish_template_goal() {
    local yaw
    yaw="$(python3 - "${TEMPLATE_GOAL_QX}" "${TEMPLATE_GOAL_QY}" "${TEMPLATE_GOAL_QZ}" "${TEMPLATE_GOAL_QW}" <<'PY'
import math
import sys
qx, qy, qz, qw = map(float, sys.argv[1:5])
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
print(math.atan2(siny_cosp, cosy_cosp))
PY
)"
    rosrun scout_local_planner send_fixed_goal.py \
        --goal-topic "${TEMPLATE_GOAL_TOPIC}" \
        --frame "${TEMPLATE_GOAL_FRAME}" \
        --x "${TEMPLATE_GOAL_X}" \
        --y "${TEMPLATE_GOAL_Y}" \
        --yaw "${yaw}" \
        --repeat-count "${TEMPLATE_GOAL_REPEAT_COUNT}" \
        --repeat-rate "${TEMPLATE_GOAL_REPEAT_RATE}" \
        --wait-subscriber-timeout "${TEMPLATE_GOAL_WAIT_SUBSCRIBER_TIMEOUT}"
}

write_config_summary() {
    local config_path="${BAG_PATH}.txt"
    local git_head
    local git_status
    git_head="$(git -C /home/a/scout_ws rev-parse --short HEAD 2>/dev/null || echo unknown)"
    git_status="$(git -C /home/a/scout_ws status --short 2>/dev/null || true)"

    cat > "${config_path}" <<EOF
bag_path=${BAG_PATH}.bag
bag_name=${BAG_NAME}
bag_date=${BAG_DATE}
bag_time=${BAG_TIME}
git_head=${git_head}

PATH_ID=${PATH_ID}
SCENARIO=${SCENARIO}
SCENARIOS_FILE=${SCENARIOS_FILE}
CONDITION=${CONDITION}
RUN_ID=${RUN_ID}
PATH_MODE=${PATH_MODE}
TEMPLATE_NAME=${TEMPLATE_NAME}
PATH_FILE=${PATH_FILE}
GLOBAL_PATH_TOPIC=${GLOBAL_PATH_TOPIC}
PATH_SOURCE_OUTPUT_TOPIC=${PATH_SOURCE_OUTPUT_TOPIC}
RETIME_METHOD=${RETIME_METHOD}
EXPERIMENT_GROUP=${EXPERIMENT_GROUP}
CONTROLLER_VARIANT=${CONTROLLER_VARIANT}
EXTERNAL_PROFILE_MODE=${EXTERNAL_PROFILE_MODE}
RETIME_PROFILE_DIR=${RETIME_PROFILE_DIR}
RETIME_V_MAX=${RETIME_V_MAX}
RETIME_A_MAX=${RETIME_A_MAX}
RETIME_DECEL_MAX=${RETIME_DECEL_MAX}
RETIME_J_MAX=${RETIME_J_MAX}
RETIME_DS=${RETIME_DS}
RETIME_DELTA_TIME=${RETIME_DELTA_TIME}
BIAGIOTTI_OMEGA_N=${BIAGIOTTI_OMEGA_N}
BIAGIOTTI_DAMPING_RATIO=${BIAGIOTTI_DAMPING_RATIO}
EXTERNAL_SPEED_PROFILE_CSV=${EXTERNAL_SPEED_PROFILE_CSV}
EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE=${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}
EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT=${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT}
EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT=${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT}
EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT=${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT}
TEMPLATE_GOAL_TOPIC=${TEMPLATE_GOAL_TOPIC}
TEMPLATE_GOAL_FRAME=${TEMPLATE_GOAL_FRAME}
TEMPLATE_GOAL_X=${TEMPLATE_GOAL_X}
TEMPLATE_GOAL_Y=${TEMPLATE_GOAL_Y}
TEMPLATE_GOAL_Z=${TEMPLATE_GOAL_Z}
TEMPLATE_GOAL_QX=${TEMPLATE_GOAL_QX}
TEMPLATE_GOAL_QY=${TEMPLATE_GOAL_QY}
TEMPLATE_GOAL_QZ=${TEMPLATE_GOAL_QZ}
TEMPLATE_GOAL_QW=${TEMPLATE_GOAL_QW}

Q_SLOSH=${Q_SLOSH}
Q_SLOSH_ETA_DOT=${Q_SLOSH_ETA_DOT}
MPC_Q_V=${MPC_Q_V}
MPC_R_A=${MPC_R_A}
MPC_R_DA=${MPC_R_DA}
MPC_CMD_VEL_LEAD_TIME=${MPC_CMD_VEL_LEAD_TIME}
TERMINAL_FACTOR_SLOSH_ETA=${TERMINAL_FACTOR_SLOSH_ETA}
TERMINAL_FACTOR_SLOSH_ETA_DOT=${TERMINAL_FACTOR_SLOSH_ETA_DOT}
VEHICLE_V_MAX=${VEHICLE_V_MAX}

git_status:
${git_status}
EOF

    echo "[run_sim_fixed_path_bag] Wrote config summary: ${config_path}"
}

publish_config_summary() {
    local summary
    summary="bag=${BAG_NAME}; condition=${CONDITION}; group=${EXPERIMENT_GROUP}; controller_variant=${CONTROLLER_VARIANT}; external_profile_mode=${EXTERNAL_PROFILE_MODE}; path=${PATH_ID}; run=${RUN_ID}; path_mode=${PATH_MODE}; global_path_topic=${GLOBAL_PATH_TOPIC}; path_source_output_topic=${PATH_SOURCE_OUTPUT_TOPIC}; Q_slosh=${Q_SLOSH}; Q_eta_dot=${Q_SLOSH_ETA_DOT}; Q_v=${MPC_Q_V}; R_a=${MPC_R_A}; R_da=${MPC_R_DA}; lead=${MPC_CMD_VEL_LEAD_TIME}; terminal_eta=${TERMINAL_FACTOR_SLOSH_ETA}; terminal_eta_dot=${TERMINAL_FACTOR_SLOSH_ETA_DOT}; vehicle_v_max=${VEHICLE_V_MAX}; retime=${RETIME_METHOD}; external_speed_csv=${EXTERNAL_SPEED_PROFILE_CSV}; external_profile_cap=${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}"
    rostopic pub -l /experiment/config_summary std_msgs/String "data: '${summary}'" >/dev/null &
    local pid=$!
    pids+=("${pid}")
    sleep 1
}

wait_until_fixed_path_start() {
    python3 - "$PATH_FILE" "$APPROACH_START_TIMEOUT" "$APPROACH_START_POS_TOL" \
        "$APPROACH_START_YAW_TOL" "$APPROACH_START_HOLD_S" <<'PY'
import json
import math
import sys

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

path_file = sys.argv[1]
timeout = float(sys.argv[2])
pos_tol = float(sys.argv[3])
yaw_tol = float(sys.argv[4])
hold_s = float(sys.argv[5])

with open(path_file, "r", encoding="utf-8") as fh:
    data = json.load(fh)

frame_id = data.get("frame_id", "map") or "map"
start = data["poses"][0]
target_x = float(start["x"])
target_y = float(start["y"])
if "qz" in start and "qw" in start:
    target_yaw = euler_from_quaternion([
        float(start.get("qx", 0.0)),
        float(start.get("qy", 0.0)),
        float(start.get("qz", 0.0)),
        float(start.get("qw", 1.0)),
    ])[2]
elif len(data["poses"]) > 1:
    p1 = data["poses"][1]
    target_yaw = math.atan2(float(p1["y"]) - target_y, float(p1["x"]) - target_x)
else:
    target_yaw = 0.0

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

rospy.init_node("wait_fixed_path_start", anonymous=True)
buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
listener = tf2_ros.TransformListener(buf)
deadline = rospy.Time.now() + rospy.Duration(timeout)
ready_since = None
rate = rospy.Rate(10.0)

rospy.loginfo(
    "Waiting for fixed-path start: frame=%s x=%.3f y=%.3f yaw=%.3f pos_tol=%.3f yaw_tol=%.3f",
    frame_id, target_x, target_y, target_yaw, pos_tol, yaw_tol,
)

while not rospy.is_shutdown() and rospy.Time.now() < deadline:
    try:
        tf_msg = buf.lookup_transform(frame_id, "base_link", rospy.Time(0), rospy.Duration(0.2))
    except Exception as exc:
        rospy.logwarn_throttle(1.0, "Waiting for start TF: %s", exc)
        rate.sleep()
        continue

    tr = tf_msg.transform.translation
    rot = tf_msg.transform.rotation
    yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
    pos_err = math.hypot(float(tr.x) - target_x, float(tr.y) - target_y)
    yaw_err = abs(wrap(yaw - target_yaw))
    in_tol = pos_err <= pos_tol and yaw_err <= yaw_tol

    if in_tol:
        if ready_since is None:
            ready_since = rospy.Time.now()
        held = (rospy.Time.now() - ready_since).to_sec()
        rospy.loginfo_throttle(
            1.0,
            "At fixed-path start: pos_err=%.3f yaw_err=%.3f hold=%.2f/%.2f",
            pos_err, yaw_err, held, hold_s,
        )
        if held >= hold_s:
            rospy.loginfo("Fixed-path start reached: pos_err=%.3f yaw_err=%.3f", pos_err, yaw_err)
            sys.exit(0)
    else:
        ready_since = None
        rospy.loginfo_throttle(
            1.0,
            "Approaching fixed-path start: pos_err=%.3f yaw_err=%.3f",
            pos_err, yaw_err,
        )
    rate.sleep()

rospy.logerr("Timed out waiting for fixed-path start")
sys.exit(1)
PY
}

TOPICS=(
    /clock
    /tf
    /tf_static
    /cmd_vel
    /odom
    /imu/data
    /scan_front
    /joint_states
    /map
    /scout/goal
    /scout/current_goal
    /scout/global_path
    /scout/global_path_fixed
    /scout/global_path_smooth
    /mpc/reference_path
    /local_path
    /mpc/solve_ms
    /mpc/status_val
    /mpc/cost_breakdown
    /mpc_status
    /terminal/mode
    /terminal/recovery_latched
    /terminal/goal_info
    /terminal/cmd_v_pre_clamp
    /terminal/cmd_v_post_clamp
    /profile_cap/active
    /profile_cap/v_profile
    /profile_cap/cmd_v_pre_cap
    /profile_cap/cmd_v_post_cap
    /profile_cap/implied_ax
    /profile_cap/implied_jerk
    /experiment/config_summary
    /reference/v_ref
    /reference/v_ref_horizon
    /reference/v_path
    /reference/kappa
    /reference/s
    /reference/s_horizon
    /reference/implied_ax
    /reference/implied_ay
    /reference/implied_jerk
    /reference/implied_ax_abs_p95
    /reference/implied_ay_abs_p95
    /reference/implied_jerk_abs_p95
    /rpp_speed_reg/active
    /rpp_speed_reg/curvature
    /rpp_speed_reg/curvature_active
    /rpp_speed_reg/approach_active
    /rpp_speed_reg/v_raw
    /rpp_speed_reg/v_curvature_cap
    /rpp_speed_reg/v_approach_cap
    /rpp_speed_reg/v_out
    /slosh/state
    /slosh/eta_norm
    /slosh/eta_dot_norm
    /slosh/modal_energy
    /slosh/modal_energy_norm
    /slosh/excitation_ay_abs
    /slosh/excitation_alpha_abs
    /slosh/height
    /slosh/height_pred_max
    /slosh/h_visual
    /slosh/h_visual_quality
    /slosh/q_slosh_eta
    /slosh/ax_est
    /slosh/ay_est
    /slosh/alpha_est
    /slosh/omega_est_used
    /slosh/imu_omega_z_filtered
    /slosh/imu_ay_bias
    /slosh/imu_ay_filtered
    /slosh/imu_ay_bias_ready
    /slosh/constraint_active
    /slosh/v_des_eff
)

REPLAY_ARGS=(
    rosrun scout_local_planner fixed_global_path_runner.py
    --mode replay
    --path-file "${PATH_FILE}"
    --output-topic "${PATH_SOURCE_OUTPUT_TOPIC}"
    --start-pos-tol "${START_POS_TOL}"
    --start-yaw-tol "${START_YAW_TOL}"
)

if [[ "${START_GATE}" == "true" ]]; then
    :
else
    REPLAY_ARGS+=(--skip-start-wait)
fi

if [[ "${PATH_PUBLISH_ONCE_KEEPALIVE}" == "true" ]]; then
    REPLAY_ARGS+=(--publish-once-keepalive)
fi

TEMPLATE_ARGS=(
    rosrun scout_local_planner template_fixed_path_generator.py
    --template "${TEMPLATE_NAME}"
    --start-heading "${TEMPLATE_START_HEADING}"
    --goal-topic "${TEMPLATE_GOAL_TOPIC}"
    --output-topic "${PATH_SOURCE_OUTPUT_TOPIC}"
    --path-file "${PATH_FILE}"
    --publish-count 0
)

MPC_ARGS=(
    scout_local_planner slosh_experiment_sim.launch
    experiment_group:="${EXPERIMENT_GROUP}"
    controller_variant:="${CONTROLLER_VARIANT}"
    external_profile_mode:="${EXTERNAL_PROFILE_MODE}"
    global_path_topic:="${GLOBAL_PATH_TOPIC}"
    Q_slosh:="${Q_SLOSH}"
    Q_slosh_eta_dot:="${Q_SLOSH_ETA_DOT}"
    mpc_Q_v:="${MPC_Q_V}"
    mpc_R_a:="${MPC_R_A}"
    mpc_R_da:="${MPC_R_DA}"
    mpc_cmd_vel_lead_time:="${MPC_CMD_VEL_LEAD_TIME}"
    terminal_factor_slosh_eta:="${TERMINAL_FACTOR_SLOSH_ETA}"
    terminal_factor_slosh_eta_dot:="${TERMINAL_FACTOR_SLOSH_ETA_DOT}"
    vehicle_v_max:="${VEHICLE_V_MAX}"
    external_profile_execution_cap_enable:="${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}"
    external_profile_execution_accel_limit:="${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT}"
    external_profile_execution_decel_limit:="${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT}"
    external_profile_execution_jerk_limit:="${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT}"
)

APPROACH_MPC_ARGS=(
    scout_local_planner slosh_experiment_sim.launch
    global_path_topic:="${APPROACH_GLOBAL_PATH_TOPIC}"
    Q_slosh:=0
    Q_slosh_eta_dot:=0.0
)

echo "============================================================"
echo "  Sim fixed-path slosh trial"
echo "============================================================"
echo "  PATH_ID              = ${PATH_ID}"
echo "  SCENARIO             = ${SCENARIO}"
echo "  CONDITION            = ${CONDITION}"
echo "  EXPERIMENT_GROUP     = ${EXPERIMENT_GROUP}"
echo "  CONTROLLER_VARIANT   = ${CONTROLLER_VARIANT}"
echo "  EXTERNAL_PROFILE_MODE= ${EXTERNAL_PROFILE_MODE}"
echo "  RUN_ID               = ${RUN_ID}"
echo "  PATH_MODE            = ${PATH_MODE}"
echo "  TEMPLATE_NAME        = ${TEMPLATE_NAME}"
echo "  PATH_FILE            = ${PATH_FILE}"
echo "  GLOBAL_PATH_TOPIC    = ${GLOBAL_PATH_TOPIC}"
echo "  PATH_SOURCE_OUTPUT   = ${PATH_SOURCE_OUTPUT_TOPIC}"
echo "  TEMPLATE_GOAL        = ${TEMPLATE_GOAL_TOPIC} (${TEMPLATE_GOAL_X}, ${TEMPLATE_GOAL_Y})"
echo "  goal_publish         = repeat=${TEMPLATE_GOAL_REPEAT_COUNT}, rate=${TEMPLATE_GOAL_REPEAT_RATE}Hz, wait_sub=${TEMPLATE_GOAL_WAIT_SUBSCRIBER_TIMEOUT}s"
echo "  BAG_PATH             = ${BAG_PATH}.bag"
echo "  Q_slosh              = ${Q_SLOSH}"
echo "  Q_slosh_eta_dot      = ${Q_SLOSH_ETA_DOT}"
echo "  mpc_Q_v              = ${MPC_Q_V}"
echo "  mpc_R_a              = ${MPC_R_A}"
echo "  mpc_R_da             = ${MPC_R_DA}"
echo "  cmd_vel_lead_time    = ${MPC_CMD_VEL_LEAD_TIME}"
echo "  term_factor_eta      = ${TERMINAL_FACTOR_SLOSH_ETA}"
echo "  term_factor_eta_dot  = ${TERMINAL_FACTOR_SLOSH_ETA_DOT}"
echo "  vehicle_v_max        = ${VEHICLE_V_MAX}"
echo "  retime_method        = ${RETIME_METHOD}"
echo "  external_speed_csv   = ${EXTERNAL_SPEED_PROFILE_CSV:-<none yet>}"
echo "  external_profile_cap = ${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}"
echo "  approach_start       = ${APPROACH_START_ENABLE}"
echo "  start_delay          = ${START_DELAY}s"
echo "  start_gate           = ${START_GATE}"
echo "  path_once_keepalive  = ${PATH_PUBLISH_ONCE_KEEPALIVE}"
echo "  record_duration      = ${RECORD_DURATION}s (0 means Ctrl+C)"
echo "============================================================"

write_config_summary

if [[ "${START_DELAY}" != "0" && "${START_DELAY}" != "0.0" ]]; then
    echo "[run_sim_fixed_path_bag] Waiting ${START_DELAY}s for sim/nav stack to settle..."
    sleep "${START_DELAY}"
fi

if [[ "${PATH_MODE}" == "replay" ]]; then
    prepare_external_speed_profile
    write_config_summary
fi

if [[ "${APPROACH_START_ENABLE}" == "true" ]]; then
    start_bg "approach MPC to fixed-path start" roslaunch "${APPROACH_MPC_ARGS[@]}"
    approach_pid="${LAST_PID}"
    sleep 2
    rosrun scout_local_planner fixed_global_path_runner.py \
        --mode goal_only \
        --path-file "${PATH_FILE}" \
        --publish-start-goal \
        --goal-topic "${APPROACH_GOAL_TOPIC}" \
        --goal-repeat-count 10 \
        --goal-repeat-rate 5
    wait_until_fixed_path_start
    stop_pid "${approach_pid}"
    stop_zero_cmd
    sleep 1
fi

start_bg "rosbag record" rosbag record -O "${BAG_PATH}" "${TOPICS[@]}"
sleep "${RECORD_WARMUP_S}"
publish_config_summary

if [[ "${PATH_MODE}" == "template_goal" ]]; then
    start_bg "template path generator" "${TEMPLATE_ARGS[@]}"
    sleep "${TEMPLATE_GENERATOR_WARMUP_S}"
    echo "[run_sim_fixed_path_bag] Publishing template goal on ${TEMPLATE_GOAL_TOPIC}"
    publish_template_goal
    if [[ "${RETIME_METHOD}" != "none" || -n "${EXTERNAL_SPEED_PROFILE_CSV}" ]]; then
        wait_for_path_file 20
        prepare_external_speed_profile
        write_config_summary
        publish_config_summary
    fi
    start_bg "MPC ${CONDITION}" roslaunch "${MPC_ARGS[@]}" external_speed_profile_csv:="${EXTERNAL_SPEED_PROFILE_CSV}"
    sleep "${MPC_WARMUP_S}"
elif [[ "${PATH_MODE}" == "global_goal" ]]; then
    start_bg "MPC ${CONDITION}" roslaunch "${MPC_ARGS[@]}" external_speed_profile_csv:="${EXTERNAL_SPEED_PROFILE_CSV}"
    sleep "${MPC_WARMUP_S}"
    echo "[run_sim_fixed_path_bag] Publishing global goal on ${TEMPLATE_GOAL_TOPIC}"
    publish_template_goal
else
    start_bg "fixed path replay" "${REPLAY_ARGS[@]}"
    sleep "${PATH_WARMUP_S}"
    start_bg "MPC ${CONDITION}" roslaunch "${MPC_ARGS[@]}" external_speed_profile_csv:="${EXTERNAL_SPEED_PROFILE_CSV}"
fi

echo "[run_sim_fixed_path_bag] Trial is running. Press Ctrl+C after REACHED, or set RECORD_DURATION for auto-stop."

if [[ "${RECORD_DURATION}" != "0" && "${RECORD_DURATION}" != "0.0" ]]; then
    sleep "${RECORD_DURATION}"
    cleanup
fi

wait -n "${pids[@]}"
echo "[run_sim_fixed_path_bag] One trial process exited; stopping the rest."
