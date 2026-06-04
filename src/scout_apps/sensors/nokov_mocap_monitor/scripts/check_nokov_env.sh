#!/usr/bin/env bash
set -euo pipefail

NOKOV_SERVER="${NOKOV_SERVER:-${1:-10.1.1.198}}"
MOCAP_TRACKER="${MOCAP_TRACKER:-${2:-Scout}}"
RAW_TOPIC="/vrpn_client_node/${MOCAP_TRACKER}/pose"

check_topic() {
  local topic="$1"
  local label="$2"
  if ! rostopic list 2>/dev/null | grep -qx "$topic"; then
    echo "[WARN] ${label} topic not found: ${topic}"
    return 0
  fi
  echo "[OK] ${label} topic exists: ${topic}"
  timeout 5s rostopic echo -n 1 "$topic" >/dev/null 2>&1 \
    && echo "[OK] ${label} has at least one message" \
    || echo "[WARN] ${label} did not produce a message within 5s"
}

echo "[nokov] server=${NOKOV_SERVER} tracker=${MOCAP_TRACKER}"

if rospack find vrpn_client_ros >/dev/null 2>&1; then
  echo "[OK] vrpn_client_ros: $(rospack find vrpn_client_ros)"
else
  echo "[ERR] vrpn_client_ros not found. Install with: sudo apt-get install ros-noetic-vrpn-client-ros" >&2
fi

if ping -c 2 -W 1 "$NOKOV_SERVER" >/dev/null 2>&1; then
  echo "[OK] ping ${NOKOV_SERVER}"
else
  echo "[WARN] cannot ping ${NOKOV_SERVER}; check cable/IP/firewall/XINGYING VRPN binding"
fi

if ! rostopic list >/dev/null 2>&1; then
  echo "[WARN] ROS master is not available; start roscore or the Nokov monitor launch first"
  exit 0
fi

check_topic "$RAW_TOPIC" "raw VRPN pose"
check_topic "/mocap/scout_pose" "monitor pose"
check_topic "/mocap/scout_odom" "monitor odom"
check_topic "/mocap/scout_path" "monitor path"
check_topic "/mocap/status" "monitor status"

echo ""
echo "[isolation checklist]"
echo "  - Nokov monitor must not publish /odom or /cmd_vel."
echo "  - SPMPC must not subscribe to /mocap/scout_odom."
echo "  - Default monitor launch must not publish map->base_link or odom->base_link TF."
echo "  - Mocap data is for RViz/rosbag/offline analysis only."
