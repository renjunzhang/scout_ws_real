#!/usr/bin/env bash
# Continue the frozen G3 planned order after a retained failed postflight.
#
# This helper does not retry or overwrite the failed row.  It obtains the exact
# frozen runner environment from the original G3 wrapper's validate-only mode,
# records a continuation ledger, and executes only Row 07/08.  G3 remains FAIL
# unless the original preregistered analyzer says otherwise.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G3-CONTINUE][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORIGINAL_WRAPPER="${SCRIPT_DIR}/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
VALIDATOR="${SCRIPT_DIR}/analysis/validate_g3_online_rgb_trial.py"
ANALYZER="${SCRIPT_DIR}/analysis/analyze_g3_w5_vs_bsmooth.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"

G3_ROW="${G3_ROW:-}"
DATE="${DATE:-20260801}"
STAMP="${STAMP:-$(date +%H%M%S)}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"
ACKNOWLEDGE_FAILED_PREVIOUS_ROW="${ACKNOWLEDGE_FAILED_PREVIOUS_ROW:-NO}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g3_processed_imu_w5_vs_bsmooth/H0}"

case "${G3_ROW}" in
  07)
    BLOCK=04
    POSITION=01
    METHOD=Bsmooth
    WEIGHT=0
    RUN_LABEL=DEV_G3_H0_C1_Bsmooth_b04_p01_a01
    PREVIOUS_LABEL=DEV_G3_H0_C1_Bsmooth_b03_p02_a01
    ;;
  08)
    BLOCK=04
    POSITION=02
    METHOD=W5
    WEIGHT=5
    RUN_LABEL=DEV_G3_H0_C1_W5_b04_p02_a01
    PREVIOUS_LABEL=DEV_G3_H0_C1_Bsmooth_b04_p01_a01
    ;;
  *) fail "set G3_ROW=07 or G3_ROW=08" ;;
esac

[[ "${ACKNOWLEDGE_FAILED_PREVIOUS_ROW}" == "YES" ]] || \
  fail "set ACKNOWLEDGE_FAILED_PREVIOUS_ROW=YES; this preserves the failed row and is not a retry"
if ! truthy "${VALIDATE_ONLY}"; then
  [[ "${ARM_MOTION}" == "YES" ]] || fail "set ARM_MOTION=YES only when the area is clear"
  [[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || fail "set CONFIRM_RGB_GEOMETRY=YES after checking the frozen geometry"
fi

for required in "${ORIGINAL_WRAPPER}" "${RUNNER}" "${VALIDATOR}" "${ANALYZER}" \
  "${SUMMARIZER}" "${CAMERA_PREP}"; do
  [[ -s "${required}" ]] || fail "missing required frozen tool: ${required}"
done

previous_report="${RUN_OUT_DIR}/${PREVIOUS_LABEL}_g3_postflight.json"
[[ -s "${previous_report}" ]] || fail "previous postflight is missing: ${previous_report}"
previous_status="$(python3 - "${previous_report}" <<'PY'
import json
import sys
report = json.load(open(sys.argv[1], encoding="utf-8"))
status = report.get("status")
if status not in {"PASS", "FAIL"}:
    raise SystemExit("previous postflight status must be PASS or FAIL")
print(status)
PY
)"

if [[ "${G3_ROW}" == "07" && "${previous_status}" != "FAIL" ]]; then
  fail "Row 07 continuation is only for the retained Row 06 FAIL"
fi

bag_path="${RUN_OUT_DIR}/${RUN_LABEL}.bag"
[[ ! -e "${bag_path}" && ! -e "${bag_path}.active" ]] || \
  fail "output already exists: ${bag_path}"

prereg_file="${RUN_OUT_DIR}/G3_processed_imu_W5_vs_Bsmooth_prereg.env"
order_file="${RUN_OUT_DIR}/G3_processed_imu_W5_vs_Bsmooth_order.csv"
metric_file="${RUN_OUT_DIR}/G3_rgb_sync_and_metric.yaml"
online_config_file="${RUN_OUT_DIR}/G3_online_liquid_config.env"
bundle_file="${RUN_OUT_DIR}/G3_prereg_bundle.sha256"
for artifact in "${prereg_file}" "${order_file}" "${metric_file}" \
  "${online_config_file}" "${bundle_file}"; do
  [[ -s "${artifact}" ]] || fail "missing prereg artifact: ${artifact}"
done
sha256sum -c "${bundle_file}" >/dev/null || fail "G3 prereg bundle hash check failed"

validation_output="$(
  DATE="${DATE}" STAMP="${STAMP}" G3_ROW="${G3_ROW}" VALIDATE_ONLY=true \
    bash "${ORIGINAL_WRAPPER}"
)" || fail "original frozen wrapper validate-only failed"
printf '%s\n' "${validation_output}"
runner_line="$(awk '/^[[:space:]]+env / {print; exit}' <<< "${validation_output}")"
[[ -n "${runner_line}" ]] || fail "could not extract the frozen runner command"

if truthy "${VALIDATE_ONLY}"; then
  echo "[G3-CONTINUE] validate-only PASS"
  echo "  row=${G3_ROW} previous=${PREVIOUS_LABEL} status=${previous_status}"
  echo "  failed row is retained; no retry/overwrite is performed"
  echo "  frozen runner:${runner_line}"
  exit 0
fi

PREREG_SHA256="$(sha256sum "${prereg_file}" | awk '{print $1}')"
ORDER_SHA256="$(sha256sum "${order_file}" | awk '{print $1}')"
OUTCOME_WINDOW_RULE_SHA256="$(sha256sum "${metric_file}" | awk '{print $1}')"
ONLINE_CONFIG_SHA256="$(sha256sum "${online_config_file}" | awk '{print $1}')"
SOURCE_REPORT_SHA256="36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442"
G2C_EVIDENCE_SHA256="06f626f4b3107721da9b904f7e5e9fa5b0126c191fb736d0dac5f65c87e9c602"
RGB_CALIBRATION_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml"
RGB_CALIBRATION_SHA256="7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01"

binding_file="${RUN_OUT_DIR}/${RUN_LABEL}_g3_binding.env"
printf '%s\n' \
  "g3_row=${G3_ROW}" \
  "block=${BLOCK}" \
  "position=${POSITION}" \
  "condition=${METHOD}" \
  "pilot_method=${METHOD}" \
  "source_report_sha256=${SOURCE_REPORT_SHA256}" \
  "g2c_evidence_sha256=${G2C_EVIDENCE_SHA256}" \
  "prereg_sha256=${PREREG_SHA256}" \
  "order_sha256=${ORDER_SHA256}" \
  "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
  > "${binding_file}"

ledger="${RUN_OUT_DIR}/G3_CONTINUATION_LEDGER.jsonl"
if [[ -e "${ledger}" ]]; then
  chmod u+w "${ledger}"
fi
python3 - "${ledger}" "${G3_ROW}" "${RUN_LABEL}" "${PREVIOUS_LABEL}" \
  "${previous_report}" "${previous_status}" "${ORIGINAL_WRAPPER}" "${BASH_SOURCE[0]}" <<'PY'
import hashlib
import json
import os
import sys
from datetime import datetime, timezone

ledger, row, label, previous_label, report_path, previous_status, wrapper, helper = sys.argv[1:]
def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()
entry = {
    "event": "CONTINUE_PLANNED_ROW_AFTER_RETAINED_FAILURE",
    "utc": datetime.now(timezone.utc).isoformat(),
    "row": row,
    "run_label": label,
    "previous_run_label": previous_label,
    "previous_status": previous_status,
    "previous_postflight": report_path,
    "previous_postflight_sha256": digest(report_path),
    "frozen_wrapper_sha256": digest(wrapper),
    "continuation_helper_sha256": digest(helper),
    "retry": False,
    "replaces_previous_row": False,
}
with open(ledger, "a", encoding="utf-8") as stream:
    stream.write(json.dumps(entry, sort_keys=True) + "\n")
os.chmod(ledger, 0o444)
PY

cleanup_ledger_mode() {
  if [[ -e "${ledger}" ]]; then chmod a-w "${ledger}" 2>/dev/null || true; fi
}
trap cleanup_ledger_mode EXIT

bash "${CAMERA_PREP}"

publisher_count() {
  local info
  info="$(rostopic info "$1" 2>/dev/null || true)"
  awk '
    /^Publishers:/ {inside=1; next}
    /^Subscribers:/ {inside=0}
    inside && /^[[:space:]]+\*/ {count++}
    END {print count+0}
  ' <<< "${info}"
}
for topic in /liquid/measurement /liquid/height /liquid/height_lcr \
  /liquid/height_median /liquid/debug_image; do
  count="$(publisher_count "${topic}")"
  [[ "${count}" == "0" ]] || fail "unexpected publisher(s) on ${topic}: ${count}"
done

online_log="${RUN_OUT_DIR}/${RUN_LABEL}_online_liquid.log"
online_pid=""
cleanup_online() {
  if [[ -n "${online_pid}" ]] && kill -0 "${online_pid}" 2>/dev/null; then
    kill -INT "${online_pid}" 2>/dev/null || true
    wait "${online_pid}" 2>/dev/null || true
  fi
  cleanup_ledger_mode
}
trap cleanup_online EXIT INT TERM

roslaunch realsense_liquid_measurement online_liquid_height.launch \
  calibration:="${RGB_CALIBRATION_FILE}" \
  image_topic:=/camera/color/image_raw \
  measurement_topic:=/liquid/measurement \
  process_every:=1 zero_frames:=30 publish_debug:=false height_bias_mm:=0.0 \
  hue1_low:=0 hue1_high:=12 hue2_low:=161 hue2_high:=179 \
  sat_min:=101 val_min:=167 > "${online_log}" 2>&1 &
online_pid=$!
sleep 2
kill -0 "${online_pid}" 2>/dev/null || fail "online RGB node exited during startup"

ready_log="${RUN_OUT_DIR}/${RUN_LABEL}_online_liquid_ready.log"
timeout 20s rostopic echo -n 20 \
  --filter "m.valid and m.zero_locked and m.status_code == 0" \
  /liquid/measurement > "${ready_log}" 2>&1 || \
  fail "online RGB did not provide 20 clean zero-locked measurements"
ready_count="$(grep -Ec '^valid: (True|true)$' "${ready_log}" || true)"
(( ready_count >= 20 )) || fail "online RGB ready count=${ready_count}, expected 20"

echo "[G3-CONTINUE] Executing frozen Row ${G3_ROW}; Row 06 remains retained FAIL."
python3 - "${runner_line}" <<'PY'
import shlex
import subprocess
import sys
argv = shlex.split(sys.argv[1].strip())
if not argv or argv[0] != "env":
    raise SystemExit("frozen command did not start with env")
raise SystemExit(subprocess.run(argv, check=False).returncode)
PY

python3 "${VALIDATOR}" \
  --bag "${bag_path}" \
  --condition "${METHOD}" \
  --row "${G3_ROW}" \
  --block "${BLOCK}" \
  --position "${POSITION}" \
  --expected-weight "${WEIGHT}" \
  --expected-v-ref 0.20 \
  --t-hvis-tail-sec 5.0 \
  --t-motion-max-sec 42 \
  --min-duration-sec 65 \
  --max-pre-motion-sec 23 \
  --rgb-calibration-sha256 "${RGB_CALIBRATION_SHA256}" \
  --online-config-sha256 "${ONLINE_CONFIG_SHA256}" \
  --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
  --prereg-sha256 "${PREREG_SHA256}" \
  --source-report-sha256 "${SOURCE_REPORT_SHA256}" \
  --hash-bag

python3 "${SUMMARIZER}" "${bag_path}"

if [[ "${G3_ROW}" == "08" ]]; then
  set +e
  python3 "${ANALYZER}" \
    --root "${RUN_OUT_DIR}" \
    --delta-h-dev-mm 0.10 \
    --minimum-positive-blocks 3 \
    --expected-pairs 4 \
    --prereg-sha256 "${PREREG_SHA256}" \
    --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
    --source-report-sha256 "${SOURCE_REPORT_SHA256}"
  analyzer_code=$?
  set -e
  if (( analyzer_code != 0 )); then
    echo "[G3-CONTINUE] Row 08 is archived, but frozen G3 analyzer reports FAIL." >&2
    exit "${analyzer_code}"
  fi
fi

echo "[G3-CONTINUE] Row ${G3_ROW} postflight PASS; retained previous failure was not replaced."
