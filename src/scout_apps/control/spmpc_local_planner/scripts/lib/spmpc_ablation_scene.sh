# Frozen map/path pairs for the development ablation entry only.
# Sourced after historical scene defaults, before validation/output paths.
SMOKE_SCENE="${ABLATION_SCENE:-20260829_c02}"
case "${SMOKE_SCENE}" in
  20260829_c02)
    # Preserve the original scene, labels and output directory.
    ;;
  20260907_c03)
    FROZEN_PATH_FILE=/home/geist/fixed_paths/real/20260907_spmpc_mocap_execution_chain/candidates/mocap_compact_s_C03.json
    FROZEN_PATH_SHA256=bda11f74194bb65d7df3b4ba5f1f5c3cc756dafdfd7bc06e753a288e30f7e006
    FROZEN_MAP_FILE=/home/geist/scout_maps/real/20260907_mocap_exec/map_carto_20260907_mocap_exec_v1.pbstream
    FROZEN_MAP_SHA256=86085992407b90571454d1f3c2a1379f602f0259df9535c60a6119470ccbcda6
    OUTPUT_SERIES="${OUTPUT_SERIES}_${SMOKE_SCENE}"
    RUN_LABEL_PREFIX="${RUN_LABEL_PREFIX}_${SMOKE_SCENE}"
    BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID}_${SMOKE_SCENE}"
    ;;
  *) fail "unknown ablation scene: ${SMOKE_SCENE}" ;;
esac
OPERATOR_NOTE="${OPERATOR_NOTE}; scene=${SMOKE_SCENE}"
