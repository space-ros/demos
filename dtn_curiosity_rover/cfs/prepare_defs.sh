#!/bin/bash
# Assemble the "dtn" cFS mission configuration (/cfs/dtn_defs) for this demo.
#
# Starts from the cFE sample_defs, which match the cFS release, and adds BPNode's
# per-node tables and startup scripts from nasa/bp dtn_defs. BPNode's own cFE config
# headers are not used: they predate cFS v7.0.0. Each change below is checked, so the
# build stops if a new cFS or BPNode release no longer matches it.
#
#   prepare_defs.sh <overlay dir> <ground address> <rover address>
set -euo pipefail

OVERLAY=$1
GROUND_ADDR=$2
ROVER_ADDR=$3
DEFS=/cfs/dtn_defs

fail() { echo "prepare_defs: $*" >&2; exit 1; }

# Replace exactly one match of a sed expression in a file, or fail
patch_once() {
  local expr=$1 file=$2 check=$3
  sed -i -E "${expr}" "${file}"
  grep -qE "${check}" "${file}" || fail "patch did not apply to ${file}: ${expr}"
}

valid_ipv4() { [[ $1 =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; }
valid_ipv4 "${GROUND_ADDR}" || fail "DTN_GROUND_ADDR is not an IPv4 address: ${GROUND_ADDR}"
valid_ipv4 "${ROVER_ADDR}" || fail "DTN_ROVER_ADDR is not an IPv4 address: ${ROVER_ADDR}"

cd /cfs
cp -r cfe/cmake/sample_defs "${DEFS}"
cp -r apps/bpnode/dtn_defs/cpu1 apps/bpnode/dtn_defs/cpu2 "${DEFS}/"
mkdir -p "${DEFS}/tables"
cp apps/bpnode/dtn_defs/tables/*.c "${DEFS}/tables/"
rm -f "${DEFS}/cpu1_cfe_es_startup.scr"

# BPNode's UDP convergence layer is a PSP module shipped with BPNode
echo 'list(APPEND MISSION_MODULE_SEARCH_PATH "apps/bpnode/psp_mod")' >> "${DEFS}/mission_build_custom.cmake"

# racs2_bridge_client.so is longer than OSAL's default 20-character file names
echo 'set(OSAL_CONFIG_MAX_FILE_NAME 32)' >> "${DEFS}/default_osconfig.cmake"

# Values BPNode needs from its dtn_defs cfe_mission_cfg.h: longer file names and
# paths for bundle storage, more Software Bus pipes
patch_once 's/^(#define CFE_MISSION_MAX_PATH_LEN) .*/\1 128/' "${DEFS}/example_mission_cfg.h" '^#define CFE_MISSION_MAX_PATH_LEN 128$'
patch_once 's/^(#define CFE_MISSION_MAX_FILE_LEN) .*/\1 64/' "${DEFS}/example_mission_cfg.h" '^#define CFE_MISSION_MAX_FILE_LEN 64$'
patch_once 's/^(#define CFE_MISSION_SB_MAX_PIPES) .*/\1 64/' "${DEFS}/example_mission_cfg.h" '^#define CFE_MISSION_SB_MAX_PIPES 64$'

# Table names that SCH_LAB and TO_LAB register in cFS v7.0.0
patch_once 's/SCH_LAB_APP\.ScheduleTable/SCH_LAB_APP.Schedule/' "${DEFS}/tables/sch_lab_table.c" 'SCH_LAB_APP\.Schedule,'
patch_once 's/TO_LAB_APP\.TO_LAB_Subs/TO_LAB_APP.Subscriptions/' "${DEFS}/tables/to_lab_sub.c" 'TO_LAB_APP\.Subscriptions,'
grep -q '"Schedule"' apps/sch_lab/fsw/src/sch_lab_app.c || fail "SCH_LAB no longer registers its table as \"Schedule\""
grep -q '"Subscriptions"' apps/to_lab/fsw/src/to_lab_app.c || fail "TO_LAB no longer registers its table as \"Subscriptions\""

# Demo overlay: two-node targets, startup scripts, BPNode tables, table name limits
cp -r "${OVERLAY}/." "${DEFS}/"

# DTN link addresses of the two nodes, compiled into BPNode's contact tables
patch_once "s/@DTN_ROVER_ADDR@/${ROVER_ADDR}/" "${DEFS}/cpu1/tables/bpnode_contacts.c" "\"${ROVER_ADDR//./\\.}\""
patch_once "s/@DTN_GROUND_ADDR@/${GROUND_ADDR}/" "${DEFS}/cpu2/tables/bpnode_contacts.c" "\"${GROUND_ADDR//./\\.}\""
if grep -rq '@DTN_' "${DEFS}"; then fail "unfilled address placeholder in ${DEFS}"; fi

echo "prepare_defs: ground ${GROUND_ADDR}, rover ${ROVER_ADDR}"
