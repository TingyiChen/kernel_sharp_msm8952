#!/bin/bash

# Wrapper around checkpatch.pl to filter results.

set -e

export STATIC_ANALYSIS_SRC_DIR=$(dirname $(readlink -f $0))

source ${STATIC_ANALYSIS_SRC_DIR}/../envsetup.sh
export OUT_DIR=$(readlink -m ${OUT_DIR:-${ROOT_DIR}/out/${BRANCH}})
export DIST_DIR=$(readlink -m ${DIST_DIR:-${OUT_DIR}/dist})
mkdir -p ${DIST_DIR}

# Save the original value before converting to abs path.
REPO_PATH=${KERNEL_DIR}
export KERNEL_DIR=$(readlink -m ${KERNEL_DIR})

CHECKPATCH_PL_PATH="${KERNEL_DIR}/scripts/checkpatch.pl"
GIT_SHA1="HEAD"
POOL_SIZE=32
PATCH_DIR="${OUT_DIR}/checkpatch/patches"
CHECKED_DIR="${OUT_DIR}/checkpatch/checked"
COMMAND_FILE="${OUT_DIR}/checkpatch/checkpatch_commands"
BLACKLIST_FILE="${STATIC_ANALYSIS_SRC_DIR}/checkpatch_blacklist"
RESULTS_PATH=${DIST_DIR}/checkpatch_violations.txt
RETURN_CODE=0

# Parse flags.
CHECKPATCH_ARGS=()
while [[ $# -gt 0 ]]; do
  next="$1"
  case ${next} in
  --git_sha1)
    GIT_SHA1="$2"
    shift
    ;;
  --repo_prop)
    if [[ ! -f "$2" ]]; then
      echo "Failed to find file $2"
      exit 1
    fi
    GIT_SHA1=$(grep -E "${REPO_PATH} [0-9a-f]+" "$2" | awk '{print $2}')
    if [[ -z "${GIT_SHA1}" ]]; then
      echo "Failed to find repo ${REPO_PATH} in $2"
      exit 1
    fi
    shift
    ;;
  --blacklisted_checks)
    BLACKLIST_FILE="$2"
    shift
    ;;
  --pool_size)
    POOL_SIZE="$2"
    shift
    ;;
  --help)
    echo "Gets a patch from git, passes it checkpatch.pl, and then reports"
    echo "the subset of violations we choose to enforce."
    echo ""
    echo "Usage: $0"
    echo "  <--git_sha1 nnn> (Defaults to HEAD)"
    echo "  <--repo_prop path> (Gets SHA1 from this file, instead of --git_sha1."
    echo "      Expects sha1's to be indexed by KERNEL_DIR)"
    echo "  <--blacklisted_checks path_to_file> (Defaults to checkpatch_blacklist)"
    echo "  <--pool_size num_subprocs> (Defaults to 32)"
    echo "  <args for checkpatch.pl>"
    exit 0
    ;;
  *)
    CHECKPATCH_ARGS+=("$1")
    ;;
  esac
  shift
done


# Clean up from any previous run.
if [[ -d "${PATCH_DIR}" ]]; then
  rm -fr "${PATCH_DIR}"
fi
mkdir -p "${PATCH_DIR}"

if [[ -d "${CHECKED_DIR}" ]]; then
  rm -fr "${CHECKED_DIR}"
fi
mkdir -p "${CHECKED_DIR}"

if [[ -f "${COMMAND_FILE}" ]]; then
  rm -fr "${COMMAND_FILE}"
fi

# Update blacklist.
if [[ -f "${BLACKLIST_FILE}" ]]; then
  IGNORED_ERRORS=$(grep -v '^#' ${BLACKLIST_FILE} | paste -s -d,)
  if [[ -n "${IGNORED_ERRORS}" ]]; then
    CHECKPATCH_ARGS+=(--ignore)
    CHECKPATCH_ARGS+=("${IGNORED_ERRORS}")
  fi
fi

# Check the patch for errors.
echo "========================================================"
echo " Running static analysis..."
echo "    Using KERNEL_DIR: ${KERNEL_DIR}"

cd ${KERNEL_DIR}
git format-patch --quiet -o "${PATCH_DIR}" "${GIT_SHA1}^1"
echo "    Analyzing $(ls -l ${PATCH_DIR} | grep -c [.]patch) commits"

# Generate a list of checkpatch commands for later consumption by xargs.
declare -a EXPECTED_OUTPUTS=()
for PATCH_FILE in ${PATCH_DIR}/*.patch; do
  OUTPUT_FILE="${CHECKED_DIR}/$(basename ${PATCH_FILE%.patch}.checked)"
  EXPECTED_OUTPUTS=("${EXPECTED_OUTPUTS[@]}" "${OUTPUT_FILE}")
  # Ignore return code from checkpatch.pl, since it may be due to violations
  echo "${CHECKPATCH_PL_PATH} ${CHECKPATCH_ARGS[*]} ${PATCH_FILE} > ${OUTPUT_FILE} || true" \
    >> "${COMMAND_FILE}"
done

# Use xargs to run checkpatch.pl in a pool of subprocesses.
cat "${COMMAND_FILE}" | xargs -I CMD --max-procs=${POOL_SIZE} bash -c CMD

# Verify checkpatch produced output for every patch.
ACTUAL_OUTPUTS=($(ls "${CHECKED_DIR}"/*.checked))
if [[ ${#EXPECTED_OUTPUTS[@]} -gt ${#ACTUAL_OUTPUTS[@]} ]]; then
  MISSING=$(echo ${EXPECTED_OUTPUTS[@]} ${ACTUAL_OUTPUTS[@]} | tr ' ' '\n' | sort | uniq -u)
  echo "Missing expected outputs: ${MISSING}"
  RETURN_CODE=1
fi

# Verify none of the output is due to invalid usage.
USAGE_ERRORS=$(grep -r -E "^Usage: ${CHECKPATCH_PL_PATH}" "${CHECKED_DIR}" || true)
if [[ -n ${USAGE_ERRORS} ]]; then
  echo "    Found invalid calls to checkpatch.pl"
  RETURN_CODE=1
fi

# Search through all output files and aggregate errors.
{ grep -r -h -E -A1 "^ERROR:" "${CHECKED_DIR}" || true; } > "${RESULTS_PATH}"

NUM_VIOLATIONS=$(grep -E -c "^ERROR:" "${RESULTS_PATH}" || true)
echo "    Found ${NUM_VIOLATIONS} violations"
if [[ ${NUM_VIOLATIONS} -ne 0 ]]; then
  echo "    Details in $(basename ${RESULTS_PATH})"
  RETURN_CODE=1
fi

echo "======Finished running static analysis.======"
exit ${RETURN_CODE}

