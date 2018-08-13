#!/bin/bash

# Usage:
#   build/build_test.sh

export MAKE_ARGS=$@
export ROOT_DIR=$(dirname $(readlink -f $0))
export NET_TEST=${ROOT_DIR}/../kernel/tests/net/test
export BUILD_CONFIG=build/build.config.net_test

test=all_tests.sh
set -e
source ${ROOT_DIR}/envsetup.sh
export OUT_DIR=$(readlink -m ${OUT_DIR:-${ROOT_DIR}/out/${BRANCH}})
mkdir -p ${OUT_DIR}

# Normally this comes from build.config but build.config.net_test
# is generic, assume there is only one source directory.
export KERNEL_DIR=$(readlink -m ${KERNEL_DIR}/*)

echo "========================================================"
echo " Building kernel and running tests "

cd ${OUT_DIR}
$NET_TEST/run_net_test.sh --builder $test

echo "======Finished running tests======"
