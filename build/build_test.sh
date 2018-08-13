#!/bin/bash

# Usage:
#   build/build_test.sh

export MAKE_ARGS=$@
export ROOT_DIR=$(dirname $(readlink -f $0))
export NET_TEST=${ROOT_DIR}/../kernel/tests/net/test

# if device has its own build.config.net_test in the
# root (via manifest copy rule) then use it, otherwise
# use the default one in the build/ directory
export BUILD_CONFIG=build.config.net_test
if [ ! -e build.config.net_test ]; then
    export BUILD_CONFIG=build/${BUILD_CONFIG}
fi

test=all_tests.sh
set -e
source ${ROOT_DIR}/envsetup.sh
export OUT_DIR=$(readlink -m ${OUT_DIR:-${ROOT_DIR}/out/${BRANCH}})
mkdir -p ${OUT_DIR}

export KERNEL_DIR=$(readlink -m ${KERNEL_DIR})

echo "========================================================"
echo " Building kernel and running tests "
echo "    Using KERNEL_DIR: " ${KERNEL_DIR}
echo "    Using OUT_DIR   : " ${OUT_DIR}

cd ${OUT_DIR}
$NET_TEST/run_net_test.sh --builder $test

echo "======Finished running tests======"
