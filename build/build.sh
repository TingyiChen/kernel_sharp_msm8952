#!/bin/bash

# Usage:
#   build/build.sh <make options>*
# or:
#   OUT_DIR=<out dir> DIST_DIR=<dist dir> build/build.sh <make options>*
#
# Example:
#   OUT_DIR=output DIST_DIR=dist build/build.sh -j24

set -e

export ROOT_DIR=$(readlink -f $(dirname $0)/..)

source "${ROOT_DIR}/build/envsetup.sh"

export MAKE_ARGS=$@
export OUT_DIR=$(readlink -m ${OUT_DIR:-${ROOT_DIR}/out/${BRANCH}})
export DIST_DIR=$(readlink -m ${DIST_DIR:-${OUT_DIR}/dist})

cd ${ROOT_DIR}

archsubarch="ARCH=${ARCH}"
if [ -n "$SUBARCH" ]; then
  archsubarch="${archsubarch} SUBARCH=${SUBARCH}"
fi

mkdir -p ${OUT_DIR}
echo "========================================================"
echo " Setting up for build"
set -x
(cd ${KERNEL_DIR} && \
 make O=${OUT_DIR} $archsubarch CROSS_COMPILE=${CROSS_COMPILE} mrproper && \
 make O=${OUT_DIR} $archsubarch CROSS_COMPILE=${CROSS_COMPILE} ${DEFCONFIG})
set +x

echo "========================================================"
echo " Building kernel"
set -x
(cd ${OUT_DIR} && \
 make O=${OUT_DIR} $archsubarch CROSS_COMPILE=${CROSS_COMPILE} -j8 $@)
set +x

if [ "${EXTRA_CMDS}" != "" ]; then
  echo "========================================================"
  echo " Running extra build command(s):"
  set -x
  eval ${EXTRA_CMDS}
  set +x
fi

OVERLAYS_OUT=""
for ODM_DIR in ${ODM_DIRS}; do
  OVERLAY_DIR=${ROOT_DIR}/device/${ODM_DIR}/overlays

  if [ -d ${OVERLAY_DIR} ]; then
    OVERLAY_OUT_DIR=${OUT_DIR}/overlays/${ODM_DIR}
    mkdir -p ${OVERLAY_OUT_DIR}
    make -C ${OVERLAY_DIR} DTC=${OUT_DIR}/scripts/dtc/dtc OUT_DIR=${OVERLAY_OUT_DIR}
    OVERLAYS=$(find ${OVERLAY_OUT_DIR} -name "*.dtbo")
    OVERLAYS_OUT="$OVERLAYS_OUT $OVERLAYS"
  fi
done

mkdir -p ${DIST_DIR}
echo "========================================================"
echo " Copying files"
for FILE in ${FILES}; do
  echo "  $FILE"
  cp ${OUT_DIR}/${FILE} ${DIST_DIR}/
done

for FILE in ${OVERLAYS_OUT}; do
  OVERLAY_DIST_DIR=${DIST_DIR}/$(dirname ${FILE#${OUT_DIR}/overlays/})
  echo "  ${FILE#${OUT_DIR}/}"
  mkdir -p ${OVERLAY_DIST_DIR}
  cp ${FILE} ${OVERLAY_DIST_DIR}/
done

if [ -n "${IN_KERNEL_MODULES}" ]; then
  MODULES=$(find ${OUT_DIR} -name "*.ko")
  for FILE in ${MODULES}; do
    echo "  ${FILE#${OUT_DIR}/}"
    cp ${FILE} ${DIST_DIR}
  done
fi

echo "========================================================"
echo " Files copied to ${DIST_DIR}"
