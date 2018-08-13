# source this file. Don't run it.
#
# Usage:
#   source build/envsetup.sh
#     to setup your path and cross compiler so that a kernel build command is
#     just:
#       make -j24


# TODO: Use a $(gettop) style method.
export ROOT_DIR=$PWD

export BUILD_CONFIG=${BUILD_CONFIG:-build.config}
. ${ROOT_DIR}/${BUILD_CONFIG}

# Mitigate dup paths
PATH=${PATH//"${ROOT_DIR}/${LINUX_GCC_CROSS_COMPILE_PREBUILTS_BIN}:"}

export PATH=${ROOT_DIR}/${LINUX_GCC_CROSS_COMPILE_PREBUILTS_BIN}:${PATH}
export $(sed -n -e 's/\([^=]\)=.*/\1/p' ${ROOT_DIR}/${BUILD_CONFIG})

