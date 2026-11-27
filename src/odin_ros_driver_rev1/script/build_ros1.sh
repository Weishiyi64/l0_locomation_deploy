#!/usr/bin/env bash
set -euo pipefail

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ros_fold is 3 levels up from script location (src/ros_driver/script -> ros_fold)
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

FOUND_SDK_PREFIX=""
FOUND_SDK_LIB=""
CANDIDATE_PREFIXES=()

append_prefix() {
    local value="$1"
    [[ -z "${value}" ]] && return
    for existing in "${CANDIDATE_PREFIXES[@]}"; do
        if [[ "${existing}" == "${value}" ]]; then
            return
        fi
    done
    CANDIDATE_PREFIXES+=("${value}")
}

check_sdk_prefix() {
    local prefix="$1"
    [[ -z "${prefix}" ]] && return 1
    local header="${prefix}/include/odin_lidar_api.h"
    if [[ ! -f "${header}" ]]; then
        return 1
    fi
    local lib=""
    for libdir in lib lib64 lib/x86_64-linux-gnu build/sdk; do
        if [[ -f "${prefix}/${libdir}/libodin_sdk.a" ]]; then
            lib="${prefix}/${libdir}/libodin_sdk.a"
            break
        fi
    done
    if [[ -z "${lib}" ]]; then
        return 1
    fi
    FOUND_SDK_PREFIX="${prefix}"
    FOUND_SDK_LIB="${lib}"
    return 0
}

build_sdk_module() {
    local sdk_dir="$1"
    local install_prefix="$2"
    local build_type="$3"

    echo "========================================"
    echo "Building sdk_api module..."
    echo "Install prefix: ${install_prefix}"
    echo "Build type:     ${build_type}"
    echo "========================================"

    # Keep SDK build artefacts inside the ROS workspace build area (next to the
    # colcon/catkin outputs) instead of polluting the sdk_api source tree.
    local sdk_build_dir="${WORKSPACE_DIR}/build/odin_sdk"

    cmake -S "${sdk_dir}" -B "${sdk_build_dir}" \
        -DCMAKE_BUILD_TYPE="${build_type}" \
        -DCMAKE_INSTALL_PREFIX="${install_prefix}"

    cmake --build "${sdk_build_dir}" -j$(nproc)
    cmake --install "${sdk_build_dir}"

    echo "SDK installed to: ${install_prefix}"
}

# Extract -DCMAKE_BUILD_TYPE=<value> from forwarded args ("$@") so that the
# SDK build picks up the same build type as the catkin build.
# Falls back to Release if the user did not specify one.
detect_build_type() {
    local fallback="${1:-Release}"
    shift || true
    for arg in "$@"; do
        case "${arg}" in
            -DCMAKE_BUILD_TYPE=*)
                echo "${arg#-DCMAKE_BUILD_TYPE=}"
                return 0
                ;;
        esac
    done
    echo "${fallback}"
}

# Cap build parallelism so peak compiler memory stays bounded.
# Several nodes in this package (post_process_node and the other PCL / OpenCV /
# roscpp translation units) need more than 1.5 GB of compiler RSS each, so the
# default "one job per core" can exhaust RAM on 8 GB boards such as Jetson and
# push the whole desktop into swap. Budget ~2 GB per job, leave 2 GB for the
# rest of the system, and never exceed the core count.
default_build_jobs() {
    local cpus mem_gb jobs
    cpus="$(nproc)"
    mem_gb=$(( $(awk '/^MemTotal:/ {print $2}' /proc/meminfo) / 1024 / 1024 ))
    jobs=$(( (mem_gb - 2) / 2 ))
    if (( jobs < 1 )); then
        jobs=1
    fi
    if (( jobs > cpus )); then
        jobs="${cpus}"
    fi
    echo "${jobs}"
}

ensure_sdk() {
    # Check if sdk_api submodule exists
    SDK_API_MODULE="${SCRIPT_DIR}/../module/sdk_api"
    
    # Set default SDK install prefix if not specified
    if [[ -z "${ODIN_SDK_PREFIX:-}" ]]; then
        export ODIN_SDK_PREFIX="${WORKSPACE_DIR}/devel"
    fi
    
    if [[ -f "${SDK_API_MODULE}/CMakeLists.txt" ]] && [[ -f "${SDK_API_MODULE}/sdk/CMakeLists.txt" ]]; then
        # Build and install SDK module first; reuse the same CMAKE_BUILD_TYPE
        # the caller passed to catkin_make (default: Release) so SDK debug
        # symbols match driver debug symbols.
        build_sdk_module "${SDK_API_MODULE}" "${ODIN_SDK_PREFIX}" "${SDK_BUILD_TYPE}"
        return
    fi
    
    # SDK submodule not found, check for pre-installed SDK
    CANDIDATE_PREFIXES=()
    append_prefix "${ODIN_SDK_PREFIX:-}"
    append_prefix "/usr/local"
    append_prefix "/usr"

    for prefix in "${CANDIDATE_PREFIXES[@]}"; do
        if check_sdk_prefix "${prefix}"; then
            export ODIN_SDK_PREFIX="${FOUND_SDK_PREFIX}"
            echo "Using pre-installed ODIN SDK from ${FOUND_SDK_PREFIX} (lib: ${FOUND_SDK_LIB})"
            return
        fi
    done

    cat <<EOF >&2
ODIN SDK not found. Either:
  1. Initialize the sdk_api submodule: git submodule update --init module/sdk_api
  2. Install SDK system-wide (e.g. sudo make install)
  3. Set ODIN_SDK_PREFIX=/path/to/install
EOF
    exit 1
}

if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
    echo "ROS workspace not found under ${WORKSPACE_DIR}/src." >&2
    exit 1
fi

if [[ -n "${ROS1_SETUP:-}" ]]; then
    # shellcheck disable=SC1090
    source "${ROS1_SETUP}"
fi

# Pick up CMAKE_BUILD_TYPE from forwarded args (e.g. `-DCMAKE_BUILD_TYPE=Debug`)
# so the SDK is built with the same configuration as the driver. Explicit
# SDK_BUILD_TYPE env wins.
if [[ -z "${SDK_BUILD_TYPE:-}" ]]; then
    SDK_BUILD_TYPE="$(detect_build_type Release "$@")"
fi
export SDK_BUILD_TYPE

ensure_sdk

if ! command -v catkin_make >/dev/null 2>&1; then
    echo "catkin_make command not found. Source your ROS 1 environment or set ROS1_SETUP=/path/to/setup.bash." >&2
    exit 1
fi

export ROS_VERSION=1

# catkin_make appends its own -j/-l only when neither the forwarded make args
# nor MAKEFLAGS carry them (see handle_make_arguments in catkin/builder.py), so
# exporting MAKEFLAGS here is enough to cap the driver build. Set
# ODIN_BUILD_JOBS=<n> to choose the job count, or export MAKEFLAGS yourself to
# take full control.
if [[ -z "${MAKEFLAGS:-}" ]]; then
    ODIN_BUILD_JOBS="${ODIN_BUILD_JOBS:-$(default_build_jobs)}"
    export MAKEFLAGS="-j${ODIN_BUILD_JOBS} -l$(( ODIN_BUILD_JOBS + 1 ))"
    echo "Build parallelism: MAKEFLAGS=${MAKEFLAGS} (override with ODIN_BUILD_JOBS=<n>)"
fi

# Build with all outputs inside WORKSPACE_DIR
cd "${WORKSPACE_DIR}"
catkin_make install \
    --directory "${WORKSPACE_DIR}" \
    --build "${WORKSPACE_DIR}/build" \
    "$@"

echo "Build complete. Artifacts in ${WORKSPACE_DIR}/{build,devel,install}"
