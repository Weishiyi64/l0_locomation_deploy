#!/bin/bash
# Generate public package for ros_driver
# Excludes .git, build directories and compiled files
# Usage: ./genpubpkg.sh [beta|release]

set -e
export TZ=Asia/Shanghai

# Parse arguments
BUILD_TYPE="${1:-release}"
if [[ "$BUILD_TYPE" != "beta" && "$BUILD_TYPE" != "release" ]]; then
    echo "Usage: $0 [beta|release]"
    echo "  beta    - Beta version package"
    echo "  release - Release version package (default)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Derive the package name from the actual folder name so the output package
# follows whatever the driver directory is renamed to, instead of a hardcoded name.
PACKAGE_NAME="$(basename "$SCRIPT_DIR")"
VERSION=$(grep -oP '(?<=<version>)[^<]+' "$SCRIPT_DIR/package.xml" 2>/dev/null || echo "unknown")
DATE=$(date +%Y%m%d%H%M%S)
OUTPUT_NAME="${PACKAGE_NAME}_v${VERSION}_${BUILD_TYPE}_${DATE}"
OUTPUT_FILE="$(dirname "$SCRIPT_DIR")/${OUTPUT_NAME}.tar.xz"

echo "=== Generating public package ==="
echo "  Source:  $SCRIPT_DIR"
echo "  Version: $VERSION"
echo "  Type:    $BUILD_TYPE"
echo "  Output:  $OUTPUT_FILE"

# Create temp directory
TEMP_DIR=$(mktemp -d)
TEMP_PKG="$TEMP_DIR/$PACKAGE_NAME"

echo "  Copying files..."

# Copy ros_driver to temp, excluding unwanted files.
#
# module/ is packaged as a WHITELIST: only entries listed in MODULE_INCLUDE
# are copied (e.g. odinsdkstresstool is a dev/stress-test tool and must never
# ship in a public package). To ship another module, add it to the list.
MODULE_INCLUDE=(
    "sdk_api"
)
MODULE_FILTERS=()
for entry in "${MODULE_INCLUDE[@]}"; do
    MODULE_FILTERS+=("--include=/module/${entry}/")
done
MODULE_FILTERS+=("--exclude=/module/*")

# NOTE: this release ships NO prebuilt device plugins. libodin-device-*.so
# (e.g. Odin2Special) is excluded by the generic '*.so' rules below and is
# expected to be built from source in the package. To switch back to binary
# plugin distribution, re-add BEFORE the '*.so' excludes:
#   --include='/module/sdk_api/lib/*/libodin-device-*.so*'
# (rsync applies the first matching pattern; later '*.so' excludes win).
# Do NOT put comments inside the rsync continuation — a '#' there ends the
# command line and silently drops the remaining args.
# NOTE: vendored OpenSSL prebuilt static libs (libssl.a / libcrypto.a) under
# module/sdk_api/lib/<arch>/ are DELIVERABLES and must ship in the package.
# Without them the customer-side CMake would fall back to ExternalProject
# source-build of OpenSSL (needs perl/make), breaking the "link directly,
# zero compile" contract. The include below must come BEFORE the generic
# '*.a' exclude — rsync picks the first matching filter.
rsync -a --progress \
    --exclude='.git' \
    --exclude='.git*' \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='devel/' \
    --exclude='*.tar.xz' \
    --exclude='*.tar.gz' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.cache' \
    --exclude='CMakeCache.txt' \
    --exclude='CMakeFiles/' \
    --exclude='Makefile' \
    --exclude='cmake_install.cmake' \
    --exclude='*.o' \
    --include='/module/sdk_api/lib/*/*.a' \
    --exclude='*.a' \
    --exclude='*.so' \
    --exclude='*.so.*' \
    --exclude='*.deb' \
    "$SCRIPT_DIR/" "$TEMP_PKG/"

# ---------------------------------------------------------------------------
# Generate COMMIT_INFO.txt
# Captures git commit IDs for ros_driver, sdk_api and protocol submodules.
# ---------------------------------------------------------------------------
# Device plugins are built from source in this release (see the *.so note
# above), so Odin2Special support depends on its submodule being checked out
# under sdk/device/Odin2Special at packaging time. If the source is missing
# the package silently loses Odin2Special support: DeviceLoader then falls
# back to builtin Odin1Lite for Odin2Special devices.
if [[ ! -f "$SCRIPT_DIR/module/sdk_api/sdk/device/Odin2Special/CMakeLists.txt" ]]; then
    echo "  WARNING: Odin2Special submodule source not checked out;"
    echo "           the package will fall back to builtin Odin1Lite for Odin2Special devices."
fi

echo "  Writing commit info..."

# Helper: print one repo block. Args: <label> <repo_path>
print_repo_info() {
    local label="$1"
    local repo="$2"
    printf '[%s]\n' "$label"
    if ! command -v git &>/dev/null; then
        printf '  (git not found)\n'
        return
    fi
    if ! git -C "$repo" rev-parse HEAD &>/dev/null 2>&1; then
        printf '  (not a git repository or no commits)\n'
        return
    fi
    local commit branch date subject
    commit=$(git -C "$repo" rev-parse HEAD)
    branch=$(git -C "$repo" rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'detached')
    date=$(git -C "$repo" log -1 --format='%ci' HEAD)
    subject=$(git -C "$repo" log -1 --format='%s' HEAD)
    printf '  Commit  : %s\n' "$commit"
    printf '  Branch  : %s\n' "$branch"
    printf '  Date    : %s\n' "$date"
    printf '  Message : %s\n' "$subject"
}

COMMIT_FILE="$TEMP_DIR/COMMIT_INFO.txt"
{
    printf '================================================\n'
    printf '  Package  : %s v%s (%s)\n' "$PACKAGE_NAME" "$VERSION" "$BUILD_TYPE"
    printf '  Generated: %s\n' "$(date '+%Y-%m-%d %H:%M:%S %z')"
    printf '================================================\n'
    printf '\n'
    print_repo_info "ros_driver"          "$SCRIPT_DIR"
    printf '\n'
    print_repo_info "sdk_api"             "$SCRIPT_DIR/module/sdk_api"
    printf '\n'
    print_repo_info "protocol (sdk_api)"  "$SCRIPT_DIR/module/sdk_api/docs/protocol"
    printf '\n'
    print_repo_info "Odin2Special (plugin)" "$SCRIPT_DIR/module/sdk_api/sdk/device/Odin2Special"
    printf '\n'
    printf '================================================\n'
} > "$COMMIT_FILE"

echo "  Compressing..."

# Create tar.xz archive
tar -C "$TEMP_DIR" -cJf "$OUTPUT_FILE" "$PACKAGE_NAME" COMMIT_INFO.txt

# Cleanup temp directory
rm -rf "$TEMP_DIR"

# Show result
SIZE=$(du -h "$OUTPUT_FILE" | cut -f1)
echo ""
echo "=== Package created successfully ==="
echo "  File: $OUTPUT_FILE"
echo "  Size: $SIZE"