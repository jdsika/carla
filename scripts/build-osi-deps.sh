#!/usr/bin/env bash
# --------------------------------------------------------------------------
# CARLA — Phase A: Build and install OSI dependencies
#
# Builds asam-osi-utilities (and its transitive deps: protobuf, lz4, zstd)
# into a self-contained install prefix that Phase B (CARLA CMake configure)
# consumes via find_package().
#
# Usage:
#   ./scripts/build-osi-deps.sh [--config Release|Debug] [--prefix <path>]
#                                [--triplet <vcpkg-triplet>] [--clean]
#
# Environment:
#   VCPKG_ROOT — path to vcpkg checkout (required unless --no-vcpkg)
#
# Output:
#   <prefix>/        — installed OSIUtilities + OSI CMake configs and libs
#   <build-dir>/vcpkg_installed/<triplet>/  — vcpkg-managed dependencies
# --------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OSI_SRC="${CARLA_ROOT}/externals/asam-osi-utilities"

# Defaults
CONFIG="Release"
PREFIX="${CARLA_ROOT}/Build/osi-deps"
BUILD_DIR="${CARLA_ROOT}/Build/osi-build"
TRIPLET=""
CLEAN=0

# --------------------------------------------------------------------------
# Parse arguments
# --------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)   CONFIG="$2"; shift 2 ;;
    --prefix)   PREFIX="$2"; shift 2 ;;
    --triplet)  TRIPLET="$2"; shift 2 ;;
    --clean)    CLEAN=1; shift ;;
    -h|--help)
      echo "Usage: $0 [--config Release|Debug] [--prefix <path>] [--triplet <triplet>] [--clean]"
      exit 0
      ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

# --------------------------------------------------------------------------
# Validate
# --------------------------------------------------------------------------
if [[ ! -f "${OSI_SRC}/CMakeLists.txt" ]]; then
  echo "ERROR: asam-osi-utilities submodule not found at ${OSI_SRC}"
  echo "Run:  git submodule update --init --recursive externals/asam-osi-utilities"
  exit 1
fi

if [[ -z "${VCPKG_ROOT:-}" ]]; then
  echo "ERROR: VCPKG_ROOT is not set. Set it to your vcpkg installation."
  echo "  e.g.: export VCPKG_ROOT=\$HOME/vcpkg"
  exit 1
fi

if [[ ! -f "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" ]]; then
  echo "ERROR: vcpkg toolchain not found at ${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
  exit 1
fi

# --------------------------------------------------------------------------
# Auto-detect triplet if not specified
# --------------------------------------------------------------------------
if [[ -z "${TRIPLET}" ]]; then
  case "$(uname -s)" in
    Linux*)  TRIPLET="x64-linux" ;;
    Darwin*)
      if [[ "$(uname -m)" == "arm64" ]]; then
        TRIPLET="arm64-osx"
      else
        TRIPLET="x64-osx"
      fi
      ;;
    MINGW*|MSYS*|CYGWIN*) TRIPLET="x64-windows-static-md" ;;
    *)       TRIPLET="x64-linux" ;;
  esac
fi

echo "========================================"
echo "  CARLA OSI Dependencies — Phase A"
echo "========================================"
echo "  Source:   ${OSI_SRC}"
echo "  Build:    ${BUILD_DIR}"
echo "  Prefix:   ${PREFIX}"
echo "  Config:   ${CONFIG}"
echo "  Triplet:  ${TRIPLET}"
echo "  VCPKG:    ${VCPKG_ROOT}"
echo "========================================"

# --------------------------------------------------------------------------
# Clean if requested
# --------------------------------------------------------------------------
if [[ "${CLEAN}" -eq 1 ]]; then
  echo "Cleaning previous build..."
  rm -rf "${BUILD_DIR}" "${PREFIX}"
fi

# --------------------------------------------------------------------------
# Phase A: Configure
# --------------------------------------------------------------------------
echo ""
echo ">>> Configuring asam-osi-utilities..."

cmake -S "${OSI_SRC}" \
      -B "${BUILD_DIR}" \
      -G Ninja \
      -DCMAKE_BUILD_TYPE="${CONFIG}" \
      -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" \
      -DVCPKG_TARGET_TRIPLET="${TRIPLET}" \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TESTING=OFF \
      -DBUILD_DOCS=OFF

# --------------------------------------------------------------------------
# Phase A: Build
# --------------------------------------------------------------------------
echo ""
echo ">>> Building asam-osi-utilities..."

cmake --build "${BUILD_DIR}" --config "${CONFIG}" --parallel

# --------------------------------------------------------------------------
# Phase A: Install
# --------------------------------------------------------------------------
echo ""
echo ">>> Installing to ${PREFIX}..."

cmake --install "${BUILD_DIR}" --config "${CONFIG}" --prefix "${PREFIX}"

# --------------------------------------------------------------------------
# Summary
# --------------------------------------------------------------------------
VCPKG_INSTALLED="${BUILD_DIR}/vcpkg_installed/${TRIPLET}"

echo ""
echo "========================================"
echo "  Phase A complete!"
echo "========================================"
echo ""
echo "To build CARLA with OSI support (Phase B):"
echo ""
echo "  cmake -G Ninja -S . -B Build \\"
echo "    --toolchain CMake/Toolchain.cmake \\"
echo "    -DCMAKE_BUILD_TYPE=${CONFIG} \\"
echo "    -DENABLE_OSI=ON \\"
echo "    -DCARLA_OSI_PREFIX=${PREFIX} \\"
echo "    -DCARLA_OSI_VCPKG_INSTALLED=${VCPKG_INSTALLED}"
echo ""
echo "Or use the preset:"
echo ""
echo "  export CARLA_OSI_PREFIX=${PREFIX}"
echo "  export CARLA_OSI_VCPKG_INSTALLED=${VCPKG_INSTALLED}"
echo "  cmake --preset Release  # (after updating presets for OSI)"
echo ""
