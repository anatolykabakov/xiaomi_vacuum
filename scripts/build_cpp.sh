#!/usr/bin/env bash
# C++ build on host: Conan 2 + CMake (Ninja) from repository root.
# Also used from build_cpp_docker.sh (WORKDIR=/src) and build.sh.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
BUILD_TESTING="${BUILD_TESTING:-ON}"
BUILD_PYBIND="${BUILD_PYBIND:-OFF}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"
RUN_CTEST=false
CLEAN=true
CONAN_EXTRA=()

usage() {
  cat <<'EOF'
Usage: scripts/build_cpp.sh [options]

Environment:
  BUILD_TYPE     Debug or Release (default: Debug)
  BUILD_TESTING  ON or OFF (default: ON) — Conan &:build_testing and -DBUILD_TESTING
  BUILD_PYBIND    ON or OFF (default: OFF) — Conan &:build_pybind and -DBUILD_PYBIND
  JOBS           parallel compile jobs (default: nproc)

Options:
  --clean           remove build/<BUILD_TYPE> before Conan/CMake
  --build-type T    Debug|Release
  --no-test         skip ctest after build
  --run-tests       run ctest when BUILD_TESTING is ON (default; use after --no-test to re-enable)
  --profile:host F         Conan --profile:host (repeatable)
  --profile:build F        Conan --profile:build (repeatable)
  --conan-host-profile F     same as --profile:host
  --conan-build-profile F    same as --profile:build
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN=true
      shift
      ;;
    --build-type)
      BUILD_TYPE="${2:?--build-type needs a value}"
      shift 2
      ;;
    --no-test)
      RUN_CTEST=false
      shift
      ;;
    --run-tests)
      RUN_CTEST=true
      shift
      ;;
    --profile:host)
      CONAN_EXTRA+=(--profile:host "${2:?}")
      shift 2
      ;;
    --profile:build)
      CONAN_EXTRA+=(--profile:build "${2:?}")
      shift 2
      ;;
    --conan-host-profile)
      CONAN_EXTRA+=(--profile:host "${2:?}")
      shift 2
      ;;
    --conan-build-profile)
      CONAN_EXTRA+=(--profile:build "${2:?}")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

case "${BUILD_TYPE}" in
  Debug | Release) ;;
  *)
    echo "BUILD_TYPE must be Debug or Release (got: ${BUILD_TYPE})" >&2
    exit 1
    ;;
esac

cd "${ROOT_DIR}"

BUILD_DIR="${ROOT_DIR}/build/${BUILD_TYPE}"
TOOLCHAIN="${BUILD_DIR}/generators/conan_toolchain.cmake"

if [[ "${CLEAN}" == true ]]; then
  echo "[build] clean: ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

CONAN_TEST_OPT="&:build_testing=False"
CMAKE_TESTING=OFF
case "${BUILD_TESTING}" in
  ON | on | 1 | true | TRUE | yes | YES)
    CONAN_TEST_OPT="&:build_testing=True"
    CMAKE_TESTING=ON
    ;;
  OFF | off | 0 | false | FALSE | no | NO) ;;
  *)
    echo "BUILD_TESTING must be ON or OFF (got: ${BUILD_TESTING})" >&2
    exit 1
    ;;
esac

CONAN_PYBIND_OPT="&:build_pybind=False"
CMAKE_PYBIND=OFF
case "${BUILD_PYBIND}" in
  ON | on | 1 | true | TRUE | yes | YES)
    CONAN_PYBIND_OPT="&:build_pybind=True"
    CMAKE_PYBIND=ON
    ;;
  OFF | off | 0 | false | FALSE | no | NO) ;;
  *)
    echo "BUILD_PYBIND must be ON or OFF (got: ${BUILD_PYBIND})" >&2
    exit 1
    ;;
esac

echo "[build] Conan install (build_type=${BUILD_TYPE}, ${CONAN_TEST_OPT}, ${CONAN_PYBIND_OPT})"
conan install "${ROOT_DIR}/cpp/conan" \
  --output-folder="${ROOT_DIR}" \
  --build=missing \
  -s "build_type=${BUILD_TYPE}" \
  -o "${CONAN_TEST_OPT}" \
  -o "${CONAN_PYBIND_OPT}" \
  "${CONAN_EXTRA[@]}"

echo "[build] CMake configure + Ninja (${JOBS} jobs)"
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -G Ninja \
  "-DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN}" \
  "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
  "-DBUILD_TESTING=${CMAKE_TESTING}" \
  "-DBUILD_PYBIND=${CMAKE_PYBIND}" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

if [[ "${RUN_CTEST}" == true && "${CMAKE_TESTING}" == ON ]]; then
  echo "[build] ctest"
  ctest --test-dir "${BUILD_DIR}" --output-on-failure
fi

if [[ "${CMAKE_PYBIND}" == ON ]]; then
  echo "[build] pybind: ${BUILD_DIR}/cpp/pybind/pyxiaomi_robot*.so"
fi
echo "[build] Done: ${BUILD_DIR}/cpp/app/xiaomi_robot"
