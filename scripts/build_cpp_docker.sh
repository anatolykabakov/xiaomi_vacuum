#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="xiaomi-robot-trusty-build"
SRC_DIR_INNER="/src"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_DIR_HOST="$ROOT_DIR/build"
INNER_ARGS=()
CLEAN=0
USE_DOCKER=1
HAS_CONAN_PROFILE=0
# Build machine inside image: x86_64 Trusty, GCC 4.8 + gnu11 (tool_requires / protoc).
DOCKER_BUILD_PROFILE="cpp/conan/profiles/build-linux-x86_64.profile"
# Target: ARMv7 toolchain (see cpp/cmake/toolchain-armv7.cmake in profile).
DOCKER_HOST_PROFILE="cpp/conan/profiles/armv7.profile"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/build_cpp_docker.sh [--clean] [--build-type Release|Debug] [--host]
      [--conan-host-profile PATH] [--conan-build-profile PATH]

Runs scripts/build_cpp.sh (Conan + CMake + Ninja) in Docker or on the host (--host).

Docker: repo mounted at /src, image from docker/Dockerfile.
Default Conan: --profile:host armv7.profile, --profile:build build-linux-x86_64.profile.
Conan cache: .conan2-docker in the repo (not .conan2) so binaries match Trusty glibc.

Note: run the shell script with bash (not execve the file as a binary).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN=1
      shift
      ;;
    --build-type)
      BUILD_TYPE="${2:-Release}"
      shift 2
      ;;
    --host)
      USE_DOCKER=0
      shift
      ;;
    --conan-host-profile|--conan-build-profile)
      HAS_CONAN_PROFILE=1
      INNER_ARGS+=("$1" "${2:?argument required}")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown arg: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

cd "$ROOT_DIR"
export BUILD_TYPE

if [[ "$USE_DOCKER" -eq 1 && "$HAS_CONAN_PROFILE" -eq 0 ]]; then
  INNER_ARGS=(
    --conan-host-profile "$DOCKER_HOST_PROFILE"
    --conan-build-profile "$DOCKER_BUILD_PROFILE"
    "${INNER_ARGS[@]}"
  )
fi

# WORKDIR in container is /src (repo root).
HOST_ARGS=(./scripts/build_cpp.sh)
[[ "$CLEAN" -eq 1 ]] && HOST_ARGS+=(--clean)
HOST_ARGS+=(--build-type "$BUILD_TYPE")
HOST_ARGS+=("${INNER_ARGS[@]}")

if [[ "$USE_DOCKER" -eq 0 ]]; then
  exec bash "${HOST_ARGS[@]}"
fi

NEED_BUILD=false
if ! docker image inspect "${IMAGE}:latest" >/dev/null 2>&1; then
  NEED_BUILD=true
fi
if [[ "$NEED_BUILD" == "true" ]]; then
  echo "[build] building docker image ${IMAGE}"
  docker build -f "$ROOT_DIR/docker/Dockerfile" -t "$IMAGE" "$ROOT_DIR"
fi

DOCKER_TTY=()
if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY=(-it)
fi
# Entrypoint runs as root briefly, adds passwd for HOST_UID (so git clone works), then gosu.
# CONAN_HOME separate from host .conan2: host-built protoc needs newer glibc than Trusty.
docker run --rm "${DOCKER_TTY[@]}" \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  -e HOME="$SRC_DIR_INNER" \
  -e CONAN_HOME="$SRC_DIR_INNER/.conan2-docker" \
  -e BUILD_TYPE="$BUILD_TYPE" \
  -e "BUILD_TESTING=${BUILD_TESTING:-ON}" \
  -e "JOBS=${JOBS:-}" \
  -v "$ROOT_DIR:$SRC_DIR_INNER" \
  -w "$SRC_DIR_INNER" \
  "$IMAGE" \
  bash "${HOST_ARGS[@]}"

echo "[build] Done: ${ROOT_DIR}/build/${BUILD_TYPE}/cpp/app/xiaomi_robot"
