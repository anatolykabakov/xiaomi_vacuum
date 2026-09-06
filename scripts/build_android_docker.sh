#!/usr/bin/env bash
# Сборка APK через Docker (JDK 17 + Android SDK + Gradle) или на хосте (--host).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="xiaomi-robot-android-build"
DOCKERFILE="$ROOT_DIR/docker/Dockerfile.android"
SRC_DIR_INNER="/workspace"
ANDROID_REL="android"
CLEAN=0
USE_DOCKER=1
GRADLE_TASKS=(assembleDebug)

usage() {
  cat <<'EOF'
Usage:
  ./scripts/build_android_docker.sh [--clean] [--host] [--release]

Docker (default): образ xiaomi-robot-android-build из docker/Dockerfile.android,
репозиторий монтируется в /workspace, Gradle запускается в android/.

APK (debug): android/app/build/outputs/apk/debug/app-debug.apk
APK (release, без подписи): android/app/build/outputs/apk/release/app-release-unsigned.apk

  --clean    Удалить android/app/build, android/build, android/.gradle в корне проекта
  --host     Собрать на этой машине (нужны JAVA_HOME, ANDROID_SDK_ROOT или ANDROID_HOME)
  --release  assembleRelease вместо assembleDebug
  -h, --help Эта справка
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN=1
      shift
      ;;
    --host)
      USE_DOCKER=0
      shift
      ;;
    --release)
      GRADLE_TASKS=(assembleRelease)
      shift
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

if [[ $CLEAN -eq 1 ]]; then
  rm -rf \
    "$ROOT_DIR/$ANDROID_REL/app/build" \
    "$ROOT_DIR/$ANDROID_REL/build" \
    "$ROOT_DIR/$ANDROID_REL/.gradle"
fi

run_gradle() {
  (cd "$ROOT_DIR/$ANDROID_REL" && exec gradle --no-daemon "${GRADLE_TASKS[@]}")
}

docker run --rm "${DOCKER_TTY[@]}" \
  -u "$(id -u):$(id -g)" \
  -e GRADLE_USER_HOME=/workspace/.gradle-android-docker \
  -v "$ROOT_DIR:$SRC_DIR_INNER" \
  -w "$SRC_DIR_INNER/$ANDROID_REL" \
  "$IMAGE" \
  gradle --no-daemon "${GRADLE_TASKS[@]}"

echo "[android] APK: $ROOT_DIR/$ANDROID_REL/app/build/outputs/apk/debug/app-debug.apk"
echo "adb install -r $ROOT_DIR/$ANDROID_REL/app/build/outputs/apk/debug/app-debug.apk"
