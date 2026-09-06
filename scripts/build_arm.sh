#!/usr/bin/env bash
# ARM-сборка xiaomi_robot ХОСТОВЫМ кросс-gcc 11 (нужен C++17 для adas-middleware) против
# sysroot робота (glibc 2.19). Канонический docker-билд (Trusty, gcc 4.8) C++17 не умеет.
#
# Что тут нестандартного и зачем (см. также doc/CROSS_COMPILE_CONAN_SYSROOT.md):
#  * --sysroot НЕ убирает встроенный /usr/arm-linux-gnueabihf/include (хостовая glibc 2.35),
#    и он ищется РАНЬШЕ sysroot -> -nostdinc + явный порядок: libstdc++ -> gcc -> sysroot.
#  * libstdc++ gcc-11 (и conan-зависимости) ссылаются на glibc>=2.25..2.32:
#    __libc_single_threaded, getrandom, pthread_cond_clockwait, pthread_mutex_clocklock,
#    __explicit_bzero_chk -> cpp/cmake/glibc_compat/{glibc_compat.c,compat_decls.h}.
#  * libstdc++/libgcc линкуем статически: на роботе libstdc++ от gcc 4.8.
#  * protoc для кодогенерации берём из conan build-context (x86_64, та же версия, что
#    заголовки protobuf), иначе подхватывается системный /usr/bin/protoc и .pb.* несовместимы.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SR="${ROBOT_SYSROOT:-$ROOT/sysroot}"
B="${BUILD_DIR:-$ROOT/build-arm}"
JOBS="${JOBS:-$(nproc)}"
DEPLOY="${1:-}"   # --deploy => scp на робота

[ -d "$SR/usr/include" ] || { echo "нет sysroot: $SR" >&2; exit 1; }
command -v arm-linux-gnueabihf-g++ >/dev/null || { echo "нет arm-linux-gnueabihf-g++" >&2; exit 1; }

# 1. conan-зависимости под armv7 (пересобирает недостающее)
TC="$B/conan/build/Release/generators/conan_toolchain.cmake"
if [ ! -f "$TC" ]; then
  echo "[arm] conan install"
  conan install "$ROOT/cpp/conan/conanfile.py" -of "$B/conan" --build=missing -s build_type=Release \
    -pr:h="$ROOT/cpp/conan/profiles/armv7.profile" -pr:b="$ROOT/cpp/conan/profiles/build-linux-x86_64.profile" \
    -o '&:build_testing=False' -o '&:build_pybind=False'
fi
# хостовый protoc из build-context пакета protobuf_BUILD
PF=$(grep -oE 'set\(protobuf_BUILD_PACKAGE_FOLDER_RELEASE "[^"]+"' "$B/conan/build/Release/generators/module-Protobuf_BUILD-release-armv7-data.cmake" | sed 's/.*"\(.*\)"/\1/')
PROTOC="$PF/bin/protoc"; "$PROTOC" --version >/dev/null

# 2. порядок include без хостовых arm-заголовков
GCCV=$(arm-linux-gnueabihf-g++ -dumpversion | cut -d. -f1)
GCCINC=/usr/lib/gcc-cross/arm-linux-gnueabihf/$GCCV/include
CXXINC=/usr/arm-linux-gnueabihf/include/c++/$GCCV
CINC="-nostdinc -isystem $GCCINC -isystem $SR/usr/include/arm-linux-gnueabihf -isystem $SR/usr/include"
CXXINCS="-nostdinc -isystem $CXXINC -isystem $CXXINC/arm-linux-gnueabihf -isystem $CXXINC/backward -isystem $GCCINC -isystem $SR/usr/include/arm-linux-gnueabihf -isystem $SR/usr/include"
COMPAT="$ROOT/cpp/cmake/glibc_compat"

# 3. шим совместимости с glibc 2.19
mkdir -p "$B"
arm-linux-gnueabihf-gcc --sysroot="$SR" $CINC -O2 -c "$COMPAT/glibc_compat.c" -o "$B/glibc_compat.o"

# 4. configure + build
LD="--sysroot=$SR -B$SR/usr/lib/arm-linux-gnueabihf -static-libstdc++ -static-libgcc -no-pie $B/glibc_compat.o -lpthread"
cmake -S "$ROOT" -B "$B" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$TC" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PYBIND=OFF \
  -DXIAOMI_ROBOT_ENABLE_DRIVERS=ON -DProtobuf_PROTOC_EXECUTABLE="$PROTOC" \
  -DCMAKE_C_FLAGS="--sysroot=$SR $CINC" -DCMAKE_CXX_FLAGS="--sysroot=$SR $CXXINCS -include $COMPAT/compat_decls.h" \
  -DCMAKE_EXE_LINKER_FLAGS="$LD" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build "$B" --parallel "$JOBS"

# 5. проверка совместимости с роботом
BIN="$B/cpp/app/xiaomi_robot"
echo "[arm] $(file -b "$BIN" | cut -d, -f1-2)"
echo "[arm] max GLIBC: $(arm-linux-gnueabihf-readelf -V "$BIN" | grep -oE 'GLIBC_2\.[0-9]+' | sort -t. -k2 -n | tail -1)  (робот: 2.19)"
echo "[arm] GLIBCXX:   $(arm-linux-gnueabihf-readelf -V "$BIN" | grep -oE 'GLIBCXX_[0-9.]+' | sort -V | tail -1 || echo 'нет (статический)')"

# 6. деплой: бинарь в /mnt/data (корень робота почти полон), симлинк /opt/xiaomi_robot
if [ "$DEPLOY" = "--deploy" ]; then
  IP="${ROBOT_IP:-192.168.0.137}"
  echo "[arm] deploy -> $IP"
  scp "$BIN" "root@$IP:/mnt/data/xiaomi_robot.new"
  ssh "root@$IP" 'pkill -9 -x xiaomi_robot 2>/dev/null; chmod +x /mnt/data/xiaomi_robot.new && mv -f /mnt/data/xiaomi_robot.new /mnt/data/xiaomi_robot && rm -f /opt/xiaomi_robot && ln -s /mnt/data/xiaomi_robot /opt/xiaomi_robot && ls -la /opt/xiaomi_robot'
fi
echo "[arm] Done: $BIN"
