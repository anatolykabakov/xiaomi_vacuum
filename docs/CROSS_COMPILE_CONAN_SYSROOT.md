# Cross-compile через Conan + sysroot (рабочие заметки)

Этот документ фиксирует практический сценарий кросс-компиляции `xiaomi_robot` на хосте с Conan и sysroot с робота, включая типовые проблемы и проверки совместимости.

## Цель

Собрать ARM-бинарь, совместимый с роботом (`Ubuntu 14.04`, glibc 2.19), не собирая проект на роботе.

---

## Что важно про целевую систему

- Робот: `armv7l`, Ubuntu 14.04.3
- Loader: `/lib/ld-linux-armhf.so.3`
- Runtime libc: EGLIBC/GLIBC 2.19

Критично: в итоговом бинаре не должно быть требований `GLIBC_2.28+`.

---

## Подготовка sysroot

Sysroot должен содержать не только `libc`, но и startup/runtime части toolchain (`crtbegin.o`, `crtend.o`, `libgcc`, `libstdc++`).

Пример загрузки с робота:

```bash
mkdir -p sysroot/{lib,usr/lib,usr/include} && \
rsync -av --delete root@192.168.0.137:/lib/ sysroot/lib/ && \
rsync -av --delete root@192.168.0.137:/usr/lib/ sysroot/usr/lib/ && \
rsync -av --delete root@192.168.0.137:/usr/include/ sysroot/usr/include/ && \
rsync -av --delete root@192.168.0.137:/usr/lib/gcc/arm-linux-gnueabihf/ sysroot/usr/lib/gcc/arm-linux-gnueabihf/
```

Проверки:

```bash
ls -la sysroot/usr/lib/gcc/arm-linux-gnueabihf/*/crtbegin*.o
ls -la sysroot/usr/lib/gcc/arm-linux-gnueabihf/*/crtend*.o
```

---

## Conan профили (актуальное состояние)

Используются два профиля:

- `cpp/conan/profiles/armv7.profile` (host profile для target ARM)
- `cpp/conan/profiles/build-linux-x86_64.profile` (build machine profile)

Практически корректная связка для текущего хоста:

- ARM compiler: `arm-linux-gnueabihf-gcc`, `arm-linux-gnueabihf-g++`
- Build compiler: `/usr/bin/gcc`, `/usr/bin/g++`

---

## Команды (host build)

### 1) Conan install

```bash
conan install cpp/conan/conanfile.py \
  -of build/conan \
  --build=missing \
  -pr:h=cpp/conan/profiles/armv7.profile \
  -pr:b=cpp/conan/profiles/build-linux-x86_64.profile
```

### 2) CMake configure

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=build/conan/build/Release/generators/conan_toolchain.cmake \
  -DROBOT_SYSROOT=sysroot \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 3) Build

```bash
cmake --build build -j"$(nproc)"
```

---

## Проверка совместимости бинаря

Проверка формата:

```bash
file build/cpp/app/xiaomi_robot
```

Ожидаемо:

- `ELF 32-bit ... ARM, EABI5`
- `interpreter /lib/ld-linux-armhf.so.3`

Проверка GLIBC:

```bash
arm-linux-gnueabihf-readelf -V build/cpp/app/xiaomi_robot | rg GLIBC_
```

Хороший результат для робота: только `GLIBC_2.4` (или значения не выше 2.19).
Плохой результат: появление `GLIBC_2.28`, `GLIBC_2.32`, `GLIBC_2.34` и выше.

---

## Типовые проблемы и причины

### 1) `Could not find toolchain file: build/conan/conan_toolchain.cmake`

Причина: в Conan 2 toolchain часто генерируется в:

- `build/conan/build/Release/generators/conan_toolchain.cmake`

Использовать именно этот путь в `-DCMAKE_TOOLCHAIN_FILE`.

### 2) Конфликт путей `/src/...` vs локальный workspace

Ошибка вида:

- `CMakeCache.txt ... different than /src/build ...`

Решение: чистить build-кэш (включая `3pp/player-lib/build` и stamp/tmp).

### 3) На роботе не хватает места при установке dev-пакетов

Ошибка:

- `No space left on device`

Решение: `apt-get clean`, очистка `/var/cache/apt/archives`, затем установка минимально нужных пакетов.

### 4) В бинаре появляются новые `GLIBC_*`

Причина: линковка подмешала host runtime/toolchain библиотеки вместо целевого набора.

Решение:

- перепроверить sysroot,
- пересобрать зависимости,
- проверять `readelf -V` после каждой полной пересборки.

---

## Docker путь (fallback/основной стабильный)

Для воспроизводимой сборки в проекте используется:

```bash
./scripts/build.sh --clean
```

Это остается базовым способом, если host cross-build нестабилен.

---

## Деплой

```bash
scp build/cpp/app/xiaomi_robot root@192.168.0.137:/opt/xiaomi_robot
```

Запуск на роботе (пример):

```bash
ssh root@192.168.0.137 "/opt/xiaomi_robot --config /opt/config.json"
```
