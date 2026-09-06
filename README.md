

# Build & Run
```bash
git submodule update --init --recursive

# build host to run tests
./scripts/build_cpp.sh --clean
# cross-compile ARM binary for robot (deploy via scp)
./scripts/build_cpp_docker.sh --clean

./scripts/build_android_docker.sh
```

# Deploy
```bash
adb uninstall com.xiaomi.robotjoy

scp build/Release/cpp/app/xiaomi_robot cpp/config/config.json root@192.168.0.137:/opt
ssh root@192.168.0.137
/opt/xiaomi_robot --config /opt/config.json
./build/cpp/app/xiaomi_robot --config cpp/config/config.json
```


## Lint & Static analysis
```bash
sudo apt install pre-commit
pip install pre-commit
sudo apt install cmake-format clang-format
git ls-files -- . | xargs pre-commit run --files

python3 scripts/run_clang_tidy.py -p build -clang-tidy-binary /usr/bin/clang-tidy-20 '.*/cpp/.*\.cpp$' 2>&1 | tee tidy.log
```

# Debug
```
ulimit -c unlimited
/opt/xiaomi_robot --config /opt/config.json
ls -la /mnt/data/rockrobo/rrlog/core.*
cp /mnt/data/rockrobo/rrlog/core.*.xiaomi_robot .
scp root@192.168.0.137:/root/core.1778869403.11.xiaomi_robot .

sudo apt install gdb-multiarch
gdb-multiarch build/Release/cpp/app/xiaomi_robot core.1778869403.11.xiaomi_robot
(gdb) bt full
(gdb) thread apply all bt full
```

## Связанные проекты (GitHub)

- [codetiger/VacuumRobot — lidar-reader](https://github.com/codetiger/VacuumRobot/blob/main/Research/Software/Firmware/lidar-reader/README.md) — протокол **Delta‑2D** по UART, в README — идея **GPIO для мотора LDS** (на части прошивок нет `/sys/class/gpio`).
- [arne48/xiaomi_bridge](https://github.com/arne48/xiaomi_bridge) — **ROS**-мост к внутреннему **Player** на роботе (`/scan`, `/cmd_vel` и т.д.); типичный обход отсутствия sysfs-GPIO — оставить поднятым стек/сенсоры через штатные процессы.
- [PaulTerrasi/Xiaomi_Mi_ROS](https://github.com/PaulTerrasi/Xiaomi_Mi_ROS) — **SSH/root**, установка **ROS**, визуализация лидара (Gen1).
- [dgiese/dustcloud](https://github.com/dgiese/dustcloud) — модификация прошивок, **root**, материалы по железу/прошивкам Xiaomi vacuum.
- [marcelrv/XiaomiRobotVacuumProtocol](https://github.com/marcelrv/XiaomiRobotVacuumProtocol) — описание **miIO / облачного** слоя (не низкоуровневый UART к лидару, но полезно для верхнего уровня).
- [salihmarangoz/xiaomi_lds_myhome](https://github.com/salihmarangoz/xiaomi_lds_myhome) — эксперименты со **LDS Gen1** и ROS; автор описывает **внешнее питание мотора** (Arduino), исходники драйвера в репозитории не публиковал.
