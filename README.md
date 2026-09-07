# xiaomi_vacuum

Автономный стек для робота-пылесоса Roborock поверх штатного Player: лидар → карта и поза →
цель → траектория → команда на колёса. Цикл уборки: построить карту, исследовать помещение,
пройти змейкой всю свободную площадь, вернуться на базу. Сервисы `Platform`, `Slam`, `Mission`,
`Planner`, `Control`, `Safety` на общем middleware; тот же цикл целиком гоняется в симуляторе.

Документация — [`docs/README.md`](docs/README.md): начать с [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
и [`docs/ROBOT_LIMITS.md`](docs/ROBOT_LIMITS.md).

## Сборка

```bash
# нативно, с тестами и биндингами (Platform не собирается: нужен libplayerc)
./scripts/build_cpp.sh
./build/Debug/cpp/tests/xiaomi_robot_test

# под робота (ARM, glibc 2.19): хостовый кросс-gcc против sysroot робота
./scripts/build_arm.sh            # бинарь: build-arm/cpp/app/xiaomi_robot
./scripts/build_arm.sh --deploy   # + scp на робота: /mnt/data/xiaomi_robot, симлинк /opt/xiaomi_robot
```

Требуется C++17. Рецепт и все особенности кросс-сборки — в заголовке `scripts/build_arm.sh`.

## Симулятор

Полный цикл уборки на синтетической комнате, через биндинги к тем же C++-алгоритмам, что едут
на роботе:

```bash
BUILD_PYBIND=ON ./scripts/build_cpp.sh
PYTHONPATH=build/Debug/cpp/pybind python3 scripts/sim/run_cycle.py --out /tmp/cycle
```

Печатает метрики и возвращает 0, если цикл пройден: карта построена, площадь покрыта, робот
у дока, бампер срабатывал не чаще допустимого.
Подробно — [`docs/SIMULATION.md`](docs/SIMULATION.md).

## Запуск на роботе

```bash
./scripts/start.sh start  192.168.0.137   # RoboController → ручной режим → keepalive → xiaomi_robot
./scripts/start.sh status 192.168.0.137
./scripts/start.sh stop   192.168.0.137   # снять режим, вернуть на базу, погасить
```

Тот же скрипт работает на роботе локально (`/opt/start.sh start`). Чек-лист перед запуском —
[`docs/PREDRIVE.md`](docs/PREDRIVE.md). Почему без этой последовательности робот не едет и лидар
пуст — [`docs/PLATFORM.md`](docs/PLATFORM.md).

## Интерфейсы наружу (ZMQ, protobuf `proto/`)

- **PUB `tcp://robot:9091`** — `slam/map`, `slam/pose`, `robot/state`, `planner/trajectory`,
  `planner/state`, `mission/state`
- **SUB `tcp://robot:9090`** — `ZmqMessage{mission}` старт и стоп уборки; `ZmqMessage{goal}` цель
  планировщику; `ZmqMessage{cmd_vel}` телеоперация

Записать поток с хоста — `tools/bag_record --host 192.168.0.137 --out run.bag`
([`docs/BAGS.md`](docs/BAGS.md)). Пример движения через ZMQ с обратной связью по одометрии —
`tools/drive_zmq.c`.

## Lint & Static analysis

```bash
sudo apt install pre-commit cmake-format clang-format
pip install pre-commit
git ls-files -- . | xargs pre-commit run --files
python3 scripts/run_clang_tidy.py -p build -clang-tidy-binary /usr/bin/clang-tidy-20 '.*/cpp/.*\.cpp$' 2>&1 | tee tidy.log
```

## Debug

```bash
ulimit -c unlimited
/opt/xiaomi_robot --config /opt/config.json
scp root@192.168.0.137:/mnt/data/rockrobo/rrlog/core.*.xiaomi_robot .
sudo apt install gdb-multiarch
gdb-multiarch build-arm/cpp/app/xiaomi_robot core.*.xiaomi_robot
```
