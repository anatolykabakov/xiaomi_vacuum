# System design

Как устроен репозиторий и по каким правилам он написан. Что делает система — в
[`ARCHITECTURE.md`](ARCHITECTURE.md); где не едет — в [`ROBOT_LIMITS.md`](ROBOT_LIMITS.md).

## Раскладка

```
cpp/
  app/
    main.cpp                  точка входа: конфиг, логирование, RobotApp
    include/
      types.h                 типы сообщений внутренней шины (структуры C++)
      utils/topics.h          имена топиков — единственное место объявления
      robot_config.h          геометрия и лимиты робота (из ruby_chassis.cfg)
      robot_app.h             сборка стека на менеджере
      services/*.h            Platform, Slam, Mission, Planner, Control, Safety, Log, ZmqBridge
      algo/*.h                чистые алгоритмы: occupancy_mapping, frontier_explorer,
                              coverage_planner, path_planner, pure_pursuit
      drivers/*.h             клиент Player (libplayerc), только при XIAOMI_ROBOT_ENABLE_DRIVERS
    src/                      зеркало include/
  mw/include/mw/              middleware: manager.hpp (сервисы, топики, таймеры, параметры), logger.h
  cmake/glibc_compat/         шим совместимости gcc-11 ↔ glibc 2.19 робота (только ARM-сборка)
  conan/                      conanfile и профили (armv7, build-linux-x86_64)
  pybind/                     биндинги алгоритмов и типов — pyxiaomi_robot, для симулятора и тестов
  tests/                      gtest: алгоритмы
proto/                        protobuf только для ZMQ-границы
scripts/
  build_arm.sh                кросс-сборка под робота (+ --deploy)
  build_cpp.sh                нативная сборка с тестами (conan + cmake), BUILD_PYBIND=ON для биндингов
  start.sh                    весь стек на роботе: start | stop | status, локально и по ssh
  lidar_up.sh                 поднять стрим лидара и читать его
  sim/                        2D-симулятор (лазер на PNG-карте) и run_cycle.py — полный цикл уборки
tools/
  bag_record.cpp              запись ZMQ-топиков робота в файл с хоста
  drive_zmq.c                 движение через ZMQ с обратной связью по одометрии
docs/                         эта документация
sysroot/                      корневая ФС робота для кросс-сборки (не в git)
```

## Соглашения

- **Сервис** = класс от `mw::middleware::Service` в `services/`, один файл на сервис.
  Переопределяет `getName()` (имя в статистике и для параметров) и `configure()`, где и только где
  объявляются подписки (`subscribe<T>`), таймеры (`scheduleTimer`, мс) и параметры
  (`registerParameter`). У сервиса свой поток; колбэки не пересекаются.
- **Топики** — только через `topics::k*`, литералы в сервисах запрещены. Один издатель на топик,
  кроме `controls/cmd_vel` (Control и телеоперация; действует последняя команда) и `planner/goal`
  (Mission и внешняя цель по ZMQ).
- **Типы** — структуры в `types.h`, у каждого сообщения `timestamp` в микросекундах часов шины
  (`now()`). Protobuf — только в `ZmqBridge`: на ARM структуры дешевле.
- **Алгоритмы** без middleware лежат в `algo/`, собираются в `xiaomi_robot_algo`, покрываются gtest
  и выведены в биндинги. Сервисы их только вызывают. Так весь цикл уборки проверяется в симуляторе
  без робота, а на робота едет тот же код.
- **Параметры** именуются с префиксом сервиса: `platform_*`, `slam_*`, `mission_*`, `plan_*`,
  `ctrl_*`, `safety_*`.
- **Логи** — spdlog в `/opt/xiaomi_robot.log`, теги `LASER/ODOM/IMU/CMD_VEL/POSE/MAP/TRAJ/PLANNER/MISSION/STATE/SAFETY`.
  Скрипты (`start.sh status`, `drive_zmq`) разбирают `LASER`, `ODOM`, `CMD_VEL` — не переименовывать.

## Сборка

| что | команда | заметки |
|---|---|---|
| нативно + тесты + биндинги | `BUILD_PYBIND=ON scripts/build_cpp.sh` | `DRIVERS=OFF`, Platform не собирается |
| под робота | `scripts/build_arm.sh [--deploy]` | хостовый gcc-11 против sysroot робота, C++17 |

Требование C++17 — от менеджера (`std::optional`, `variant`, `string_view`). Детали кросс-сборки —
в заголовке `scripts/build_arm.sh` и [`CROSS_COMPILE_CONAN_SYSROOT.md`](CROSS_COMPILE_CONAN_SYSROOT.md).

## Запуск

Симулятор: `python3 scripts/sim/run_cycle.py` — [`SIMULATION.md`](SIMULATION.md).
Робот: `scripts/start.sh start 192.168.0.137` — почему именно такой порядок, в
[`PLATFORM.md`](PLATFORM.md); чек-лист — [`PREDRIVE.md`](PREDRIVE.md).
