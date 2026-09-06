# План: MapNode -> ZMQ/Protobuf -> Android MapView

## Цель

Сделать минимальный end-to-end контур отображения карты:

1. `MapNode` в C++ строит occupancy grid `300x300` по лазеру (Bresenham).
2. Нода публикует карту по TCP через ZMQ в protobuf-сообщении.
3. Android принимает сообщение и отображает карту на экране.

---

## 0. Зафиксировать контракт данных

Создать `proto/map.proto` с сообщением кадра карты (пример полей):

- `uint32 width`
- `uint32 height`
- `float resolution`
- `float origin_x`
- `float origin_y`
- `float robot_x`
- `float robot_y`
- `float robot_yaw`
- `uint64 timestamp_us`
- `bytes cells`

Семантика `cells`:

- `0` = free
- `100` = occupied
- `255` = unknown

---

## 1. C++: MapNode (occupancy 300x300)

### 1.1 Типы и структура

В `cpp/app/include/types.h` добавить структуры для карты, например:

- `MapMeta` (width, height, resolution, origin)
- `MapData` (timestamp + `std::vector<uint8_t> cells`)
- опционально `Pose2D` (x, y, yaw)

### 1.2 Нода

Добавить файлы:

- `cpp/app/include/nodes/map_node.h`
- `cpp/app/src/nodes/map_node.cpp`

Логика:

- Подписка на `laser_data`.
- Подписка на `odometry_data`.
- Хранить последнюю позу робота.
- Держать grid `300x300` в памяти (`255` на старте).
- Таймер публикации (например 2-10 Гц).

### 1.3 Обновление карты (Bresenham)

Для каждого луча:

1. Вычислить угол луча.
2. Получить world-точку попадания.
3. Перевести:
   - позу робота в клетку `(rx, ry)`,
   - hit-точку в клетку `(hx, hy)`.
4. Пройти Bresenham от `(rx, ry)` до `(hx, hy)`:
   - промежуточные клетки пометить `free(0)`,
   - конечную клетку пометить `occupied(100)` (если не max-range).
5. Проверять границы и clamp индексов.

Формула индекса:

- `idx = y * width + x`

### 1.4 Интеграция в приложение

В `cpp/app/src/robot_app.cpp`:

- зарегистрировать ноду карты через `RegisterService<...>()`.

В конфиг добавить параметры карты/порта, если нужно:

- `android_map_port`
- `map_publish_hz`

---

## 2. C++: публикация карты по TCP через ZMQ + protobuf

### 2.1 Зависимости

В `cpp/conan/conanfile.py` добавить:

- `zeromq`
- `cppzmq`

### 2.2 Генерация protobuf

Обновить `proto/CMakeLists.txt`, чтобы генерировались:

- `cmd_vel.proto`
- `map.proto`

### 2.3 PUB-сокет

В `MapNode` или отдельной `MapPublisherNode`:

- создать `ZMQ_PUB`,
- bind на `tcp://*:5556` (или из конфига),
- сериализовать `MapFrame` в protobuf,
- отправлять бинарный payload.

Опционально topic-фрейм:

- frame 0: `"map"`
- frame 1: protobuf bytes

### 2.4 Частота/нагрузка

- Размер сетки: `300x300 = 90 000` байт сырых данных.
- Начать с 2-5 Гц.
- Публиковать по dirty-flag (если карта изменилась).

---

## 3. Android: прием и отображение

### 3.1 Прото на Android

`android/app/build.gradle.kts` уже смотрит в `../../proto`, после добавления `map.proto` классы сгенерируются автоматически.

### 3.2 Клиент приема

Создать `MapZmqClient.kt`:

- отдельный поток,
- `SUB` к `tcp://<robot_ip>:5556`,
- подписка на topic `"map"` (или все),
- `MapFrame.parseFrom(bytes)`,
- передача данных в UI через main-thread.

### 3.3 Отрисовка

Создать `MapView.kt`:

- хранит текущий frame,
- в `onDraw` рисует bitmap по `cells`,
- цвета:
  - `0` free -> светлый,
  - `100` occupied -> черный,
  - `255` unknown -> серый,
- рисует робота (круг + heading по `yaw`).

### 3.4 Встраивание в экран

В `activity_main.xml`:

- добавить `MapView`,
- оставить `JoystickView`.

В `MainActivity.kt`:

- старт `MapZmqClient` при Connect,
- стоп при Disconnect/onDestroy,
- `mapView.updateFrame(frame)` при получении данных.

---

## 4. Порядок внедрения (минимальный путь)

1. `map.proto` + сборка protobuf (C++ и Android).
2. `MapNode` с grid `300x300`.
3. Bresenham update по `laser_data`.
4. ZMQ PUB `MapFrame`.
5. Android SUB + `MapView`.
6. Подключение lifecycle в `MainActivity`.

---

## 5. Критерий готовности первой итерации

- Приложение подключается к роботу и получает бинарные protobuf кадры карты.
- На экране отображается occupancy grid `300x300`.
- Поза робота обновляется и рисуется поверх карты.
- При обрыве сети приложение не падает, reconnect работает.
