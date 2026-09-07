# Беги: запись потока с робота

`tools/bag_record` — хостовый бинарь: подписывается на PUB робота (`tcp://robot:9091`) и пишет
всё, что приходит, в файл. Ничего на робот ставить не нужно, робот не нагружается.

```bash
./build/Debug/tools/bag_record --host 192.168.0.137 --out run.bag              # до Ctrl+C
./build/Debug/tools/bag_record --host 192.168.0.137 --out run.bag --duration 120
./build/Debug/tools/bag_record --topics slam/pose,mission/state --out states.bag
```

По окончании печатает сводку: сообщений по топикам и объём. Собирается вместе с проектом
(`BUILD_TOOLS`, по умолчанию включён; в ARM-сборке выключен).

## Что попадает в бег

Всё, что `ZmqBridge` публикует наружу: `slam/map`, `slam/pose`, `robot/state`,
`planner/trajectory`, `planner/state`, `mission/state`. Сырые сенсоры (лидар, одометрия) по ZMQ
не идут — их пишет `Log` на самом роботе (`/opt/xiaomi_robot.log`, теги `LASER`, `ODOM`).

## Формат

Заголовок `XRBAG1\n`, дальше записи подряд:

| поле | тип | смысл |
|---|---|---|
| `recv_us` | uint64 LE | время приёма на хосте, мкс с эпохи |
| `len` | uint32 LE | длина сообщения |
| `data` | `len` байт | `xiaomi.robot.ZmqMessage` — тот же protobuf, что шёл по сети (`proto/zmq_message.proto`) |

Внутри `ZmqMessage`: `timestamp` робота (мкс), `topic`, и одно из полей полезной нагрузки.
Две метки времени — робота и хоста — позволяют увидеть задержку и пропуски.

## Чтение

```bash
python3 scripts/bag_info.py run.bag                 # топики, частоты, объёмы
python3 scripts/bag_info.py run.bag --dump 20       # первые записи: время, топик, тип, размер
python3 scripts/bag_info.py run.bag --topic slam/pose --dump 5
```

Сводка не требует сгенерированных protobuf-модулей (конверт разбирается вручную). Для полного
декодирования полезной нагрузки:

```bash
protoc -I proto --python_out=scripts/pb proto/*.proto
python3 scripts/bag_info.py run.bag --dump 5 --decode
```

Свой разбор — по формату выше: `struct.unpack("<QI", ...)`, затем `ZmqMessage.ParseFromString`.
