# updater_example

Учебный процесс **«OTA updater»**: UDP-сервер принимает текстовую команду, качает `.pkg` по HTTP, проверяет MD5 и вызывает shell-скрипт «установки».

Это **не** реализация miIO: на пылесосе команды идут в **бинарном зашифрованном** виде на **UDP 54321**. Здесь — упрощённый поток для понимания: **команда → загрузка → проверка → скрипт**.

## Зависимости

- CMake, C++17, `pthread`
- Во время работы: **`curl`**, **`md5sum`** (в PATH)

## Структура проекта

- `include/updater/` — заголовки (`logger`, `udp_listener`, `ota_controller`, `updater_app`)
- `src/` — реализации и `main.cpp`
- `install_firmware.sh` — заглушка установки

## Сборка

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Запуск

```bash
./build/updater_example ./install_firmware.sh
```

Путь к скрипту можно не указывать: тогда берётся `install_firmware.sh` рядом с бинарником (относительно `argv[0]`).

Сервер слушает **UDP порт 54322** (не 54321, чтобы не пересекаться с другими службами на хосте).

## Протокол

Одна строка в **одной** UDP-датаграмме (без встроенных переводов строки внутри полей):

```text
OTA <http-url> <md5_32_hex>
```

Пример:

```text
OTA http://192.168.1.10:8000/firmware.pkg d41d8cd98f00b204e9800998ecf8427e
```

- URL должен начинаться с `http://` или `https://`.
- MD5 — ровно **32** шестнадцатеричных символа (как у `md5sum` без имени файла во втором поле не путать: в команду нужно подставить **только хэш**).

Пока идёт одна OTA, повторные команды отклоняются сообщением `OTA already in progress`.

## Проверка вручную

Терминал 1 — HTTP с файлом (имя в URL должно совпадать с реальным файлом):

```bash
echo test > /tmp/test.pkg
cd /tmp && python3 -m http.server 8000
```

Терминал 2 — updater (из каталога `updater_example`):

```bash
./build/updater_example ./install_firmware.sh
```

Терминал 3 — отправка команды (`md5sum` должен отработать по **существующему** файлу):

```bash
printf 'OTA http://127.0.0.1:8000/test.pkg %s\n' "$(md5sum /tmp/test.pkg | awk '{print $1}')" | nc -u 127.0.0.1 54322
```

Если `$(md5sum …)` подставляет пустую строку (несуществующий путь), в логе будет явная ошибка про отсутствующий MD5.

## Файлы

| Путь | Назначение |
|------|------------|
| `src/main.cpp` | Точка входа |
| `src/logger.cpp`, `include/updater/logger.h` | Потокобезопасный лог |
| `src/udp_listener.cpp`, `include/updater/udp_listener.h` | UDP bind + `recvfrom` |
| `src/ota_controller.cpp`, `include/updater/ota_controller.h` | Парсинг `OTA`, фоновая загрузка / MD5 / `sh` |
| `src/updater_app.cpp`, `include/updater/updater_app.h` | Цикл приёма датаграмм |
| `install_firmware.sh` | Заглушка установки |
| `CMakeLists.txt` | Сборка `updater_example` |

Скачанный пакет пишется в **`/tmp/updater_example.pkg`** (хардкод для демо).

## Ограничения и безопасность

- Использование `std::system` и строки из сети — **только для локального эксперимента**; в проде нужны жёсткий парсинг, отказ от shell, отдельный пользователь, таймауты, проверка подписи образа и т.д.
- Реальный робот использует **miIO по UDP:54321** и свой установщик прошивки, а не этот текстовый протокол.
