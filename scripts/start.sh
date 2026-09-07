#!/usr/bin/env bash
# start.sh — поднять/остановить ВЕСЬ стек робота:
#   RoboController -> ручной режим (MCU активен, LDS стримит, робот стоит на месте)
#   -> keepalive ручного режима -> xiaomi_robot.
#
# Работает и на роботе, и с хоста:
#   на роботе:  /opt/start.sh [start|stop|status]            (IP по умолчанию 127.0.0.1)
#   с хоста:    scripts/start.sh start  192.168.0.137        -> задеплоит себя в /opt/start.sh
#               scripts/start.sh status 192.168.0.137           и выполнит команду на роботе по ssh
#               scripts/start.sh stop   192.168.0.137
# Режим выбирается автоматически: если рядом нет /opt/rockrobo/.../RoboController — мы на хосте.
# HTTP к Valetudo идёт через bash /dev/tcp (на роботе нет curl).
set -u
CMD="${1:-start}"
IP="${2:-127.0.0.1}"

ROBOCON=/opt/rockrobo/cleaner/bin/RoboController
XR=/opt/xiaomi_robot
XR_CFG=/opt/config.json
XR_LOG=/opt/xiaomi_robot.log
KA_PID=/tmp/xr_keepalive.pid
CAP="/api/v2/robot/capabilities"

# ---------- HTTP через /dev/tcp ----------
http_put() {  # $1 path, $2 json  -> 0 если HTTP 200
  local body="$2" rc
  exec 3<>"/dev/tcp/$IP/80" 2>/dev/null || return 1
  printf 'PUT %s HTTP/1.0\r\nHost: x\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s' \
    "$1" "${#body}" "$body" >&3
  head -c 15 <&3 | grep -q " 200 "; rc=$?
  exec 3<&-
  return $rc
}
robot_state() {
  exec 3<>"/dev/tcp/$IP/80" 2>/dev/null || { echo "valetudo недоступен"; return; }
  printf 'GET /api/v2/robot/state/attributes HTTP/1.0\r\nHost: x\r\n\r\n' >&3
  local js; js=$(cat <&3); exec 3<&-
  # состояние (StatusStateAttribute) + батарея (BatteryStateAttribute), без fan_speed
  local st bat
  st=$(echo "$js" | grep -oE '"__class":"StatusStateAttribute","metaData":\{[^}]*\},"value":"[a-z_]+"' | grep -oE '"value":"[a-z_]+"' | cut -d'"' -f4)
  bat=$(echo "$js" | grep -oE '"__class":"BatteryStateAttribute","metaData":\{[^}]*\},"level":[0-9]+,"flag":"[a-z]+"' | grep -oE '"flag":"[a-z]+"' | cut -d'"' -f4)
  echo "${st:-?} / батарея: ${bat:-?}"
}
export -f http_put
export IP CAP

# Полный detach: новая сессия (setsid), иммунитет к SIGHUP (nohup), fd 0/1/2 -> файлы,
# унаследованные fd 3..9 закрыты, ребёнок уходит под init. Иначе процесс держит пайпы sshd
# и ssh-сессия висит до таймаута.
daemonize() {  # daemonize LOGFILE cmd args...
  local log="$1"; shift
  ( exec 3>&- 4>&- 5>&- 6>&- 7>&- 8>&- 9>&- 2>/dev/null
    setsid nohup "$@" </dev/null >>"$log" 2>&1 & )
}

# ---------- удалённый режим (хост): деплой + ssh ----------
if [ ! -x "$ROBOCON" ]; then
  if [ "$IP" = "127.0.0.1" ] || [ "$IP" = "localhost" ]; then
    echo "с хоста укажи IP робота: $0 $CMD 192.168.0.137" >&2
    exit 1
  fi
  scp -q "$0" "root@$IP:/opt/start.sh" || { echo "scp не удался" >&2; exit 1; }
  exec ssh "root@$IP" "chmod +x /opt/start.sh; /opt/start.sh $CMD 127.0.0.1"
fi

# ---------- локальный режим (на роботе) ----------
keepalive_up() { [ -f "$KA_PID" ] && kill -0 "$(cat "$KA_PID")" 2>/dev/null; }

case "$CMD" in
start)
  # 1. RoboController — переводит MCU в активный режим по команде
  if ! pgrep -x RoboController >/dev/null; then
    echo "[start] RoboController"
    daemonize /tmp/robocon.log "$ROBOCON"
    sleep 5
  fi
  # 2. ручной режим: LDS стримит, робот стоит (с ретраями — RoboController поднимается не сразу)
  ok=0
  for _ in 1 2 3 4 5 6; do
    http_put "$CAP/HighResolutionManualControlCapability" '{"action":"enable"}' && { ok=1; break; }
    sleep 2
  done
  [ "$ok" = 1 ] && echo "[start] ручной режим включён" || echo "[start] ВНИМАНИЕ: ручной режим не включился (нет 200)"
  # 3. keepalive: нулевой вектор каждые 2с, иначе ручной режим отваливается и LDS гаснет
  if ! keepalive_up; then
    daemonize /dev/null bash -c 'echo $$ > '"$KA_PID"'; while :; do http_put "$CAP/HighResolutionManualControlCapability" "{\"action\":\"move\",\"vector\":{\"velocity\":0,\"angle\":0}}"; sleep 2; done'
    sleep 1; echo "[start] keepalive (pid $(cat "$KA_PID" 2>/dev/null || echo ?))"
  fi
  # 4. xiaomi_robot
  if ! pgrep -x xiaomi_robot >/dev/null; then
    echo "[start] xiaomi_robot"
    daemonize /tmp/xiaomi_robot.out bash -c "cd /opt && exec $XR -c $XR_CFG"
  fi
  sleep 5
  exec "$0" status "$IP"
  ;;
stop)
  echo "[stop] останавливаю стек"
  keepalive_up && kill "$(cat "$KA_PID")" 2>/dev/null; rm -f "$KA_PID"
  pkill -9 -x xiaomi_robot 2>/dev/null
  http_put "$CAP/HighResolutionManualControlCapability" '{"action":"disable"}'
  # вернуть на базу ПОКА RoboController жив (ручной режим мог сдать робота с дока)
  http_put "$CAP/BasicControlCapability" '{"action":"home"}'
  for _ in $(seq 1 20); do robot_state | grep -q 'charg' && break; sleep 2; done
  pkill -x RoboController 2>/dev/null; sleep 1; pkill -9 -x RoboController 2>/dev/null
  echo "[stop] готово. робот: $(robot_state)"
  ;;
status)
  echo "RoboController: $(pgrep -x RoboController >/dev/null && echo up || echo down)"
  echo "keepalive:      $(keepalive_up && echo up || echo down)"
  echo "xiaomi_robot:   $(pgrep -x xiaomi_robot >/dev/null && echo "up (pid $(pgrep -x xiaomi_robot | head -1))" || echo down)"
  echo "робот:          $(robot_state)"
  last=$(grep LASER "$XR_LOG" 2>/dev/null | tail -1)
  cnt=$(echo "$last" | sed -nE 's/.*LASER: [0-9]+ [-0-9.]+ [-0-9.]+ ([0-9]+).*/\1/p')
  echo "лидар:          $([ -n "$cnt" ] && echo "последний скан count=$cnt" || echo "нет данных в логе")"
  ;;
*)
  echo "usage: $0 [start|stop|status] [IP]" >&2
  exit 1
  ;;
esac
