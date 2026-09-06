#!/usr/bin/env python3
"""
Отправка CmdVel (protobuf) на xiaomi_robot: POST /cmd_vel (Content-Type: application/x-protobuf).

HTTP делаем через urllib (короткий запрос): python-requests добавляет длинные заголовки,
а сервер на роботе раньше читал только первые 512 байт — тело protobuf обрезалось → HTTP 400.

  pip install pygame

Примеры:
  python3 scripts/joystick_cmdvel.py -H 192.168.0.137 -p 9090
  python3 scripts/joystick_cmdvel.py -H 192.168.0.137 -p 9090 --keyboard
"""

from __future__ import annotations

import argparse
import dataclasses
import http.client
import math
import os
import struct
import sys
import time
from typing import Tuple

# proto3: xiaomi.robot.CmdVel: double vx=1, vy=2, w=3 (wire type 1)


def _encode_cmd_vel(vx: float, vy: float, w: float) -> bytes:
    # proto3: все поля по умолчанию — пустая сериализация; так же как curl --data-binary ""
    if (
        math.isclose(vx, 0.0, abs_tol=1e-9)
        and math.isclose(vy, 0.0, abs_tol=1e-9)
        and math.isclose(w, 0.0, abs_tol=1e-9)
    ):
        return b""
    out = bytearray()
    for field, val in ((1, vx), (2, vy), (3, w)):
        out.append((field << 3) | 1)
        out += struct.pack("<d", val)
    return bytes(out)


@dataclasses.dataclass
class Config:
    host: str
    port: int
    hz: float
    max_v: float
    max_w: float
    keyboard: bool
    use_https: bool
    # Дифференциал обычно не реагирует на vy; по умолчанию A/D — поворот (w), не стрейф.
    strafe: bool


def _post_cmd_vel(
    host: str,
    port: int,
    use_https: bool,
    vx: float,
    vy: float,
    w: float,
    timeout: float,
) -> None:
    body = _encode_cmd_vel(vx, vy, w)
    conn_cls = http.client.HTTPSConnection if use_https else http.client.HTTPConnection
    conn = conn_cls(host, port, timeout=timeout)
    try:
        conn.request(
            "POST",
            "/cmd_vel",
            body,
            headers={
                "Content-Type": "application/x-protobuf",
                "Content-Length": str(len(body)),
                "Connection": "close",
                "User-Agent": "joystick_cmdvel",
            },
        )
        resp = conn.getresponse()
        payload = resp.read()
        if resp.status < 200 or resp.status > 299:
            raise RuntimeError(f"HTTP {resp.status}: {payload!r}")
    finally:
        conn.close()


def _vel_from_joystick(js, max_v: float, max_w: float) -> Tuple[float, float, float]:
    ax0 = float(js.get_axis(0)) if js.get_numaxes() > 0 else 0.0
    ax1 = float(js.get_axis(1)) if js.get_numaxes() > 1 else 0.0
    # Правая палка по X — часто ось 2 или 3
    w_axis = 3 if js.get_numaxes() > 3 else 2
    axw = float(js.get_axis(w_axis)) if js.get_numaxes() > w_axis else 0.0
    if js.get_numaxes() <= 2:
        axw = 0.0

    vy = max(-1.0, min(1.0, ax0)) * max_v
    vx = -max(-1.0, min(1.0, ax1)) * max_v
    w = max(-1.0, min(1.0, axw)) * max_w
    return vx, vy, w


def _vel_from_keys(
    k,
    max_v: float,
    max_w: float,
    *,
    strafe: bool,
    pg,
) -> Tuple[float, float, float]:
    """W/S — vx, Q/E — w; A/D: при strafe — vy, иначе w (трактор для дифф. привода)."""
    vx = 0.0
    vy = 0.0
    ww = 0.0
    if k[pg.K_w] or k[pg.K_UP]:
        vx += 1.0
    if k[pg.K_s] or k[pg.K_DOWN]:
        vx -= 1.0
    if k[pg.K_a] or k[pg.K_LEFT]:
        if strafe:
            vy -= 1.0
        else:
            ww += 1.0
    if k[pg.K_d] or k[pg.K_RIGHT]:
        if strafe:
            vy += 1.0
        else:
            ww -= 1.0
    if k[pg.K_q]:
        ww += 1.0
    if k[pg.K_e]:
        ww -= 1.0
    return vx * max_v, vy * max_v, ww * max_w


def main() -> int:
    try:
        import pygame
    except ImportError:
        print("pip install pygame", file=sys.stderr)
        return 1

    ap = argparse.ArgumentParser()
    ap.add_argument("-H", "--host", default="192.168.0.137")
    ap.add_argument("-p", "--port", type=int, default=9090)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--max-v", type=float, default=0.3)
    ap.add_argument("--max-w", type=float, default=0.6)
    ap.add_argument("--keyboard", action="store_true", help="WASD + Q/E, без джойстика")
    ap.add_argument(
        "--strafe",
        action="store_true",
        help="A/D (и стрелки влево/вправо) — vy; по умолчанию A/D — поворот w (дифф. шасси)",
    )
    ap.add_argument("--https", action="store_true", help="https вместо http")
    args = ap.parse_args()

    cfg = Config(
        host=args.host.strip(),
        port=args.port,
        hz=args.hz,
        max_v=args.max_v,
        max_w=args.max_w,
        keyboard=args.keyboard,
        use_https=args.https,
        strafe=args.strafe,
    )
    if not cfg.host:
        return 1

    print(
        f"POST {'https' if cfg.use_https else 'http'}://{cfg.host}:{cfg.port}/cmd_vel @ {cfg.hz} Hz"
    )

    os.environ.setdefault("SDL_VIDEODRIVER", "x11" if "DISPLAY" in os.environ else "dummy")
    pygame.init()
    pygame.joystick.init()
    if not cfg.keyboard and pygame.joystick.get_count() < 1:
        print("Нет джойстика. Подключи геймпад или запусти с --keyboard", file=sys.stderr)
        return 1

    if not cfg.keyboard:
        js = pygame.joystick.Joystick(0)
        js.init()
        print("Джойстик:", js.get_name())
    else:
        js = None
        if cfg.strafe:
            print("Клавиатура: A/D — стрейф (vy), W/S, Q/E; Esc")
        else:
            print(
                "Клавиатура: W/S вперёд-назад, A/D — поворот, Q/E — поворот, Esc (--strafe для vy)"
            )

    w, h = (320, 200)
    screen = pygame.display.set_mode((w, h))
    pygame.display.set_caption("cmd_vel → robot (close window to quit)")
    clock = pygame.time.Clock()
    t0 = time.time()
    n_ok = 0
    err = None
    run = True
    while run:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                run = False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                run = False

        keys = pygame.key.get_pressed()
        if cfg.keyboard:
            vx, vy, wz = _vel_from_keys(keys, cfg.max_v, cfg.max_w, strafe=cfg.strafe, pg=pygame)
        else:
            vx, vy, wz = _vel_from_joystick(js, cfg.max_v, cfg.max_w)
        if keys[pygame.K_c]:
            vx, vy, wz = 0.0, 0.0, 0.0

        try:
            _post_cmd_vel(cfg.host, cfg.port, cfg.use_https, vx, vy, wz, timeout=0.5)
            n_ok += 1
        except Exception as e:
            err = e
            run = False
            break

        if n_ok == 1 or (n_ok % int(max(cfg.hz, 1) * 2)) == 0:
            sys.stdout.write(
                f"\r vx={vx:+.3f} vy={vy:+.3f} w={wz:+.3f}  t={time.time() - t0:.1f}s  "
            )
            sys.stdout.flush()
        screen.fill((24, 24, 32))
        pygame.display.flip()
        clock.tick(cfg.hz)

    print()
    if err is not None:
        print("Ошибка:", err, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
