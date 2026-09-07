#!/bin/sh
# Учебный скрипт «установки»: в реальности здесь swupdate, разбор .pkg, прошивка разделов и т.д.
set -eu
PKG="$1"
echo "[install_firmware] received package: $PKG"
if ! test -f "$PKG"; then
  echo "[install_firmware] error: file missing" >&2
  exit 1
fi
echo "[install_firmware] size: $(wc -c < "$PKG") bytes"
echo "[install_firmware] (demo) pretend flash + reboot…"
sleep 1
echo "[install_firmware] done."
