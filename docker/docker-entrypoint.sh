#!/usr/bin/env bash
# Git needs a passwd entry for the effective UID. docker run -u uid:gid does not create one.
# Start as root, useradd for HOST_UID, then sudo -E (no password) so PATH and -e HOME are kept.
# Trusty su(1) often hits PAM "Authentication failure" and drops PATH even with -p.
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  exec "$@"
fi

uid="${HOST_UID:-1000}"
gid="${HOST_GID:-1000}"

if ! getent passwd "$uid" >/dev/null 2>&1; then
  if ! getent group "$gid" >/dev/null 2>&1; then
    groupadd -g "$gid" hostbuild
  fi
  useradd -u "$uid" -g "$gid" -M -d /tmp -s /bin/bash hostbuild
fi

name="$(getent passwd "$uid" | cut -d: -f1)"
{
  # Main sudoers sets secure_path without conda; that overrides PATH even with sudo -E.
  echo 'Defaults secure_path=/opt/conda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
  printf '%s ALL=(ALL) NOPASSWD:ALL\n' "$name"
} >/etc/sudoers.d/docker-build
chmod 0440 /etc/sudoers.d/docker-build

exec sudo -E -u "$name" -- "$@"
