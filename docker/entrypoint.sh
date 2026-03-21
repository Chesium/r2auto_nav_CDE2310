#!/usr/bin/env bash
set -euo pipefail

NAV_WS="${NAV_WS:-$HOME/nav_ws}"
EXAMPLE_ENV="/usr/local/share/navws/ros_network.env.example"
TARGET_ENV="${NAV_WS}/docker/ros_network.env"

if [[ -d "${NAV_WS}/docker" && ! -f "${TARGET_ENV}" && -f "${EXAMPLE_ENV}" ]]; then
  cp "${EXAMPLE_ENV}" "${TARGET_ENV}"
fi

exec "$@"
