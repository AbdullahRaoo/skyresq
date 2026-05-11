#!/bin/bash
# install.sh — register the drone-ros2 systemd service on the Raspberry Pi.
#
# Idempotent. Run as a regular user; will invoke sudo for the few steps that
# need it (installing the unit file and the EnvironmentFile).
#
# Usage:
#   ./deploy/install.sh
#   sudo systemctl status drone-ros2
#   sudo journalctl -u drone-ros2 -f
#
# Uninstall:
#   sudo systemctl disable --now drone-ros2
#   sudo rm /etc/systemd/system/drone-ros2.service /etc/default/drone-ros2
#   sudo systemctl daemon-reload

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [[ "$(id -u)" -eq 0 ]]; then
    echo "ERROR: run this script as a regular user, not root." >&2
    echo "       (it will sudo for the few steps that need root)" >&2
    exit 1
fi

UNIT_SRC="${SCRIPT_DIR}/drone-ros2.service"
UNIT_DST="/etc/systemd/system/drone-ros2.service"
ENV_SRC="${SCRIPT_DIR}/drone-ros2.env"
ENV_DST="/etc/default/drone-ros2"

echo "[install] Installing systemd unit  -> ${UNIT_DST}"
sudo install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"

if [[ ! -f "${ENV_DST}" ]]; then
    echo "[install] Installing default env -> ${ENV_DST}"
    sudo install -m 0644 "${ENV_SRC}" "${ENV_DST}"
else
    echo "[install] Existing ${ENV_DST} preserved (edit manually if needed)"
fi

echo "[install] Reloading systemd"
sudo systemctl daemon-reload

echo "[install] Enabling drone-ros2 (will start on next boot)"
sudo systemctl enable drone-ros2

cat <<EOF

Installation complete.

Edit configuration:
    sudo nano /etc/default/drone-ros2

Start now (without rebooting):
    sudo systemctl start drone-ros2

Watch live output:
    sudo journalctl -u drone-ros2 -f

Status:
    sudo systemctl status drone-ros2

EOF
