#!/bin/bash
# One-time installer: copies systemd units into /etc/systemd/system/, reloads
# the daemon, enables auto-start on boot, and starts the pipeline now.
#
# Re-runnable: safe to re-run after editing any unit file. It re-copies and
# reloads cleanly.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR="$SCRIPT_DIR/systemd"

if [ "$EUID" -ne 0 ]; then
    echo "Re-running with sudo..."
    exec sudo bash "$0" "$@"
fi

echo "==> Copying $UNIT_DIR/*.{service,target} to /etc/systemd/system/"
cp -v "$UNIT_DIR"/*.service /etc/systemd/system/
cp -v "$UNIT_DIR"/*.target /etc/systemd/system/

echo "==> systemctl daemon-reload"
systemctl daemon-reload

echo "==> Enabling skyresq-core.target (auto-start on boot)"
systemctl enable skyresq-core.target

# Enable each core service so the target pulls them in on boot.
for svc in skyresq-mavlink-bridge skyresq-rtsp-camera skyresq-gimbal-controller \
           skyresq-visual-servo skyresq-payload-servo skyresq-person-detector \
           skyresq-geo-localiser skyresq-gcs-link; do
    systemctl enable "${svc}.service"
done

echo "==> Starting pipeline now"
systemctl start skyresq-core.target

echo
echo "Installed. Useful commands:"
echo "  systemctl status 'skyresq-*'              # overview"
echo "  journalctl -u skyresq-mavlink-bridge -f    # tail a node's log"
echo "  systemctl restart skyresq-gimbal-controller  # restart one node"
echo "  ~/Drone/ops/stop.sh                       # stop entire pipeline"
echo "  ~/Drone/ops/start.sh                      # start entire pipeline"
echo
echo "sar_orchestrator is NOT auto-enabled. Start manually when ready:"
echo "  sudo systemctl start skyresq-sar-orchestrator"
