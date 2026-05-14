#!/bin/bash
# Tail the logs of one or all SkyResQ services.
# Usage: logs.sh                  # all
#        logs.sh gimbal           # just gimbal_controller
#        logs.sh mavlink-bridge   # by service short name
if [ -z "$1" ]; then
    journalctl -fu 'skyresq-*'
else
    # Try exact match first, then prefix match
    if systemctl list-units --no-pager --no-legend "skyresq-$1.service" | grep -q .; then
        journalctl -fu "skyresq-$1.service"
    else
        journalctl -fu "skyresq-$1*"
    fi
fi
