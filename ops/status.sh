#!/bin/bash
# Quick overview of all SkyResQ services + their last 5 log lines.
for svc in $(systemctl list-units --no-pager --no-legend 'skyresq-*.service' | awk '{print $1}'); do
    state=$(systemctl is-active "$svc")
    printf "%-40s %s\n" "$svc" "$state"
done
echo
echo "--- recent errors (last 5 min) ---"
journalctl -u 'skyresq-*' --since '5 min ago' --no-pager -p err 2>/dev/null | tail -20
