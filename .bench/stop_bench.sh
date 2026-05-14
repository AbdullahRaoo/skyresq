#!/bin/bash
# Quick stop for any background bench pipeline + cleanup
pgrep -af 'bench_pipeline\.sh|ros2 run drone_vision' | grep -v grep \
    | awk '{print $1}' | xargs -r kill 2>/dev/null
sleep 1
pgrep -af 'ros2 run drone_vision' | grep -v grep \
    | awk '{print $1}' | xargs -r kill -9 2>/dev/null
echo "stopped"
