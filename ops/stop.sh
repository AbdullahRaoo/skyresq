#!/bin/bash
# Stop the SkyResQ pipeline. Also stops the optional sar_orchestrator if running.
sudo systemctl stop skyresq-sar-orchestrator 2>/dev/null
sudo systemctl stop 'skyresq-*.service'
echo "Pipeline stopped."
