#!/bin/bash
# Start the SkyResQ pipeline (does not enable auto-start; use install.sh for that).
sudo systemctl start skyresq-core.target
echo "Pipeline started. Status:"
systemctl --no-pager status skyresq-core.target | head -3
