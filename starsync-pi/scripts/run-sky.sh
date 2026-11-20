#!/bin/sh
# run-sky.sh - on SKY (172.16.6.78), as root, after VEGA and ALTAIR are running:
#     cd ~/starsync ; su ; sh run-sky.sh
# Fires the pulses: COMMON 10 per second, an ordering pair every 0.5 s. Ctrl-C stops it.
cd "$(dirname "$0")" || exit 1
slay skygen 2>/dev/null
./skygen "$@"
