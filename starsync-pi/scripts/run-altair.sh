#!/bin/sh
# run-altair.sh - on ALTAIR (172.16.6.77), as root:
#     cd ~/starsync ; su ; sh run-altair.sh
# Runs in this window and prints ALTAIR's status every second. Ctrl-C stops it.
# If the watchdog restarts the program (exit code 42) or it crashes, it starts again.
cd "$(dirname "$0")" || exit 1
slay starsync 2>/dev/null
# fresh start: forget the last stamp of an EARLIER run (it may belong to a VEGA that has
# rebooted since, and would freeze every new stamp). Watchdog restarts below keep it.
rm -f /dev/shmem/starsync_keep_*
while true; do
    ./starsync -r slave -i altair "$@"
    rc=$?
    if [ $rc -eq 42 ]; then echo "### watchdog restarted starsync"
    elif [ $rc -gt 128 ] && [ $rc -ne 130 ]; then echo "### starsync crashed (exit $rc) - restarting"
    else echo "starsync stopped (exit $rc)"; break; fi
    sleep 1
done
