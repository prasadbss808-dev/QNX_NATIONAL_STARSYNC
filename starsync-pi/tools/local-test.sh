#!/bin/sh
# local-test.sh - OPTIONAL laptop check before using the Pis (about 3 minutes).
# VEGA, ALTAIR and SKY run as programs on this laptop; the SKY wires are shared
# memory (tools/linux.c). The referee scores everything; every judge fault is injected.
#   make host ; sh tools/local-test.sh        (dashboard: http://localhost:8080)
cd "$(dirname "$0")/.." || exit 1
B=build/host
pkill -x starsync; pkill -x skygen; pkill -f referee.py; sleep 1
rm -f /dev/shm/starsync_keep_* /dev/shm/starsync_wire /tmp/ref_in; mkfifo /tmp/ref_in
V=127.0.0.1:31903 ; A=127.0.0.1:31913
# cmd(): send one command to a node (address:port) through scripts/cmd.sh - the judges' fault injection
cmd() { sh scripts/cmd.sh "$@"; }
(sleep 100000 > /tmp/ref_in &)                          # keeps the fifo open
python3 laptop/referee.py --vega $V --altair $A < /tmp/ref_in > /tmp/ref.log 2>&1 &
# loop(): run a program again whenever the watchdog ends it with exit 42 - like run-altair.sh on a Pi
loop() { while true; do "$@"; [ $? -eq 42 ] || break; echo "### restarted after watchdog"; done; }
STARSYNC_WIRE=A loop $B/starsync -r master -i vega   -P 31919 -c 31903 -L 127.0.0.1 > /tmp/vega.log 2>&1 &
sleep 1
STARSYNC_WIRE=B loop $B/starsync -r slave  -i altair -P 31929 -c 31913 -L 127.0.0.1 -m 127.0.0.1:31919 > /tmp/altair.log 2>&1 &
$B/skygen 127.0.0.1 > /tmp/sky.log 2>&1 &
# step(): print the time and the name of the next test step, so the log reads like a script
step() { echo "==== $(date +%T) $*"; }
step "wait for lock";        sleep 15; cmd $A status | sed -n 2,3p
step "calibrate";            echo c > /tmp/ref_in; sleep 3; echo z > /tmp/ref_in; sleep 8
step "drift altair +20 ppm"; cmd $A drift 20;      sleep 12
step "ramp 0.5 ppm/s";       cmd $A ramp 0.5;      sleep 10; cmd $A ramp 0; sleep 5
step "delay 0-2000 us both"; cmd $V delay 0 2000; cmd $A delay 0 2000; sleep 15
                             cmd $V netoff; cmd $A netoff; sleep 6
step "one-way +300 us";      cmd $V asym 300;      sleep 8; cmd $V netoff; sleep 6
step "blackout 15 s";        cmd $V blackout 15; cmd $A blackout 15; sleep 20
step "step -3000 us";        cmd $A step -3000;    sleep 8
step "hang servo (watchdog)";cmd $A hang servo 2000; sleep 8
step "vega health fault";    cmd $V health fault;  sleep 4; cmd $V health ok; sleep 5
step "report";               cmd $A status; echo r > /tmp/ref_in; sleep 2
pkill -f referee.py; pkill -x starsync; pkill -x skygen; pkill -f "sleep 100000"; sleep 1
tail -22 /tmp/ref.log
