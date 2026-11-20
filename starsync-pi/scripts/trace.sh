#!/bin/sh
# trace.sh - on a Pi, as root, WHILE starsync/skygen run: record a kernel event trace
# for the QNX System Profiler, then list StarSync's own user events in it.
#     sh trace.sh altair 3      -> /dev/shmem/altair.kev (open it in the IDE's System Profiler)
# It records into RAM (/dev/shmem) in normal mode at priority 65 (above the capture thread, 63,
# which keeps core 2 busy), so tracelogger always keeps up and never gets stuck.
# /dev/shmem is cleared at reboot: copy the .kev to the laptop first.
# User event codes: 100 info  101 state change  102 command  103 warning  104 watchdog
#                   110 servo tick (lateness ns, U ns)  111 pulse (kind, number)
#                   112 PTP exchange (raw offset us, delay us)
name=${1:-node}; secs=${2:-3}; f=/dev/shmem/$name.kev
command -v tracelogger >/dev/null || { echo "no tracelogger on this Pi - copy it from the laptop (README: QNX tools)"; exit 1; }
slay tracelogger 2>/dev/null; sleep 1            # only one trace can run at a time
rm -f "$f"
echo "recording $secs s of kernel events -> $f"
on -p 65 tracelogger -n 0 -s "$secs" -f "$f" || exit 1
ls -l "$f"
echo "copy to the laptop:  scp qnxuser@<this-pi>:$f ."
command -v traceprinter >/dev/null || exit 0
traceprinter -f "$f" > /dev/shmem/$name.txt
echo "--- StarSync user events in the trace (first 15):"
grep USREVENT /dev/shmem/$name.txt | head -15
for c in 101 102 103 104 110 111 112; do echo "  event $c: $(grep -c "EVENT:$c[ ,]" /dev/shmem/$name.txt)"; done
