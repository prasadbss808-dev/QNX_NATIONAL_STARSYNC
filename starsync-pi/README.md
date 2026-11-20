# StarSync — fault-tolerant clock alignment on 3 × Raspberry Pi 5 (QNX 8)

Team **PRIORITY ZERO** (QNXHACK-HYD-002) · BSS Prasad & Balu Banothu · QNX eHACK 2026 national finale ·
Problem Statement 6: *Fault-Tolerant Clock Alignment for Distributed Radio Telescope Observations*

VEGA is the time reference. ALTAIR keeps its clock aligned to VEGA over plain Ethernet with
**IEEE 1588 PTPv2** (Sync, Follow_Up, Delay_Req, Delay_Resp, Announce). A **2-state Kalman filter**
estimates offset **and** drift. ALTAIR hands out **monotonic** corrected timestamps, each tagged
VALID / PREDICTED / UNRELIABLE with an honest ±U. It detects sync loss and recovers in bounded
time. SKY fires pulses that both telescopes see at the same instant, so the laptop measures the
**true** offset between them.

**Code on the Pis: 1238 lines of C (POSIX + QNX APIs), 1433 with comments** (`make lines`).
Written for readability: one topic per file, clear names (`offset_ns`, `drift_ppb`, `var_offset` ...),
one step per line, no tricks, and every function starts with a comment saying what it does and why.

| Board | IP | Runs |
|---|---|---|
| VEGA | 172.16.6.76 | `starsync -r master -i vega` (PTP master, reference) |
| ALTAIR | 172.16.6.77 | `starsync -r slave -i altair` (PTP slave, follower) |
| SKY | 172.16.6.78 | `skygen` (the "star": pulse generator) |
| Laptop | 172.16.6.100 | `laptop/referee.py` + dashboard, `scripts/cmd.sh` (commands) |

---------------------------------------------------------------------------------------------

## 1. Quick start

```sh
# 1. laptop: build for the Pis and copy to all three (QNX SDP 8 installed)
unzip starsync-pi.zip && cd starsync-pi
source ~/qnx800/qnxsdp-env.sh
make qnx                       # -> build/qnx/starsync skygen hog
sh scripts/deploy.sh           # -> ~/starsync on .76 .77 .78 (asks the qnxuser password)

# 2. laptop: referee + dashboard (own terminal), then open http://localhost:8080
python3 laptop/referee.py

# 3. each Pi (own terminal):  ssh qnxuser@<ip> ; cd starsync ; su
ham &                          # VEGA and ALTAIR: optional, only if your QNX SDP has HAM
sh run-vega.sh                 # on VEGA   172.16.6.76
sh run-altair.sh               # on ALTAIR 172.16.6.77
sh run-sky.sh                  # on SKY    172.16.6.78 (after VEGA and ALTAIR run)
```
When ALTAIR shows LOCKED for ~20 s: **Calibrate asymmetry** on the dashboard, wait 10 s, **Reset scores**.

---------------------------------------------------------------------------------------------

## 2. Problem sheet → where it is done

| Requirement | How | Code |
|---|---|---|
| Two-node sync, one reference | VEGA = PTP master, ALTAIR = PTP slave: Sync, Follow_Up, Delay_Req, Delay_Resp, Announce over UDP 319/320 | `ptp.c` master_rx / slave_rx |
| Offset **and** drift, variable delay | T1..T4 → offset z, delay d, error bound ε; **Kalman filter** (predict → surprise → update); slow packets get a larger R and count less, > 2 ms ignored | `estimator.c` est_measurement |
| Monotonic corrected timestamps | corrected clock = a line whose slope is steered (never jumps) + guard: every stamp > the previous one, kept in shared memory (survives restarts) | `clock.c` clock_stamp |
| Detect unreliable time | 400 ms without data → HOLDOVER (TIMEOUT); VEGA clockClass 255 → HOLDOVER (REF_FAULT); 3 far-off measurements → CLOCK_STEP; watchdog | `estimator.c` est_tick |
| Mark timestamps | VALID (U ≤ tol) / PREDICTED (holdover, U ≤ tol) / UNRELIABLE | `estimator.c` est_tick |
| Gradual / sudden drift | 4 Kalman surprises on one side → drift doubt reopened (P11 += (20 ppm)²) → new drift learned in ~1–2 s | `estimator.c` est_measurement |
| Bounded recovery | 32 measurements/s, learned drift kept, fast slew → LOCKED again in ~0.1–0.6 s | `estimator.c` RECOVERING |
| Six key metrics | measured on the laptop from the real pulses (true offset) | `laptop/referee.py` |
| Mandatory CLI | console line every second; `cat /dev/starsync/status`; `sh scripts/cmd.sh altair status` | `commands.c` status_text |
| Stretch: asymmetry | referee measures the steady offset from the pulses → `calib` to ALTAIR | `calib` command |
| Stretch: advanced holdover | runs on the learned drift; U grows by how wrong that drift could be | `estimator.c` HOLDOVER |

### QNX features used

| QNX feature | Where | Code |
|---|---|---|
| High-resolution timers + pulses | per-thread channel + timer, `SIGEV_PULSE`, `MsgReceivePulse`, `TIMER_TOLERANCE` 10 µs, `procmgr_ability(HIGH_RESOLUTION_TIMER)` | `platform.c` sleep_until |
| `ClockCycles()` time base | free-running counter → ns, never adjusted | `platform.c` hw_now |
| Priorities + core pinning | `SCHED_FIFO`, `ThreadCtl(_NTO_TCTL_RUNMASK)`; servo wake-up lateness measured live; `hog` load test | `platform.c` set_thread, `hog.c` |
| Resource manager | `/dev/starsync/status`, `/time`, `/ctl` via `resmgr_attach` + message passing | `resmgr.c` |
| Custom `devctl` | `DCMD_STARSYNC_TIME`: corrected time + U + validity in one message; `starsync -q` measures the round trip | `resmgr.c` io_devctl, time_client |
| HAM (optional) | used only if the QNX SDP ships `<ha/ham.h>` (our QNX 8 SDP does not): heartbeat 200 ms, 3 missed → restart | `ham.c` ham_start |
| Watchdog on its own core | priority 70, core 3, every 10 ms; late → UNRELIABLE; stuck → exit 42 → restart | `threads.c` watchdog_thread |
| Shared memory | `/dev/shmem/starsync_keep_<node>`: last stamp + restart count survive restarts | `clock.c` clock_init |
| slogger2 + kernel trace | every event → console + `slog2info` + System Profiler user event | `platform.c` qlog, qmark |
| Direct RP1 GPIO | `mmap_device_memory` of the Pi 5 RP1 registers; one register write raises both COMMON pins | `gpio.c` gpio_* |
| Real-time networking | PTP on its own thread (64, core 2), stamped the moment `poll()` wakes; DSCP EF via `IP_TOS` | `threads.c` rx_thread |

Threads on VEGA and ALTAIR: watchdog 70 / core 3 · rx 64 and capture 63 / core 2 · servo 55 / core 1 ·
report 20, commands 12, resource manager 12 / core 0. SKY: skygen 60 / core 3.

---------------------------------------------------------------------------------------------

## 3. Connections

**Ethernet:** VEGA, ALTAIR, SKY and the laptop into the TL-SG105 switch (power the switch!).

**Pulse wires:** 6 female-to-female jumper wires, every wire one-to-one (no breadboard).
Connect with the boards switched off. Never touch pins 2 and 4 (5 V).

| Wire | From SKY | To | Purpose |
|---|---|---|---|
| 1 | pin 11 (GPIO17) | VEGA pin 13 (GPIO27) | COMMON pulse for VEGA |
| 2 | pin 13 (GPIO27) | ALTAIR pin 13 (GPIO27) | COMMON pulse for ALTAIR (same register write as wire 1) |
| 3 | pin 15 (GPIO22) | VEGA pin 15 (GPIO22) | ordering pulse A |
| 4 | pin 16 (GPIO23) | ALTAIR pin 15 (GPIO22) | ordering pulse B |
| 5 | pin 6 (GND) | VEGA pin 6 (GND) | common ground (required) |
| 6 | pin 9 (GND) | ALTAIR pin 9 (GND) | common ground (required) |

Pins 1 and 2 are at the end of the header away from the USB ports; odd pins on the inner row.
**Wiring test:** on SKY (as root, before run-sky.sh) run `./skygen test`: all four outputs go
1 s HIGH / 1 s LOW and must read back `ok`; on VEGA and ALTAIR the `PULSES` count in
`cat /dev/starsync/status` must go up. Ctrl-C ends the test.

---------------------------------------------------------------------------------------------

## 4. Commands

### The CLI (on VEGA or ALTAIR)
```sh
cat /dev/starsync/status                 # full status (QNX resource manager)
cat /dev/starsync/time                   # corrected time + U + validity, for any program
./starsync -q                            # same time via devctl(), 1000 calls: avg / max round trip
echo "drift 20" > /dev/starsync/ctl      # any command, on the Pi itself
sh scripts/cmd.sh altair status          # the same from the laptop
```

### The judges' faults (dashboard buttons, or from the laptop)
```sh
C="sh scripts/cmd.sh"
$C altair drift 20          # oscillator +20 ppm     ($C altair ramp 0.5 = gradual drift)
$C altair clear             # remove injected drift
$C all delay 0 2000         # variable delay 0..2000 us
$C vega asym 300            # one-way extra delay ALTAIR->VEGA
$C all netoff               # remove network faults
$C all blackout 15          # temporary sync loss 15 s   (or pull ALTAIR's cable)
$C altair step -3000        # sudden clock jump of -3 ms
$C vega health fault        # VEGA declares itself faulty   ($C vega health ok)
$C altair hang servo 2000   # freeze one thread -> watchdog
```

### QNX real-time and restart proof (on ALTAIR, second ssh window, as root)
```sh
./hog 10 30                 # 4 busy threads at priority 10: nothing changes
./hog 60 3 1                # priority 60 on core 1 steals the servo (55): watchdog catches it
slay -s KILL starsync       # crash the program: run-altair.sh restarts it within 1 s,
                            #   and /dev/shmem keeps every new stamp after the old ones
```

### slogger2, System Profiler, function instrumentation
User codes: 100 info · 101 state change · 102 command · 103 warning · 104 watchdog;
System Profiler number events: 110 servo tick (lateness ns, U ns) · 111 pulse · 112 PTP exchange.
```sh
slog2info -b starsync -S          # StarSync's log with severities
slog2info -b starsync -e 101      # only the state changes
slog2info -b starsync -w          # live
sh trace.sh altair 3              # 3 s kernel trace into RAM -> /dev/shmem/altair.kev, lists our events
scp qnxuser@172.16.6.77:/dev/shmem/altair.kev .   # laptop: drag into a Momentics project, double-click
```
Function instrumentation: on the laptop `make prof`, copy `build/qnx/starsync-prof` to ALTAIR, then
```sh
slay starsync ; rm -f /dev/shmem/starsync_keep_*
QPROF_KERNEL_TRACE=1 ./starsync-prof -r slave -i altair &   # function entry/exit -> kernel trace
sh trace.sh altair-prof 3 ; slay starsync-prof
```
The profiling build is slower: use it to show where the time goes, not for accuracy numbers.

### Laptop-only test (no Pis needed, Linux)
```sh
make test                         # Kalman estimator against exact truth -> TEST PASSED
make host ; sh tools/local-test.sh   # whole system + every fault on one laptop (~3 min)
```

---------------------------------------------------------------------------------------------

## 5. Debugging guide

| You see | Do |
|---|---|
| `cannot use port 319` | run as root (`su`); or starsync runs twice: `slay starsync` |
| ALTAIR stays `INIT` | `ping 172.16.6.76`; VEGA must print `PTP: slave 172.16.6.77 joined` |
| `NOTE: no SKY pulse on pin 13` | on SKY: stop skygen, `./skygen test` (all `ok`); then check that node's wire to pin 13 and GND |
| pulses on both boards but dashboard `pulses 0` | an old stamp in `/dev/shmem` freezes the guard: Ctrl-C on VEGA and ALTAIR, `rm -f /dev/shmem/starsync_keep_*`, start again (the run scripts do this on every fresh start) |
| `resmgr: cannot register`, `HR timers OFF` | not root |
| `HAM: not running` | start it as root: `ham &` (StarSync works without it; the status shows `HAM off`) |
| `ha/ham.h: No such file` | not possible any more: `make qnx` builds without HAM if the SDP has no HAM |
| dashboard "ALTAIR silent" | laptop IP must be 172.16.6.100; `sudo ufw allow 31910/udp` |
| `WATCHDOG: ... stuck` | a thread was blocked; the run script restarts starsync |
| `scp: ... subsystem` in deploy | use `scp -O` in `scripts/deploy.sh` |
| `slog2info` shows no `starsync` | `pidin \| grep slogger2`; start it with `slogger2 &`, restart starsync |
| `tracelogger: not found` | laptop: `find $QNX_TARGET/aarch64le -name tracelogger`, `scp` it to the Pi |
| `Help, we're not keeping up` | the trace is written too slowly: use `trace.sh` (records into RAM, normal mode) or a shorter time |
| `tracelog_start: Resource busy` | another tracelogger still runs: `slay tracelogger` (trace.sh does this itself) |
| tracelogger hangs (`pidin` shows it `READY`) | it was stuck behind the capture thread on core 2: `slay starsync`, then use `trace.sh` (it runs tracelogger at priority 65) |
| `make prof`: cannot find -lprofilingS | skip function instrumentation |

---------------------------------------------------------------------------------------------

## 6. Files

```
src/starsync.h   settings, names, the estimator's memory and the list of functions (read this first)
src/node.h       the memory of one node (VEGA or ALTAIR), shared by the four files below
src/main.c       start: options, sockets, GPIO, start the threads, answer text commands
src/ptp.c        PTP messages: build them, VEGA answers, ALTAIR collects T1..T4
src/threads.c    the 5 real-time threads: rx, servo, capture, report, watchdog
src/commands.c   the status screen, the time, and the commands (faults, tuning)
src/clock.c      the corrected clock: never jumps, never backwards, fault injection
src/estimator.c  the brain: Kalman filter (offset + drift), U, state machine, clock steering
src/platform.c   QNX: ClockCycles, high-resolution timers + pulses, priorities, cores, slogger2, trace
src/gpio.c       Raspberry Pi 5 GPIO (RP1 registers)
src/resmgr.c     QNX resource manager /dev/starsync/{status,time,ctl} + devctl
src/ham.c        QNX HAM supervision (optional: only built in if the SDP has HAM)
src/skygen.c     SKY: the pulses (+ ./skygen test wiring check)
src/hog.c        CPU load test (priorities and cores)
laptop/          referee.py (scores, all six metrics) + dashboard.html (live dashboard)
scripts/         run-vega/altair/sky.sh, deploy.sh, cmd.sh, trace.sh
tools/           laptop tests only: linux.c (Linux stand-in for the QNX parts), test_sync.c, local-test.sh
docs/            StarSync_Finale_Deck.pptx (jury presentation)
```
`make lines` prints every file's line count with comments and code only.

---------------------------------------------------------------------------------------------

## 7. Tested

* `make test` (Kalman estimator against known truth: 3 ms offset, 25 ppm drift, +20 ppm jump,
  −3 ms step, 15 s blackout, 0–2 ms delay storm, delay spikes): **0 trusted-but-wrong ticks** out of
  3 525, max trusted error 88 µs.
* `make host ; sh tools/local-test.sh` (whole system on one laptop, every judge fault, watchdog
  restart, reference fault): **honesty 100 %, 0 ordering errors, 0 backward timestamps**,
  recoveries 125 ms (15 s sync loss), 562 ms (−3 ms clock step), 125 ms (reference fault).
  The laptop's own pulse capture is uncertain by tens of µs; on a Pi it is under 1 µs.
* System Profiler on ALTAIR (3 s kernel trace, 0 of 233 buffers dropped, QNX 8.0.0): StarSync uses exactly
  one core (25.5 % = core 2, the capture thread, by design); all its threads together waited 3 ms for a CPU in
  3 s; kernel time 0.3 %.
* Real boards (earlier build): LOCKED in 2.1 s, corrected offset −3.9 … +2.8 µs, U 5–10 µs,
  loss detected in 400 ms, holdover U +1.55 µs/s.
