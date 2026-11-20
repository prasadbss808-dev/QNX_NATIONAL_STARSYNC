# StarSync - build
#   make qnx    -> build/qnx/   the programs for the Raspberry Pi 5 (QNX 8)
#                  first:  source ~/qnx800/qnxsdp-env.sh
#   make host   -> build/host/  laptop test build (tools/linux.c replaces the QNX parts)
#   make prof   -> build/qnx/starsync-prof  function-instrumented build (QNX Application
#                  Profiler / System Profiler); NOT for accuracy runs - it is slower
#   make test   -> estimator test against exact truth
#   make lines  -> lines of C that run on the Pis: with comments / code only

QNX  = qcc -Vgcc_ntoaarch64le -O2 -Wall -Wextra -std=gnu11
HOST = gcc -O2 -Wall -Wextra -std=gnu11
NODE = src/main.c src/ptp.c src/threads.c src/commands.c src/clock.c src/estimator.c
QNXPARTS = src/platform.c src/gpio.c src/resmgr.c src/ham.c
# HAM is optional: use it only if this QNX SDP has the header (QNX 8 may not ship it)
HAM = $(if $(wildcard $(QNX_TARGET)/usr/include/ha/ham.h),-DHAVE_HAM -lham,)

qnx:
	mkdir -p build/qnx
	$(QNX) -o build/qnx/starsync  $(NODE) $(QNXPARTS) -lm -lsocket -lslog2 $(HAM)
	$(QNX) -o build/qnx/skygen    src/skygen.c src/platform.c src/gpio.c -lsocket -lslog2
	$(QNX) -o build/qnx/hog       src/hog.c src/platform.c -lslog2
	@echo "built build/qnx/ ($(if $(HAM),with,without) HAM)  ->  now: sh scripts/deploy.sh"

# every function of OUR code reports entry/exit (not hw_now/gpio_read_all: the capture
# thread calls them millions of times a second). -lprofilingS must come last.
prof:
	mkdir -p build/qnx
	$(QNX) -g -finstrument-functions -finstrument-functions-exclude-function-list=hw_now,gpio_read_all \
	       -o build/qnx/starsync-prof $(NODE) $(QNXPARTS) -lm -lsocket -lslog2 $(HAM) -lprofilingS
	@echo "built build/qnx/starsync-prof  ->  scp it to the Pi (see README: profiling)"

host:
	mkdir -p build/host
	$(HOST) -o build/host/starsync $(NODE) tools/linux.c -lm -lpthread -lrt
	$(HOST) -o build/host/skygen   src/skygen.c tools/linux.c -lpthread -lrt

test:
	mkdir -p build
	$(HOST) -o build/test_sync tools/test_sync.c src/estimator.c src/clock.c tools/linux.c -lm -lpthread -lrt
	./build/test_sync | grep -v "^SYNC"

lines:
	@echo "C on the Pis        with comments / code only (no comments, no blank lines)"
	@for f in src/*.c src/*.h; do printf "  %-16s %8d / %d\n" $$f $$(wc -l < $$f) \
	    $$(gcc -fpreprocessed -dD -E -P $$f 2>/dev/null | grep -cv '^[[:space:]]*$$'); done
	@printf "  %-16s %8d / %d\n" total $$(cat src/*.c src/*.h | wc -l) \
	    $$(for f in src/*.c src/*.h; do gcc -fpreprocessed -dD -E -P $$f 2>/dev/null; done | grep -cv '^[[:space:]]*$$')

clean:
	rm -rf build
