/* linux.c - LAPTOP TESTS ONLY (never on the Pis): replaces platform.c, gpio.c, resmgr.c and ham.c
 * so the same StarSync code runs on Linux. The SKY wires become one word of shared
 * memory: skygen "drives" it, every starsync "reads" it.
 *   STARSYNC_WIRE=A  -> wired like VEGA   (SKY pins 11 and 15)
 *   STARSYNC_WIRE=B  -> wired like ALTAIR (SKY pins 13 and 16) */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include "../src/starsync.h"

/* platform_init() (laptop): nothing to set up on Linux.
 * Why: same call as on QNX, so starsync.c runs unchanged. */
void platform_init(void) { }

/* qlog() (laptop): prints the message (no slogger2 on Linux). */
void qlog(int code, const char *fmt, ...) { va_list ap; (void)code; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); }

/* qmark() (laptop): no kernel trace on Linux, so it does nothing. */
void qmark(int code, int a, int b) { (void)code; (void)a; (void)b; }

/* hr_timers_ok() (laptop): Linux timers are always high resolution. */
int  hr_timers_ok(void) { return 1; }

/* set_thread() (laptop): normal scheduling, no pinning (not needed for the logic test). */
void set_thread(int prio, int core) { (void)prio; (void)core; }

/* resmgr_start() (laptop): no QNX resource manager on Linux. */
void resmgr_start(void) { printf("(laptop build: no /dev/starsync resource manager)\n"); }

/* time_client() (laptop): devctl exists only on QNX. */
int  time_client(void) { printf("(laptop build: no devctl)\n"); return 1; }

/* ham_start() (laptop): HAM exists only on QNX, so nothing to register. */
void ham_start(const char *name) { (void)name; }

/* ham_beat() (laptop): no HAM, no heartbeat. */
void ham_beat(void) { }

/* ham_on() (laptop): always 0 - HAM is not running on Linux. */
int  ham_on(void) { return 0; }

/* tick_us() (laptop): no ClockPeriod() on Linux. */
int  tick_us(void) { return 0; }

/* hw_now() (laptop): CLOCK_MONOTONIC in ns, the Linux stand-in for ClockCycles().
 * Why: gives the shared code the same never-adjusted time base as on the Pis. */
int64_t hw_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * SEC + ts.tv_nsec;
}

/* sleep_until() (laptop): clock_nanosleep() to an absolute time.
 * Why: Linux stand-in for the QNX timer + pulse. */
void sleep_until(int64_t t)
{
    struct timespec ts = { t / SEC, t % SEC };
    if (t > hw_now()) clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* start_thread() (laptop): starts a POSIX thread running fn (same as on QNX). */
void start_thread(void *(*fn)(void *))
{
    pthread_t t;
    pthread_create(&t, NULL, fn, NULL);
}

static volatile uint32_t *wire;
static int side_b;

/* gpio_init() (laptop): the SKY wires become one shared-memory word (/dev/shm/starsync_wire).
 * STARSYNC_WIRE=B makes this node read ALTAIR's wires, otherwise VEGA's.
 * Why: lets skygen and both nodes exchange "pulses" on one laptop. */
int gpio_init(void)
{
    int fd = shm_open("/starsync_wire", O_RDWR | O_CREAT, 0666);
    if (fd < 0 || ftruncate(fd, 4) != 0) return -1;
    wire = mmap(NULL, 4, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    side_b = getenv("STARSYNC_WIRE") && getenv("STARSYNC_WIRE")[0] == 'B';
    return wire == MAP_FAILED ? -1 : 0;
}

/* gpio_setup() (laptop): nothing to configure in shared memory. */
void gpio_setup(int pin, int output) { (void)pin; (void)output; }

/* gpio_write() (laptop): sets or clears the "wire" bits atomically, all at once (like RP1). */
void gpio_write(uint32_t pins, int level)
{
    if (level) __sync_fetch_and_or(wire, pins);
    else       __sync_fetch_and_and(wire, ~pins);
}

/* gpio_read_all() (laptop): what this node's pins 13 and 15 see, then a 20 us nap.
 * Why: same bit layout as the Pi's register; the nap keeps a laptop core free. */
uint32_t gpio_read_all(void)
{
    struct timespec nap = { 0, 20000 };
    uint32_t w = *wire, r = 0;               /* read at once (the caller just took the time), */
    if (w >> (side_b ? SKY_COMMON_B : SKY_COMMON_A) & 1) r |= 1u << PIN_COMMON;
    if (w >> (side_b ? SKY_PAIR_B : SKY_PAIR_A) & 1) r |= 1u << PIN_STAGGER;
    nanosleep(&nap, NULL);                   /* then nap: don't burn a whole laptop core */
    return r;
}
