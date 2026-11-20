/* platform.c - the QNX basics: time, high-resolution timers, priorities, cores, logging. */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <sys/neutrino.h>
#include <sys/slog2.h>
#include <sys/syspage.h>
#include <sys/procmgr.h>
#include <sys/trace.h>
#include "starsync.h"

static uint64_t cycles_per_sec;                     /* ClockCycles() ticks per second */
static int hr_ok;                                   /* high-resolution timers granted? */
static slog2_buffer_t slog;                         /* our slogger2 buffer */
extern char *__progname;                            /* "starsync", "skygen" or "hog" */
/* platform_init(): every program calls this first: read the counter's speed, ask QNX for
 * high-resolution timers (needs root) and register our slogger2 log. */
void platform_init(void)
{
    static slog2_buffer_set_config_t config = {
        .num_buffers = 1, .verbosity_level = SLOG2_INFO, .buffer_config = { { "events", 8 } } };   /* 8 x 4 KB */
    cycles_per_sec = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    procmgr_ability(0, PROCMGR_ADN_ROOT | PROCMGR_AOP_ALLOW | PROCMGR_AID_HIGH_RESOLUTION_TIMER, PROCMGR_AID_EOL);
    config.buffer_set_name = __progname;
    if (slog2_register(&config, &slog, 0) == -1) slog = NULL;   /* no slogger2: console only */
}
/* qlog(): one message to three places: the console, slogger2 (slog2info -b starsync)
 * and the kernel trace (System Profiler). */
void qlog(int code, const char *fmt, ...)
{
    char m[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m, sizeof m, fmt, ap);
    va_end(ap);
    fputs(m, stdout);
    m[strcspn(m, "\n")] = 0;                        /* the log and the trace take one line */
    if (slog) slog2f(slog, code, code >= L_WARN ? SLOG2_WARNING : SLOG2_INFO, "%s", m);
    trace_logf(code, "%s", m);                      /* costs nothing when no trace runs */
}
/* qmark(): two numbers into the kernel trace, to see our activity in the System Profiler. */
void qmark(int code, int a, int b) { trace_logi(code, (unsigned)a, (unsigned)b); }
/* hw_now(): QNX's free-running hardware counter in nanoseconds. Never adjusted, never jumps. */
int64_t hw_now(void)
{
    uint64_t c = ClockCycles();
    uint64_t whole_seconds = c / cycles_per_sec;    /* split to avoid overflow */
    uint64_t rest = c % cycles_per_sec;
    return (int64_t)(whole_seconds * 1000000000ULL + rest * 1000000000ULL / cycles_per_sec);
}
/* set_thread(): give the calling thread a fixed real-time priority and ONE core to run on. */
void set_thread(int prio, int core)
{
    struct sched_param sp = { .sched_priority = prio };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    ThreadCtl(_NTO_TCTL_RUNMASK, (void *)(uintptr_t)(1u << core));
}
/* start_thread(): start a thread; it sets its own priority and core with set_thread(). */
void start_thread(void *(*fn)(void *))
{
    pthread_t t;
    pthread_create(&t, NULL, fn, NULL);
}
/* hr_timers_ok(): did QNX accept our 10 us timer tolerance? (shown in the status) */
int hr_timers_ok(void) { return hr_ok; }
/* tick_us(): the QNX system tick in us (shown in the status, to compare with our timers). */
int tick_us(void)
{
    struct _clockperiod cp;
    if (ClockPeriod(CLOCK_REALTIME, NULL, &cp, 0) == -1) return 0;
    return (int)(cp.nsec / 1000);
}
/* sleep_until(): sleep until hardware time t, precisely. On its first call a thread gets its
 * own channel and timer; the timer sends a pulse and MsgReceivePulse() waits for it. */
static __thread int my_channel = -1;
static __thread timer_t my_timer;
void sleep_until(int64_t t)
{
    int64_t ns = t - hw_now();
    if (ns <= 0) return;
    if (my_channel < 0) {                           /* first time for this thread */
        struct sched_param sp;
        int policy;
        pthread_getschedparam(pthread_self(), &policy, &sp);
        my_channel = ChannelCreate(0);
        int connection = ConnectAttach(0, 0, my_channel, _NTO_SIDE_CHANNEL, 0);
        struct sigevent ev;
        SIGEV_PULSE_INIT(&ev, connection, sp.sched_priority, _PULSE_CODE_MINAVAIL, 0);
        timer_create(CLOCK_MONOTONIC, &ev, &my_timer);
        struct itimerspec tolerance = { .it_value.tv_nsec = 10000 };   /* 10 us: high resolution */
        hr_ok = timer_settime(my_timer, TIMER_TOLERANCE, &tolerance, NULL) == 0;
    }
    struct itimerspec it = { .it_value = { ns / SEC, ns % SEC } };
    timer_settime(my_timer, 0, &it, NULL);          /* one shot, relative */
    struct _pulse pulse;
    MsgReceivePulse(my_channel, &pulse, sizeof pulse, NULL);
}
