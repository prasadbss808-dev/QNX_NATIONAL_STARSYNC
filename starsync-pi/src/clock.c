/* clock.c - the corrected clock.
 *   raw time       = hardware counter + any oscillator fault the judges inject
 *   corrected time = our estimate of VEGA's time: a straight line through raw time whose
 *                    slope (rate) the servo steers. Stamps never jump and never go backwards. */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include "starsync.h"

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
/* injected oscillator fault: error = seg_offset + seg_drift * t + ramp * t^2 / 2, t since seg_start */
static int64_t seg_start;
static double  seg_offset, seg_drift, ramp;
/* the corrected clock: corrected = line_corr + line_rate * (raw - line_raw) */
static int64_t line_raw, line_corr;
static double  line_rate = 1.0;
static int     ready;                               /* first estimate applied? */
/* kept in shared memory, so it survives a restart */
typedef struct { int64_t last_stamp; int restarts; } keep_t;
static keep_t *keep, keep_local;
/* clock_init(): open /dev/shmem/starsync_keep_<node>, which remembers the last stamp and the
 * restart count, so that stamps never go backwards even after the watchdog restarts us. */
void clock_init(const char *node, int master)
{
    char name[64]; void *p = MAP_FAILED;
    snprintf(name, sizeof name, "/starsync_keep_%s", node);
    int fd = shm_open(name, O_RDWR | O_CREAT, 0666);
    if (fd >= 0 && ftruncate(fd, sizeof(keep_t)) == 0)
        p = mmap(NULL, sizeof(keep_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (fd >= 0) close(fd);
    keep = (p == MAP_FAILED) ? &keep_local : p;     /* no shared memory: keep it in RAM */
    seg_start = hw_now(); ready = master;           /* VEGA's corrected time = its raw time */
}
/* fault_error(): the injected oscillator error (ns) at hardware time hw. */
static double fault_error(int64_t hw)
{
    double t = (double)(hw - seg_start);
    return seg_offset + seg_drift * t + 0.5 * ramp * t * t / 1e9;
}
/* clock_raw(): this board's "crystal" time = hardware time + injected error. */
int64_t clock_raw(int64_t hw)
{
    pthread_mutex_lock(&lock);
    int64_t raw = hw + (int64_t)fault_error(hw);
    pthread_mutex_unlock(&lock);
    return raw;
}
/* clock_fault(): inject a fault: 'd' drift (ppm), 'r' ramp (ppm/s), 's' step (ns), 'c' clear.
 * A new segment starts where the old one ends, so only a step makes raw time jump. */
void clock_fault(char what, double value)
{
    pthread_mutex_lock(&lock);
    int64_t now = hw_now();
    seg_offset = fault_error(now);                  /* continue from where we are */
    seg_drift += ramp * (double)(now - seg_start) / 1e9; seg_start = now;
    if (what == 'd') seg_drift = value * 1e-6;
    if (what == 'r') ramp = value * 1e-6;
    if (what == 's') seg_offset += value;
    if (what == 'c') { seg_drift = 0; ramp = 0; }
    pthread_mutex_unlock(&lock);
}
/* clock_faults(): the drift and ramp injected right now (for the status screen). */
void clock_faults(double *ppm, double *ramp_ppm) { *ppm = seg_drift * 1e6; *ramp_ppm = ramp * 1e6; }
/* clock_set_line(): the servo steers the clock by setting a new line through (raw, corrected)
 * with a new rate - the clock bends, it never jumps. */
void clock_set_line(int64_t raw, int64_t corrected, double rate)
{
    pthread_mutex_lock(&lock);
    line_raw = raw; line_corr = corrected; line_rate = rate; ready = 1;
    pthread_mutex_unlock(&lock);
}
/* clock_corrected(): raw time -> our estimate of VEGA's time. */
int64_t clock_corrected(int64_t raw)
{
    pthread_mutex_lock(&lock);
    int64_t c = line_corr + (int64_t)(line_rate * (double)(raw - line_raw));
    pthread_mutex_unlock(&lock);
    return c;
}
/* clock_stamp(): like clock_corrected(), but always later than the last stamp (monotonic guard). */
int64_t clock_stamp(int64_t raw)
{
    pthread_mutex_lock(&lock);
    int64_t c = line_corr + (int64_t)(line_rate * (double)(raw - line_raw));
    if (c <= keep->last_stamp) c = keep->last_stamp + 1;   /* never backwards */
    keep->last_stamp = c;
    pthread_mutex_unlock(&lock);
    return c;
}
/* clock_ready(): 1 once the clock has its first estimate - no stamps before that. */
int clock_ready(void) { return ready; }
/* clock_restarts(): the watchdog restart counter, kept in shared memory. */
int *clock_restarts(void) { return &keep->restarts; }
