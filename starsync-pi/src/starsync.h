/* starsync.h - settings, names and functions shared by all StarSync programs.
 * Every time in StarSync is an int64_t number of nanoseconds. */
#ifndef STARSYNC_H
#define STARSYNC_H
#include <stdint.h>

/* ---------- units (nanoseconds) ---------- */
#define US  1000LL                  /* one microsecond */
#define MS  1000000LL               /* one millisecond */
#define SEC 1000000000LL            /* one second      */
/* ---------- network ---------- */
#define IP_VEGA     "172.16.6.76"   /* reference telescope = PTP master */
#define IP_LAPTOP   "172.16.6.100"  /* referee + dashboard              */
#define PORT_PTP    319             /* PTP event port (general = 320)   */
#define PORT_CMD    31903           /* text commands                    */
#define PORT_REPORT 31910           /* report lines to the laptop       */
/* ---------- GPIO numbers (header pin in the comment) ---------- */
#define PIN_COMMON   27             /* VEGA/ALTAIR input pin 13: COMMON pulse   */
#define PIN_STAGGER  22             /* VEGA/ALTAIR input pin 15: ordering pulse */
#define SKY_COMMON_A 17             /* SKY output pin 11 -> VEGA pin 13         */
#define SKY_COMMON_B 27             /* SKY output pin 13 -> ALTAIR pin 13       */
#define SKY_PAIR_A   22             /* SKY output pin 15 -> VEGA pin 15         */
#define SKY_PAIR_B   23             /* SKY output pin 16 -> ALTAIR pin 15       */
#define BIT(pin)     (1u << (pin))  /* one GPIO as a bit mask                   */
/* ---------- timing ---------- */
#define TICK (SEC / 32)             /* the servo runs 32 times a second         */
#define LOSS (400 * MS)             /* no measurement for this long = sync lost */
/* ---------- states, validity, causes ---------- */
enum { INIT, ACQUIRING, LOCKED, DEGRADED, HOLDOVER, RECOVERING, REFERENCE };
enum { VALID, PREDICTED, UNRELIABLE };
enum { NO_CAUSE, TIMEOUT, CLOCK_STEP, REF_FAULT, WATCHDOG };
extern const char *const STATE_NAME[], *const VALID_NAME[], *const CAUSE_NAME[];
/* ---------- log codes: slog2info -e CODE, System Profiler user events ---------- */
enum { L_INFO = 100, L_STATE, L_CMD, L_WARN, L_WATCHDOG, M_SERVO = 110, M_PULSE, M_PTP };
/* ---------- the estimator's memory (estimator.c) ---------- */
typedef struct {
    int64_t tol;                    /* declared tolerance, e.g. 100 us          */
    int64_t calib;                  /* asymmetry correction added to offsets    */
    int     state, cause, validity; /* state machine and what stamps are worth  */
    int64_t state_since;            /* when the current state began             */
    int     good_ticks;             /* good ticks in a row (to leave a state)   */
    int     step_checks;            /* huge surprises in a row (clock step?)    */
    int     ref_fault;              /* VEGA announced that it is faulty         */
    /* the last measurement, for the status screen */
    double  raw_off, corr_off, delay, eps;   /* offset, corrected offset, delay, error bound */
    double  min_delay;              /* smallest delay seen (slowly forgotten)   */
    int64_t min_delay_t;            /* when min_delay was last updated          */
    /* the Kalman filter: what we believe, and how unsure we are */
    int     kf_started;
    double  offset_ns;              /* ALTAIR raw time - VEGA time              */
    double  drift_ppb;              /* how fast that offset grows (ppb = ns/s)  */
    double  var_offset, cov, var_drift;      /* uncertainty (covariance P00 P01 P11) */
    int64_t kf_time;                /* the time the filter's numbers refer to   */
    double  surprise;               /* last measurement minus prediction        */
    double  noise_sq;               /* typical surprise size, squared           */
    int     surprise_side, side_count;       /* surprises on the same side in a row  */
    /* the last 16 measurements, for the proof bound and "fast packet" count */
    int64_t hist_t[16];
    double  hist_off[16], hist_eps[16];
    int     hist_n;
    int     fast[16], fast_i;
    /* uncertainty U: how far from VEGA's time we could be */
    double  U_meas, U, err, U_at_loss;       /* U at last measurement, U now, clock error */
    /* times and counters for the status screen */
    int64_t last_t, last_c, loss_t, back_t, recovery, detect, outage;
    int64_t am_from, am_to;         /* after a clock step: distrust these stamps */
    unsigned am_seq;
    unsigned n_ok, n_slow, n_bad, n_steps, n_losses;
} est_t;
/* ---------- platform.c: QNX time, timers, threads, logs ---------- */
void     platform_init(void);
int64_t  hw_now(void);              /* ClockCycles() in ns, never adjusted       */
void     sleep_until(int64_t t);    /* high-resolution timer + pulse             */
void     set_thread(int prio, int core);     /* SCHED_FIFO priority, one core   */
void     start_thread(void *(*fn)(void *));
int      hr_timers_ok(void);
int      tick_us(void);             /* the QNX system tick                       */
void     qlog(int code, const char *fmt, ...);  /* console + slogger2 + trace   */
void     qmark(int code, int a, int b);         /* two numbers into the trace   */
/* ---------- gpio.c: Raspberry Pi 5 GPIO (RP1 chip) ---------- */
int      gpio_init(void);
void     gpio_setup(int pin, int output);
uint32_t gpio_read_all(void);
void     gpio_write(uint32_t pins, int level);
/* ---------- clock.c: the corrected clock ---------- */
void    clock_init(const char *node, int master);
int64_t clock_raw(int64_t hw);      /* hardware time + injected oscillator fault */
int64_t clock_corrected(int64_t raw);        /* raw -> our estimate of VEGA time */
int64_t clock_stamp(int64_t raw);   /* corrected, and never backwards            */
void    clock_set_line(int64_t raw, int64_t corrected, double rate);
int     clock_ready(void);
void    clock_fault(char what, double value);
void    clock_faults(double *ppm, double *ramp);
int    *clock_restarts(void);
/* ---------- estimator.c: Kalman filter + state machine ---------- */
void est_init(est_t *s, int64_t tol);
void est_measurement(est_t *s, int64_t t1, int64_t t2, int64_t t3, int64_t t4, int ref_fault);
void est_tick(est_t *s, int64_t now);
/* ---------- commands.c, resmgr.c, ham.c ---------- */
int  status_text(char *out, int n);
int  time_now(int64_t *corrected, double *U, int *validity);
int  time_text(char *out, int n);
int  do_command(const char *cmd, char *reply, int n);
void resmgr_start(void);
int  time_client(void);
void ham_start(const char *name);
void ham_beat(void);
int  ham_on(void);
#endif
