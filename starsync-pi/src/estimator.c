/* estimator.c - ALTAIR's brain.
 *   est_measurement(): one PTP exchange -> Kalman filter -> offset and drift (+ how sure we are)
 *   est_tick():        32 times a second -> uncertainty U, state machine, validity, steer the clock */
#include <math.h>
#include <string.h>
#include "starsync.h"

const char *const STATE_NAME[] = { "INIT", "ACQUIRING", "LOCKED", "DEGRADED", "HOLDOVER", "RECOVERING", "REFERENCE" };
const char *const VALID_NAME[] = { "VALID", "PREDICTED", "UNRELIABLE" };
const char *const CAUSE_NAME[] = { "-", "TIMEOUT", "CLOCK_STEP", "REF_FAULT", "WATCHDOG" };
#define STAMP_NOISE     (2.0 * US)          /* noise of one software timestamp              */
#define MEAS_NOISE_MIN  (3.0 * US)          /* a measurement is never trusted more than this */
#define MAX_ERROR_BOUND (2.0 * MS)          /* slower measurements are ignored              */
#define OFFSET_WANDER   1e4                 /* offset may wander 100 ns per sqrt(second)    */
#define DRIFT_WANDER    400.0               /* drift may wander 20 ppb per sqrt(second)     */
#define DRIFT_RESET     (20000.0 * 20000.0) /* "drift changed": allow 20 ppm more doubt     */
#define HOLDOVER_DRIFT  1000.0              /* holdover: drift may be 1 ppm off (ppb)       */
#define PROOF_DRIFT     20000.0             /* proof: drift may have jumped 20 ppm (ppb)    */
/* est_init(): start empty: INIT, U = 1 s (unknown), UNRELIABLE until PTP proves better. */
void est_init(est_t *s, int64_t tol)
{
    memset(s, 0, sizeof *s);
    s->tol = tol; s->min_delay = 1e18; s->U = 1e9; s->validity = UNRELIABLE;
}
/* set_state(): change state, remember why and when, and log it (console, slogger2, trace). */
static void set_state(est_t *s, int new_state, int cause, int64_t now)
{
    if (new_state == s->state) return;
    qlog(L_STATE, "SYNC: %s -> %s  (cause %s, U %.1f us, e %.1f us)\n", STATE_NAME[s->state],
         STATE_NAME[new_state], CAUSE_NAME[cause], s->U / 1e3, s->err / 1e3);
    s->state = new_state; s->cause = cause; s->state_since = now; s->good_ticks = 0;
}
/* est_measurement(): one PTP exchange in.
 *   T1 VEGA sends Sync -> T2 we get it;   T3 we send Delay_Req -> T4 VEGA gets it
 *   offset = ((T2-T1) - (T4-T3)) / 2      delay = ((T2-T1) + (T4-T3)) / 2
 * Then the Kalman filter updates offset and drift, and U is recomputed. */
void est_measurement(est_t *s, int64_t t1, int64_t t2, int64_t t3, int64_t t4, int ref_fault)
{
    double forward = (double)(t2 - t1);             /* = delay + offset */
    double backward = (double)(t4 - t3);            /* = delay - offset */
    double z = (forward - backward) / 2.0 + (double)s->calib;    /* measured offset */
    double d = (forward + backward) / 2.0;                        /* measured delay  */
    s->ref_fault = ref_fault;
    if (d < 0 || d > 50.0 * MS) { s->n_bad++; return; }          /* impossible: ignore */
    /* error bound eps: any delay above the smallest one seen could all be on one side */
    if (s->min_delay_t)
        s->min_delay += 500.0 * (double)(t2 - s->min_delay_t) / 1e9;   /* forget 0.5 us per second */
    s->min_delay_t = t2;
    if (d < s->min_delay) s->min_delay = d;
    double eps = d - s->min_delay + STAMP_NOISE;
    s->raw_off = z; s->delay = d; s->eps = eps;                  /* for the status screen */
    s->corr_off = (double)(clock_corrected(t2) - t2) + z;
    s->fast[s->fast_i % 16] = eps <= 20.0 * US;    /* was this a fast packet? */
    s->fast_i++;
    if (eps > MAX_ERROR_BOUND) { s->n_slow++; return; }          /* far too slow: ignore */
    if (ref_fault) return;                          /* VEGA says it is faulty: don't use it */
    double R = MEAS_NOISE_MIN * MEAS_NOISE_MIN + eps * eps / 3.0;  /* how noisy this measurement is */
    if (!s->kf_started) {                           /* the first measurement starts the filter */
        s->offset_ns = z; s->drift_ppb = 0;         /* offset as measured, drift unknown */
        s->var_offset = R; s->cov = 0; s->var_drift = DRIFT_RESET;
        s->kf_time = t2; s->noise_sq = R; s->kf_started = 1;
    } else {
        /* KALMAN STEP 1 - PREDICT: move our belief to now; the doubt grows */
        double dt = (double)(t2 - s->kf_time) / 1e9;
        s->offset_ns += s->drift_ppb * dt;
        double grow_offset = 2 * dt * s->cov + dt * dt * s->var_drift + DRIFT_WANDER * dt * dt * dt / 3 + OFFSET_WANDER * dt;
        double grow_cov = dt * s->var_drift + DRIFT_WANDER * dt * dt / 2;
        s->var_offset += grow_offset; s->cov += grow_cov; s->var_drift += DRIFT_WANDER * dt;
        s->kf_time = t2;
        /* KALMAN STEP 2 - SURPRISE: measured minus predicted */
        double surprise = z - s->offset_ns;
        double expected_sq = s->var_offset + R;     /* how big a surprise should be (squared) */
        double limit = fmax(8.0 * sqrt(s->noise_sq), 100.0 * US);
        limit = fmax(limit, 2.0 * s->U);
        if (fabs(surprise) > eps + limit) {
            /* far too big to be noise: hold it. Three in a row = the clock really jumped */
            s->step_checks++;
            if (s->step_checks < 3) { s->n_bad++; return; }
            qlog(L_WARN, "SYNC: CLOCK STEP of %+.1f us confirmed by 3 measurements -> restart the offset\n",
                 surprise / 1e3);
            s->n_steps++;
            s->am_from = s->last_c; s->am_to = clock_corrected(t2); s->am_seq++;   /* distrust these stamps */
            s->offset_ns = z; s->var_offset = R; s->cov = 0;          /* new offset, keep the drift */
            s->hist_n = 0; s->step_checks = 0;
            s->back_t = t2; s->outage = 0; s->detect = t2 - s->last_t;
            set_state(s, RECOVERING, CLOCK_STEP, t2);
        } else {
            if (eps < 50.0 * US) s->step_checks = 0;
            /* drift changed? then surprises keep landing on the same side */
            int side = 0;
            if (surprise > 2 * sqrt(expected_sq)) side = 1;
            if (surprise < -2 * sqrt(expected_sq)) side = -1;
            if (side != 0 && side == s->surprise_side) s->side_count++;
            else if (side != 0) s->side_count = 1;
            else s->side_count = 0;
            s->surprise_side = side;
            if (s->side_count >= 4) {
                qlog(L_WARN, "SYNC: drift changed (4 surprises on one side) -> Kalman opens up the drift\n");
                s->var_drift += DRIFT_RESET; s->var_offset += surprise * surprise;
                s->side_count = 0; expected_sq = s->var_offset + R;
            }
            /* KALMAN STEP 3 - UPDATE: move part of the way towards the measurement */
            double gain_offset = s->var_offset / expected_sq;
            double gain_drift = s->cov / expected_sq;
            s->offset_ns += gain_offset * surprise;
            s->drift_ppb += gain_drift * surprise;
            s->var_drift -= gain_drift * s->cov;    /* the doubt shrinks */
            s->cov *= 1 - gain_offset; s->var_offset *= 1 - gain_offset;
            s->surprise = surprise;
            if (eps <= 20.0 * US)                   /* remember how noisy fast packets are */
                s->noise_sq = 0.9 * s->noise_sq + 0.1 * surprise * surprise;
        }
    }
    /* U = the larger of the filter's own 3-sigma and a PROOF: measurement i says "the truth is
     * within eps_i of z_i"; carried to now (the drift may have jumped since), the tightest wins. */
    int h = s->hist_n % 16;
    s->hist_t[h] = t2; s->hist_off[h] = z; s->hist_eps[h] = eps; s->hist_n++;
    double proof = 1e18;
    for (int i = 0; i < 16 && i < s->hist_n; i++) {
        double age = (double)(t2 - s->hist_t[i]) / 1e9;
        double bound = fabs(s->offset_ns - s->drift_ppb * age - s->hist_off[i]) + s->hist_eps[i]
                       + 2.0 * sqrt(s->noise_sq) + PROOF_DRIFT * age;
        if (bound < proof) proof = bound;
    }
    s->U_meas = fmax(3.0 * sqrt(s->var_offset), proof);
    s->n_ok++; s->last_t = t2; s->last_c = clock_corrected(t2);
    if (s->state == INIT) set_state(s, ACQUIRING, NO_CAUSE, t2);
    if (s->state == HOLDOVER) {                     /* the link is back */
        s->back_t = t2; s->outage = t2 - s->loss_t;
        set_state(s, RECOVERING, s->cause, t2);
    }
}
/* holdover(): the link is lost: remember when and how sure we were, then coast on the drift. */
static void holdover(est_t *s, int64_t now, int cause)
{
    s->loss_t = s->last_t; s->detect = now - s->last_t; s->U_at_loss = s->U; s->n_losses++;
    set_state(s, HOLDOVER, cause, now);
}
/* est_tick() - 32 times a second: where is VEGA's time now, how far off is our clock (err),
 * how sure are we (U), which state are we in, what are stamps worth, and steer the clock. */
void est_tick(est_t *s, int64_t now)
{
    if (s->state == INIT) return;
    double offset_now = s->offset_ns + s->drift_ppb * (double)(now - s->kf_time) / 1e9;
    double target = (double)now - offset_now;       /* VEGA's time now, as we believe it */
    int64_t c_now = clock_corrected(now);
    s->err = (double)c_now - target;                /* our corrected clock's error */
    /* U grows with time since the last measurement (or since the link was lost) */
    double grow = 3.0 * sqrt(s->var_drift) + HOLDOVER_DRIFT;     /* ns per second */
    double coast = grow * (double)(now - s->kf_time) / 1e9;          /* growth since the last measurement */
    if (s->state == HOLDOVER) s->U = s->U_at_loss + grow * (double)(now - s->loss_t) / 1e9;
    else s->U = s->U_meas + coast + fabs(s->err);
    int ok = s->U <= s->tol && fabs(s->err) < s->tol / 4.0;
    int64_t age = now - s->last_t;                  /* time since the last measurement */
    /* the state machine */
    if (s->state == ACQUIRING) {                    /* first lock: 1 s (32 ticks) of good ticks */
        s->good_ticks = ok ? s->good_ticks + 1 : 0;
        if (s->good_ticks >= 32) set_state(s, s->U > s->tol / 2.0 ? DEGRADED : LOCKED, NO_CAUSE, now);
    } else if (s->state == RECOVERING) {            /* after a loss: 4 good ticks */
        s->good_ticks = ok ? s->good_ticks + 1 : 0;
        if (age > LOSS) holdover(s, now, TIMEOUT);
        else if (s->good_ticks >= 4) {
                s->recovery = now - s->back_t;
                qlog(L_STATE, "SYNC: recovered in %.0f ms (outage %.2f s, loss noticed after %.0f ms)\n",
                     s->recovery / 1e6, s->outage / 1e9, s->detect / 1e6);
            set_state(s, s->U > s->tol / 2.0 ? DEGRADED : LOCKED, NO_CAUSE, now);
        }
    } else if (s->state == LOCKED || s->state == DEGRADED) {
        if (s->ref_fault) holdover(s, now, REF_FAULT);              /* VEGA is faulty */
        else if (age > LOSS) holdover(s, now, TIMEOUT);             /* no measurement for 400 ms */
        else {
            int fast = 0;                           /* fast packets among the last 16 */
            for (int i = 0; i < 16; i++) fast += s->fast[i];
            set_state(s, (fast < 8 || s->U > s->tol / 2.0) ? DEGRADED : LOCKED, NO_CAUSE, now);
        }
    }
    /* what a timestamp is worth right now */
    if (s->step_checks) s->validity = UNRELIABLE;   /* a suspicious jump is being checked */
    else if (s->state == HOLDOVER)
        s->validity = (s->U <= s->tol && s->cause != CLOCK_STEP) ? PREDICTED : UNRELIABLE;
    else if (s->state == LOCKED || s->state == DEGRADED || s->state == RECOVERING)
        s->validity = s->U <= s->tol ? VALID : UNRELIABLE;
    else s->validity = UNRELIABLE;
    /* steer the corrected clock: change its RATE, never jump (except the very first time) */
    if (!clock_ready()) { clock_set_line(now, (int64_t)target, 1.0 - s->drift_ppb * 1e-9); s->err = 0; return; }
    double max_slew = 500e-6, settle_s = 1.0;      /* normal: at most 500 ppm, error gone in ~1 s */
    if (s->state == ACQUIRING || s->state == RECOVERING) { max_slew = 0.02; settle_s = 0.1; }   /* catch up fast */
    double correction = s->err / (settle_s * 1e9);
    if (correction > max_slew) correction = max_slew;
    if (correction < -max_slew) correction = -max_slew;
    clock_set_line(now, c_now, 1.0 - s->drift_ppb * 1e-9 - correction);
}
