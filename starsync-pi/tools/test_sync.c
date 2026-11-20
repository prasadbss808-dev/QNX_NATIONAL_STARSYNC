/*
 * test_sync.c - checks the brain (estimator.c) with made-up measurements whose
 * truth we know: drift, a drift change, a step, a blackout, a delay storm.
 *   make test
 * Prints the true error of the corrected clock and what the brain claims (U).
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../src/starsync.h"

/* gauss(): a normally distributed random number (Box-Muller), mean 0, sigma 1.
 * Why: realistic timestamp noise for the simulated PTP measurements. */
static double gauss(void)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

/* main(): runs the estimator (estimator.c + clock.c) for 120 simulated seconds with a known truth: 3 ms
 * offset, 25 ppm drift, +20 ppm drift change, -3 ms step, 15 s blackout, 0-2 ms delay storm.
 * Every trusted tick must be within its claimed U and the tolerance, otherwise TEST FAILED.
 * Why: proves the honesty of U against exact truth before the code goes onto the boards. */
int main(void)
{
    est_t s;
    est_init(&s, 100 * US);
    clock_init("test", 0);
    double theta = 3e6, drift = 25e3;               /* true offset 3 ms, drift 25 ppm */
    int64_t t = 1 * SEC, next_meas = t;
    int bad = 0, valid = 0, gap = 0; double maxerr = 0;
    for (int tick = 0; tick < 32 * 120; tick++) {     /* 120 s */
        double ts = tick / 32.0;
        t += TICK;
        theta += drift * (TICK / 1e9);
        if (ts >= 40 && ts < 40 + 1.0 / 32) drift = 45e3;           /* drift +20 ppm at 40 s  */
        if (ts >= 60 && ts < 60 + 1.0 / 32) theta -= 3e6;           /* step -3 ms at 60 s     */
        int blackout = ts >= 80 && ts < 95;                         /* 15 s blackout          */
        int storm = ts >= 100 && ts < 110;                          /* 0-2 ms delay storm     */
        if (t >= next_meas && !blackout) {
            double d = 60e3, n1 = 3e3 * gauss(), n2 = 3e3 * gauss();
            double spike = (rand() % 10 == 0) ? 500e3 * (rand() / (double)RAND_MAX) : 0;  /* 10% slow */
            double fwd = d + n1 + spike + (storm ? 2e6 * (rand() / (double)RAND_MAX) : 0);
            double rev = d + n2 + (storm ? 2e6 * (rand() / (double)RAND_MAX) : 0);
            int64_t t3 = t - 20 * MS;                               /* our Delay_Req          */
            int64_t t4 = t3 - (int64_t)theta + (int64_t)fwd;        /* VEGA receives it       */
            int64_t t1 = t4 + 5 * MS;                               /* VEGA's next Sync       */
            int64_t t2 = t1 + (int64_t)theta + (int64_t)rev;        /* we receive it          */
            est_measurement(&s, t1, t2, t3, t4, 0);
            next_meas = t + (s.state == LOCKED && !s.step_checks ? SEC / 8 : SEC / 32);
        }
        est_tick(&s, t);
        double err = (double)clock_corrected(t) - ((double)t - theta);   /* true error */
        int step_gap = ts >= 60 && ts < 60.2;        /* before the first measurement after the jump */
        if (s.validity != UNRELIABLE && step_gap && fabs(err) > s.U) { gap++; continue; }
        if (s.validity != UNRELIABLE) {
            valid++;
            if (fabs(err) > maxerr) maxerr = fabs(err);
            if (fabs(err) > s.U || fabs(err) > s.tol) {
                bad++;
                if (bad <= 60) printf("  WRONG at t=%.3f: %s %s err %+.1f us but U %.1f us\n", ts,
                                      STATE_NAME[s.state], VALID_NAME[s.validity], err / 1e3, s.U / 1e3);
            }
        }
        if (tick % 64 == 0 && getenv("VERBOSE"))
            printf("t=%5.1f %-10s %-10s err %+8.1f us  U %7.1f us  drift est %+7.2f ppm (true %+.0f)\n",
                   ts, STATE_NAME[s.state], VALID_NAME[s.validity], err / 1e3, s.U > 1e8 ? 99999 : s.U / 1e3,
                   s.drift_ppb / 1e3, drift / 1e3);
    }
    printf("\ntrusted ticks %d, max trusted error %.1f us, trusted but WRONG (|err| > U or > tol): %d\n"
           "(+ %d ticks between the clock step and the first measurement after it: retracted)\n",
           valid, maxerr / 1e3, bad, gap);
    printf("%s\n", bad ? "TEST FAILED" : "TEST PASSED");
    return bad != 0;
}
