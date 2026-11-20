/* commands.c - what people see and type: the status text, the time, and the commands
 * (the dashboard, `sh scripts/cmd.sh` and /dev/starsync all end up here). */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "node.h"

/* status_text(): the full status screen: state, U, offset, drift, delay, recovery, counters,
 * pulses, injected faults and QNX health. Used by "status" and cat /dev/starsync/status. */
int status_text(char *o, int n)
{
    est_t *s = &g.est;
    double ppm, ramp, avg, max;
    int k = 0;                                      /* characters written so far */
    lat_stats(&avg, &max); clock_faults(&ppm, &ramp);
    k += snprintf(o + k, n - k, "StarSync %s - %s   up %.1f s   watchdog restarts %d\n", g.name,
                  g.is_master ? "PTP master (reference)" : "PTP slave", (hw_now() - g.start) / 1e9,
                  *clock_restarts());
    if (!g.is_master) {
        double in_state_s = (clock_raw(hw_now()) - s->state_since) / 1e9;
        double min_delay_us = s->min_delay > 1e17 ? 0 : s->min_delay / 1e3;
        k += snprintf(o + k, n - k, "STATE      %-11s for %.1f s   cause %s\n", STATE_NAME[s->state],
                      in_state_s, CAUSE_NAME[s->cause]);
        k += snprintf(o + k, n - k, "VALIDITY   %-11s U = +-%.1f us   tolerance +-%lld us\n",
                      VALID_NAME[g.validity], g.U / 1e3, (long long)(s->tol / US));
        k += snprintf(o + k, n - k, "OFFSET     raw %+.1f us   corrected %+.1f us   (to VEGA, last exchange)\n",
                      s->raw_off / 1e3, s->corr_off / 1e3);
        k += snprintf(o + k, n - k, "DRIFT      %+.3f ppm   (+-%.3f)\n", s->drift_ppb / 1e3, sqrt(s->var_drift) / 1e3);
        k += snprintf(o + k, n - k, "DELAY      one-way %.1f us   minimum %.1f us   error bound %.1f us\n",
                      s->delay / 1e3, min_delay_us, s->eps / 1e3);
        k += snprintf(o + k, n - k, "RECOVERY   last %.0f ms   (loss detected after %.0f ms, outage %.2f s, losses %u)\n",
                      s->recovery / 1e6, s->detect / 1e6, s->outage / 1e9, s->n_losses);
        k += snprintf(o + k, n - k, "COUNTS     ok %u  slow %u  rejected %u  clock-steps %u   calib %+.1f us\n",
                      s->n_ok, s->n_slow, s->n_bad, s->n_steps, s->calib / 1e3);
    }
    k += snprintf(o + k, n - k, "PULSES     common %u (pin 13)   ordering %u (pin 15)%s\n", g.n_common, g.n_order,
                  g.have_gpio ? "" : "   GPIO NOT AVAILABLE");
    k += snprintf(o + k, n - k, "FAULTS     drift %+.2f ppm  ramp %+.2f ppm/s | delay %.0f-%.0f us  asym %.0f us | "
                  "blackout %s | health %s\n", ppm, ramp, g.delay_min_us, g.delay_max_us, g.asym_us,
                  hw_now() < g.blackout_until ? "ON" : "off", g.health_fault ? "FAULT" : "ok");
    k += snprintf(o + k, n - k, "QNX        HR timers %s (tick %d us) | servo wake-up late: avg %.1f  max %.1f us | "
                  "watchdog %s | HAM %s\n", hr_timers_ok() ? "ON" : "OFF", tick_us(), avg, max,
                  g.wd_fault ? "FAULT" : "ok (core 3)", ham_on() ? "supervising" : "off");
    if (k >= n) k = n - 1;   /* the text was cut to fit */
    return k;
}
/* time_now(): the corrected time right now (never backwards), with its U and validity.
 * Returns -1 before the first estimate. Used by /dev/starsync/time and devctl. */
int time_now(int64_t *corrected, double *U, int *validity)
{
    if (!clock_ready()) return -1;
    *corrected = clock_stamp(clock_raw(hw_now())); *U = g.U; *validity = g.validity;
    return 0;
}
/* time_text(): the time as one line, for: cat /dev/starsync/time */
int time_text(char *o, int n)
{
    int64_t c; double U; int v;
    if (time_now(&c, &U, &v) != 0) return snprintf(o, n, "not synchronised yet\n");
    return snprintf(o, n, "%lld.%09lld s   U +-%.1f us   %s\n", (long long)(c / SEC), (long long)(c % SEC),
                    U / 1e3, VALID_NAME[v]);
}
/* do_command(): run one text command and write the reply. This is how the judges' faults
 * are injected (drift, ramp, step, delay, asym, blackout, health, hang) and how we tune (tol, calib). */
int do_command(const char *cmd, char *r, int n)
{
    char word[16] = "", word2[16] = "";
    double a = 0, b = 0;
    sscanf(cmd, "%15s %lf %lf", word, &a, &b);     /* e.g. "delay 0 2000"      */
    sscanf(cmd, "%*s %15s %lf", word2, &b);        /* e.g. "hang servo 2000"   */
    if (strcmp(word, "status") == 0) return status_text(r, n);
    qlog(L_CMD, ">> command: %s\n", cmd);
    if (strcmp(word, "drift") == 0) clock_fault('d', a);   /* oscillator drift, ppm      */
    else if (strcmp(word, "ramp") == 0) clock_fault('r', a);   /* gradual drift, ppm/s       */
    else if (strcmp(word, "step") == 0) clock_fault('s', a * US);   /* clock jump, us             */
    else if (strcmp(word, "clear") == 0) clock_fault('c', 0);   /* remove drift and ramp      */
    else if (strcmp(word, "delay") == 0) {          /* random extra delay, us     */
        g.delay_min_us = a; g.delay_max_us = b > a ? b : a;
    } else if (strcmp(word, "asym") == 0)           /* one-way extra delay, us    */
        g.asym_us = a;
    else if (strcmp(word, "blackout") == 0)         /* no packets for a seconds   */
        g.blackout_until = hw_now() + (int64_t)(a * SEC);
    else if (strcmp(word, "netoff") == 0) {         /* remove all network faults  */
        g.delay_min_us = 0; g.delay_max_us = 0; g.asym_us = 0; g.blackout_until = 0;
    } else if (strcmp(word, "health") == 0)         /* "health fault" / "health ok" */
        g.health_fault = strcmp(word2, "fault") == 0;
    else if (strcmp(word, "tol") == 0) g.est.tol = (int64_t)(a * US);   /* declared tolerance, us     */
    else if (strcmp(word, "calib") == 0) g.est.calib += (int64_t)(a * US);   /* asymmetry correction, us   */
    else if (strcmp(word, "hang") == 0) {           /* freeze one thread, ms      */
        for (int i = 0; i < 4; i++)
            if (strcmp(word2, THREAD_NAME[i]) == 0) g.hang_ms[i] = (int)b;
    } else
        return snprintf(r, n, "unknown '%s'. Commands: status drift ramp step clear delay asym blackout "
                        "netoff health tol calib hang\n", word);
    return snprintf(r, n, "ok: %s\n", cmd);
}
