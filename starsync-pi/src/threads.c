/* threads.c - the five real-time threads of a node. Priority / core:
 *   watchdog 70 / 3    rx 64 / 2    capture 63 / 2    servo 55 / 1    report 20 / 0 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include "node.h"

/* beat(): a thread says "I am alive" for the watchdog.
 * The "hang" command freezes the thread right here, to test the watchdog. */
static void beat(int thread)
{
    g.heartbeat[thread] = hw_now();
    if (g.hang_ms[thread]) {
        qlog(L_WARN, "self-test: %s thread frozen for %d ms\n", THREAD_NAME[thread], g.hang_ms[thread]);
        sleep_until(hw_now() + g.hang_ms[thread] * MS);
        g.hang_ms[thread] = 0;
    }
}
/* lat_stats(): average and worst servo wake-up lateness over the last 10 s, in us.
 * Shows how precisely the QNX high-resolution timer wakes us. */
void lat_stats(double *avg, double *max)
{
    int n = g.late_i < 320 ? g.late_i : 320;
    *avg = 0; *max = 0;
    for (int i = 0; i < n; i++) {
        *avg += g.late_us[i] / n;
        if (g.late_us[i] > *max) *max = g.late_us[i];
    }
}
/* rx_thread() - priority 64, core 2: wait for PTP packets and take the arrival time FIRST.
 * A dedicated high-priority thread keeps the time between arrival and stamp small. */
void *rx_thread(void *arg)
{
    set_thread(64, 2);
    struct pollfd fds[2] = { { g.sock_event, POLLIN, 0 }, { g.sock_general, POLLIN, 0 } };
    for (;;) {
        int ready = poll(fds, 2, 100);              /* wait up to 100 ms for a packet */
        int64_t t_arrival = clock_raw(hw_now());    /* timestamp first */
        beat(RX);
        for (int k = 0; ready > 0 && k < 2; k++) {
            if (!(fds[k].revents & POLLIN)) continue;
            uint8_t msg[128]; struct sockaddr_in from; socklen_t from_len = sizeof from;
            int len = recvfrom(fds[k].fd, msg, sizeof msg, 0, (struct sockaddr *)&from, &from_len);
            if (len < 44 || (msg[1] & 0x0F) != 2) continue;   /* not a PTP version 2 message */
            int64_t t = t_arrival;
            if (!net_fault(&t)) continue;                     /* judges' blackout: dropped */
            if (g.is_master) master_rx(msg, ptp_type(msg), ptp_seq(msg), t, from);
            else if (from.sin_addr.s_addr == g.vega.sin_addr.s_addr)
                slave_rx(msg, ptp_type(msg), ptp_seq(msg), ptp_time(msg), t);
        }
    }
    return arg;
}
/* servo_thread() - priority 55, core 1, 32 times a second on a high-resolution timer.
 * ALTAIR starts a PTP exchange (Delay_Req = T3) and runs the Kalman tick that steers the clock. */
void *servo_thread(void *arg)
{
    set_thread(55, 1);
    int64_t next = hw_now(), next_request = 0;      /* next wake-up, next Delay_Req (raw time) */
    for (;;) {
        next += TICK;
        sleep_until(next);
        int64_t now = hw_now();
        g.late_us[g.late_i % 320] = (now - next) / 1e3; g.late_i++;   /* how late did QNX wake us? */
        qmark(M_SERVO, (int)(now - next), (int)fmin(g.U, 2e9));  /* lateness ns, U ns */
        if (now - next > TICK) next = now;          /* very late: don't try to catch up */
        beat(SERVO);
        if (g.is_master) {                          /* VEGA is the reference itself */
            g.validity = (g.health_fault || g.wd_fault) ? UNRELIABLE : VALID; g.U = 2 * US;
            continue;
        }
        int64_t raw = clock_raw(now);
        if (raw >= next_request && now >= g.blackout_until) {    /* start a new exchange */
            uint8_t m[64];
            pthread_mutex_lock(&g.lock);
            g.seq++;
            int len = ptp_build(m, DELAY_REQ, g.seq, 0, NULL, 0);
            g.got_sync = 0; g.got_follow_up = 0; g.got_delay_resp = 0;     /* a new exchange */
            g.t3 = clock_raw(hw_now());             /* T3: stamp, then send at once */
            pthread_mutex_unlock(&g.lock);
            send_to(g.sock_event, m, len, g.vega, g.master_port);
            if (g.est.state == LOCKED && !g.est.step_checks) next_request = raw + SEC / 8;   /* LOCKED: 8/s */
            else next_request = raw + SEC / 32;                                           /* learning: 32/s */
        }
        pthread_mutex_lock(&g.lock);
        est_tick(&g.est, raw);                      /* Kalman tick: U, state, steering */
        g.validity = g.wd_fault ? UNRELIABLE : g.est.validity; g.U = g.est.U;
        pthread_mutex_unlock(&g.lock);
    }
    return arg;
}
/* add_event(): stamp one SKY pulse and put it into the ring for the report thread.
 * The ring needs no lock: only capture writes head, only report moves tail. */
static void add_event(char kind, unsigned seq, int64_t hw, double half_gap)
{
    unsigned h = g.head;
    qmark(M_PULSE, kind, (int)seq);                 /* System Profiler: pulse seen */
    if (!clock_ready()) return;                     /* no stamps before the first estimate */
    if (h - g.tail >= 256) return;                  /* ring full */
    pulse_t *p = &g.ring[h % 256];
    p->kind = kind; p->seq = seq;
    p->raw = clock_raw(hw);
    p->stamp = clock_stamp(p->raw);                 /* corrected, never backwards */
    p->U = g.U + half_gap;
    p->validity = (g.U + half_gap > g.est.tol) ? UNRELIABLE : g.validity;
    g.head = h + 1;                                 /* now the report thread may read it */
}
/* capture_thread() - priority 63, core 2: read the GPIO pins in a tight loop and stamp each
 * rising edge. On the Pi 5 this is faster and steadier than a GPIO interrupt. */
void *capture_thread(void *arg)
{
    set_thread(63, 2);
    uint32_t before = gpio_read_all();
    int64_t t_before = hw_now(), last_beat = t_before;
    for (;;) {
        int64_t t_now = hw_now();                   /* time first, then the pins */
        uint32_t pins = gpio_read_all();
        uint32_t rising = pins & ~before;           /* pins that went from 0 to 1 */
        int64_t middle = t_before + (t_now - t_before) / 2;      /* the edge came between reads */
        double half_gap = (t_now - t_before) / 2.0;
        if (rising & BIT(PIN_COMMON)) { g.n_common++; add_event('C', g.n_common, middle, half_gap); }
        if (rising & BIT(PIN_STAGGER)) { g.n_order++; add_event('S', g.n_order, middle, half_gap); }
        before = pins; t_before = t_now;
        if (t_now - last_beat > MS) { beat(CAPTURE); last_beat = t_now; }
    }
    return arg;
}
/* to_laptop(): send one text line to the referee on the laptop. */
static void to_laptop(const char *text)
{
    send_to(g.sock_out, text, strlen(text), g.laptop, PORT_REPORT);
}
/* report_thread() - priority 20, core 0, every 20 ms: send pulse stamps (EV), distrusted
 * stamps (AM) and the status (ST) to the laptop. Slow work stays off the timing cores. */
void *report_thread(void *arg)
{
    set_thread(20, 0);
    char line[256];
    unsigned am_seen = 0;
    int64_t next_status = 0, next_print = 0;
    for (;;) {
        sleep_until(hw_now() + 20 * MS);
        beat(REPORT);
        int64_t now = hw_now();
        est_t *s = &g.est;
        while (g.tail != g.head) {                  /* EV: one line per pulse */
            pulse_t *p = &g.ring[g.tail % 256];
            snprintf(line, sizeof line, "EV %s %c %u %lld %lld %.0f %s\n", g.name, p->kind, p->seq,
                     (long long)p->stamp, (long long)p->raw, p->U, VALID_NAME[p->validity]);
            to_laptop(line); g.tail++;
        }
        if (s->am_seq != am_seen) {                 /* AM: after a clock step, distrust these */
            snprintf(line, sizeof line, "AM %s %lld %lld\n", g.name, (long long)s->am_from, (long long)s->am_to);
            am_seen = s->am_seq; to_laptop(line);
        }
        if (now >= next_status) {                   /* ST: status, twice a second */
            double avg, max;
            lat_stats(&avg, &max); next_status = now + SEC / 2;
            const char *state = g.is_master ? "REFERENCE" : STATE_NAME[s->state];
            snprintf(line, sizeof line, "ST %s %s %s %.0f %.0f %.0f %.0f %.0f %.0f %.0f %d %.1f %.1f\n",
                     g.name, state, VALID_NAME[g.validity], g.U, s->raw_off, s->corr_off, s->drift_ppb,
                     s->delay, s->recovery / 1e6, s->detect / 1e6, *clock_restarts(), avg, max);
            to_laptop(line);
        }
        if (now < next_print) continue;             /* console: once a second */
        next_print = now + SEC;
        double up = (now - g.start) / 1e9;
        if (g.is_master) {
            printf("[%s %6.1fs] REFERENCE %s  health %s  wd %s  pulses %u/%u\n", g.name, up,
                   VALID_NAME[g.validity], g.health_fault ? "FAULT" : "ok", g.wd_fault ? "FAULT" : "ok",
                   g.n_common, g.n_order);
        } else {
            double U_us = g.U > 1e8 ? 99999.0 : g.U / 1e3;
            printf("[%s %6.1fs] %-10s %-10s U=%6.1fus off=%+9.1fus corr=%+6.1fus drift=%+7.3fppm delay=%5.1fus rec=%.0fms\n",
                   g.name, up, STATE_NAME[s->state], VALID_NAME[g.validity], U_us, s->raw_off / 1e3,
                   s->corr_off / 1e3, s->drift_ppb / 1e3, s->delay / 1e3, s->recovery / 1e6);
        }
        if (g.n_common == 0 && (int)up % 30 == 10)
            qlog(L_WARN, "NOTE: no SKY pulse on pin 13%s. Wires: SKY pin 11/13 -> pin 13, GND -> GND. "
                 "Test on SKY: ./skygen test\n", g.have_gpio ? "" : " (GPIO NOT AVAILABLE)");
    }
    return arg;
}
/* watchdog_thread() - priority 70, alone on core 3, every 10 ms: a late thread makes all stamps
 * UNRELIABLE, a stuck thread restarts the program (exit 42). Also sends HAM's heartbeat. */
void *watchdog_thread(void *arg)
{
    set_thread(70, 3);
    const int64_t warn_after[4]    = { 300 * MS, 100 * MS, 50 * MS, 300 * MS };   /* rx servo capture report */
    const int64_t restart_after[4] = { 600 * MS, 300 * MS, 300 * MS, 1000 * MS };
    int64_t clean_since = hw_now(), last_ham = 0;
    sleep_until(hw_now() + 2 * SEC);                /* let every thread start */
    ham_start(g.name);                              /* HAM now watches us */
    for (;;) {
        sleep_until(hw_now() + 10 * MS);
        int64_t now = hw_now();
        if (now - last_ham >= 200 * MS) { ham_beat(); last_ham = now; }   /* to HAM: still alive */
        int late = 0;
        for (int i = 0; i < 4; i++) {
            if (!g.watched[i]) continue;
            int64_t silent = now - g.heartbeat[i];
            if (silent > warn_after[i]) late = 1;
            if (silent > restart_after[i]) {        /* stuck: restart the program */
                qlog(L_WATCHDOG, "WATCHDOG: %s thread stuck for %lld ms -> restarting starsync\n",
                     THREAD_NAME[i], (long long)(silent / MS));
                (*clock_restarts())++;
                _exit(42);                          /* the run script starts us again */
            }
        }
        if (late && !g.wd_fault) qlog(L_WATCHDOG, "WATCHDOG: a thread is late -> all timestamps UNRELIABLE\n");
        if (late) { g.wd_fault = 1; clean_since = now; }
        else if (g.wd_fault && now - clean_since > 500 * MS) {          /* 0.5 s clean: trust again */
            g.wd_fault = 0; qlog(L_INFO, "WATCHDOG: all threads on time again\n");
        }
    }
    return arg;
}
