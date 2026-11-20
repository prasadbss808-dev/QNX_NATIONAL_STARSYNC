/* skygen.c - SKY, the "star" both telescopes observe (run on 172.16.6.78 as root).
 *   Every 100 ms a COMMON pulse: pin 11 -> VEGA pin 13 and pin 13 -> ALTAIR pin 13, raised by
 *   ONE register write, so both telescopes see it at the same instant.
 *   Every 0.5 s an ordering pair: pin 15 -> VEGA, pin 16 -> ALTAIR, a known gap apart, in random
 *   order. SKY tells the referee the truth:  TR <pair> <A = VEGA first | B = ALTAIR first> <gap_us>
 *   ./skygen test : wiring check - all four outputs 1 s HIGH, 1 s LOW, read back. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "starsync.h"

/* wait_exact(): sleep on a timer until 200 us before t, then spin until exactly t
 * (the timer saves CPU, the short spin removes the timer's jitter). */
static void wait_exact(int64_t t)
{
    sleep_until(t - 200 * US);
    while (hw_now() < t) ;
}
/* pin_check(): "ok" if the pin reads back the level we wrote, else "NOT DRIVEN". */
static const char *pin_check(uint32_t levels, int pin, int level)
{
    if ((int)((levels >> pin) & 1) == level) return "ok";
    return "NOT DRIVEN";
}
/* wiring_test(): toggle all four outputs every second and show whether each one follows.
 * On VEGA and ALTAIR the PULSES count in the status must go up. Runs until Ctrl-C. */
static void wiring_test(uint32_t all_pins)
{
    for (int level = 1;; level = !level) {
        gpio_write(all_pins, level);
        sleep_until(hw_now() + 20 * MS);
        uint32_t r = gpio_read_all();
        printf("%s  pin 11 %s  pin 13 %s  pin 15 %s  pin 16 %s   (nodes: PULSES in status must count up)\n",
               level ? "HIGH" : "LOW ", pin_check(r, SKY_COMMON_A, level), pin_check(r, SKY_COMMON_B, level),
               pin_check(r, SKY_PAIR_A, level), pin_check(r, SKY_PAIR_B, level));
        fflush(stdout);
        sleep_until(hw_now() + 980 * MS);
    }
}
/* main(): set up the four output pins (priority 60, core 3), then fire pulses forever. */
int main(int argc, char **argv)
{
    static const int gaps_us[] = { 1000, 500, 200, 100, 50, 20 };   /* ordering gaps, getting harder */
    struct sockaddr_in referee = { .sin_family = AF_INET, .sin_port = htons(PORT_REPORT) };
    inet_aton(argc > 1 ? argv[1] : IP_LAPTOP, &referee.sin_addr);
    platform_init();
    set_thread(60, 3);
    if (gpio_init() < 0) { printf("skygen: cannot map the GPIO registers (run as root)\n"); return 1; }
    gpio_setup(SKY_COMMON_A, 1);
    gpio_setup(SKY_COMMON_B, 1);
    gpio_setup(SKY_PAIR_A, 1);
    gpio_setup(SKY_PAIR_B, 1);
    const uint32_t common = BIT(SKY_COMMON_A) | BIT(SKY_COMMON_B);
    const uint32_t all = common | BIT(SKY_PAIR_A) | BIT(SKY_PAIR_B);
    if (argc > 1 && strcmp(argv[1], "test") == 0) wiring_test(all);   /* never returns */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    qlog(L_INFO, "skygen: COMMON 10 Hz on pins 11+13, ordering pairs on pins 15/16 every 0.5 s, truth to %s\n",
         inet_ntoa(referee.sin_addr));
    unsigned tick = 0, pair = 0, seed = 7;
    int64_t next = (hw_now() / (100 * MS) + 1) * 100 * MS;      /* the next whole 100 ms */
    for (;; next += 100 * MS) {
        wait_exact(next);                           /* COMMON pulse: 1 ms long */
        gpio_write(common, 1);
        qmark(M_PULSE, 'C', (int)tick);
        sleep_until(hw_now() + MS);
        gpio_write(common, 0);
        tick++;
        if (tick % 50 == 0) printf("skygen: %u pulses, %u ordering pairs\n", tick, pair);
        if (tick % 5 != 0) continue;   /* ordering pair only every 5th tick */
        int gap = gaps_us[(pair / 4) % 6];
        int vega_first = rand_r(&seed) & 1;
        int first = vega_first ? SKY_PAIR_A : SKY_PAIR_B;
        int second = vega_first ? SKY_PAIR_B : SKY_PAIR_A;
        wait_exact(next + 40 * MS);                 /* 40 ms after COMMON */
        gpio_write(BIT(first), 1);
        wait_exact(next + 40 * MS + gap * US);      /* then the other one, gap later */
        gpio_write(BIT(second), 1);
        sleep_until(hw_now() + MS);
        gpio_write(BIT(first) | BIT(second), 0);
        pair++;
        char line[64];
        int n = snprintf(line, sizeof line, "TR %u %c %d\n", pair, vega_first ? 'A' : 'B', gap);
        sendto(sock, line, n, 0, (struct sockaddr *)&referee, sizeof referee);
    }
}
