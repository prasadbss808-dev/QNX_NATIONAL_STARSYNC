/* main.c - StarSync starts here. One program, two roles:
 *   VEGA   : starsync -r master -i vega     the reference (PTP master)
 *   ALTAIR : starsync -r slave  -i altair   follows VEGA and keeps a corrected clock */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "node.h"

node_t g;                                           /* this node's memory (node.h) */
const char *const THREAD_NAME[] = { "rx", "servo", "capture", "report" };
/* udp_socket(): open a UDP socket on a port (0 = any), marked as real-time traffic (DSCP EF)
 * so the switch lets these packets wait less. */
static int udp_socket(int port)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0), tos = 0xB8, one = 1;   /* 0xB8 = DSCP EF, "expedited forwarding" */
    setsockopt(s, IPPROTO_IP, IP_TOS, &tos, sizeof tos);
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = INADDR_ANY };
    if (port != 0 && bind(s, (struct sockaddr *)&a, sizeof a) < 0) {
        printf("cannot use port %d (root? running twice?)\n", port); exit(1);
    }
    return s;
}
/* stop(): Ctrl-C ends the program through exit(), so a profiling build can save its data. */
static void stop(int sig) { (void)sig; exit(0); }
/* main(): read the options, set everything up, start the threads and the resource manager,
 * then answer text commands on UDP forever. */
int main(int argc, char **argv)
{
    int opt, tol_us = 100;
    strcpy(g.master_ip, IP_VEGA); strcpy(g.laptop_ip, IP_LAPTOP); strcpy(g.name, "NODE");   /* defaults */
    g.ptp_port = PORT_PTP; g.master_port = PORT_PTP; g.cmd_port = PORT_CMD;
    while ((opt = getopt(argc, argv, "qr:i:m:L:t:P:c:")) != -1) {
        if (opt == 'q') { platform_init(); return time_client(); }   /* -q: ask the running starsync */
        else if (opt == 'r') g.is_master = strcmp(optarg, "master") == 0;            /* -r master | slave */
        else if (opt == 'i') {                      /* -i name, in capitals */
            int i;
            for (i = 0; i < 15 && optarg[i]; i++) g.name[i] = toupper((unsigned char)optarg[i]);
            g.name[i] = 0;
        } else if (opt == 'm') {                    /* -m VEGA's IP or IP:PORT */
            snprintf(g.master_ip, sizeof g.master_ip, "%s", optarg);
            char *colon = strchr(g.master_ip, ':');
            if (colon) { *colon = 0; g.master_port = atoi(colon + 1); }
        }
        else if (opt == 'L') snprintf(g.laptop_ip, sizeof g.laptop_ip, "%s", optarg);  /* -L laptop IP     */
        else if (opt == 't') tol_us = atoi(optarg);                                    /* -t tolerance, us */
        else if (opt == 'P') g.ptp_port = atoi(optarg);                                /* -P PTP port      */
        else if (opt == 'c') g.cmd_port = atoi(optarg);                                /* -c command port  */
        else {
            printf("usage: starsync -r master|slave -i NAME [-m vega_ip[:port]] [-L laptop_ip] [-t tol_us] [-P ptp_port] [-c cmd_port]\n"
                   "       starsync -q   (ask the running starsync for the time via devctl)\n");
            return 1;
        }
    }
    setvbuf(stdout, NULL, _IOLBF, 0);               /* print whole lines at once */
    signal(SIGINT, stop); signal(SIGTERM, stop);
    platform_init();                                /* QNX: clock rate, HR timers, slogger2 */
    clock_init(g.name, g.is_master);                /* corrected clock + shared memory    */
    pthread_mutex_init(&g.lock, NULL);
    est_init(&g.est, tol_us * US); g.start = hw_now();
    memcpy(g.id, g.name, 8);                        /* PTP clockIdentity = our name */
    g.vega.sin_family = AF_INET; inet_aton(g.master_ip, &g.vega.sin_addr);       /* where VEGA is   */
    g.laptop.sin_family = AF_INET; inet_aton(g.laptop_ip, &g.laptop.sin_addr);   /* and the laptop */
    g.sock_event = udp_socket(g.ptp_port);          /* PTP event messages   (319) */
    g.sock_general = udp_socket(g.ptp_port + 1);    /* PTP general messages (320) */
    g.sock_cmd = udp_socket(g.cmd_port);            /* text commands              */
    g.sock_out = udp_socket(0);                     /* reports to the laptop      */
    g.have_gpio = gpio_init() == 0;                 /* SKY's pulses on pins 13 and 15 */
    if (g.have_gpio) { gpio_setup(PIN_COMMON, 0); gpio_setup(PIN_STAGGER, 0); }
    qlog(L_INFO, "starsync %s: %s, PTP ports %d/%d, commands on %d, reports to %s, tolerance %d us, restarts %d%s\n",
         g.name, g.is_master ? "PTP MASTER (reference)" : "PTP SLAVE", g.ptp_port, g.ptp_port + 1, g.cmd_port,
         g.laptop_ip, tol_us, *clock_restarts(), g.have_gpio ? "" : ", NO GPIO (run as root)");
    for (int i = 0; i < 4; i++) {                   /* the watchdog watches these threads */
        g.heartbeat[i] = hw_now(); g.watched[i] = (i != CAPTURE) || g.have_gpio;
    }
    start_thread(watchdog_thread); start_thread(rx_thread); start_thread(servo_thread); start_thread(report_thread);
    if (g.have_gpio) start_thread(capture_thread);
    resmgr_start();                                 /* /dev/starsync */
    set_thread(12, 0);                              /* this thread: text commands on UDP */
    for (;;) {
        char cmd[128], reply[2048];
        struct sockaddr_in from; socklen_t from_len = sizeof from;
        int len = recvfrom(g.sock_cmd, cmd, sizeof cmd - 1, 0, (struct sockaddr *)&from, &from_len);
        if (len <= 0) continue;
        cmd[len] = 0; cmd[strcspn(cmd, "\r\n")] = 0;    /* end the text, drop the line end */
        len = do_command(cmd, reply, sizeof reply);
        sendto(g.sock_cmd, reply, len, 0, (struct sockaddr *)&from, from_len);
    }
}
