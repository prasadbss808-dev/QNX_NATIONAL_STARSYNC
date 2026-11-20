/* node.h - the memory of one telescope node (VEGA or ALTAIR).
 * There is exactly one, called g. It is shared by main.c, ptp.c, threads.c and commands.c. */
#ifndef NODE_H
#define NODE_H
#include <pthread.h>
#include <netinet/in.h>
#include "starsync.h"

enum { SYNC = 0, DELAY_REQ = 1, FOLLOW_UP = 8, DELAY_RESP = 9, ANNOUNCE = 11 };   /* PTP message types */
enum { RX, SERVO, CAPTURE, REPORT };             /* the threads the watchdog watches */
extern const char *const THREAD_NAME[];
typedef struct {                    /* one SKY pulse, as this node stamped it */
    char     kind;                  /* 'C' = COMMON pulse, 'S' = ordering pulse */
    unsigned seq;                   /* pulse number                     */
    int64_t  raw;                   /* raw time of the edge             */
    int64_t  stamp;                 /* corrected time = what we report  */
    double   U;                     /* its uncertainty (ns)             */
    int      validity;
} pulse_t;
typedef struct {
    /* who we are */
    char     name[16];              /* "VEGA" or "ALTAIR"                */
    int      is_master;             /* 1 = VEGA, the reference           */
    uint8_t  id[8];                 /* PTP clockIdentity = our name      */
    /* addresses and sockets */
    char     master_ip[32], laptop_ip[32];
    int      ptp_port, master_port, cmd_port;
    int      sock_event, sock_general, sock_cmd, sock_out;
    struct sockaddr_in vega, laptop;
    /* the PTP exchange in progress (ALTAIR) - protected by lock */
    pthread_mutex_t lock;
    uint16_t seq;                   /* number of our latest Delay_Req    */
    int64_t  t1, t2, t3, t4;
    int      got_sync, got_follow_up, got_delay_resp;
    int      vega_class;            /* from Announce: 248 healthy, 255 faulty */
    /* the estimator, and what we publish */
    est_t    est;                   /* protected by lock                 */
    int      validity;
    double   U;
    int64_t  start;
    /* faults injected by the judges' commands */
    int      health_fault;
    double   delay_min_us, delay_max_us, asym_us;
    int64_t  blackout_until;
    /* SKY pulses: a ring of 256; capture writes at head, report reads at tail */
    int      have_gpio;
    pulse_t  ring[256];
    volatile unsigned head, tail;
    volatile unsigned n_common, n_order;
    /* watchdog */
    volatile int64_t heartbeat[4];  /* last "I am alive" of each thread  */
    volatile int     hang_ms[4];    /* "hang" self-test: freeze a thread */
    volatile int     watched[4];
    int      wd_fault;              /* a thread is late                  */
    /* how late QNX woke the servo, last 320 ticks (10 s), in us */
    double   late_us[320];
    int      late_i;
} node_t;
extern node_t g;
/* ptp.c */
void send_to(int sock, const void *buf, int len, struct sockaddr_in to, int port);
int  ptp_build(uint8_t *msg, int type, uint16_t seq, int64_t time_ns, const uint8_t *requester, int clock_class);
int  ptp_type(const uint8_t *msg);
int  ptp_seq(const uint8_t *msg);
int64_t ptp_time(const uint8_t *msg);
int  net_fault(int64_t *t);
void master_rx(const uint8_t *msg, int type, uint16_t seq, int64_t t4, struct sockaddr_in from);
void slave_rx(const uint8_t *msg, int type, uint16_t seq, int64_t msg_time, int64_t t_arrival);
/* threads.c */
void *rx_thread(void *arg);
void *servo_thread(void *arg);
void *capture_thread(void *arg);
void *report_thread(void *arg);
void *watchdog_thread(void *arg);
void  lat_stats(double *avg, double *max);
#endif
