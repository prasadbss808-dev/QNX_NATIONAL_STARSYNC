/* ptp.c - the PTP messages (IEEE 1588-2008, version 2, two-step, unicast over UDP).
 *   VEGA (master) answers.  ALTAIR (slave) collects the four times T1, T2, T3, T4. */
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "node.h"

/* send_to(): send one UDP packet to address `to`, port `port`. Used for every message. */
void send_to(int sock, const void *buf, int len, struct sockaddr_in to, int port)
{
    to.sin_port = htons(port);
    sendto(sock, buf, len, 0, (struct sockaddr *)&to, sizeof to);
}
/* ptp_build(): write one PTP message into msg and return its length.
 * We use no PTP library, so every byte of the standard header is filled in here. */
int ptp_build(uint8_t *msg, int type, uint16_t seq, int64_t time_ns, const uint8_t *requester, int clock_class)
{
    int len = 44;                                   /* Sync, Delay_Req, Follow_Up */
    if (type == DELAY_RESP) len = 54;
    if (type == ANNOUNCE) len = 64;
    uint64_t seconds = (uint64_t)time_ns / SEC;
    uint32_t nanos = (uint32_t)((uint64_t)time_ns % SEC);
    memset(msg, 0, len);
    msg[0] = type; msg[1] = 2; msg[3] = len;        /* message type, PTP version 2, length */
    msg[6] = 0x04;                                  /* flag: unicast                       */
    if (type == SYNC) msg[6] |= 0x02;               /* flag: two-step (a Follow_Up comes)  */
    memcpy(msg + 20, g.id, 8); msg[29] = 1;         /* sender: our clock identity, port 1  */
    msg[30] = seq >> 8; msg[31] = seq & 0xFF;       /* sequence number                     */
    if (type == SYNC)       msg[32] = 0;            /* control field (from PTP v1)    */
    if (type == DELAY_REQ)  msg[32] = 1;
    if (type == FOLLOW_UP)  msg[32] = 2;
    if (type == DELAY_RESP) msg[32] = 3;
    if (type == ANNOUNCE)   msg[32] = 5;
    msg[33] = 0x7F;                                 /* message interval: not used          */
    for (int i = 0; i < 6; i++) msg[34 + i] = (uint8_t)(seconds >> (40 - 8 * i));   /* 48-bit seconds */
    for (int i = 0; i < 4; i++) msg[40 + i] = (uint8_t)(nanos >> (24 - 8 * i));     /* 32-bit ns      */
    if (type == DELAY_RESP) { memcpy(msg + 44, requester, 8); msg[53] = 1; }         /* who asked      */
    if (type == ANNOUNCE) {
        msg[47] = 128; msg[52] = 128;               /* priorities 1 and 2                  */
        msg[48] = clock_class;                      /* 248 = healthy, 255 = faulty         */
        msg[49] = 0xFE; msg[50] = 0xFF; msg[51] = 0xFF;   /* accuracy, variance: unknown   */
        memcpy(msg + 53, g.id, 8);                  /* grandmaster = us                    */
        msg[63] = 0xA0;                             /* time source: internal oscillator    */
    }
    return len;
}
/* ptp_type(), ptp_seq(), ptp_time(): read the type, sequence number and timestamp of a message. */
int ptp_type(const uint8_t *msg) { return msg[0] & 0x0F; }
int ptp_seq(const uint8_t *msg) { return msg[30] << 8 | msg[31]; }
int64_t ptp_time(const uint8_t *msg)
{
    int64_t seconds = 0;
    for (int i = 0; i < 6; i++) seconds = seconds << 8 | msg[34 + i];
    int64_t nanos = (int64_t)msg[40] << 24 | msg[41] << 16 | msg[42] << 8 | msg[43];
    return seconds * SEC + nanos;
}
/* net_fault(): apply the judges' network faults to a packet we just received.
 * Returns 0 = drop it (blackout). Otherwise its arrival time *t is made later, because
 * a packet stamped later is exactly a packet that travelled longer. */
int net_fault(int64_t *t)
{
    static unsigned seed = 1;
    if (hw_now() < g.blackout_until) return 0;
    double spread_us = g.delay_max_us - g.delay_min_us;
    double extra_us = g.asym_us + g.delay_min_us + spread_us * rand_r(&seed) / RAND_MAX;   /* random in min..max */
    *t += (int64_t)(extra_us * US);
    return 1;
}
/* master_rx() - VEGA: answer ALTAIR's Delay_Req with Delay_Resp (T4), then Sync (T1) and
 * Follow_Up (the exact T1). Every 8th time also an Announce saying whether VEGA is healthy. */
void master_rx(const uint8_t *msg, int type, uint16_t seq, int64_t t4, struct sockaddr_in from)
{
    static in_addr_t last_slave = 0;                /* the last slave we heard from */
    static unsigned rounds = 0;
    if (type != DELAY_REQ) return;   /* VEGA only answers Delay_Req */
    if (from.sin_addr.s_addr != last_slave) {      /* a (new) slave talks to us */
        qlog(L_INFO, "PTP: slave %s joined\n", inet_ntoa(from.sin_addr)); last_slave = from.sin_addr.s_addr;
    }
    int event_port = ntohs(from.sin_port), general_port = event_port + 1;   /* ALTAIR's 319 and 320 */
    int clock_class = 248;                          /* 248 = healthy                 */
    if (g.health_fault || g.wd_fault) clock_class = 255;   /* 255 = faulty: don't follow me */
    uint8_t m[64];
    int len;
    len = ptp_build(m, DELAY_RESP, seq, t4, msg + 20, 0);   /* 1. Delay_Resp carries T4 */
    send_to(g.sock_general, m, len, from, general_port);
    len = ptp_build(m, SYNC, seq, 0, NULL, 0);              /* 2. Sync: stamp T1, send at once */
    int64_t t1 = clock_raw(hw_now());
    send_to(g.sock_event, m, len, from, event_port);
    len = ptp_build(m, FOLLOW_UP, seq, t1, NULL, 0);        /* 3. Follow_Up carries the exact T1 */
    send_to(g.sock_general, m, len, from, general_port);
    if (rounds % 8 == 0 || clock_class == 255) {            /* 4. Announce: our health */
        len = ptp_build(m, ANNOUNCE, seq, 0, NULL, clock_class);
        send_to(g.sock_general, m, len, from, general_port);
    }
    rounds++;
}
/* slave_rx() - ALTAIR: collect T2 (Sync arrived), T1 (Follow_Up) and T4 (Delay_Resp) for our
 * latest exchange. When all four times are in, they become one measurement for the Kalman filter. */
void slave_rx(const uint8_t *msg, int type, uint16_t seq, int64_t msg_time, int64_t t_arrival)
{
    if (type == ANNOUNCE) { g.vega_class = msg[48]; return; }       /* VEGA's health */
    pthread_mutex_lock(&g.lock);
    int complete = g.got_sync && g.got_follow_up && g.got_delay_resp;
    if (seq == g.seq && !complete) {                /* an answer to our latest Delay_Req */
        if (type == SYNC) { g.t2 = t_arrival; g.got_sync = 1; }                 /* T2: Sync arrived here    */
        if (type == FOLLOW_UP) { g.t1 = msg_time; g.got_follow_up = 1; }        /* T1: VEGA sent the Sync   */
        if (type == DELAY_RESP && memcmp(msg + 44, g.id, 8) == 0) {             /* T4: our Delay_Req arrived */
            g.t4 = msg_time; g.got_delay_resp = 1;
        }
        if (g.got_sync && g.got_follow_up && g.got_delay_resp) {
            est_measurement(&g.est, g.t1, g.t2, g.t3, g.t4, g.vega_class == 255);
            qmark(M_PTP, (int)(g.est.raw_off / 1e3), (int)(g.est.delay / 1e3));   /* in us */
        }
    }
    pthread_mutex_unlock(&g.lock);
}
