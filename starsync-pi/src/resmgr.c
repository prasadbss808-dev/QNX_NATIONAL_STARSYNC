/* resmgr.c - StarSync as a QNX resource manager. Other programs use it like files:
 *   cat /dev/starsync/status       cat /dev/starsync/time       echo "drift 20" > /dev/starsync/ctl
 * and devctl(DCMD_STARSYNC_TIME) returns the time in one QNX message (try: starsync -q). */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <devctl.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include "starsync.h"

typedef struct {                                    /* the devctl answer */
    int64_t corrected_ns;
    double  U_ns;
    int     validity;
} starsync_time_t;
#define DCMD_STARSYNC_TIME __DIOF(_DCMD_MISC, 0x51, starsync_time_t)
static resmgr_connect_funcs_t connect_funcs;
static resmgr_io_funcs_t io_funcs;
static iofunc_attr_t attr_status, attr_time, attr_ctl;   /* one per file */
static char last_reply[2048] = "no command yet\n";
/* io_read(): someone reads one of our files: answer with the status, the time,
 * or the reply to the last command - all in one read, then end of file. */
static int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
    int status = iofunc_read_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;
    char buf[2048];
    int n = 0;
    if (ocb->offset == 0) {                         /* first read: the whole text */
        if (ocb->attr == &attr_status) n = status_text(buf, sizeof buf);
        else if (ocb->attr == &attr_time) n = time_text(buf, sizeof buf);
        else n = snprintf(buf, sizeof buf, "%s", last_reply);
        if (n > (int)_IO_READ_GET_NBYTES(msg)) n = _IO_READ_GET_NBYTES(msg);
        ocb->offset = n;                            /* next read: end of file */
    }
    MsgReply(ctp->rcvid, n, buf, n);                /* reply now: buf lives on this stack */
    return _RESMGR_NOREPLY;
}
/* io_write(): someone writes to /dev/starsync/ctl: run it as a command. */
static int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb)
{
    int status = iofunc_write_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE || ocb->attr != &attr_ctl) return ENOSYS;
    size_t n = _IO_WRITE_GET_NBYTES(msg);
    char cmd[128] = "";
    size_t take = n < sizeof cmd - 1 ? n : sizeof cmd - 1;
    resmgr_msgget(ctp, cmd, take, sizeof msg->i);  /* the text follows the message header */
    cmd[strcspn(cmd, "\r\n")] = 0;
    do_command(cmd, last_reply, sizeof last_reply);
    _IO_SET_WRITE_NBYTES(ctp, n);
    return _RESMGR_NPARTS(0);
}
/* io_devctl(): answer DCMD_STARSYNC_TIME with the corrected time, U and validity.
 * The fastest way for another program to get synchronised time. */
static int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb)
{
    int status = iofunc_devctl_default(ctp, msg, ocb);
    if (status != _RESMGR_DEFAULT) return status;
    if (msg->i.dcmd != DCMD_STARSYNC_TIME) return ENOSYS;
    starsync_time_t *answer = _DEVCTL_DATA(msg->i);   /* the answer goes after the header */
    if (time_now(&answer->corrected_ns, &answer->U_ns, &answer->validity) != 0)
        return EAGAIN;                              /* not synchronised yet */
    memset(&msg->o, 0, sizeof msg->o);
    msg->o.nbytes = sizeof *answer;
    return _RESMGR_PTR(ctp, &msg->o, sizeof msg->o + sizeof *answer);
}
/* time_client() - "starsync -q": ask the running starsync for the time 1000 times through
 * devctl, and print the time and how long one question takes (average and worst). */
int time_client(void)
{
    starsync_time_t t; double sum = 0, max = 0;
    int fd = open("/dev/starsync/time", O_RDONLY);
    if (fd < 0) { printf("starsync is not running on this board\n"); return 1; }
    for (int i = 0; i < 1000; i++) {
        int64_t start = hw_now();
        if (devctl(fd, DCMD_STARSYNC_TIME, &t, sizeof t, NULL) != EOK) {
            printf("not synchronised yet\n");
            return 1;
        }
        double us = (hw_now() - start) / 1e3;      /* one round trip, in us */
        sum += us;
        if (us > max) max = us;
    }
    printf("corrected time %lld.%09lld s  U +-%.1f us  %s\n", (long long)(t.corrected_ns / SEC),
           (long long)(t.corrected_ns % SEC), t.U_ns / 1e3, VALID_NAME[t.validity]);
    printf("devctl() round trip: avg %.2f us, max %.2f us (1000 calls)\n", sum / 1000, max);
    return close(fd);
}
/* resmgr_thread() - priority 12, core 0: register the three files with QNX, then serve
 * every open / read / write / devctl message forever. */
static void *resmgr_thread(void *arg)
{
    set_thread(12, 0);
    dispatch_t *dpp = dispatch_create();
    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, _RESMGR_IO_NFUNCS, &io_funcs);
    io_funcs.read = io_read; io_funcs.write = io_write; io_funcs.devctl = io_devctl;   /* our handlers */
    iofunc_attr_init(&attr_status, S_IFNAM | 0444, NULL, NULL);   /* read-only  */
    iofunc_attr_init(&attr_time, S_IFNAM | 0444, NULL, NULL);     /* read-only  */
    iofunc_attr_init(&attr_ctl, S_IFNAM | 0666, NULL, NULL);      /* read-write */
    int ok = dpp != NULL;
    if (ok && resmgr_attach(dpp, NULL, "/dev/starsync/status", _FTYPE_ANY, 0, &connect_funcs, &io_funcs, &attr_status) == -1)
        ok = 0;
    if (ok && resmgr_attach(dpp, NULL, "/dev/starsync/time", _FTYPE_ANY, 0, &connect_funcs, &io_funcs, &attr_time) == -1)
        ok = 0;
    if (ok && resmgr_attach(dpp, NULL, "/dev/starsync/ctl", _FTYPE_ANY, 0, &connect_funcs, &io_funcs, &attr_ctl) == -1)
        ok = 0;
    if (!ok) { qlog(L_WARN, "resmgr: cannot register /dev/starsync (run as root)\n"); return arg; }
    qlog(L_INFO, "resmgr: /dev/starsync/status, /dev/starsync/time and /dev/starsync/ctl are ready\n");
    dispatch_context_t *ctp = dispatch_context_alloc(dpp);
    while ((ctp = dispatch_block(ctp)) != NULL)     /* wait for a message ...  */
        dispatch_handler(ctp);                      /* ... and handle it       */
    return arg;
}
/* resmgr_start(): run the resource manager in its own thread, away from the timing threads. */
void resmgr_start(void) { start_thread(resmgr_thread); }
