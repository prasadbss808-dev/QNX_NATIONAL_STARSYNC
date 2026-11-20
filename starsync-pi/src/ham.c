/* ham.c - QNX's High Availability Manager (HAM) watches our watchdog from OUTSIDE the process:
 * the watchdog sends a heartbeat every 200 ms; after 3 missed beats HAM kills StarSync and the
 * run script starts it again. (Start HAM as root with: ham &)
 * HAM is optional: the Makefile defines HAVE_HAM only if this QNX SDP has <ha/ham.h>. */
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include "starsync.h"
#ifdef HAVE_HAM
#include <ha/ham.h>
#endif

static int ham_ok;                                  /* is HAM supervising us? */
/* ham_start(): register with HAM: heartbeat every 200 ms, 3 missed -> SIGKILL. */
void ham_start(const char *name)
{
#ifdef HAVE_HAM
    char entity_name[48];
    snprintf(entity_name, sizeof entity_name, "starsync-%s-%d", name, getpid());
    if (ham_connect(0) != -1) {
        ham_entity_t *me = ham_attach_self(entity_name, 200 * MS, 0, 3, 0);
        if (me != NULL) {
            ham_condition_t *hung = ham_condition(me, CONDHBEATMISSEDHIGH, "hung", 0);
            if (hung != NULL && ham_action_notify_signal(hung, "kill", 0, getpid(), SIGKILL, 0, 0, 0) != NULL)
                ham_ok = 1;
        }
    }
    if (ham_ok) qlog(L_INFO, "HAM: supervising the watchdog (heartbeat 200 ms, 3 missed -> restart)\n");
    else qlog(L_INFO, "HAM: not running - start it as root with: ham\n");
#else
    (void)name;
    qlog(L_INFO, "HAM: not in this QNX SDP - the watchdog + run script restart StarSync\n");
#endif
}
/* ham_beat(): "still alive" - the watchdog calls this every 200 ms. */
void ham_beat(void)
{
#ifdef HAVE_HAM
    if (ham_ok) ham_heartbeat();
#endif
}
/* ham_on(): 1 if HAM supervises us (shown in the status). */
int ham_on(void) { return ham_ok; }
