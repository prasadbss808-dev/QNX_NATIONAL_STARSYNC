/* hog.c - CPU load test for the priorities and cores.
 *   ./hog 10 30   : busy threads at priority 10 on all 4 cores for 30 s -> StarSync (55-70) is not disturbed
 *   ./hog 60 3 1  : priority 60 on core 1 for 3 s steals the servo (55) -> the watchdog must catch it */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "starsync.h"

static int prio;                                    /* priority of the busy threads */
static int64_t until;                               /* when to stop */
/* spin(): one busy thread on one core, burning CPU until the end time. */
static void *spin(void *core)
{
    set_thread(prio, (int)(intptr_t)core);
    while (hw_now() < until) ;
    return NULL;
}
/* main(): hog PRIORITY SECONDS [CORE] - one busy thread per core (or only on CORE). */
int main(int argc, char **argv)
{
    if (argc < 3) { printf("usage: hog PRIORITY SECONDS [CORE]   (no CORE = all 4 cores)\n"); return 1; }
    platform_init();
    prio = atoi(argv[1]);
    until = hw_now() + (int64_t)(atof(argv[2]) * SEC);
    int first = 0, last = 3;
    if (argc > 3) { first = atoi(argv[3]); last = first; }
    pthread_t t[4];
    qlog(L_WARN, "hog: busy at priority %d on core %d..%d for %s s\n", prio, first, last, argv[2]);
    for (int c = first; c <= last; c++) pthread_create(&t[c], NULL, spin, (void *)(intptr_t)c);
    for (int c = first; c <= last; c++) pthread_join(t[c], NULL);
    qlog(L_INFO, "hog: done\n");
    return 0;
}
